import threading, time, traceback
from unittest.mock import patch
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os, json, pymysql
from datetime import datetime

from isc_task_optimizer.settings import (
    TARGET_DATE,
    DB_NEW_POLL_PERIOD_SEC,
    DB_NEW_DEBOUNCE_SEC,
    SCHEDULE_POLL_PERIOD_SEC,
)

try:
    from isc_main_logic.main_logic import run_logic
except Exception:
    import sys, pathlib
    HERE = pathlib.Path(__file__).resolve()
    ROOT = HERE.parents[6]
    sys.path.append(str(ROOT))
    from isc_main_logic.main_logic import run_logic


class OptimizerNode(Node):
    def __init__(self) -> None:
        super().__init__("isc_task_optimizer")

        self.declare_parameter("target_date", TARGET_DATE)
        self.declare_parameter("db_new_poll_period_sec", DB_NEW_POLL_PERIOD_SEC)
        self.declare_parameter("db_new_debounce_sec", DB_NEW_DEBOUNCE_SEC)
        self.declare_parameter("schedule_poll_period_sec", SCHEDULE_POLL_PERIOD_SEC)

        self.target_date = self.get_parameter("target_date").value
        self.db_new_poll_period_sec = int(self.get_parameter("db_new_poll_period_sec").value)
        self.db_new_debounce_sec = int(self.get_parameter("db_new_debounce_sec").value)
        self.schedule_poll_period_sec = int(self.get_parameter("schedule_poll_period_sec").value)

        self.optimizer_summary = None
        self._summary_pub = self.create_publisher(String, "optimizer_summary", 10)

        # Separate spin for new orders
        self._last_seen_order = None
        self._new_orders_event = threading.Event()
        self._stop_event = threading.Event()
        self._run_lock = threading.Lock()

        self._optimizer_thread = threading.Thread(target=self._optimizer_loop, daemon=True)
        self._optimizer_thread.start()

        self._db_new_poll_timer = self.create_timer(
            self.db_new_poll_period_sec, self._poll_db_for_new_orders
        )
        self._schedule_poll_timer = self.create_timer(
            self.schedule_poll_period_sec, self._poll_schedule_trigger_optimize
        )

        # Initial run
        self._new_orders_event.set()
        self.get_logger().info(
            f"Started. Reading from MySQL connection: date='{self.target_date}'"
        )


    def _parse_target_date(self, target_date: str):
        return datetime.strptime(target_date, "%d/%m/%Y").date()


    # === Function: Create connection === #
    def _get_mysql_conn(self):
        return pymysql.connect(
            host=os.getenv("DB_HOST"),
            port=int(os.getenv("DB_PORT")),
            user=os.getenv("DB_USER"),
            password=os.getenv("DB_PASSWORD"),
            database=os.getenv("DB_NAME"),
            cursorclass=pymysql.cursors.DictCursor,
            autocommit=True,
        )
    

    # === Function: Schedule poll trigger === #
    # Fires every schedule_poll_period_sec and triggers the same optimisation pipeline.
    def _poll_schedule_trigger_optimize(self) -> None:
        self.get_logger().info("Polling schedule trigger for optimization...")
        self._new_orders_event.set()


    # === Function: Polls new orders === #
    # Compares the new order (OrderDate) with the last seen order (OrderDate)
    def _poll_db_for_new_orders(self) -> None:
        self.get_logger().info("Polling DB for new orders...")
        try:
            marker = self._query_max_active_order(self.target_date)
            if marker is None:
                return

            if self._last_seen_order is None:
                self._last_seen_order = marker
                return

            if marker > self._last_seen_order:
                self.get_logger().info("New active orders detected.")
                self._last_seen_order = marker
                self._new_orders_event.set()

        except Exception as e:
            self.get_logger().error(f"Polling failed: {e}\n{traceback.format_exc()}")


    # === Function: Optimizer loop === #
    def _optimizer_loop(self) -> None:
        while not self._stop_event.is_set():
            if not self._new_orders_event.wait(timeout=1.0):
                continue

            self._new_orders_event.clear()

            time.sleep(self.db_new_debounce_sec)


            if self._new_orders_event.is_set():
                continue

            if not self._run_lock.acquire(blocking=False):
                continue

            try:
                self._on_tick()
            finally:
                self._run_lock.release()


    # === Function: On tick === #
    def _on_tick(self) -> None:
        try:
            with patch("builtins.input", lambda *args, **kwargs: "n"):
                summary = run_logic(self.target_date)

            self.optimizer_summary = summary            
            msg = String()
            msg.data = json.dumps(self.optimizer_summary or {}, default=str)
            self._summary_pub.publish(msg)

            size = len(summary or {})
            print()
            self.get_logger().info(
                f"Simulation complete for {self.target_date} — merchants summarized: {size}"
            )

        except Exception as e:
            self.get_logger().error(f"Simulator call failed: {e}\n{traceback.format_exc()}")


    # === Function: Query max active order marker === #
    def _query_max_active_order(self, target_date: str):
        d = self._parse_target_date(target_date)

        conn = self._get_mysql_conn()

        try:
            with conn.cursor() as cur:
                sql = """
                    SELECT MAX(OrderDateTime) AS max_dt
                    FROM iSCOrderData
                    WHERE IsComplete = 0
                    AND DATE(OrderDateTime) = %s;
                    """
                cur.execute(sql, (d,))
                row = cur.fetchone()
                return row["max_dt"]
        finally:
            conn.close()


    # === Function: Destroy node === #
    def destroy_node(self):
        self._stop_event.set()
        self._new_orders_event.set()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = OptimizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
