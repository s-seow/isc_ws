import threading, time, traceback
from unittest.mock import patch
import rclpy
from rclpy.node import Node
import os
from datetime import datetime
import pymysql

from isc_task_optimizer.settings import TARGET_DATE_DEFAULT, POLL_PERIOD_SEC_DEFAULT, DEBOUNCE_SEC_DEFAULT

try:
    from isc_main_logic.main import run_logic
except Exception:
    import sys, pathlib
    HERE = pathlib.Path(__file__).resolve()
    ROOT = HERE.parents[6]  
    sys.path.append(str(ROOT))
    from isc_main_logic.main import run_logic


class OptimizerNode(Node):
    def __init__(self) -> None:
        super().__init__("isc_task_optimizer")

        self.declare_parameter("target_date", TARGET_DATE_DEFAULT)  
        self.declare_parameter("poll_period_sec", POLL_PERIOD_SEC_DEFAULT)
        self.declare_parameter("debounce_sec", DEBOUNCE_SEC_DEFAULT)

        self.target_date = self.get_parameter("target_date").value
        self.poll_period = int(self.get_parameter("poll_period_sec").value)
        self.debounce_sec = int(self.get_parameter("debounce_sec").value) 
        #   ros2 run isc_task_optimizer optimizer_node --ros-args -p target_date:=1/12/2024 -p poll_period_sec:=5 -p debounce_sec:=10

        # Separate spin for new orders
        self._last_seen_order = None

        self._new_orders_event = threading.Event()
        self._stop_event = threading.Event()
        self._run_lock = threading.Lock()

        self._optimizer_thread = threading.Thread(target=self._optimizer_loop, daemon=True)
        self._optimizer_thread.start()

        self._timer = self.create_timer(self.poll_period, self._poll_for_new_orders)

        # Initial run
        self._new_orders_event.set()  
        self.get_logger().info(
            f"Started. Reading from MySQL connection: date='{self.target_date}'"
        )


    def _parse_target_date(self, target_date: str):
        return datetime.strptime(target_date, "%d/%m/%Y").date()


    # === Function: Polls new orders === #
    # Compares the new order (OrderDate) with the last seen order (OrderDate)
    def _poll_for_new_orders(self) -> None:
        self.get_logger().info("poll tick")
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


    # === Function: Optimiser loop === #
    def _optimizer_loop(self) -> None:
        while not self._stop_event.is_set():
            if not self._new_orders_event.wait(timeout=1.0):
                continue

            self._new_orders_event.clear()
            time.sleep(self.debounce_sec)

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

        host = os.getenv("DB_HOST")
        port = int(os.getenv("DB_PORT"))
        user = os.getenv("DB_USER")
        password = os.getenv("DB_PASSWORD")
        db = os.getenv("DB_NAME")

        conn = pymysql.connect(
            host=host,
            port=port,
            user=user,
            password=password,
            database=db,
            cursorclass=pymysql.cursors.DictCursor,
            autocommit=True,
        )

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
