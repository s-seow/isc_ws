import json
import threading
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timezone, timedelta

class SummarySubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("isc_summary_subscriber")
        self.declare_parameter("topic", "/optimizer_summary")
        topic = self.get_parameter("topic").value
        self._tz = timezone(timedelta(hours=8))

        self._lock = threading.Lock()
        self._payload: Optional[Dict[str, Any]] = None
        self._last_update_utc: Optional[str] = None

        self.create_subscription(String, topic, self._on_msg, 10)
        self.get_logger().info(f"Subscribed to '{topic}'")
        self.get_logger().info("Type merchant name and press Enter. Type 'exit' to quit.")

        threading.Thread(target=self._input_loop, daemon=True).start()

    def _on_msg(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        # fail events
        if not raw:
            self.get_logger().warning("Received empty optimizer summary.")
            return
        try:
            data = json.loads(raw)
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
            return
        
        with self._lock:
            self._payload = data
            self._last_update_utc = datetime.now(self._tz).strftime("%Y-%m-%dT%H:%M:%SZ")

        self.get_logger().info(f"Received optimizer summary: {len(data)} merchants")

    def _input_loop(self) -> None:
        while rclpy.ok():
            try:
                merchant = input("").strip()
            except EOFError:
                return

            if not merchant:
                continue

            if merchant.lower() in ("exit", "quit"):
                rclpy.shutdown()
                return

            with self._lock:
                payload = self._payload
                last_update_utc = self._last_update_utc

            if payload is None:
                print("No payload received yet. Wait for /optimizer_summary to publish.")
                continue

            if merchant in payload:
                self._print_merchant_block(merchant, payload[merchant], last_update_utc)
                continue

            close = [k for k in payload.keys() if k.lower() == merchant.lower()]
            if len(close) == 1:
                k = close[0]
                self._print_merchant_block(k, payload[k], last_update_utc)
                continue

            print(f"Merchant not found: {merchant}")
            suggestions = self._suggest_merchants(payload, merchant)
            if suggestions:
                print("Did you mean:")
                for s in suggestions:
                    print(f"  - {s}")

    @staticmethod
    def _suggest_merchants(payload: Dict[str, Any], query: str, max_n: int = 8):
        q = query.lower()
        hits = [k for k in payload.keys() if q in k.lower()]
        return hits[:max_n]

    def _print_merchant_block(self, merchant: str, obj: Any, last_update_utc: Optional[str]) -> None:
        if not isinstance(obj, dict):
            print(f"Merchant: {merchant}")
            print(f"  (value is {type(obj).__name__}, expected dict)")
            print(obj)
            return

        home = obj.get("home", "(unknown)")
        mwc = int(obj.get("merchant_window_capacity", 0))

        runs = obj.get("runs_by_terminal", {}) or {}
        r_t1 = int(runs.get("T1", 0))
        r_t2 = int(runs.get("T2", 0))
        r_t3 = int(runs.get("T3", 0))

        robots_by_terminal = obj.get("robots_by_terminal", {}) or {}
        rb_t1 = int(robots_by_terminal.get("T1", 0))
        rb_t2 = int(robots_by_terminal.get("T2", 0))
        rb_t3 = int(robots_by_terminal.get("T3", 0))

        robots_total = int(obj.get("robots_total", 0))

        print(f"\nMerchant: {merchant}")
        print(f"Home: {home}")
        print(f"Merchant window capacity: {mwc}")
        print(f"Runs by terminal (runs_by_terminal): T1={r_t1}, T2={r_t2}, T3={r_t3}")
        print(f"Robots by terminal (robots_by_terminal): T1={rb_t1}, T2={rb_t2}, T3={rb_t3}")
        print(f"Total robots needed for all runs: {robots_total}")
        if last_update_utc is not None:
            print(f"\n(payload last updated: {last_update_utc})\n")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SummarySubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
