import traceback
from unittest.mock import patch

import rclpy
from rclpy.node import Node

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

        self.declare_parameter("target_date", "1/12/2024")  
        self.declare_parameter("poll_period_sec", 5)

        self.target_date = self.get_parameter("target_date").get_parameter_value().string_value
        poll_period = self.get_parameter("poll_period_sec").get_parameter_value().integer_value
        #ros2 run isc_task_optimizer optimizer_node --ros-args -p target_date:=1/12/2024 -p poll_period_sec:=5

        self.get_logger().info(
            f"Started. Reading from MySQL connection: target_date='{self.target_date}', "
            f"period={poll_period}s"
        )

        self._timer = self.create_timer(poll_period, self._on_tick)


    def _on_tick(self) -> None:
        try:
            # run_simulation takes target_date and reads from MySQL
            with patch("builtins.input", lambda *args, **kwargs: "n"):
                summary = run_logic(self.target_date)

            size = len(summary or {})
            print()
            self.get_logger().info(
                f"Simulation complete for {self.target_date} — merchants summarized: {size}"
            )

        except Exception as e:
            self.get_logger().error(f"Simulator call failed: {e}\n{traceback.format_exc()}")


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
