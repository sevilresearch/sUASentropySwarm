'''
Created by Juan José Molina - March 2026 for ROS2 

'''
#!/usr/bin/env python3
# entropy_swarm_node.py
import time
import threading
import rclpy
import traceback
from rclpy.node import Node

from .CentralControl import CentralControl
from .Strategies.Crazyswarm2Strategy import Crazyswarm2Strategy



class EntropySwarmNode(Node):
    
    def __init__(self):
        super().__init__("entropy_swarm")

        # same GUI inputs as your original
        self.declare_parameter("num_of_uavs", 3)
        self.declare_parameter("separating_distance", 5.0)
        self.declare_parameter("row_length", 3)
        
        self.declare_parameter("takeoff_delay_sec", 25.0)
        self.takeoff_delay = float(self.get_parameter("takeoff_delay_sec").value)
 
         
        n = int(self.get_parameter("num_of_uavs").value)
        sep = float(self.get_parameter("separating_distance").value)
        rows = int(self.get_parameter("row_length").value)

        # Configure ROS resources used by the strategy (TF + node handle)
        Crazyswarm2Strategy.configure_ros(self)

        # Build controller + run (same sequence as your GUI)
        strategy = Crazyswarm2Strategy()
        self.controller = CentralControl(strategy=strategy)

        self.controller.init_system(num_of_uavs=n, separating_distance=sep, row_length=rows)
        self.controller.connect_to_environment()
        self.controller.uav_init()  # calls move_by_heading(0,0,-2,1,0) :contentReference[oaicite:5]{index=5}

        def _run():
            try:
                self.get_logger().info(f"Waiting {self.takeoff_delay:.1f}s for takeoff...")
                time.sleep(self.takeoff_delay)

                self.get_logger().info("Starting entropyFormation()")
                self.controller.entropyFormation()

            except Exception:
                self.get_logger().error("Exception inside entropyFormation():\n" + traceback.format_exc())

            finally:
                self.get_logger().info("Landing + shutdown")
                self.controller.close_connection()
                rclpy.shutdown()

        threading.Thread(target=_run, daemon=True).start()


def main():
    rclpy.init()
    node = EntropySwarmNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()