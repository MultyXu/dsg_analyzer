#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hydra_ros import DsgSubscriber
import spark_dsg as dsg

class DsgAnalyzer(Node):
    def __init__(self):
        super().__init__('dsg_analyzer')

        self.dsg = None
        self.dsg_lock = threading.Lock()
        self.last_dsg_time = None
        DsgSubscriber(self, "/hamilton/hydra/backend/dsg", self.dsg_callback)

    def print_dsg_statistics(self, time, G):
        self.get_logger().info(f"DSG Statistics at time {time:.2f} seconds:")
        self.get_logger().info(f"Number of objects: {G.get_layer(dsg.DsgLayers.OBJECTS).num_nodes()}")
        self.get_logger().info(f"Number of places: {G.get_layer(dsg.DsgLayers.PLACES).num_nodes()}")
        self.get_logger().info(f"Number of rooms: {G.get_layer(dsg.DsgLayers.ROOMS).num_nodes()}")
        
    def dsg_callback(self, header, dsg):
        with self.dsg_lock:
            self.dsg = dsg
            self.last_dsg_time = self.get_clock().now().nanoseconds * 1e-9
            self.print_dsg_statistics(self.last_dsg_time, self.dsg)

def main(args=None):
    rclpy.init(args=args)

    dsg_analyzer = DsgAnalyzer()

    rclpy.spin(dsg_analyzer)

    dsg_analyzer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()