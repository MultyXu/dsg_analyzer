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
        
        self.msg_string = ""
        # publish data for the rviz panel
        self.publisher_ = self.create_publisher(String, '/dsg_statistics', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def print_dsg_statistics(self, time, G):
        self.msg_string = f"DSG Statistics at time {time:.2f} seconds:\n"
        self.msg_string += f"Number of objects: {G.get_layer(dsg.DsgLayers.OBJECTS).num_nodes()}\n"
        # self.msg_string += f"Number of places: {G.get_layer(dsg.DsgLayers.PLACES).num_nodes()}\n"
        self.msg_string += f"Number of places: {G.get_layer(dsg.DsgLayers.MESH_PLACES).num_nodes()}\n"
        self.msg_string += f"Number of rooms: {G.get_layer(dsg.DsgLayers.ROOMS).num_nodes()}\n"

        # publish in info logger
        self.get_logger().info(self.msg_string)
    
    def timer_callback(self):
        # create a string message and publish it
        msg = String()
        msg.data = self.msg_string
        self.publisher_.publish(msg)

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