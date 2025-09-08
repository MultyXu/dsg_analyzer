#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from hydra_ros import DsgSubscriber
import spark_dsg as dsg
from dsg_analyzer.dsg_analyzer import get_object_label_histogram

# def get_object_label_histogram(G: dsg.DynamicSceneGraph):
#     key = G.get_layer_key(dsg.DsgLayers.OBJECTS)
#     labelspace = G.get_labelspace(key.layer, key.partition)

#     histogram = {}
#     for name in labelspace.names_to_labels:
#         histogram[name] = 0

#     for node in G.get_layer(dsg.DsgLayers.OBJECTS).nodes:
#         label = labelspace.get_node_category(node)
#         if label is not None:
#             histogram[label] += 1
#         else:
#             raise ValueError(f"Node {node.id.str()} has no label in the labelspace.")

#     return histogram

class DsgAnalyzerNode(Node):
    def __init__(self):
        super().__init__('dsg_analyzer')

        self.dsg = None
        self.dsg_lock = threading.Lock()
        self.last_dsg_time = None
        DsgSubscriber(self, "/hamilton/hydra/backend/dsg", self.dsg_callback)
        
        self.msg_string = ""
        self.semantic_histogram = {}
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

        all_histogram = get_object_label_histogram(G)
        self.semantic_histogram = {k: v for k, v in all_histogram.items() if v > 0}
        self.msg_string += f"Object label histogram: {self.semantic_histogram}\n"
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

    dsg_analyzer_node = DsgAnalyzerNode()

    rclpy.spin(dsg_analyzer_node)

    dsg_analyzer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()