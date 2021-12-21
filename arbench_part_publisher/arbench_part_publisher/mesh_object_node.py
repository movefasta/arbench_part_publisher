#!/usr/bin/env python
import rclpy
import sys
import threading
import visualization_msgs.msg
from rclpy.exceptions import ROSInterruptException

class MeshVizPublisher(object):
    def __init__(self, marker, rate = 50):
        rclpy.init(args=sys.argv)
        node = rclpy.create_node('mesh_viz_object')
        self.node = node
        self.thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        self.thread.start()
        self.rate = node.create_rate(2)
        self.pub_viz = node.create_publisher(visualization_msgs.msg.Marker, "/visualization_marker", 10)
        self.marker = marker
        
    def run(self):
        while rclpy.ok():
            self.publish()
            self.rate.sleep()

    def publish(self):
        self.marker.header.stamp = self.node.get_clock().now().to_msg()
        self.pub_viz.publish(self.marker)

def main():
    import argparse
    import json
    parser = argparse.ArgumentParser(description = "Mesh marker publisher for a part.")
    parser.add_argument("tf_topic",
                        help="The tf frame that the mesh follows.")
    parser.add_argument("meshlocation",
                        help="The location of the mesh")
    parser.add_argument("jsonfile",
                        help="The location of the jsonfile describing the mesh_marker")
    parser.add_argument("--ns",
                        help="namespace of the part",
                        required=False)
    parser.add_argument("--rate",
                        help="update rate of the publisher",
                        required=False,
                        default=50, type=float)
    args, unknown_args = parser.parse_known_args()

    with open(args.jsonfile,'r') as f:
        mesh_props = json.load(f)
    
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = args.tf_topic
    marker.ns = ""
    
    part_mesh_viz = MeshVizPublisher(marker=marker, rate=args.rate)

    try:
        part_mesh_viz.run()
    except ROSInterruptException:
        pass
    
