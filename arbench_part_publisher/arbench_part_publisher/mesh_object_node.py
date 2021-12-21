#!/usr/bin/env python
import rospy
import visualization_msgs.msg

class MeshVizPublisher(object):
    def __init__(self, marker, rate = 50):
        rospy.init_node("mesh_viz_object", anonymous=True)
        self.pub_viz = rospy.Publisher("/visualization_marker", visualization_msgs.msg.Marker, queue_size = 1)
        self.rate = rospy.Rate(rate)
        self.marker = marker
        
    def run(self):
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()

    def publish(self):
        self.marker.header.stamp = rospy.Time.now()
        self.pub_viz.publish(self.marker)

if __name__ == "__main__":
    import argparse
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
    parser.add_argument(
    parser.add_argument("--rate",
                        help="update rate of the publisher",
                        required=False
                        default=50, type=float)
    args, unknown_args = parser.parse_known_args()

    with open(args.jsonfile,'r') as f:
        mesh_props = json.load(f)
    
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = args.tf_topic
    marker.ns = ""
    
    part_mesh_viz_ = MeshVizPublisher(marker=marker, rate=arg.rate)

    try:
        part_mesh_viz.run()
    except rospy.ROSInterruptException:
        pass
    
