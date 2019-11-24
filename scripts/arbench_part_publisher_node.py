#!/usr/bin/env python
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#
# Publisher for frames defined in FreeCAD ARBench.
#
# The purpose of this script is to provide a publisher of the frames of
# interest attached to a part. These frames can be updated using the services
# provided.
#
# Author: Mathias Hauan Arbo
# Website: https://github.com/mahaarbo/arbench_part_publisher
# Date: 18. Jan. 2018

import rospy
import tf
import tf2_ros
import tf2_msgs
import geometry_msgs.msg
import threading
import arbench_part_publisher.srv as srvs
import visualization_msgs.msg as viz_msg


class PartsPublisher(object):
    """This is an object which takes a set of arbench_part descriptors,
    and publishes their information. It also exposes a set of services
    which can be used to modify the frames involved.

    mesh_freq defines how many tf publishings should happen before we
    publish. Set it to -1 to never republish
    """
    def __init__(self, parts, prefix="", rate=50, mesh_freq=10):
        rospy.init_node("parts_publisher", anonymous=True)
        self.prefix = prefix  # namespace of all services
        self.rate = rospy.Rate(rate)
        self.pub_tf = rospy.Publisher("/tf",
                                      tf2_msgs.msg.TFMessage,
                                      queue_size=1)
        self.pub_viz = rospy.Publisher("/visualization_marker_array",
                                       viz_msg.MarkerArray,
                                       queue_size=1)
        self.parts = parts  # List of arbench parts
        self.lock = threading.Lock()
        self.services = []  # List of all services
        self.mesh_freq = mesh_freq  # every mesh_freq'd publish, add mesh too.
        self.setup_parts()
        self.publish_viz()

    def setup_parts(self):
        prefix = self.prefix
        # To bind the part label to the call, we use a bit hacky lambda
        # approach
        for part in self.parts:
            self.services += [
                rospy.Service(prefix+"/"+part.safe_label+"/remove_frame",
                              srvs.RemoveFrame,
                              lambda req: self.remove_frame(part, req)),
                rospy.Service(prefix+"/"+part.safe_label+"/add_frame",
                              srvs.AddFrame,
                              lambda req: self.add_frame(part, req)),
                rospy.Service(prefix+"/"+part.safe_label+"/get_frames",
                              srvs.GetFrames,
                              lambda req: self.get_frames(part, req))
            ]

    def remove_frame(self, part, req):
        with self.lock:
            if req.frame_id in part.frames.keys():
                del part.frames[req.frame_id]
        return srvs.RemoveFrameResponse()

    def add_frame(self, part, req):
        child_frame_id = req.tr.child_frame_id
        with self.lock:
            part.frames[child_frame_id] = req.tr
        return srvs.AddFrameResponse()


    def get_frames(self, part, req):
        with self.lock:
            return srvs.GetFramesResponse(part.frames.keys())

    def publish_tf(self):
        fl = []
        with self.lock:
            for part in self.parts:
                for fname in part.frames.keys():
                    part.frames[fname].header.stamp = rospy.Time.now()
                    fl.append(part.frames[fname])
            self.pub_tf.publish(fl)

    def publish_viz(self):
        ml = []
        with self.lock:
            for idx, part in enumerate(self.parts):
                if part.marker is not None:
                    part.marker.ns = self.prefix
                    part.marker.id = idx
                    ml.append(part.marker)
            self.pub_viz.publish(viz_msg.MarkerArray(ml))

    def run(self):
        idx = 0
        while not rospy.is_shutdown():
            if idx == self.mesh_freq:
                idx = 0
                self.publish_viz()
            if self.mesh_freq >= 0:
                # Set mesh_freq to -1 to never update
                idx += 1
            self.publish_tf()
            self.rate.sleep()


class ArbenchPart(object):
    """This is an Arbench part."""
    def __init__(self, label, frames={}, mesh_uri="", marker=None):
        self.label = label
        self.safe_label = label.replace("-", "_")
        self.frames = frames
        # Setup marker
        if mesh_uri != "":
            self.set_marker(mesh_uri)
        else:
            self.marker = marker
        # Setup part frame
        tr = geometry_msgs.msg.TransformStamped()
        tr.header.frame_id = "world"
        tr.child_frame_id = label + "_part_frame"
        tr.transform.translation.x = 0
        tr.transform.translation.y = 0
        tr.transform.translation.z = 0
        tr.transform.rotation.x = 0
        tr.transform.rotation.y = 0
        tr.transform.rotation.z = 0
        tr.transform.rotation.w = 0
        self.frames[tr.child_frame_id] = tr

    def __eq__(self, other):
        """It is assumed each part has a unique label"""
        if self.label == other.label:
            return True
        else:
            return False

    def set_marker(self, mesh_uri):
        self.marker = viz_msg.Marker()
        self.marker.header.frame_id = self.label+"_part_frame"
        self.marker.id = 0
        self.marker.type = viz_msg.Marker.MESH_RESOURCE
        self.marker.action = 0
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1
        self.marker.scale.x = 1e-3
        self.marker.scale.y = 1e-3
        self.marker.scale.z = 1e-3
        self.marker.color.r = .53
        self.marker.color.g = .53
        self.marker.color.b = .53
        self.marker.color.a = 1
        self.marker.lifetime = rospy.Time.from_sec(0)
        self.marker.mesh_resource = mesh_uri

    def add_feature(self, label, x, y, z, qx, qy, qz, qw):
        """Add a feature to the part.."""
        tr = geometry_msgs.msg.TransformStamped()
        tr.header.frame_id = self.label + "_part_frame"
        # ROS namespacing doesn't allow "-" to be used
        tr.child_frame_id = self.label + "_" + label.replace("-", "_")
        tr.transform.translation.x = x
        tr.transform.translation.y = y
        tr.transform.translation.z = z
        tr.transform.rotation.x = qx
        tr.transform.rotation.y = qy
        tr.transform.rotation.z = qz
        tr.transform.rotation.w = qw
        self.frames[tr.child_frame_id] = tr


if __name__ == "__main__":
    import argparse
    import json
    parser = argparse.ArgumentParser(description="TF publisher for a part and its frames.")
    parser.add_argument("part_name",
                        help="The part name",
                        default="tfpart")
    parser.add_argument("-rate",
                        help="update rate of the publisher",
                        required=False,
                        default=50, type=float)
    parser.add_argument("-xyz",
                        help="initial xyz coordinates. E.g. -xyz 0.1 0.2 0.3",
                        nargs=3, default=[0.0, 0.0, 0.0], type=float)
    parser.add_argument("-quat",
                        help="initial quaternion. E.g. -quat 0.0 0.0 0.0 1.0",
                        nargs=4, default=[0.0, 0.0, 0.0, 1.0], type=float)
    parser.add_argument("-jsonfile",
                        help="Json file from which to import feature frames",
                        type=str)
    parser.add_argument("-mesh", default="",
                        help="STL or COLLADA file for the mesh",
                        type=str)
    parser.add_argument("-rgba",
                        help="rgba for mesh. E.g. -rgba 0.1 0.2 0.3 1.0",
                        nargs=4, default=[.53, .53, .53, 1.], type=float)

    args, unknown_args = parser.parse_known_args()

    # Define part and modify according to arguments
    part = ArbenchPart(label=args.part_name,
                       mesh_uri=args.mesh)
    tr = part.frames[args.part_name+"_part_frame"]
    tr.transform.translation.x = args.xyz[0]
    tr.transform.translation.y = args.xyz[1]
    tr.transform.translation.z = args.xyz[2]
    tr.transform.rotation.x = args.quat[0]
    tr.transform.rotation.y = args.quat[1]
    tr.transform.rotation.z = args.quat[2]
    tr.transform.rotation.w = args.quat[3]
    part.marker.color.r = args.rgba[0]
    part.marker.color.g = args.rgba[1]
    part.marker.color.b = args.rgba[2]
    part.marker.color.a = args.rgba[3]

    part_pub = PartsPublisher([part], rate=50, mesh_freq=300)
    try:
        part_pub.run()
    except rospy.ROSInterruptException:
        pass
