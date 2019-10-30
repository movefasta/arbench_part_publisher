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

class BasePublisher(object):
    def __init__(self, prefix="", rate=50):
        rospy.init_node("arbench_part_publisher", anonymous=True)
        self.prefix = prefix
        self.pub_tf = rospy.Publisher("/tf",
                                      tf2_msgs.msg.TFMessage,
                                      queue_size=1)
        self.rate = rospy.Rate(rate)
        self.frames = {}
        self.lock = threading.Lock()

        self.services = [
            rospy.Service(prefix + "/remove_frame",
                          srvs.RemoveFrame,
                          self.removeFrameHandler),
            rospy.Service(prefix + "/add_frame",
                          srvs.AddFrame,
                          self.addFrameHandler),
            rospy.Service(prefix + "/add_frame_xyzq",
                          srvs.AddFrameXYZQ,
                          self.addFrameXYZQHandler),
            rospy.Service(prefix + "/get_frames",
                          srvs.GetFrames,
                          self.getFramesHandler)
        ]

    def run(self):
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()

    def publish(self):
        fl = []
        with self.lock:
            for fname, frame in self.frames.items():
                frame.header.stamp = rospy.Time.now()
                fl.append(frame)
            self.pub_tf.publish(fl)

    def removeFrame(self, frame_id):
        if not self.prefix == "":
            if not str(frame_id).startswith(self.prefix+"_"):
                frame_id = self.prefix + "_" + frame_id
        with self.lock:
            if frame_id in self.frames.keys():
                del self.frames[frame_id]

    def removeFrameHandler(self, req):
        self.removeFrame(req.frame_id)
        return srvs.RemoveFrameResponse()

    def addFrame(self, f):
        if not self.prefix == "":
            if not str(f.child_frame_id).startswith(self.prefix + "_"):
                f.child_frame_id = self.prefix + "_" + str(f.child_frame_id)
        with self.lock:
            self.frames[f.child_frame_id] = f

    def addFrameHandler(self, req):
        self.addFrame(req.tr)
        return srvs.AddFrameResponse()

    def addFrameXYZQ(self, parent_id, frame_id, origin, rotation):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent_id
        if not self.prefix == "":
            if not str(frame_id).startswith(self.prefix + "_"):
                frame_id = self.prefix + "_" + str(frame_id)
        t.child_frame_id = frame_id
        t.transform.translation = origin
        t.transform.rotation = rotation
        with self.lock:
            self.frames[frame_id] = t

    def addFrameXYZQHandler(self, req):
        self.addFrameXYZQ(req.parent_id, req.frame_id,
                          req.origin, req.rotation)
        return srvs.AddFrameXYZQResponse()

    def addFramePlacement(self, parent_id, frame_id, placement):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent_id
        if not self.prefix == "":
            if not str(frame_id).startswith(self.prefix + "_"):
                frame_id = self.prefix + "_" + frame_id
        t.child_frame_id = frame_id
        placement2transform(placement, t)
        with self.lock:
            self.frames[frame_id] = t

    def getFrames(self):
        return self.frames.keys()

    def getFramesHandler(self, req):
        with self.lock:
            return srvs.GetFramesResponse(self.getFrames())


class ARBenchPartPublisher(BasePublisher):
    def __init__(self, part_name,
                 x=0, y=0, z=0,  # initial origin
                 qx=0, qy=0, qz=0, qw=1,  # initial quaternion
                 rate=50, mesh="",  # Rate and optional mesh
                 red=0.53, green=0.53, blue=0.53, alpha=1):  # meshcol
        BasePublisher.__init__(self, prefix=part_name, rate=rate)
        part_frame = geometry_msgs.msg.TransformStamped()
        part_frame.header.frame_id = "world"
        part_frame.child_frame_id = self.prefix + "_part_frame"
        part_frame.transform.translation.x = x
        part_frame.transform.translation.y = y
        part_frame.transform.translation.z = z
        part_frame.transform.rotation.x = qx
        part_frame.transform.rotation.y = qy
        part_frame.transform.rotation.z = qz
        part_frame.transform.rotation.w = qw
        self.addFrame(part_frame)

        if not mesh == "":
            self.mesh_available = True
            self.pub_mesh = rospy.Publisher("visualization_marker",
                                            viz_msg.Marker,
                                            queue_size=1)
            self.mesh_marker = viz_msg.Marker()
            self.mesh_marker.header.frame_id = self.prefix + "_part_frame"
            self.mesh_marker.ns = self.prefix
            self.mesh_marker.id = 0
            self.mesh_marker.type = viz_msg.Marker.MESH_RESOURCE
            self.mesh_marker.action = 0
            self.mesh_marker.pose.position.x = 0
            self.mesh_marker.pose.position.y = 0
            self.mesh_marker.pose.position.z = 0
            self.mesh_marker.pose.orientation.x = 0
            self.mesh_marker.pose.orientation.y = 0
            self.mesh_marker.pose.orientation.z = 0
            self.mesh_marker.pose.orientation.w = 1
            self.mesh_marker.scale.x = 1e-3
            self.mesh_marker.scale.y = 1e-3
            self.mesh_marker.scale.z = 1e-3
            self.mesh_marker.color.r = red
            self.mesh_marker.color.g = green
            self.mesh_marker.color.b = blue
            self.mesh_marker.color.a = alpha
            self.mesh_marker.lifetime = rospy.Time.from_sec(1.0/rate)
            self.mesh_marker.mesh_resource = mesh
        else:
            self.mesh_available = False

    def publish(self):
        fl = []
        with self.lock:
            for fname, frame in self.frames.items():
                frame.header.stamp = rospy.Time.now()
                fl.append(frame)
            if self.mesh_available:
                mesh_marker = self.mesh_marker
                mesh_marker.header.stamp = rospy.Time.now()
                self.pub_mesh.publish(mesh_marker)
            self.pub_tf.publish(fl)

def placement2transform(pl, tr):
    tr.transform.translation.x = pl["origin"][0]
    tr.transform.translation.y = pl["origin"][1]
    tr.transform.translation.z = pl["origin"][2]
    q = tf.transformations.quaternion_about_axis(pl["rotation"]["angle"],
                                                 pl["rotation"]["axis"])
    tr.transform.rotation.x = q[0]
    tr.transform.rotation.y = q[1]
    tr.transform.rotation.z = q[2]
    tr.transform.rotation.w = q[3]


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

    part_tf = ARBenchPartPublisher(part_name=args.part_name,
                                   x=args.xyz[0], y=args.xyz[1], z=args.xyz[2],
                                   qx=args.quat[0], qy=args.quat[1],
                                   qz=args.quat[2], qw=args.quat[3],
                                   rate=args.rate, mesh=args.mesh,
                                   red=args.rgba[0], green=args.rgba[1], blue=args.rgba[2],
                                   alpha=args.rgba[3])
    if args.jsonfile is not None:
        with open(args.jsonfile, 'r') as f:
            part_props = json.load(f)
        feat_dict = part_props["features"]
        for fname in feat_dict.keys():
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = args.part_name+"_part_frame"
            t.child_frame_id = fname
            placement2transform(feat_dict[fname]["featureplacement"], t)
            part_tf.addFrame(t)
    try:
        part_tf.run()
    except rospy.ROSInterruptException:
        pass
