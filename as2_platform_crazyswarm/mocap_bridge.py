#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Bridge mocap4r2 RigidBodies to motion_capture_tracking NamedPoseArray.

Subscribes to /mocap/rigid_bodies (mocap4r2_msgs/RigidBodies) and republishes
on /poses (motion_capture_tracking_interfaces/NamedPoseArray) so the Crazyswarm2
server receives external positions.

Name mapping is read from the project config file (config/config.yaml).
Each drone entry may have:
    <namespace>:
      platform:
        ros__parameters:
          cf_name: "cf1"       # name the crazyflie server expects on /poses
          mocap_id: "34"       # streaming ID published by mocap4r2
"""

import os

from mocap4r2_msgs.msg import RigidBodies
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import yaml


class MocapBridge(Node):
    def __init__(self):
        super().__init__('mocap_bridge')

        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        self._id_to_cf: dict[str, str] = self._load_mapping(config_file)
        if self._id_to_cf:
            self.get_logger().info(f'mocap_bridge name mapping: {self._id_to_cf}')
        else:
            self.get_logger().warn(
                'mocap_bridge: no mocap_id entries found in config — '
                'forwarding all rigid bodies with their original names')

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # The crazyflie server subscribes to /poses with a 100 Hz deadline —
        # the publisher must offer a matching deadline or messages are dropped.
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            deadline=Duration(nanoseconds=int(1e9 / 100)),
        )

        self.sub = self.create_subscription(
            RigidBodies, '/mocap/rigid_bodies', self._cb, sub_qos)
        self.pub = self.create_publisher(NamedPoseArray, '/poses', pub_qos)

    def _load_mapping(self, config_file: str) -> dict[str, str]:
        """Build {mocap_id -> cf_name} from config.yaml drone entries."""
        if not config_file or not os.path.isfile(config_file):
            return {}
        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)

        mapping = {}
        for key, val in cfg.items():
            if not isinstance(val, dict):
                continue
            try:
                params = val['platform']['ros__parameters']
                mocap_id = str(params['mocap_id'])
                cf_name = str(params['cf_name'])
                mapping[mocap_id] = cf_name
            except (KeyError, TypeError):
                continue
        return mapping

    def _cb(self, msg: RigidBodies):
        out = NamedPoseArray()
        out.header = msg.header
        for rb in msg.rigidbodies:
            name = self._id_to_cf.get(rb.rigid_body_name, rb.rigid_body_name)
            np = NamedPose()
            np.name = name
            np.pose = rb.pose
            out.poses.append(np)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = MocapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
