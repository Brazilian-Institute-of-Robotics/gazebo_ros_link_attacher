#!/usr/bin/env python3

# Copyright (c) 2023, SENAI Cimatec
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys
import rclpy
from gazebo_ros_link_attacher.srv import Attach


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('demo_attach_links')
    node.get_logger().info("Creating service to /attach")

    attach_srv = node.create_client(Attach, '/attach')
    while not attach_srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for attach service...")

    node.get_logger().info("Created service to /attach")

    # Link them
    node.get_logger().info("Attaching cube1 and cube2")

    req = Attach.Request()
    req.model_name_1 = "cube1"
    req.link_name_1 = "link"
    req.model_name_2 = "cube2"
    req.link_name_2 = "link"

    resp = attach_srv.call_async(req)
    rclpy.spin_until_future_complete(node, resp)

    node.get_logger().info("Attaching cube2 and cube3")

    req = Attach.Request()
    req.model_name_1 = "cube2"
    req.link_name_1 = "link"
    req.model_name_2 = "cube3"
    req.link_name_2 = "link"

    resp = attach_srv.call_async(req)
    rclpy.spin_until_future_complete(node, resp)

    node.get_logger().info("Attaching cube3 and cube1")
    req = Attach.Request()
    req.model_name_1 = "cube3"
    req.link_name_1 = "link"
    req.model_name_2 = "cube1"
    req.link_name_2 = "link"

    resp = attach_srv.call_async(req)
    rclpy.spin_until_future_complete(node, resp)
