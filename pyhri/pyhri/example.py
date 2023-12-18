# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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

import cv2
from pyhri import HRIListener
import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.node import Node


class NodeShowFaces(Node):
    def __init__(self):
        super().__init__('hri_show_faces')
        self.hri_listener = HRIListener('hri_show_faces_listener')  # this creates another node
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        for id, face in self.hri_listener.faces.items():
            if (face.cropped):
                cv2.imshow(f'Cropped face {id}', face.cropped)
            if (face.aligned):
                cv2.imshow(f'Aligned face {id}', face.aligned)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    node = NodeShowFaces()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.destroy_node()


if __name__ == '__main__':
    main()
