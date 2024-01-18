# Copyright (c) 2024, SENAI Cimatec
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

import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    # Start Gazebo with an empty world file
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch/gazebo.launch.py')
    world_file = os.path.join(
        get_package_share_directory('gazebo_ros_link_attacher'), 'worlds/test_attacher.world')

    if not pathlib.Path(world_file).exists():
        exc = 'World file ' + world_file + ' does not exist'
        raise Exception(exc)

    launch_args = [('world', world_file),
                   ('verbose', 'true'),
                   ('debug', 'true'),
                   ('gui', 'true'),
                   ('pause', 'false')]

    print("world file: ", world_file)
    gazebo_launch_description = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(gazebo_launch), launch_arguments=launch_args)

    group = GroupAction([
        gazebo_launch_description,
    ])

    return [group]


def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
