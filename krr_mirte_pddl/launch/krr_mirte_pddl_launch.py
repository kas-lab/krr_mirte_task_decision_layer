# Copyright 2025 KAS-Lab
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    krr_mirte_pddl_path = get_package_share_directory('krr_mirte_pddl')
    plansys_path = get_package_share_directory('plansys2_bringup')
    plansys2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                plansys_path,
                'launch',
                'plansys2_bringup_launch_distributed.py')),
            launch_arguments={
                'model_file':
                    krr_mirte_pddl_path + '/pddl/domain_p5.pddl',
                'problem_file':
                    krr_mirte_pddl_path + '/pddl/problem_p5.pddl',
                # 'params_file':
                #     krr_mirte_pddl_path + '/config/plansys2_params.yaml',
            }.items()
        )

    waypoints_file = os.path.join(krr_mirte_pddl_path, 'config', 'waypoints.yml')
    krr_mirte_pddl_node = Node(
        package='krr_mirte_pddl',
        executable='krr_mirte_pddl',
        # parameters=[waypoints_file]
    )

    pddl_move_action_node = Node(
        package='krr_mirte_pddl',
        executable='action_move',
        parameters=[waypoints_file, {'action_name': 'move'}]
    )

    return LaunchDescription([
        plansys2_bringup,
        krr_mirte_pddl_node,
        pddl_move_action_node,
    ])