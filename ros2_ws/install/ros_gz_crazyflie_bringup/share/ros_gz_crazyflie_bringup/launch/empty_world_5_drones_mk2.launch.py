# Copyright 2022 Open Source Robotics Foundation, Inc.
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch
    
    # Set the environment variable
    os.environ['GZ_SIM_RESOURCE_PATH'] = f"/home/{os.getenv('USER')}/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
    
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_model_path = os.getenv('GZ_SIM_RESOURCE_PATH')

    # Load SDF files for each drone
    robot_descs = []
    for i in range(5):
        sdf_file = os.path.join(gz_model_path, f'crazyflie{i}', 'model.sdf')
        with open(sdf_file, 'r') as infp:
            robot_descs.append(infp.read())

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'empty_world_5_drones_mk2.sdf -r'
        ])}.items(),
    )

    bridge_nodes = []
    control_nodes = []
    for i in range(5):
        bridge_nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'ros_gz_bridge_{i}',
                namespace=f'crazyflie{i}',
                parameters=[{
                    'config_file': os.path.join(pkg_project_bringup, 'config', f'bridge_config_crazyflie{i}.yaml'),
                }],
                output='screen'
            )
        )
        control_nodes.append(
            Node(
                package='ros_gz_crazyflie_control',
                executable='my_control_services_mk2',
                name=f'crazyflie_control_{i}',
                namespace=f'crazyflie{i}',
                parameters=[{
                    'robot_description': robot_descs[i],
                    'robot_name': f'crazyflie{i}',
                    'robot_namespace': f'crazyflie{i}',
                    'robot_index': i,
                    'robot_prefix': f'/crazyflie{i}',
                    }],
                output='screen'
            )
        )

    return LaunchDescription(
        [gz_sim]
        + bridge_nodes
        + control_nodes
    )