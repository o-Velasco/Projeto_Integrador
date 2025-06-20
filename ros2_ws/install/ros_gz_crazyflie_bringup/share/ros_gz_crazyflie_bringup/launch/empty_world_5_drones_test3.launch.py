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

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Set the GZ_SIM_RESOURCE_PATH if not already set
    gz_model_path = os.getenv('GZ_SIM_RESOURCE_PATH')
    if gz_model_path is None:
        gz_model_path = f"/home/{os.getenv('USER')}/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_model_path

    # Load the SDF file from "description" package
    sdf_file = os.path.join(gz_model_path, 'crazyflie', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world with 5 drones already in it
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'empty_world_cf0.sdf -r'
        ])}.items(),
    )

    # Define drone names (matching the models in the world file)
    drone_names = ['crazyflie0']

    # Create nodes list starting with Gazebo
    nodes = [gz_sim]

    # Create bridge and control nodes for each drone
    for drone_name in drone_names:
        
        # Bridge node for each drone using parameter file approach
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{drone_name}',
            parameters=[{
                'config_file': os.path.join(pkg_project_bringup, 'config', f'bridge_config_{drone_name}.yaml'),
            }],
            output='screen'
        )
        
        # Control service node for each drone
        control = Node(
            package='ros_gz_crazyflie_control',
            executable='control_services',
            name=f'control_{drone_name}',
            output='screen',
            parameters=[
                {'hover_height': 0.5},
                {'robot_prefix': f'/{drone_name}'},
                {'incoming_twist_topic': '/cmd_vel'},
                {'max_ang_z_rate': 0.4},
            ]
        )

        # Add nodes to the list
        nodes.extend([bridge, control])

    return LaunchDescription(nodes)