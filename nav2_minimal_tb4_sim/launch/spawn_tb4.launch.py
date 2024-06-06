# Copyright (C) 2023 Open Source Robotics Foundation
# Copyright (C) 2024 Open Navigation LLC
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
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_simulator = LaunchConfiguration('use_simulator')
    robot_name = LaunchConfiguration('robot_name')
    # robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose', default='-8.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot4',
        description='name of the robot')

    # declare_robot_sdf_cmd = DeclareLaunchArgument(
    #     'robot_sdf',
    #     default_value=os.path.join(desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro'),
    #     description='Full path to robot sdf file to spawn the robot in gazebo')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[
            {
                'config_file': os.path.join(
                    sim_dir, 'configs', 'tb4_bridge.yaml'
                ),
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
    )

    camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_image',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=['/rgbd_camera/image'])

    camera_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_depth',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=['/rgbd_camera/depth_image'])

    spawn_model = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            # '-file', Command(['xacro', ' ', robot_sdf]), # TODO SDF file is unhappy, not sure why
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(desc_dir)).parent.resolve()))

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    # ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(set_env_vars_resources)

    ld.add_action(bridge)
    ld.add_action(camera_bridge_image)
    ld.add_action(camera_bridge_depth)
    ld.add_action(spawn_model)
    return ld
