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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


import xacro


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    pkg_project_share = os.path.join(
        get_package_share_directory('sam_bot_nav2_gz'))

    # https://github.com/ros-planning/moveit2/blob/main/moveit_configs_utils/moveit_configs_utils/moveit_configs_builder.py
    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="sam_bot_nav2_gz_moveit_config").to_moveit_configs()

    #for v, k in moveit_config.items():
    #    print(v, k)
    #    print('******************')
    #exit()
    #print(moveit_config["planning_scene_monitor_parameters"])
    #print('**************************')

    
    #moveit_controllers = {
    #    moveit_config["moveit_simple_controller_manager"],
    #    moveit_config["moveit_controller_manager"],
    #}

    #trajectory_execution = {
    #    moveit_config["moveit_manage_controllers"],
    #}

    #planning_scene_monitor_parameters = {
    #    "publish_planning_scene": True,
    #    "publish_geometry_updates": True,
    #    "publish_state_updates": True,
    #    "publish_transforms_updates": True,
    #}

    print(dir(moveit_config))

    # Start the actual move_group node/action server
    # https://moveit.picknik.ai/main/doc/concepts/move_group.html
    # https://github.com/ros-planning/moveit2_tutorials/blob/main/doc/examples/move_group_interface/launch/move_group.launch.py
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {'use_sim_time': use_sim_time}],
    )

    robot_description = moveit_config.robot_description
 
    gz_sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
    
    # https://github.com/ros/robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, 
                    {'use_sim_time': use_sim_time}],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot_arm",
            "-topic",
            "robot_description",
            "-z",
            ".25",
            "-x",
            "-1.7",
        ],
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time
        }]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # Visualize in RViz:
    # The rviz variable is a Node that starts RViz, a 3D visualization tool for ROS. It's started with a specific configuration file.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],        
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("sam_bot_nav2_gz_moveit_config"), "config", "moveit.rviz"),
        ],
    )

    # Load controllers:
    # The load_joint_state_broadcaster, load_joint_velocity_controller, and load_imu_sensor_broadcaster variables are instances of ExecuteProcess
    # that load and activate specific controllers.
    # https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--namespace",
            "/",
        ],
    )

    load_joint_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_group_controller",
            "--controller-manager",
            "/controller_manager",
            "--namespace",
            "/",
        ],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("sam_bot_nav2_gz_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    # https://control.ros.org/iron/doc/getting_started/getting_started.html
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    relay_robot_description = Node(
        name="relay_robot_description",
        package="topic_tools",
        executable="relay",
        parameters=[{
            "input_topic": "/robot_description",
            "output_topic": "/controller_manager/robot_description",
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )


    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.path.join(pkg_project_share, '../'),
        ),
        bridge,
        # Launch gazebo environment
        gz_sim,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_velocity_controller],
            )
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
        move_group_node,
        rviz,
        #relay_robot_description,
        #ros2_control_node,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
