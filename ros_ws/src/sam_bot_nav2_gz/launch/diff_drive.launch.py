#https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/

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
"""
This Python script is a ROS2 (Robot Operating System) launch file used to start multiple nodes (independent processes) at once. 
This launch file is for a differential drive robot simulation in Gazebo, a 3D robotics simulator, and includes navigation
functionality provided by the Nav2 package.

The generate_launch_description function is the main function that ROS2 uses to execute the launch file. It begins by setting 
up paths to various packages and files. It uses the get_package_share_directory function from ament_index_python.packages to get 
the paths to the installed ROS packages. It also sets up the path to the robot's URDF (Unified Robot Description Format) file, 
which describes the robot's physical configuration.

The URDF file is processed with xacro.process_file to convert it into XML format. This is stored in the robot_desc variable. 
The script then sets up the parameters for the Nav2 navigation system. It uses the RewrittenYaml class from nav2_common.launch 
to load and possibly modify the parameters from a YAML file.

The gz_sim variable is an instance of IncludeLaunchDescription, which includes another launch file. This other launch file starts 
the Gazebo simulator with a specific world file. The spawn variable is a Node that spawns the robot into the Gazebo world. It uses 
the create executable from the ros_gz_sim package.

The robot_state_publisher is another Node that publishes the state of the robot (the positions of all its joints) to a ROS topic. 
The rviz variable is a Node that starts RViz, a 3D visualization tool for ROS. It's started with a specific configuration file.

The robot_localization_node is a Node that starts an EKF (Extended Kalman Filter) for fusing sensor data for localization. The bridge 
variable is a Node that starts a bridge between ROS topics and Gazebo messages. This allows ROS nodes and Gazebo to communicate.

The bringup_cmd variable is another instance of IncludeLaunchDescription that includes a launch file for starting the Nav2 navigation 
system. The load_joint_state_broadcaster, load_joint_velocity_controller, and load_imu_sensor_broadcaster variables are instances of 
Node that load and activate specific controllers.

Finally, the function returns a LaunchDescription that includes all the nodes, processes, and event handlers that should be started 
when the launch file is run. The SetEnvironmentVariable action is used to set the GZ_SIM_RESOURCE_PATH environment variable. The 
DeclareLaunchArgument actions are used to declare launch arguments. The RegisterEventHandler actions are used to load and activate 
the controllers when the robot is spawned into the Gazebo world.
"""

# https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter

# ROS2 (Robot Operating System) launch file  used to start multiple nodes (independent processes) at once. This launch file is for a
# differential drive robot simulation in Gazebo, a 3D robotics simulator, and includes navigation functionality provided by the Nav2 package.
# Define the generate_launch_description function, which is the main function that ROS2 uses to execute the launch file.
def generate_launch_description():

    # Setup project paths:
    # The function begins by setting up paths to various packages and files. It uses the get_package_share_directory function
    # from ament_index_python.packages to get the paths to the installed ROS packages. It also sets up the path to the robot's URDF
    # (Unified Robot Description Format) file, which describes the robot's physical configuration.
    pkg_project_share = get_package_share_directory("sam_bot_nav2_gz")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    urdf_file = os.path.join(pkg_project_share, "src/description/",
                             "sam_bot_description_ros2_control.urdf")

    # https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Substitutions.html
    pkg_world_file = LaunchConfiguration("pkg_world_file")
    nav2_params = LaunchConfiguration("nav2_params")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    SetParameter(name='use_sim_time', value=use_sim_time)

    # Load the SDF file from "description" package:
    # The URDF file is processed with xacro.process_file to convert it into XML format. This is stored in the robot_desc variable.
    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toxml()

    # Setup to launch the simulator and Gazebo world:
    # The gz_sim variable is an instance of IncludeLaunchDescription, which includes another launch file. This other launch file starts
    # the Gazebo simulator with a specific world file.
    # https://gazebosim.org/docs/harmonic/ros2_interop
    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')]),
                                      launch_arguments=[('gz_args', [' -r -v 4 ', pkg_world_file])])

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    # The robot_state_publisher is another Node that publishes the state of the robot (the positions of all its joints) to a ROS topic.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": use_sim_time,
        }],
    )

    # The spawn variable is a Node that spawns the robot into the Gazebo world. It uses the create executable from the ros_gz_sim package.
    # https://gazebosim.org/docs/harmonic/spawn_urdf
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "sam_bot",
            "-topic",
            "robot_description",
            "-z",
            "0.3",
            "-x",
            "-1.7",
        ],
        output="screen",
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
            "diff_drive_base_controller",
            "--controller-manager",
            "/controller_manager",
            "--namespace",
            "/",
        ],
    )

    load_imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--namespace",
            "/",
        ],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    # The bridge variable is a Node that starts a bridge between ROS topics and Gazebo messages. This allows ROS nodes and Gazebo to communicate.
    # https://gazebosim.org/docs/harmonic/ros2_integration
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file":
            os.path.join(pkg_project_share, "config", "ros_gz_example_bridge.yaml"),
            "qos_overrides./tf_static.publisher.durability":
            "transient_local",
            "use_sim_time":
            use_sim_time,
        }],
        output="screen",
    )

    # Visualize in RViz:
    # The rviz variable is a Node that starts RViz, a 3D visualization tool for ROS. It's started with a specific configuration file.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_project_share, "rviz", "navigation_config.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Start navigation:
    # The bringup_cmd variable is another instance of IncludeLaunchDescription that includes a launch file for starting the Nav2 navigation system.
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, "launch", "bringup_launch.py"])),
        launch_arguments={
            "params_file": nav2_params,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
        }.items(),
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

    # Start relay nodes:
    # The relay_odom, relay_cmd_vel, and relay_imu variables are Node instances that relay messages from one ROS topic to another.

    # Finally, the function returns a LaunchDescription that includes all the nodes, processes, and event handlers that should be started
    # when the launch file is run. The SetParameter action is used to set the use_sim_time parameter to True, which tells ROS to use the
    # simulated time from Gazebo. The RegisterEventHandler actions are used to load and activate the controllers when the robot is spawned
    # into the Gazebo world.
    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.path.join(pkg_project_share, "models"),
        ),
        DeclareLaunchArgument(
            name="urdf_file",
            default_value=PathJoinSubstitution([
                pkg_project_share, "src/description/", "sam_bot_description_ros2_control.urdf"
            ]),
            description="Path to the robot description file",
        ),
        DeclareLaunchArgument(
            name="map_yaml_file",
            default_value=PathJoinSubstitution(
                [pkg_project_share, "maps", "turtlebot3_world.yaml"]),
            description="Path to the map file",
        ),
        DeclareLaunchArgument(
            name="nav2_params",
            default_value=PathJoinSubstitution([pkg_project_share, "config/",
                                                "nav2_params.yaml"]),
            description="Path to the navigation parameters file",
        ),
        DeclareLaunchArgument(
            name="pkg_world_file",
            default_value=PathJoinSubstitution(
                [pkg_project_share, "world/", "turtlebot3_world.sdf"]),
            description="Path to the world file",
        ),
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="True",
            description="Flag to enable use_sim_time",
        ),
        DeclareLaunchArgument(name="rviz", default_value="true", description="Open RViz."),
        # https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawn,
            on_exit=[load_joint_state_broadcaster],
        )),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_joint_velocity_controller],
        )),
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=load_joint_velocity_controller,
            on_exit=[load_imu_sensor_broadcaster],
        )),
        bridge,
        gz_sim,
        spawn,
        robot_state_publisher,
        relay_robot_description,
        rviz,
        bringup_cmd,
    ])

# https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html
