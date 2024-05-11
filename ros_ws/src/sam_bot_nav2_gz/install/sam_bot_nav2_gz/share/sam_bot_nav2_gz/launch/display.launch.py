import launch
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os


def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sam_bot_nav2_gz"
    ).find("sam_bot_nav2_gz")
    default_model_path = os.path.join(
        pkg_share, "src/description/sam_bot_description.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    gz_models_path = os.path.join(pkg_share, "models")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")
    run_headless = LaunchConfiguration("run_headless")
    pkg_world_file = LaunchConfiguration("pkg_world_file")


    # gazebo have to be executed with shell=False, or test_launch won't terminate it
    #   see: https://github.com/ros2/launch/issues/545
    # This code is form taken ros_gz_sim and modified to work with shell=False
    #   see: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_sim.launch.py.in
    #        https://github.com/gazebosim/ros_gz_project_template
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')]),
                                      launch_arguments=[('gz_args', [' -r -v 4 ', pkg_world_file])])
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")]),
             "use_sim_time": use_sim_time,
            }
        ],
    )


    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "sam_bot",
            "-topic",
            "robot_description",
            "-z",
            "0.15",
            "-x",
            "0.0",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/sky_cam@sensor_msgs/msg/Image@gz.msgs.Image",
            "/robot_cam@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            # Clock message is necessary for the diff_drive_controller to accept commands https://github.com/ros-controls/gz_ros2_control/issues/106
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(AndSubstitution(NotSubstitution(run_headless), use_rviz)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return launch.LaunchDescription(
        [
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=gz_models_path,
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_MODEL_PATH",
                value=gz_models_path,
            ),
            
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="True",
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                name="pkg_world_file",
                default_value=PathJoinSubstitution(
                    [pkg_share, "world/", "empty.sdf"]),
                description="Path to the world file",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value="3",
                description="Verbosity level for Ignition Gazebo (0~4).",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="",
                description="Extra args for Gazebo (ie. '-s' for running headless)",
            ),
            DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            ),
            gz_sim,
            spawn_entity,
            robot_state_publisher_node,
            rviz_node,
            bridge,
        ]
    )
