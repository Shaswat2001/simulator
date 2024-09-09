from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    declared_param_list = []

    declared_param_list.append(DeclareLaunchArgument(
            "use_rviz",
            default_value="True",
            description="Visualize robot in RViZ"))

    declared_param_list.append(DeclareLaunchArgument(
        "description_package",
        default_value="simulator",
        description="ROS2 package name"))

    declared_param_list.append(DeclareLaunchArgument(
            "description_file",
            default_value="parrot_bebop.xacro",
            description="URDF/XACRO description file with the robot."))
    
    use_rviz = LaunchConfiguration('use_rviz')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file])
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content,value_type=str)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name='joint_state_publisher',
    )

    robot_state_publisher_node = Node(
        name='robot_state_publisher',
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )
    
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ]

    return LaunchDescription(declared_param_list + nodes_to_start)
