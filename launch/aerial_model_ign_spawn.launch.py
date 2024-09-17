import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
  
  gazebo_models_path = 'models'
  robot_name = LaunchConfiguration('robot_name')
  sdf_model = LaunchConfiguration('sdf_model')
  pkg_path = os.path.join(get_package_share_directory("aerial_models"))
  rviz_config_file = os.path.join(pkg_path, "rviz", "robot.rviz")

  robot_name_launch_arg = DeclareLaunchArgument(
      'robot_name',
      default_value='parrot_bebop'
  )
  sdf_model_launch_arg = DeclareLaunchArgument(
      'sdf_model',
      default_value=os.path.join(pkg_path,'models/parrot_bebop','model.sdf')
  )

  gz_spawn_entity = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', robot_name,
                    '-x', '5.0',
                    '-z', '0.46',
                    '-Y', '1.57',
                    '-file', sdf_model],
                 output='screen')

  gz_ros2_bridge = Node(
      package='ros_gz_bridge',
      executable='parameter_bridge',
      arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
      output='screen'
  )

  rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )


  return LaunchDescription([
        robot_name_launch_arg,
        sdf_model_launch_arg,
        gz_spawn_entity,
        gz_ros2_bridge,
        rviz2
  ])