cmake_minimum_required(VERSION 3.5)
project(simulator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# find dependencies
find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(uamv0 REQUIRED)
find_package(uam_msgs REQUIRED)
find_package (Eigen3 3.4 REQUIRED)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts/libraries
  DESTINATION share/${PROJECT_NAME})

add_library(ros_kinematic SHARED scripts/ros_kinematic.cpp)
ament_target_dependencies(ros_kinematic rclcpp std_msgs geometry_msgs rosgraph_msgs uam_msgs)
target_link_libraries(ros_kinematic ${GAZEBO_LIBRARIES})

add_library(gazebo_manipulator_kinematics_plugin SHARED scripts/gazebo_manipulator_kinematics_plugin.cpp)
ament_target_dependencies(gazebo_manipulator_kinematics_plugin rclcpp std_msgs geometry_msgs rosgraph_msgs)
target_link_libraries(gazebo_manipulator_kinematics_plugin ${GAZEBO_LIBRARIES})

add_library(ros_pid SHARED scripts/ros_pid.cpp)
ament_target_dependencies(ros_pid rclcpp std_msgs geometry_msgs rosgraph_msgs uam_msgs)
target_link_libraries(ros_pid ${GAZEBO_LIBRARIES})

add_library(ros_manipulator SHARED scripts/ros_manipulator.cpp)
ament_target_dependencies(ros_manipulator rclcpp std_msgs geometry_msgs rosgraph_msgs)
target_link_libraries(ros_manipulator ${GAZEBO_LIBRARIES})

add_library(ros_kinematic_uav SHARED scripts/ros_kinematic_uav.cpp)
ament_target_dependencies(ros_kinematic_uav rclcpp std_msgs geometry_msgs rosgraph_msgs uam_msgs)
target_link_libraries(ros_kinematic_uav ${GAZEBO_LIBRARIES})

add_library(ros_kinematic_vel_uav SHARED scripts/ros_kinematic_vel_uav.cpp)
ament_target_dependencies(ros_kinematic_vel_uav rclcpp std_msgs geometry_msgs rosgraph_msgs uam_msgs)
target_link_libraries(ros_kinematic_vel_uav ${GAZEBO_LIBRARIES})

add_library(ros_kinematic_vel_uam SHARED scripts/ros_kinematic_vel_uam.cpp)
ament_target_dependencies(ros_kinematic_vel_uam rclcpp std_msgs geometry_msgs rosgraph_msgs uam_msgs)
target_link_libraries(ros_kinematic_vel_uam ${GAZEBO_LIBRARIES})

add_library(ros_dynamics SHARED scripts/ros_dynamics.cpp)
ament_target_dependencies(ros_dynamics rclcpp std_msgs geometry_msgs rosgraph_msgs uam_msgs)
target_link_libraries(ros_dynamics ${GAZEBO_LIBRARIES} Eigen3::Eigen)

add_library(ros_kinematic_vel_obs SHARED scripts/ros_kinematic_vel_obs.cpp)
ament_target_dependencies(ros_kinematic_vel_obs rclcpp std_msgs geometry_msgs rosgraph_msgs uam_msgs)
target_link_libraries(ros_kinematic_vel_obs ${GAZEBO_LIBRARIES} Eigen3::Eigen)


ament_package()
