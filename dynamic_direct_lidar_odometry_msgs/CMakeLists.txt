cmake_minimum_required(VERSION 3.0.2)
project(dynamic_direct_lidar_odometry_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_service_files(
  DIRECTORY srv
  FILES
  save_pcd.srv
  save_trajectories.srv
  clear_trajectories.srv
)

generate_messages()

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
)