cmake_minimum_required(VERSION 3.10.0)
project(dynamic_direct_lidar_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Debug")

find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  dynamic_direct_lidar_odometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  message_generation
  jsk_recognition_msgs
  cv_bridge
  image_transport
  octomap_ros
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_direct_lidar_odometry_msgs
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
    cv_bridge
    image_transport
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    nano_gicp
    nanoflann
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})

# NanoGICP
add_library(nano_gicp STATIC
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
target_include_directories(nano_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Detection
add_library(detection STATIC
  src/detection/detection.cpp
)
target_link_libraries(detection ${PCL_LIBRARIES} ${OpenMP_LIBS} ${catkin_LIBRARIES} tracking)
target_include_directories(detection PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Tracking
add_library(tracking STATIC
  src/tracking/tracking.cpp
  src/tracking/bounding_box_filter.cpp
  src/tracking/kalman.cpp
  src/tracking/hungarian.cpp
)
target_link_libraries(tracking ${PCL_LIBRARIES} ${OpenMP_LIBS} ${catkin_LIBRARIES})
target_include_directories(tracking PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Utility
add_library(util STATIC
  src/util/trajectories_server.cpp
  src/util/trajectory.cpp
)
target_link_libraries(util ${catkin_LIBRARIES})
target_include_directories(util PUBLIC include)

# Odometry Node
add_executable(ddlo_odom_node src/odometry/odom_node.cc src/odometry/odom.cc)
add_dependencies(ddlo_odom_node ${catkin_EXPORTED_TARGETS})
target_compile_options(ddlo_odom_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(ddlo_odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp detection)

# Mapping Node
add_executable (ddlo_map_node src/odometry/map_node.cc src/odometry/map.cc)
add_dependencies(ddlo_map_node ${catkin_EXPORTED_TARGETS})
target_compile_options(ddlo_map_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(ddlo_map_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads)

# Trajectories
add_executable(trajectories_server src/util/trajectories_server.cpp)
add_dependencies(trajectories_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectories_server ${catkin_LIBRARIES} util)
