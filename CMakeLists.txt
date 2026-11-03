cmake_minimum_required(VERSION 3.0.2)
project(overwatch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs nav_msgs geometry_msgs message_generation)
find_package(Eigen3 REQUIRED)

add_message_files(FILES State.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES overwatch_core
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs message_runtime)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# ROS-free core, unit-tested on synthetic inputs
add_library(overwatch_core src/speed_governor.cpp)

add_executable(overwatch_node src/overwatch_node.cpp)
add_dependencies(overwatch_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(overwatch_node overwatch_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_speed_governor test/test_speed_governor.cpp)
  target_link_libraries(test_speed_governor overwatch_core)
endif()
