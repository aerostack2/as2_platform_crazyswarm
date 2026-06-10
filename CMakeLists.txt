cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME as2_platform_crazyswarm)
project(${PROJECT_NAME})

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_python REQUIRED)

set(PROJECT_DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  nav_msgs
  as2_core
  as2_msgs
  geometry_msgs
  crazyflie_interfaces
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/crazyswarm_node.cpp
  src/crazyswarm_platform.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  ${PROJECT_DEPENDENCIES}
)

add_executable(${PROJECT_NAME}_swarm_node
  src/crazyswarm_swarm_nodes.cpp
  src/crazyswarm_platform.cpp)

ament_target_dependencies(${PROJECT_NAME}_swarm_node
  ${PROJECT_DEPENDENCIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_swarm_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  as2_platform_crazyswarm/mocap_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mocap_bridge
)

ament_package()
