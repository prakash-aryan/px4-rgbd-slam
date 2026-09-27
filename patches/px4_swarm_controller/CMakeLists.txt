cmake_minimum_required(VERSION 3.8)
project(px4_swarm_controller)

# Minimal single-drone subset of the upstream px4_swarm_controller.
# Builds only the arming + waypoint nodes (no multi-drone weighted-topology
# controllers, no simulation_node.py, no custom_msgs dependency).

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${YAML_CPP_INCLUDE_DIRS})

add_executable(arming src/Arming.cpp)
ament_target_dependencies(arming rclcpp px4_msgs)

add_executable(waypoint src/ChangeWaypoint.cpp)
ament_target_dependencies(waypoint rclcpp px4_msgs)
target_link_libraries(waypoint ${YAML_CPP_LIBRARIES})

install(TARGETS arming waypoint
        DESTINATION lib/${PROJECT_NAME})

ament_package()
