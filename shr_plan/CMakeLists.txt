cmake_minimum_required(VERSION 3.5)
project(shr_plan)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

#if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif ()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pddl2cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shr_msgs REQUIRED)
find_package(shr_parameters REQUIRED)
find_package(shr_utils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)


generate_bt_header(bt_shr_actions
    pddl/high_level_domain.pddl
    pddl/low_level_domain.pddl
    )

set(dependencies
    ament_index_cpp
    nav2_msgs
    rclcpp
    rclcpp_action
    shr_msgs
    shr_parameters
    shr_utils
    tf2_ros
    behaviortree_cpp_v3
    builtin_interfaces
    )

add_executable(planning_controller_node src/planning_controller_node.cpp)
ament_target_dependencies(planning_controller_node ${dependencies})


add_executable(time_publisher_node src/time_publisher_node.cpp)
ament_target_dependencies(time_publisher_node ${dependencies})



target_link_libraries(planning_controller_node
    bt_shr_actions
    )

include_directories(
    include
)

install(TARGETS
    planning_controller_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    
install(TARGETS
    time_publisher_node
    DESTINATION lib/${PROJECT_NAME}
  )

install(
    DIRECTORY launch pddl
    DESTINATION share/shr_plan
)


ament_export_dependencies(${dependencies})
ament_package()
