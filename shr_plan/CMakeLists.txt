cmake_minimum_required(VERSION 3.5)
project(shr_plan)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(cff_plan_solver REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pddl2cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shr_msgs REQUIRED)
find_package(shr_utils REQUIRED)
find_package(tf2_ros REQUIRED)


generate_bt_header(bt_shr_actions
    pddl/medicine_domain.pddl
    pddl/food_domain.pddl
    pddl/midnight_domain.pddl
    pddl/high_level_domain.pddl
    )

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    nav2_msgs
    tf2_ros
    shr_msgs
    shr_utils
    cff_plan_solver
    )

add_executable(planning_controller_node src/planning_controller_node.cpp)
ament_target_dependencies(planning_controller_node ${dependencies})

#add_executable(planning_controller_node_sim src/planning_controller_node.cpp)
#add_compile_definitions(planning_controller_node_sim USE_SIM)
#ament_target_dependencies(planning_controller_node_sim ${dependencies})

generate_parameter_library(shr_plan_parameters
    src/shr_plan_parameters.yaml
    )

target_link_libraries(planning_controller_node
    shr_plan_parameters
    bt_shr_actions
    )

add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster_node ${dependencies})
target_link_libraries(tf_broadcaster_node
    shr_plan_parameters
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

install(
    DIRECTORY launch pddl
    DESTINATION share/shr_plan
)


ament_export_dependencies(${dependencies})
ament_package()
