cmake_minimum_required(VERSION 3.5)
project(shr_plan)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pioneer_shr_msg REQUIRED)
find_package(generate_parameter_library REQUIRED)

set(dependencies
        rclcpp
        rclcpp_action
        plansys2_msgs
        nav2_msgs
        plansys2_domain_expert
        plansys2_executor
        plansys2_planner
        plansys2_problem_expert
        plansys2_pddl_parser
        tf2_ros
        pioneer_shr_msg
        )
add_executable(planning_controller_node src/planning_controller_node.cpp)
ament_target_dependencies(planning_controller_node ${dependencies})

generate_parameter_library(shr_plan_parameters
        src/shr_plan_parameters.yaml
        )

target_link_libraries(planning_controller_node
        shr_plan_parameters
        )


add_executable(notify_action_node src/notify_action_node.cpp)
ament_target_dependencies(notify_action_node ${dependencies})

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

install(TARGETS
        planning_controller_node
        notify_action_node
        move_action_node
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
