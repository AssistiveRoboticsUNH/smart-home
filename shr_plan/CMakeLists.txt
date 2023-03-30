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
find_package(shr_msgs REQUIRED)
find_package(shr_utils REQUIRED)
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
        shr_msgs
        shr_utils
        )
add_executable(planning_controller_node src/planning_controller_node.cpp)
ament_target_dependencies(planning_controller_node ${dependencies})

generate_parameter_library(shr_plan_parameters
        src/shr_plan_parameters.yaml
        )

target_link_libraries(planning_controller_node
        shr_plan_parameters
        )


add_executable(notify_automated_action_node src/actions/notify_automated_action_node.cpp)
ament_target_dependencies(notify_automated_action_node ${dependencies})
target_link_libraries(notify_automated_action_node
        shr_plan_parameters
        )

add_executable(notify_recorded_video_node src/actions/notify_recorded_action_node.cpp)
ament_target_dependencies(notify_recorded_video_node ${dependencies})
target_link_libraries(notify_recorded_video_node
        shr_plan_parameters
        )


add_executable(move_action_node src/actions/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})
target_link_libraries(move_action_node
        shr_plan_parameters
        )

add_executable(guide_action_node src/actions/guide_action_node.cpp)
ament_target_dependencies(guide_action_node ${dependencies})
target_link_libraries(guide_action_node
        shr_plan_parameters
        )


add_executable(call_action_node src/actions/call_action_node.cpp)
ament_target_dependencies(call_action_node ${dependencies})
target_link_libraries(call_action_node
        shr_plan_parameters
)

add_executable(none_action_node src/actions/none_action_node.cpp)
ament_target_dependencies(none_action_node ${dependencies})
target_link_libraries(none_action_node
        shr_plan_parameters
        )

add_executable(detect_person_action_node src/actions/detect_person_action_node.cpp)
ament_target_dependencies(detect_person_action_node ${dependencies})
target_link_libraries(detect_person_action_node
        shr_plan_parameters
        )

add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster_node ${dependencies})
target_link_libraries(tf_broadcaster_node
        shr_plan_parameters
        )

include_directories(
        include
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    set(dependencies ${dependencies} ament_index_cpp)
    add_subdirectory(test)
endif()

install(TARGETS
        call_action_node
        detect_person_action_node
        guide_action_node
        move_action_node
        none_action_node
        notify_automated_action_node
        notify_recorded_video_node
        planning_controller_node
        tf_broadcaster_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(
        DIRECTORY launch pddl params
        DESTINATION share/shr_plan
)


ament_export_dependencies(${dependencies})

ament_package()
