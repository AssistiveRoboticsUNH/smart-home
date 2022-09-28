cmake_minimum_required(VERSION 3.5)
project(pioneer_shr)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


find_package(ament_cmake REQUIRED)

find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)

set(action_files
        action/SkillRequest.action
        )

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${action_files}
        DEPENDENCIES
        action_msgs
        )

add_executable(approach_person_service src/approachPersonService.cpp)

ament_export_dependencies(
        approach_person_service
)
ament_target_dependencies(approach_person_service
        action_msgs
        nav2_msgs
        std_msgs
        tf2_geometry_msgs
        geometry_msgs
        tf2_ros
        rclcpp_components
        )
#add_executable(run_script_service src/runScriptService.cpp)
#target_link_libraries(
#	run_script_service
#	${catkin_LIBRARIES}
#)
#
#add_executable(db_monitor_service src/dbMonitorService.cpp)
#target_link_libraries(
#	db_monitor_service
#	${catkin_LIBRARIES}
#	${CPR_LIBRARIES}
#)
#
#add_executable(move_to_service src/moveToService.cpp)
#target_link_libraries(
#	move_to_service
#	${catkin_LIBRARIES}
#)
#
#add_executable(executive src/executive.cpp)
#target_link_libraries(
#	executive
#	${catkin_LIBRARIES}
#)


install(TARGETS
        approach_person_service
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
