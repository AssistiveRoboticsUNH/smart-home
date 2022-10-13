cmake_minimum_required(VERSION 3.5)
project(shr_actions_cpp)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(shr_msgs REQUIRED)
find_package(shr_utils REQUIRED)

set(dependencies
        nav2_msgs
        rclcpp
        rclcpp_action
        rclcpp_components
        shr_msgs
        shr_utils
        tf2_ros
        )
add_executable(find_person_node src/find_person_action.cpp)
ament_target_dependencies(find_person_node ${dependencies})

include_directories(
        include
)

install(TARGETS
        find_person_node
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()
