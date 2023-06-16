cmake_minimum_required(VERSION 3.5)
project(shr_utils)

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
        )
add_library(${PROJECT_NAME} SHARED src/utils.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

#include_directories(
#        include
#)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(
        DIRECTORY include/
        DESTINATION include
)




ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

ament_package()
