cmake_minimum_required(VERSION 3.5)
project(pioneer_shr_cpp)

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
        rclcpp_components
        )
add_library(find_person_node src/find_person_action.cpp)
#target_include_directories(find_person_node PRIVATE
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)
#target_compile_definitions(find_person_node
#        PRIVATE "Find_PERSON_CPP_PUBLIC")


ament_target_dependencies(find_person_node ${dependencies})

rclcpp_components_register_node(find_person_node PLUGIN "find_person_request::FindPersonRequestActionServer" EXECUTABLE find_person_action_server)

include_directories(
        include
)
install(
        DIRECTORY include/
        DESTINATION include
)

install(TARGETS
        find_person_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )


ament_package()
