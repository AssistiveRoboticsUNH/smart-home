cmake_minimum_required(VERSION 3.8)
project(yaml_tf_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
        rclcpp
        tf2_ros
        geometry_msgs
        ament_index_cpp
        )

add_executable(aptag_broadcast_node src/aptag_broadcast.cpp)
ament_target_dependencies(aptag_broadcast_node ${dependencies})
target_include_directories(aptag_broadcast_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(aptag_broadcast_node PUBLIC ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(aptag_broadcast_node yaml-cpp)


target_include_directories(aptag_broadcast_node  PUBLIC
        ${YAML_CPP_LIBRARIES}
        )

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
        aptag_broadcast_node
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
