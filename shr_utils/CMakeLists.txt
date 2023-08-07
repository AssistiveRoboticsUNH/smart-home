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
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(shr_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(assimp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


set(dependencies
        rclcpp
        rclcpp_action
        nav2_msgs
        tf2_ros
        shr_msgs
        )
add_library(${PROJECT_NAME} SHARED src/utils.cpp src/geometry.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} assimp::assimp Eigen3::Eigen)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

if (BUILD_TESTING)
    add_executable(${PROJECT_NAME}_test test/test.cpp)
    ament_target_dependencies(${PROJECT_NAME}_test ${dependencies})
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    target_include_directories(${PROJECT_NAME}_test PUBLIC include/)

    install(
        DIRECTORY test/data
        DESTINATION share/shr_utils/test/
    )

endif ()


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
ament_export_dependencies(${dependencies} assimp Eigen3)
ament_export_include_directories(include)

ament_package()
