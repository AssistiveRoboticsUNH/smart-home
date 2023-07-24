cmake_minimum_required(VERSION 3.8)
project(shr_parameters)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(shr_parameters
    params/shr_parameters.yaml
    )

install(DIRECTORY params
        DESTINATION share/shr_parameters
    )

install(
    TARGETS shr_parameters
    EXPORT shr_parametersTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

ament_export_targets(shr_parametersTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(${CMAKE_INSTALL_PREFIX}/include/shr_parameters)

ament_package()
