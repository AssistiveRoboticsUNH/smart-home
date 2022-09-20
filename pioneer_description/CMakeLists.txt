cmake_minimum_required(VERSION 3.5)
project(pioneer_description)

# find dependencies
find_package(ament_cmake REQUIRED)


install(
    DIRECTORY launch meshes urdf rviz
    DESTINATION share/pioneer_description
)


ament_package()
