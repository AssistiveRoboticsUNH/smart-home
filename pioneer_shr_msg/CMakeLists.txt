cmake_minimum_required(VERSION 3.5)
project(pioneer_shr_msg)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
        "msg/SmartSensor.msg"
        )

set(action_files
        action/SkillRequest.action
        action/EmptyRequest.action
        action/RecognizeRequest.action
        action/RecognizeTrainRequest.action
        action/ReadScriptRequest.action
        action/PlayAudioRequest.action
        action/PlayVideoRequest.action
        action/OpenImageRequest.action
        action/CallRequest.action
        )

rosidl_generate_interfaces(pioneer_shr_msg
        ${msg_files}
        ${action_files}
        DEPENDENCIES
        action_msgs
        builtin_interfaces
        )


ament_export_dependencies(rosidl_default_runtime)

install(
        DIRECTORY resources
        DESTINATION share/pioneer_shr_msg
)
ament_package()
