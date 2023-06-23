//// Copyright 2019 Intelligent Robotics Lab
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.
//
//
#include <memory>
#include <string>
#include <map>
#include <vector>
//
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shr_msgs/action/read_script_request.hpp"
//
#include <shr_parameters.hpp>
#include <std_msgs/msg/bool.hpp>
//
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

class FramePublisher : public rclcpp::Node {
public:
    FramePublisher(const std::string id, const std::string loc)
            : Node("frame_publisher"+loc) {
//          Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        clock_ = rclcpp::Clock(rcl_clock_type_e::RCL_ROS_TIME);
        auto func = [this]() -> void { timer_callback(); };
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), func);
        id_ = id + "_robot_pos";
        loc_ = loc;
    }

    void timer_callback() {
        auto param = get_parameter("tf_values." + loc_);
        std::vector<double> pose_ = param.as_double_array();

        geometry_msgs::msg::TransformStamped t;

        // Fill in the message
        t.header.frame_id = "map";
        t.child_frame_id = id_;

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = pose_[0];
        t.transform.translation.y = pose_[1];
        t.transform.translation.z = pose_[2];

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        t.transform.rotation.x = pose_[3];
        t.transform.rotation.y = pose_[4];
        t.transform.rotation.z = pose_[5];
        t.transform.rotation.w = pose_[6];

        t.header.stamp = clock_.now();
        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string id_;
    std::string loc_;
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;


    auto n = std::make_shared<rclcpp::Node>("params");
    auto param_listener = std::make_shared<shr_parameters::ParamListener>(n);
    auto params = param_listener->get_params();

    std::vector<std::shared_ptr<FramePublisher>> nodes;
    for (auto loc : params.tf_frames){
        auto node = std::make_shared<FramePublisher>(loc, loc);
        auto listener = std::make_shared<shr_parameters::ParamListener>(node);
        exe.add_node(node);
        nodes.push_back(node);
    }

    exe.spin();

    rclcpp::shutdown();

    return 0;
}