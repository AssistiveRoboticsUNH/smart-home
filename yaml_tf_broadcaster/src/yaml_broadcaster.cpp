//
// Created by ola on 7/19/23.
//

#include <memory>
#include <string>
#include <map>
#include <vector>
//
#include "rclcpp/rclcpp.hpp"
//
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <Eigen/Dense>
#include <filesystem>

class AptagFramePublisher : public rclcpp::Node {
public:
    AptagFramePublisher(const std::vector<std::tuple<Eigen::Matrix4d, std::string, std::string>> &rot_vec,
                        const std::vector<std::tuple<Eigen::VectorXd, std::string, std::string>> &quat_vec)
            : Node("aptag_frame_publisher") {
//          Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        clock_ = rclcpp::Clock(rcl_clock_type_e::RCL_ROS_TIME);

        for (const auto &tup: rot_vec) {
            auto transformation_matrix = std::get<0>(tup);
            auto frame_id = std::get<1>(tup);
            auto id = std::get<2>(tup);

            Eigen::Affine3d affine(transformation_matrix);
            Eigen::Quaterniond quaternion(affine.linear());
            Eigen::Vector3d translation(affine.translation());

            // Fill in the message
            geometry_msgs::msg::TransformStamped t;
            t.header.frame_id = frame_id;
            t.child_frame_id = id;
            t.transform.translation.x = translation.x();
            t.transform.translation.y = translation.y();
            t.transform.translation.z = translation.z();
            t.transform.rotation.x = quaternion.x();
            t.transform.rotation.y = quaternion.y();
            t.transform.rotation.z = quaternion.z();
            t.transform.rotation.w = quaternion.w();

            t_vec.push_back(t);

        }

        for (const auto &tup: quat_vec) {
            auto transformation_vector_ = std::get<0>(tup);
            auto frame_id = std::get<1>(tup);
            auto id = std::get<2>(tup);

            // Fill in the message
            geometry_msgs::msg::TransformStamped t;
            t.header.frame_id = frame_id;
            t.child_frame_id = id;
            t.transform.translation.x = transformation_vector_(0);
            t.transform.translation.y = transformation_vector_(1);
            t.transform.translation.z = transformation_vector_(2);
            t.transform.rotation.x = transformation_vector_(3);
            t.transform.rotation.y = transformation_vector_(4);
            t.transform.rotation.z = transformation_vector_(5);
            t.transform.rotation.w = transformation_vector_(6);

            t_vec.push_back(t);
        }

//        auto func = [this]() -> void { timer_callback_matrix(); };
        for (auto &t: t_vec) {
            t.header.stamp = clock_.now();
            tf_broadcaster_->sendTransform(t);
        }
//        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), func);

    }

//    void timer_callback_matrix() {
//        // Send the transformation
//        for (auto &t: t_vec) {
//            t.header.stamp = clock_.now();
//            tf_broadcaster_->sendTransform(t);
//        }
//
//    }


    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::Clock clock_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::TransformStamped> t_vec;
};

class GetParams : public rclcpp::Node {
public:
    GetParams()
            : Node("get_params") {

        this->declare_parameter("yaml_file_name",
                                "aptags_location.yaml"); // specifies the name of the yaml file
    }

    std::string getYamlFileName() {
        return this->get_parameter("yaml_file_name").as_string();
    }
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    auto node_params = std::make_shared<GetParams>();

    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("yaml_tf_broadcaster");
    auto file_path = pkg_dir / "config" / node_params->getYamlFileName();

    // Load YAML file
    YAML::Node yaml_data = YAML::LoadFile(file_path);

    std::vector<std::tuple<Eigen::Matrix4d, std::string, std::string>> matrix_vec;
    std::vector<std::tuple<Eigen::VectorXd, std::string, std::string>> vector_vec;
    // Extract transformations from YAML data
    YAML::Node transformations = yaml_data["transformations"];

    for (const auto &transformation: transformations) {
        if (transformation["matrix"]) {
            // Extract ID and matrix
            const std::string aptag_id = transformation["id"].as<std::string>();
            const std::string frame_id = transformation["frame_id"].as<std::string>();

            Eigen::Matrix4d matrix;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    matrix(i, j) = transformation["matrix"][i][j].as<double>();
                }
            }
            matrix_vec.push_back({matrix, frame_id, aptag_id});
        } else {
            // Extract ID and matrix
            const std::string aptag_id = transformation["id"].as<std::string>();
            const std::string frame_id = transformation["frame_id"].as<std::string>();

            Eigen::VectorXd vector(7);
            for (int i = 0; i < 7; ++i) {
                vector(i) = transformation["transform"][i].as<double>();
            }
            vector_vec.push_back({vector, frame_id, aptag_id});

        }
    }

    auto node = std::make_shared<AptagFramePublisher>(matrix_vec, vector_vec);
    exe.add_node(node);

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
