#include "rclcpp/rclcpp.hpp"
#include "shr_msgs/msg/world_state.hpp"
#include <memory>
#include "tf2_ros/buffer.h"
#include <shr_parameters.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_listener.h>

#pragma once

class WorldStateListener : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr eating_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr charging_sub_;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr time_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr taking_medicine_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr good_weather_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<shr_msgs::msg::WorldState> world_state_;
    std::mutex tf_buffer_mtx;
    std::mutex terminate_mtx;
    std::mutex world_state_mtx;
    bool terminate_node_;
    std::shared_ptr<shr_parameters::ParamListener> param_listener_;
    std::unordered_map<std::string, Eigen::MatrixXd> mesh_vert_map_robot;
    std::unordered_map<std::string, Eigen::MatrixXd> mesh_vert_map_person;

public:

    WorldStateListener(const std::string &node_name, std::shared_ptr<shr_parameters::ParamListener> param_listener)
            : rclcpp::Node(
            node_name) {
        terminate_node_ = false;
        world_state_ = std::make_shared<shr_msgs::msg::WorldState>();
        param_listener_ = param_listener;
        auto params = param_listener->get_params();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, true);

        eating_sub_ = create_subscription<std_msgs::msg::Int32>(
                params.topics.person_eating, 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(world_state_mtx);
                    world_state_->person_eating = msg->data;
                });
        taking_medicine_sub_ = create_subscription<std_msgs::msg::Int32>(
                params.topics.person_taking_medicine, 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(world_state_mtx);
                    world_state_->person_taking_medicine = msg->data;
                });
        time_sub_ = create_subscription<builtin_interfaces::msg::Time>(
                params.topics.time, 10, [this](const builtin_interfaces::msg::Time::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(world_state_mtx);
                    world_state_->time = *msg;

                    // 🔍 Debugging: Print received time
                    // RCLCPP_INFO(rclcpp::get_logger(std::string("user=") + "high_level_domain_Idle" + "started"), "⏳ Received protocol time update: sec = %d, nanosec = %d", msg->sec, msg->nanosec);
                });

        charging_sub_ = create_subscription<std_msgs::msg::Int32>(
                params.topics.robot_charging, 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(world_state_mtx);
                    world_state_->robot_charging = msg->data;
                });

        good_weather_sub_ = create_subscription<std_msgs::msg::Int32>(
                params.topics.good_weather, 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(world_state_mtx);
                    world_state_->good_weather = msg->data;
                });

        std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_resources");
        auto mesh_file_robot = (pkg_dir / "resources" / "howie_robot_.obj").string();
        auto [mesh_verts_robot, mesh_names_robot] = shr_utils::load_meshes(mesh_file_robot);
        for (int i = 0; i < mesh_names_robot.size(); i++) {
            auto name_robot = mesh_names_robot[i];
            auto verts_robot = mesh_verts_robot[i];
            mesh_vert_map_robot[name_robot] = verts_robot;
        }

        auto mesh_file_person = (pkg_dir / "resources" / "exter_blend_mesh_person.obj").string();
        auto [mesh_verts_person, mesh_names_person] = shr_utils::load_meshes(mesh_file_person);
        for (int i = 0; i < mesh_names_person.size(); i++) {
            auto name_person = mesh_names_person[i];
            auto verts_person = mesh_verts_person[i];
            mesh_vert_map_person[name_person] = verts_person;
        }
    }

    bool check_robot_at_loc(const std::string &loc) {
        if (mesh_vert_map_robot.find(loc) == mesh_vert_map_robot.end()) {
            return false;
        }
        auto verts = mesh_vert_map_robot.at(loc);
        Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());

        auto params = param_listener_->get_params();
        geometry_msgs::msg::TransformStamped robot_location;
        std::lock_guard<std::mutex> lock(tf_buffer_mtx);
        try {
//            robot_location = tf_buffer_->lookupTransform("odom", params.robot_tf, tf2::TimePointZero); //TODO fix
//            RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", "unity", params.robot_tf.c_str(), ex.what());


            robot_location = tf_buffer_->lookupTransform("unity", params.robot_tf, tf2::TimePointZero, std::chrono::seconds(10)); //TODO fix

        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", "unity", params.robot_tf.c_str(), ex.what());
            return false;
        }

        Eigen::Vector3d point = {robot_location.transform.translation.x, robot_location.transform.translation.y, 0.0};
        // cause it doesnt matter sice its 2D robot_location.transform.translation.z};
        return shr_utils::PointInMesh(point, verts, verts2d);
    }

    bool check_person_at_loc(const std::string &loc) {
        if (mesh_vert_map_person.find(loc) == mesh_vert_map_person.end()) {
            return false;
        }
        auto verts = mesh_vert_map_person.at(loc);
        Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());

        auto params = param_listener_->get_params();
        geometry_msgs::msg::TransformStamped patient_location;
        std::lock_guard<std::mutex> lock(tf_buffer_mtx);
        try {
//            patient_location = tf_buffer_->lookupTransform("odom", params.person_tf, tf2::TimePointZero); //TODO fix
// changed from odom to unity because odom is not fixed
            patient_location = tf_buffer_->lookupTransform("unity", params.person_tf, tf2::TimePointZero, std::chrono::seconds(100000)); //TODO fix

        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", "unity", params.person_tf.c_str(), ex.what());
            return false;
        }

        Eigen::Vector3d point = {patient_location.transform.translation.x, patient_location.transform.translation.y, 0.0};
        // cause it doesnt matter sice its 2D robot_location.transform.translation.z};
        return shr_utils::PointInMesh(point, verts, verts2d);
    }

    std::optional<geometry_msgs::msg::TransformStamped> get_tf(const std::string &base, const std::string &frame) {
        geometry_msgs::msg::TransformStamped robot_location;
        std::lock_guard<std::mutex> lock(tf_buffer_mtx);
        try {
            robot_location = tf_buffer_->lookupTransform(base, frame, tf2::TimePointZero, std::chrono::seconds(50000)); //TODO fix
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", base.c_str(), frame.c_str(), ex.what());
            return {};
        }
        return robot_location;
    }

    shr_parameters::Params get_params() {
        std::lock_guard<std::mutex> lock(world_state_mtx);
        return param_listener_->get_params();
    }

    void terminate_node() {
        std::lock_guard<std::mutex> lock(terminate_mtx);
        terminate_node_ = true;
    }

    bool should_terminate_node() {
        std::lock_guard<std::mutex> lock(terminate_mtx);
        return terminate_node_;
    }

    std::shared_ptr<shr_msgs::msg::WorldState> get_world_state_msg() {
        std::lock_guard<std::mutex> lock(world_state_mtx);
        return world_state_;
    }
};
