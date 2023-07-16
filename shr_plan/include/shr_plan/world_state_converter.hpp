#include "rclcpp/rclcpp.hpp"
#include "shr_msgs/msg/world_state.hpp"
#include <memory>
#include <shr_parameters.hpp>

#pragma once

class WorldStateListener : public rclcpp::Node {
private:
    rclcpp::Subscription<shr_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    std::shared_ptr<shr_msgs::msg::WorldState> world_state_;
    std::mutex mtx;
    bool terminate_node_;
    std::shared_ptr<shr_parameters::ParamListener> param_listener_;
    std::unordered_map<std::string, Eigen::MatrixXd> mesh_vert_map_;
public:

    WorldStateListener(const std::string &node_name, std::shared_ptr<shr_parameters::ParamListener> param_listener)
            : rclcpp::Node(
            node_name) {
        terminate_node_ = false;
        world_state_ = nullptr;
        param_listener_ = param_listener;
        auto params = param_listener->get_params();
        world_state_sub_ = create_subscription<shr_msgs::msg::WorldState>(
                params.world_state_topic, 10, [this](const shr_msgs::msg::WorldState::SharedPtr msg) {
                    set_world_state_msg(msg);
                });

        std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_resources");
        auto mesh_file = (pkg_dir / "resources" / "room_areas.obj").string();
        auto [mesh_verts, mesh_names] = shr_utils::load_meshes(mesh_file);
        for (int i = 0; i < mesh_names.size(); i++) {
            auto name = mesh_names[i];
            auto verts = mesh_verts[i];
            mesh_vert_map_[name] = verts;
        }
    }

    bool check_robot_at_loc(const std::string& loc){
        if (mesh_vert_map_.find(loc) == mesh_vert_map_.end()){
            return false;
        }
        auto verts = mesh_vert_map_.at(loc);
        Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());
        Eigen::Vector3d point = {world_state_->robot_location.x, world_state_->robot_location.y, world_state_->robot_location.z};
        return shr_utils::PointInMesh(point, verts, verts2d);
    }

    bool check_person_at_loc(const std::string& loc){
        if (mesh_vert_map_.find(loc) == mesh_vert_map_.end()){
            return false;
        }
        auto verts = mesh_vert_map_.at(loc);
        Eigen::MatrixXd verts2d = verts.block(0, 0, 2, verts.cols());
        Eigen::Vector3d point = {world_state_->patient_location.x, world_state_->patient_location.y, world_state_->patient_location.z};
        return shr_utils::PointInMesh(point, verts, verts2d);
    }


    shr_parameters::Params get_params() {
        std::lock_guard<std::mutex> lock(mtx);
        return param_listener_->get_params();
    }

    void terminate_node() {
        std::lock_guard<std::mutex> lock(mtx);
        terminate_node_ = true;
    }

    bool should_terminate_node() {
        std::lock_guard<std::mutex> lock(mtx);
        return terminate_node_;
    }

    void set_world_state_msg(const std::shared_ptr<shr_msgs::msg::WorldState> &msg) {
        std::lock_guard<std::mutex> lock(mtx);
        world_state_ = msg;
    }

    std::shared_ptr<shr_msgs::msg::WorldState> get_world_state_msg() {
        std::lock_guard<std::mutex> lock(mtx);
        return world_state_;
    }
};
