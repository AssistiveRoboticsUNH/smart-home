// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <memory>
#include <string>
#include <map>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shr_msgs/action/play_video_request.hpp"

#include <shr_plan_parameters.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class NotifyVideo : public plansys2::ActionExecutorClient {
public:
  NotifyVideo(const std::string &action, const std::string &file_name, int wait_time,
              const std::string &sensor_topic)
      : plansys2::ActionExecutorClient(action, 500ms) {

    set_parameter(rclcpp::Parameter("action_name", action));
    file_name_ = file_name;
    wait_time_ = wait_time;
    sensor_topic_ = sensor_topic;
    sensor_sub_ = create_subscription<std_msgs::msg::Bool>(sensor_topic, 10,
                                                                    std::bind(&NotifyVideo::callback, this,
                                                                              std::placeholders::_1));

  }

  void callback(const std_msgs::msg::Bool::SharedPtr msg) {
    sensor_ = msg->data;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) {
    send_feedback(0.0, "Begin video");

    navigation_action_client_ =
        rclcpp_action::create_client<shr_msgs::action::PlayVideoRequest>(shared_from_this(), "play_video");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for /play_video action server...");

      is_action_server_ready =
          navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "/play_video action server ready");

    auto send_goal_options = rclcpp_action::Client<shr_msgs::action::PlayVideoRequest>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) {
      waiting_for_response_ = false;
      start_time_ = now();
    };

    start_time_ = now();
    waiting_for_response_ = true;
    sensor_ = false;
    navigation_goal_.file_name = file_name_;
    future_navigation_goal_handle_ = navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:

  void do_work() {
    rclcpp::Time cur_time;
    cur_time = now();
    auto time_diff = cur_time - start_time_;
    send_feedback(time_diff.seconds() / wait_time_, "waiting for response");

    if (sensor_) {
        if (sensor_topic_ == "/smartthings_sensors_motion_pills"){
            finish(true, 1.0, "Person took medicine");
        }
        else if (sensor_topic_ == "/smartthings_sensors_motion_food"){
            finish(true, 1.0, "Person ate food");
        }
    }

    if (time_diff.seconds() > wait_time_ && !waiting_for_response_) {
      finish(false, 1.0, "Person failed to respond to prompt");
    }

  }

  using AudioGoalHandle =
      rclcpp_action::ClientGoalHandle<shr_msgs::action::PlayVideoRequest>;

  rclcpp_action::Client<shr_msgs::action::PlayVideoRequest>::SharedPtr navigation_action_client_;
  std::shared_future<AudioGoalHandle::SharedPtr> future_navigation_goal_handle_;
  AudioGoalHandle::SharedPtr navigation_goal_handle_;
  shr_msgs::action::PlayVideoRequest::Goal navigation_goal_;


  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> sensor_sub_;
  std::string file_name_;
  std::string sensor_topic_;
  int wait_time_;
  rclcpp::Time start_time_;
  bool waiting_for_response_;
  bool sensor_;

};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  auto parameter_node = std::make_shared<rclcpp::Node>("notify_recorded_parameter_node");
  auto param_listener = std::make_shared<shr_plan_parameters::ParamListener>(parameter_node);
  auto params = param_listener->get_params();

  std::vector<std::shared_ptr<NotifyVideo>> all_nodes;
  for (auto i = 0ul; i < params.notify_recorded_actions.actions.size(); i++) {
    auto action = params.notify_recorded_actions.actions[i];
    auto file_name = params.notify_recorded_actions.file_names[i];
    auto wait_time = params.notify_recorded_actions.wait_times[i];
    auto sensor_topic = params.notify_recorded_actions.topics[i];
    auto ind = all_nodes.size();
    all_nodes.push_back(std::make_shared<NotifyVideo>(action, file_name, wait_time, sensor_topic));
    all_nodes[ind]->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    exe.add_node(all_nodes[ind]->get_node_base_interface());
  }

  exe.spin();
  rclcpp::shutdown();

  return 0;
}