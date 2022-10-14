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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


#include "std_msgs/msg/bool.hpp"
#include "shr_msgs/action/gather_information_request.hpp"
#include "shr_msgs/msg/midnight_warning_protocol.hpp"
#include "shr_msgs/msg/medicine_reminder_protocol.hpp"
#include "shr_msgs/msg/world_state.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"

#include <rclcpp_action/client.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <shr_plan_parameters.hpp>


namespace planning_controller {

    typedef enum {
        IDLE, GATHERING_INFO, PLANNING, EXECUTING
    } StateType;

    class PlanningControllerSpin : public rclcpp::Node {
    public:
        PlanningControllerSpin()
                : rclcpp::Node("planing_controller_spin") {
            using std::placeholders::_1;
            param_listener_ = std::make_shared<shr_plan_parameters::ParamListener>(get_node_parameters_interface());
            params_ = param_listener_->get_params();

            world_state_sub_ = this->create_subscription<shr_msgs::msg::WorldState>(
                    params_.world_state_topic, 10,
                    std::bind(&PlanningControllerSpin::update_world_state_callback, this, _1));

            action_hub_sub_ = create_subscription<plansys2_msgs::msg::ActionExecution>(
                    "actions_hub", rclcpp::QoS(100).reliable(),
                    std::bind(&PlanningControllerSpin::action_hub_callback, this, _1));
        }

        shr_msgs::msg::WorldState get_world_state() {
            std::lock_guard<std::mutex> lock(mutex_);
            new_world_ = false;
            return world_state_;
        }

        bool has_new_world_state() {
            return new_world_;
        }


    private:

        void update_world_state_callback(const shr_msgs::msg::WorldState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (world_state_ != *msg){
                world_state_ = *msg;
                new_world_ = true;
            }
        }


        void action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
            if (msg->type == plansys2_msgs::msg::ActionExecution::FINISH) {
                // action finished
                std::cout << "Action: " << msg->action << " has completed!!!!!!!!!!" << std::endl;
            }
        }

        shr_msgs::msg::WorldState world_state_;
        bool new_world_;
        rclcpp::Subscription<shr_msgs::msg::WorldState>::SharedPtr world_state_sub_;
        rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_hub_sub_;
        std::shared_ptr<shr_plan_parameters::ParamListener> param_listener_;
        shr_plan_parameters::Params params_;
        std::mutex mutable mutex_;

    };

    class PlanningController : public rclcpp::Node {
    public:
        PlanningController()
                : rclcpp::Node("planing_controller"), state_(IDLE) {

            param_listener_ = std::make_shared<shr_plan_parameters::ParamListener>(get_node_parameters_interface());
            params_ = param_listener_->get_params();

            using std::placeholders::_1;

//            world_state_sub_ = this->create_subscription<shr_msgs::msg::WorldState>(
//                    params_.world_state_topic, 10,
//                    std::bind(&PlanningController::update_world_state_callback, spin_node, _1));
//
////            navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
////                    spin_node, "navigate_to_pose");
            gathering_info_client_ = rclcpp_action::create_client<shr_msgs::action::GatherInformationRequest>(
                    this, "gather_information");
//
//
//            action_hub_sub_ = create_subscription<plansys2_msgs::msg::ActionExecution>(
//                    "actions_hub", rclcpp::QoS(100).reliable(),
//                    std::bind(&PlanningController::action_hub_callback, spin_node, _1));

//            tf_buffer_ =
//                    std::make_unique<tf2_ros::Buffer>(this->get_clock());
//            tf_listener_ =
//                    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//            world_state_ = std::make_shared<shr_msgs::msg::WorldState>();



        }

        void set_world_state(shr_msgs::msg::WorldState world_state) {
            world_state_ = world_state;
        }

        void init() {
            domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
            planner_client_ = std::make_shared<plansys2::PlannerClient>();
            problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
            executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        }


        StateType get_transition() {
            if (world_state_.too_late_to_leave == 1 &&
                world_state_.patient_location == world_state_.door_location) {
                active_protocol = "midnight_warning";
                return PLANNING;
            } else if (world_state_.too_late_to_leave == 1 &&
                       world_state_.patient_location.empty() && world_state_.door_open) {
                active_protocol = "midnight_warning";
                requested_states = {"patient_location"};
                return GATHERING_INFO;
            }

            if (world_state_.took_medicine == 0 && world_state_.time_to_take_medicine == 1) {
                active_protocol = "medicine_reminder";
                if (world_state_.patient_location.empty()) {
                    requested_states = {"patient_location"};
                    return GATHERING_INFO;
                } else {
                    return PLANNING;
                }
            }

            return IDLE;
        }

        void step() {
            switch (state_) {
                case IDLE: {
                    state_ = get_transition();
                    break;
                }
                case GATHERING_INFO: {
                    state_ = get_transition();
                    if (state_ != GATHERING_INFO){
                        gathering_info_client_->async_cancel_all_goals();
                    }

                    if (!gathering_info) {
                        auto result_callback = [this](
                                const rclcpp_action::ClientGoalHandle<shr_msgs::action::GatherInformationRequest>::WrappedResult &res) {
                            gathering_info = false;
                            if (res.code == rclcpp_action::ResultCode::SUCCEEDED){
                                world_state_ = res.result->world_state;
                            }
                        };
                        auto send_goal_options = rclcpp_action::Client<shr_msgs::action::GatherInformationRequest>::SendGoalOptions();
                        send_goal_options.result_callback = result_callback;
                        auto request = shr_msgs::action::GatherInformationRequest::Goal();
                        request.states = requested_states;
                        gathering_info_client_->async_send_goal(request, send_goal_options);
                        gathering_info = true;
                    }
                    break;
                }
                case PLANNING: {

                    init_knowledge();
                    init_predicates();
                    set_goal();
                    world_changed_ = false;

                    // Compute the plan
                    auto domain = domain_expert_->getDomain();
                    auto problem = problem_expert_->getProblem();
                    auto plan = planner_client_->getPlan(domain, problem);

                    if (!plan.has_value()) {
                        std::cout << "Could not find plan to reach goal " <<
                                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                        break;
                    }

                    // Execute the plan
                    if (executor_client_->start_plan_execution(plan.value())) {
                        state_ = EXECUTING;
                    }

                    break;
                }
                case EXECUTING: {

                    auto feedback = executor_client_->getFeedBack();

                    for (const auto &action_feedback: feedback.action_execution_status) {
                        std::cout << "[" << action_feedback.action << " " <<
                                  action_feedback.completion * 100.0 << "%]";
                    }
                    std::cout << std::endl;

                    if (world_changed_) {
                        executor_client_->cancel_plan_execution();
                    }

                    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                        if (executor_client_->getResult().value().success) {
                            std::cout << "Successful finished " << std::endl;
                            state_ = IDLE;
                            active_protocol = "";
                        } else {
                            state_ = PLANNING;
                        }
                    }

                    break;
                }

            }
        }

        bool world_changed_ = false;

    private:

//        void update_protocol_callback(const std_msgs::msg::String::SharedPtr msg) {
//            active_protocol = msg->data;
//        }




        void init_knowledge() {
            for (const auto &instance: problem_expert_->getInstances()) {
                problem_expert_->removeInstance(instance);
            }

            if (active_protocol == "midnight_warning") {
                problem_expert_->addInstance(plansys2::Instance{world_state_.door_location, "landmark"});
                problem_expert_->addInstance(plansys2::Instance{"home", "landmark"});
                problem_expert_->addInstance(plansys2::Instance{"pioneer", "robot"});
                problem_expert_->addInstance(plansys2::Instance{params_.patient_name, "person"});
                problem_expert_->addInstance(plansys2::Instance{"midnight_warning", "automated_message"});
                problem_expert_->addInstance(plansys2::Instance{"midnight_warning_video", "recorded_message"});
            }
//            else if (protocol_ == "take_pills") {
//                problem_expert_->addInstance(plansys2::Instance{"door", "landmark"});
//                problem_expert_->addInstance(plansys2::Instance{"home", "landmark"});
//                problem_expert_->addInstance(plansys2::Instance{"bedroom", "landmark"});
//                problem_expert_->addInstance(plansys2::Instance{"kitchen", "landmark"});
//
//                problem_expert_->addInstance(plansys2::Instance{"medicine_robot_msg", "message"});
//                problem_expert_->addInstance(plansys2::Instance{"medicine_phone_msg", "phonemessage"});
//
//                problem_expert_->addInstance(plansys2::Instance{"mediciness", "sensor"});
//
////                door kitchen bedroom home - landmark
////                medicine_robot_msg - message
////                medicine_phone_msg - phonemessage
////                mediciness - sensor
//            }

        }

        void init_predicates() {
            for (const auto &pred: problem_expert_->getPredicates()) {
                problem_expert_->removePredicate(pred);
            }

            if (active_protocol == "midnight_warning") {
                problem_expert_->addPredicate(plansys2::Predicate("(robot_at pioneer home)"));
                problem_expert_->addPredicate(plansys2::Predicate(
                        "(person_at " + params_.patient_name + " " + world_state_.door_location + ")"));
                problem_expert_->addPredicate(
                        plansys2::Predicate("(give_message_location " + world_state_.door_location + ")"));
                if (world_state_.door_open == 1) {
                    problem_expert_->addPredicate(plansys2::Predicate("(automated_message_given midnight_warning)"));
                }
            }
//            else if (protocol_ == "take_pills") {
//                problem_expert_->addPredicate(plansys2::Predicate("(robot_at pioneer home)"));
//                problem_expert_->addPredicate(plansys2::Predicate("(message_at midnight_warning door)"));
//            }

        }

        void set_goal() {
            if (active_protocol == "midnight_warning") {
                if (world_state_.door_open == 1) {
                    problem_expert_->setGoal(plansys2::Goal(
                            "(and(robot_at pioneer " + world_state_.door_location +
                            ")(recorded_message_given midnight_warning_video))"));
                } else {
                    problem_expert_->setGoal(
                            plansys2::Goal("(and(robot_at pioneer " + world_state_.door_location +
                                           ")(automated_message_given midnight_warning))"));
                }
            }
//            else if (protocol_ == "take_pills") {
//                problem_expert_->setGoal(plansys2::Goal("(and(robot_at pioneer door)(notified midnight_warning))"));
//            }

        }


        StateType state_;

        std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
        std::shared_ptr<plansys2::PlannerClient> planner_client_;
        std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
        std::shared_ptr<plansys2::ExecutorClient> executor_client_;

        std::shared_ptr<shr_plan_parameters::ParamListener> param_listener_;
        shr_plan_parameters::Params params_;


        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
        rclcpp_action::Client<shr_msgs::action::GatherInformationRequest>::SharedPtr gathering_info_client_;

//        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
//        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

//        std::shared_ptr<PlanningControllerSpin> spin_node;

        bool gathering_info = false;
        shr_msgs::msg::WorldState world_state_;
        std::string active_protocol;
        std::vector<std::string> requested_states;
    };

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planning_controller::PlanningController>();
    auto spin_node = std::make_shared<planning_controller::PlanningControllerSpin>();
    node->init();

    rclcpp::Rate rate(5);
    while (spin_node->get_world_state() == shr_msgs::msg::WorldState()) {
        std::cout << "Waiting for world update" << std::endl;
        rate.sleep();
        rclcpp::spin_some(spin_node->get_node_base_interface());
    }

    std::thread thread(
            [spin_node](){
                rclcpp::spin(spin_node->get_node_base_interface());
            }
    );

    while (rclcpp::ok()) {
        if (spin_node->has_new_world_state()){
            node->set_world_state(spin_node->get_world_state());
        }
        node->step();
        rclcpp::spin_some(node->get_node_base_interface());
        rate.sleep();
    }

    thread.join();

    rclcpp::shutdown();

    return 0;
}