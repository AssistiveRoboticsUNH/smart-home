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

#include <shr_plan_parameters.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "shr_plan/utils.hpp"


namespace planning_controller {

    struct World {
        int pills_motion_sensor = -1;
        int door_motion_sensor = -1;
        int door_sensor = -1;
        std::string person_location;
    };

    bool fully_updated(const World& world) {
        auto tmp = World();
        return world.pills_motion_sensor != tmp.pills_motion_sensor &&
               world.door_motion_sensor != tmp.door_motion_sensor &&
               world.door_sensor != tmp.door_sensor &&
               world.person_location != tmp.person_location;
    }

    class PlanningController : public rclcpp::Node {
    public:
        PlanningController()
                : rclcpp::Node("planing_controller"), state_(IDLE) {

            param_listener_ = std::make_shared<shr_plan_parameters::ParamListener>(get_node_parameters_interface());
            params_ = param_listener_->get_params();

            using std::placeholders::_1;
            pill_motion_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                    params_.senor_pills_motion_topic, 10,
                    std::bind(&PlanningController::pill_motion_callback, this, _1));
            door_motion_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                    params_.sensors_door_motion_topic, 10,
                    std::bind(&PlanningController::door_motion_callback, this, _1));
            door_open_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                    params_.sensors_door_open_topic, 10, std::bind(&PlanningController::door_open_callback, this, _1));

            protocol_sub_ = this->create_subscription<std_msgs::msg::String>(
                    params_.update_protocol_topic, 10,
                    std::bind(&PlanningController::update_protocol_callback, this, _1));

            navigation_action_client_ =
                    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                            this,
                            "navigate_to_pose");

            tf_buffer_ =
                    std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
                    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // assume position to initialize
            person_last_location_ = params_.locations[0];

        }

        void init() {
            domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
            planner_client_ = std::make_shared<plansys2::PlannerClient>();
            problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
            executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        }

        void step() {
            switch (state_) {
                case IDLE: {
                    if (!protocol_.empty()) {
                        world_ = World();
                        state_ = KNOWLEDGE_GATHERING;
                    }
                    break;
                }
                case KNOWLEDGE_GATHERING: {
                    if (fully_updated(world_)) {
                        state_ = PLANNING;
                    }
                    if (world_.person_location.empty() && !navigating_){
                        auto result_callback = [this](auto) {navigating_ = false;};
                        location_ind_ = (location_ind_ + 1) % params_.locations.size();
                        shr_plan::send_nav_request(*tf_buffer_, params_.locations[location_ind_], now(), navigation_action_client_,
                                                   std::nullopt, result_callback);
                        navigating_ = true;
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
                    if (world_changed_) {
                        executor_client_->cancel_plan_execution();
                    } else {
                        auto feedback = executor_client_->getFeedBack();

                        for (const auto &action_feedback: feedback.action_execution_status) {
                            std::cout << "[" << action_feedback.action << " " <<
                                      action_feedback.completion * 100.0 << "%]";
                        }
                        std::cout << std::endl;
                    }

                    if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                        if (executor_client_->getResult().value().success) {
                            std::cout << "Successful finished " << std::endl;
                            state_ = IDLE;
                            protocol_ = "";
                        } else {
                            state_ = PLANNING;
//                            for (const auto &action_feedback: feedback.action_execution_status) {
//                                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
//                                    std::cout << "[" << action_feedback.action << "] finished with error: " <<
//                                              action_feedback.message_status << std::endl;
//                                }
//                                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::CANCELLED) {
//                                    std::cout << "[" << action_feedback.action << "] finished with cancel: " <<
//                                              action_feedback.message_status << std::endl;
//                                }
//                            }
                        }

                    }

                    break;
                }

            }
        }

    private:
        void pill_motion_callback(const std_msgs::msg::Bool::SharedPtr msg) {
            if (world_.pills_motion_sensor != msg->data) {
                world_changed_ = true;
            }
            world_.pills_motion_sensor = msg->data;
        }

        void door_motion_callback(const std_msgs::msg::Bool::SharedPtr msg) {
            if (world_.door_motion_sensor != msg->data) {
                world_changed_ = true;
            }
            world_.door_motion_sensor = msg->data;
        }

        void door_open_callback(const std_msgs::msg::Bool::SharedPtr msg) {
            if (world_.door_sensor != msg->data) {
                world_changed_ = true;
            }
            world_.door_sensor = msg->data;
        }

        void update_protocol_callback(const std_msgs::msg::String::SharedPtr msg) {
            protocol_ = msg->data;
        }

        void init_knowledge() {
            for (const auto &instance: problem_expert_->getInstances()) {
                problem_expert_->removeInstance(instance);
            }

            if (protocol_ == "midnight_warning") {
                problem_expert_->addInstance(plansys2::Instance{"door", "landmark"});
                problem_expert_->addInstance(plansys2::Instance{"home", "landmark"});
                problem_expert_->addInstance(plansys2::Instance{"pioneer", "robot"});
                problem_expert_->addInstance(plansys2::Instance{"midnight_warning", "message"});
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

            if (protocol_ == "midnight_warning") {
                problem_expert_->addPredicate(plansys2::Predicate("(robot_at pioneer home)"));
                problem_expert_->addPredicate(plansys2::Predicate("(message_at midnight_warning door)"));
            }
//            else if (protocol_ == "take_pills") {
//                problem_expert_->addPredicate(plansys2::Predicate("(robot_at pioneer home)"));
//                problem_expert_->addPredicate(plansys2::Predicate("(message_at midnight_warning door)"));
//            }

        }

        void set_goal() {
            if (protocol_ == "midnight_warning") {
                problem_expert_->setGoal(plansys2::Goal("(and(robot_at pioneer door)(notified midnight_warning))"));
            }
//            else if (protocol_ == "take_pills") {
//                problem_expert_->setGoal(plansys2::Goal("(and(robot_at pioneer door)(notified midnight_warning))"));
//            }

        }

        typedef enum {
            IDLE, KNOWLEDGE_GATHERING, PLANNING, EXECUTING
        } StateType;
        StateType state_;

        std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
        std::shared_ptr<plansys2::PlannerClient> planner_client_;
        std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
        std::shared_ptr<plansys2::ExecutorClient> executor_client_;

        std::shared_ptr<shr_plan_parameters::ParamListener> param_listener_;
        shr_plan_parameters::Params params_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pill_motion_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_motion_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr door_open_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr protocol_sub_;

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        World world_;
        bool world_changed_ = false;
        std::string protocol_;
        std::string person_last_location_;
        int location_ind_ = -1;
        bool navigating_;
    };

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<planning_controller::PlanningController>();

    node->init();

    rclcpp::Rate rate(5);
    while (rclcpp::ok()) {
        node->step();

        rate.sleep();
        rclcpp::spin_some(node->get_node_base_interface());
    }

    rclcpp::shutdown();

    return 0;
}