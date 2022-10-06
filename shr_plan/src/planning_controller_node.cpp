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

class PatrollingController : public rclcpp::Node
{
public:
    PatrollingController()
            : rclcpp::Node("plaining_controller"), state_(STARTING)
    {
    }

    void init()
    {
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        executor_client_ = std::make_shared<plansys2::ExecutorClient>();
        init_knowledge();
    }

    void init_knowledge()
    {
        problem_expert_->addInstance(plansys2::Instance{"door", "landmark"});
        problem_expert_->addInstance(plansys2::Instance{"home", "landmark"});
        problem_expert_->addInstance(plansys2::Instance{"pioneer", "robot"});
        problem_expert_->addInstance(plansys2::Instance{"midnight_warning", "message"});
    }

    void init_predicates()
    {
//        for (const auto& pred : problem_expert_->getPredicates()){
//            problem_expert_->removePredicate(pred);
//        }
        problem_expert_->addPredicate(plansys2::Predicate("(robot_at pioneer home)"));
        problem_expert_->addPredicate(plansys2::Predicate("(message_at midnight_warning door)"));

    }

    void step()
    {
        switch (state_) {
            case STARTING:
            {
                init_predicates();

                // Set the goal for next state
                problem_expert_->setGoal(plansys2::Goal("(and(robot_at pioneer door)(notified midnight_warning))"));

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
            case EXECUTING:
            {
                auto feedback = executor_client_->getFeedBack();

                for (const auto & action_feedback : feedback.action_execution_status) {
                    std::cout << "[" << action_feedback.action << " " <<
                              action_feedback.completion * 100.0 << "%]";
                }
                std::cout << std::endl;

                if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
                    if (executor_client_->getResult().value().success) {
                        std::cout << "Successful finished " << std::endl;
                    } else {
                        for (const auto & action_feedback : feedback.action_execution_status) {
                            if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                                          action_feedback.message_status << std::endl;
                            }
                        }
                    }
                    state_ = STARTING;
                }

                break;
            }
        }
    }

private:
    typedef enum {STARTING, EXECUTING} StateType;
    StateType state_;

    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrollingController>();

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