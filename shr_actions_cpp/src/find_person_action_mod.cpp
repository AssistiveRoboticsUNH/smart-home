#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "shr_msgs/action/find_person_request.hpp"
#include "shr_msgs/action/rotate_request.hpp"
#include "shr_msgs/action/recognize_request.hpp"
//#include "nav2_msgs/action/navigate_to_pose.hpp" // when using the navigation stack
#include "shr_msgs/action/navigate_to_goal.hpp" // when using the ompl

#include "shr_utils/utils.hpp"


namespace find_person_request {
    using namespace std::placeholders;

    class FindPersonRequestActionServer : public rclcpp::Node {
    public:
        using FindPersonRequest = shr_msgs::action::FindPersonRequest;
        using GoalHandleFindPersonRequest = rclcpp_action::ServerGoalHandle<FindPersonRequest>;


        explicit FindPersonRequestActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : Node("find_person_request_action_server", options) {

            tf_buffer_ =
                    std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ =
                    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            this->action_server_ = rclcpp_action::create_server<FindPersonRequest>(
                    this,
                    "find_person",
                    std::bind(&FindPersonRequestActionServer::handle_goal, this, _1, _2),
                    std::bind(&FindPersonRequestActionServer::handle_cancel, this, _1),
                    std::bind(&FindPersonRequestActionServer::handle_accepted, this, _1));

            // navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
             //       this, "navigate_to_pose");  // when using navigation stack

            navigation_action_client_ = rclcpp_action::create_client<shr_msgs::action::NavigateToGoal>(
                           this, "navigate_to_goal");  // when using ompl package
            rotate_client_ = rclcpp_action::create_client<shr_msgs::action::RotateRequest>(
                    this, "rotate");
            recognize_face_client_ = rclcpp_action::create_client<shr_msgs::action::RecognizeRequest>(
                    this, "recognize_face");
            if (!rotate_client_->wait_for_action_server()) {
                RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            }

            if (!recognize_face_client_->wait_for_action_server()) {
                RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            }

            if (!navigation_action_client_->wait_for_action_server()) {
                RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
            }
        }

    private:
        rclcpp_action::Server<FindPersonRequest>::SharedPtr action_server_;
        //rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;  // when using navigation stack
        rclcpp_action::Client<shr_msgs::action::NavigateToGoal>::SharedPtr navigation_action_client_;  // when using ompl stack
        rclcpp_action::Client<shr_msgs::action::RotateRequest>::SharedPtr rotate_client_;
        rclcpp_action::Client<shr_msgs::action::RecognizeRequest>::SharedPtr recognize_face_client_;

//        std::shared_future<rclcpp_action::ClientGoalHandle<shr_msgs::action::RecognizeRequest>::SharedPtr> recognize_face_goal_;
//        rclcpp_action::ClientGoalHandle<shr_msgs::action::RotateRequest>::SharedPtr rotate_goal_;
//        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const FindPersonRequest::Goal> goal) {
            (void) uuid;
            std::stringstream ss;
            for (const auto &loc: goal->locations) {
                ss << loc << " ";
            }
            RCLCPP_INFO(this->get_logger(), "Received find person request at locations request with order %s",
                        ss.str().c_str());
//            if (rotating_ || recognizing_ || navigating_) {
//                RCLCPP_INFO(this->get_logger(), "Goal rejected because action already in progress");
//                return rclcpp_action::GoalResponse::REJECT;
//            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        void cancel_all_goals() {
            navigation_action_client_->async_cancel_all_goals();
            rotate_client_->async_cancel_all_goals();
            recognize_face_client_->async_cancel_all_goals();

        }

        rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleFindPersonRequest> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void) goal_handle;

            cancel_all_goals();

//            cancelled_ = true;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleFindPersonRequest> goal_handle) {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
//            cancelled_ = false;
            std::thread{std::bind(&FindPersonRequestActionServer::execute, this, _1), goal_handle}.detach();
        }


        void execute(const std::shared_ptr<GoalHandleFindPersonRequest> goal_handle) {

            RCLCPP_INFO(this->get_logger(), "Executing goal");

            auto result = std::make_shared<FindPersonRequest::Result>();
            auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<FindPersonRequest::Feedback>();

            bool found_person = false;
            bool moving = false;
//            bool rotating = false;
            bool recognizing = false;
            bool cancelled = false;
            int i = 0;

            int location_ind = shr_utils::get_nearest_location(*tf_buffer_, goal->locations);
            rclcpp::Rate loop_rate(10);
            while (!found_person) {
                if (goal_handle->is_canceling()) {
                    cancelled = true;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "find person goal canceled");
                    return;
                }
                if (!moving && !found_person && (i % 2) == 0) {
                    rotate_360(goal, &moving);
                    i++;
                } else if (!moving && !found_person) {
                    navigate(goal, &moving, &location_ind);
                    i++;
                }

                if (!recognizing && !found_person) {
                    bool rotating = moving && (i % 2) == 1;
                    recognize_patient(goal, &rotating, &recognizing, &found_person);
                }
                loop_rate.sleep();
            }

            cancel_all_goals();

            result->location = goal->locations[location_ind];
            result->ind = location_ind;
            goal_handle->succeed(result);

        }

        void rotate_360(const std::shared_ptr<const FindPersonRequest::Goal> &goal, bool *moving) {
            auto goal_msg = shr_msgs::action::RotateRequest::Goal();
            goal_msg.angle = 2 * M_PI;
            goal_msg.total_time = 10.0;
            auto send_goal_options = rclcpp_action::Client<shr_msgs::action::RotateRequest>::SendGoalOptions();
            auto result_callback = [this, goal, moving](auto) {
                *moving = false;
            };
            send_goal_options.result_callback = result_callback;
            send_goal_options.goal_response_callback = [this, moving](auto future) {
                auto goal_handle = future.get();
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "rotate goal was rejected by server");
                    *moving = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "rotate accepted by server, waiting for result");
                    *moving = true;
                }
            };

            rotate_client_->async_send_goal(goal_msg, send_goal_options);

            *moving = true;
        }

        void navigate(const std::shared_ptr<const FindPersonRequest::Goal> &goal, bool *moving, int* location_ind) {
            auto result_callback = [this, moving](auto) {
                *moving = false;
            };
            auto goal_response_callback = [this, moving](auto future) {
                auto goal_handle = future.get();
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "navigating goal was rejected by server");
                    *moving = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "navigating goal accepted by server, waiting for result");
                    *moving = true;
                }
            };
            *location_ind = (*location_ind + 1) % goal->locations.size();
            shr_utils::send_nav_request_custom(*tf_buffer_, goal->locations[*location_ind], now(),
                                                           navigation_action_client_,
                                                           goal_response_callback, std::nullopt, result_callback);
            *moving = true;
        }


        void
        recognize_patient(const std::shared_ptr<const FindPersonRequest::Goal> &goal, bool *rotating, bool *recognizing,
                          bool *found_person) {
            auto goal_msg = shr_msgs::action::RecognizeRequest::Goal();
            auto send_goal_options = rclcpp_action::Client<shr_msgs::action::RecognizeRequest>::SendGoalOptions();
            auto result_callback = [this, goal, rotating, recognizing, found_person](
                    const rclcpp_action::ClientGoalHandle<shr_msgs::action::RecognizeRequest>::WrappedResult &response) {
              if (*rotating) {
                    for (const auto &name: response.result->names) {
                        if (name == goal->name) {
                            *found_person = true;
                        }
                    }
                }
                *recognizing = false;
            };
            send_goal_options.result_callback = result_callback;

            send_goal_options.goal_response_callback = [this, recognizing](auto future) {
                auto goal_handle = future.get();
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "recognize goal was rejected by server");
                    *recognizing = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "recognize goal accepted by server, waiting for result");
                    *recognizing = true;
                }
            };
            recognize_face_client_->async_send_goal(goal_msg, send_goal_options);
            *recognizing = true;
        }
    };  // class FindPersonRequestActionServer

}  // namespace action_tutorials_cpp



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto action_server = std::make_shared<find_person_request::FindPersonRequestActionServer>();

    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}
