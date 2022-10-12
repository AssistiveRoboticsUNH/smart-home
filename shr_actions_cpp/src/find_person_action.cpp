#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "shr_msgs/action/find_person_request.hpp"
#include "shr_msgs/action/rotate_request.hpp"
#include "shr_msgs/action/recognize_request.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "shr_utils/utils.hpp"


namespace find_person_request {
    using namespace std::placeholders;

    class FindPersonRequestActionServer : public rclcpp::Node {
    public:
        using FindPersonRequest = shr_msgs::action::FindPersonRequest;
        using GoalHandleFindPersonRequest = rclcpp_action::ServerGoalHandle<FindPersonRequest>;


        explicit FindPersonRequestActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : Node("find_person_request_action_server", options) {
            navigation_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                    this, "navigate_to_pose");
            rotate_client_ = rclcpp_action::create_client<shr_msgs::action::RotateRequest>(
                    this, "rotate");
            recognize_face_client_ = rclcpp_action::create_client<shr_msgs::action::RecognizeRequest>(
                    this, "recognize_face");

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
        }

    private:
        rclcpp_action::Server<FindPersonRequest>::SharedPtr action_server_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
        rclcpp_action::Client<shr_msgs::action::RotateRequest>::SharedPtr rotate_client_;
        rclcpp_action::Client<shr_msgs::action::RecognizeRequest>::SharedPtr recognize_face_client_;

        rclcpp_action::ClientGoalHandle<shr_msgs::action::RecognizeRequest>::SharedPtr recognize_face_goal_;
        rclcpp_action::ClientGoalHandle<shr_msgs::action::RotateRequest>::SharedPtr rotate_goal_;
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        int location_ind_ = 0;
        bool navigating_;
        bool rotating_;
        bool recognizing_;
        bool found_person_;

        rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const FindPersonRequest::Goal> goal) {
            std::stringstream ss;
            for (const auto &loc: goal->locations)
                ss << loc << " ";
            RCLCPP_INFO(this->get_logger(), "Received find person request at locations request with order %s",
                        ss.str().c_str());
            (void) uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleFindPersonRequest> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void) goal_handle;

            if (navigation_goal_) {
                navigation_action_client_->async_cancel_goal(navigation_goal_);
            }
            if (rotate_goal_) {
                rotate_client_->async_cancel_goal(rotate_goal_);
            }
            if (recognize_face_goal_) {
                recognize_face_client_->async_cancel_goal(recognize_face_goal_);
            }

            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleFindPersonRequest> goal_handle) {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&FindPersonRequestActionServer::execute, this, _1), goal_handle}.detach();
        }



        void execute(const std::shared_ptr<GoalHandleFindPersonRequest> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(1);
            auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<FindPersonRequest::Feedback>();

            found_person_ = false;
            navigating_ = false;
            rotating_ = false;
            recognizing_ = false;
            location_ind_ = shr_utils::get_nearest_location(*tf_buffer_, goal->locations);
            while (!found_person_) {
                if (!navigating_ && !rotating_) {
                    rotate_360(goal);
                }

                if (!recognizing_) {
                    recognize_patient(goal);
                }

            }

            auto result = std::make_shared<FindPersonRequest::Result>();
            result->location = goal->locations[location_ind_];
            result->ind = location_ind_;
            goal_handle->succeed(result);

        }

        void rotate_360(const std::shared_ptr<const FindPersonRequest::Goal> &goal) {
            auto goal_msg = shr_msgs::action::RotateRequest::Goal();
            goal_msg.angle = 2 * M_PI;
            goal_msg.total_time = 10.0;
            auto send_goal_options = rclcpp_action::Client<shr_msgs::action::RotateRequest>::SendGoalOptions();
            auto result_callback = [this, goal](auto) {
                if (!found_person_){
                    navigate(goal);
                }
                rotating_ = false;
            };
            send_goal_options.result_callback = result_callback;
            rotate_client_->async_send_goal(goal_msg, send_goal_options);
            rotating_ = true;
        }

        void navigate(const std::shared_ptr<const FindPersonRequest::Goal> &goal)  {
            auto result_callback = [this](auto) {
                navigating_ = false;
            };
            location_ind_ = (location_ind_ + 1) % goal->locations.size();
            shr_utils::send_nav_request(*tf_buffer_, goal->locations[location_ind_], now(),
                                        navigation_action_client_,
                                        std::nullopt, result_callback);
            navigating_ = true;
        }


        void recognize_patient(const std::shared_ptr<const FindPersonRequest::Goal> &goal) {
            auto goal_msg = shr_msgs::action::RecognizeRequest::Goal();
            auto send_goal_options = rclcpp_action::Client<shr_msgs::action::RecognizeRequest>::SendGoalOptions();
            auto result_callback = [this, goal](
                    const rclcpp_action::ClientGoalHandle<shr_msgs::action::RecognizeRequest>::WrappedResult &response) {
                if (rotating_) {
                    for (const auto &name: response.result->names) {
                        if (name == goal->name) {
                            found_person_ = true;
                        }
                    }
                }
                recognizing_ = false;
            };
            send_goal_options.result_callback = result_callback;
            recognize_face_client_->async_send_goal(goal_msg, send_goal_options);
            recognizing_ = true;
        }
    };  // class FindPersonRequestActionServer

}  // namespace action_tutorials_cpp



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto action_server = std::make_shared<find_person_request::FindPersonRequestActionServer>();

    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;
}