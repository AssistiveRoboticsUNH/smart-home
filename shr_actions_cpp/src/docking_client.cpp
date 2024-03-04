#include "shr_msgs/action/docking_request.hpp"
//#include "shr_msgs/action/localize_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("action_server_cancellation");

    auto client = rclcpp_action::create_client<shr_msgs::action::DockingRequest>(node, "docking");
    //auto client = rclcpp_action::create_client<shr_msgs::action::LocalizeRequest>(node, "localize");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return 1;
    }

    auto cancel_future = client->async_cancel_all_goals();
    RCLCPP_INFO(node->get_logger(), "Cancellation was requested.");

    rclcpp::spin_until_future_complete(node, cancel_future);

    rclcpp::shutdown();
    return 0;
}
