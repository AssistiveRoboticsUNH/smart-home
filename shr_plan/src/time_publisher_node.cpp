#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

class NewYorkTimePublisher : public rclcpp::Node {
public:
  NewYorkTimePublisher() : Node("new_york_time_publisher") {
    publisher_ = create_publisher<builtin_interfaces::msg::Time>("/protocol_time", 10);

    // Start publishing the New York time with an initial delay of 1 second
    //timer_ = create_wall_timer(10, std::bind(&NewYorkTimePublisher::publishNewYorkTime, this));
    timer_ = this->create_wall_timer(
      500ms, std::bind(&NewYorkTimePublisher::publishNewYorkTime, this));
  }

private:
  void publishNewYorkTime() {
    try {
      // Get the current time in the New York timezone
        auto ny_time = std::chrono::system_clock::now() - std::chrono::hours(4);

        // Calculate the elapsed time since the start of the day in seconds
        auto time_since_midnight = ny_time.time_since_epoch() % std::chrono::hours(24);

        // Convert the elapsed time to ROS 2 time format
        builtin_interfaces::msg::Time ros_time;
        ros_time.sec = static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(
                time_since_midnight)
                .count());
        ros_time.nanosec = 0; // The elapsed time is in seconds, nanoseconds can be set to 0

        // Publish the elapsed time
        publisher_->publish(ros_time);


        //  auto gmt_time = std::chrono::system_clock::now();

      // // Convert GMT time to New York local time
      // auto ny_time_zone = date::locate_zone("America/New_York");
      // auto ny_time_local = date::zoned_time<std::chrono::system_clock::duration>(ny_time_zone, gmt_time);

      // // Convert the New York local time to ROS 2 time format
      // builtin_interfaces::msg::Time ros_time;
      // ros_time.sec = std::chrono::duration_cast<std::chrono::seconds>(
      //     ny_time_local.get_sys_time().time_since_epoch()).count();
      // ros_time.nanosec = 0; 
  

    } catch (const std::exception &ex) {
      // Handle any exceptions
      RCLCPP_ERROR(this->get_logger(), "Error during New York time publication: %s", ex.what());
    }
  }

private:
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NewYorkTimePublisher>());
  rclcpp::shutdown();
  return 0;
}
