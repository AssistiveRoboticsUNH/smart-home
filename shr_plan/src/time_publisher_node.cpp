#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include<ctime>

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

      // Get the current system time
        time_t t; // t passed as argument in function time()
        struct tm * tt; // declaring variable for localtime()
        time (&t); //passing argument to time()
        tt = localtime(&t);

        // Convert hours and minutes to seconds
        int total_seconds = (tt->tm_hour * 3600) + (tt->tm_min * 60) + tt->tm_sec;
        std::cout << "total_seconds: " << total_seconds << std::endl;

        // Convert to ROS 2 time format
        builtin_interfaces::msg::Time ros_time;
        ros_time.sec = total_seconds;
        ros_time.nanosec = 0; // No nanoseconds in this example

        // Publish the time
        publisher_->publish(ros_time);

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
