#include "rclcpp/rclcpp.hpp" // Include ROS 2 client library
#include "std_msgs/msg/string.hpp" // Include the String message type
#include <chrono> // Include chrono for time-related functions
#include <sstream> // Include stringstream for building strings

using namespace std::chrono_literals; // Use chrono literals for time units

class StringPublisher : public rclcpp::Node { // Define a class that inherits from rclcpp::Node
public:
    StringPublisher() : Node("string_publisher"), count_(0) { // Constructor
        // Create a publisher that publishes String messages to the "topic" topic
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // Create a timer that calls the timer_callback function every 1 second
        timer_ = this->create_wall_timer(1s, std::bind(&StringPublisher::timer_callback, this));
    }

private:
    void timer_callback() { // Callback function for the timer
        auto message = std_msgs::msg::String(); // Create a String message
        std::stringstream ss; // Create a stringstream object
        ss << "Hello, ROS 2! Count: " << count_++; // Build the message string
        message.data = ss.str(); // Set the message data
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // Log the message
        publisher_->publish(message); // Publish the message
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // Publisher object
    rclcpp::TimerBase::SharedPtr timer_; // Timer object
    size_t count_; // Message counter
};

int main(int argc, char *argv[]) { // Main function
    rclcpp::init(argc, argv); // Initialize ROS 2
    rclcpp::spin(std::make_shared<StringPublisher>()); // Spin the node
    rclcpp::shutdown(); // Shutdown ROS 2
    return 0;
}