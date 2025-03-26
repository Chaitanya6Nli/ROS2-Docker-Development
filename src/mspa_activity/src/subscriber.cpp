#include "rclcpp/rclcpp.hpp" // Include ROS 2 client library
#include "std_msgs/msg/string.hpp" // Include the String message type

using std::placeholders::_1; // Use placeholders for callback arguments

class StringSubscriber : public rclcpp::Node { // Define a class that inherits from rclcpp::Node
public:
    StringSubscriber() : Node("string_subscriber") { // Constructor
        // Create a subscriber that subscribes to the "topic" topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&StringSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) { // Callback function for the subscriber
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str()); // Log the received message
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; // Subscriber object
};

int main(int argc, char *argv[]) { // Main function
    rclcpp::init(argc, argv); // Initialize ROS 2
    rclcpp::spin(std::make_shared<StringSubscriber>()); // Spin the node
    rclcpp::shutdown(); // Shutdown ROS 2
    return 0;
}