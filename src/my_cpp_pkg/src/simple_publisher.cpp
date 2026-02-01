#include <rclcpp/rclcpp.hpp> // core ROS2 Library
#include <std_msgs/msg/string.hpp> // includes the string message type definition

// rclcpp is the namespace for the ROS2 C++ Client Library, it is the family name for all C++ ROS2 classes and functions
// std_msgs is the namespace for standard message definitions, msg is the sub-namespace for message types
// rclcpp::Node is the class for creating a ROS2 node, which we inherit to create our own node class
// rclcpp::Publisher is the class for creating a publisher object, and is the tool for sending messages
// rclcpp::init is the function to initialize the ROS2 system (or start the ROS2 client library)

#include <chrono>
using namespace std::chrono_literals;

class simplePublisher : public rclcpp::Node { // create a class derived from rclcpp::Node, inherting its properties
    public:
        simplePublisher() : Node("simple_publisher"), counter_(0) // constructor
        {
            pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10); // create publisher object, defining topic name and queue size
            timer_ = create_wall_timer(1s, std::bind(&simplePublisher::timerCallback, this)); // create a timer to trigger periodic callbacks every second

            RCLCPP_INFO(get_logger(), "Publishing at 1Hz"); // log info message to indicate publishing rate
        }
    private:
        unsigned int counter_; // counter variable to keep track of number of messages published
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; // publisher object, with its own type definition, smart pointer --> cleans up memeory automatically
        rclcpp::TimerBase::SharedPtr timer_; // timer object to trigger periodic callbacks

        void timerCallback() { // callback function triggered by the timer
            auto message = std_msgs::msg::String(); // create a new string message object, auto keyword deduces the type automatically by compiler
            message.data = "Hello ROS2 - Counter " + std::to_string(counter_++); // Converts the number to text so it can be added to the string.
            pub_->publish(message); // publish the message using the publisher object
        }
};

int main(int argc, char* argv[]) { // standard C++ main function, it accepts command line arguments, int argc is the number of arguments, char* argv[] is an array of argument strings

    rclcpp::init(argc, argv); // master power switch, initializes the ROS2 system
    auto node = std::make_shared<simplePublisher>(); // create an instance of the simplePublisher node using a smart pointer, which means memory is managed automatically and lifecyle is handled
    rclcpp::spin(node); // spin function keeps the node alive and processing callbacks (like timerCallback) until shutdown
    rclcpp::shutdown(); // cleanly shutdown the ROS2 system, releasing resources

    return 0;
}
