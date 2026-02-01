#include <rclcpp/rclcpp.hpp> // core ROS2 Library
#include <std_msgs/msg/string.hpp> // includes the string message type definition

using std::placeholders::_1; // for binding the callback function

class SimpleQoSSubscriber : public rclcpp::Node 
{
    public:
        SimpleQoSSubscriber() : Node("simple_subscriber") // constructor 
        {
            sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleQoSSubscriber::msgCallback,this, _1)); // create subscription object, defining topic name, queue size, and binding the callback function
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; // subscription object with its own type definition, smart pointer --> cleans up memory automatically

        void msgCallback(const std_msgs::msg::String &msg) const
        {
            RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str()); // log the received message to the console
        }
};


int main(int argc, char* argv[]) { // standard C++ main function, it accepts command line arguments, int argc is the number of arguments, char* argv[] is an array of argument strings
    rclcpp::init(argc, argv); // master power switch, initializes the ROS2 system

    auto node = std::make_shared<SimpleQoSSubscriber>(); // create an instance of the SimpleSubscriber node using a smart pointer
    rclcpp::spin(node); // spin function keeps the node alive and processing callbacks (like msgCallback) until shutdown
    rclcpp::shutdown(); // cleanly shutdown the ROS2 system, releasing resources

    return 0;
}
