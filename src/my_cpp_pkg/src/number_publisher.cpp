
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"


class NumberPublisher: public rclcpp::Node {
    private: 
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
        rclcpp::TimerBase::SharedPtr number_timer_;

        //used int64_t to make sure its exactly 64 bits (8 bytes), regardless of what comp is being used.
        //this also helps the code act identically on every machine. 
        int64_t number_;

    public:
        NumberPublisher(): Node("number_publisher") {

            //initialize the number first. 
            number_ = 0; 

            number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10); //publisher object
            number_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NumberPublisher::publishNumber, this));
            RCLCPP_INFO(this->get_logger(), "Number Publisher has started.");

        }

        void publishNumber() {
            auto msg = example_interfaces::msg::Int64();
            msg.data = number_;
            number_publisher_ -> publish(msg);

            number_++;
        }
};

int main (int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}