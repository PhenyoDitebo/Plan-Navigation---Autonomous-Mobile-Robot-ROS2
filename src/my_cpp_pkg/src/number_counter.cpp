#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

//place holder for the number subscriber
using std::placeholders::_1;

class NumberCounter : public rclcpp::Node {
    private:
        rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
        int64_t counter_;


    public:
        NumberCounter(): Node ("number_counter") {

            counter_ = 0;
            number_subscriber_ = this ->create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&NumberCounter::callbackNumber, this, _1));

        }
    
        //functions
        void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg) {
            counter_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "Counter: %ld", counter_);
        }
        
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}