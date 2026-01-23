#include "rclcpp/rclcpp.hpp" // Include the C++ library from ROS 2, which contains the rclcpp::Node class.

//create a class that inherits from the node class. 
class MyCustomNode: public rclcpp::Node {
    private:
        int counter_;
        rclcpp::TimerBase::SharedPtr timer_;

    public: 
        MyCustomNode() : Node("my_node_name"), counter_(0) { //constructor
            timer_ = this -> create_wall_timer(std::chrono::seconds(1), std::bind(&MyCustomNode::print_hello, this));
        }

        //callback function
        void print_hello() {
            RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
            counter_++;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); //this initializes the ROS 2 Communications. 

    auto node = std::make_shared<MyCustomNode>(); //making a new node object from the class above, using a shared pointer (type of smart pointer)
    rclcpp::spin(node); //making the node spin, this keeps it alive/active. Prevents the main funtion from finishing/exiting the program. Its a blocking call.
    rclcpp::shutdown(); //when node is stopped (ctrl + c), we use this to shut down ROS 2 Coms

    return 0;
}

