# include "rclcpp/rclcpp.hpp"
# include "nav_msgs/msg/path.hpp"
# include "geometry_msgs/msg/twist.hpp"
# include "geometry_msgs/msg/pose_stamped.hpp"
# include "tf2_ros/buffer.h" // The Buffer is like a database that stores the history of where every part of the robot was for the last few seconds
# include "tf2_ros/transform_listener.h" // It automatically "hears" the transform data being broadcast by other nodes (like your robot state publisher) and stuffs that data into the Buffer mentioned above.


namespace bumperbot_motion { // we use namespace to avoid "name collisions."

    // going to receieve the path calculated by the planner.
    class PDMotionPlanner: public rclcpp::Node {
        public:
            PDMotionPlanner();

        private:
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_; // will receive path from the planner package. Need to subscribe to that topic to do so.
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; // will publish a velocity message, twist.
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_; // this will show the position and orientation of the robot. And also when the car was at that point, and also the frame id.
    };
}