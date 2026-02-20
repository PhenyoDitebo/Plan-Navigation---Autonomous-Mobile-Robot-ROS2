#include "bumperbot_motion/pd_motion_planner.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// ------------------------------------- CONSTRUCTOR -------------------------------------------
namespace bumperbot_motion {
    PDMotionPlanner::PDMotionPlanner(): Node("pd_motion_planner_node"), 
    kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3), max_angular_velocity_(1.0) {

        // ------------------------- DECLARING PARAMETERS ------------------------------------

        declare_parameter<double>("kp", kp_);
        declare_parameter<double>("kd", kd_);
        declare_parameter<double>("step_size", step_size_);
        declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
        declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

        // -------------------------- SAVING DATA INTO THE PARAMETERS -----------------------------

        kp_ = get_parameter("kp").as_double();
        kd_ = get_parameter("kd").as_double();
        step_size_ = get_parameter("step_size").as_double();
        max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

        // ------------------------------------ INITIALIZATION ------------------------------------------------

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/a_star/path", 10, std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);

        // ------------------------------- RETRIEVE CURRENT POSITION OF THE BUMPER ---------------------------
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        RCLCPP_INFO(this->get_logger(), "tf buffer created");

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(this->get_logger(), "tf listener created");

        // ------------------------------- CONTROL LOOP LOGIC -----------------------------------
        control_loop_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PDMotionPlanner::controlLoop, this));
        // run the control loop every 100ms.
    }

    // ------------------------------------ FUNCTION DECLARATION -----------------------------------
    void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {
        global_plan_ = *path;
    }

    void PDMotionPlanner::controlLoop() {
        if (global_plan_.poses.empty()) { // if we have no path, no need to do anything.
        }

        geometry_msgs::msg::TransformStamped robot_pose;
        
        try {
            robot_pose = tf_buffer_ -> lookupTransform("odom", "base_footprint", tf2::TimePointZero); //odom - odometry frame
        } 
        catch(tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
            return;
        }


        // ------------------------- print the 2 frame ids for comparison -----------------------
        // will use for velocity command. 
        RCLCPP_INFO(get_logger(), "frame_id Robot Pose: %s", robot_pose.header.frame_id.c_str());
        RCLCPP_INFO(get_logger(), "frame_id Global Plan: %s", global_plan_.header.frame_id.c_str());

    }

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PDMotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}