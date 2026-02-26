#include "bumperbot_motion/pd_motion_planner.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

        if (!transformPlan(robot_pose.header.frame_id)) {
            RCLCPP_ERROR(get_logger(), "Failed to transform Global Plan from '%s' to Robot Frame '%s'.");
            return;
        }
        
    }

    // -------------------------------------------------------- PATH TRANSFORM ----------------------------------------------------------
    // This will take a path (or a list of positions) and shift them all from their current reference frame (e.g. map), 
    // to the new target frame (e.g. odom, base_link)
    bool PDMotionPlanner::transformPlan(const std::string &frame) {
        // 1. Efficiency Check: If we are already in the target frame, do nothing.
        if (global_plan_.header.frame_id == frame) {
            return true;
        }

        geometry_msgs::msg::TransformStamped transform;

        try {
            // 2. Lookup the transform from the plan's current frame to the new target frame.
            // tf2::TimePointZero fetches the latest available transform.
            transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
        }
        catch(tf2::LookupException &ex) {
            // 3. Handle cases where the coordinate frames aren't connected in the TF tree.
            RCLCPP_ERROR_STREAM(get_logger(), "Couldn't transform plan from frame " << global_plan_.header.frame_id << " to " << frame);
            return false;
        }

        // 4. Loop through every pose in the path and apply the transformation.
        for(auto &pose : global_plan_.poses) { // this will update all the poses in the path
            // turn geometry messages into TF2 objects
            // tf2::doTransform(input, output, transform_stamped)
            // Here, we overwrite the existing pose with its transformed version.
            tf2::doTransform(pose, pose, transform);
        }

        global_plan_.header.frame_id = frame;
        return true;
    }

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PDMotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}