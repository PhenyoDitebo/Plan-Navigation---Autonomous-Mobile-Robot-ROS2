#include "bumperbot_motion/pd_motion_planner.hpp"

namespace bumperbot_motion {
    PDMotionPlanner::PDMotionPlanner(): Node("pd_motion_planner_node"), 
    kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3), max_angular_velocity_(1.0) {

        // ------------------------- DECLARING PARAMETERS ------------------------------------

        declare_parameter<double>("kp", kp_);
        declare_parameter<double>("kd", kd_);
        declare_parameter<double>("step_size", step_size_);
        declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
        declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

        // -------------------------- SAVING DATA INTO THE PARAMETERS -------------------------

        kp_ = get_parameter("kp").as_double();
        kd_ = get_parameter("kd").as_double();
        step_size_ = get_parameter("step_size").as_double();
        max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();
        
    }
}