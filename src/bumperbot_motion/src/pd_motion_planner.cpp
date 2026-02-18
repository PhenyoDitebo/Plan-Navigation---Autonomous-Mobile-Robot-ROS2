#include "bumperbot_motion/pd_motion_planner.hpp"

namespace bumperbot_motion {
    PDMotionPlanner::PDMotionPlanner(): Node("pd_motion_planner_node"), 
    kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3), max_angular_velocity_(1.0) {

    }
}