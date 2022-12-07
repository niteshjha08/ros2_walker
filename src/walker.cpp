#include <ros2_walker/walker.hpp>


Walker::Walker() {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Walker::scan_callback, this, std::placeholders::_1));
    current_state_ = "forward";
    obstacle_margin_ = 0.5;
}

void Walker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float min_range = msg->range_min;
    float max_range = msg->range_max;
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    float angle_increment = msg->angle_increment;
    float range = msg->ranges[0];

    double min_distance = max_range;
    int start_idx = 40;
    int end_idx = 320;
    for(int i = start_idx; i< end_idx; i++) {
        if (msg->ranges[i] < min_distance) {
            min_distance = msg->ranges[i];
        }
    }

    if (range < obstacle_margin_) {
        current_state_ = "turn";
    } else {
        current_state_ = "forward";
    }

    if (current_state_ == "forward") {
        cmd_vel_.linear.x = 0.5;
        cmd_vel_.angular.z = 0.0;
    } else if (current_state_ == "turn") {
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.5;
    }

    vel_pub_->publish(cmd_vel_);
}
