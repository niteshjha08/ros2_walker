/**
MIT License

Copyright (c) 2022 Nitesh Jha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*
**/

/**
 * @file walker.cpp
 * @author Nitesh Jha (niteshj@umd.edu)
 * @brief source code for walker class
 * @version 0.1
 * @date 2022-12-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros2_walker/walker.hpp>

Walker::Walker() : Node("walker") {
    // Create publisher to cmd_vel topic
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  // Create subscriber for scan topic
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&Walker::scan_callback, this, std::placeholders::_1));
      // initialize current state to forward
  current_state_ = "forward";
  // minimum distance to obstacle possible
  obstacle_margin_ = 0.5;
}

void Walker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Find minimum distance to obstacle
  double min_distance = 10;
  int start_idx = 40;
  int end_idx = 320;

  for (int i = 0; i < static_cast<int>(msg->ranges.size()); i++) {
    if (i <= start_idx || i >= end_idx) {
      if (!std::isnan(msg->ranges[i])) {
        double scan_dist = msg->ranges[i];
        if (scan_dist < min_distance) {
          min_distance = scan_dist;
        }
      }
    }
  }
    // If obstacle is too close, turn, otherwise go forward
  if (min_distance < obstacle_margin_) {
    current_state_ = "turn";
  } else {
    current_state_ = "forward";
  }
    // Set linear and angular velocity based on current state
  if (current_state_ == "forward") {
    cmd_vel_.linear.x = 0.2;
    cmd_vel_.angular.z = 0.0;
  } else if (current_state_ == "turn") {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.2;
  }
    // Publish cmd_vel
  vel_pub_->publish(cmd_vel_);
}
