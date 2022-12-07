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
 * @file walker.hpp
 * @author Nitesh Jha (niteshj@umd.edu)
 * @brief header file for walker class
 * @version 0.1
 * @date 2022-12-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef INCLUDE_ROS2_WALKER_WALKER_HPP_
#define INCLUDE_ROS2_WALKER_WALKER_HPP_

#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @brief Walker class declaration consisting of constructor, scan callbacks and cmd_vel publisher
 * 
 */
class Walker : public rclcpp::Node {
 public:
 /**
  * @brief Constructor for Walker class where all the publishers and subscribers are initialized
  * 
  */
  Walker();

 private:
    /**
     * @brief Callback function for scan subscriber, in which the robot is made to move forward or 
     * turn based on the distance from the obstacle
     * 
     * @param msg 
     */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  /**
   * @brief Publisher for cmd_vel topic
   * 
   */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  /**
   * @brief Subscriber for laser scan topic
   * 
   */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    /**
     * @brief Twist message to be published on cmd_vel topic
     * 
     */
  geometry_msgs::msg::Twist cmd_vel_;
    /**
     * @brief Current state of the robot
     * 
     */
  std::string current_state_;
    /**
     * @brief Minimum distance from the obstacle possible
     * 
     */
  double obstacle_margin_;
};

#endif  // INCLUDE_ROS2_WALKER_WALKER_HPP_
