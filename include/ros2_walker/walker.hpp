#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Walker : public rclcpp::Node {
    public:
        Walker();

    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

        geometry_msgs::msg::Twist cmd_vel_;

        std::string current_state_;

        double obstacle_margin_;
};