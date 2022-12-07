#include <rclcpp/rclcpp.hpp>
#include <ros2_walker/walker.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Walker>());
    rclcpp::shutdown();
    return 0;
}