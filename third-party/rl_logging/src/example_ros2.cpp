#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rl_logging/ros2_logging.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("rl_logging_example");

    rl::Logger log = rl::logging::Ros2Logger::create(ros_node->get_logger());

    log.info("rl_logging ROS2 example started");

    int x = 8;
    int y = 5;
    log.error() << "Stream logging: " << x << " != " << y;

    rl::Logger log_child = log.getChild("child");
    log_child.info("A child is born.");

    log.warn("rl_logging ROS2 example is going to end");

    return 0;
}
