#ifndef RL_LOGGING_ROS2_LOGGING_HPP_INCLUDED
#define RL_LOGGING_ROS2_LOGGING_HPP_INCLUDED

#include <rclcpp/logging.hpp>

#include "logging_interface.hpp"

namespace rl::logging {

class Ros2Logger : public rl::logging::internal::LoggerInterface
{
public:
    // std::make_shared cannot access the private constructor.
    // Therefore, a helper struct is used that makes the constructor accessible.

    static Logger create(rclcpp::Logger logger)
    {
        struct MakeConstructorPublic : public Ros2Logger {
            MakeConstructorPublic(rclcpp::Logger logger):Ros2Logger(logger) {}
        };
        return Logger(std::make_shared<MakeConstructorPublic>(logger));
    }

    std::shared_ptr<LoggerInterface> getChild(const std::string &child_name)
    {
        struct MakeConstructorPublic : public Ros2Logger {
            MakeConstructorPublic(rclcpp::Logger logger):Ros2Logger(logger) {}
        };
        return std::make_shared<MakeConstructorPublic>(logger_.get_child(child_name));
    }

    void debug(const std::string &message) { RCLCPP_DEBUG(logger_, message.c_str()); }
    void info( const std::string &message) { RCLCPP_INFO( logger_, message.c_str()); }
    void warn( const std::string &message) { RCLCPP_WARN( logger_, message.c_str()); }
    void error(const std::string &message) { RCLCPP_ERROR(logger_, message.c_str()); }
    void fatal(const std::string &message) { RCLCPP_FATAL(logger_, message.c_str()); }

private:
    Ros2Logger(rclcpp::Logger logger) : logger_(logger) {}

    rclcpp::Logger logger_;
};

} // namespace rl::logging

#endif // RL_LOGGING_ROS2_LOGGING_HPP_INCLUDED
