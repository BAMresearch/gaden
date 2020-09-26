#ifndef GADEN2_RVIZ_PUBLICATION_BASE_HPP_INCLUDED
#define GADEN2_RVIZ_PUBLICATION_BASE_HPP_INCLUDED

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace gaden2::rviz {

class VisualisationBase
{
public:
    VisualisationBase(const std::string &node_name);
    ~VisualisationBase();

    std::shared_ptr<rclcpp::Node> getNode();

    inline rclcpp::Time getTimeNow() const
    {
        return node_clock_->now();
    }

private:
    void spinRosNode();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Clock> node_clock_;

    std::thread thread_ros_spin_;
};

} // namespace gaden2::rviz

#endif // GADEN2_RVIZ_PUBLICATION_BASE_HPP_INCLUDED
