#ifndef GADEN2_RVIZ_PUBLICATION_BASE_HPP_INCLUDED
#define GADEN2_RVIZ_PUBLICATION_BASE_HPP_INCLUDED

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace gaden2::rviz {

class VisualisationBase
{
public:
    static constexpr char DEFAULT_STATIC_MARKERS_TOPIC_NAME[] = "gaden2_static";

    VisualisationBase(const std::string &node_name,
                      const std::string &static_markers_topic_name = DEFAULT_STATIC_MARKERS_TOPIC_NAME);
    ~VisualisationBase();

    std::shared_ptr<rclcpp::Node> getNode();

    inline rclcpp::Time getTimeNow() const
    {
        return node_clock_->now();
    }

    inline void publishStaticMarker(const visualization_msgs::msg::Marker &marker)
    {
        publisher_static_markers_->publish(marker);
    }

private:
    void spinRosNode();

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Clock> node_clock_;

    std::thread thread_ros_spin_;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> publisher_static_markers_;
};

} // namespace gaden2::rviz

#endif // GADEN2_RVIZ_PUBLICATION_BASE_HPP_INCLUDED
