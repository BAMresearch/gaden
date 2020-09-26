#ifndef GADEN2_RVIZ_ENVIRONMENT_MODEL_HPP_INCLUDED
#define GADEN2_RVIZ_ENVIRONMENT_MODEL_HPP_INCLUDED

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>

namespace gaden2 {

class EnvironmentModelPlane;

namespace rviz {

class VisualisationBase;

class EnvironmentVisualisationPlane
{
public:
    // define default variables, so that pybind11 can also use them
    static constexpr char DEFAULT_TOPIC_NAME[] = "environment_marker";
    static constexpr int DEFAULT_PUBLICATION_INTERVAL = 5000;
    static constexpr char DEFAULT_MARKER_NAMESPACE[] = "environment";
    static constexpr int DEFAULT_MARKER_ID = 0;
    static constexpr char DEFAULT_MARKER_FRAME_ID[] = "map";

    EnvironmentVisualisationPlane(std::shared_ptr<VisualisationBase> visualisation_base,
                                  std::shared_ptr<EnvironmentModelPlane> model,
                                  const std::string &topic_name = DEFAULT_TOPIC_NAME,
                                  int publication_interval = DEFAULT_PUBLICATION_INTERVAL, // [ms], special values: 0 = do not publish, -1 = publish once on creation
                                  const std::string &marker_namespace = DEFAULT_MARKER_NAMESPACE,
                                  int marker_id = DEFAULT_MARKER_ID,
                                  const std::string &marker_frame_id = DEFAULT_MARKER_FRAME_ID);
    ~EnvironmentVisualisationPlane();

private:
    void publishEnvironment();

    std::shared_ptr<VisualisationBase> visualisation_base_;

    std::shared_ptr<rclcpp::TimerBase> timer_publication_;
    visualization_msgs::msg::Marker marker_plane_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> publisher_marker_;
};

}} // namespace gaden2::rviz

#endif // GADEN2_RVIZ_ENVIRONMENT_MODEL_HPP_INCLUDED
