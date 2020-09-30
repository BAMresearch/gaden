#ifndef GADEN2_RVIZ_GAS_SOURCE_VISUALISATION_HPP_INCLUDED
#define GADEN2_RVIZ_GAS_SOURCE_VISUALISATION_HPP_INCLUDED

#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <vector>

namespace rclcpp {
class TimerBase;
}

namespace gaden2 {

class GasSource;

namespace rviz {

class VisualisationBase;

class GasSourceVisualisation
{
public:
    // define default variables, so that pybind11 can also use them
    //static constexpr char DEFAULT_TOPIC_NAME[] = "gas_source_marker";
    static constexpr int DEFAULT_PUBLICATION_INTERVAL = 5000;
    static constexpr char DEFAULT_MARKER_NAMESPACE[] = "gas_sources";
    static constexpr int DEFAULT_MARKER_ID = 0;
    static constexpr char DEFAULT_MARKER_FRAME_ID[] = "map";

    GasSourceVisualisation(std::shared_ptr<VisualisationBase> visualisation_base,
                           const std::vector<std::shared_ptr<GasSource>> &gas_sources,
                           //const std::string &topic_name = DEFAULT_TOPIC_NAME,
                           int publication_interval = DEFAULT_PUBLICATION_INTERVAL, // [ms], special values: 0 = do not publish, -1 = publish once on creation
                           const std::string &marker_namespace = DEFAULT_MARKER_NAMESPACE,
                           int marker_id = DEFAULT_MARKER_ID,
                           const std::string &marker_frame_id = DEFAULT_MARKER_FRAME_ID);
    ~GasSourceVisualisation();

private:
    void publish();

    std::shared_ptr<VisualisationBase> visualisation_base_;

    std::shared_ptr<rclcpp::TimerBase> timer_publication_;
    visualization_msgs::msg::Marker marker_;
};

}} // namespace gaden2::rviz

#endif // GADEN2_RVIZ_GAS_SOURCE_VISUALISATION_HPP_INCLUDED
