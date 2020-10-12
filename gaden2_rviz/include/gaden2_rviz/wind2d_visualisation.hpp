#ifndef GADEN2_RVIZ_WIND2D_VISUALISATION_HPP_INCLUDED
#define GADEN2_RVIZ_WIND2D_VISUALISATION_HPP_INCLUDED

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>

#include <functional>
#include <memory>
#include <string>
//#include <vector>

namespace gaden2 {

class WindModel;

namespace rviz {

class VisualisationBase;

class Wind2dVisualisation
{
public:
    static constexpr char DEFAULT_TOPIC_NAME[] = "wind_visualisation";
    static constexpr double DEFAULT_RESOLUTION = 5.0; // [m]
    static constexpr double DEFAULT_Z = 10.0; // [m]
    static constexpr char DEFAULT_MARKER_NAMESPACE[] = "wind2d";
    static constexpr char DEFAULT_MARKER_FRAME_ID[] = "map";

    Wind2dVisualisation(std::shared_ptr<VisualisationBase> visualisation_base,
                        std::shared_ptr<WindModel> wind_model,
                        double resolution = DEFAULT_RESOLUTION, // [m], place a wind arrow each ... m
                        double z = DEFAULT_Z, // [m], altitude that is visualised
                        const std::string &topic_name = DEFAULT_TOPIC_NAME,
                        const std::string &marker_namespace = DEFAULT_MARKER_NAMESPACE,
                        const std::string &marker_frame_id = DEFAULT_MARKER_FRAME_ID);

    ~Wind2dVisualisation();

    void publish();

private:
    void iterateCoordinates(std::function<void (const Eigen::Vector3d &, size_t)> f);

    std::shared_ptr<VisualisationBase> visualisation_base_;
    std::shared_ptr<WindModel> wind_model_;

    visualization_msgs::msg::MarkerArray marker_array_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> publisher_;
    unsigned callback_id_;

    double resolution_;
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    double z_;
};

//class GasSourceVisualisation
//{
//public:
//    // define default variables, so that pybind11 can also use them
//    //static constexpr char DEFAULT_TOPIC_NAME[] = "gas_source_marker";
//    static constexpr int DEFAULT_PUBLICATION_INTERVAL = 5000;
//    static constexpr char DEFAULT_MARKER_NAMESPACE[] = "gas_sources";
//    static constexpr int DEFAULT_MARKER_ID = 0;
//    static constexpr char DEFAULT_MARKER_FRAME_ID[] = "map";

//    GasSourceVisualisation(std::shared_ptr<VisualisationBase> visualisation_base,
//                           const std::vector<std::shared_ptr<GasSource>> &gas_sources,
//                           //const std::string &topic_name = DEFAULT_TOPIC_NAME,
//                           int publication_interval = DEFAULT_PUBLICATION_INTERVAL, // [ms], special values: 0 = do not publish, -1 = publish once on creation
//                           const std::string &marker_namespace = DEFAULT_MARKER_NAMESPACE,
//                           int marker_id = DEFAULT_MARKER_ID,
//                           const std::string &marker_frame_id = DEFAULT_MARKER_FRAME_ID);
//    ~GasSourceVisualisation();

//private:
//    void publish();

//    std::shared_ptr<VisualisationBase> visualisation_base_;

//    std::shared_ptr<rclcpp::TimerBase> timer_publication_;
//    visualization_msgs::msg::Marker marker_;
//};

}} // namespace gaden2::rviz

#endif // GADEN2_RVIZ_WIND2D_VISUALISATION_HPP_INCLUDED
