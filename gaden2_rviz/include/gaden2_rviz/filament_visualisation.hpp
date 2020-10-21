#ifndef GADEN2_RVIZ_FILAMENT_VISUALISATION_HPP_INCLUDED
#define GADEN2_RVIZ_FILAMENT_VISUALISATION_HPP_INCLUDED

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>

namespace gaden2 {

class FilamentGasModel;

namespace rviz {

class VisualisationBase;

class FilamentVisualisation
{
public:
    static constexpr double DEFAULT_MARKER_SCALE = 0.1;
    static constexpr char DEFAULT_TOPIC_NAME[] = "filament_visualisation";
    static constexpr char DEFAULT_MARKER_NAMESPACE[] = "filament";
    static constexpr char DEFAULT_MARKER_FRAME_ID[] = "map";

    FilamentVisualisation(std::shared_ptr<VisualisationBase> visualisation_base,
                          std::shared_ptr<FilamentGasModel> filament_model,
                          double marker_scale = DEFAULT_MARKER_SCALE,
                          const std::string &topic_name = DEFAULT_TOPIC_NAME,
                          const std::string &marker_namespace = DEFAULT_MARKER_NAMESPACE,
                          const std::string &marker_frame_id = DEFAULT_MARKER_FRAME_ID);

    ~FilamentVisualisation();

private:
    void publish();

    std::shared_ptr<VisualisationBase> visualisation_base_;
    std::shared_ptr<FilamentGasModel> filament_model_;

    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> publisher_;
    visualization_msgs::msg::Marker marker_;

    unsigned callback_id_;
};

}} // namespace gaden2::rviz

#endif // GADEN2_RVIZ_FILAMENT_VISUALISATION_HPP_INCLUDED
