#include <gaden2/filament_model.hpp>
#include <gaden2_rviz/filament_visualisation.hpp>
#include <gaden2_rviz/visualisation_base.hpp>
#include <gaden2_rviz/helpers/ros_type_conversions.hpp>

namespace gaden2::rviz {

FilamentVisualisation::FilamentVisualisation(std::shared_ptr<VisualisationBase> visualisation_base,
                                             std::shared_ptr<FilamentGasModel> filament_model,
                                             double marker_scale,
                                             const std::string &topic_name,
                                             const std::string &marker_namespace,
                                             const std::string &marker_frame_id)
    : visualisation_base_(visualisation_base)
    , filament_model_(filament_model)
{
    marker_.header.frame_id = marker_frame_id;
    marker_.ns = marker_namespace;
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::POINTS;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.scale = ros_type_conversion::getVector3(marker_scale);
    marker_.color = ros_type_conversion::colors::red;

    publisher_ = visualisation_base_->getNode()->create_publisher<visualization_msgs::msg::Marker>(topic_name, 5);

    callback_id_ = filament_model_->addPostIncrementCallback([this]() { publish(); });
}

FilamentVisualisation::~FilamentVisualisation()
{
    filament_model_->removeCallback(callback_id_);
}

void FilamentVisualisation::publish()
{
    marker_.header.stamp = visualisation_base_->getTimeNow();
    const std::list<Filament> &filaments = filament_model_->getFilaments();

    marker_.points.resize(filaments.size());
    marker_.colors.resize(filaments.size(), marker_.color);

    size_t i = 0;
    for (const Filament &filament : filaments)
    {
        marker_.points[i] = ros_type_conversion::getPointFrom(filament.position);
        ++i;
    }

    publisher_->publish(marker_);
}

} // namespace gaden2::rviz
