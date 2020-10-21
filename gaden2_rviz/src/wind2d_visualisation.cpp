#include <gaden2/wind_model.hpp>
#include <gaden2_rviz/visualisation_base.hpp>
#include <gaden2_rviz/wind2d_visualisation.hpp>
#include <gaden2_rviz/helpers/ros_type_conversions.hpp>

#include <iostream>

namespace gaden2::rviz {

Wind2dVisualisation::Wind2dVisualisation(std::shared_ptr<VisualisationBase> visualisation_base,
                                         std::shared_ptr<WindModel> wind_model,
                                         double resolution,
                                         double z,
                                         const std::string &topic_name,
                                         const std::string &marker_namespace,
                                         const std::string &marker_frame_id)
    : visualisation_base_(visualisation_base)
    , wind_model_(wind_model)
    , resolution_(resolution)
    , z_(z)
{
    x_min_ = wind_model_->getEnvironmentMin()(0);
    y_min_ = wind_model_->getEnvironmentMin()(1);

    x_max_ = wind_model_->getEnvironmentMax()(0);
    y_max_ = wind_model_->getEnvironmentMax()(1);

    auto marker_color = ros_type_conversion::colors::blue;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = marker_frame_id;
    marker.ns = marker_namespace;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale = ros_type_conversion::getVector3(0.1, 0.4, 0); // shaft diameter, head diameter, 0 or head length
    marker.color = marker_color;
    marker.points.emplace_back();
    marker.points.emplace_back();
    marker.colors.push_back(marker_color);
    marker.colors.push_back(marker_color);

    iterateCoordinates([&](const Eigen::Vector3d &p, size_t i)
    {
        marker.id = i;
        marker.points.at(0) = ros_type_conversion::getPointFrom(p);
        marker_array_.markers.push_back(marker);
    });

    publisher_ = visualisation_base_->getNode()->create_publisher<visualization_msgs::msg::MarkerArray>(topic_name, 5);
    publish();

    callback_id_ = wind_model_->addPostIncrementCallback([this]() { publish(); });
}

Wind2dVisualisation::~Wind2dVisualisation()
{
    wind_model_->removeCallback(callback_id_);
}

void Wind2dVisualisation::iterateCoordinates(std::function<void (const Eigen::Vector3d &, size_t)> f)
{
    size_t i = 0;
    Eigen::Vector3d p;
    p(2) = z_;
    for (p(0) = x_min_; p(0) <= x_max_; p(0) += resolution_)
        for (p(1) = y_min_; p(1) <= y_max_; p(1) += resolution_)
        {
            f(p, i);
            ++i;
        }
}

void Wind2dVisualisation::publish()
{
    iterateCoordinates([&](const Eigen::Vector3d &p, size_t i)
    {
        Eigen::Vector3d wind_velocity = wind_model_->getWindVelocityAt(p);

        visualization_msgs::msg::Marker &marker = marker_array_.markers.at(i);
        marker.points.at(1) = ros_type_conversion::getPointFrom(p + wind_velocity);
    });

    publisher_->publish(marker_array_);
}

} // namespace gaden2::rviz
