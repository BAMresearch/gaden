#include <gaden2/environment_model_plane.hpp>
#include <gaden2_rviz/environment_visualisation_plane.hpp>
#include <gaden2_rviz/visualisation_base.hpp>
#include <gaden2_rviz/helpers/ros_type_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace gaden2::rviz {

EnvironmentVisualisationPlane::EnvironmentVisualisationPlane(std::shared_ptr<VisualisationBase> visualisation_base,
                                                             std::shared_ptr<EnvironmentModelPlane> model,
                                                             int publication_interval,
                                                             const std::string &marker_namespace,
                                                             int marker_id,
                                                             const std::string &marker_frame_id)
    : visualisation_base_(visualisation_base)
{
    auto node = visualisation_base_->getNode();

    marker_plane_.header.frame_id = marker_frame_id;
    marker_plane_.ns = marker_namespace;
    marker_plane_.id = marker_id;
    marker_plane_.type = visualization_msgs::msg::Marker::CUBE;
    marker_plane_.action = visualization_msgs::msg::Marker::ADD;
    marker_plane_.pose.position = ros_type_conversion::getPointFrom(model->getPlaneCenterCoordinates());
    // orientation is default initialised to (0,0,0,1)
    marker_plane_.scale = ros_type_conversion::getVector3From(model->getPlaneDimensions());
    marker_plane_.color = ros_type_conversion::getColor(0.5, 0.5, 0.5);

    if (publication_interval > 0)
    {
        publishEnvironment();
        timer_publication_ = node->create_wall_timer(std::chrono::milliseconds(publication_interval), [this](){publishEnvironment();});
    }
    else if (publication_interval == -1)
        publishEnvironment();
}

EnvironmentVisualisationPlane::~EnvironmentVisualisationPlane()
{
    timer_publication_.reset();
}

void EnvironmentVisualisationPlane::publishEnvironment()
{
    marker_plane_.header.stamp = visualisation_base_->getTimeNow();
    visualisation_base_->publishStaticMarker(marker_plane_);
}

} // namespace gaden2::rviz
