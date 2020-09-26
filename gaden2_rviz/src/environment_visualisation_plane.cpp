#include <gaden2/environment_model_plane.hpp>
#include <gaden2_rviz/environment_visualisation_plane.hpp>
#include <gaden2_rviz/visualisation_base.hpp>
#include <gaden2_rviz/helpers/ros_type_conversions.hpp>

namespace gaden2::rviz {

EnvironmentVisualisationPlane::EnvironmentVisualisationPlane(std::shared_ptr<VisualisationBase> visualisation_base,
                                                             std::shared_ptr<EnvironmentModelPlane> model,
                                                             const std::string &topic_name,
                                                             int publication_interval,
                                                             const std::string &marker_namespace,
                                                             int marker_id,
                                                             const std::string &marker_frame_id)
    : visualisation_base_(visualisation_base)
{
    auto node = visualisation_base_->getNode();
    publisher_marker_ = node->create_publisher<visualization_msgs::msg::Marker>(topic_name, 5);

    marker_plane_.header.frame_id = marker_frame_id;
    marker_plane_.ns = marker_namespace;
    marker_plane_.id = marker_id;
    marker_plane_.type = visualization_msgs::msg::Marker::CUBE;
    marker_plane_.action = visualization_msgs::msg::Marker::ADD;
    marker_plane_.pose.position = ros_type_conversion::getPointFrom(model->getCenterCoordinates());
    // orientation is default initialised to (0,0,0,1)
    marker_plane_.scale = ros_type_conversion::getVector3From(model->getDimensions());
    marker_plane_.color = ros_type_conversion::getColor(0.5, 0.5, 0.5);

    if (publication_interval > 0)
    {
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
    publisher_marker_->publish(marker_plane_);
}

} // namespace gaden2::rviz
