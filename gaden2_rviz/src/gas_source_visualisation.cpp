#include <gaden2/gas_source.hpp>
#include <gaden2_rviz/gas_source_visualisation.hpp>
#include <gaden2_rviz/visualisation_base.hpp>
#include <gaden2_rviz/helpers/ros_type_conversions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace gaden2::rviz {

GasSourceVisualisation::GasSourceVisualisation(std::shared_ptr<VisualisationBase> visualisation_base,
                                               const std::vector<std::shared_ptr<GasSource>> &gas_sources,
                                               int publication_interval,
                                               const std::string &marker_namespace,
                                               int marker_id,
                                               const std::string &marker_frame_id)
    : visualisation_base_(visualisation_base)
{
    auto node = visualisation_base_->getNode();

    constexpr double GAS_SOURCE_WIDTH = 0.1;
    constexpr double GAS_SOURCE_HEIGHT = 0.3;

    marker_.header.frame_id = marker_frame_id;
    marker_.ns = marker_namespace;
    marker_.id = marker_id;
    marker_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    //marker_.pose.position = ros_type_conversion::getPoint(0,0,0);
    // position is default initialised to (0,0,0)
    // orientation is default initialised to (0,0,0,1)
    marker_.scale = ros_type_conversion::getVector3(GAS_SOURCE_WIDTH, GAS_SOURCE_WIDTH, GAS_SOURCE_HEIGHT);
    marker_.color = ros_type_conversion::getColor(0.0, 0.0, 1.0);

    for (const std::shared_ptr<GasSource> &gas_source : gas_sources)
    {
        const Eigen::Vector3d &p = gas_source->getPosition();
        marker_.points.push_back(
                    ros_type_conversion::getPoint(p[0], p[1], p[2] - 0.5*GAS_SOURCE_HEIGHT));
    }

    if (publication_interval > 0)
    {
        publish();
        timer_publication_ = node->create_wall_timer(std::chrono::milliseconds(publication_interval), [this](){publish();});
    }
    else if (publication_interval == -1)
        publish();
}

GasSourceVisualisation::~GasSourceVisualisation()
{
    timer_publication_.reset();
}

void GasSourceVisualisation::publish()
{
    marker_.header.stamp = visualisation_base_->getTimeNow();
    visualisation_base_->publishStaticMarker(marker_);
}

} // namespace gaden2::rviz
