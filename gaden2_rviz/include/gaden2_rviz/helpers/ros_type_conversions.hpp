#ifndef GADEN2_RVIZ_HELPERS_ROS_TYPE_CONVERSIONS_HPP_INCLUDED
#define GADEN2_RVIZ_HELPERS_ROS_TYPE_CONVERSIONS_HPP_INCLUDED

#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <string>

namespace gaden2::rviz::ros_type_conversion {

geometry_msgs::msg::Vector3 getVector3(double x, double y, double z);
geometry_msgs::msg::Vector3 getVector3(double value);
geometry_msgs::msg::Vector3 getVector3From(const Eigen::Vector3d &v);

geometry_msgs::msg::Point getPoint(double x, double y, double z);
geometry_msgs::msg::Point getPoint(double value);
geometry_msgs::msg::Point getPointFrom(const Eigen::Vector3d &v);

std_msgs::msg::ColorRGBA getColor(float r, float g, float b, float a = 1.0f);

namespace colors {
extern const std_msgs::msg::ColorRGBA black;
extern const std_msgs::msg::ColorRGBA white;
extern const std_msgs::msg::ColorRGBA grey;
extern const std_msgs::msg::ColorRGBA red;
extern const std_msgs::msg::ColorRGBA green;
extern const std_msgs::msg::ColorRGBA blue;
extern const std_msgs::msg::ColorRGBA yellow;
extern const std_msgs::msg::ColorRGBA orange;
extern const std_msgs::msg::ColorRGBA violet;
} // namespace colors

} // namespace gaden2::rviz::ros_type_conversion

#endif // GADEN2_RVIZ_HELPERS_ROS_TYPE_CONVERSIONS_HPP_INCLUDED
