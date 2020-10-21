#include <gaden2_rviz/helpers/ros_type_conversions.hpp>

#include <stdexcept>

namespace gaden2::rviz::ros_type_conversion {

geometry_msgs::msg::Vector3 getVector3(double x, double y, double z)
{
    geometry_msgs::msg::Vector3 vector(rosidl_runtime_cpp::MessageInitialization::SKIP);
    vector.x = x;
    vector.y = y;
    vector.z = z;
    return vector;
}

geometry_msgs::msg::Vector3 getVector3(double value)
{
    return getVector3(value, value, value);
}

geometry_msgs::msg::Vector3 getVector3From(const Eigen::Vector3d &v)
{
    return getVector3(v[0], v[1], v[2]);
}

geometry_msgs::msg::Point getPoint(double x, double y, double z)
{
    geometry_msgs::msg::Point point(rosidl_runtime_cpp::MessageInitialization::SKIP);
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

geometry_msgs::msg::Point getPoint(double value)
{
    return getPoint(value, value, value);
}

geometry_msgs::msg::Point getPointFrom(const Eigen::Vector3d &v)
{
    return getPoint(v[0], v[1], v[2]);
}

std_msgs::msg::ColorRGBA getColor(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA c(rosidl_runtime_cpp::MessageInitialization::SKIP);
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
}

namespace colors {

extern const std_msgs::msg::ColorRGBA black = getColor(0,0,0);
extern const std_msgs::msg::ColorRGBA white = getColor(1,1,1);
extern const std_msgs::msg::ColorRGBA grey = getColor(0.5, 0.5, 0.5);
extern const std_msgs::msg::ColorRGBA red = getColor(1,0,0);
extern const std_msgs::msg::ColorRGBA green = getColor(0,1,0);
extern const std_msgs::msg::ColorRGBA blue = getColor(0,0,1);
extern const std_msgs::msg::ColorRGBA yellow = getColor(1,1,0);
extern const std_msgs::msg::ColorRGBA orange = getColor(1.0, 0.5, 0.0);
extern const std_msgs::msg::ColorRGBA violet = getColor(0.5, 0.0, 1.0);

} // namespace colors

} // namespace gaden2::rviz::ros_type_conversion
