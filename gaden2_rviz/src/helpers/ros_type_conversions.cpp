#include <gaden2_rviz/helpers/ros_type_conversions.hpp>

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

} // namespace gaden2::rviz::ros_type_conversion
