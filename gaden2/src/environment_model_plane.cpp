#include <gaden2/environment_model_plane.hpp>

namespace gaden2 {

EnvironmentModelPlane::EnvironmentModelPlane(double x_min, double x_max, double y_min, double y_max, rl::Logger parent_logger)
    : EnvironmentModel(parent_logger)
    , world_min_(x_min, y_min, -1)
    , world_max_(x_max, y_max, 0)
    //, x_min_(x_min), x_max_(x_max)
    //, y_min_(y_min), y_max_(y_max)
{
    logger.info() << "Created plane environment model.";
}

EnvironmentModelPlane::~EnvironmentModelPlane()
{}

Eigen::Vector3d EnvironmentModelPlane::getEnvironmentMin() const
{
    return world_min_;
}

Eigen::Vector3d EnvironmentModelPlane::getEnvironmentMax() const
{
    return world_max_;
}

Eigen::Vector3d EnvironmentModelPlane::getCenterCoordinates() const
{
    return 0.5 * (world_min_ + world_max_);
}

Eigen::Vector3d EnvironmentModelPlane::getDimensions() const
{
    return world_max_ - world_min_;
}

} // namespace gaden2
