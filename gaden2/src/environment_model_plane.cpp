#include <gaden2/environment_model_plane.hpp>

namespace gaden2 {

static constexpr double z_min = -1.0;

EnvironmentModelPlane::EnvironmentModelPlane(double x_min,
                                             double x_max,
                                             double y_min,
                                             double y_max,
                                             double z_max,
                                             rl::Logger parent_logger)
    : EnvironmentModel(parent_logger)
    , world_min_(x_min, y_min, z_min)
    , world_max_(x_max, y_max, z_max)
    , plane_min_(world_min_)
    , plane_max_(x_max, y_max, 0)
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

Occupancy EnvironmentModelPlane::getOccupancy(const Eigen::Vector3d &p) const
{
    if ((p.array() < world_min_.array()).any() || (p.array() > world_max_.array()).any())
        return Occupancy::OutOfWorld;
    else if ((p.array() >= plane_min_.array()).any() && (p.array() <= plane_max_.array()).any())
        return Occupancy::Occupied;
    else
        return Occupancy::Free;
}

Eigen::Vector3d EnvironmentModelPlane::getPlaneCenterCoordinates() const
{
    return 0.5 * (plane_min_ + plane_max_);
}

Eigen::Vector3d EnvironmentModelPlane::getPlaneDimensions() const
{
    return plane_max_ - plane_min_;
}

} // namespace gaden2
