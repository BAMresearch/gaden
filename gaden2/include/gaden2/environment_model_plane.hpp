#ifndef GADEN2_ENVIRONMENT_MODEL_PLANE_HPP_INCLUDED
#define GADEN2_ENVIRONMENT_MODEL_PLANE_HPP_INCLUDED

#include "environment_model.hpp"
#include "logger.hpp"

#include <Eigen/Core>

namespace gaden2 {

class EnvironmentModelPlane : public EnvironmentModel
{
public:
    static constexpr double DEFAULT_X_MIN = -50.0;
    static constexpr double DEFAULT_X_MAX = 50.0;
    static constexpr double DEFAULT_Y_MIN = -25.0;
    static constexpr double DEFAULT_Y_MAX = 25.0;
    static constexpr double DEFAULT_Z_MAX = 50.0;

    // TODO Adjust Python export
    EnvironmentModelPlane(double x_min = DEFAULT_X_MIN,
                          double x_max = DEFAULT_X_MAX,
                          double y_min = DEFAULT_Y_MIN,
                          double y_max = DEFAULT_Y_MAX,
                          double z_max = DEFAULT_Z_MAX,
                          rl::Logger parent_logger = getStandardLogger());
    ~EnvironmentModelPlane();

    Eigen::Vector3d getEnvironmentMin() const;
    Eigen::Vector3d getEnvironmentMax() const;

    Occupancy getOccupancy(const Eigen::Vector3d &p) const;

    Eigen::Vector3d getPlaneCenterCoordinates() const;
    Eigen::Vector3d getPlaneDimensions() const;

private:
    Eigen::Vector3d world_min_; // [m] minimum in world coordinates
    Eigen::Vector3d world_max_; // [m] maximum in world coordinates
    Eigen::Vector3d plane_min_; // [m]
    Eigen::Vector3d plane_max_; // [m]
};

} // namespace gaden2

#endif // GADEN2_ENVIRONMENT_MODEL_PLANE_HPP_INCLUDED
