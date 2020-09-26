#ifndef GADEN2_ENVIRONMENT_MODEL_PLANE_HPP_INCLUDED
#define GADEN2_ENVIRONMENT_MODEL_PLANE_HPP_INCLUDED

#include "environment_model.hpp"
#include "logger.hpp"

#include <Eigen/Core>

namespace gaden2 {

class EnvironmentModelPlane : public EnvironmentModel
{
public:
    EnvironmentModelPlane(double x_min = -50.0, double x_max = 50.0,
                          double y_min = -25.0, double y_max = 25.0,
                          rl::Logger parent_logger = getStandardLogger());
    ~EnvironmentModelPlane();

    Eigen::Vector3d getEnvironmentMin() const;
    Eigen::Vector3d getEnvironmentMax() const;

    Eigen::Vector3d getCenterCoordinates() const;
    Eigen::Vector3d getDimensions() const;

private:
    Eigen::Vector3d world_min_; // [length] minimum in world coordinates
    Eigen::Vector3d world_max_; // [length] maximum in world coordinates
    //double x_min_, x_max_;
    //double y_min_, y_max_;
};

} // namespace gaden2

#endif // GADEN2_ENVIRONMENT_MODEL_PLANE_HPP_INCLUDED
