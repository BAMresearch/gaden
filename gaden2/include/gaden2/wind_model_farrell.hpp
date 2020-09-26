#ifndef GADEN2_WIND_MODEL_FARRELL_HPP_INCLUDED
#define GADEN2_WIND_MODEL_FARRELL_HPP_INCLUDED

#include "wind_model.hpp"

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

#include <memory>
#include <string>

namespace gaden2 {

class EnvironmentModel;
class FarrellColouredNoiseGenerator;

struct FarrellsWindModelConfiguration
{
    double u0;              // [m/s] mean wind velocity in x-direction
    double v0;              // [m/s] mean wind velocity in y-direction
    double k_x;             // [m2/s] diffusivity term in x-direction, recommended range: [1, 30]
    double k_y;             // [m2/s] diffusivity term in y-direction, recommended range: [1, 30]
    double noise_gain;      // [] Input gain constant for boundary condition noise generation.
    double noise_damp;      // [] Damping ratio for boundary condition noise generation.
    double noise_bandwidth; // [] Bandwidth for boundary condition noise generation.
};
std::string toString(const FarrellsWindModelConfiguration &config, size_t indention = 0);

class FarrellsWindModel : public WindModel
{
public:
    FarrellsWindModel(const std::shared_ptr<EnvironmentModel> &environment_model,
                      rl::Logger &parent_logger);
    ~FarrellsWindModel();

    virtual void increment(double time_step, double total_sim_time);

    Eigen::Vector3d getWindVelocityAt(const Eigen::Vector3d &position);

private:
    void applyBoundaryConditions(double dt);
    std::tuple<Eigen::ArrayXXd, Eigen::ArrayXXd> getCentred1stDifferences(const Eigen::ArrayXXd &f);
    std::tuple<Eigen::ArrayXXd, Eigen::ArrayXXd> getCentred2ndDifferences(const Eigen::ArrayXXd &f);

    FarrellsWindModelConfiguration config_;
    std::unique_ptr<FarrellColouredNoiseGenerator> noise_generator_;

    Eigen::Vector3d environment_min_;

    double dx_, dy_; // grid point spacing in x/y direction
    Eigen::Array3d delta_grid; // (dx, dy, dz), dz has no meaning

    Eigen::ArrayXXd u_, v_; // wind velocity field in x/y direction

    Eigen::Array<double, 8, 1> corner_means_;
    Eigen::ArrayXd ramp_x_, ramp_y_;
};

} // namespace gaden2

#endif // GADEN2_WIND_MODEL_FARRELL_HPP_INCLUDED
