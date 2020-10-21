// This is a C++ port of the Python code from the
// Insect Robotics Group, University of Edinburgh
// Available at: https://github.com/InsectRobotics/pompy

#include <gaden2/wind_model_farrell_noise.hpp>

namespace gaden2 {

FarrellColouredNoiseGenerator::FarrellColouredNoiseGenerator(const Eigen::Matrix2Xd &init_state,
        double damping,
        double bandwidth,
        double gain,
        bool use_original_farrell_updates)
    : state(init_state)
    , use_original_farrell_updates_(use_original_farrell_updates)
{
    double bandwidth_pow2 = bandwidth * bandwidth;

    A <<               0,                          1,
         -bandwidth_pow2,   -2 * damping * bandwidth;

    B <<                     0,
         gain * bandwidth_pow2;
}

void FarrellColouredNoiseGenerator::update(double dt)
{
    Eigen::RowVectorXd rnd = Eigen::RowVectorXd::NullaryExpr(state.cols(), [&]() {
        return distribution_(random_engine_); });

    if (use_original_farrell_updates_) // apply Farrell et al. (2002) update
    {
        state += dt * (A*state + B*rnd);
    }
    else // apply update with Euler-Maruyama integration
    {
        state += dt * A*state + B*rnd*std::sqrt(dt);
    }
}

Eigen::ArrayXd FarrellColouredNoiseGenerator::getNoise() const
{
    return state.row(0);
}

} // namespace gaden2
