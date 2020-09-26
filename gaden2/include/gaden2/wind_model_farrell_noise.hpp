#ifndef GADEN2_WIND_MODEL_FARRELL_NOISE_HPP_INCLUDED
#define GADEN2_WIND_MODEL_FARRELL_NOISE_HPP_INCLUDED

#include <Eigen/Core>

#include <random>

namespace gaden2 {

class FarrellColouredNoiseGenerator
{
public:
    FarrellColouredNoiseGenerator(const Eigen::Matrix2Xd &init_state,
                                  double damping = 0.1,
                                  double bandwidth = 0.2,
                                  double gain = 1.0,
                                  bool use_original_farrell_updates = false);

    void update(double dt);

    Eigen::ArrayXd getNoise() const;

private:
    std::default_random_engine random_engine_;
    std::normal_distribution<double> distribution_;

    Eigen::Matrix2d A;
    Eigen::Vector2d B;

    Eigen::Matrix2Xd state;

    bool use_original_farrell_updates_;
};

} // namespace gaden2

#endif // GADEN2_WIND_MODEL_FARRELL_NOISE_HPP_INCLUDED
