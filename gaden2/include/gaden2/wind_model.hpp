#ifndef GADEN2_WIND_MODEL_HPP_INCLUDED
#define GADEN2_WIND_MODEL_HPP_INCLUDED

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

namespace gaden2 {

class WindModel
{
public:
    WindModel(rl::Logger &parent_logger)
        : logger(parent_logger.getChild("WindModel"))
    {}

    virtual ~WindModel() {}

    virtual void increment(double time_step, double total_sim_time) = 0;

    virtual Eigen::Vector3d getWindVelocityAt(const Eigen::Vector3d &position) = 0;

protected:
    rl::Logger logger;
};

} // namespace gaden2

#endif // GADEN2_WIND_MODEL_HPP_INCLUDED
