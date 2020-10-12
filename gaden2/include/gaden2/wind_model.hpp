#ifndef GADEN2_WIND_MODEL_HPP_INCLUDED
#define GADEN2_WIND_MODEL_HPP_INCLUDED

#include "simulation_element.hpp"

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

namespace gaden2 {

class WindModel : public SimulationElement
{
public:
    WindModel(rl::Logger &parent_logger)
        : logger(parent_logger.getChild("WindModel"))
    {}

    virtual ~WindModel() {}

    virtual Eigen::Vector3d getWindVelocityAt(const Eigen::Vector3d &position) = 0;

    virtual Eigen::Vector3d getEnvironmentMin() const = 0;
    virtual Eigen::Vector3d getEnvironmentMax() const = 0;

protected:
    virtual void performIncrement(double time_step, double total_sim_time)
    {
        (void)time_step;
        (void)total_sim_time;
    }

    rl::Logger logger;
};

} // namespace gaden2

#endif // GADEN2_WIND_MODEL_HPP_INCLUDED
