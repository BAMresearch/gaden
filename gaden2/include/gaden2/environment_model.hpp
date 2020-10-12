#ifndef GADEN2_ENVIRONMENT_MODEL_HPP_INCLUDED
#define GADEN2_ENVIRONMENT_MODEL_HPP_INCLUDED

#include "simulation_element.hpp"

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

namespace gaden2 {

enum class Occupancy : int32_t { Free = 0, Occupied = 1, Outlet = 2, OutOfWorld = 3 };

class EnvironmentModel : public SimulationElement
{
public:
    EnvironmentModel(rl::Logger parent_logger)
        : logger(parent_logger.getChild("EnvironmentModel"))
    {}

    virtual ~EnvironmentModel() {}

    virtual Eigen::Vector3d getEnvironmentMin() const = 0;
    virtual Eigen::Vector3d getEnvironmentMax() const = 0;

    virtual Occupancy getOccupancy(const Eigen::Vector3d &p) const = 0;

protected:
    virtual void performIncrement(double time_step, double total_sim_time)
    {
        (void)time_step;
        (void)total_sim_time;
    }

    rl::Logger logger;
};

} // namespace gaden2

#endif // GADEN2_ENVIRONMENT_MODEL_HPP_INCLUDED
