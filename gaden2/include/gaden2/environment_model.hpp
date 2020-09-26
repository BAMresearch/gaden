#ifndef GADEN2_ENVIRONMENT_MODEL_HPP_INCLUDED
#define GADEN2_ENVIRONMENT_MODEL_HPP_INCLUDED

#include <Eigen/Core>
#include <rl_logging/logging_interface.hpp>

namespace gaden2 {

class EnvironmentModel
{
public:
    EnvironmentModel(rl::Logger parent_logger)
        : logger(parent_logger.getChild("EnvironmentModel"))
    {}

    virtual ~EnvironmentModel() {}

    virtual Eigen::Vector3d getEnvironmentMin() const = 0;
    virtual Eigen::Vector3d getEnvironmentMax() const = 0;

protected:
    rl::Logger logger;
};

} // namespace gaden2

#endif // GADEN2_ENVIRONMENT_MODEL_HPP_INCLUDED
