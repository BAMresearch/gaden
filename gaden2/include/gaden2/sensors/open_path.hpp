#ifndef GADEN2_SENSORS_OPEN_PATH_HPP_INCLUDED
#define GADEN2_SENSORS_OPEN_PATH_HPP_INCLUDED

#include <rl_logging/numbered_instance.hpp>

#include <memory>

namespace gaden2 {

class Simulator;

namespace sensors {

class OpenPath : public rl::logging::NumberedInstance
{
public:
    OpenPath(std::shared_ptr<gaden2::Simulator> simulator);
    ~OpenPath();

private:
    std::shared_ptr<gaden2::Simulator> simulator_;
};

}} // namespace gaden2::sensors

#endif // GADEN2_SENSORS_OPEN_PATH_HPP_INCLUDED
