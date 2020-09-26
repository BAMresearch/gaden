#include <gaden2/simulator.hpp>

namespace gaden2 {

Simulator::Simulator(rl::Logger logger)
    : logger_(logger)
{
    logger_.info("Created simulator");
}

Simulator::~Simulator()
{
    logger_.info("Destructing simulator");
}

rl::Logger & Simulator::getLogger()
{
    return logger_;
}

std::string Simulator::getName()
{
    return "Gaden2Simulator";
}

} // namespace gaden2
