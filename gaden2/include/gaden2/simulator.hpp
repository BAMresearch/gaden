#ifndef GADEN2_SIMULATOR_HPP_INCLUDED
#define GADEN2_SIMULATOR_HPP_INCLUDED

#include "logger.hpp"

#include <string>

namespace gaden2 {

class Simulator
{
public:
    Simulator(rl::Logger logger = getStandardLogger());
    ~Simulator();

    rl::Logger & getLogger();
    std::string getName();

private:
    rl::Logger logger_;
};

} // namespace gaden2

#endif // GADEN2_SIMULATOR_HPP_INCLUDED
