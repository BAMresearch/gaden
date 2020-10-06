#ifndef GADEN2_SIMULATION_ELEMENT_HPP_INCLUDED
#define GADEN2_SIMULATION_ELEMENT_HPP_INCLUDED

#include <string>

namespace gaden2 {

class SimulationElement
{
public:
    SimulationElement() {}
    virtual ~SimulationElement() {}

    virtual void increment(double time_step, double total_sim_time) = 0;

    virtual void startRecord(const std::string &file) = 0;
    virtual void stopRecord() = 0;
};

} // namespace gaden2

#endif // GADEN2_SIMULATION_ELEMENT_HPP_INCLUDED
