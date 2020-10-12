#ifndef GADEN2_SIMULATION_ELEMENT_HPP_INCLUDED
#define GADEN2_SIMULATION_ELEMENT_HPP_INCLUDED

#include <functional>
#include <map>
#include <string>
#include <queue>

namespace gaden2 {

class SimulationElement
{
public:
    SimulationElement();
    virtual ~SimulationElement() {}

    unsigned addPreIncrementCallback(std::function<void ()> callback);
    unsigned addPostIncrementCallback(std::function<void ()> callback);
    void removeCallback(unsigned id);

    void increment(double time_step, double total_sim_time);

    virtual void startRecord(const std::string &file) = 0;
    virtual void stopRecord() = 0;

protected:
    virtual void performIncrement(double time_step, double total_sim_time) = 0;

private:
    unsigned getNextId();

    std::map<unsigned, std::function<void ()>> pre_increment_callbacks_;
    std::map<unsigned, std::function<void ()>> post_increment_callbacks_;
    unsigned next_id_;
    std::queue<unsigned> unused_ids_;
};

} // namespace gaden2

#endif // GADEN2_SIMULATION_ELEMENT_HPP_INCLUDED
