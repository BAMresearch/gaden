#include <gaden2/simulation_element.hpp>

namespace gaden2 {

SimulationElement::SimulationElement()
    : next_id_(0)
{}

void SimulationElement::increment(double time_step, double total_sim_time)
{
    for (auto &item : pre_increment_callbacks_)
        item.second();

    performIncrement(time_step, total_sim_time);

    for (auto &item : post_increment_callbacks_)
        item.second();
}

unsigned SimulationElement::addPreIncrementCallback(std::function<void ()> callback)
{
    unsigned id = getNextId();
    pre_increment_callbacks_.emplace(id, callback);
    return id;
}

unsigned SimulationElement::addPostIncrementCallback(std::function<void ()> callback)
{
    unsigned id = getNextId();
    post_increment_callbacks_.emplace(id, callback);
    return id;
}

void SimulationElement::removeCallback(unsigned int id)
{
    pre_increment_callbacks_.erase(id);
    post_increment_callbacks_.erase(id);
    unused_ids_.push(id);
}

unsigned SimulationElement::getNextId()
{
    if (unused_ids_.empty())
        return next_id_++;
    else
    {
        unsigned id = unused_ids_.front();
        unused_ids_.pop();
        return id;
    }
}

} // namespace gaden2
