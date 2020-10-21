#include <gaden2/simulation_element.hpp>
#include <gaden2/simulator.hpp>

#include <stdexcept>

namespace gaden2 {

Simulator::Simulator(std::shared_ptr<SimulationElement> simulation_element,
                     double dt,
                     rl::Logger logger)
    : logger_(logger)
    , mode_(Mode::Simulate)
    , dt_(dt)
    , t_sim_(0.0)
    , simulation_element_(simulation_element)
{
    logger_.info("Created simulator");
}

Simulator::Simulator(const std::string &file,
                     rl::Logger logger)
    : logger_(logger)
    , mode_(Mode::PlayRecord)
    , t_sim_(0.0)
{
    logger_.info() << "Loading recorded simulation: " << file;
}

Simulator::~Simulator()
{
    logger_.info("Destructing simulator");
}

rl::Logger & Simulator::getLogger()
{
    return logger_;
}

void Simulator::increment(double dt)
{
    if (dt <= 0)
        throw std::invalid_argument("dt must be positive");

    if (mode_ == Mode::Simulate)
    {
        performSimulationIncrement(dt);
    }
    else
    {
        throw std::runtime_error("Playing recorded simulations not supported yet");
    }
}

double Simulator::increment()
{
    if (mode_ == Mode::Simulate)
    {
        if (dt_ <= 0)
            throw std::runtime_error("No valid default dt set for increment(), so increment(dt) must be called");
        performSimulationIncrement(dt_);
        return dt_;
    }
    else
    {
        return 0;
    }
}

void Simulator::startRecord(const std::string &file)
{
    recording_ = true;
}

void Simulator::stopRecord()
{
    recording_ = false;
    // TODO close file handle
}

void Simulator::performSimulationIncrement(double dt)
{
    // caller must make sure that dt > 0
    t_sim_ += dt;
    simulation_element_->increment(dt, t_sim_);
}

} // namespace gaden2
