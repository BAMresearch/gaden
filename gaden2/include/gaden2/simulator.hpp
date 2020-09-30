#ifndef GADEN2_SIMULATOR_HPP_INCLUDED
#define GADEN2_SIMULATOR_HPP_INCLUDED

#include "logger.hpp"

#include <memory>
#include <string>

namespace gaden2 {

class GasDispersionModel;

class Simulator
{
public:
    static constexpr double DEFAULT_DT = -1.0;

    Simulator(std::shared_ptr<GasDispersionModel> dispersion_model,
              double dt = DEFAULT_DT, // dt <= 0 --> dt must be passed to each call of increment
              rl::Logger logger = getStandardLogger());

    /**
     * @brief Load recorded simulation
     * @param file
     * @param logger
     */
    Simulator(const std::string &file,
              rl::Logger logger = getStandardLogger());

    ~Simulator();

    rl::Logger & getLogger();

    void increment(double dt);
    double increment();

    void startRecord(const std::string &file);
    void stopRecord();

private:
    enum class Mode { Simulate, PlayRecord };

    void performSimulationIncrement(double dt);

    rl::Logger logger_;

    Mode mode_;

    double dt_;
    double t_sim_;

    std::shared_ptr<GasDispersionModel> dispersion_model_;
    bool recording_;
};

} // namespace gaden2

#endif // GADEN2_SIMULATOR_HPP_INCLUDED
