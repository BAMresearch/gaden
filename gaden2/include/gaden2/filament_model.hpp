#ifndef GADEN2_FILAMENT_MODEL_HPP_INCLUDED
#define GADEN2_FILAMENT_MODEL_HPP_INCLUDED

#include "filament.hpp"
#include "gas_dispersion_model.hpp"
#include "gases.hpp"
#include "logger.hpp"

#include <list>
#include <memory>
#include <random>
#include <vector>

namespace gaden2 {

class EnvironmentModel;
class GasSourceFilamentModel;
class WindModel;

class FilamentGasModel : public GasDispersionModel
{
public:
    static std::shared_ptr<gases::GasBase> getDefaultEnvironmentGas()
    {
        return std::make_shared<gases::Air>();
    }

    // TODO Add all parameters
    // TODO Adjust Python export
    FilamentGasModel(std::vector<std::shared_ptr<GasSourceFilamentModel>> gas_sources,
                     std::shared_ptr<gases::GasBase> environment_gas = getDefaultEnvironmentGas(),
                     rl::Logger parent_logger = getStandardLogger());

    void increment(double time_step, double total_sim_time);

    double getConcentrationAt(const Eigen::Vector3d &position); // returns [ppm]

private:
    void addNewFilaments(double time_step);
    void updateFilamentPositions(double time_step);

    enum class UpdatePositionResult { Okay, FilamentVanished };
    UpdatePositionResult updateFilamentPosition(Filament &filament, double time_step);
    UpdatePositionResult testAndSetPosition(Eigen::Vector3d &position, const Eigen::Vector3d &candidate);

    // random
    //std::mt19937 random_engine_;
    std::default_random_engine random_engine_;
    std::normal_distribution<double> filament_spawn_distribution_;
    std::normal_distribution<double> filament_stochastic_movement_distribution_;

    // configuration parameters
    double filament_initial_radius_;    // [m]
    double filament_growth_gamma_;      // [m2/s]
    //double gas_density_factor_;         // [kg/m3]
    double gas_density_delta_;          // [kg/m3]

    // derived parameters
    double environment_gas_dynamic_viscosity_;

    std::shared_ptr<EnvironmentModel> environment_model_;
    std::shared_ptr<WindModel> wind_model_;
    std::vector<std::shared_ptr<GasSourceFilamentModel>> gas_sources_;
    std::list<Filament> filaments_;
};

} // namespace gaden2

#endif // GADEN2_FILAMENT_MODEL_HPP_INCLUDED
