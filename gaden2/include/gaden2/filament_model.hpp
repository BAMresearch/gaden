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
    static constexpr double DEFAULT_FILAMENT_NOISE_STD = 0.1; // [m]
    static constexpr double DEFAULT_FILAMENT_GROWTH_GAMMA = 0.01; // [m2/s]

    static std::shared_ptr<gases::GasBase> getDefaultEnvironmentGas()
    {
        return std::make_shared<gases::Air>();
    }

    FilamentGasModel(std::shared_ptr<EnvironmentModel> environment_model,
                     std::shared_ptr<WindModel> wind_model,
                     std::vector<std::shared_ptr<GasSourceFilamentModel>> gas_sources,
                     double filament_noise_std = DEFAULT_FILAMENT_NOISE_STD, // [m], sigma of the white noise added to the filament's position on each iteration
                     double filament_growth_gamma = DEFAULT_FILAMENT_GROWTH_GAMMA, // [m2/s], gamma that controls the rate of growth in Farrell's paper
                     std::shared_ptr<gases::GasBase> environment_gas = getDefaultEnvironmentGas(),
                     rl::Logger parent_logger = getStandardLogger());

    void startRecord(const std::string &file);
    void stopRecord();

    double getConcentrationAt(const Eigen::Vector3d &position); // returns [ppm]

    const std::list<Filament> & getFilaments() const;

private:
    void performIncrement(double time_step, double total_sim_time);

    void addNewFilaments(double time_step);
    void updateFilamentPositions(double time_step);

    enum class UpdatePositionResult { KeepFilament, RemoveFilament };
    UpdatePositionResult updateFilamentPosition(Filament &filament, double time_step);
    UpdatePositionResult testAndSetPosition(Eigen::Vector3d &position, const Eigen::Vector3d &candidate);

    std::shared_ptr<EnvironmentModel> environment_model_;
    std::shared_ptr<WindModel> wind_model_;
    std::vector<std::shared_ptr<GasSourceFilamentModel>> gas_sources_;

    // model parameters
    double filament_growth_gamma_;              // [m2/s]
    double gas_density_delta_;                  // [kg/m3]
    double environment_gas_dynamic_viscosity_;  // [kg/(m*s)] = [Pa s]

    // random
    //std::mt19937 random_engine_;
    std::default_random_engine random_engine_;
    std::normal_distribution<double> filament_stochastic_movement_distribution_;

    std::list<Filament> filaments_;
};

} // namespace gaden2

#endif // GADEN2_FILAMENT_MODEL_HPP_INCLUDED
