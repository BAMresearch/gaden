#include <gaden2/gas_source_filament_model.hpp>
#include <gaden2/gases.hpp>

namespace gaden2 {

GasSourceFilamentModel::GasSourceFilamentModel(Eigen::Vector3d position,
                                               std::shared_ptr<gases::GasBase> gas,
                                               double release_rate,
                                               unsigned num_filaments_per_second,
                                               double filament_initial_radius,
                                               double filament_spawn_radius)
    : GasSource(position, gas, release_rate)
    , num_filaments_per_second_(num_filaments_per_second)
    , filament_initial_radius_(filament_initial_radius)
    , num_filaments_release_fraction_(0.0)
{
    double molar_release_rate = release_rate_           // [kg/h] /
                                / gas_->getMolarMass()  // [kg/mol] /
                                / 3600.0;               // [s/h] = [mol/s]

    mol_per_filament_ = molar_release_rate / num_filaments_per_second_; // [mol/s] / [1/s] = [mol]

    // Create random distributions
    filament_spawn_distribution_ = std::normal_distribution<double>(0, filament_spawn_radius / 3.0);
        // Set standard deviation to spawn_radius/3, such that 99.73%
        // of all filaments will spawn within that radius
}

} // namespace gaden2
