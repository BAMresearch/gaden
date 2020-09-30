#include <gaden2/environment_model.hpp>
#include <gaden2/filament_model.hpp>
#include <gaden2/gas_source_filament_model.hpp>
#include <gaden2/wind_model.hpp>
#include <gaden2/helpers/ideal_gas.hpp>

#include <algorithm>
#include <stdexcept>

namespace gaden2 {

FilamentGasModel::FilamentGasModel(std::vector<std::shared_ptr<GasSourceFilamentModel> > gas_sources,
                                   std::shared_ptr<gases::GasBase> environment_gas,
                                   rl::Logger parent_logger)
    : GasDispersionModel(parent_logger)
    , gas_sources_(gas_sources)
{
    if (gas_sources_.empty())
        throw std::runtime_error("At least one gas source required");

    double filament_spawn_radius = 1.0; // [m]
        // radius around the source in which the filaments are spawned

    filament_initial_radius_ = 0.1; // [m], R(0) in Farrell's paper
    filament_growth_gamma_ = 0.01; // [m2/s]
        // gamma that controls the rate of growth in Farrell's paper

    double filament_noise_std = 0.1; // [m]
        // Sigma of the white noise added to the filament's position
        // on each iteration

    environment_gas_dynamic_viscosity_ = environment_gas->getDynamicViscosity();

    double gas_mass_density = gas_sources_.at(0)->getGas()->getMassDensity(); // [kg/m3]
    if (!std::all_of(gas_sources_.begin(),
                     gas_sources_.end(),
                     [&](const std::shared_ptr<GasSourceFilamentModel> &gas_source)
                     {
                        return gas_source->getGas()->getMassDensity() == gas_mass_density;
                     }))
        throw std::runtime_error("Currently, all gas source must emit gas with the same mass density");

    gas_density_delta_ = environment_gas->getMassDensity() - gas_mass_density; // [kg/m3]

    // Create random distributions
    filament_spawn_distribution_ = std::normal_distribution<double>(0, filament_spawn_radius / 3.0);
        // Set standard deviation to spawn_radius/3, such that 99.73%
        // of all filaments will spawn within that radius
    filament_stochastic_movement_distribution_ = std::normal_distribution<double>(0, filament_noise_std);
}

void FilamentGasModel::increment(double time_step, double total_sim_time)
{
    (void)total_sim_time;
    addNewFilaments(time_step);
    updateFilamentPositions(time_step);
}

double FilamentGasModel::getConcentrationAt(const Eigen::Vector3d &position)
{
    double concentration = 0; // [mol/m3]

    for (const Filament &filament : filaments_)
    {
//        if (env_model_->hasObstacleBetweenPoints(position, filament.position))
//            continue;
        concentration += filament.getConcentrationAt(position);
    }

    // convert mol/m3 to ppm
    return ideal_gas::getMolarFractionAtNormalConditions(concentration) * 1e6;
}

void FilamentGasModel::addNewFilaments(double time_step)
{
    for (const std::shared_ptr<GasSourceFilamentModel> &gas_source : gas_sources_)
    {
        // TODO Make sure that the number of filaments is met for a period of several seconds?
        unsigned num_filaments = time_step * gas_source->getNumFilamentsPerSecond();

        for (unsigned i = 0; i < num_filaments; ++i)
        {
            // Set position of new filament within the specified radius around the gas source location
            Eigen::Vector3d vec_random = Eigen::Vector3d::NullaryExpr([&]() {
                return filament_spawn_distribution_(random_engine_); });

            Eigen::Vector3d filament_position = gas_source->getPosition() + vec_random;

            filaments_.emplace_back(filament_position,
                                    filament_initial_radius_,
                                    gas_source->getMolPerFilament());
        }
    }
}

void FilamentGasModel::updateFilamentPositions(double time_step)
{
    for (auto it = filaments_.begin(); it != filaments_.end();)
    {
        if (updateFilamentPosition(*it, time_step) == UpdatePositionResult::FilamentVanished)
            it = filaments_.erase(it);
        else
            ++it;
    }
}

//Update the filaments location in the 3D environment
// According to Farrell Filament model, a filament is afected by three components of the wind flow.
// 1. Va (large scale wind) -> Advection (Va) -> Movement of a filament as a whole by wind) -> from CFD
// 2. Vm (middle scale wind)-> Movement of the filament with respect the center of the "plume" -> modeled as white noise
// 3. Vd (small scale wind) -> Difussion or change of the filament shape (growth with time)
// We also consider Gravity and Bouyant Forces given the gas molecular mass
FilamentGasModel::UpdatePositionResult
FilamentGasModel::updateFilamentPosition(Filament &filament, double time_step)
{
    Eigen::Vector3d new_position;

    // 1. Simulate Advection (Va)
    // Large scale wind-eddies -> Movement of a filament as a whole by wind
    // --------------------------------------------------------------------
    new_position = filament.position + time_step * wind_model_->getWindVelocityAt(filament.position);

    if (testAndSetPosition(filament.position, new_position) == UpdatePositionResult::FilamentVanished)
        return UpdatePositionResult::FilamentVanished;

    // 2. Simulate Gravity & Bouyant Force
    // -----------------------------------
    // Stokes' law: Fd = 6 pi µ R v
    // Bouyancy: Fb = rho_air * 4/3 pi R^3
    // Gravity: Fg = rho_gas * 4/3 pi R^3
    // Equilibrium: Fd = Fb - Fg
    //     v = 2/9 (rho_air - rho_gas)/µ g R^2
    static double bouyancy_factor =
            2.0/9.0 *                           // [] *
            constants::g /                      // [m/s2] /
            environment_gas_dynamic_viscosity_; // [kg/(m*s)] = [m2/(kg*s)]

    //double radius_squared = filament.getSquaredRadius() * 0.75e-2; // [m2]
    double radius_squared = filament.getSquaredRadius() * 0.05; // [m2]
        // TODO: magic factor (0.75e-2), there was one in the original GADEN as well
    //double gas_fraction = getMolarFraction(filament.getConcentrationAtCentre()); // []
    double radius = 3.0 * std::sqrt(filament.getSquaredRadius());
    double radius_pow3 = radius*radius*radius;
    double average_concentration = filament.gas_amount / (4/3 * M_PI * radius_pow3);
    double gas_fraction = ideal_gas::getMolarFractionAtNormalConditions(average_concentration);

    double terminal_buoyancy_velocity_z =
            bouyancy_factor *       // [m2/(kg*s)] *
            gas_density_delta_ *    // [kg/m3] *
            radius_squared *        // [m2] *
            gas_fraction;           // [] = [m/s]

    Eigen::Vector3d terminal_buoyancy_velocity(
                0,
                0,
                terminal_buoyancy_velocity_z);

    new_position = filament.position + terminal_buoyancy_velocity * time_step;

    if (testAndSetPosition(filament.position, new_position) == UpdatePositionResult::FilamentVanished)
        return UpdatePositionResult::FilamentVanished;

    // 3. Add some variability (stochastic process)
    // Vm (middle scale wind)-> Movement of the filament with respect to the
    // center of the "plume" -> modeled as Gaussian white noise
    // ---------------------------------------------------------------------
    Eigen::Vector3d vec_random = Eigen::Vector3d::NullaryExpr([&]() {
        return filament_stochastic_movement_distribution_(random_engine_); });

    new_position = filament.position + vec_random;

    if (testAndSetPosition(filament.position, new_position) == UpdatePositionResult::FilamentVanished)
        return UpdatePositionResult::FilamentVanished;

    // 4. Filament growth with time (this affects the posterior estimation of
    //                               gas concentration at each cell)
    // Vd (small scale wind eddies) --> Difussion or change of the filament
    //                                  shape (growth with time)
    // R = sigma of a 3D gaussian --> Increasing sigma with time
    // ------------------------------------------------------------------------
    double radius_squared_delta = filament_growth_gamma_ * time_step; // m2/s * s = m2
    filament.addToSquaredRadius(radius_squared_delta);

    return UpdatePositionResult::Okay;
}

FilamentGasModel::UpdatePositionResult
FilamentGasModel::testAndSetPosition(Eigen::Vector3d &position, const Eigen::Vector3d &candidate)
{
    switch (environment_model_->getOccupancy(candidate))
    {
    case Occupancy::Free:
        // Free and valid location... update filament position
        position = candidate;
        return UpdatePositionResult::Okay;
    case Occupancy::Outlet:
    case Occupancy::OutOfWorld:
        // The location corresponds to an outlet! Delete filament!
        //logger.info() << "Filament reached outlet at " << toString(candidate);
        return UpdatePositionResult::FilamentVanished;
    default:
        // The location falls in an obstacle --> illegal movement, do not apply it
        return UpdatePositionResult::Okay;
    }
}

} // namespace gaden2
