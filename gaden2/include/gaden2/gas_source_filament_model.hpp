#ifndef GADEN2_GAS_SOURCE_FILAMENT_MODEL_HPP_INCLUDED
#define GADEN2_GAS_SOURCE_FILAMENT_MODEL_HPP_INCLUDED

#include "gas_source.hpp"

#include <random>

namespace gaden2 {

class GasSourceFilamentModel : public GasSource
{
    friend class FilamentGasModel;
public:
    static constexpr unsigned DEFAULT_NUM_FILAMENTS_PER_SECOND = 10;
    static constexpr double DEFAULT_FILAMENT_INITIAL_RADIUS = 0.1; // [m]
    static constexpr double DEFAULT_FILAMENT_SPAWN_RADIUS = 0.1; // [m]

    // TODO Adjust Python export
    GasSourceFilamentModel(Eigen::Vector3d position,
                           std::shared_ptr<gases::GasBase> gas,
                           double release_rate, // [kg/h]
                           unsigned num_filaments_per_second = DEFAULT_NUM_FILAMENTS_PER_SECOND, // [1/s]
                           double filament_initial_radius = DEFAULT_FILAMENT_INITIAL_RADIUS, // [m], R(0) in Farrell's paper
                           double filament_spawn_radius = DEFAULT_FILAMENT_SPAWN_RADIUS); // [m], radius around the source in which the filaments are spawned

private:
    unsigned num_filaments_per_second_; // [1/s]
    double mol_per_filament_; // [mol]

    double filament_initial_radius_;
    std::normal_distribution<double> filament_spawn_distribution_;

    double num_filaments_release_fraction_;
        // used to ensure that the correct amount of filaments is released in the long term
};

} // namespace gaden2

#endif // GADEN2_GAS_SOURCE_FILAMENT_MODEL_HPP_INCLUDED
