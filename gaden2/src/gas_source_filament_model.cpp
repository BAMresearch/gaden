#include <gaden2/gas_source_filament_model.hpp>

namespace gaden2 {

GasSourceFilamentModel::GasSourceFilamentModel(Eigen::Vector3d position,
                                               std::shared_ptr<gases::GasBase> gas,
                                               double release_rate,
                                               unsigned num_filaments_per_second,
                                               double mol_per_filament)
    : GasSource(position, gas, release_rate)
    , num_filaments_per_second_(num_filaments_per_second)
    , mol_per_filament_(mol_per_filament)
{}

} // namespace gaden2
