#ifndef GADEN2_GAS_SOURCE_FILAMENT_MODEL_HPP_INCLUDED
#define GADEN2_GAS_SOURCE_FILAMENT_MODEL_HPP_INCLUDED

#include "gas_source.hpp"

namespace gaden2 {

class GasSourceFilamentModel : public GasSource
{
public:
    GasSourceFilamentModel(Eigen::Vector3d position,
                           std::shared_ptr<gases::GasBase> gas,
                           double release_rate,
                           unsigned num_filaments_per_second,
                           double mol_per_filament);

    inline unsigned getNumFilamentsPerSecond() const { return  num_filaments_per_second_; }
    inline double getMolPerFilament() const { return mol_per_filament_; }

private:
    unsigned num_filaments_per_second_; // [1/s]
    double mol_per_filament_; // [mol]
};

} // namespace gaden2

#endif // GADEN2_GAS_SOURCE_FILAMENT_MODEL_HPP_INCLUDED
