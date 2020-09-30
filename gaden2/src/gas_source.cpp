#include <gaden2/gas_source.hpp>
#include <gaden2/gases.hpp>

namespace gaden2 {

GasSource::GasSource(Eigen::Vector3d position,
                     std::shared_ptr<gases::GasBase> gas,
                     double release_rate)
    : position_(position)
    , gas_(gas)
    , release_rate_(release_rate)
{}

} // namespace gaden2
