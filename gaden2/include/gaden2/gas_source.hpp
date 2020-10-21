#ifndef GADEN2_GAS_SOURCE_HPP_INCLUDED
#define GADEN2_GAS_SOURCE_HPP_INCLUDED

#include <Eigen/Core>

#include <memory>

namespace gaden2 {

namespace gases {
class GasBase;
}

class GasSource
{
public:
    GasSource(Eigen::Vector3d position,
              std::shared_ptr<gases::GasBase> gas,
              double release_rate); // [kg/h]

    virtual ~GasSource() {}

    inline const Eigen::Vector3d & getPosition() const { return  position_; }
    inline const std::shared_ptr<gases::GasBase> & getGas() const { return gas_; }
    inline double getReleaseRate() const { return release_rate_; }

protected:
    Eigen::Vector3d position_; // [m, m, m]
    std::shared_ptr<gases::GasBase> gas_;
    double release_rate_; // [kg/h]

    //bool variable_release_rate_;
    // If false, the release rate of the gas source will be used;
    // if true, a poisson process is used with the release rate of the
    // gas source as the distribution's lambda parameter
};

} // namespace gaden2

#endif // GADEN2_GAS_SOURCE_HPP_INCLUDED
