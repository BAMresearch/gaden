#ifndef GADEN2_FILAMENT_HPP_INCLUDED
#define GADEN2_FILAMENT_HPP_INCLUDED

#include <cmath>
#include <Eigen/Core>

namespace gaden2 {

static constexpr double PI_POW3 = M_PI * M_PI * M_PI;
static const double SQRT_8_PI_POW3_INV = 1.0 / std::sqrt(8.0 * PI_POW3);

class Filament
{
public:
    Filament(Eigen::Vector3d initial_position,
             double initial_radius,
             double gas_amount_mol)
        : position(initial_position)
        , gas_amount(gas_amount_mol)
        , radius_pow2_(initial_radius * initial_radius)
        , concentration_factor_(gas_amount_mol * SQRT_8_PI_POW3_INV)
    {
        updateRadius();
    }

    inline double getSquaredRadius() const { return radius_pow2_; }
    inline double getSquared3SigmaRadius() const { return radius_pow2_x_9_; }

    inline void addToSquaredRadius(double delta_radius_pow2)
    {
        radius_pow2_ += delta_radius_pow2;
        updateRadius();
    }

    inline double getConcentrationAt(const Eigen::Vector3d &x) const // returns [mol/m3]
    {
        double distance_pow2 = (x - position).squaredNorm();
        // Neglect influence out of the 3 sigma range
        if (distance_pow2 > radius_pow2_x_9_) return 0;
        return buffered_concentration_factor_ * std::exp(-distance_pow2 * radius_pow2_inv_);
    }

    inline double getConcentrationUncheckedAt(const Eigen::Vector3d &x) const // returns [mol/m3]
    {
        // Same as getConcentrationAt, but without checking the 3 sigma range.
        double distance_pow2 = (x - position).squaredNorm();
        return buffered_concentration_factor_ * std::exp(-distance_pow2 * radius_pow2_inv_);
    }

    inline double getConcentrationAtCentre() const // returns [mol/m3]
    {
        return buffered_concentration_factor_;
    }

    Eigen::Vector3d position; // [m] center of the filament
    double gas_amount; // [mol]

private:
    inline void updateRadius()
    {
        radius_pow2_x_9_ = 9.0 * radius_pow2_;
        radius_pow2_inv_ = 1.0 / radius_pow2_;
        double radius_inverse = std::sqrt(radius_pow2_inv_);
        double radius_pow3_inv = radius_pow2_inv_ * radius_inverse;
        buffered_concentration_factor_ = concentration_factor_ * radius_pow3_inv;
    }

    double radius_pow2_; // [m2] Controls the size of the filament,
                         // R^2(t) in Farrell's paper (sigma of a 3D gaussian)
    double radius_pow2_x_9_; // (3*R)^2 = 9*R^2
    double radius_pow2_inv_; // 1/R^2
    double concentration_factor_; // [mol] Q / sqrt(8 * pi^3), with Q being
                                  // the amount of gas in the filament
    double buffered_concentration_factor_; // [mol/m3], Q / sqrt(8 * pi^3) / R^3
};

} // namespace gaden2

#endif // GADEN2_FILAMENT_HPP_INCLUDED
