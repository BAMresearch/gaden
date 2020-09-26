#ifndef GADEN2_HELPERS_INTERPOLATION_HPP_INCLUDED
#define GADEN2_HELPERS_INTERPOLATION_HPP_INCLUDED

#include <Eigen/Core>

namespace gaden2 {

/**
 * @brief interpolateBilinear
 * Given a function f and its values at the four points
 * P11=(x1,y1), P12=(x1,y2), P21(x2,y1), P22(x2,y2),
 * i.e. f11=f(P11) etc., interpolateBilinear returns
 * the interpolated function value f(p) at p=(x,y)
 * with x1<x2, x1<=x<=x2, y1<y2 and y1<=y<=y2.
 */
double interpolateBilinear(const Eigen::Vector2d &p,
                           const Eigen::Vector2d &P11,
                           const Eigen::Vector2d &P22,
                           double f11, double f12,
                           double f21, double f22);

} // namespace gaden2

#endif // GADEN2_HELPERS_INTERPOLATION_HPP_INCLUDED
