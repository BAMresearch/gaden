#include <gaden2/helpers/interpolation.hpp>

namespace gaden2 {

double interpolateBilinear(const Eigen::Vector2d &p,
                           const Eigen::Vector2d &P11,
                           const Eigen::Vector2d &P22,
                           double f11, double f12,
                           double f21, double f22)
{
    // linear interpolation in x-direction
    double x2_x1_diff_inv = 1.0 / (P22[0] - P11[0]); // 1/(x2 - x1)
    double x_factor1 = (P22[0] - p[0]) * x2_x1_diff_inv; // (x2 - x)/(x2 - x1)
    double x_factor2 = (p[0] - P11[0]) * x2_x1_diff_inv; // (x - x1)/(x2 - x1)
    double f_x_y1 = x_factor1 * f11 + x_factor2 * f21;
    double f_x_y2 = x_factor1 * f12 + x_factor2 * f22;

    // interpolate in y-direction
    double y2_y1_diff_inv = 1.0 / (P22[1] - P11[1]); // 1/(y2 - y1)
    double y_factor1 = (P22[1] - p[1]) * y2_y1_diff_inv; // (y2 - y)/(y2 - y1)
    double y_factor2 = (p[1] - P11[1]) * y2_y1_diff_inv; // (y - y1)/(y2 - y1)
    return y_factor1 * f_x_y1 + y_factor2 * f_x_y2;
}

} // namespace gaden2
