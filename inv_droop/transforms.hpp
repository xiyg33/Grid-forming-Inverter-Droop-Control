#ifndef INV_DROOP_TRANSFORMS_HPP
#define INV_DROOP_TRANSFORMS_HPP

#include <cmath>

namespace inv_droop {

class MathTransform {
public:
    // Park transform aligned with existing model convention.
    static void abc2dq(double a, double b, double c, double wt, double &d, double &q)
    {
        const double sqrt3_over_2 = std::sqrt(3.0) / 2.0;
        const double alpha = (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c);
        const double beta = (2.0 / 3.0) * (sqrt3_over_2 * b - sqrt3_over_2 * c);

        const double cos_wt = std::cos(wt);
        const double sin_wt = std::sin(wt);

        d = alpha * sin_wt - beta * cos_wt;
        q = alpha * cos_wt + beta * sin_wt;
    }

    // Inverse transform matching abc2dq convention above.
    static void dq2abc(double d, double q, double wt, double &a, double &b, double &c)
    {
        const double sqrt3_over_2 = std::sqrt(3.0) / 2.0;
        const double cos_wt = std::cos(wt);
        const double sin_wt = std::sin(wt);

        const double alpha = d * sin_wt + q * cos_wt;
        const double beta = -d * cos_wt + q * sin_wt;

        a = alpha;
        b = -0.5 * alpha + sqrt3_over_2 * beta;
        c = -0.5 * alpha - sqrt3_over_2 * beta;
    }
};

}  // namespace inv_droop

#endif
