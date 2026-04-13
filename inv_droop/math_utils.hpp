#ifndef INV_DROOP_MATH_UTILS_HPP
#define INV_DROOP_MATH_UTILS_HPP

#include <cmath>

#include "config.hpp"

namespace inv_droop {

// Saturation helper reused by controllers.
inline double clampValue(double x, double x_min, double x_max)
{
    if (x > x_max) return x_max;
    if (x < x_min) return x_min;
    return x;
}

// Wrap angle to [0, 2*pi).
inline double wrapTo2Pi(double x)
{
    x = std::fmod(x, config::kTwoPi);
    if (x < 0.0) x += config::kTwoPi;
    return x;
}

// Simulink-style dead-zone with shifted output outside the band.
inline double applyDeadZone(double u, double low, double high)
{
    if (u > high) return u - high;
    if (u < low) return u - low;
    return 0.0;
}

}  // namespace inv_droop

#endif
