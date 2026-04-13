#ifndef INV_DROOP_CONTROL_BLOCKS_HPP
#define INV_DROOP_CONTROL_BLOCKS_HPP

#include <cmath>

#include "config.hpp"
#include "math_utils.hpp"

namespace inv_droop {

class PIController {
private:
    double kp_;
    double ki_;
    double out_min_;
    double out_max_;
    double err_deadband_;
    double aw_kb_;
    double integral_state_;

public:
    PIController(double kp, double ki, double out_min = -1.0e30, double out_max = 1.0e30,
                 double err_deadband = 0.0, double aw_kb = 0.0)
        : kp_(kp),
          ki_(ki),
          out_min_(out_min),
          out_max_(out_max),
          err_deadband_(err_deadband),
          aw_kb_(aw_kb),
          integral_state_(0.0)
    {
    }

    double step(double err, double Ts)
    {
        // Back-calculation anti-windup PI.
        const double e = (std::fabs(err) < err_deadband_) ? 0.0 : err;
        const double p_out = kp_ * e;
        const double u_unsat = p_out + integral_state_;
        const double u_sat = clampValue(u_unsat, out_min_, out_max_);
        const double aw_term = aw_kb_ * (u_sat - u_unsat);

        integral_state_ += (ki_ * e + aw_term) * Ts;

        const double u_next_unsat = p_out + integral_state_;
        return clampValue(u_next_unsat, out_min_, out_max_);
    }

    void reset()
    {
        integral_state_ = 0.0;
    }
};

class DiscreteIIR {
private:
    double b_[3];
    double a_[3];
    double x_hist_[2];
    double y_hist_[2];

public:
    DiscreteIIR(const double num[3], const double den[3])
    {
        for (int i = 0; i < 3; ++i) {
            b_[i] = num[i];
            a_[i] = den[i];
        }
        reset();
    }

    double step(double x)
    {
        // Direct-form I realization with two-sample history.
        const double y = b_[0] * x
                       + b_[1] * x_hist_[0]
                       + b_[2] * x_hist_[1]
                       - a_[1] * y_hist_[0]
                       - a_[2] * y_hist_[1];

        x_hist_[1] = x_hist_[0];
        x_hist_[0] = x;
        y_hist_[1] = y_hist_[0];
        y_hist_[0] = y;

        return y;
    }

    void reset()
    {
        x_hist_[0] = 0.0;
        x_hist_[1] = 0.0;
        y_hist_[0] = 0.0;
        y_hist_[1] = 0.0;
    }
};

class PhaseGenerator {
private:
    long double phase_unwrapped_;
    long double sum_err_;

public:
    PhaseGenerator()
        : phase_unwrapped_(static_cast<long double>(config::kWtGfmInitRad)),
          sum_err_(0.0L)
    {
    }

    double step(double w_total_pu, double Ts)
    {
        // Kahan summation limits long-run phase accumulation drift.
        const long double delta = static_cast<long double>(w_total_pu)
                                * static_cast<long double>(config::kOmegaBaseRadS)
                                * static_cast<long double>(Ts);
        const long double y = delta - sum_err_;
        const long double t = phase_unwrapped_ + y;
        sum_err_ = (t - phase_unwrapped_) - y;
        phase_unwrapped_ = t;

        return wrapTo2Pi(static_cast<double>(phase_unwrapped_));
    }

    void reset()
    {
        phase_unwrapped_ = static_cast<long double>(config::kWtGfmInitRad);
        sum_err_ = 0.0L;
    }
};

}  // namespace inv_droop

#endif
