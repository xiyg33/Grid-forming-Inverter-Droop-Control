#ifndef INV_DROOP_CONTROLLERS_HPP
#define INV_DROOP_CONTROLLERS_HPP

#include "config.hpp"
#include "control_blocks.hpp"
#include "math_utils.hpp"

namespace inv_droop {

class GflPll {
private:
    PIController pi_;
    double phase_;
    double delta_w_rad_s_;

public:
    GflPll()
        : pi_(config::kPllKp,
              config::kPllKi,
              config::kGflWPuMin * config::kOmegaBaseRadS,
              config::kGflWPuMax * config::kOmegaBaseRadS,
              config::kGflErrDeadband,
              config::kGflAwKb),
          phase_(0.0),
          delta_w_rad_s_(0.0)
    {
    }

    double step(double vcq_gfl, double Ts)
    {
        // PLL output is absolute electrical angle; PI output is delta omega.
        delta_w_rad_s_ = pi_.step(vcq_gfl, Ts);
        const double w = config::kOmegaBaseRadS + delta_w_rad_s_;
        phase_ = wrapTo2Pi(phase_ + w * Ts);
        return phase_;
    }

    void reset()
    {
        pi_.reset();
        phase_ = 0.0;
        delta_w_rad_s_ = 0.0;
    }
};

class PresynController {
private:
    PIController pi_;

public:
    PresynController()
        : pi_(config::kPllKp,
              config::kPllKi,
              config::kPresynWPuMin * config::kOmegaBaseRadS,
              config::kPresynWPuMax * config::kOmegaBaseRadS,
              config::kPresynErrDeadband,
              config::kPresynAwKb)
    {
    }

    double step(double vcq_presyn, bool enabled, double Ts)
    {
        if (!enabled) {
            // Reset avoids carry-over when presync window is inactive.
            pi_.reset();
            return 0.0;
        }

        const double err = -vcq_presyn;
        const double delta_w_rad_s = pi_.step(err, Ts);
        return delta_w_rad_s / config::kOmegaBaseRadS;
    }

    void reset()
    {
        pi_.reset();
    }
};

class PowerCalculator {
public:
    // Instantaneous dq power model.
    static void calc(double vd, double vq, double id, double iq, double &p_raw, double &q_raw)
    {
        p_raw = vd * id + vq * iq;
        q_raw = vq * id - vd * iq;
    }
};

class DroopController {
public:
    void step(double pset, double qset, double pe, double qe, double presyn_w_pu,
              double &w_pu, double &e_pu, double &w_total_pu)
    {
        // Frequency and voltage droop with dead-zone/saturation constraints.
        const double delta_w = config::kMp * (pset - pe);
        w_pu = 1.0 + applyDeadZone(delta_w, config::kW0DeadzoneLow, config::kW0DeadzoneHigh);
        e_pu = 1.0 + config::kNq * (qset - qe);

        w_pu = clampValue(w_pu, config::kWPuMin, config::kWPuMax);
        e_pu = clampValue(e_pu, config::kEPuMin, config::kEPuMax);
        w_total_pu = w_pu + presyn_w_pu;
    }
};

}  // namespace inv_droop

#endif
