#ifndef INV_DROOP_INVERTER_CONTROL_HPP
#define INV_DROOP_INVERTER_CONTROL_HPP

#include <cmath>

#include "config.hpp"
#include "control_blocks.hpp"
#include "controllers.hpp"
#include "transforms.hpp"

namespace inv_droop {

class InverterControl {
private:
    // Stage-1 sampled and transformed signals.
    struct MeasuredSignals {
        double vcd;
        double vcq;
        double ild;
        double ilq;
        double iod;
        double ioq;
        double vcq_gfl;
        double vcq_presyn;
    };

    // Stage-2 power path and filtered estimates.
    struct PowerState {
        double p_raw;
        double q_raw;
        double pe;
        double qe;
    };

    // Stage-3 droop and sinusoidal voltage references.
    struct ReferenceState {
        double w_pu;
        double e_pu;
        double w_total_pu;
        double wt_gfm;
        double ed_ref;
        double eq_ref;
    };

    // Stage-4 dual-loop controller outputs.
    struct DualLoopState {
        double id_ref_gfm;
        double iq_ref_gfm;
        double ud_ref;
        double uq_ref;
        double ua_ref;
        double ub_ref;
        double uc_ref;
    };

    GflPll pll_;
    PresynController presyn_;
    PIController vloop_d_;
    PIController vloop_q_;
    PIController iloop_d_;
    PIController iloop_q_;
    double wt_gfl_;
    double wt_gfl_delay_;
    double wt_gfm_delay_;
    unsigned long long step_count_;
    DiscreteIIR pe_lpf_;
    DiscreteIIR qe_lpf_;
    DiscreteIIR vff_d_lpf_;
    DiscreteIIR vff_q_lpf_;
    DroopController droop_;
    PhaseGenerator phase_gfm_;

    bool isPresynActive(double Ts) const
    {
        // Windowed presync scheduling aligned with discrete step counter.
        const double t_now = static_cast<double>(step_count_) * Ts;
        return (t_now >= config::kPresynStartS) && (t_now < config::kPresynEndS);
    }

    MeasuredSignals measureSignals(const double *vgrid_abc,
                                   const double *vpcc_abc,
                                   const double *ilabc,
                                   const double *ioabc,
                                   double Ts)
    {
        // Use delayed angles to match original discrete execution order.
        const double wt_gfm_ud = wt_gfm_delay_;
        const double wt_gfl_ud = wt_gfl_delay_;

        MeasuredSignals signals = {};

        double vgrid_d = 0.0;
        MathTransform::abc2dq(vgrid_abc[0], vgrid_abc[1], vgrid_abc[2], wt_gfl_ud, vgrid_d, signals.vcq_gfl);
        (void)vgrid_d;
        wt_gfl_ = pll_.step(signals.vcq_gfl, Ts);

        double vpcc_d_presyn = 0.0;
        MathTransform::abc2dq(vpcc_abc[0], vpcc_abc[1], vpcc_abc[2], wt_gfl_ud, vpcc_d_presyn, signals.vcq_presyn);
        (void)vpcc_d_presyn;

        MathTransform::abc2dq(vpcc_abc[0], vpcc_abc[1], vpcc_abc[2], wt_gfm_ud, signals.vcd, signals.vcq);
        MathTransform::abc2dq(ilabc[0], ilabc[1], ilabc[2], wt_gfm_ud, signals.ild, signals.ilq);
        MathTransform::abc2dq(ioabc[0], ioabc[1], ioabc[2], wt_gfm_ud, signals.iod, signals.ioq);

        return signals;
    }

    PowerState computePowerState(const MeasuredSignals &signals)
    {
        PowerState power = {};
        PowerCalculator::calc(signals.vcd, signals.vcq, signals.iod, signals.ioq, power.p_raw, power.q_raw);
        power.pe = pe_lpf_.step(power.p_raw);
        power.qe = qe_lpf_.step(power.q_raw);
        return power;
    }

    ReferenceState computeReferenceState(const MeasuredSignals &signals,
                                         const PowerState &power,
                                         double pset,
                                         double qset,
                                         double Ts)
    {
        ReferenceState refs = {};

        const double presyn_w_pu = presyn_.step(signals.vcq_presyn, isPresynActive(Ts), Ts);
        droop_.step(pset, qset, power.pe, power.qe, presyn_w_pu, refs.w_pu, refs.e_pu, refs.w_total_pu);

        refs.wt_gfm = phase_gfm_.step(refs.w_total_pu, Ts);

        const double va_ref = refs.e_pu * std::sin(refs.wt_gfm + 0.0);
        const double vb_ref = refs.e_pu * std::sin(refs.wt_gfm - (config::kTwoPi / 3.0));
        const double vc_ref = refs.e_pu * std::sin(refs.wt_gfm + (config::kTwoPi / 3.0));
        MathTransform::abc2dq(va_ref, vb_ref, vc_ref, refs.wt_gfm, refs.ed_ref, refs.eq_ref);

        return refs;
    }

    DualLoopState computeDualLoopState(const MeasuredSignals &signals,
                                       const ReferenceState &refs,
                                       double Ts)
    {
        // Voltage loop -> current loop cascade with decoupling and feedforward.
        DualLoopState dual_loop = {};

        const double icd = signals.ild - signals.iod;
        const double icq = signals.ilq - signals.ioq;
        const double wL = config::kLPu * refs.w_total_pu;
        const double wC = config::kCPu * refs.w_total_pu;

        const double vd_ref = refs.ed_ref + config::kWlVGain * signals.ioq - config::kRvGain * signals.iod;
        const double vq_ref = refs.eq_ref - config::kWlVGain * signals.iod - config::kRvGain * signals.ioq;

        const double id_ref_0 = vloop_d_.step(vd_ref - signals.vcd, Ts) - wC * signals.vcq;
        const double iq_ref_0 = vloop_q_.step(vq_ref - signals.vcq, Ts) + wC * signals.vcd;

        dual_loop.id_ref_gfm = id_ref_0 - config::kKad * icd;
        dual_loop.iq_ref_gfm = iq_ref_0 - config::kKad * icq;

        const double vd_ff = vff_d_lpf_.step(signals.vcd);
        const double vq_ff = vff_q_lpf_.step(signals.vcq);

        const double ud_star = vd_ff - wL * signals.ilq + iloop_d_.step(dual_loop.id_ref_gfm - signals.ild, Ts);
        const double uq_star = vq_ff + wL * signals.ild + iloop_q_.step(dual_loop.iq_ref_gfm - signals.ilq, Ts);

        dual_loop.ud_ref = config::kDqGain * ud_star;
        dual_loop.uq_ref = config::kDqGain * uq_star;
        MathTransform::dq2abc(dual_loop.ud_ref, dual_loop.uq_ref, refs.wt_gfm,
                              dual_loop.ua_ref, dual_loop.ub_ref, dual_loop.uc_ref);

        return dual_loop;
    }

    void writeOutput0(const MeasuredSignals &signals, double *out0) const
    {
        out0[output::kO0Vcd] = signals.vcd;
        out0[output::kO0Vcq] = signals.vcq;
        out0[output::kO0Ild] = signals.ild;
        out0[output::kO0Ilq] = signals.ilq;
        out0[output::kO0Iod] = signals.iod;
        out0[output::kO0Ioq] = signals.ioq;
        out0[output::kO0WtPll] = wt_gfl_;
    }

    void writeOutput1(const PowerState &power, const ReferenceState &refs, double *out1) const
    {
        out1[output::kO1PRaw] = power.p_raw;
        out1[output::kO1QRaw] = power.q_raw;
        out1[output::kO1Pe] = power.pe;
        out1[output::kO1Qe] = power.qe;
        out1[output::kO1WPu] = refs.w_pu;
        out1[output::kO1EPu] = refs.e_pu;
        out1[output::kO1WTotalPu] = refs.w_total_pu;
        out1[output::kO1WtGfm] = refs.wt_gfm;
        out1[output::kO1EdRef] = refs.ed_ref;
        out1[output::kO1EqRef] = refs.eq_ref;
    }

    void writeOutput2(const DualLoopState &dual_loop, double *out2) const
    {
        out2[output::kO2IdRefGfm] = dual_loop.id_ref_gfm;
        out2[output::kO2IqRefGfm] = dual_loop.iq_ref_gfm;
        out2[output::kO2UdRef] = dual_loop.ud_ref;
        out2[output::kO2UqRef] = dual_loop.uq_ref;
    }

    void writeOutput3(const DualLoopState &dual_loop, double *out3) const
    {
        out3[output::kO3UaRef] = dual_loop.ua_ref;
        out3[output::kO3UbRef] = dual_loop.ub_ref;
        out3[output::kO3UcRef] = dual_loop.uc_ref;
    }

    void updateDelayedStates(double wt_gfm)
    {
        // Commit delayed states at end of cycle.
        wt_gfl_delay_ = wt_gfl_;
        wt_gfm_delay_ = wt_gfm;
        ++step_count_;
    }

public:
    InverterControl()
        : pll_(),
          presyn_(),
          vloop_d_(config::kVLoopKp, config::kVLoopKi, config::kVLoopOutMin, config::kVLoopOutMax,
                   config::kVLoopErrDeadband, config::kVLoopAwKb),
          vloop_q_(config::kVLoopKp, config::kVLoopKi, config::kVLoopOutMin, config::kVLoopOutMax,
                   config::kVLoopErrDeadband, config::kVLoopAwKb),
          iloop_d_(config::kILoopKp, config::kILoopKi, config::kILoopOutMin, config::kILoopOutMax,
                   config::kILoopErrDeadband, config::kILoopAwKb),
          iloop_q_(config::kILoopKp, config::kILoopKi, config::kILoopOutMin, config::kILoopOutMax,
                   config::kILoopErrDeadband, config::kILoopAwKb),
          wt_gfl_(0.0),
          wt_gfl_delay_(0.0),
          wt_gfm_delay_(config::kWtGfmInitRad),
          step_count_(0ULL),
          pe_lpf_(config::kPeLpfNum, config::kPeLpfDen),
          qe_lpf_(config::kQeLpfNum, config::kQeLpfDen),
          vff_d_lpf_(config::kVffDLpfNum, config::kVffDLpfDen),
          vff_q_lpf_(config::kVffQLpfNum, config::kVffQLpfDen),
          droop_(),
          phase_gfm_()
    {
    }

    void reset()
    {
        pll_.reset();
        presyn_.reset();
        vloop_d_.reset();
        vloop_q_.reset();
        iloop_d_.reset();
        iloop_q_.reset();
        wt_gfl_ = 0.0;
        wt_gfl_delay_ = 0.0;
        wt_gfm_delay_ = config::kWtGfmInitRad;
        step_count_ = 0ULL;
        pe_lpf_.reset();
        qe_lpf_.reset();
        vff_d_lpf_.reset();
        vff_q_lpf_.reset();
        phase_gfm_.reset();
    }

    void step(const double *vgrid_abc,
              const double *vpcc_abc,
              const double *ilabc,
              const double *ioabc,
              double qset,
              double pset,
              double Ts,
              double *out0,
              double *out1,
              double *out2,
              double *out3)
    {
        // Pipeline: measure -> power -> droop/reference -> dual-loop -> output.
        const MeasuredSignals signals = measureSignals(vgrid_abc, vpcc_abc, ilabc, ioabc, Ts);
        const PowerState power = computePowerState(signals);
        const ReferenceState refs = computeReferenceState(signals, power, pset, qset, Ts);
        const DualLoopState dual_loop = computeDualLoopState(signals, refs, Ts);

        writeOutput0(signals, out0);
        writeOutput1(power, refs, out1);
        writeOutput2(dual_loop, out2);
        writeOutput3(dual_loop, out3);
        updateDelayedStates(refs.wt_gfm);
    }
};

}  // namespace inv_droop

#endif
