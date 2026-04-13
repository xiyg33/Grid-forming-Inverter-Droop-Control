#ifndef INV_DROOP_CONFIG_HPP
#define INV_DROOP_CONFIG_HPP

namespace inv_droop {
namespace config {

// Shared signal width for abc vectors.
constexpr int kThreePhaseWidth = 3;

// Discrete control period and electrical base quantities.
constexpr double kTsCtrl = 50e-6;
constexpr double kFBaseHz = 50.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kOmegaBaseRadS = kTwoPi * kFBaseHz;

// dq reference gain mapping to modulation domain.
constexpr double kVBase = 310.3;
constexpr double kVBaseDchv = 800.0;
constexpr double kDqGain = kVBase * (2.0 / kVBaseDchv);

// PLL tuning and output limits.
constexpr double kPllKp = 180.0;
constexpr double kPllKi = 3200.0;
constexpr double kGflWPuMin = -0.01;
constexpr double kGflWPuMax = 0.01;
constexpr double kGflErrDeadband = 1e-4;
constexpr double kGflAwKb = 17.8;

// Presynchronization active window and loop constraints.
constexpr double kPresynStartS = 0.25;
constexpr double kPresynEndS = 0.45;
constexpr double kPresynWPuMin = -0.01;
constexpr double kPresynWPuMax = 0.01;
constexpr double kPresynErrDeadband = 1e-4;
constexpr double kPresynAwKb = 17.8;

// Droop gains.
constexpr double kMp = 0.005;
constexpr double kNq = 0.33333;

// Droop saturation and dead-zone configuration.
constexpr double kWPuMin = 0.8;
constexpr double kWPuMax = 1.2;
constexpr double kEPuMin = 0.8;
constexpr double kEPuMax = 1.2;
constexpr double kW0DeadzoneLow = -1e-4;
constexpr double kW0DeadzoneHigh = 1e-4;

// Dual-loop preparation parameters.
constexpr double kLPu = 0.081585;
constexpr double kCPu = 0.027219;
constexpr double kKad = 0.2;
constexpr double kWlVGain = 0.0;
constexpr double kRvGain = 0.0;

// Voltage-loop PI gains and limits.
constexpr double kVLoopKp = 0.2;
constexpr double kVLoopKi = 30.0;
constexpr double kVLoopOutMin = -1.2;
constexpr double kVLoopOutMax = 1.2;
constexpr double kVLoopErrDeadband = 1e-4;
constexpr double kVLoopAwKb = 150.0;

// Current-loop PI gains and limits.
constexpr double kILoopKp = 0.65;
constexpr double kILoopKi = 43.27;
constexpr double kILoopOutMin = -1.2;
constexpr double kILoopOutMax = 1.2;
constexpr double kILoopErrDeadband = 1e-4;
constexpr double kILoopAwKb = 66.7;

// UdUq Feedforward LPF coefficients (discrete 1st order form).
constexpr double kVffLpfNum0 = 0.015585500605;
constexpr double kVffLpfDen1 = -0.984414499395;
static const double kVffDLpfNum[3] = {kVffLpfNum0, 0.0, 0.0};
static const double kVffDLpfDen[3] = {1.0, kVffLpfDen1, 0.0};
static const double kVffQLpfNum[3] = {kVffLpfNum0, 0.0, 0.0};
static const double kVffQLpfDen[3] = {1.0, kVffLpfDen1, 0.0};

// Initial GFM phase.
constexpr double kWtGfmInitRad = 0.0;

// Power LPF coefficients.
static const double kPeLpfNum[3] = {0.009380498, 0.0, 0.0};
static const double kPeLpfDen[3] = {1.0, -0.990619502, 0.0};
static const double kQeLpfNum[3] = {0.009380498, 0.0, 0.0};
static const double kQeLpfDen[3] = {1.0, -0.990619502, 0.0};

}  // namespace config

namespace output {

// Output port 0: measured dq quantities and PLL angle.
enum Port0Index {
    kO0Vcd = 0,
    kO0Vcq,
    kO0Ild,
    kO0Ilq,
    kO0Iod,
    kO0Ioq,
    kO0WtPll,
    kPort0Width
};

// Output port 1: power, droop states and voltage references.
enum Port1Index {
    kO1PRaw = 0,
    kO1QRaw,
    kO1Pe,
    kO1Qe,
    kO1WPu,
    kO1EPu,
    kO1WTotalPu,
    kO1WtGfm,
    kO1EdRef,
    kO1EqRef,
    kPort1Width
};

// Output port 2: dual-loop current and dq voltage references.
enum Port2Index {
    kO2IdRefGfm = 0,
    kO2IqRefGfm,
    kO2UdRef,
    kO2UqRef,
    kPort2Width
};

// Output port 3: three-phase modulation references.
enum Port3Index {
    kO3UaRef = 0,
    kO3UbRef,
    kO3UcRef,
    kPort3Width
};

}  // namespace output
}  // namespace inv_droop

#endif
