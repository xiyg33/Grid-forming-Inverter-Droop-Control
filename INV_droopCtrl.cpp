#define S_FUNCTION_NAME  INV_droopCtrl
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <cmath>

#include "inv_droop/inverter_control.hpp"

// Keep local aliases short for S-Function glue code readability.
namespace config = inv_droop::config;
namespace output = inv_droop::output;

// Defensive zeroing: on any invalid path, outputs remain deterministic.
static void zeroOutput(real_T *signal, int width)
{
    for (int i = 0; i < width; ++i) {
        signal[i] = 0.0;
    }
}

static bool isFiniteSignal(const real_T *signal, int width)
{
    for (int i = 0; i < width; ++i) {
        if (!std::isfinite(signal[i])) {
            return false;
        }
    }
    return true;
}

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    if (!ssSetNumInputPorts(S, 6)) return;

    ssSetInputPortWidth(S, 0, config::kThreePhaseWidth);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    ssSetInputPortWidth(S, 1, config::kThreePhaseWidth);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortRequiredContiguous(S, 1, 1);

    ssSetInputPortWidth(S, 2, config::kThreePhaseWidth);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortRequiredContiguous(S, 2, 1);

    ssSetInputPortWidth(S, 3, config::kThreePhaseWidth);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortRequiredContiguous(S, 3, 1);

    ssSetInputPortWidth(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortRequiredContiguous(S, 4, 1);

    ssSetInputPortWidth(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortRequiredContiguous(S, 5, 1);

    if (!ssSetNumOutputPorts(S, 4)) return;
    ssSetOutputPortWidth(S, 0, output::kPort0Width);
    ssSetOutputPortWidth(S, 1, output::kPort1Width);
    ssSetOutputPortWidth(S, 2, output::kPort2Width);
    ssSetOutputPortWidth(S, 3, output::kPort3Width);

    ssSetNumSampleTimes(S, 1);

    if (!ssSetNumDWork(S, 0)) return;
    ssSetNumPWork(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, config::kTsCtrl);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    inv_droop::InverterControl *controller = new inv_droop::InverterControl();
    ssGetPWork(S)[0] = (void *)controller;

    if (controller != NULL) {
        controller->reset();
    }
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    inv_droop::InverterControl *controller = (inv_droop::InverterControl *)ssGetPWork(S)[0];
    if (controller == NULL) return;

    if (ssIsSampleHit(S, 0, tid)) {
        // Read all inputs first; control core stays independent from SimStruct API.
        const real_T *vgrid_abc = (const real_T *)ssGetInputPortSignal(S, 0);
        const real_T *vpcc_abc = (const real_T *)ssGetInputPortSignal(S, 1);
        const real_T *ilabc = (const real_T *)ssGetInputPortSignal(S, 2);
        const real_T *ioabc = (const real_T *)ssGetInputPortSignal(S, 3);
        const real_T *qset_ptr = (const real_T *)ssGetInputPortSignal(S, 4);
        const real_T *pset_ptr = (const real_T *)ssGetInputPortSignal(S, 5);

        real_T *out0 = ssGetOutputPortRealSignal(S, 0);
        real_T *out1 = ssGetOutputPortRealSignal(S, 1);
        real_T *out2 = ssGetOutputPortRealSignal(S, 2);
        real_T *out3 = ssGetOutputPortRealSignal(S, 3);

        zeroOutput(out0, output::kPort0Width);
        zeroOutput(out1, output::kPort1Width);
        zeroOutput(out2, output::kPort2Width);
        zeroOutput(out3, output::kPort3Width);

        if (vgrid_abc == NULL || vpcc_abc == NULL || ilabc == NULL || ioabc == NULL || qset_ptr == NULL || pset_ptr == NULL) {
            return;
        }

        if (!isFiniteSignal(vgrid_abc, config::kThreePhaseWidth) ||
            !isFiniteSignal(vpcc_abc, config::kThreePhaseWidth) ||
            !isFiniteSignal(ilabc, config::kThreePhaseWidth) ||
            !isFiniteSignal(ioabc, config::kThreePhaseWidth) ||
            !std::isfinite(qset_ptr[0]) ||
            !std::isfinite(pset_ptr[0])) {
            return;
        }

        // Single-step control dispatch using fixed sample time.
        controller->step(vgrid_abc,
                         vpcc_abc,
                         ilabc,
                         ioabc,
                         qset_ptr[0],
                         pset_ptr[0],
                         config::kTsCtrl,
                         out0,
                         out1,
                         out2,
                         out3);
    }
}

static void mdlTerminate(SimStruct *S)
{
    inv_droop::InverterControl *controller = (inv_droop::InverterControl *)ssGetPWork(S)[0];
    if (controller != NULL) {
        delete controller;
        ssGetPWork(S)[0] = NULL;
    }
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
