
#define S_FUNCTION_NAME  RimlessWheelSFuncPlant
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include "simstruc.h"

static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    return;
  }

  ssSetNumContStates(S, 2);
  ssSetNumDiscStates(S, 0);

  if (!ssSetNumInputPorts(S, 0)) return;
  if (!ssSetNumOutputPorts(S, 1)) return;
  ssSetOutputPortWidth(S, 0, 2);

  ssSetNumSampleTimes(S, 1);
  ssSetNumRWork(S, 0);
  ssSetNumIWork(S, 0);
  ssSetNumPWork(S, 0);
  ssSetNumModes(S, 0);
  ssSetNumNonsampledZCs(S, 1);

  ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

  ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
  real_T* x0 = ssGetContStates(S);
  x0[0] = .1;
  x0[1] = 0;
}
#endif /* MDL_INITIALIZE_CONDITIONS */


const double rw_gamma = 0.08, alpha = 0.3927;

static void mdlOutputs(SimStruct *S, int_T tid)
{
  real_T *x = ssGetContStates(S);
  real_T *y = (real_T*) ssGetOutputPortSignal(S, 0);
  
  if (ssIsMajorTimeStep(S)) {
    // then check for zero-crossing events and apply them
    if (x[0] >= rw_gamma + alpha) {
      x[0] = rw_gamma-alpha;
      x[1] = cos(2*alpha)*x[1];
    } else if (x[0] <= rw_gamma - alpha) {
      x[0] = rw_gamma+alpha;
      x[1] = cos(2*alpha)*x[1];
    }
  }

  y[0] = x[0];
  y[1] = x[1];
}

#define MDL_ZERO_CROSSINGS   /* Change to #undef to remove function */
#if defined(MDL_ZERO_CROSSINGS) && (defined(MATLAB_MEX_FILE) || defined(NRT))
  /* Function: mdlZeroCrossings ===============================================
   * Abstract:
   *    If your S-function has registered CONTINUOUS_SAMPLE_TIME and there
   *    are signals entering the S-function or internally generated signals
   *    which have discontinuities, you can use this method to locate the
   *    discontinuities. When called, this method must update the
   *    ssGetNonsampleZCs(S) vector.
   */
static void mdlZeroCrossings(SimStruct *S)
{ 
  real_T *x = ssGetContStates(S);
  real_T *zcSignals = ssGetNonsampledZCs(S);  
  zcSignals[0] = (x[1]>=0 ? 1.0 : -1.0)*(x[0]-rw_gamma) - alpha;
}
#endif /* MDL_ZERO_CROSSINGS */

#define MDL_DERIVATIVES 
#if defined(MDL_DERIVATIVES)
static void mdlDerivatives(SimStruct *S)
{
  const real_T *x = ssGetContStates(S);
  real_T *dx = ssGetdX(S);
  
  dx[0] = x[1];
  dx[1] = 9.8*sin(x[0]);
}
#endif /* MDL_DERIVATIVES */

static void mdlTerminate(SimStruct *S)
{
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
