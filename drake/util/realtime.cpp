#define S_FUNCTION_NAME  realtime
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define UNUSED(x) (void)(x)

#include "timeUtil.h"

const double TIME_NOT_YET_SET = -1.0;

#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S, 2);
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);
  ssSetNumInputPorts(S, 0);
  ssSetNumOutputPorts(S, 0);

  ssSetNumSampleTimes(S, 1);
  ssSetNumDWork(S, 1);  // wall t0
  ssSetDWorkWidth(S, 0, 1);
  ssSetDWorkDataType(S, 0, SS_DOUBLE);
}

#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 0)));
}

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
  UNUSED(S);
	// todo: verify dialog parameters here
}
#endif /* MDL_CHECK_PARAMETERS */


#define MDL_START
static void mdlStart(SimStruct *S)
{
  double *wall_t0 = (double*) ssGetDWork(S,0);
  *wall_t0 = TIME_NOT_YET_SET;
;
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
  UNUSED(tid);

  double *wall_t0 = (double*) ssGetDWork(S,0);
  if (*wall_t0 == TIME_NOT_YET_SET) {
    // t0 not set yet, set it now.  (note: used to set this in mdlStart, but there can be a big pause between mdlStart and the full simulation start)
    *wall_t0 = getTimeOfDay();
    return;
  }

  double wall_t = timevalToDouble(tv),
  			simtime = ssGetT(S),
  			realtime_factor = mxGetScalar(ssGetSFcnParam(S, 1));

  double t_diff = (wall_t - *wall_t0)*realtime_factor - simtime;
// mexPrintf("wall time: %f, sim time: %f, (scaled) diff: %f\n", wall_t-*wall_t0, simtime, t_diff);

  if (t_diff<0.0) {  // then simtime > scaled wall_time
    nanoSleep(-t_diff);
  } else if (t_diff>1.0) {  // then I'm behind by more than 1 second
    mexPrintf("at time %f, I'm behind by %f sec\n", simtime, t_diff);
    ssSetErrorStatus(S, "Simulink is not keeping up with real time.  Consider reducing demands on your ODE solver, or optimizing your code.");
  }
}


#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid)
{
  UNUSED(S);
  UNUSED(tid);
}


static void mdlTerminate(SimStruct *S)
{
  UNUSED(S);
}

#ifdef MATLAB_MEX_FILE    /* Is this file being compiled as a
                             MEX-file? */
#include "simulink.c"     /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"      /* Code generation registration
                             function */
#endif
