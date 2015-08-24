#define S_FUNCTION_NAME  realtime
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"
#include <chrono>
#include <thread>

#define UNUSED(x) (void)(x)

#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S, 2);
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);
  ssSetNumInputPorts(S, 0);
  ssSetNumOutputPorts(S, 0);

  ssSetNumSampleTimes(S, 1);
  ssSetNumPWork(S, 1);  // start time
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
  ssSetPWorkValue(S,0,NULL);
}

typedef std::chrono::system_clock TimeClock;  // would love to use steady_clock, but it seems to not compile on all platforms (e.g. MSVC 2013 Win64)
typedef std::chrono::duration<double> TimeDuration;
typedef std::chrono::time_point<TimeClock,TimeDuration> TimePoint;

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
  UNUSED(tid);

  TimePoint *wall_clock_start_time = static_cast<TimePoint*>(ssGetPWorkValue(S,0));
  if (!wall_clock_start_time) {
    // t0 not set yet, set it now.  (note: used to set this in mdlStart, but there can be a big pause between mdlStart and the full simulation start)
    wall_clock_start_time = new TimePoint();
    *wall_clock_start_time = TimeClock::now();
    ssSetPWorkValue(S,0,wall_clock_start_time);
  }

  double sim_time = ssGetT(S);
  double realtime_factor = mxGetScalar(ssGetSFcnParam(S, 1));
  TimePoint wall_time = TimeClock::now();
  TimePoint desired_time = *wall_clock_start_time + TimeDuration(sim_time/realtime_factor);
  if (desired_time>wall_time) { // could probably just call sleep_until, but just in case
    std::this_thread::sleep_until(desired_time);
  } else if (wall_time>desired_time+std::chrono::duration<double>(1.0/realtime_factor)) {
    mexPrintf("at time %f, I'm behind by more than 1 (scaled) second\n", sim_time);
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
