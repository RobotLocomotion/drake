#define S_FUNCTION_NAME  lcmLogger
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#include <lcm/lcm.h>

#define UNUSED(x) (void)(x)

static double simtime = -1.0;
static lcm_t* lcm = NULL;

#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S, 2);  // lcm channel regex, 
                             // workspace variable name to write log into
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);
  ssSetNumInputPorts(S, 0);
  ssSetNumOutputPorts(S, 0);
  
  ssSetNumSampleTimes(S, 1);
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

static void message_handler (const lcm_recv_buf_t *rbuf, const char *channel, void *u)
{
  mexPrintf("got message at simtime %f\n",simtime);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
  if (lcm)
    ssSetErrorStatus(S, "LCM already exists. I've assumed that only one of these is used at a time (though it might not be catastrophic if there were more).");

  lcm = lcm_create(NULL);
  if (!lcm)
    ssSetErrorStatus(S, "Failed to create LCM object in lcmLog block.");
  
  char* channel_regex = mxArrayToString(ssGetSFcnParam(S, 0));
  
  mexPrintf("Logging LCM channels '%s'\n",channel_regex);
  lcm_subscribe(lcm, channel_regex, message_handler, NULL);

  mxFree(channel_regex);
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
  UNUSED(tid);
  simtime = ssGetT(S);
  
  if (lcm)
    lcm_handle(lcm);
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
  if (lcm)
    lcm_destroy(lcm);
  
  char* log_variable_name = mxArrayToString(ssGetSFcnParam(S, 1));

  mexPrintf("Writing LCM log to %s\n",log_variable_name);
  // todo: write log to mxArray named in S
  
  mxFree(log_variable_name);
  
}

#ifdef MATLAB_MEX_FILE    /* Is this file being compiled as a 
                             MEX-file? */
#include "simulink.c"     /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"      /* Code generation registration 
                             function */
#endif
