#define S_FUNCTION_NAME  lcmLogger
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#include <inttypes.h>
#include <sys/select.h>
#include <lcm/lcm.h>

#include <string>
#include <list>
using namespace std;


#define UNUSED(x) (void)(x)

class LCMMessage
{
public:
  double simtime;
  string channel;
  char* data = NULL;

  virtual ~LCMMessage(void)
  {
    if (!data) delete[] data;
  }
};

static double simtime = -1.0;
static lcm_t* lcm = NULL;
list<LCMMessage> message_log;


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
  mexPrintf("simtime %f\t",simtime);
  mexPrintf("channel %s\n",channel);
  
  LCMMessage m;
  m.simtime = simtime;
  m.channel = channel;
  m.data = new char[rbuf->data_size];
  memcpy(m.data,rbuf->data,rbuf->data_size);
  
  message_log.push_back(m);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
  if (lcm) ssSetErrorStatus(S, "LCM already exists. I've assumed that only one of these is used at a time (though it might not be catastrophic if there were more).");

  lcm = lcm_create(NULL);
  if (!lcm) ssSetErrorStatus(S, "Failed to create LCM object in lcmLog block.");
  
  char* channel_regex = mxArrayToString(ssGetSFcnParam(S, 0));
  
  mexPrintf("Logging LCM channels '%s'\n",channel_regex);
  lcm_subscribe(lcm, channel_regex, message_handler, NULL);

  mxFree(channel_regex);
}


#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid) 
{
  UNUSED(tid);
  
  if (!lcm) ssSetErrorStatus(S, "Invalid LCM object.");
  
  simtime = ssGetT(S);

  // setup the LCM file descriptor for waiting.
  int lcm_fd = lcm_get_fileno(lcm);
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(lcm_fd, &fds);
  
  // wait a limited amount of time for an incoming message
  struct timeval timeout = {
    0,  // seconds
    0   // microseconds
  };
  int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);
  
  if(status!=0 && FD_ISSET(lcm_fd, &fds)) {
    // LCM has events ready to be processed.
    lcm_handle(lcm);
  }
}


static void mdlTerminate(SimStruct *S) 
{
  UNUSED(S);
  if (!lcm) ssSetErrorStatus(S, "Invalid LCM object.");
  
  lcm_destroy(lcm);
  lcm = NULL;
  
  char* log_variable_name = mxArrayToString(ssGetSFcnParam(S, 1));

  mexPrintf("Writing LCM log to %s\n",log_variable_name);

  for (list<LCMMessage>::iterator iter=message_log.begin(); iter!=message_log.end(); iter++) {
    mexPrintf("got %s at time %f\n", iter->channel.c_str(), iter->simtime);
  }
  
  // todo: write to variable.  drake mex pointer?
  message_log.clear();
  
  mxFree(log_variable_name);
}

#ifdef MATLAB_MEX_FILE    /* Is this file being compiled as a 
                             MEX-file? */
#include "simulink.c"     /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"      /* Code generation registration 
                             function */
#endif
