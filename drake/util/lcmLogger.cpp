/* lcmLogger
 *   An S-function, with no inputs and no outputs, that will simply
 * collect LCM traffic on the known channel and save it to a workspace
 * variable in a matlab structure.
 *
 * Use the mex file publishLCMLog to publish those messages back out.
 */

#define S_FUNCTION_NAME lcmLogger
#define S_FUNCTION_LEVEL 2
#include <simstruc.h>

#include <lcm/lcm.h>

#include <string>
#include <list>

#ifdef WIN32
#define __STDC_FORMAT_MACROS  // Enable integer types
#include <lcm/WinPorting.h>
#else
#include <inttypes.h>
#include <sys/select.h>
#endif

using namespace std;

#define UNUSED(x) (void)(x)

class LCMMessage {
 public:
  double simtime;
  string channel;
  mxArray *data;

  virtual ~LCMMessage(void) {
    // don't actually free this memory, as I'm going to return it to matlab
    //    if (!data) mxDestroyArray(data);
  }
};

static double simtime = -1.0;
static lcm_t *lcm = NULL;
list<LCMMessage> message_log;

#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S) {
  ssSetNumSFcnParams(S, 2);  // lcm channel regex,
                             // workspace variable name to write log into
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);
  ssSetNumInputPorts(S, 0);
  ssSetNumOutputPorts(S, 0);

  ssSetNumSampleTimes(S, 1);
}

#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S) { ssSetSampleTime(S, 0, 0); }

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S) {
  UNUSED(S);
  // todo: verify dialog parameters here
}
#endif /* MDL_CHECK_PARAMETERS */

static void message_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                            void *u) {
  LCMMessage m;
  m.simtime = simtime;
  m.channel = channel;

  mwSize size[2] = {static_cast<mwSize>(rbuf->data_size), 1};
  m.data = mxCreateCharArray(2, size);
  memcpy(mxGetData(m.data), rbuf->data, rbuf->data_size);
  mexMakeArrayPersistent(m.data);

  //  mexPrintf("lcm log: %s at %f\n", channel, m.simtime);
  message_log.push_back(m);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
  if (lcm)
    ssSetErrorStatus(S,
                     "LCM already exists. I've assumed that only one of these "
                     "is used at a time (though it might not be catastrophic "
                     "if there were more).");

  lcm = lcm_create(NULL);
  if (!lcm)
    ssSetErrorStatus(S, "Failed to create LCM object in lcmLogger s-function.");

  char *channel_regex = mxArrayToString(ssGetSFcnParam(S, 0));
  lcm_subscribe(lcm, channel_regex, message_handler, NULL);

  mxFree(channel_regex);
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
  UNUSED(tid);

  simtime = ssGetT(S);

  while (true) {
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

    if (status == 0) {
      break;  // no messages
    } else if (FD_ISSET(lcm_fd, &fds)) {
      // LCM has events ready to be processed.
      lcm_handle(lcm);
    }
  }
}

#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid) {
  UNUSED(S);
  UNUSED(tid);
}

static void mdlTerminate(SimStruct *S) {
  UNUSED(S);
  if (!lcm) ssSetErrorStatus(S, "Invalid LCM object.");

  lcm_destroy(lcm);
  lcm = NULL;

  char *log_variable_name = mxArrayToString(ssGetSFcnParam(S, 1));

  //  mexPrintf("Writing LCM log with %d records to
  //  %s\n", message_log.size(), log_variable_name);

  const char *fieldnames[] = {"simtime", "channel", "data"};

  mxArray *pData = mxCreateStructMatrix(message_log.size(), 1, 3, fieldnames);

  int i = 0;
  for (list<LCMMessage>::iterator iter = message_log.begin();
       iter != message_log.end(); iter++) {
    mxSetFieldByNumber(pData, i, 0, mxCreateDoubleScalar(iter->simtime));
    mxSetFieldByNumber(pData, i, 1, mxCreateString(iter->channel.c_str()));
    mxSetFieldByNumber(pData, i, 2, iter->data);
    i++;
  }

  mexPutVariable("base", log_variable_name, pData);

  message_log.clear();
  mxFree(log_variable_name);
}

#ifdef MATLAB_MEX_FILE  // Is this file being compiled as a MEX-file?
#include "simulink.c"   // MEX-file interface mechanism
#else
// NOLINTNEXTLINE(build/include)
#include "cg_sfun.h"    // Code generation registration function
#endif
