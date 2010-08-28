#define S_FUNCTION_NAME  realtime
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#include <sys/time.h>
#include <time.h>
#include <unistd.h>


#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S,1);
  ssSetNumContStates(S,0);
  ssSetNumDiscStates(S,0);
  ssSetNumInputPorts(S,0);
  ssSetNumOutputPorts(S,0);

  ssSetNumSampleTimes(S,1);
  ssSetNumPWork(S,1);
}

#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S,0,mxGetScalar(ssGetSFcnParam(S,0)));
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
  struct timeval* p = new struct timeval;
  ssGetPWork(S)[0] = p;
  gettimeofday(p,NULL);
}

/* Subtract the `struct timeval' values X and Y,
 * storing the result in RESULT.
 * Return 1 if the difference is negative, otherwise 0.  
 * (from http://www.gnu.org/s/libc/manual/html_node/Elapsed-Time.html)
 */
int timeval_subtract(struct timeval* result, struct timeval *x, struct timeval* y)
{
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_usec < y->tv_usec) {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000) {
    int nsec = (x->tv_usec - y->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }
  
  /* Compute the time remaining to wait.
   * tv_usec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;
  
  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}


#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
  struct timeval tv, tv_elapsed_time, tv_sim_time, tv_diff;

  // get elapsed time
  gettimeofday(&tv,NULL);
  timeval_subtract(&tv_elapsed_time,&tv,(struct timeval*)ssGetPWork(S)[0]);

  // get simulation time
  time_T simtime = ssGetT(S);
  tv_sim_time.tv_sec = (long)simtime;
  tv_sim_time.tv_usec = (simtime - tv_sim_time.tv_sec)*1000000;
  
  if (!timeval_subtract(&tv_diff,&tv_sim_time,&tv_elapsed_time)) {
    struct timespec tosleep;
    tosleep.tv_sec = tv_diff.tv_sec;
    tosleep.tv_nsec = tv_diff.tv_usec * 1000;
    nanosleep(&tosleep,NULL);
  } else if (tv_diff.tv_sec<-1) {  // then I'm behind by more than 1 second
    mexPrintf("%d sec, %d usec\n", tv_diff.tv_sec, tv_diff.tv_usec);
    ssSetErrorStatus(S,"Simulink is not keeping up with real time.  Consider reducing demands on your ODE solver, or optimizing your code.");  
  }
}



#define MDL_OUTPUTS
static void mdlOutputs(SimStruct *S, int_T tid) {}


static void mdlTerminate(SimStruct *S) 
{
  struct timeval *p = (struct timeval*) ssGetPWork(S)[0];
  delete p;
}

#ifdef MATLAB_MEX_FILE    /* Is this file being compiled as a 
                             MEX-file? */
#include "simulink.c"     /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"      /* Code generation registration 
                             function */
#endif