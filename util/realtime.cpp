#define S_FUNCTION_NAME  realtime
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"


#ifdef _WIN32 || _WIN64

  #include <Winsock2.h>
  #include <time.h>
  #include <windows.h>
  #include <time.h>
        
struct timespec {
    long tv_sec;
    long tv_nsec;
};

const unsigned int BILLION = 1000000000;
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL

int gettimeofday(struct timeval *tv);

int
nanosleep (const timespec *requested_delay,
timespec *remaining_delay);

int gettimeofday(struct timeval *tv, void* junk)  // second argument only here for compatibility
{
  FILETIME ft;
  unsigned __int64 tmpres = 0;
  static int tzflag;
  
  if (NULL != tv)
 {
    GetSystemTimeAsFileTime(&ft);
    
    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;
    
    /*converting file time to unix epoch*/
    tmpres -= DELTA_EPOCH_IN_MICROSECS;
    tmpres /= 10;  /*convert into microseconds*/
    tv->tv_sec = (long)(tmpres / 1000000UL);
    tv->tv_usec = (long)(tmpres % 1000000UL);
  }
  return 0;
}

int
nanosleep (const timespec *requested_delay,
timespec *remaining_delay)
{
  static bool initialized;
  /* Number of performance counter increments per nanosecond,
   * or zero if it could not be determined.  */
  static double ticks_per_nanosecond;
  
  if (requested_delay->tv_nsec < 0 || BILLION <= requested_delay->tv_nsec)
  {
    errno = 90;
    return -1;
  }
  
  /* For requested delays of one second or more, 15ms resolution is
    sufficient.  */
  if (requested_delay->tv_sec == 0) 
  {
    if (!initialized) 
    {
      /* Initialize ticks_per_nanosecond.  */
      LARGE_INTEGER ticks_per_second;
      
      if (QueryPerformanceFrequency(&ticks_per_second))
        ticks_per_nanosecond =
                (double) ticks_per_second.QuadPart / 1000000000.0;
      
      initialized = true;
    }
    if (ticks_per_nanosecond)
    {
      /* QueryPerformanceFrequency worked.  We can use
       * QueryPerformanceCounter.  Use a combination of Sleep and
       * busy-looping.  */
      /* Number of milliseconds to pass to the Sleep function.
       * Since Sleep can take up to 8 ms less or 8 ms more than requested
       * (or maybe more if the system is loaded), we subtract 10 ms.  */
      int sleep_millis = (int) requested_delay->tv_nsec / 1000000 - 10;
      /* Determine how many ticks to delay.  */
      LONGLONG wait_ticks = requested_delay->tv_nsec *
              ticks_per_nanosecond;
      /* Start.  */
      LARGE_INTEGER counter_before;
      if (QueryPerformanceCounter(&counter_before))
      {
        /* Wait until the performance counter has reached this value.
         * We don't need to worry about overflow, because the performance
         * counter is reset at reboot, and with a frequency of 3.6E6
         * ticks per second 63 bits suffice for over 80000 years.  */
        LONGLONG wait_until = counter_before.QuadPart + wait_ticks;
        /* Use Sleep for the longest part.  */
        if (sleep_millis > 0)
          Sleep(sleep_millis);
        /* Busy-loop for the rest.  */
        for (;;)
        {
          LARGE_INTEGER counter_after;
          if (!QueryPerformanceCounter(&counter_after))
            /* QueryPerformanceCounter failed, but succeeded earlier.
             * Should not happen.  */
            break;
          if (counter_after.QuadPart >= wait_until)
            /* The requested time has elapsed.  */
            break;
        }
        goto done;
      }
    }
  }
  /* Implementation for long delays and as fallback.  */
  Sleep(requested_delay->tv_sec * 1000 + requested_delay->tv_nsec / 1000000);
  
  done:
    /* Sleep is not interruptible.  So there is no remaining delay.  */
    if (remaining_delay != NULL)
    {
      remaining_delay->tv_sec = 0;
      remaining_delay->tv_nsec = 0;
    }
    return 0;
}

#else  // non windows

#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#endif


#define MDL_INITIAL_SIZES
static void mdlInitializeSizes(SimStruct *S)
{
  ssSetNumSFcnParams(S, 1);
  ssSetNumContStates(S, 0);
  ssSetNumDiscStates(S, 0);
  ssSetNumInputPorts(S, 0);
  ssSetNumOutputPorts(S, 0);
  
  ssSetNumSampleTimes(S, 1);
  ssSetNumPWork(S, 1);
}

#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 0)));
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
  struct timeval* p = new struct timeval;
  ssGetPWork(S)[0] = p;
  gettimeofday(p, NULL);
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
  gettimeofday(&tv, NULL);
  timeval_subtract(&tv_elapsed_time, &tv, (struct timeval*)ssGetPWork(S)[0]);
  
  // get simulation time
  time_T simtime = ssGetT(S);
  tv_sim_time.tv_sec = (long)simtime;
  tv_sim_time.tv_usec = (simtime - tv_sim_time.tv_sec)*1000000;
  
  if (!timeval_subtract(&tv_diff, &tv_sim_time, &tv_elapsed_time)) {
    timespec tosleep;
    tosleep.tv_sec = tv_diff.tv_sec;
    tosleep.tv_nsec = tv_diff.tv_usec * 1000;
    nanosleep(&tosleep, NULL);
  } else if (tv_diff.tv_sec<-1) {  // then I'm behind by more than 1 second
    mexPrintf("at time %d, I'm behind by %d sec, %d usec\n", tv_sim_time.tv_sec, tv_diff.tv_sec, tv_diff.tv_usec);
    ssSetErrorStatus(S, "Simulink is not keeping up with real time.  Consider reducing demands on your ODE solver, or optimizing your code.");
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
