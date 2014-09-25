#define S_FUNCTION_NAME  realtime
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

#define UNUSED(x) (void)(x)

#if defined(_WIN32) || defined(_WIN64)

  #include <Winsock2.h>
  #include <time.h>
  #include <windows.h>
  #include <time.h>

struct timespec {
    long tv_sec;
    long tv_nsec;
};
typedef long suseconds_t;

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

double timevalToDouble(struct timeval tv)
{
	double t;
	// set the struct values
  if ( tv.tv_sec < 0 )
     t = (double)tv.tv_sec - (double)tv.tv_usec / 1000000.0;
  else
     t = (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
  return t;
}

struct timeval doubleToTimeval(double t)
{
  // create the struct timeval
	struct timeval tv;
	tv.tv_sec = (time_t) t;
	tv.tv_usec = (suseconds_t)((t - tv.tv_sec ) * 1000000.0 );

	// return the timeval struct
	return( tv );
}

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
  struct timeval tv;
  gettimeofday(&tv, NULL);

  double *wall_t0 = (double*) ssGetDWork(S,0);
  if (*wall_t0 == TIME_NOT_YET_SET) {
    // t0 not set yet, set it now.  (note: used to set this in mdlStart, but there can be a big pause between mdlStart and the full simulation start)
    *wall_t0 = timevalToDouble(tv);
    return;
  }

  double wall_t = timevalToDouble(tv),
  			simtime = ssGetT(S),
  			realtime_factor = mxGetScalar(ssGetSFcnParam(S, 1));

  double t_diff = (wall_t - *wall_t0)*realtime_factor - simtime;
// mexPrintf("wall time: %f, sim time: %f, (scaled) diff: %f\n", wall_t-*wall_t0, simtime, t_diff);

  if (t_diff<0.0) {  // then simtime > scaled wall_time
    timespec tosleep;
    tv = doubleToTimeval(-t_diff);
    tosleep.tv_sec = tv.tv_sec;
    tosleep.tv_nsec = tv.tv_usec * 1000;
//    mexPrintf("sleeping...  %d sec, %d nsec\n", tosleep.tv_sec, tosleep.tv_nsec);
    nanosleep(&tosleep, NULL);
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
