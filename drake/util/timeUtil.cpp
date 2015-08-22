
#include "timeUtil.h"

#if defined(_WIN32) || defined(_WIN64)

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
  static double ticks_per_nanosecond = 0.0;

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
      LONGLONG wait_ticks = static_cast<LONGLONG>(requested_delay->tv_nsec *
                                                  ticks_per_nanosecond);
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

#endif // windows

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
  tv.tv_sec = static_cast<decltype(tv.tv_sec)>(t);
  tv.tv_usec = static_cast<decltype(tv.tv_usec)>((t - tv.tv_sec ) * 1000000.0 );

  // return the timeval struct
  return( tv );
}

double getTimeOfDay(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return timevalToDouble(tv);
}

void nanoSleep(double duration) {
  timespec tosleep;
  struct timeval tv = doubleToTimeval(duration);
  tosleep.tv_sec = tv.tv_sec;
  tosleep.tv_nsec = tv.tv_usec * 1000;
//    mexPrintf("sleeping...  %d sec, %d nsec\n", tosleep.tv_sec, tosleep.tv_nsec);
  nanosleep(&tosleep, NULL);
}