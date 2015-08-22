#ifndef DRAKE_WINTIME_H
#define DRAKE_WINTIME_H

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



#else  // non windows
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#endif


extern double timevalToDouble(struct timeval tv);
extern struct timeval doubleToTimeval(double t);
extern double getTimeOfDay(void);
extern void nanoSleep(double duration);

#endif //DRAKE_WINTIME_H
