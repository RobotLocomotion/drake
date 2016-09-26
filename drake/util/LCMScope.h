#pragma once

#include "drake/lcmt_scope_data.h"
#include <string.h>
#include <time.h>
#include <lcm/lcm.h>

/*Utility to automate the use of our matlab scope infrastructure with C++
programs.
This code wraps the lcmt_scope_data type so you don't have get so dirty with
pointers
and adds time on the x-axis so you don't have to.

Zack Jackowski, Aug 2010
*/
class LCMScope {
  lcmt_scope_data data;
  lcm_t* lcm;
  char* lcm_chan;
  timeval start;

 private:
  // http://www.mpp.mpg.de/~huber/util/timevaldiff.c
  /// @return the (finishtime - starttime), in milliseconds
  double timevaldiff(struct timeval* starttime, struct timeval* finishtime) {
    double msec;
    msec = (finishtime->tv_sec - starttime->tv_sec) * 1000.0;
    msec += (finishtime->tv_usec - starttime->tv_usec) / 1000.0;
    return msec;
  }

 public:
  LCMScope(lcm_t* passed_lcm, char* chan_name) {
    lcm = passed_lcm;

    data.scope_id = 1;
    data.num_points = 100;
    data.linespec = "";
    lcm_chan = chan_name;

    data.resetOnXval = false;

    data.xdata = 0;
    data.ydata = 0;
    gettimeofday(&start, 0);
  }

  void setScopeID(int id) { data.scope_id = id; }

  void setNumPoints(int num_points) { data.num_points = num_points; }

  void setLineSpec(char* spec) { data.linespec = spec; }

  void setResetOnXval(bool value) { data.resetOnXval = value; }

  void setLCMChannel(char* channel) { lcm_chan = channel; }

  /*
  Send out data using an xy pair with the current message settings
  */
  void publish(double x_data, double y_data) {
    data.xdata = x_data;
    data.ydata = y_data;
    lcmt_scope_data_publish(lcm, lcm_chan, &data);
  }

  /*
  Send out data using elapsed program time for x
  */
  void publish(double y_data) {
    timeval end;
    gettimeofday(&end, 0);
    data.xdata = timevaldiff(&end, &start);
    data.ydata = y_data;
    lcmt_scope_data_publish(lcm, lcm_chan, &data);
  }

  /*
  Send out the message using the currently stored data
  */
  void publish(void) { lcmt_scope_data_publish(lcm, lcm_chan, &data); }
};
