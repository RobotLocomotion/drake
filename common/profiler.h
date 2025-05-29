#pragma once

/* Using INSTRUMENT_FUNCTION:

In the source.cc you want to profile:
  1. #include "drake/common/profiler.h"
  2. At the top of your function, insert INSTRUMENT_FUNCTION("Description");
  3. Add "//common:profiler", in the BAZEL.build deps = [].

In the main application:
  1. Include #include "drake/common/profiler.h"
  2. Print with: std::cout << TableOfAverages() << "\n";
  3. Add "//common:profiler", to the dependencies.
  4. Add copts = ["-DENABLE_TIMERS"],

Building:
Compile with `bazel build --copt=-DENABLE_TIMERS path/to:target`.

TableOfAverages() report per column:
  - Time/sample: Per call average time in seconds (i.e.=[Total time]/[Samples]).
  - Samples: Number of calls.
  - Total time: Total time spent by the specific function. This includes self
    time plus time spent in any other functions called within scope.
  - Self: Time spent in the scope of the instrumented function. This time does
    not include the time spent in other instrumented functions, called whether
    from the scope of this function or transitively.
  - Label: label provided to the INSTRUMENT_FUNCTION macro.

Sel total is the sum of all self times. If we don't miss anything important,
this number should be in close agreement with the total execution time or the
total execution time of the function or code block we are interested in.

*/

#include <chrono>
#include <ratio>
#include <string>
#include <utility>

#include "drake/common/scope_exit.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace common {

/** Index used to identify a registered timer.  */
using TimerIndex = TypeSafeIndex<class TimerTag>;

// TODO(SeanCurtis-TRI): Need to figure out how to prevent calling timer
// functions *before* calling start.

/**  Basic timer. A timer that can start and report the amount of time elapsed
 since calling start().

   times    t₀         t₁      t₂       t₃         t₄            t₅
          ┄┄┾━━━━━━━━━━┿━━━━━━━┿━━━━━━━━┿━━━━━━━━━━┿━━━━━━━━━━━━━┽┄┄┄┄
   f()      s()        e()     e()      e()        e()           e()

 The timeline above shows a series of %Timer method invocations: `s()` and
 `e()`, representing start() and elapsed(), respectively. At each call of
 elapsed, the duration elapsed between that time and the time that start() was
 called is returned (i.e., `tᵢ - t₀`, for `i > 0`).  */
class Timer {
 public:
  Timer() = default;

  /**  Starts the timer running.  */
  void start();

  /**  Reports the time elapsed between this call and the last invocation of
   start() in the units specified.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template <typename Units>
  double elapsed() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, Units>(end - start_).count();
  }

 protected:
  /**  The time (in clock cycles) at which the timer was started.  */
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

///////////////////////////////////////////////////////////////////////////

/**  Lap timer. A timer which supports the idea of "laps", a sequence of timer
 milestones in an otherwise uninterrupted window of time.

   times    t₀         t₁      t₂       t₃         t₄            t₅     t₆
          ┄┄┾━━━━━━━━━━┿━━━━━━━┿━━━━━━━━┿━━━━━━━━━━┿━━━━━━━━━━━━━┽┄┄┄┄┄┄┤
   f()      s()        l()     l()      l()        l()           l()    a()
   laps           1         2        3         4            5

 The timeline above shows a series of %LapTimer method invocations: `s()`,
 `l()`, and `a()`, representing start(), lap(), and average(), respectively.
 At each call of lap(), the duration elapsed between that call and the
 previous invocation is reported, but the clock never stops. Finally, the
 call to average() reports to the total time spanned by the invocation to
 start() and the _last_ call to lap() and divides it by the
 number of calls to lap() (i.e., `(t₅ - t₀) / 5`).

 This class does not store the times for the individual laps; if that
 granularity of information is needed, the caller should restore the returned
 lap times.

 The lap timer can also be used to measure the average duration of intervals by
 alternating calls to start() and lap().

   times    t₀         t₁      t₂       t₃         t₄            t₅     t₆
          ┄┄┾━━━━━━━━━━┽┄┄┄┄┄┄┄┾━━━━━━━━┽┄┄┄┄┄┄┄┄┄┄┾━━━━━━━━━━━━━┽┄┄┄┄┄┄┤
   f()      s()        l()     s()      l()        s()           l()    a()
   laps           1                  2                      3

 The timeline above shows the measurement of three, non-contiguous intervals,
 each bound by a pair of calls to start() and lap(). In this case, average()
 will report the average interval length as `(t₅ - t₄ + t₃ - t₂ + t₁ + t₀) / 3`.
 */
class LapTimer : public Timer {
 public:
  using Units = std::chrono::nanoseconds::period;

  LapTimer() = default;

  /**  Reports the time elapsed from the previous call to lap() or start() to
   this call.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template <typename Units>
  double lap() {
    auto now = std::chrono::high_resolution_clock::now();
    auto delta = now - start_;
    start_ = now;
    total_ += std::chrono::nanoseconds(delta);
    ++lapCount_;
    return std::chrono::duration<double, Units>(delta).count();
  }

  /**  Reports the average lap time across all recorded laps.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template <typename Units>
  double average() const {
    const double total_time =
        std::chrono::duration_cast<std::chrono::duration<double, Units>>(total_)
            .count();
    return total_time / lapCount_;
  }

  /**  Reports the total time measured.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template <typename Units>
  double total() const {
    return std::chrono::duration_cast<std::chrono::duration<double, Units>>(
               total_)
        .count();
  }

  /**  Reports the number of calls to laps.  */
  int laps() const { return lapCount_; }

 protected:
  //  The total accrued time of timed intervals (in nanoseconds).
  std::chrono::nanoseconds total_{0};

  //  The total number of calls to lap().
  int lapCount_{0};
};

}  // namespace common
}  // namespace drake

#ifdef ENABLE_TIMERS

/**  Creates a lap timer which uses the given label for display.

 @param  displayString  The string to display when reporting the profiling
                        results.
 @returns  The identifier for the created timer.  */
drake::common::TimerIndex addTimer(std::string displayString);

/**  Starts the timer with the given identifier.

 @param  index  The timer identifier supplied by addTimer.  */
void startTimer(drake::common::TimerIndex index);

/**  Stops the timer with the given identifier.

 @param  index  The timer identifier supplied by addTimer.  */
void stopTimer(drake::common::TimerIndex index);

/**  Lap the ith timer.

 @param  index  The timer identifier supplied by addTimer.  */
double lapTimer(drake::common::TimerIndex index);

double lapTimerSeconds(drake::common::TimerIndex index);
drake::common::TimerIndex PushTimer(drake::common::TimerIndex index);
void PopTimer();

void UpdateStack(double dt);

/**  Reports the average time of the ith timer in seconds.

 @param  index  The timer identifier supplied by addTimer.
 @returns  The average time of all laps.  */
double averageTimeInSec(drake::common::TimerIndex index);

/**  Creates a string representing a table of all of the averages.  */
std::string TableOfAverages();

// N.B. Static variables do not need to be captured in a lambda, they are
// global!
// https://stackoverflow.com/questions/13827855/capturing-a-static-variable-by-reference-in-a-c11-lambda
#define INSTRUMENT_FUNCTION(details_string)      \
  std::string description = __func__;            \
  description += "(): ";                         \
  description += details_string;                 \
  static const drake::common::TimerIndex timer = \
      addTimer(std::move(description));          \
  PushTimer(timer);                              \
  startTimer(timer);                             \
  drake::ScopeExit guard([]() {                  \
    const double dt = lapTimerSeconds(timer);    \
    UpdateStack(dt);                             \
    PopTimer();                                  \
  })

#else  // not defined ENABLE_TIMERS

#define addTimer(displayString) (common::TimerIndex(0))
#define startTimer(index) ((void)index)
#define stopTimer(index) ((void)index)
#define lapTimer(index) ((void)index)
#define PushTimer(index) ((void)index)
#define PopTimer()
#define averageTimeInSec(index) ((void)index)
#define TableOfAverages()    \
  ("Profiling turned off.\n" \
  "Enable profiling by running with --copt=-DENABLE_TIMERS.")
#define INSTRUMENT_FUNCTION(details_string) ((void)details_string)

#endif  // ENABLE_TIMERS
