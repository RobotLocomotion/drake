/** Example of profiling subprograms, using the C++ `perf` controller.

    See README.md for usage.
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

// Include this header to use the PerfController.
#include "drake/tools/performance/perf_controller.h"

namespace drake {
namespace tools {
namespace performance {

// Define a bunch of busy work for the profiler to sample, with distinct call
// stacks.

double busy_work(std::function<double(double)> fn) {
  const auto start = std::chrono::high_resolution_clock::now();
  double result{};
  for (int k = 0;; ++k) {
    result = fn(0.001 * k);
    const auto now = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration<double, std::milli>(now - start);
    if (duration.count() > 1000.0) {
      break;
    }
  }
  return result;
}
double busy_work_1() {
  return busy_work([](double x) {
    return std::sin(x);
  });
}

double busy_work_2() {
  return busy_work([](double x) {
    return std::cos(x);
  });
}

double busy_work_3() {
  return busy_work([](double x) {
    return std::tan(x);
  });
}

double busy_work_4() {
  return busy_work([](double x) {
    return std::asin(x);
  });
}

double busy_work_5() {
  return busy_work([](double x) {
    return std::acos(x);
  });
}

namespace {

int do_main(int, const char*[]) {
  // Get access to the singleton controller.
  auto& controller = ThePerfController();

  // We can check if control is available, that is, if the communication scheme
  // provided by perf_controlled_record.py was set up.
  //
  // This check-and-fail isn't necessary in most programs, but here we want to
  // make sure controller commands are doing something.
  //
  // If control is not available, the controller commands will just be
  // no-ops. For most programs, it will be convenient to run them with
  // instrumentation in place, but with or without the perf control setup.
  if (!controller.is_control_available()) {
    throw std::runtime_error("perf control is unavailable!");
  }

  // Note that sampling is initially turned off when running under
  // perf_controlled_record.py.

  busy_work_1();

  // We can turn sampling on (resume) and off (pause) manually.
  controller.resume();
  busy_work_2();
  controller.pause();

  busy_work_3();

  {
    // We can use a ScopedPerfSampling instance to turn sampling on for the
    // duration of a scope.
    ScopedPerfSampling scoped;
    busy_work_4();
  }

  busy_work_5();

  return 0;
}

}  // namespace
}  // namespace performance
}  // namespace tools
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::tools::performance::do_main(argc, argv);
}
