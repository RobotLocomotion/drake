#include "planning/parallelism.h"

#include <charconv>
#include <cstdlib>
#include <thread>
#include <type_traits>

#include "drake/common/text_logging.h"

namespace anzu {
namespace planning {
namespace {
constexpr char kMaxThreadsEnvVar[] = "DRAKE_NUM_THREADS";

// Helper function to get the maximum number of threads as (potentially)
// specified by the user via environment variable.
int DoGetMaxNumThreads() {
  const int hardware_concurrency = std::thread::hardware_concurrency();

  const char* const max_threads_var = std::getenv(kMaxThreadsEnvVar);

  if (max_threads_var == nullptr) {
    drake::log()->debug(
        "Environment variable {} not found, initializing max threads from "
        " hardware concurrency {}",
        kMaxThreadsEnvVar, hardware_concurrency);
    return hardware_concurrency;
  }

  int max_threads = 0;
  const std::string_view max_threads_var_view(max_threads_var);
  const char* const first = max_threads_var_view.data();
  const char* const last = first + max_threads_var_view.size();
  const auto [ptr, error] = std::from_chars(first, last, max_threads);

  // Did we fail to parse the environment variable, or did it contain
  // additional unparsed text?
  if (ptr != last || error != std::errc()) {
    drake::log()->error(
        "Failed to parse environment variable {} with value {}, falling back "
        "to initializing max threads from hardware concurrency {}",
        kMaxThreadsEnvVar, max_threads_var, hardware_concurrency);
    return hardware_concurrency;
  }

  // Is the value outside the allowed range [1, hardware_concurrency]?
  if (max_threads < 1 || max_threads > hardware_concurrency) {
    drake::log()->warn(
        "Provided value {} of environment variable {} is out of range [1, {}], "
        "falling back to initializing max threads from hardware concurrency {}",
        max_threads, kMaxThreadsEnvVar, hardware_concurrency,
        hardware_concurrency);
    return hardware_concurrency;
  }

  drake::log()->debug(
      "Initializing max threads to {} from environment variable {}",
      max_threads, kMaxThreadsEnvVar);
  return max_threads;
}

// Helper function to retrieve the maximum number of threads. The value is
// latched the first time this function is called.
int GetMaxNumThreads() {
  static const int max_num_threads = DoGetMaxNumThreads();
  return max_num_threads;
}

}  // namespace

Parallelism::Parallelism(const bool parallelize) {
  if (parallelize) {
    num_threads_ = GetMaxNumThreads();
  } else {
    num_threads_ = 1;
  }
}

}  // namespace planning
}  // namespace anzu
