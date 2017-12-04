#include "drake/common/text_logging.h"

#include <mutex>

#include "drake/common/never_destroyed.h"

namespace drake {

#ifdef HAVE_SPDLOG

namespace {
// Returns the default logger.  NOTE: This function assumes that it is mutexed,
// as in the initializer of a static local.
std::shared_ptr<logging::logger> onetime_create_log() {
  // Check if anyone has already set up a logger named "console".  If so, we
  // will just return it; if not, we'll create our own default one.
  std::shared_ptr<logging::logger> result(spdlog::get("console"));
  if (!result) {
    // We use the logger_mt (instead of logger_st) so more than one thread can
    // use this logger and have their messages be staggered by line, instead of
    // co-mingling their character bytes.
    result = spdlog::stderr_logger_mt("console");
  }
  return result;
}
}  // namespace

logging::logger* log() {
  static const never_destroyed<std::shared_ptr<logging::logger>> g_logger(
      onetime_create_log());
  return g_logger.access().get();
}

#else  // HAVE_SPDLOG

logging::logger::logger() {}

logging::logger* log() {
  // A do-nothing logger instance.
  static logging::logger g_logger;
  return &g_logger;
}

#endif  // HAVE_SPDLOG

}  // namespace drake
