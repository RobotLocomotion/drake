#include "drake/common/text_logging.h"

#include <mutex>

namespace drake {

#ifdef HAVE_SPDLOG

namespace {
// Returns a new pointer (caller-owned) to a shared_ptr to a logger.
//
// NOTE: This function assumes that it is mutexed, as in the initializer of
// a static local.
std::shared_ptr<logging::logger>* onetime_create_log() {
  std::shared_ptr<logging::logger>* result =
      new std::shared_ptr<logging::logger>(spdlog::get("console"));
  if (!*result) {
    *result = spdlog::stderr_logger_st("console");
  }
  return result;
}
}  // namespace

logging::logger* log() {
  // The following line creates a static shared_ptr to the logger; this
  // guarantees the underling logger object will not be freed.
  static const std::shared_ptr<logging::logger>* const g_logger =
      onetime_create_log();
  return g_logger->get();
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
