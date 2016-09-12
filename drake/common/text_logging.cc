#include "drake/common/text_logging.h"

#include <mutex>
#include <stdexcept>

#ifndef _MSC_VER
#include <gflags/gflags.h>

// Declare the --spdlog_level gflags option.
namespace {
constexpr const char kUnchanged[] = "unchanged";
constexpr const char kTrace[] = "trace";
constexpr const char kDebug[] = "debug";
}  // namespace
DEFINE_string(spdlog_level, kUnchanged,
              "sets the spdlog output threshold; "
              "possible values are 'unchanged', 'trace', 'debug'");

// Check the gflags settings for validity.  If spdlog is enabled, update its
// configuration to match the flags.
namespace {
void maybe_process_gflags(drake::logging::logger* logger_to_update) {
  const bool want_unchanged = (FLAGS_spdlog_level == kUnchanged);
  const bool want_trace = (FLAGS_spdlog_level == kTrace);
  const bool want_debug = (FLAGS_spdlog_level == kDebug);
  if (!want_unchanged && !want_trace && !want_debug) {
    logger_to_update->critical("Unknown spdlog_level {}", FLAGS_spdlog_level);
    throw std::runtime_error("Unknown spdlog level");
  }
#ifdef HAVE_SPDLOG
  if (want_trace) {
    logger_to_update->set_level(spdlog::level::trace);
  } else if (want_debug) {
    logger_to_update->set_level(spdlog::level::debug);
  }
#endif
}
}  // namespace

#else  // _MSC_VER

// For the moment, we can't get libgflags.dll working within Drake.
// Thus, we have to omit the gflags integration glue on windows.
namespace {
void maybe_process_gflags(drake::logging::logger*) {}
}  // namespace

#endif

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
    // We use the logger_mt (instead of logger_st) so more than one thread can
    // use this logger and have their messages be staggered by line, instead of
    // co-mingling their character bytes.
    *result = spdlog::stderr_logger_mt("console");
    maybe_process_gflags(result->get());
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

logging::logger::logger() {
  maybe_process_gflags(this);
}

logging::logger* log() {
  // A do-nothing logger instance.
  static logging::logger g_logger;
  return &g_logger;
}

#endif  // HAVE_SPDLOG

}  // namespace drake
