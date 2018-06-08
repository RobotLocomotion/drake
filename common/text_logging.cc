#include "drake/common/text_logging.h"

#include <mutex>

#include "drake/common/never_destroyed.h"

namespace drake {

void SetLoggerLevel(const std::shared_ptr<logging::logger>& logger_ptr,
                    const std::string& logger_level) {
  const bool want_unchanged = (logger_level == "unchanged");
  const bool want_trace = (logger_level == "trace");
  const bool want_debug = (logger_level == "debug");
  const bool want_info = (logger_level == "info");
  const bool want_warn = (logger_level == "warn");
  const bool want_err = (logger_level == "err");
  const bool want_critical = (logger_level == "critical");
  const bool want_off = (logger_level == "off");
  if (
        !want_unchanged &&
        !want_trace &&
        !want_debug &&
        !want_info &&
        !want_warn &&
        !want_err &&
        !want_critical &&
        !want_off
      ) {
    logger_ptr->critical("Unknown spdlog_level {}", logger_level);
    throw std::runtime_error("Unknown spdlog level");
  }
#ifdef HAVE_SPDLOG
  if (want_trace) {
    logger_ptr->set_level(spdlog::level::trace);
  } else if (want_debug) {
    logger_ptr->set_level(spdlog::level::debug);
  } else if (want_info) {
    logger_ptr->set_level(spdlog::level::info);
  } else if (want_warn) {
    logger_ptr->set_level(spdlog::level::warn);
  } else if (want_err) {
    logger_ptr->set_level(spdlog::level::err);
  } else if (want_critical) {
    logger_ptr->set_level(spdlog::level::critical);
  } else if (want_off) {
    logger_ptr->set_level(spdlog::level::off);
  }
#endif
}

#ifdef HAVE_SPDLOG

namespace {
// Returns a named logger.  NOTE: This function assumes that it is mutexed,
// as in the initializer of a static local.
std::shared_ptr<logging::logger> ReturnLoggerAlwaysNamed(
        const std::string& logger_name) {
  // Check if anyone has already set up a logger named logger_name. If so, we
  // will just return it; if not, we'll create our own.
  std::shared_ptr<logging::logger> logger_ptr(spdlog::get(logger_name));
  if (!logger_ptr) {
    // We use the logger_mt (instead of logger_st) so more than one thread can
    // use this logger and have their messages be staggered by line, instead of
    // co-mingling their character bytes.
    logger_ptr = spdlog::stderr_logger_mt(logger_name);
  }
  return logger_ptr;
}
}  // namespace

std::shared_ptr<logging::logger> ReturnDefaultLogger() {
  return ReturnLoggerAlwaysNamed("console");
}

logging::logger* log() {
  static const never_destroyed<std::shared_ptr<logging::logger>> g_logger(
          ReturnDefaultLogger());
  return g_logger.access().get();
}

std::shared_ptr<logging::logger> ReturnLoggerPreferablyNamed(
        const std::string& logger_name) {
  std::shared_ptr<logging::logger> logger_ptr = spdlog::get(logger_name);
  if (!logger_ptr) {
    logger_ptr = ReturnDefaultLogger();
  }
  return logger_ptr;
}

std::shared_ptr<logging::logger> MakeLoggerSetLevel(
        const std::string& logger_name,
        const std::string& logger_level) {
  std::shared_ptr<logging::logger> logger_ptr = ReturnLoggerAlwaysNamed(
          logger_name);
  SetLoggerLevel(logger_ptr, logger_level);
  return logger_ptr;
}

#else  // HAVE_SPDLOG

logging::logger::logger() {}

logging::logger* log() {
  // A do-nothing logger instance.
  static logging::logger g_logger;
  return &g_logger;
}

std::shared_ptr<logging::logger> ReturnLoggerPreferablyNamed(
        const std::string& logger_name) {
  std::shared_ptr<logging::logger> g_logger;
  return g_logger;
}

std::shared_ptr<logging::logger> MakeLoggerSetLevel(
        const std::string& logger_name,
        const std::string& logger_level) {
  std::shared_ptr<logging::logger> g_logger;
  return g_logger;
}

std::shared_ptr<logging::logger> ReturnDefaultLogger() {
  std::shared_ptr<logging::logger> g_logger;
  return g_logger;
}

#endif  // HAVE_SPDLOG

}  // namespace drake
