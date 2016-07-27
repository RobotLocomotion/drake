#pragma once

#include <memory>

#ifdef HAVE_SPDLOG
#ifndef NDEBUG
#define SPDLOG_DEBUG_ON 1
#define SPDLOG_TRACE_ON 1
#endif
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#undef SPDLOG_DEBUG_ON
#undef SPDLOG_TRACE_ON
#endif

#include "drake/drakeCommon_export.h"

namespace drake {

#ifdef HAVE_SPDLOG
// If we have spdlog, just alias logger into our namespace.
namespace logging {
using spdlog::logger;
}

#else  // HAVE_SPDLOG
// If we don't have spdlog, we need to stub out logger.

namespace logging {

/// A stubbed-out version of `spdlog::logger`.  Implements only those methods
/// that we expect to use, as spdlog's API does change from time to time.
class logger {
 public:
  template <typename... Args>
  void trace(const char* fmt, const Args&... args) {};
  template <typename... Args>
  void debug(const char* fmt, const Args&... args) {};
  template <typename... Args>
  void info(const char* fmt, const Args&... args) {};
  template <typename... Args>
  void warn(const char* fmt, const Args&... args) {};
  template <typename... Args>
  void error(const char* fmt, const Args&... args) {};
  template <typename... Args>
  void critical(const char* fmt, const Args&... args) {};

  template <typename T> void trace(const T&) {};
  template <typename T> void debug(const T&) {};
  template <typename T> void info(const T&) {};
  template <typename T> void warn(const T&) {};
  template <typename T> void error(const T&) {};
  template <typename T> void critical(const T&) {};
};

} // namespace logging

#define SPDLOG_TRACE(logger, ...)
#define SPDLOG_DEBUG(logger, ...)

#endif  // HAVE_SPDLOG

/// Retrieve an instance of a logger to use for logging; for example:
///   `drake::log()->info("potato!")`
std::shared_ptr<logging::logger> log();

} // namespace drake
