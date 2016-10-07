#pragma once

/**
@file
This is the entry point for all text logging within Drake.
Once you've included this file, the suggested ways you
should write log messages include:
<pre>
  drake::log()->trace("Some trace message: {} {}", something, some_other);
</pre>
Similarly, it provides:
<pre>
  drake::log()->debug(...);
  drake::log()->info(...);
  drake::log()->warn(...);
  drake::log()->error(...);
  drake::log()->critical(...);
</pre>
If you want to log objects that are expensive to serialize, these macros will
not be compiled if debugging is turned off (-DNDEBUG is set):
<pre>
  SPDLOG_TRACE(drake::log(), "message: {}", something_conditionally_compiled);
  SPDLOG_DEBUG(drake::log(), "message: {}", something_conditionally_compiled);
</pre>
The format string syntax is fmtlib; see http://fmtlib.net/3.0.0/syntax.html.
In particular, any class that overloads `operator<<` for `ostream` can be
printed without any special handling.
*/

#include <memory>

#ifdef HAVE_SPDLOG
// Before including spdlog, activate the SPDLOG_DEBUG and SPDLOG_TRACE macros
// if and only if Drake is being compiled in debug mode.  When not in debug
// mode, they are no-ops and their arguments are not evaluated.
#ifndef NDEBUG
#define SPDLOG_DEBUG_ON 1
#define SPDLOG_TRACE_ON 1
#endif
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#endif

#include "drake/common/drake_export.h"

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
class DRAKE_EXPORT logger {
 public:
  logger();

  logger(const logger&) = delete;
  logger& operator=(const logger&) = delete;

  template <typename... Args>
  void trace(const char* fmt, const Args&... args) {}
  template <typename... Args>
  void debug(const char* fmt, const Args&... args) {}
  template <typename... Args>
  void info(const char* fmt, const Args&... args) {}
  template <typename... Args>
  void warn(const char* fmt, const Args&... args) {}
  template <typename... Args>
  void error(const char* fmt, const Args&... args) {}
  template <typename... Args>
  void critical(const char* fmt, const Args&... args) {}

  template <typename T> void trace(const T&) {}
  template <typename T> void debug(const T&) {}
  template <typename T> void info(const T&) {}
  template <typename T> void warn(const T&) {}
  template <typename T> void error(const T&) {}
  template <typename T> void critical(const T&) {}
};

}  // namespace logging

#define SPDLOG_TRACE(logger, ...)
#define SPDLOG_DEBUG(logger, ...)

#endif  // HAVE_SPDLOG

/// Retrieve an instance of a logger to use for logging; for example:
///   `drake::log()->info("potato!")`
DRAKE_EXPORT logging::logger* log();

}  // namespace drake
