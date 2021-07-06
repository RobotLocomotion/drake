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
  DRAKE_LOGGER_TRACE("message: {}", something_conditionally_compiled);
  DRAKE_LOGGER_DEBUG("message: {}", something_conditionally_compiled);
</pre>

The format string syntax is fmtlib; see https://fmt.dev/latest/syntax.html.
In particular, any class that overloads `operator<<` for `ostream` can be
printed without any special handling.  (Note that the documentation link
provides syntax for the latest version of fmtlib; the version of fmtlib
used by Drake might be older.)
*/

#include <string>

#ifndef DRAKE_DOXYGEN_CXX
#ifdef HAVE_SPDLOG
#ifndef NDEBUG

// When in Debug builds, before including spdlog we set the compile-time
// minimum log threshold so that spdlog defaults to enabling all log levels.
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

// Provide operative macros only when spdlog is available and Debug is enabled.
#define DRAKE_LOGGER_TRACE(...)                                               \
  do {                                                                        \
    /* Capture the drake::log() in a temporary, using a relatively unique */  \
    /* variable name to avoid potential variable name shadowing warnings. */  \
    ::drake::logging::logger* const drake_spdlog_macro_logger_alias =         \
        ::drake::log();                                                       \
    if (drake_spdlog_macro_logger_alias->level() <= spdlog::level::trace) {   \
      SPDLOG_LOGGER_TRACE(drake_spdlog_macro_logger_alias, __VA_ARGS__);      \
    }                                                                         \
  } while (0)
#define DRAKE_LOGGER_DEBUG(...)                                               \
  do {                                                                        \
    /* Capture the drake::log() in a temporary, using a relatively unique */  \
    /* variable name to avoid potential variable name shadowing warnings. */  \
    ::drake::logging::logger* const drake_spdlog_macro_logger_alias =         \
        ::drake::log();                                                       \
    if (drake_spdlog_macro_logger_alias->level() <= spdlog::level::debug) {   \
      SPDLOG_LOGGER_DEBUG(drake_spdlog_macro_logger_alias, __VA_ARGS__);      \
    }                                                                         \
  } while (0)

#else

// Spdlog is available, but we are doing a non-Debug build.
#define DRAKE_LOGGER_TRACE(...)
#define DRAKE_LOGGER_DEBUG(...)

#endif

/* clang-format off */
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
/* clang-format on */

#else  // HAVE_SPDLOG

// We always want text_logging.h to provide fmt support to those who include
// it, even if spdlog is disabled.

/* clang-format off */
#include <fmt/format.h>
#include <fmt/ostream.h>
/* clang-format on */

#endif  // HAVE_SPDLOG
#endif  // DRAKE_DOXYGEN_CXX

#include "drake/common/drake_copyable.h"

namespace drake {

#ifdef HAVE_SPDLOG
namespace logging {

// If we have spdlog, just alias logger into our namespace.
/// The drake::logging::logger class provides text logging methods.
/// See the text_logging.h documentation for a short tutorial.
using logger = spdlog::logger;

/// When spdlog is enabled in this build, drake::logging::sink is an alias for
/// spdlog::sinks::sink.  When spdlog is disabled, it is an empty class.
using spdlog::sinks::sink;

/// True only if spdlog is enabled in this build.
constexpr bool kHaveSpdlog = true;

}  // namespace logging

#else  // HAVE_SPDLOG
// If we don't have spdlog, we need to stub out logger.

namespace logging {
constexpr bool kHaveSpdlog = false;

// A stubbed-out version of `spdlog::logger`.  Implements only those methods
// that we expect to use, as spdlog's API does change from time to time.
class logger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(logger)

  logger();

  template <typename... Args>
  void trace(const char*, const Args&...) {}
  template <typename... Args>
  void debug(const char*, const Args&...) {}
  template <typename... Args>
  void info(const char*, const Args&...) {}
  template <typename... Args>
  void warn(const char*, const Args&...) {}
  template <typename... Args>
  void error(const char*, const Args&...) {}
  template <typename... Args>
  void critical(const char*, const Args&...) {}

  template <typename T> void trace(const T&) {}
  template <typename T> void debug(const T&) {}
  template <typename T> void info(const T&) {}
  template <typename T> void warn(const T&) {}
  template <typename T> void error(const T&) {}
  template <typename T> void critical(const T&) {}
};

// A stubbed-out version of `spdlog::sinks::sink`.
class sink {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(sink)

  sink();
};

}  // namespace logging

#define DRAKE_LOGGER_TRACE(...)
#define DRAKE_LOGGER_DEBUG(...)

#endif  // HAVE_SPDLOG

/// Retrieve an instance of a logger to use for logging; for example:
/// <pre>
///   drake::log()->info("potato!")
/// </pre>
///
/// See the text_logging.h documentation for a short tutorial.
logging::logger* log();

namespace logging {

/// (Advanced) Retrieves the default sink for all Drake logs.  When spdlog is
/// enabled, the return value can be cast to spdlog::sinks::dist_sink_mt and
/// thus allows consumers of Drake to redirect Drake's text logs to locations
/// other than the default of stderr.  When spdlog is disabled, the return
/// value is an empty class.
sink* get_dist_sink();

/// When constructed, logs a message (at "warn" severity); the destructor is
/// guaranteed to be trivial.  This is useful for declaring an instance of this
/// class as a function-static global, so that a warning is logged the first
/// time the program encounters some code, but does not repeat the warning on
/// subsequent encounters within the same process.
///
/// For example:
/// <pre>
/// double* SanityCheck(double* data) {
///   if (!data) {
///     static const logging::Warn log_once("Bad data!");
///     return alternative_data();
///   }
///   return data;
/// }
/// </pre>
struct Warn {
  template <typename... Args>
  Warn(const char* a, const Args&... b) {
    drake::log()->warn(a, b...);
  }
};

/// Invokes `drake::log()->set_level(level)`.
/// @param level Must be a string from spdlog enumerations: `trace`, `debug`,
/// `info`, `warn`, `err`, `critical`, `off`, or `unchanged` (not an enum, but
/// useful for command-line).
/// @return The string value of the previous log level. If SPDLOG is disabled,
/// then this returns an empty string.
std::string set_log_level(const std::string& level);

/// The "unchanged" string to pass to set_log_level() so as to achieve a no-op.
extern const char* const kSetLogLevelUnchanged;

/// An end-user help string suitable to describe the effects of set_log_level().
extern const char* const kSetLogLevelHelpMessage;

/// Invokes `drake::log()->set_pattern(pattern)`.
/// @param pattern Formatting for message. For more information, see:
/// https://github.com/gabime/spdlog/wiki/3.-Custom-formatting
void set_log_pattern(const std::string& pattern);

/// An end-user help string suitable to describe the effects of
/// set_log_pattern().
extern const char* const kSetLogPatternHelpMessage;

}  // namespace logging
}  // namespace drake
