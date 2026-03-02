#pragma once

/** @file
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
(Note that the documentation link provides syntax for the latest version of
fmtlib; the version of fmtlib used by Drake might be older.)

When formatting an Eigen matrix into a string you must wrap the Eigen object
with fmt_eigen(); see its documentation for details. This holds true whether it
be for logging, error messages, etc.

When logging a third-party type whose only affordance for string output is
`operator<<`, use fmt_streamed(); see its documentation for details. This is
very rare (only a couple uses in Drake so far).

When implementing a string output for a Drake type, eventually this page will
demonstrate how to use fmt::formatter<T>. In the meantime, you can implement
`operator<<` and use drake::ostream_formatter, or else use the macro helper
DRAKE_FORMATTER_AS(). Grep around in Drake's existing code to find examples.

@warning This file should only be included from cc files, not header files.
Formatting a log messages is necessarily an expensive operation, so does not
meet our style guide rules for inline functions. (This is enforced in CI by
Drake's linter.) */

#include <string>

#include "drake/common/fmt.h"

#ifndef DRAKE_DOXYGEN_CXX
#ifdef HAVE_SPDLOG
#ifndef NDEBUG

// When in Debug builds, before including spdlog we set the compile-time
// minimum log threshold so that spdlog defaults to enabling all log levels.
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

// Provide operative macros only when spdlog is available and Debug is enabled.
#define DRAKE_LOGGER_TRACE(...)                                              \
  do {                                                                       \
    /* Capture the drake::log() in a temporary, using a relatively unique */ \
    /* variable name to avoid potential variable name shadowing warnings. */ \
    ::drake::logging::logger* const drake_spdlog_macro_logger_alias =        \
        ::drake::log();                                                      \
    if (drake_spdlog_macro_logger_alias->level() <= spdlog::level::trace) {  \
      SPDLOG_LOGGER_TRACE(drake_spdlog_macro_logger_alias, __VA_ARGS__);     \
    }                                                                        \
  } while (0)
#define DRAKE_LOGGER_DEBUG(...)                                              \
  do {                                                                       \
    /* Capture the drake::log() in a temporary, using a relatively unique */ \
    /* variable name to avoid potential variable name shadowing warnings. */ \
    ::drake::logging::logger* const drake_spdlog_macro_logger_alias =        \
        ::drake::log();                                                      \
    if (drake_spdlog_macro_logger_alias->level() <= spdlog::level::debug) {  \
      SPDLOG_LOGGER_DEBUG(drake_spdlog_macro_logger_alias, __VA_ARGS__);     \
    }                                                                        \
  } while (0)

#else

// Spdlog is available, but we are doing a non-Debug build.
#define DRAKE_LOGGER_TRACE(...)
#define DRAKE_LOGGER_DEBUG(...)

#endif

#include <spdlog/spdlog.h>

#endif  // HAVE_SPDLOG
#endif  // DRAKE_DOXYGEN_CXX

#include "drake/common/drake_copyable.h"

namespace drake {

#ifdef HAVE_SPDLOG
namespace logging {

// If we have spdlog, just alias logger into our namespace.
/** The drake::logging::logger class provides text logging methods.
See the text_logging.h documentation for a short tutorial. */
using logger = spdlog::logger;

/** True only if spdlog is enabled in this build. */
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(logger);

  logger();

  template <typename... Args>
  void trace(const Args&...) {}
  template <typename... Args>
  void debug(const Args&...) {}
  template <typename... Args>
  void info(const Args&...) {}
  template <typename... Args>
  void warn(const Args&...) {}
  template <typename... Args>
  void error(const Args&...) {}
  template <typename... Args>
  void critical(const Args&...) {}
};

}  // namespace logging

#define DRAKE_LOGGER_TRACE(...)
#define DRAKE_LOGGER_DEBUG(...)

#endif  // HAVE_SPDLOG

/** Retrieve an instance of a logger to use for logging; for example:
<pre>
  drake::log()->info("potato!")
</pre>

See the text_logging.h documentation for a short tutorial. */
logging::logger* log();

namespace logging {

/** When constructed, logs a message (at "warn" severity); the destructor is
guaranteed to be trivial.  This is useful for declaring an instance of this
class as a function-static global, so that a warning is logged the first time
the program encounters some code, but does not repeat the warning on subsequent
encounters within the same process.

For example:
<pre>
double* SanityCheck(double* data) {
  if (!data) {
    static const logging::Warn log_once("Bad data!");
    return alternative_data();
  }
  return data;
}
</pre> */
struct [[maybe_unused]] Warn {  // NOLINT(whitespace/braces)
  template <typename... Args>
  Warn(const char* a, const Args&... b) {
    // TODO(jwnimmer-tri) Ideally we would compile-time check our Warn format
    // strings without using fmt_runtime here, but I haven't figured out how
    // to forward the arguments properly for all versions of fmt.
    drake::log()->warn(fmt_runtime(a), b...);
  }
};

/** Sets the log threshold used by Drake's C++ code.
@param level Must be a string from spdlog enumerations: `trace`, `debug`,
`info`, `warn`, `err`, `critical`, `off`, or `unchanged` (not an enum, but
useful for command-line).
@return The string value of the previous log level. If SPDLOG is disabled, then
this returns an empty string. */
std::string set_log_level(const std::string& level);

/** The "unchanged" string to pass to set_log_level() so as to achieve a no-op.
 */
extern const char* const kSetLogLevelUnchanged;

/** An end-user help string suitable to describe the effects of set_log_level().
 */
extern const char* const kSetLogLevelHelpMessage;

/** Invokes `drake::log()->set_pattern(pattern)`.
@param pattern Formatting for message. For more information, see:
https://github.com/gabime/spdlog/wiki/3.-Custom-formatting */
void set_log_pattern(const std::string& pattern);

/** An end-user help string suitable to describe the effects of
set_log_pattern(). */
extern const char* const kSetLogPatternHelpMessage;

}  // namespace logging
}  // namespace drake
