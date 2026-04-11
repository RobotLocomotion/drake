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
If you want to log objects that are expensive to stringize, these macros will
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
`operator<<`, use fmt::streamed(); see its documentation for details. This is
very rare (only a couple uses in Drake so far).

To implement a string output for a Drake type, use the macro helper
DRAKE_FORMATTER_AS(). Grep around in Drake's existing code to find examples.

@warning This file should only be included from cc files, not header files.
Formatting a log messages is necessarily an expensive operation, so does not
meet our style guide rules for inline functions. (This is enforced in CI by
Drake's linter.) */

#include <string>
#include <string_view>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/fmt.h"
#include "drake/common/text_logging_level.h"

// Provide operative macros only in Debug builds, and exclude details from
// doxygen (the file overview already covered it).
#ifndef DRAKE_DOXYGEN_CXX
#ifndef NDEBUG

#define DRAKE_LOGGER_TRACE(...)                                               \
  do {                                                                        \
    /* Capture the drake::log() in a temporary, using a relatively unique */  \
    /* variable name to avoid potential variable name shadowing warnings. */  \
    auto* const drake_macro_logger_alias = ::drake::log();                    \
    if (drake_macro_logger_alias->should_log(drake::logging::level::trace)) { \
      drake_macro_logger_alias->trace(__VA_ARGS__);                           \
    }                                                                         \
  } while (0)
#define DRAKE_LOGGER_DEBUG(...)                                               \
  do {                                                                        \
    /* Capture the drake::log() in a temporary, using a relatively unique */  \
    /* variable name to avoid potential variable name shadowing warnings. */  \
    auto* const drake_macro_logger_alias = ::drake::log();                    \
    if (drake_macro_logger_alias->should_log(drake::logging::level::debug)) { \
      drake_macro_logger_alias->debug(__VA_ARGS__);                           \
    }                                                                         \
  } while (0)

#else  // NDEBUG

// The macros are no-ops in Release builds.
#define DRAKE_LOGGER_TRACE(...)
#define DRAKE_LOGGER_DEBUG(...)

#endif  // NDEBUG
#endif  // DRAKE_DOXYGEN_CXX

namespace drake {

#ifndef DRAKE_DOXYGEN_CXX
namespace logging {
class logger;  // Defined below.
}  // namespace logging
#endif

/** Retrieve Drake's singleton instance of the `class logger`; for example:
<pre>
  drake::log()->info("potato!")
</pre>

See the text_logging.h documentation for a short tutorial. */
logging::logger* log();

namespace logging {

/** The singleton class returned by Drake's drake::log() function, offering
functions to emit log messages. Refer to the file overview for details. */
class logger {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(logger);

  /** Logs a formatted message at `trace` severity. */
  template <typename... Args>
  void trace(fmt::format_string<Args...> pattern, Args&&... args) {
    this->log(level::trace, pattern, std::forward<Args>(args)...);
  }

  /** Logs a formatted message at `debug` severity. */
  template <typename... Args>
  void debug(fmt::format_string<Args...> pattern, Args&&... args) {
    this->log(level::debug, pattern, std::forward<Args>(args)...);
  }

  /** Logs a formatted message at `info` severity. */
  template <typename... Args>
  void info(fmt::format_string<Args...> pattern, Args&&... args) {
    this->log(level::info, pattern, std::forward<Args>(args)...);
  }

  /** Logs a formatted message at `warn` severity. */
  template <typename... Args>
  void warn(fmt::format_string<Args...> pattern, Args&&... args) {
    this->log(level::warn, pattern, std::forward<Args>(args)...);
  }

  /** Logs a formatted message at `error` severity. */
  template <typename... Args>
  void error(fmt::format_string<Args...> pattern, Args&&... args) {
    this->log(level::err, pattern, std::forward<Args>(args)...);
  }

  /** Logs a formatted message at `critical` severity. */
  template <typename... Args>
  void critical(fmt::format_string<Args...> pattern, Args&&... args) {
    this->log(level::critical, pattern, std::forward<Args>(args)...);
  }

  /** Logs a string at `trace` severity. */
  template <typename StringLike>
  void trace(const StringLike& message) {
    this->log(level::trace, std::string_view{message});
  }

  /** Logs a string at `debug` severity. */
  template <typename StringLike>
  void debug(const StringLike& message) {
    this->log(level::debug, std::string_view{message});
  }

  /** Logs a string at `info` severity. */
  template <typename StringLike>
  void info(const StringLike& message) {
    this->log(level::info, std::string_view{message});
  }

  /** Logs a string at `warn` severity. */
  template <typename StringLike>
  void warn(const StringLike& message) {
    this->log(level::warn, std::string_view{message});
  }

  /** Logs a string at `error` severity. */
  template <typename StringLike>
  void error(const StringLike& message) {
    this->log(level::err, std::string_view{message});
  }

  /** Logs a string at `critival` severity. */
  template <typename StringLike>
  void critical(const StringLike& message) {
    this->log(level::critical, std::string_view{message});
  }

  /** Logs a formatted message at the given `severity`. */
  template <typename... Args>
  void log(level_enum severity, fmt::format_string<Args...> pattern,
           Args&&... args) {
    if (should_log(severity)) {
      this->LogMessage(severity,
                       fmt::format(pattern, std::forward<Args>(args)...));
    }
  }

  /** Logs a string at the given `severity`. */
  void log(level_enum severity, std::string_view message) {
    this->LogMessage(severity, message);
  }

  /** Returns true iff the current level() threshold meets the given `severity`.
   */
  bool should_log(level_enum severity) const;

  /** Returns the current log level. */
  level_enum level() const;

  /** Sets the currently log level. */
  void set_level(level_enum severity) const;

  /** Invokes `spdlog::logger::set_pattern(pattern)`.
  This has no effect unless spdlog is enabled.
  @param pattern Formatting for message. For more information, see:
  https://github.com/gabime/spdlog/wiki/3.-Custom-formatting */
  void set_pattern(std::string_view pattern);

 private:
  friend logging::logger* drake::log();

  logger();
  ~logger();

  void LogMessage(level_enum severity, std::string_view message);

  // The runtime type of `impl_` depends on which `text_logging_impl_*.cc` file
  // is selected to be compiled.
  void* const impl_;
};

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
  explicit Warn(std::string_view message) { drake::log()->warn(message); }

  template <typename... Args>
  Warn(fmt::format_string<Args...> pattern, Args&&... args)
    requires(sizeof...(args) > 0)
  {
    drake::log()->warn(pattern, std::forward<Args>(args)...);
  }
};

/** Sets the log threshold used by Drake's C++ code.
This has no effect unless spdlog is enabled.
@param level Must be a string from level enumeration: `trace`, `debug`, `info`,
`warn`, `err`, `critical`, `off`, or `unchanged` (not an enum, but useful for
command-line).
@return The string value of the previous log level. */
std::string set_log_level(const std::string& level);

/** The "unchanged" string to pass to set_log_level() so as to achieve a no-op.
 */
extern const char* const kSetLogLevelUnchanged;

/** An end-user help string suitable to describe the effects of set_log_level().
 */
extern const char* const kSetLogLevelHelpMessage;

/** Invokes `drake::log()->set_pattern(pattern)`.
This has no effect unless spdlog is enabled.
@param pattern Formatting for message. For more information, see:
https://github.com/gabime/spdlog/wiki/3.-Custom-formatting */
void set_log_pattern(const std::string& pattern);

/** An end-user help string suitable to describe the effects of
set_log_pattern(). */
extern const char* const kSetLogPatternHelpMessage;

#ifndef DRAKE_DOXYGEN_CXX
// DRAKE DEPRECATED: The preprocessor definition HAVE_SPDLOG is deprecated
// and will be removed from Drake on or after 2026-07-01.
#ifdef HAVE_SPDLOG
[[deprecated(
    "DRAKE DEPRECATED: kHaveSpdlog is deprecated and will be removed from "
    "Drake on or after 2026-07-01.")]]
constexpr bool kHaveSpdlog = true;
#else
[[deprecated(
    "DRAKE DEPRECATED: kHaveSpdlog is deprecated and will be removed from "
    "Drake on or after 2026-07-01.")]]
constexpr bool kHaveSpdlog = false;
#endif
#endif

}  // namespace logging
}  // namespace drake
