#pragma once

#ifdef HAVE_SPDLOG

#include <spdlog/spdlog.h>

namespace drake {
namespace logging {

/** The severity level associated with a log message.
Specfic values are named like drake::logging::level::info, etc. */
using level_enum = spdlog::level::level_enum;

}  // namespace logging
}  // namespace drake

#else  // HAVE_SPDLOG

namespace drake {
namespace logging {

enum class level_enum {
  trace,
  debug,
  info,
  warn,
  err,
  critical,
  off,
};

}  // namespace logging
}  // namespace drake

#endif  // HAVE_SPDLOG

namespace drake {
namespace logging {
// We need to alias the values into a namespace, so that drake::logging::level
// maintains backwards compatibility.
namespace level {
/** The TRACE severity level associated with a log message. */
constexpr auto trace = level_enum::trace;
/** The DEBUG severity level associated with a log message. */
constexpr auto debug = level_enum::debug;
/** The INFO severity level associated with a log message. */
constexpr auto info = level_enum::info;
/** The WARNING severity level associated with a log message. */
constexpr auto warn = level_enum::warn;
/** The ERROR severity level associated with a log message. */
constexpr auto err = level_enum::err;
/** The CRITICAL severity level associated with a log message. */
constexpr auto critical = level_enum::critical;
/** The OFF severity level associated with a log message. */
constexpr auto off = level_enum::off;
}  // namespace level

}  // namespace logging
}  // namespace drake
