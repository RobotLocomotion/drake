#pragma once

namespace drake {
namespace logging {

// Ideally we would have named this just `enum class level`, but then the
// logger::level() function would end up shadowing us.
/** The severity level associated with a log message.
Specfic values are named like drake::logging::level::info, etc. */
enum class level_enum {
  trace,
  debug,
  info,
  warn,
  err,
  critical,
  off,
};

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
