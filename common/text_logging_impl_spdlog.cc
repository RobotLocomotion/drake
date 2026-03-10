#include "drake/common/text_logging_impl_spdlog.h"

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace {

spdlog::level::level_enum FromDrakeLevel(drake::logging::level_enum x) {
  switch (x) {
    case drake::logging::level::trace:
      return spdlog::level::trace;
    case drake::logging::level::debug:
      return spdlog::level::debug;
    case drake::logging::level::info:
      return spdlog::level::info;
    case drake::logging::level::warn:
      return spdlog::level::warn;
    case drake::logging::level::err:
      return spdlog::level::err;
    case drake::logging::level::critical:
      return spdlog::level::critical;
    case drake::logging::level::off:
      return spdlog::level::off;
  }
  // For simplicity in linking, we do not use `DRAKE_UNREACHABLE`.
  throw std::runtime_error("FromDrakeLevel should not reach here!");
}

drake::logging::level_enum ToDrakeLevel(spdlog::level::level_enum x) {
  switch (x) {
    case spdlog::level::trace:
      return drake::logging::level::trace;
    case spdlog::level::debug:
      return drake::logging::level::debug;
    case spdlog::level::info:
      return drake::logging::level::info;
    case spdlog::level::warn:
      return drake::logging::level::warn;
    case spdlog::level::err:
      return drake::logging::level::err;
    case spdlog::level::critical:
      return drake::logging::level::critical;
    case spdlog::level::off:
      return drake::logging::level::off;
    case spdlog::level::n_levels:
      break;
  }
  // For simplicity in linking, we do not use `DRAKE_UNREACHABLE`.
  throw std::runtime_error("ToDrakeLevel should not reach here!");
}

// Returns the default logger.  NOTE: This function assumes that it is mutexed,
// as in the initializer of a static local.
std::shared_ptr<spdlog::logger> make_spdlog_logger_singleton() {
  // Check if anyone has already set up a logger named "console".  If so, we
  // will just return it; if not, we'll create our own default one.
  std::shared_ptr<spdlog::logger> result(spdlog::get("console"));
  if (!result) {
    // We wrap our stderr sink in a dist_sink so that users can atomically swap
    // out the sinks used by all Drake logging, via dist_sink_mt's APIs.
    auto wrapper = std::make_shared<spdlog::sinks::dist_sink_mt>();
    // We use the stderr_sink_mt (instead of stderr_sink_st) so more than one
    // thread can use this logger and have their messages be staggered by line,
    // instead of co-mingling their character bytes.
    wrapper->add_sink(std::make_shared<spdlog::sinks::stderr_sink_mt>());
    result = std::make_shared<spdlog::logger>("console", std::move(wrapper));
    result->set_level(spdlog::level::info);
  }
  return result;
}

}  // namespace

namespace drake {
namespace internal {

spdlog::logger* get_spdlog_logger_singleton() {
  static const never_destroyed<std::shared_ptr<spdlog::logger>> g_logger(
      make_spdlog_logger_singleton());
  return g_logger.access().get();
}

}  // namespace internal

namespace logging {

logger::logger() : impl_{internal::get_spdlog_logger_singleton()} {}

logger::~logger() = default;

bool logger::should_log(level_enum severity) const {
  const auto* const impl = static_cast<const spdlog::logger*>(impl_);
  return impl->should_log(FromDrakeLevel(severity));
}

level_enum logger::level() const {
  const auto* const impl = static_cast<const spdlog::logger*>(impl_);
  return ToDrakeLevel(impl->level());
}

void logger::set_level(level_enum severity) const {
  auto* const impl = static_cast<spdlog::logger*>(impl_);
  return impl->set_level(FromDrakeLevel(severity));
}

void logger::set_pattern(std::string_view pattern) {
  auto* const impl = static_cast<spdlog::logger*>(impl_);
  impl->set_pattern(std::string{pattern});
}

void logger::LogUnconditionally(level_enum severity, std::string_view message) {
  auto* const impl = static_cast<spdlog::logger*>(impl_);
  return impl->log(FromDrakeLevel(severity), message);
}

}  // namespace logging
}  // namespace drake
