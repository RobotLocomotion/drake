#include "drake/common/text_logging_impl_simple.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string_view>
#include <utility>

#include <fmt/chrono.h>

#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace logging {
namespace {

static internal::SinkFunction& GetSinkFunctionSingleton() {
  static never_destroyed<internal::SinkFunction> sink_;
  return sink_.access();
}

// Writes the given text to stderr. We use fwrite (from C) instead of cerr (from
// C++) because it is significantly faster.
void WriteToStderr(std::string_view text) {
  const size_t size = 1;
  const size_t count = text.size();
  const size_t written = std::fwrite(text.data(), size, count, stderr);
  static_cast<void>(written);  // unused
}

constexpr std::array<std::string_view, 7> kLevelStrings = {
    "trace",     // BR
    "debug",     // BR
    "info",      // BR
    "warning",   // BR
    "error",     // BR
    "critical",  // BR
    "off",       // BR
};

class logger_impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(logger_impl);
  logger_impl() = default;
  level_enum level() const { return level_.load(std::memory_order_relaxed); }
  void set_level(level_enum severity) { level_ = severity; }
  bool should_log(level_enum severity) const { return severity >= level(); }
  void LogMessage(level_enum severity, std::string_view message) {
    if (!should_log(severity)) {
      return;
    }
    if (sink_ != nullptr) {
      sink_(severity, message);
      return;
    }
    // TODO(jwnimmer-tri) Once we drop support for fmt 9, we should be able to
    // simplify this using the built-in formatting instead of extracting millis
    // manually.
    const auto now = std::chrono::system_clock::now();
    const auto now_rounded_to_whole_seconds =
        std::chrono::system_clock::to_time_t(now);
    const auto remainder_millis =
        std::chrono::time_point_cast<std::chrono::milliseconds>(now)
            .time_since_epoch()
            .count() %
        1000;
    WriteToStderr(fmt::format(
        "[{:%Y-%m-%d %H:%M:%S}.{:03d}] [console] [{}] {}\n",
        fmt::localtime(now_rounded_to_whole_seconds), remainder_millis,
        kLevelStrings[static_cast<int>(severity)], message));
  }

 private:
  std::atomic<level_enum> level_{level::info};
  internal::SinkFunction& sink_{GetSinkFunctionSingleton()};
};

}  // namespace

logger::logger() : impl_{new logger_impl} {}

logger::~logger() = default;

bool logger::should_log(level_enum severity) const {
  const auto* const impl = static_cast<const logger_impl*>(impl_);
  return impl->should_log(severity);
}

level_enum logger::level() const {
  const auto* const impl = static_cast<const logger_impl*>(impl_);
  return impl->level();
}

void logger::set_level(level_enum severity) const {
  auto* const impl = static_cast<logger_impl*>(impl_);
  return impl->set_level(severity);
}

void logger::set_pattern(std::string_view /* pattern */) {}

void logger::LogMessage(level_enum severity, std::string_view message) {
  auto* const impl = static_cast<logger_impl*>(impl_);
  return impl->LogMessage(severity, message);
}

namespace internal {

void ReplaceSinkFunction(SinkFunction function) {
  GetSinkFunctionSingleton() = std::move(function);
}

}  // namespace internal
}  // namespace logging

const char* const logging::kSetLogPatternHelpMessage =
    "does nothing (spdlog is not enabled in this build)";

}  // namespace drake
