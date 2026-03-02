#include "drake/common/text_logging.h"

#include <memory>
#include <mutex>
#include <utility>

#ifdef HAVE_SPDLOG
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#endif

#include "drake/common/never_destroyed.h"

namespace drake {

#ifdef HAVE_SPDLOG

namespace {
// Returns the default logger.  NOTE: This function assumes that it is mutexed,
// as in the initializer of a static local.
std::shared_ptr<logging::logger> onetime_create_log() {
  // Check if anyone has already set up a logger named "console".  If so, we
  // will just return it; if not, we'll create our own default one.
  std::shared_ptr<logging::logger> result(spdlog::get("console"));
  if (!result) {
    // We wrap our stderr sink in a dist_sink so that users can atomically swap
    // out the sinks used by all Drake logging, via dist_sink_mt's APIs.
    auto wrapper = std::make_shared<spdlog::sinks::dist_sink_mt>();
    // We use the stderr_sink_mt (instead of stderr_sink_st) so more than one
    // thread can use this logger and have their messages be staggered by line,
    // instead of co-mingling their character bytes.
    wrapper->add_sink(std::make_shared<spdlog::sinks::stderr_sink_mt>());
    result = std::make_shared<logging::logger>("console", std::move(wrapper));
    result->set_level(spdlog::level::info);
  }
  return result;
}
}  // namespace

logging::logger* log() {
  static const never_destroyed<std::shared_ptr<logging::logger>> g_logger(
      onetime_create_log());
  return g_logger.access().get();
}

std::string logging::set_log_level(const std::string& level) {
  spdlog::level::level_enum prev_value = drake::log()->level();
  spdlog::level::level_enum value{};
  if (level == "trace") {
    value = spdlog::level::trace;
  } else if (level == "debug") {
    value = spdlog::level::debug;
  } else if (level == "info") {
    value = spdlog::level::info;
  } else if (level == "warn") {
    value = spdlog::level::warn;
  } else if (level == "err") {
    value = spdlog::level::err;
  } else if (level == "critical") {
    value = spdlog::level::critical;
  } else if (level == "off") {
    value = spdlog::level::off;
  } else if (level == "unchanged") {
    value = prev_value;
  } else {
    throw std::runtime_error(fmt::format("Unknown spdlog level: {}", level));
  }
  drake::log()->set_level(value);
  switch (prev_value) {
    case spdlog::level::trace:
      return "trace";
    case spdlog::level::debug:
      return "debug";
    case spdlog::level::info:
      return "info";
    case spdlog::level::warn:
      return "warn";
    case spdlog::level::err:
      return "err";
    case spdlog::level::critical:
      return "critical";
    case spdlog::level::off:
      return "off";
    default: {
      // N.B. `spdlog::level::level_enum` is not a `enum class`, so the
      // compiler does not know that it has a closed set of values. For
      // simplicity in linking, we do not use `DRAKE_UNREACHABLE`.
      throw std::runtime_error("Should not reach here!");
    }
  }
}

const char* const logging::kSetLogLevelHelpMessage =
    "sets the spdlog output threshold; possible values are "
    "'unchanged', "
    "'trace', "
    "'debug', "
    "'info', "
    "'warn', "
    "'err', "
    "'critical', "
    "'off'";

void logging::set_log_pattern(const std::string& pattern) {
  drake::log()->set_pattern(pattern);
}

const char* const logging::kSetLogPatternHelpMessage =
    "sets the spdlog pattern for formatting; for more information, see "
    "https://github.com/gabime/spdlog/wiki/3.-Custom-formatting";

#else  // HAVE_SPDLOG

logging::logger::logger() {}

logging::logger* log() {
  // A do-nothing logger instance.
  static logging::logger g_logger;
  return &g_logger;
}

std::string logging::set_log_level(const std::string&) {
  return "";
}

const char* const logging::kSetLogLevelHelpMessage =
    "(Text logging is unavailable.)";

void logging::set_log_pattern(const std::string&) {}

const char* const logging::kSetLogPatternHelpMessage =
    "(Text logging is unavailable.)";

#endif  // HAVE_SPDLOG

const char* const logging::kSetLogLevelUnchanged = "unchanged";

}  // namespace drake
