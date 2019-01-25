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
  }
  return result;
}
}  // namespace

logging::logger* log() {
  static const never_destroyed<std::shared_ptr<logging::logger>> g_logger(
      onetime_create_log());
  return g_logger.access().get();
}

logging::sink* logging::get_dist_sink() {
  // Extract the dist_sink_mt from Drake's logger instance.
  auto* sink = log()->sinks().empty() ? nullptr : log()->sinks().front().get();
  auto* result = dynamic_cast<spdlog::sinks::dist_sink_mt*>(sink);
  if (result == nullptr) {
    throw std::logic_error(
        "drake::logging::get_sink(): error: the spdlog sink configuration has"
        "unexpectedly changed.");
  }
  return result;
}

#else  // HAVE_SPDLOG

logging::logger::logger() {}

logging::sink::sink() {}

logging::logger* log() {
  // A do-nothing logger instance.
  static logging::logger g_logger;
  return &g_logger;
}

logging::sink* logging::get_dist_sink() {
  // An empty sink instance.
  static logging::sink g_sink;
  return &g_sink;
}

#endif  // HAVE_SPDLOG

}  // namespace drake
