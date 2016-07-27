#include "drake/common/text_logging.h"

#include <mutex>

namespace drake {

#ifdef HAVE_SPDLOG

std::shared_ptr<logging::logger> log() {
  static std::mutex log_mutex;
  std::shared_ptr<logging::logger> logger = spdlog::get("console");
  if (!logger) {
    std::lock_guard<std::mutex> guard(log_mutex);
    logger = spdlog::stderr_logger_st("console");
  }
  return logger;
}

#else  // HAVE_SPDLOG

// A do-nothing logger instance.
namespace {
const logger g_logger;
}  // anon namespace

const std::shared_ptr<logging::logger> log() {
  return std::shared_ptr<logging::logger>(&g_logger);
}

#endif  // HAVE_SPDLOG

}  // namespace drake
