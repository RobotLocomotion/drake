#include "drake/common/text_logging_spdlog.h"

#include "drake/common/text_logging_impl_spdlog.h"

namespace drake {
namespace logging {

spdlog::sinks::dist_sink_mt* get_dist_sink() {
  // Extract the dist_sink_mt from Drake's logger instance.
  spdlog::logger* spdlog_logger = internal::get_spdlog_logger_singleton();
  spdlog::sinks::sink* sink_base = spdlog_logger->sinks().empty()
                                       ? nullptr
                                       : spdlog_logger->sinks().front().get();
  auto* result = dynamic_cast<spdlog::sinks::dist_sink_mt*>(sink_base);
  if (result == nullptr) {
    throw std::logic_error(
        "drake::logging::get_sink(): error: the spdlog sink configuration has"
        "unexpectedly changed.");
  }
  return result;
}

}  // namespace logging
}  // namespace drake
