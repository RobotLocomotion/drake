#include "drake/common/text_logging.h"

namespace drake {
namespace logging {

logger::logger() : impl_{nullptr} {}

logger::~logger() = default;

bool logger::should_log(level_enum /* severity */) const {
  return false;
}

level_enum logger::level() const {
  return level::off;
}

void logger::set_level(level_enum /* severity */) const {}

void logger::set_pattern(std::string_view /* pattern */) {}

void logger::LogMessage(level_enum /* severity */,
                        std::string_view /* message */) {}

}  // namespace logging
}  // namespace drake
