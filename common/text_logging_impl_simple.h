#include <functional>
#include <string_view>

#include "drake/common/text_logging_level.h"

namespace drake {
namespace logging {
namespace internal {

using SinkFunction = std::function<void(level_enum /* severity */,
                                        std::string_view /* message */)>;

/* Redirects all logging to the given function, or when nullptr restores the
default sink (writing to stderr). This function is not thread-safe; no other
threads should be running when it is called. */
void ReplaceSinkFunction(SinkFunction function);

}  // namespace internal
}  // namespace logging
}  // namespace drake
