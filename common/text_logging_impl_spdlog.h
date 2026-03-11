#include <spdlog/spdlog.h>

namespace drake {
namespace internal {

/* Returns the singleton console logger. */
spdlog::logger* get_spdlog_logger_singleton();

}  // namespace internal
}  // namespace drake
