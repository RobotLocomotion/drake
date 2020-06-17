#ifdef HAVE_SPDLOG
#include "drake/bindings/pydrake/common/text_logging_pybind.h"

#include <memory>

#include <spdlog/sinks/dist_sink.h>

#include "drake/common/text_logging.h"
#endif

namespace drake {
namespace pydrake {
namespace internal {

#ifdef HAVE_SPDLOG
pylogging_sink::pylogging_sink() {
  py::object py_logging = py::module::import("logging");
  py_logger_ = py_logging.attr("getLogger")("pydrake_spdlog_sink_");
  py::object py_formatter =
      py_logging.attr("Formatter")(py::arg("fmt") = "%(message)s");
  py::object py_handler = py_logging.attr("StreamHandler")();
  py_handler.attr("setFormatter")(py_formatter);
  py_logger_.attr("addHandler")(py_handler);
  py_logger_.attr("propagate") = false;
}

void pylogging_sink::sink_it_(const spdlog::details::log_msg& msg) {
  spdlog::memory_buf_t formatted;
  spdlog::sinks::base_sink<std::mutex>::formatter_->format(msg, formatted);
  // Use CRITICAL level to ensure that messages will always be logged.
  // Messages are filtered and formatted by drake::log() and will not
  // contain any extra mark-up from the Python logger.
  py_logger_.attr("critical")(fmt::to_string(formatted));
}

void redirectPythonLogging() {
  // Redirect all logs to Python's `logging` module
  logging::sink* const sink_base = logging::get_dist_sink();
  auto* const dist_sink = dynamic_cast<spdlog::sinks::dist_sink_mt*>(sink_base);
  auto python_sink = std::make_shared<pylogging_sink>();
  dist_sink->set_sinks({python_sink});
}
#else
void redirectPythonLogging() {}
#endif

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
