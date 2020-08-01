#ifdef HAVE_SPDLOG
#include "drake/bindings/pydrake/common/text_logging_pybind.h"

#include <memory>
#include <mutex>

// clang-format off to disable clang-format-includes
// N.B. text-logging.h must be included before spdlog headers
// to avoid "SPDLOG_ACTIVE_LEVEL" redefined warning (#13771).
#include "drake/common/text_logging.h"
// clang-format on

#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/dist_sink.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_assert.h"
#endif

namespace drake {
namespace pydrake {
namespace internal {

#ifdef HAVE_SPDLOG
namespace {
class pylogging_sink final : public spdlog::sinks::base_sink<std::mutex> {
 public:
  pylogging_sink() {
    py::object py_logging = py::module::import("logging");
    py_logger_ = py_logging.attr("getLogger")("_pydrake_spdlog_sink");
    py::object py_formatter =
        py_logging.attr("Formatter")(py::arg("fmt") = "%(message)s");
    py::object py_handler = py_logging.attr("StreamHandler")();
    py_handler.attr("setFormatter")(py_formatter);
    py_handler.attr("terminator") = "";
    py_logger_.attr("addHandler")(py_handler);
    py_logger_.attr("propagate") = false;
  }

 protected:
  void sink_it_(const spdlog::details::log_msg& msg) override {
    spdlog::memory_buf_t formatted;
    spdlog::sinks::base_sink<std::mutex>::formatter_->format(msg, formatted);
    // Use CRITICAL level to ensure that messages will always be logged.
    // Messages are filtered and formatted by drake::log() and will not
    // contain any extra mark-up from the Python logger.
    py_logger_.attr("critical")(fmt::to_string(formatted));
  }

  void flush_() override {}

 private:
  py::object py_logger_;
};
}  // namespace

void RedirectPythonLogging() {
  // Redirect all logs to Python's `logging` module
  logging::sink* const sink_base = logging::get_dist_sink();
  auto* const dist_sink = dynamic_cast<spdlog::sinks::dist_sink_mt*>(sink_base);
  DRAKE_DEMAND(dist_sink != nullptr);
  auto python_sink = std::make_shared<pylogging_sink>();
  dist_sink->set_sinks({python_sink});
}
#else
void RedirectPythonLogging() {}
#endif

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
