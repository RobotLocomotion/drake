#pragma once

#ifdef HAVE_SPDLOG
#include <mutex>

#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

using namespace pybind11::literals;  // NOLINT(build/namespaces)

template <typename Mutex>
class pylogging_sink : public spdlog::sinks::base_sink<Mutex> {
 public:
  pylogging_sink() {
    py::object py_logging = py::module::import("logging");
    py_logging.attr("basicConfig")(
        "format"_a = "%(message)s", "level"_a = "WARNING");
  }

 protected:
  void sink_it_(const spdlog::details::log_msg& msg) override {
    spdlog::memory_buf_t formatted;
    spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
    py::object py_logging = py::module::import("logging");
    py::object py_logger = py_logging.attr("getLogger")("drakelog");
    py_logger.attr("warning")(fmt::to_string(formatted));
  }

  void flush_() override {}
};

using pylogging_sink_mt = pylogging_sink<std::mutex>;
using pylogging_sink_st = pylogging_sink<spdlog::details::null_mutex>;

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
#endif
