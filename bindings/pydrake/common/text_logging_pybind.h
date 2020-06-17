#pragma once

#ifdef HAVE_SPDLOG
#include <mutex>

#include <spdlog/sinks/base_sink.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#endif

namespace drake {
namespace pydrake {
namespace internal {
#ifdef HAVE_SPDLOG
class pylogging_sink final : public spdlog::sinks::base_sink<std::mutex> {
 public:
  pylogging_sink();

 protected:
  void sink_it_(const spdlog::details::log_msg& msg) override;
  void flush_() override {}

 private:
  py::object py_logger_;
};
#endif

void redirectPythonLogging();

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
