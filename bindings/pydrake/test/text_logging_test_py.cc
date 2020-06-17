#include "pybind11/pybind11.h"

#include "drake/common/text_logging.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(text_logging_test, m) {
  m.doc() = "Test text logging";

  m.def("do_log_test", []() {
    drake::log()->debug("Test Debug message");
    drake::log()->info("Test Info message");
    drake::log()->warn("Test Warn message");
    drake::log()->error("Test Error message");
    drake::log()->critical("Test Critical message");
  });
}

}  // namespace pydrake
}  // namespace drake
