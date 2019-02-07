#include "pybind11/functional.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(compatibility_test_util, m) {
  m.def("invoke_callback", [](std::function<void()> callback) {
    // Trivial callback test.
    callback();
  });
}

}  // namespace pydrake
}  // namespace drake
