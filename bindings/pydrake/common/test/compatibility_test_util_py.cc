#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

PYDRAKE_MODULE(compatibility_test_util, m) {
  m.def("invoke_callback", [](std::function<void()> callback) {
    // Trivial callback test.
    callback();
  });
}

}  // namespace pydrake
}  // namespace drake
