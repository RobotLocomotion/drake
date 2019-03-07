#pragma once

#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace test {

// TODO(eric.cousineau): Figure out if there is a better solution than this
// hack.
/// pybind11's Python3 implementation seems to disconnect the `globals()` from
/// an embedded interpreter's `__main__` module. To remedy this, we must
/// manually synchronize these variables.
inline void SynchronizeGlobalsForPython3(py::module m) {
  py::globals().attr("update")(m.attr("__dict__"));
}

}  // namespace test
}  // namespace pydrake
}  // namespace drake
