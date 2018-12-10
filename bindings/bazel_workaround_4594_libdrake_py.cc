#include "pybind11/pybind11.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(bazel_workaround_4594_libdrake, m) {
  m.doc() =
      "Consolidated workaround for bazelbuild/bazel#4594 to load "
      "libdrake.so, to be used by `pydrake/__init__.py`.";
}

}  // namespace pydrake
}  // namespace drake
