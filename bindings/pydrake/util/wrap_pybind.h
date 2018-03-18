/// @file
/// Defines convenience utilities to wrap pybind11 methods and classes.

#pragma once

#include <string>
#include <utility>

#include "pybind11/pybind11.h"

namespace drake {
namespace pydrake {

// Defines a function in module `m`, and mirrors to module `mirror` for
// backwards compatibility. Throws an error if any mirrored methods already
// exist and do not match the original value.
class MirrorDef {
 public:
  MirrorDef(py::module m, py::module mirror)
      : m_(m), mirror_(mirror) {}

  template <typename... Args>
  MirrorDef& def(const char* name, Args&&... args) {
    m_.def(name, std::forward<Args>(args)...);
    do_mirror(name);
    return *this;
  }

 private:
  void do_mirror(const char* name) {
    py::object value = m_.attr(name);
    // Ensure we won't shadow anything in the mirror; if an attribute of this
    // name exists and is not exactly the same, throw an error.
    py::object mirror_value = py::getattr(mirror_, name, py::none());
    if (!mirror_value.is_none() && !mirror_value.is(value))
      throw py::cast_error(py::str(
          "Mirroring: '{}' from {} already exists in {} and will be shadowed")
              .format(name, m_, mirror_).cast<std::string>());
    mirror_.attr(name) = value;
  }

  py::module m_;
  py::module mirror_;
};

}  // namespace pydrake
}  // namespace drake
