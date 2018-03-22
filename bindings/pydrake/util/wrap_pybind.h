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
template <typename From, typename To>
class MirrorDef {
 public:
  MirrorDef(From* m, To* mirror)
      : m_(m), mirror_(mirror) {}

  template <typename... Args>
  MirrorDef& def(const char* name, Args&&... args) {
    m_->def(name, std::forward<Args>(args)...);
    mirror_->def(name, std::forward<Args>(args)...);
    return *this;
  }

 private:
  From* m_{};
  To* mirror_{};
};

}  // namespace pydrake
}  // namespace drake
