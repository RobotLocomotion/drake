#pragma once

#include <typeinfo>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"

// Any binding file that binds a function whose signature involves a
// `DrakeLcmInterface*` (as a parameter or a return value) must include this
// header, so that every translation unit sees the same specialization.
// Strictly, only converting such a pointer from C++ to Python needs it, but we
// do not try to track which bindings do that.

#ifdef PYDRAKE_USE_NANOBIND
// Some C++ subclasses of DrakeLcmInterface use multiple inheritance, e.g.,
// LcmInterfaceSystem also inherits LeafSystem, and are bound in Python without
// a DrakeLcmInterface base class. By default, nanobind uses RTTI to label a
// DrakeLcmInterface* as the most-derived registered type (e.g.,
// LcmInterfaceSystem), which would wrap the (non-zero offset)
// DrakeLcmInterface subobject pointer as that type and crash on first use.
//
// Instead, we label the pointer as DrakeLcm when it is one, and otherwise as
// plain DrakeLcmInterface, which is always a correct view of the pointer. Note
// that this means other registered subclasses lose their specific Python type
// when returned from C++ as a DrakeLcmInterface*. That the label differs from
// the most-derived type, and that DrakeLcm is promoted, are both intentional
// (and tested; see LcmBuses.Find in lcm_test.py).
namespace nanobind::detail {
template <>
struct type_hook<drake::lcm::DrakeLcmInterface> {
  static const std::type_info* get(drake::lcm::DrakeLcmInterface* p) {
    if (dynamic_cast<drake::lcm::DrakeLcm*>(p) != nullptr) {
      return &typeid(drake::lcm::DrakeLcm);
    }
    return &typeid(drake::lcm::DrakeLcmInterface);
  }
};
}  // namespace nanobind::detail
#endif  // PYDRAKE_USE_NANOBIND
