#pragma once

#include <variant>

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

// TODO(SeanCurtis-TRI) When pybind issue 3019 gets resolved, we won't need to
//  define this locally anymore. In fact, it will probably cause link errors.
namespace pybind11 {
namespace detail {

template <>
struct type_caster<std::monostate> {
 public:
  PYBIND11_TYPE_CASTER(std::monostate, _("None"));

  bool load(handle src, bool) { return src.ptr() == Py_None; }

  static handle cast(
      std::monostate, return_value_policy /* policy */, handle /* parent */) {
    Py_RETURN_NONE;
  }
};

}  // namespace detail
}  // namespace pybind11
