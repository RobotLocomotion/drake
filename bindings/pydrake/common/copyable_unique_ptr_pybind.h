#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/unused.h"

namespace pybind11 {
namespace detail {

// Casts `copyable_unique_ptr<T>` as `T` for Python.
// N.B. At present, this only supports C++ to Python conversion.
template <typename T>
struct type_caster<drake::copyable_unique_ptr<T>> {
  using Type = drake::copyable_unique_ptr<T>;
  using value_conv = make_caster<T>;

  // N.B. This macro assumes placement in `pybind11::detail`.
  PYBIND11_TYPE_CASTER(Type, type_caster<T>::name);

  bool load(handle src, bool convert) {
    drake::unused(src, convert);
    throw cast_error(
        "Converting Python object to C++ drake::copyable_unique_ptr<> is not "
        "yet implemented.");
  }

  static handle cast(Type src, return_value_policy policy, handle parent) {
    switch (policy) {
      case return_value_policy::reference:
      case return_value_policy::reference_internal: {
        return value_conv::cast(src.get(), policy, parent);
      }
      default:
        throw cast_error(
            "Only reference or reference_internal are permitted "
            "return_value_policy values for drake::copyable_unique_ptr<>");
    }
  }
};

}  // namespace detail
}  // namespace pybind11
