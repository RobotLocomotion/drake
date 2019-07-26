#pragma once

/// @file
/// Helpers for defining scalars and values.

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/common/default_scalars.h"

namespace drake {
namespace pydrake {

// N.B. This should be kept in sync with the `*_DEFAULT_SCALARS` macro in
// `default_scalars.h`.
/// Type pack defining common scalar types.
using CommonScalarPack = type_pack<  // BR
    double,                          //
    AutoDiffXd,                      //
    symbolic::Expression>;

// N.B. This should be kept in sync with the `*_DEFAULT_NONSYMBOLIC_SCALARS`
// macro in `default_scalars.h`.
/// Type pack for non-symbolic common scalar types.
using NonSymbolicScalarPack = type_pack<  // BR
    double,                               //
    AutoDiffXd>;

// TODO(eric.cousineau): Simplify this (#8116).
/// Permits referencing for builtin dtypes (e.g. T = double), but then switches
/// to copying for custom dtypes (T = {AutoDiffXd, Expression}).
template <typename T>
py::return_value_policy return_value_policy_for_scalar_type() {
  if (std::is_same<T, double>::value) {
    return py::return_value_policy::reference_internal;
  } else {
    return py::return_value_policy::copy;
  }
}

namespace internal {
// For generic types, only permit conversion to the type itself.
// `AutoDiffXd` and `Expression` cannot `static_cast<>` to each other, and
// cannot `static_cast<>` to `double`.
template <typename T>
struct CastUPack {
  using Pack = type_pack<T>;
};
// For `double`, permit conversion to any common type.
template <>
struct CastUPack<double> {
  using Pack = CommonScalarPack;
};
}  // namespace internal

/// Binds `cast<T>()` explicitly.
template <typename T, typename PyClass,
    typename UPack = typename internal::CastUPack<T>::Pack>
void DefCast(PyClass* cls, const char* doc, UPack U_pack = {}) {
  using Class = typename PyClass::type;
  auto bind_scalar = [cls, doc](auto U_dummy) {
    using U = decltype(U_dummy);
    AddTemplateMethod(*cls, "cast",
        [](const Class& self) { return self.template cast<U>(); },
        GetPyParam<U>(), doc);
  };
  type_visit(bind_scalar, U_pack);
}

}  // namespace pydrake
}  // namespace drake
