#pragma once

/// @file
/// Helpers for defining scalars and values.

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
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

}  // namespace pydrake
}  // namespace drake
