#pragma once

/// @file
/// Helpers for defining Python types within the Systems framework.

#include <string>

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace pysystems {

// TODO(eric.cousineau): Deprecate this header and remove.

using pydrake::AddValueInstantiation;
using pydrake::CommonScalarPack;
using pydrake::DefClone;
using pydrake::NonSymbolicScalarPack;

}  // namespace pysystems
}  // namespace pydrake
}  // namespace drake
