#pragma once

#pragma once

/** @file This files declares the functions which bind various portions of the
 common::parsing namespace. These functions form a complete partition of all
 parsing bindings. The implementation of these methods are parceled out into
 various .cc files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

/** Defines PackageMap in the drake namespace. See parsing_py_package_map.cc. */
void DefineParsingPackageMap(py::module m);

}  // namespace pydrake
}  // namespace drake
