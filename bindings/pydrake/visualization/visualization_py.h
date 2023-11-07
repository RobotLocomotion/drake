#pragma once

/* This file declares the functions that bind the drake::visualization
namespace. These functions form a complete partition of visualization bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per visualization_py_config.cc. */
void DefineVisualizationConfig(py::module m);

/* Defines bindings per visualization_py_image_systems.cc. */
void DefineVisualizationImageSystems(py::module m);

/* Defines bindings per visualization_py_sliders.cc. */
void DefineVisualizationSliders(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
