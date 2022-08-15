#pragma once

/* This file declares the functions that bind the drake::examples namespace.
These functions form a complete partition of the drake::examples bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per examples_py_acrobot.cc. */
void DefineExamplesAcrobot(py::module m);

/* Defines bindings per examples_py_compass_gait.cc. */
void DefineExamplesCompassGait(py::module m);

/* Defines bindings per examples_py_manipulation_station.cc. */
void DefineExamplesManipulationStation(py::module m);

/* Defines bindings per examples_py_pendulum.cc. */
void DefineExamplesPendulum(py::module m);

/* Defines bindings per examples_py_quadrotor.cc. */
void DefineExamplesQuadrotor(py::module m);

/* Defines bindings per examples_py_rimless_wheel.cc. */
void DefineExamplesRimlessWheel(py::module m);

/* Defines bindings per examples_py_van_der_pol.cc. */
void DefineExamplesVanDerPol(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
