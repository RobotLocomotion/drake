#pragma once

#include <algorithm>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include <Eigen/Core>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/autodiff.h"

// The macro `PYBIND11_NUMPY_OBJECT_DTYPE` place symbols into the namespace
// `pybind11::detail`, so we should not place these in `drake::pydrake`.

PYBIND11_NUMPY_OBJECT_DTYPE(drake::AutoDiffXd);

namespace drake {
namespace pydrake {
namespace internal {

// TODO(eric.cousineau): Deprecate these methods once we support proper NumPy
// UFuncs.

// Adds math function overloads for AutoDiffXd (ADL free functions from
// `autodiff.h`) for both NumPy methods and `pydrake.math`.
// @param obj
//   If this is py::class_, this is intended to register class methods on
//   `AutoDiffXd` to overload NumPy's math methods.
//   If this is py::module, this is intended to register the overloads in
//   `pydrake.math`.
template <typename PyObject>
void BindAutoDiffMathOverloads(PyObject* obj) {
  // TODO(m-chaturvedi) Add Pybind11 documentation.
  (*obj)  // BR
      .def("log", [](const AutoDiffXd& x) { return log(x); })
      .def("abs", [](const AutoDiffXd& x) { return abs(x); })
      .def("exp", [](const AutoDiffXd& x) { return exp(x); })
      .def("sqrt", [](const AutoDiffXd& x) { return sqrt(x); })
      .def("pow", [](const AutoDiffXd& x, double y) { return pow(x, y); })
      .def("sin", [](const AutoDiffXd& x) { return sin(x); })
      .def("cos", [](const AutoDiffXd& x) { return cos(x); })
      .def("tan", [](const AutoDiffXd& x) { return tan(x); })
      .def("asin", [](const AutoDiffXd& x) { return asin(x); })
      .def("acos", [](const AutoDiffXd& x) { return acos(x); })
      .def("atan2",
          [](const AutoDiffXd& y, const AutoDiffXd& x) { return atan2(y, x); })
      .def("sinh", [](const AutoDiffXd& x) { return sinh(x); })
      .def("cosh", [](const AutoDiffXd& x) { return cosh(x); })
      .def("tanh", [](const AutoDiffXd& x) { return tanh(x); })
      .def("min",
          [](const AutoDiffXd& x, const AutoDiffXd& y) { return min(x, y); })
      .def("max",
          [](const AutoDiffXd& x, const AutoDiffXd& y) { return max(x, y); })
      .def("ceil", [](const AutoDiffXd& x) { return ceil(x); })
      .def("__ceil__", [](const AutoDiffXd& x) { return ceil(x); })
      .def("floor", [](const AutoDiffXd& x) { return floor(x); })
      .def("__floor__", [](const AutoDiffXd& x) { return floor(x); })
      // TODO(eric.cousineau): This is not a NumPy-overridable method using
      // dtype=object. Deprecate and move solely into `pydrake.math`.
      .def("inv", [](const MatrixX<AutoDiffXd>& X) -> MatrixX<AutoDiffXd> {
        return X.inverse();
      });
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
