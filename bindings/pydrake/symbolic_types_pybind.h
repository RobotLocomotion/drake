#pragma once

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/symbolic.h"

// The macro `PYBIND11_NUMPY_OBJECT_DTYPE` place symbols into the namespace
// `pybind11::detail`, so we should not place these in `drake::pydrake`.

// Whenever we want to cast any array / matrix type of `T` in C++
// (e.g. `Eigen::MatrixX<T>`) to a NumPy array, we should have it in the
// following list.
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Expression);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Formula);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Monomial);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Polynomial);
PYBIND11_NUMPY_OBJECT_DTYPE(drake::symbolic::Variable);

namespace drake {
namespace pydrake {
namespace internal {

// TODO(eric.cousineau): Deprecate these methods once we support proper NumPy
// UFuncs.

// Adds math function overloads for Expression (ADL free functions from
// `symbolic_expression.h`) for both NumPy methods and `pydrake.math`.
// @param obj
//   This is used to register functions or overloads in either
//   `pydrake.symbolic` or `pydrake.math`.
template <typename Class, typename PyObject>
void BindSymbolicMathOverloads(PyObject* obj) {
  // We denote each method as satisfying the following:
  // * UFunc - for use as a NumPy UFunc
  // * Object model - for use as Python's object model
  // * C++ - just re-spelling C++ API
  using symbolic::Expression;
  constexpr auto& doc = pydrake_doc.drake.symbolic;
  (*obj)  // BR
      // UFunc, C++
      .def("log", &symbolic::log, doc.log.doc)
      // UFunc, C++
      .def("abs", &symbolic::abs, doc.abs.doc)
      // Object model
      .def("__abs__", &symbolic::abs, doc.abs.doc)
      // UFunc, C++
      .def("exp", &symbolic::exp, doc.exp.doc)
      // UFunc, C++
      .def("sqrt", &symbolic::sqrt, doc.sqrt.doc)
      // UFunc, C++
      .def("pow",
          [](const Class& base, const Expression& exponent) {
            return pow(base, exponent);
          })
      // Object model
      .def("__pow__",
          [](const Class& base, const Expression& exponent) {
            return pow(base, exponent);
          })
      // UFunc, C++
      .def("sin", &symbolic::sin, doc.sin.doc)
      // UFunc, C++
      .def("cos", &symbolic::cos, doc.cos.doc)
      // UFunc, C++
      .def("tan", &symbolic::tan, doc.tan.doc)
      // UFunc
      .def("arcsin", &symbolic::asin, doc.asin.doc)
      // C++
      .def("asin", &symbolic::asin, doc.asin.doc)
      // UFunc
      .def("arccos", &symbolic::acos, doc.acos.doc)
      // C++
      .def("acos", &symbolic::acos, doc.acos.doc)
      // UFunc
      .def("arctan", &symbolic::atan, doc.atan.doc)
      // C++
      .def("atan", &symbolic::atan, doc.atan.doc)
      // UFunc
      .def("arctan2", &symbolic::atan2, doc.atan2.doc)
      // C++
      .def("atan2", &symbolic::atan2, doc.atan2.doc)
      // UFunc, C++
      .def("sinh", &symbolic::sinh, doc.sinh.doc)
      // UFunc, C++
      .def("cosh", &symbolic::cosh, doc.cosh.doc)
      // UFunc, C++
      .def("tanh", &symbolic::tanh, doc.tanh.doc)
      // C++
      .def("min", &symbolic::min, doc.min.doc)
      // C++
      .def("max", &symbolic::max, doc.max.doc)
      // UFunc, C++
      .def("ceil", &symbolic::ceil, doc.ceil.doc)
      // Object Model
      .def("__ceil__", &symbolic::ceil, doc.ceil.doc)
      // UFunc, C++
      .def("floor", &symbolic::floor, doc.floor.doc)
      // Object Model
      .def("__floor__", &symbolic::floor, doc.floor.doc);
}

inline void BindSymbolicMathModuleOverloads(py::module m) {
  using symbolic::Expression;
  BindSymbolicMathOverloads<Expression>(&m);
  m  // BR
     // TODO(eric.cousineau): This is not a NumPy-overridable method using
     // dtype=object. Deprecate and move solely into `pydrake.math`.
      .def(
          "inv",
          [](const MatrixX<Expression>& X) -> MatrixX<Expression> {
            return X.inverse();
          },
          "Symbolic matrix inverse");
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
