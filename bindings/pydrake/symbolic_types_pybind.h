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
  using symbolic::Expression;
  constexpr auto& doc = pydrake_doc.drake.symbolic;
  (*obj)  // BR
      .def("log", &symbolic::log, doc.log.doc)
      .def("abs", &symbolic::abs, doc.abs.doc)
      .def("__abs__", &symbolic::abs, doc.abs.doc)
      .def("exp", &symbolic::exp, doc.exp.doc)
      .def("sqrt", &symbolic::sqrt, doc.sqrt.doc)
      .def("pow",
          [](const Class& base, const Expression& exponent) {
            return pow(base, exponent);
          })
      .def("sin", &symbolic::sin, doc.sin.doc)
      .def("cos", &symbolic::cos, doc.cos.doc)
      .def("tan", &symbolic::tan, doc.tan.doc)
      .def("arcsin", &symbolic::asin, doc.asin.doc)
      .def("asin", &symbolic::asin, doc.asin.doc)
      .def("arccos", &symbolic::acos, doc.acos.doc)
      .def("acos", &symbolic::acos, doc.acos.doc)
      .def("arctan", &symbolic::atan, doc.atan.doc)
      .def("atan", &symbolic::atan, doc.atan.doc)
      .def("arctan2", &symbolic::atan2, doc.atan2.doc)
      .def("atan2", &symbolic::atan2, doc.atan2.doc)
      .def("sinh", &symbolic::sinh, doc.sinh.doc)
      .def("cosh", &symbolic::cosh, doc.cosh.doc)
      .def("tanh", &symbolic::tanh, doc.tanh.doc)
      .def("min", &symbolic::min, doc.min.doc)
      .def("max", &symbolic::max, doc.max.doc)
      .def("ceil", &symbolic::ceil, doc.ceil.doc)
      .def("__ceil__", &symbolic::ceil, doc.ceil.doc)
      .def("floor", &symbolic::floor, doc.floor.doc)
      .def("__floor__", &symbolic::floor, doc.floor.doc)
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
