#pragma once

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

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

template <typename PyObject>
void BindSymbolicOverloads(PyObject obj) {
  using symbolic::Expression;
  (*obj)  // BR
      .def("log", &symbolic::log)
      .def("abs", &symbolic::abs)
      .def("exp", &symbolic::exp)
      .def("sqrt", &symbolic::sqrt)
      .def("pow", py::overload_cast<const Expression&, const Expression&>(
                      &symbolic::pow))
      .def("sin", &symbolic::sin)
      .def("cos", &symbolic::cos)
      .def("tan", &symbolic::tan)
      .def("asin", &symbolic::asin)
      .def("acos", &symbolic::acos)
      .def("atan", &symbolic::atan)
      .def("atan2", &symbolic::atan2)
      .def("sinh", &symbolic::sinh)
      .def("cosh", &symbolic::cosh)
      .def("tanh", &symbolic::tanh)
      .def("min", &symbolic::min)
      .def("max", &symbolic::max)
      .def("ceil", &symbolic::ceil)
      .def("floor", &symbolic::floor)
      // Matrix overloads.
      .def("inv", [](const MatrixX<Expression>& X) -> MatrixX<Expression> {
        return X.inverse();
      });
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
