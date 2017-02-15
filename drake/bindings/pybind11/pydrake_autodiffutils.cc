#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "drake/bindings/pybind11/pydrake_autodiff_types.h"


namespace py = pybind11;

using std::sin;
using std::cos;

/**
 * Force Eigen to evaluate an autodiff expression. We need this function
 * because, for example, adding two Eigen::AutoDiffXd values produces an
 * Eigen::AutoDiffScalar<Eigen::CWiseBinaryOp> which cannot be returned to
 * python. This just forces an evaluation and conversion to AutoDiffXd which
 * would normally happen automatically in C++.
 */
template <typename Derived>
AutoDiffXd eval(const Eigen::AutoDiffScalar<Derived>& x) {
  return AutoDiffXd(x.value(), x.derivatives());
}

PYBIND11_PLUGIN(_pydrake_autodiffutils) {
  py::module m("_pydrake_autodiffutils", "Bindings for Eigen AutoDiff Scalars");

  py::class_<AutoDiffXd>(m, "AutoDiffXd")
    .def("__init__",
         [](AutoDiffXd& self,
            double value,
            const Eigen::VectorXd& derivatives) {
           new (&self) AutoDiffXd(value, derivatives);
         })
    .def("value", [](const AutoDiffXd& self) {
      return self.value();
    })
    .def("derivatives", [](const AutoDiffXd& self) {
      return self.derivatives();
    })
    .def("sin", [](const AutoDiffXd& self) { return eval(sin(self)); })
    .def("cos", [](const AutoDiffXd& self) { return eval(cos(self)); })
    .def("__add__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self + other);
    })
    .def("__add__", [](const AutoDiffXd& self, double other) {
      return eval(self + other);
    })
    .def("__radd__", [](const AutoDiffXd& self, double other) {
      return eval(other + self);
    })
    .def("__sub__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self - other);
    })
    .def("__sub__", [](const AutoDiffXd& self, double other) {
      return eval(self - other);
    })
    .def("__rsub__", [](const AutoDiffXd& self, double other) {
      return eval(other - self);
    })
    .def("__mul__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self * other);
    })
    .def("__mul__", [](const AutoDiffXd& self, double other) {
      return eval(self * other);
    })
    .def("__rmul__", [](const AutoDiffXd& self, double other) {
      return eval(other * self);
    })
    .def("__truediv__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self / other);
    })
    .def("__truediv__", [](const AutoDiffXd& self, double other) {
      return eval(self / other);
    })
    .def("__rtruediv__", [](const AutoDiffXd& self, double other) {
      return eval(other / self);
    });

  return m.ptr();
}
