#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

using std::sin;
using std::cos;

namespace drake {
namespace pydrake {

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

PYBIND11_MODULE(_autodiffutils_py, m) {
  m.doc() = "Bindings for Eigen AutoDiff Scalars";

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
    }, py::is_operator())
    .def("__add__", [](const AutoDiffXd& self, double other) {
      return eval(self + other);
    }, py::is_operator())
    .def("__radd__", [](const AutoDiffXd& self, double other) {
      return eval(other + self);
    }, py::is_operator())
    .def("__sub__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self - other);
    }, py::is_operator())
    .def("__sub__", [](const AutoDiffXd& self, double other) {
      return eval(self - other);
    }, py::is_operator())
    .def("__rsub__", [](const AutoDiffXd& self, double other) {
      return eval(other - self);
    }, py::is_operator())
    .def("__mul__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self * other);
    }, py::is_operator())
    .def("__mul__", [](const AutoDiffXd& self, double other) {
      return eval(self * other);
    }, py::is_operator())
    .def("__rmul__", [](const AutoDiffXd& self, double other) {
      return eval(other * self);
    }, py::is_operator())
    .def("__truediv__", [](const AutoDiffXd& self, const AutoDiffXd& other) {
      return eval(self / other);
    }, py::is_operator())
    .def("__truediv__", [](const AutoDiffXd& self, double other) {
      return eval(self / other);
    }, py::is_operator())
    .def("__rtruediv__", [](const AutoDiffXd& self, double other) {
      return eval(other / self);
    }, py::is_operator());
}

}  // namespace pydrake
}  // namespace drake
