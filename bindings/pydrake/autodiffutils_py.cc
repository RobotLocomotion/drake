#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

using Eigen::AutoDiffScalar;
using std::sin;
using std::cos;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(_autodiffutils_py, m) {
  m.doc() = "Bindings for Eigen AutoDiff Scalars";

  py::class_<AutoDiffXd> autodiff(m, "AutoDiffXd");
  autodiff
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
    .def(py::self + py::self)
    .def(py::self + double())
    .def(double() + py::self)
    .def(py::self - py::self)
    .def(py::self - double())
    .def(double() - py::self)
    .def(py::self * py::self)
    .def(py::self * double())
    .def(double() * py::self)
    .def(py::self / py::self)
    .def(py::self / double())
    .def(double() / py::self)
    .def("__pow__",
         [](const AutoDiffXd& base, int exponent) {
           return pow(base, exponent);
         }, py::is_operator());

    // Add overloads for `sin` and `cos`.
    auto math = py::module::import("pydrake.math");
    math
      .def("sin", [](const AutoDiffXd& x) { return sin(x); })
      .def("cos", [](const AutoDiffXd& x) { return cos(x); });
    // Re-define for backwards compatibility.
    autodiff
      .def("sin", [](const AutoDiffXd& x) { return sin(x); })
      .def("cos", [](const AutoDiffXd& x) { return cos(x); });
}

}  // namespace pydrake
}  // namespace drake
