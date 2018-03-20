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
    .def(py::init<const double&, const Eigen::VectorXd&>())
    .def("value", [](const AutoDiffXd& self) {
      return self.value();
    })
    .def("derivatives", [](const AutoDiffXd& self) {
      return self.derivatives();
    })
    .def("__str__", [](const AutoDiffXd& self) {
      return py::str("AD{{{}, nderiv={}}}").format(
          self.value(), self.derivatives().size());
    })
    .def("__repr__", [](const AutoDiffXd& self) {
      return py::str("<AutoDiffXd {} nderiv={}>").format(
          self.value(), self.derivatives().size());
    })
    // Arithmetic
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
    // Logical comparison
    .def(py::self == py::self)
    .def(py::self == double())
    .def(py::self != py::self)
    .def(py::self != double())
    .def(py::self < py::self)
    .def(py::self < double())
    .def(py::self <= py::self)
    .def(py::self <= double())
    .def(py::self > py::self)
    .def(py::self > double())
    .def(py::self >= py::self)
    .def(py::self >= double())
    // Additional math
    .def("__pow__",
         [](const AutoDiffXd& base, int exponent) {
           return pow(base, exponent);
         }, py::is_operator());

    // Add overloads for `sin` and `cos`.
    auto math = py::module::import("pydrake.math");
    math
      .def("log", [](const AutoDiffXd& x) { return log(x); })
      .def("sin", [](const AutoDiffXd& x) { return sin(x); })
      .def("cos", [](const AutoDiffXd& x) { return cos(x); })
      .def("tan", [](const AutoDiffXd& x) { return tan(x); })
      .def("asin", [](const AutoDiffXd& x) { return asin(x); })
      .def("acos", [](const AutoDiffXd& x) { return acos(x); })
      .def("atan2",
           [](const AutoDiffXd& y, const AutoDiffXd& x) {
             return atan2(y, x);
           }, py::arg("y"), py::arg("x"))
      .def("sinh", [](const AutoDiffXd& x) { return sinh(x); })
      .def("cosh", [](const AutoDiffXd& x) { return cosh(x); })
      .def("tanh", [](const AutoDiffXd& x) { return tanh(x); });
    // Re-define a subset for backwards compatibility.
    autodiff
      .def("sin", [](const AutoDiffXd& x) { return sin(x); })
      .def("cos", [](const AutoDiffXd& x) { return cos(x); });
}

}  // namespace pydrake
}  // namespace drake
