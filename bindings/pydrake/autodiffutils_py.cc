#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"

using Eigen::AutoDiffScalar;
using std::cos;
using std::sin;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(autodiffutils, m) {
  m.doc() = "Bindings for Eigen AutoDiff Scalars";

  // Install NumPy warning filtres.
  // N.B. This may interfere with other code, but until that is a confirmed
  // issue, we should aggressively try to avoid these warnings.
  py::module::import("pydrake.common.deprecation")
      .attr("install_numpy_warning_filters")();

  // TODO(m-chaturvedi) Add Pybind11 documentation.
  py::class_<AutoDiffXd> autodiff(m, "AutoDiffXd");
  autodiff  // BR
      .def(py::init<double>())
      .def(py::init<const double&, const Eigen::VectorXd&>())
      .def("value", [](const AutoDiffXd& self) { return self.value(); })
      .def("derivatives",
          [](const AutoDiffXd& self) { return self.derivatives(); })
      .def("__str__",
          [](const AutoDiffXd& self) {
            return py::str("AD{{{}, nderiv={}}}")
                .format(self.value(), self.derivatives().size());
          })
      .def("__repr__",
          [](const AutoDiffXd& self) {
            return py::str("<AutoDiffXd {} nderiv={}>")
                .format(self.value(), self.derivatives().size());
          })
      // Arithmetic
      .def(-py::self)
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
          },
          py::is_operator())
      .def("__abs__", [](const AutoDiffXd& x) { return abs(x); });
  DefCopyAndDeepCopy(&autodiff);

  py::implicitly_convertible<double, AutoDiffXd>();
  py::implicitly_convertible<int, AutoDiffXd>();

  // Add overloads for `math` functions.
  auto math = py::module::import("pydrake.math");
  MirrorDef<py::module, decltype(autodiff)>(&math, &autodiff)
      .def("log", [](const AutoDiffXd& x) { return log(x); })
      .def("abs", [](const AutoDiffXd& x) { return abs(x); })
      .def("exp", [](const AutoDiffXd& x) { return exp(x); })
      .def("sqrt", [](const AutoDiffXd& x) { return sqrt(x); })
      .def("pow", [](const AutoDiffXd& x, int y) { return pow(x, y); })
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
      .def("floor", [](const AutoDiffXd& x) { return floor(x); })
      // Matrix
      .def("inv", [](const MatrixX<AutoDiffXd>& X) -> MatrixX<AutoDiffXd> {
        return X.inverse();
      });
  // Mirror for numpy.
  autodiff.attr("arcsin") = autodiff.attr("asin");
  autodiff.attr("arccos") = autodiff.attr("acos");
  autodiff.attr("arctan2") = autodiff.attr("atan2");
}

}  // namespace pydrake
}  // namespace drake
