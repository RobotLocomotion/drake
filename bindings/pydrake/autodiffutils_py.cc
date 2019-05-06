#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
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
          [](const AutoDiffXd& base, double exponent) {
            return pow(base, exponent);
          },
          py::is_operator())
      .def("__abs__", [](const AutoDiffXd& x) { return abs(x); });
  DefCopyAndDeepCopy(&autodiff);

  py::implicitly_convertible<double, AutoDiffXd>();
  py::implicitly_convertible<int, AutoDiffXd>();

  pydrake::internal::BindAutoDiffMathOverloads(&autodiff);

  // Mirror for numpy.
  autodiff.attr("arcsin") = autodiff.attr("asin");
  autodiff.attr("arccos") = autodiff.attr("acos");
  autodiff.attr("arctan2") = autodiff.attr("atan2");
}

}  // namespace pydrake
}  // namespace drake
