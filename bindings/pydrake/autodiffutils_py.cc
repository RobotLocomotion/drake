#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_throw.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using std::cos;
using std::sin;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(autodiffutils, m) {
  m.doc() = "Bindings for Eigen AutoDiff Scalars";

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  // Install NumPy warning filtres.
  // N.B. This may interfere with other code, but until that is a confirmed
  // issue, we should aggressively try to avoid these warnings.
  py::module::import("pydrake.common.deprecation")
      .attr("install_numpy_warning_filters")();

  // TODO(m-chaturvedi) Add Pybind11 documentation.
  py::class_<AutoDiffXd> autodiff(m, "AutoDiffXd");
  autodiff  // BR
      .def(py::init<double>())
      .def(py::init<const double&, const VectorXd&>())
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
  DefPickle(&autodiff,
      [](const AutoDiffXd& self) {
        return py::make_tuple(self.value(), self.derivatives());
      },
      [](py::tuple t) {
        DRAKE_THROW_UNLESS(t.size() == 2);
        return AutoDiffXd(t[0].cast<double>(), t[1].cast<VectorXd>());
      });
  DefCopyAndDeepCopy(&autodiff);

  py::implicitly_convertible<double, AutoDiffXd>();
  py::implicitly_convertible<int, AutoDiffXd>();

  pydrake::internal::BindAutoDiffMathOverloads(&autodiff);

  // Mirror for numpy.
  autodiff.attr("arcsin") = autodiff.attr("asin");
  autodiff.attr("arccos") = autodiff.attr("acos");
  autodiff.attr("arctan2") = autodiff.attr("atan2");

  m.def("initializeAutoDiff",
      [](const Eigen::MatrixXd& mat, Eigen::DenseIndex num_derivatives,
          Eigen::DenseIndex deriv_num_start) {
        return initializeAutoDiff(mat, num_derivatives, deriv_num_start);
      },
      py::arg("mat"), py::arg("num_derivatives") = -1,
      py::arg("deriv_num_start") = 0, doc.initializeAutoDiff.doc_3args);

  m.def("autoDiffToValueMatrix",
      [](const MatrixX<AutoDiffXd>& autodiff_matrix) {
        return autoDiffToValueMatrix(autodiff_matrix);
      },
      py::arg("autodiff_matrix"), doc.autoDiffToValueMatrix.doc);

  m.def("autoDiffToGradientMatrix",
      [](const MatrixX<AutoDiffXd>& autodiff_matrix) {
        return autoDiffToGradientMatrix(autodiff_matrix);
      },
      py::arg("autodiff_matrix"), doc.autoDiffToGradientMatrix.doc);

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
