#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/math_operators_pybind.h"
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
namespace internal {

void DefineAutodiffutils(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  // TODO(m-chaturvedi) Add Pybind11 documentation.
  py::class_<AutoDiffXd> autodiff(m, "AutoDiffXd");
  autodiff  // BR
      .def(py::init<double>(), py::arg("value"),
          "Constructs a value with empty derivatives.")
      .def(py::init<const double&, const VectorXd&>(), py::arg("value"),
          py::arg("derivatives"),
          "Constructs a value with the given derivatives.")
      .def(py::init<double, Eigen::Index, Eigen::Index>(), py::arg("value"),
          py::arg("size"), py::arg("offset"),
          "Constructs a value with a single partial derivative of 1.0 at the "
          "given `offset` in a vector of `size` otherwise-zero derivatives.")
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
      .def(
          "__pow__",
          [](const AutoDiffXd& base, double exponent) {
            return pow(base, exponent);
          },
          py::is_operator())
      .def("__abs__", [](const AutoDiffXd& x) { return abs(x); })
      .def(py::pickle(
          [](const AutoDiffXd& self) {
            return py::make_tuple(self.value(), self.derivatives());
          },
          [](py::tuple t) {
            DRAKE_THROW_UNLESS(t.size() == 2);
            return AutoDiffXd(t[0].cast<double>(), t[1].cast<VectorXd>());
          }));
  DefCopyAndDeepCopy(&autodiff);

  py::implicitly_convertible<double, AutoDiffXd>();
  py::implicitly_convertible<int, AutoDiffXd>();

  pydrake::internal::BindMathOperators<AutoDiffXd>(&autodiff);

  // Mirror for numpy.
  autodiff.attr("arcsin") = autodiff.attr("asin");
  autodiff.attr("arccos") = autodiff.attr("acos");
  autodiff.attr("arctan2") = autodiff.attr("atan2");

  m.def(
      "InitializeAutoDiff",
      [](const Eigen::MatrixXd& value, std::optional<int> num_derivatives,
          std::optional<int> deriv_num_start) {
        return InitializeAutoDiff(value, num_derivatives, deriv_num_start);
      },
      py::arg("value"), py::arg("num_derivatives") = std::nullopt,
      py::arg("deriv_num_start") = std::nullopt,
      doc.InitializeAutoDiff.doc_just_value);

  m.def(
      "InitializeAutoDiff",
      [](const Eigen::MatrixXd& value, const Eigen::MatrixXd& gradient) {
        return InitializeAutoDiff(value, gradient);
      },
      py::arg("value"), py::arg("gradient"),
      doc.InitializeAutoDiff.doc_value_and_gradient);

  m.def(
      "ExtractValue",
      [](const MatrixX<AutoDiffXd>& auto_diff_matrix) {
        return ExtractValue(auto_diff_matrix);
      },
      py::arg("auto_diff_matrix"), doc.ExtractValue.doc);

  m.def(
      "ExtractGradient",
      [](const MatrixX<AutoDiffXd>& auto_diff_matrix) {
        return ExtractGradient(auto_diff_matrix);
      },
      py::arg("auto_diff_matrix"), doc.ExtractGradient.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
