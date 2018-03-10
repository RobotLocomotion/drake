#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(trajectories, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::trajectories;

  using T = double;

  py::class_<PiecewisePolynomial<T>>(m, "PiecewisePolynomial")
      .def(py::init<>())
      .def(py::init<const Eigen::Ref<const MatrixX<T>>&>())
      .def_static("ZeroOrderHold", &PiecewisePolynomial<T>::ZeroOrderHold)
      .def_static("FirstOrderHold", &PiecewisePolynomial<T>::FirstOrderHold)
      .def_static("Pchip", &PiecewisePolynomial<T>::Pchip)
      .def_static(
          "Cubic",
          py::overload_cast<
              const std::vector<double>&,
              const std::vector<PiecewisePolynomial<T>::CoefficientMatrix>&,
              const PiecewisePolynomial<T>::CoefficientMatrix&,
              const PiecewisePolynomial<T>::CoefficientMatrix&>(
              &PiecewisePolynomial<T>::Cubic),
          py::arg("breaks"), py::arg("knots"), py::arg("knot_dot_start"),
          py::arg("knot_dot_end"))
      .def_static(
          "Cubic",
          py::overload_cast<
              const std::vector<double>&,
              const std::vector<PiecewisePolynomial<T>::CoefficientMatrix>&,
              const std::vector<PiecewisePolynomial<T>::CoefficientMatrix>&>(
              &PiecewisePolynomial<T>::Cubic),
          py::arg("breaks"), py::arg("knots"), py::arg("knots_dot"))
      .def_static(
          "Cubic",
          py::overload_cast<
              const std::vector<double>&,
              const std::vector<PiecewisePolynomial<T>::CoefficientMatrix>&>(
              &PiecewisePolynomial<T>::Cubic),
          py::arg("breaks"), py::arg("knots"))
      .def("value", &PiecewisePolynomial<T>::value)
      .def("rows", &PiecewisePolynomial<T>::rows)
      .def("cols", &PiecewisePolynomial<T>::cols);
}

}  // namespace pydrake
}  // namespace drake
