#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"
#include "drake/systems/controllers/dynamic_programming.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(controllers, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::controllers;

  py::module::import("pydrake.math");
  py::module::import("pydrake.symbolic");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  py::class_<DynamicProgrammingOptions::PeriodicBoundaryCondition>(
      m, "PeriodicBoundaryCondition")
      .def(py::init<int, double, double>());

  py::class_<DynamicProgrammingOptions>(m, "DynamicProgrammingOptions")
      .def(py::init<>())
      .def_readwrite("discount_factor",
                     &DynamicProgrammingOptions::discount_factor)
      .def_readwrite("periodic_boundary_conditions",
                     &DynamicProgrammingOptions::periodic_boundary_conditions)
      .def_readwrite("convergence_tol",
                     &DynamicProgrammingOptions::convergence_tol)
      .def_readwrite("visualization_callback",
                     &DynamicProgrammingOptions::visualization_callback);

  m.def("FittedValueIteration", WrapCallbacks(&FittedValueIteration));

  m.def("LinearProgrammingApproximateDynamicProgramming",
        WrapCallbacks(&LinearProgrammingApproximateDynamicProgramming));

  m.def("LinearQuadraticRegulator",
        [](const Eigen::Ref<const Eigen::MatrixXd>& A,
           const Eigen::Ref<const Eigen::MatrixXd>& B,
           const Eigen::Ref<const Eigen::MatrixXd>& Q,
           const Eigen::Ref<const Eigen::MatrixXd>& R,
           const Eigen::Ref<const Eigen::MatrixXd>& N) {
          auto result = LinearQuadraticRegulator(A, B, Q, R, N);
          return std::make_pair(result.K, result.S);
        },
        py::arg("A"), py::arg("B"), py::arg("Q"), py::arg("R"),
        py::arg("N") = Eigen::Matrix<double, 0, 0>::Zero());

  m.def("DiscreteTimeLinearQuadraticRegulator",
        [](const Eigen::Ref<const Eigen::MatrixXd>& A,
           const Eigen::Ref<const Eigen::MatrixXd>& B,
           const Eigen::Ref<const Eigen::MatrixXd>& Q,
           const Eigen::Ref<const Eigen::MatrixXd>& R) {
          auto result = DiscreteTimeLinearQuadraticRegulator(A, B, Q, R);
          return std::make_pair(result.K, result.S);
        },
        py::arg("A"), py::arg("B"), py::arg("Q"), py::arg("R"));

  m.def("LinearQuadraticRegulator",
        py::overload_cast<const systems::LinearSystem<double>&,
                          const Eigen::Ref<const Eigen::MatrixXd>&,
                          const Eigen::Ref<const Eigen::MatrixXd>&,
                          const Eigen::Ref<const Eigen::MatrixXd>&>(
            &LinearQuadraticRegulator),
        py::arg("system"), py::arg("Q"), py::arg("R"),
        py::arg("N") = Eigen::Matrix<double, 0, 0>::Zero());

  m.def("LinearQuadraticRegulator",
        py::overload_cast<const systems::System<double>&,
                          const systems::Context<double>&,
                          const Eigen::Ref<const Eigen::MatrixXd>&,
                          const Eigen::Ref<const Eigen::MatrixXd>&,
                          const Eigen::Ref<const Eigen::MatrixXd>&>(
            &LinearQuadraticRegulator),
        py::arg("system"), py::arg("context"), py::arg("Q"), py::arg("R"),
        py::arg("N") = Eigen::Matrix<double, 0, 0>::Zero());
}

}  // namespace pydrake
}  // namespace drake
