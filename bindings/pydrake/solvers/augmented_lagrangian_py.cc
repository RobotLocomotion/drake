#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/augmented_lagrangian.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(augmented_lagrangian, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "Augmented Lagrangian solver bindings for MathematicalProgram";

  py::module::import("pydrake.solvers.mathematicalprogram");
  py::module::import("pydrake.autodiffutils");

  py::class_<AugmentedLagrangianNonsmooth>(
      m, "AugmentedLagrangianNonsmooth", doc.AugmentedLagrangianNonsmooth.doc)
      .def(py::init<const MathematicalProgram*, bool>(), py::arg("prog"),
          py::arg("include_x_bounds"),
          doc.AugmentedLagrangianNonsmooth.ctor.doc)
      .def(
          "Eval",
          [](const AugmentedLagrangianNonsmooth* self,
              const Eigen::Ref<const Eigen::VectorXd>& x,
              const Eigen::VectorXd& lambda_val, double mu) {
            Eigen::VectorXd constraint_residue;
            double cost;
            const double al_val = self->Eval<double>(
                x, lambda_val, mu, &constraint_residue, &cost);
            return std::make_tuple(al_val, constraint_residue, cost);
          },
          py::arg("x"), py::arg("lambda_val"), py::arg("mu"),
          doc.AugmentedLagrangianNonsmooth.Eval.doc)
      .def(
          "Eval",
          [](const AugmentedLagrangianNonsmooth* self,
              const Eigen::Ref<const VectorX<AutoDiffXd>>& x,
              const Eigen::VectorXd& lambda_val, double mu) {
            VectorX<AutoDiffXd> constraint_residue;
            AutoDiffXd cost;
            const AutoDiffXd al_val = self->Eval<AutoDiffXd>(
                x, lambda_val, mu, &constraint_residue, &cost);
            return std::make_tuple(al_val, constraint_residue, cost);
          },
          py::arg("x"), py::arg("lambda_val"), py::arg("mu"),
          doc.AugmentedLagrangianNonsmooth.Eval.doc)
      .def("prog", &AugmentedLagrangianNonsmooth::prog, py_rvp::reference,
          doc.AugmentedLagrangianNonsmooth.prog.doc)
      .def("include_x_bounds", &AugmentedLagrangianNonsmooth::include_x_bounds,
          doc.AugmentedLagrangianNonsmooth.include_x_bounds.doc)
      .def("lagrangian_size", &AugmentedLagrangianNonsmooth::lagrangian_size,
          doc.AugmentedLagrangianNonsmooth.lagrangian_size.doc)
      .def("is_equality", &AugmentedLagrangianNonsmooth::is_equality,
          doc.AugmentedLagrangianNonsmooth.is_equality.doc)
      .def("x_lo", &AugmentedLagrangianNonsmooth::x_lo,
          py_rvp::reference_internal, doc.AugmentedLagrangianNonsmooth.x_lo.doc)
      .def("x_up", &AugmentedLagrangianNonsmooth::x_up,
          py_rvp::reference_internal,
          doc.AugmentedLagrangianNonsmooth.x_up.doc);
  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
