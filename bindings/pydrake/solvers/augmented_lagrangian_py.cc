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

  py::class_<NonsmoothAugmentedLagrangian>(
      m, "NonsmoothAugmentedLagrangian", doc.NonsmoothAugmentedLagrangian.doc)
      .def(py::init<const MathematicalProgram*, bool>(), py::arg("prog"),
          py::arg("include_x_bounds"),
          doc.NonsmoothAugmentedLagrangian.ctor.doc)
      .def(
          "Eval",
          [](const NonsmoothAugmentedLagrangian* self,
              const Eigen::Ref<const Eigen::VectorXd>& x,
              const Eigen::VectorXd& lambda_val, double mu) {
            Eigen::VectorXd constraint_residue;
            double cost;
            const double al_val = self->Eval<double>(
                x, lambda_val, mu, &constraint_residue, &cost);
            return std::make_tuple(al_val, constraint_residue, cost);
          },
          py::arg("x"), py::arg("lambda_val"), py::arg("mu"),
          doc.NonsmoothAugmentedLagrangian.Eval.doc)
      .def(
          "Eval",
          [](const NonsmoothAugmentedLagrangian* self,
              const Eigen::Ref<const VectorX<AutoDiffXd>>& x,
              const Eigen::VectorXd& lambda_val, double mu) {
            VectorX<AutoDiffXd> constraint_residue;
            AutoDiffXd cost;
            const AutoDiffXd al_val = self->Eval<AutoDiffXd>(
                x, lambda_val, mu, &constraint_residue, &cost);
            return std::make_tuple(al_val, constraint_residue, cost);
          },
          py::arg("x"), py::arg("lambda_val"), py::arg("mu"),
          doc.NonsmoothAugmentedLagrangian.Eval.doc)
      .def("prog", &NonsmoothAugmentedLagrangian::prog, py_rvp::reference,
          doc.NonsmoothAugmentedLagrangian.prog.doc)
      .def("include_x_bounds", &NonsmoothAugmentedLagrangian::include_x_bounds,
          doc.NonsmoothAugmentedLagrangian.include_x_bounds.doc)
      .def("lagrangian_size", &NonsmoothAugmentedLagrangian::lagrangian_size,
          doc.NonsmoothAugmentedLagrangian.lagrangian_size.doc)
      .def("is_equality", &NonsmoothAugmentedLagrangian::is_equality,
          doc.NonsmoothAugmentedLagrangian.is_equality.doc)
      .def("x_lo", &NonsmoothAugmentedLagrangian::x_lo,
          py_rvp::reference_internal, doc.NonsmoothAugmentedLagrangian.x_lo.doc)
      .def("x_up", &NonsmoothAugmentedLagrangian::x_up,
          py_rvp::reference_internal,
          doc.NonsmoothAugmentedLagrangian.x_up.doc);
}
}  // namespace pydrake
}  // namespace drake
