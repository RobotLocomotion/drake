#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/augmented_lagrangian.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineSolversAugmentedLagrangian(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  {
    using Class = AugmentedLagrangianNonsmooth;
    constexpr auto& cls_doc = doc.AugmentedLagrangianNonsmooth;
    py::class_<AugmentedLagrangianNonsmooth>(
        m, "AugmentedLagrangianNonsmooth", cls_doc.doc)
        .def(py::init<const MathematicalProgram*, bool>(), py::arg("prog"),
            py::arg("include_x_bounds"), cls_doc.ctor.doc)
        .def(
            "Eval",
            [](const Class* self, const Eigen::Ref<const Eigen::VectorXd>& x,
                const Eigen::VectorXd& lambda_val, double mu) {
              Eigen::VectorXd constraint_residue;
              double cost;
              const double al_val = self->Eval<double>(
                  x, lambda_val, mu, &constraint_residue, &cost);
              return std::make_tuple(al_val, constraint_residue, cost);
            },
            py::arg("x"), py::arg("lambda_val"), py::arg("mu"),
            cls_doc.Eval.doc)
        .def(
            "Eval",
            [](const Class* self,
                const Eigen::Ref<const VectorX<AutoDiffXd>>& x,
                const Eigen::VectorXd& lambda_val, double mu) {
              VectorX<AutoDiffXd> constraint_residue;
              AutoDiffXd cost;
              const AutoDiffXd al_val = self->Eval<AutoDiffXd>(
                  x, lambda_val, mu, &constraint_residue, &cost);
              return std::make_tuple(al_val, constraint_residue, cost);
            },
            py::arg("x"), py::arg("lambda_val"), py::arg("mu"),
            cls_doc.Eval.doc)
        .def("prog", &Class::prog, py_rvp::reference, cls_doc.prog.doc)
        .def("include_x_bounds", &Class::include_x_bounds,
            cls_doc.include_x_bounds.doc)
        .def("lagrangian_size", &Class::lagrangian_size,
            cls_doc.lagrangian_size.doc)
        .def("is_equality", &Class::is_equality, cls_doc.is_equality.doc)
        .def("x_lo", &Class::x_lo, py_rvp::reference_internal, cls_doc.x_lo.doc)
        .def(
            "x_up", &Class::x_up, py_rvp::reference_internal, cls_doc.x_up.doc);
  }

  {
    using Class = AugmentedLagrangianSmooth;
    constexpr auto& cls_doc = doc.AugmentedLagrangianSmooth;
    py::class_<Class>(m, "AugmentedLagrangianSmooth", cls_doc.doc)
        .def(py::init<const MathematicalProgram*, bool>(), py::arg("prog"),
            py::arg("include_x_bounds"), cls_doc.ctor.doc)
        .def(
            "Eval",
            [](const Class* self, const Eigen::Ref<const Eigen::VectorXd>& x,
                const Eigen::Ref<const Eigen::VectorXd>& s,
                const Eigen::VectorXd& lambda_val, double mu) {
              Eigen::VectorXd constraint_residue;
              double cost;
              const double al_val = self->Eval<double>(
                  x, s, lambda_val, mu, &constraint_residue, &cost);
              return std::make_tuple(al_val, constraint_residue, cost);
            },
            py::arg("x"), py::arg("s"), py::arg("lambda_val"), py::arg("mu"),
            cls_doc.Eval.doc)
        .def(
            "Eval",
            [](const Class* self,
                const Eigen::Ref<const VectorX<AutoDiffXd>>& x,
                const Eigen::Ref<const VectorX<AutoDiffXd>>& s,
                const Eigen::VectorXd& lambda_val, double mu) {
              VectorX<AutoDiffXd> constraint_residue;
              AutoDiffXd cost;
              const AutoDiffXd al_val = self->Eval<AutoDiffXd>(
                  x, s, lambda_val, mu, &constraint_residue, &cost);
              return std::make_tuple(al_val, constraint_residue, cost);
            },
            py::arg("x"), py::arg("s"), py::arg("lambda_val"), py::arg("mu"),
            cls_doc.Eval.doc)
        .def("prog", &Class::prog, py_rvp::reference, cls_doc.prog.doc)
        .def("include_x_bounds", &Class::include_x_bounds,
            cls_doc.include_x_bounds.doc)
        .def("s_size", &Class::s_size, cls_doc.s_size.doc)
        .def("lagrangian_size", &Class::lagrangian_size,
            cls_doc.lagrangian_size.doc)
        .def("is_equality", &Class::is_equality, cls_doc.is_equality.doc)
        .def("x_lo", &Class::x_lo, py_rvp::reference_internal, cls_doc.x_lo.doc)
        .def(
            "x_up", &Class::x_up, py_rvp::reference_internal, cls_doc.x_up.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
