#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/optimization/static_equilibrium_problem.h"

namespace drake {
namespace pydrake {

namespace {
PYBIND11_MODULE(optimization, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "Optimization module for MultibodyPlant motion planning";

  py::module::import("pydrake.math");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.solvers.mathematicalprogram");

  {
    py::class_<ContactWrench>(m, "ContactWrench", doc.ContactWrench.doc)
        .def_readonly("bodyA_index", &ContactWrench::bodyA_index,
            doc.ContactWrench.bodyA_index.doc)
        .def_readonly("bodyB_index", &ContactWrench::bodyB_index,
            doc.ContactWrench.bodyB_index.doc)
        .def_readonly(
            "p_WCb_W", &ContactWrench::p_WCb_W, doc.ContactWrench.p_WCb_W.doc)
        .def_readonly(
            "F_Cb_W", &ContactWrench::F_Cb_W, doc.ContactWrench.F_Cb_W.doc);
    AddValueInstantiation<ContactWrench>(m);
  }

  {
    using Class = StaticEquilibriumProblem;
    constexpr auto& cls_doc = doc.StaticEquilibriumProblem;
    py::class_<Class>(m, "StaticEquilibriumProblem", cls_doc.doc)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 systems::Context<AutoDiffXd>*,
                 const std::set<
                     std::pair<geometry::GeometryId, geometry::GeometryId>>&>(),
            py::arg("plant"), py::arg("context"),
            py::arg("ignored_collision_pairs"),
            // Keep alive, reference: `self` keeps `plant` and `context` alive.
            py::keep_alive<1, 2>(), py::keep_alive<1, 3>(), cls_doc.ctor.doc)
        .def("get_mutable_prog", &Class::get_mutable_prog,
            py_rvp::reference_internal, cls_doc.get_mutable_prog.doc)
        .def("prog", &Class::prog, py_rvp::reference_internal, cls_doc.prog.doc)
        .def("q_vars", &Class::q_vars, cls_doc.q_vars.doc)
        .def("u_vars", &Class::u_vars, cls_doc.u_vars.doc)
        .def("GetContactWrenchSolution", &Class::GetContactWrenchSolution,
            py::arg("result"), cls_doc.GetContactWrenchSolution.doc)
        .def("UpdateComplementarityTolerance",
            &Class::UpdateComplementarityTolerance, py::arg("tol"),
            cls_doc.UpdateComplementarityTolerance.doc);
  }
}
}  // namespace
}  // namespace pydrake
}  // namespace drake
