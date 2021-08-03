#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/dreal_solver.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(dreal, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  m.doc() = "dReal solver bindings for MathematicalProgram";

  py::module::import("pydrake.solvers.mathematicalprogram");
  py::module::import("pydrake.symbolic");

  auto solver = py::class_<DrealSolver, SolverInterface>(
      m, "DrealSolver", doc.DrealSolver.doc)
                    .def_static("id", &DrealSolver::id, doc.DrealSolver.id.doc);

  {
    using Class = DrealSolver::Interval;
    const auto& cls_doc = doc.DrealSolver.Interval;
    py::class_<Class>(solver, "Interval", cls_doc.doc)
        .def(py::init<double, double>(), py::arg("low"), py::arg("high"),
            cls_doc.ctor.doc)
        .def("diam", &Class::diam, cls_doc.diam.doc)
        .def("mid", &Class::mid, cls_doc.mid.doc)
        .def("low", &Class::low, cls_doc.low.doc)
        .def("high", &Class::high, cls_doc.high.doc);
  }

  {
    using Class = DrealSolver::LocalOptimization;
    const auto& cls_doc = doc.DrealSolver.LocalOptimization;
    py::enum_<Class>(solver, "LocalOptimization", cls_doc.doc)
        .value("kUse", Class::kUse, cls_doc.kUse.doc)
        .value("kNotUse", Class::kNotUse, cls_doc.kNotUse.doc);
  }

  solver.def(py::init<>(), doc.DrealSolver.ctor.doc)
      .def_static("CheckSatisfiability", &DrealSolver::CheckSatisfiability,
          py::arg("f"), py::arg("delta"),
          doc.DrealSolver.CheckSatisfiability.doc)
      .def_static("Minimize", &DrealSolver::Minimize, py::arg("objective"),
          py::arg("constraint"), py::arg("delta"),
          py::arg("local_optimization"), doc.DrealSolver.Minimize.doc);
}

}  // namespace pydrake
}  // namespace drake
