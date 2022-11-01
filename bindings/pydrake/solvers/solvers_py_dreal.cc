#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/solvers/solvers_py.h"
#include "drake/solvers/dreal_solver.h"

namespace drake {
namespace pydrake {
namespace internal {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void DefineSolversDreal(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::solvers;
  constexpr auto& doc = pydrake_doc.drake.solvers;

  auto solver = py::class_<DrealSolver, SolverInterface>(
      m, "DrealSolver", doc.DrealSolver.doc_deprecated)
                    .def_static("id",
                        WrapDeprecated(doc.DrealSolver.id.doc_deprecated,
                            &DrealSolver::id),
                        doc.DrealSolver.id.doc_deprecated);

  {
    using Class = DrealSolver::Interval;
    const auto& cls_doc = doc.DrealSolver.Interval;
    py::class_<Class>(solver, "Interval", cls_doc.doc_deprecated)
        .def(py_init_deprecated<Class, double, double>(cls_doc.doc_deprecated),
            py::arg("low"), py::arg("high"), cls_doc.doc_deprecated)
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

  solver
      .def(py_init_deprecated<DrealSolver>(doc.DrealSolver.ctor.doc_deprecated),
          doc.DrealSolver.ctor.doc_deprecated)
      .def_static("CheckSatisfiability",
          WrapDeprecated(doc.DrealSolver.CheckSatisfiability.doc_deprecated,
              &DrealSolver::CheckSatisfiability),
          py::arg("f"), py::arg("delta"),
          doc.DrealSolver.CheckSatisfiability.doc_deprecated)
      .def_static("Minimize",
          WrapDeprecated(
              doc.DrealSolver.Minimize.doc_deprecated, &DrealSolver::Minimize),
          py::arg("objective"), py::arg("constraint"), py::arg("delta"),
          py::arg("local_optimization"),
          doc.DrealSolver.Minimize.doc_deprecated);
}
#pragma GCC diagnostic pop

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
