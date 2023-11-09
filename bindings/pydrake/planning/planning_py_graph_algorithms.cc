#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/graph_algorithms/max_clique.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningGraphAlgorithms(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning::graph_algorithms;
  constexpr auto& doc = pydrake_doc.drake.planning.graph_algorithms;
  {
    const auto& cls_doc = doc.MaxCliqueSolverBase;
    py::class_<MaxCliqueSolverBase>(m, "MaxCliqueSolverBase", cls_doc.doc)
        .def("SolveMaxClique", &MaxCliqueSolverBase::SolveMaxClique,
            cls_doc.SolveMaxClique.doc);
  }
  {
    const auto& cls_doc = doc.MaxCliqueSolverViaMip;
    py::class_<MaxCliqueSolverViaMip, MaxCliqueSolverBase>(
        m, "MaxCliqueSolverViaMip", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<const std::optional<Eigen::VectorXd>&,
                 const solvers::SolverOptions&>(),
            py::arg("initial_guess"), py::arg("solver_options"),
            cls_doc.ctor.doc)
        .def("solver_options", &MaxCliqueSolverViaMip::solver_options,
            py_rvp::reference_internal,
            cls_doc.solver_options.doc)
        .def("set_initial_guess", &MaxCliqueSolverViaMip::set_initial_guess,
            py::arg("initial_guess"), cls_doc.set_initial_guess.doc)
        .def("get_initial_guess", &MaxCliqueSolverViaMip::get_initial_guess,
            cls_doc.get_initial_guess.doc)
        .def("get_solver_options", &MaxCliqueSolverViaMip::get_solver_options,
            cls_doc.get_solver_options.doc);
  }
  {
    const auto& cls_doc = doc.MaxCliqueOptions;
    py::class_<MaxCliqueOptions>(m, "MaxCliqueOptions", cls_doc.doc)
        .def_readwrite("solver", &MaxCliqueOptions::solver, py_rvp::reference);
  }
  {
    m.def("CalcMaxClique", &CalcMaxClique, py::arg("adjacency_matrix"),
        py::arg("options"), doc.CalcMaxClique.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake