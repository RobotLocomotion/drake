#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/graph_algorithms/max_clique_solver_base.h"
#include "drake/planning/graph_algorithms/max_clique_solver_via_mip.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningGraphAlgorithms(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning::graph_algorithms;
  constexpr auto& doc = pydrake_doc.drake.planning.graph_algorithms;
  {
    class PyMaxCliqueSolverBase : public py::wrapper<MaxCliqueSolverBase> {
     public:
      // Trampoline virtual methods.
      // The private virtual method of DoSolveMaxClique is made public to enable
      // Python implementations to override it.
      VectorX<bool> DoSolveMaxClique(
          const Eigen::SparseMatrix<bool>& adjacency_matrix) const override {
        PYBIND11_OVERRIDE_PURE(VectorX<bool>, MaxCliqueSolverBase,
            DoSolveMaxClique, adjacency_matrix);
      }
    };
    const auto& cls_doc = doc.MaxCliqueSolverBase;
    py::class_<MaxCliqueSolverBase, PyMaxCliqueSolverBase>(
        m, "MaxCliqueSolverBase", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("SolveMaxClique", &MaxCliqueSolverBase::SolveMaxClique,
            py::arg("adjacency_matrix"), cls_doc.SolveMaxClique.doc);
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
        .def("SetSolverOptions", &MaxCliqueSolverViaMip::SetSolverOptions,
            py::arg("solver_options"), cls_doc.SetSolverOptions.doc)
        .def("GetSolverOptions", &MaxCliqueSolverViaMip::GetSolverOptions,
            cls_doc.GetSolverOptions.doc)
        .def("SetInitialGuess", &MaxCliqueSolverViaMip::SetInitialGuess,
            py::arg("initial_guess"), cls_doc.SetInitialGuess.doc)
        .def("GetInitialGuess", &MaxCliqueSolverViaMip::GetInitialGuess,
            cls_doc.GetInitialGuess.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
