#include <memory>
#include <vector>

#include "drake/bindings/generated_docstrings/planning_graph_algorithms.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/graph_algorithms/max_clique_solver_base.h"
#include "drake/planning/graph_algorithms/max_clique_solver_via_greedy.h"
#include "drake/planning/graph_algorithms/max_clique_solver_via_mip.h"
#include "drake/planning/graph_algorithms/min_clique_cover_solver_base.h"
#include "drake/planning/graph_algorithms/min_clique_cover_solver_via_greedy.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningGraphAlgorithms(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning::graph_algorithms;
  constexpr auto& doc =
      pydrake_doc_planning_graph_algorithms.drake.planning.graph_algorithms;
  {
    class PyMaxCliqueSolverBase : public MaxCliqueSolverBase {
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
  {
    const auto& cls_doc = doc.MaxCliqueSolverViaGreedy;
    py::class_<MaxCliqueSolverViaGreedy, MaxCliqueSolverBase>(
        m, "MaxCliqueSolverViaGreedy", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc);
  }
  {
    class PyMinCliqueCoverSolverBase : public MinCliqueCoverSolverBase {
     public:
      // Trampoline virtual methods.
      // The private virtual method of DoSolveMinCliqueCover is made public to
      // enable Python implementations to override it.
      std::vector<std::set<int>> DoSolveMinCliqueCover(
          const Eigen::SparseMatrix<bool>& adjacency_matrix,
          bool partition) override {
        PYBIND11_OVERRIDE_PURE(std::vector<std::set<int>>,
            MinCliqueCoverSolverBase, DoSolveMinCliqueCover, adjacency_matrix,
            partition);
      }
    };
    const auto& cls_doc = doc.MinCliqueCoverSolverBase;
    py::class_<MinCliqueCoverSolverBase, PyMinCliqueCoverSolverBase>(
        m, "MinCliqueCoverSolverBase", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("SolveMinCliqueCover",
            &MinCliqueCoverSolverBase::SolveMinCliqueCover,
            py::arg("adjacency_matrix"), py::arg("partition") = false,
            cls_doc.SolveMinCliqueCover.doc);
  }
  {
    const auto& cls_doc = doc.MinCliqueCoverSolverViaGreedy;
    py::class_<MinCliqueCoverSolverViaGreedy, MinCliqueCoverSolverBase>(
        m, "MinCliqueCoverSolverViaGreedy", cls_doc.doc)
        .def(py::init([](MaxCliqueSolverBase& max_clique_solver,
                          int min_clique_size) {
          // The keep_alive is responsible for object lifetime, so we'll give
          // the constructor an unowned pointer.
          return std::make_unique<MinCliqueCoverSolverViaGreedy>(
              make_unowned_shared_ptr_from_raw(&max_clique_solver),
              min_clique_size);
        }),
            py::arg("max_clique_solver"), py::arg("min_clique_size") = 1,
            // Keep alive, reference: `self` keeps `max_clique_solver` alive.
            py::keep_alive<1, 2>(), cls_doc.ctor.doc)
        .def("set_min_clique_size",
            &MinCliqueCoverSolverViaGreedy::set_min_clique_size,
            py::arg("min_clique_size"), cls_doc.set_min_clique_size.doc)
        .def("get_min_clique_size",
            &MinCliqueCoverSolverViaGreedy::get_min_clique_size,
            cls_doc.get_min_clique_size.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
