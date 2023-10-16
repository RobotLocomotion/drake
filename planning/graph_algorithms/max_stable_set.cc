#include "drake/planning/graph_algorithms/max_stable_set.h"

namespace drake {
namespace planning {

MaxStableSetSolverViaMIP::MaxStableSetSolverViaMIP(
    const solvers::SolverId& solver_id)
    : solver_id_(solver_id){};

VectorX<bool> MaxStableSetSolverViaMIP::SolveMaxStableSet(SparseMatrix<bool> adjacency_matrix) {
  MathematicalProgram prog;
  VectorX<DecisionVariables>
}


}  // namespace planning
}  // namespace drake