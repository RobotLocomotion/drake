#pragma once

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

/// A class for the Unrevised Implementation of Lemke Algorithm's for solving
/// Linear Complementarity Problems (LCPs). See MobyLcpSolver for a description
/// of LCPs.
template <class T>
class UnrevisedLemkeSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrevisedLemkeSolver)

  UnrevisedLemkeSolver() = default;
  ~UnrevisedLemkeSolver() override = default;

  /// Calculates the zero tolerance that the solver would compute if the user
  /// does not specify a tolerance.
  template <class U>
  static U ComputeZeroTolerance(const MatrixX<U>& M) {
    return M.rows() * M.template lpNorm<Eigen::Infinity>() *
        (10 * std::numeric_limits<double>::epsilon());
  }

  /// Lemke's Algorithm for solving LCPs in the matrix class E, which contains
  /// all strictly semimonotone matrices, all P-matrices, and all strictly
  /// copositive matrices. The solver can be applied with occasional success to
  /// problems outside of its guaranteed matrix classes. Lemke's Algorithm is
  /// described in [Cottle 1992], Section 4.4.
  ///
  /// The solver will denote failure on return if it exceeds a problem-size
  /// dependent number of iterations.
  /// @param[in] M the LCP matrix.
  /// @param[in] q the LCP vector.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use the basis from the last
  ///                solution. This strategy can prove exceptionally
  ///                fast if solutions differ little between successive calls.
  ///                If the solver fails (returns `false`),
  ///                `z` will be set to the zero vector on return.
  /// @param[out] num_pivots the number of pivots used, on return.
  /// @param[in] zero_tol The tolerance for testing against zero. If the
  ///            tolerance is negative (default) the solver will determine a
  ///            generally reasonable tolerance.
  /// @returns `true` if the solver **believes** it has computed a solution
  ///          (which it determines by the ability to "pivot out" the
  ///          "artificial" variable (see [Cottle 1992]) and `false` otherwise.
  /// @warning The caller should verify that the algorithm has solved the LCP to
  ///          the desired tolerances on returns indicating success.
  /// @throws std::logic_error if M is not square or the dimensions of M do not
  ///         match the length of q.
  ///
  /// * [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
  ///                      Complementarity Problem. Academic Press, 1992.
  bool SolveLcpLemke(const MatrixX<T>& M, const VectorX<T>& q,
                     VectorX<T>* z, int* num_pivots,
                     const T& zero_tol = T(-1)) const;

  bool available() const override { return true; }

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();
};

}  // end namespace solvers
}  // end namespace drake
