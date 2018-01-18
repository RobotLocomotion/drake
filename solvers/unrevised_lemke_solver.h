#pragma once

#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

/// Non-template class for UnrevisedLemkeSolver<T> constants.
class UnrevisedLemkeSolverId {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrevisedLemkeSolverId);
  UnrevisedLemkeSolverId() = delete;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();
};

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
  /// copositive matrices. Lemke's Algorithm is described in [Cottle 1992],
  /// Section 4.4. 
  ///
  /// Although this solver is theoretically guaranteed to give a solution to
  /// the LCPs described above, cycling from degeneracy could cause the solver
  /// to fail (by exceeding the maximum number of iterations). Additionally,
  /// the solver can be applied with occasional success to problems outside of
  /// its guaranteed matrix classes. For these reasons, the solver returns a
  /// flag indicating success/failure.
  /// @param[in] M the LCP matrix.
  /// @param[in] q the LCP vector.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use the basis implied by z's
  ///                value as a solution. This strategy can prove exceptionally
  ///                fast if solutions differ little between successive calls.
  ///                If the solver fails (returns `false`),
  ///                `z` will be set to the zero vector on return.
  /// @param[out] num_pivots the number of pivots used, on return.
  /// @param[in] zero_tol The tolerance for testing against zero. If the
  ///            tolerance is negative (default) the solver will determine a
  ///            generally reasonable tolerance.
  /// @param[in] piv_tol The tolerance for testing against zero, specifically
  ///            used for the purpose of finding variables for pivoting. If the
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
                     VectorX<T>* z, int* num_pivots, const T& piv_tol = T(-1),
                     const T& zero_tol = T(-1)) const;

  /// Lemke's Algorithm for solving LCPs in the matrix class E, which contains
  /// all strictly semimonotone matrices, all P-matrices, and all strictly
  /// copositive matrices. Lemke's Algorithm is described in [Cottle 1992],
  /// Section 4.4.
  ///
  /// This implementation wraps that algorithm with a Tikhonov-type
  /// regularization approach. Specifically, this implementation repeatedly
  /// attempts to solve the LCP:<pre>
  /// (M + Iα)z + q = w
  /// z ≥ 0
  /// w ≥ 0
  /// zᵀw = 0
  /// </pre>
  /// where I is the identity matrix and α ≪ 1, using geometrically increasing
  /// values of α, until the LCP is solved. See SolveLcpFastRegularized() for
  /// description of the regularization process and the function parameters,
  /// which are identical. See SolveLcpLemke() for a description of Lemke's
  /// Algorithm. See SolveLcpFastRegularized() for a description of all
  /// calling parameters other than @p z, which apply equally well to this
  /// function.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use z's value as a starting
  ///                solution. **This warmstarting is generally not
  ///                recommended**: it has a predisposition to lead to a failing
  ///                pivoting sequence.
  ///
  /// @sa SolveLcpFastRegularized()
  /// @sa SolveLcpLemke()
  ///
  /// * [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
  ///                      Complementarity Problem. Academic Press, 1992.
  bool SolveLcpLemkeRegularized(const MatrixX<T>& M,
                                const VectorX<T>& q, VectorX<T>* z,
                                int* num_pivots,
                                int min_exp = -20, unsigned step_exp = 1,
                                int max_exp = 1, const T& piv_tol = T(-1),
                                const T& zero_tol = T(-1)) const;

  bool available() const override { return true; }

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

 private:
  // A structure for holding a linear complementarity problem variable.
  struct LCPVariable {
    bool z{true};        // Is this a z variable or a w variable?
    bool indep{true};    // Is this an independent (right-hand-side) or
                         // dependent (left-hand-side) variable in w* = Mz* + q.
    int index{-1};       // Index of the variable in the problem, 0...n. n 
                         // indicates that the variable is artificial.
  };

  void ClearIndexVectors() const;

  template <typename MatrixType, typename Scalar>
  void FinishLemkeSolution(const MatrixType& M, const VectorX<Scalar>& q,
                           const VectorX<Scalar>& x, VectorX<Scalar>* z) const;

  // NOTE:  The temporaries below are stored in the class to minimize
  // allocations; all are marked 'mutable' as they do not affect the
  // semantic const'ness of the class under its methods.
  // Vectors which correspond to indices into other data.
  mutable std::vector<unsigned> all_, tlist_, bas_, nonbas_, j_;
};

}  // end namespace solvers
}  // end namespace drake
