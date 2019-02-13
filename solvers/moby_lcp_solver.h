// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#pragma once

#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_base.h"

// TODO(jwnimmer-tri): This class should be renamed MobyLcpSolver to comply with
//                     style guide.

namespace drake {
namespace solvers {

/// Non-template class for MobyLcpSolver<T> constants.
class MobyLcpSolverId {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobyLcpSolverId);
  MobyLcpSolverId() = delete;

  /// @return same as SolverInterface::solver_id()
  static SolverId id();
};

/// A class for solving Linear Complementarity Problems (LCPs). Solving a LCP
/// requires finding a solution to the problem:<pre>
/// Mz + q = w
/// z ≥ 0
/// w ≥ 0
/// zᵀw = 0
/// </pre>
/// (where M ∈ ℝⁿˣⁿ and q ∈ ℝⁿ are problem inputs and z ∈ ℝⁿ and w ∈ ℝⁿ are
/// unknown vectors) or correctly reporting that such a solution does not exist.
/// In spite of their linear structure, solving LCPs is NP-Hard [Cottle 1992].
/// However, some LCPs are significantly easier to solve. For instance, it can
/// be seen that the LCP is solvable in worst-case polynomial time for the case
/// of symmetric positive-semi-definite M by formulating it as the following
/// convex quadratic program:<pre>
/// minimize:   f(z) = zᵀw = zᵀ(Mz + q)
/// subject to: z ≥ 0
///             Mz + q ≥ 0
/// </pre>
/// Note that this quadratic program's (QP) objective function at the minimum
/// z cannot be less than zero, and the LCP is only solved if the objective
/// function at the minimum is equal to zero. Since the seminal result of
/// Karmarkar, it has been known that convex QPs are solvable in polynomial
/// time [Karmarkar 1984].
///
/// The difficulty of solving an LCP is characterized by the properties of its
/// particular matrix, namely the class of matrices it belongs to. Classes
/// include, for example, Q, Q₀, P, P₀, copositive, and Z matrices.
/// [Cottle 1992] and [Murty 1998] (see pp. 224-230 in the latter) describe
/// relevant matrix classes in more detail.
///
/// * [Cottle 1992]     R. Cottle, J.-S. Pang, and R. Stone. The Linear
///                     Complementarity Problem. Academic Press, 1992.
/// * [Karmarkar 1984]  N. Karmarkar. A New Polynomial-Time Algorithm for
///                     Linear Programming. Combinatorica, 4(4), pp. 373-395.
/// * [Murty 1988]      K. Murty. Linear Complementarity, Linear and Nonlinear
///                     Programming. Heldermann Verlag, 1988.
template <class T>
class MobyLCPSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobyLCPSolver)

  MobyLCPSolver();
  ~MobyLCPSolver() final;

  void SetLoggingEnabled(bool enabled);

  /// Calculates the zero tolerance that the solver would compute if the user
  /// does not specify a tolerance.
  template <class U>
  static U ComputeZeroTolerance(const MatrixX<U>& M) {
    return M.rows() * M.template lpNorm<Eigen::Infinity>() *
        (10 * std::numeric_limits<double>::epsilon());
  }

  /// Fast pivoting algorithm for LCPs of the form M = PAPᵀ, q = Pb, where
  /// b ∈ ℝᵐ, P ∈ ℝⁿˣᵐ, and A ∈ ℝᵐˣᵐ (where A is positive definite). Therefore,
  /// q is in the range of P and M is positive semi-definite. An LCP of this
  /// form is also guaranteed to have a solution [Cottle 1992].
  ///
  /// This particular implementation focuses on the case where the solution
  /// requires few nonzero nonbasic variables, meaning that few z variables
  /// need be nonzero to find a solution to Mz + q = w. This algorithm, which is
  /// based off of Dantzig's Principle Pivoting Method I [Cottle 1992] is
  /// described in [Drumwright 2015]. This algorithm is able to use "warm"
  /// starting- a solution to a "nearby" LCP can be used to find the solution to
  /// a given LCP more quickly.
  ///
  /// Although this solver is theoretically guaranteed to give a solution to
  /// the LCPs described above, accumulated floating point error from pivoting
  /// operations could cause the solver to fail. Additionally, the solver can be
  /// applied with some success to problems outside of its guaranteed matrix
  /// class. For these reasons, the solver returns a flag indicating
  /// success/failure.
  /// @param[in] M the LCP matrix.
  /// @param[in] q the LCP vector.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use z's value as a starting
  ///                solution. If the solver fails (returns `false`), `z` will
  ///                be set to the zero vector.
  /// @param[in] zero_tol The tolerance for testing against zero. If the
  ///            tolerance is negative (default) the solver will determine a
  ///            generally reasonable tolerance.
  /// @throws std::logic_error if M is non-square or M's dimensions do not
  ///         equal q's dimension.
  /// @returns `true` if the solver succeeded and `false` otherwise.
  ///
  /// * [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
  ///                      Complementarity Problem. Academic Press, 1992.
  /// * [Drumwright 2015]  E. Drumwright. Rapidly computable viscous friction
  ///                      and no-slip rigid contact models. arXiv:
  ///                      1504.00719v1. 2015.
  bool SolveLcpFast(const MatrixX<T>& M, const VectorX<T>& q,
                    VectorX<T>* z, const T& zero_tol = T(-1)) const;

  /// Regularized version of the fast pivoting algorithm for LCPs of the form
  /// M = PAPᵀ, q = Pb, where b ∈ ℝᵐ, P ∈ ℝⁿˣᵐ, and A ∈ ℝᵐˣᵐ (where A is
  /// positive definite). Therefore, q is in the range of P and M is positive
  /// semi-definite. Please see SolveLcpFast() for more documentation about the
  /// particular algorithm.
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
  /// values of α, until the LCP is solved. Cottle et al. describe how, for
  /// sufficiently large α, the LCP will always be solvable [Cottle 1992], p.
  /// 493.
  ///
  /// Although this solver is theoretically guaranteed to give a solution to
  /// the LCPs described above, accumulated floating point error from pivoting
  /// operations could cause the solver to fail. Additionally, the solver can be
  /// applied with some success to problems outside of its guaranteed matrix
  /// class. For these reasons, the solver returns a flag indicating
  /// success/failure.
  /// @param[in] M the LCP matrix.
  /// @param[in] q the LCP vector.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use z's value as a starting
  ///                solution.
  /// @param[in] min_exp The minimum exponent for computing α over [10ᵝ, 10ᵞ] in
  ///                    steps of 10ᵟ, where β is the minimum exponent, γ is the
  ///                    maximum exponent, and δ is the stepping exponent.
  /// @param[in] step_exp The stepping exponent for computing α over [10ᵝ, 10ᵞ]
  ///                     in steps of 10ᵟ, where β is the minimum exponent, γ is
  ///                     the maximum exponent, and δ is the stepping exponent.
  /// @param[in] max_exp The maximum exponent for computing α over [10ᵝ, 10ᵞ] in
  ///                    steps of 10ᵟ, where β is the minimum exponent, γ is the
  ///                    maximum exponent, and δ is the stepping exponent.
  /// @param[in] zero_tol The tolerance for testing against zero. If the
  ///            tolerance is negative (default) the solver will determine a
  ///            generally reasonable tolerance.
  /// @throws std::logic_error if M is non-square or M's dimensions do not
  ///         equal q's dimension.
  /// @returns `true` if the solver succeeded and `false` if the solver did not
  ///          find a solution for α = 10ᵞ.
  /// @sa SolveLcpFast()
  ///
  /// * [Cottle, 1992]     R. Cottle, J.-S. Pang, and R. Stone. The Linear
  ///                      Complementarity Problem. Academic Press, 1992.
  bool SolveLcpFastRegularized(const MatrixX<T>& M,
                               const VectorX<T>& q, VectorX<T>* z,
                               int min_exp = -20, unsigned step_exp = 4,
                               int max_exp = 20, const T& zero_tol = T(-1))
                               const;

  /// Lemke's Algorithm for solving LCPs in the matrix class E, which contains
  /// all strictly semimonotone matrices, all P-matrices, and all strictly
  /// copositive matrices. Lemke's Algorithm is described in [Cottle 1992],
  /// Section 4.4. This implementation was adapted from the LEMKE Library
  /// [LEMKE] for Matlab; this particular implementation fixes a bug
  /// in LEMKE that could occur when multiple indices passed the minimum ratio
  /// test.
  ///
  /// Although this solver is theoretically guaranteed to give a solution to
  /// the LCPs described above, accumulated floating point error from pivoting
  /// operations could cause the solver to fail. Additionally, the solver can be
  /// applied with some success to problems outside of its guaranteed matrix
  /// classes. For these reasons, the solver returns a flag indicating
  /// success/failure.
  /// @param[in] M the LCP matrix.
  /// @param[in] q the LCP vector.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use z's value as a starting
  ///                solution. **This warmstarting is generally not
  ///                recommended**: it has a predisposition to lead to a failing
  ///                pivoting sequence. If the solver fails (returns `false`),
  ///                `z` will be set to the zero vector.
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
  /// * [LEMKE]          P. Fackler and M. Miranda. LEMKE.
  ///                    http://people.sc.fsu.edu/~burkardt/m\_src/lemke/lemke.m
  bool SolveLcpLemke(const MatrixX<T>& M, const VectorX<T>& q,
                     VectorX<T>* z, const T& piv_tol = T(-1),
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
                                int min_exp = -20, unsigned step_exp = 1,
                                int max_exp = 1, const T& piv_tol = T(-1),
                                const T& zero_tol = T(-1)) const;

  /// Returns the number of pivoting operations made by the last LCP solve.
  int get_num_pivots() const { return pivots_; }

  /// Resets the number of pivoting operations made by the last LCP solver to
  /// zero.
  void reset_num_pivots() { pivots_ = 0; }

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;

  void ClearIndexVectors() const;

  template <typename MatrixType, typename Scalar>
  void FinishLemkeSolution(const MatrixType& M, const VectorX<Scalar>& q,
                           const VectorX<Scalar>& x, VectorX<Scalar>* z) const;

  // TODO(sammy-tri) replace this with a proper logging hookup
  std::ostream& Log() const;

  bool log_enabled_{false};
  mutable std::ofstream null_stream_;

  // Records the number of pivoting operations used during the last solve.
  mutable unsigned pivots_{0};

  // NOTE:  The temporaries below are stored in the class to minimize
  // allocations; all are marked 'mutable' as they do not affect the
  // semantic const'ness of the class under its methods.
  // Vectors which correspond to indices into other data.
  mutable std::vector<unsigned> all_, tlist_, bas_, nonbas_, j_;
};

}  // end namespace solvers
}  // end namespace drake
