// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#pragma once

#include <fstream>
#include <vector>
#include <string>

#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

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
/// 'z' cannot be less than zero, and the LCP is only solved if the objective
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
/// [Cottle 1992]     R. Cottle, J.-S. Pang, and R. Stone. The Linear
///                   Complementarity Problem. Academic Press, 1992.
/// [Karmarkar 1984]  N. Karmarkar. A New Polynomial-Time Algorithm for
///                   Linear Programming. Combinatorica, 4(4), pp. 373-395.
/// [Murty 1988]      K. Murty. Linear Complementarity, Linear and Nonlinear
///                   Programming. Heldermann Verlag, 1988.
template <class T>
class MobyLCPSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobyLCPSolver)

  MobyLCPSolver();

  ~MobyLCPSolver() override = default;

  void SetLoggingEnabled(bool enabled);

  bool SolveLcpFast(const MatrixX<T>& M, const VectorX<T>& q,
                    VectorX<T>* z, T zero_tol = T(-1)) const;
  bool SolveLcpFastRegularized(const MatrixX<T>& M,
                               const VectorX<T>& q, VectorX<T>* z,
                               int min_exp = -20, unsigned step_exp = 4,
                               int max_exp = 20, T zero_tol = T(-1)) const;
  bool SolveLcpLemke(const MatrixX<T>& M, const VectorX<T>& q,
                     VectorX<T>* z, T piv_tol = T(-1),
                     T zero_tol = T(-1)) const;
  bool SolveLcpLemkeRegularized(const MatrixX<T>& M,
                                const VectorX<T>& q, VectorX<T>* z,
                                int min_exp = -20, unsigned step_exp = 1,
                                int max_exp = 1, T piv_tol = T(-1),
                                T zero_tol = T(-1)) const;
  bool SolveLcpLemke(const Eigen::SparseMatrix<double>& M,
                     const Eigen::VectorXd& q, Eigen::VectorXd* z,
                     double piv_tol = -1.0, double zero_tol = -1.0) const;
  bool SolveLcpLemkeRegularized(const Eigen::SparseMatrix<double>& M,
                                const Eigen::VectorXd& q, Eigen::VectorXd* z,
                                int min_exp = -20, unsigned step_exp = 4,
                                int max_exp = 20, double piv_tol = -1.0,
                                double zero_tol = -1.0) const;

  bool available() const override { return true; }

  SolutionResult Solve(MathematicalProgram& prog) const override;

  /// Returns the number of pivoting operations made by the last LCP solve.
  int get_num_pivots() const { return pivots_; }

 private:
  void ClearIndexVectors() const;

  template <typename Scalar>
  bool CheckLemkeTrivial(int n, const Scalar& zero_tol,
                         const VectorX<Scalar>& q,
                         VectorX<Scalar>* z) const;

  template <typename MatrixType, typename Scalar>
  void FinishLemkeSolution(const MatrixType& M, const VectorX<Scalar>& q,
                           const VectorX<Scalar>& x, VectorX<Scalar>* z) const;

  // TODO(sammy-tri) replace this with a proper logging hookup
  std::ostream& Log() const;
  bool log_enabled_;
  mutable std::ofstream null_stream_;

  // Records the number of pivoting operations used during the last solve.
  mutable unsigned pivots_;

  // NOTE:  The temporaries below are stored in the class to minimize
  // allocations; all are marked 'mutable' as they do not affect the
  // semantic const'ness of the class under its methods.

  // temporaries for regularized solver
  mutable MatrixX<T> MM_;
  mutable VectorX<T> wx_;

  // temporaries for fast pivoting solver
  mutable VectorX<T> z_, w_, qbas_;
  mutable MatrixX<T> Msub_, Mmix_;

  // temporaries for Lemke solver
  mutable VectorX<T> d_, Be_, u_, z0_, x_, dl_, xj_, dj_, wl_, result_;
  mutable MatrixX<T> Bl_, t1_, t2_;

  // Vectors which correspond to indices into other data.
  mutable std::vector<unsigned> all_, tlist_, bas_, nonbas_, j_;

  // temporary for sparse Lemke solver
  mutable Eigen::SparseMatrix<double> sBl_;
  mutable Eigen::SparseMatrix<double> MMs_, MMx_, eye_, diag_lambda_;
};

}  // end namespace solvers
}  // end namespace drake
