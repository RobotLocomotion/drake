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
