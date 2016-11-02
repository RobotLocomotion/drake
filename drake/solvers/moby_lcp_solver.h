// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#pragma once

#include <fstream>
#include <vector>

#include <Eigen/SparseCore>

#include "drake/common/drake_export.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

class DRAKE_EXPORT MobyLCPSolver
    : public MathematicalProgramSolverInterface {
 public:
  MobyLCPSolver();
  virtual ~MobyLCPSolver() {}
  void SetLoggingEnabled(bool enabled);

  bool SolveLcpFast(const Eigen::MatrixXd& M, const Eigen::VectorXd& q,
                    Eigen::VectorXd* z, double zero_tol = -1.0) const;
  bool SolveLcpFastRegularized(const Eigen::MatrixXd& M,
                               const Eigen::VectorXd& q, Eigen::VectorXd* z,
                               int min_exp = -20, unsigned step_exp = 4,
                               int max_exp = 20, double zero_tol = -1.0) const;
  bool SolveLcpLemke(const Eigen::MatrixXd& M, const Eigen::VectorXd& q,
                     Eigen::VectorXd* z, double piv_tol = -1.0,
                     double zero_tol = -1.0) const;
  bool SolveLcpLemkeRegularized(const Eigen::MatrixXd& M,
                                const Eigen::VectorXd& q, Eigen::VectorXd* z,
                                int min_exp = -20, unsigned step_exp = 1,
                                int max_exp = 1, double piv_tol = -1.0,
                                double zero_tol = -1.0) const;
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

 private:
  void ClearIndexVectors() const;
  bool CheckLemkeTrivial(int n, double zero_tol, const Eigen::VectorXd& q,
                         Eigen::VectorXd* z) const;
  template <typename MatrixType>
  void FinishLemkeSolution(const MatrixType& M, const Eigen::VectorXd& q,
                           Eigen::VectorXd* z) const;

  // TODO(sammy-tri) replace this with a proper logging hookup
  std::ostream& Log() const;
  bool log_enabled_;
  mutable std::ofstream null_stream_;

  // TODO(sammy-tri) why is this a member variable?
  mutable unsigned pivots_;

  // NOTE:  The temporaries below are stored in the class to minimize
  // allocations; all are marked 'mutable' as they do not affect the
  // semantic const'ness of the class under its methods.

  // temporaries for regularized solver
  mutable Eigen::MatrixXd MM_;
  mutable Eigen::VectorXd wx_;

  // temporaries for fast pivoting solver
  mutable Eigen::VectorXd z_, w_, qbas_;
  mutable Eigen::MatrixXd Msub_, Mmix_;

  // temporaries for Lemke solver
  mutable Eigen::VectorXd d_, Be_, u_, z0_, x_, dl_, xj_, dj_, wl_, result_;
  mutable Eigen::MatrixXd Bl_, Al_, t1_, t2_;

  // Vectors which correspond to indices into other data.
  mutable std::vector<unsigned> all_, tlist_, bas_, nonbas_, j_;

  // temporary for sparse Lemke solver
  mutable Eigen::SparseMatrix<double> sBl_;
  mutable Eigen::SparseMatrix<double> MMs_, MMx_, eye_, diag_lambda_;
};

}  // end namespace solvers
}  // end namespace drake
