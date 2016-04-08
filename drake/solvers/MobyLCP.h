// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#ifndef DRAKE_SOLVERS_MOBYLCP_H_
#define DRAKE_SOLVERS_MOBYLCP_H_

#include <fstream>
#include <Eigen/SparseCore>

#include "drake/drakeOptimization_export.h"

#include "MathematicalProgram.h"

namespace Drake {

class DRAKEOPTIMIZATION_EXPORT MobyLCPSolver
    : public MathematicalProgramSolverInterface {
 public:
  MobyLCPSolver();
  virtual ~MobyLCPSolver() {}
  void SetLoggingEnabled(bool);

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

  virtual bool available() const override { return true; }
  virtual bool Solve(OptimizationProblem& prog) const override;

 private:
  void ClearIndexVectors() const;
  bool CheckLemkeTrivial(int n, double zero_tol, const Eigen::VectorXd& q,
                         Eigen::VectorXd* z) const;
  template <typename MatrixType>
  void FinishLemkeSolution(const MatrixType& M, const Eigen::VectorXd& q,
                           Eigen::VectorXd* z) const;

  // TODO(sammy-tri) replace this with a proper logging hookup
  std::ostream& LOG() const;
  bool log_enabled_;
  mutable std::ofstream null_stream_;

  // TODO(sammy-tri) why is this a member variable?
  mutable unsigned pivots;

  // NOTE:  The temporaries below are stored in the class to minimize
  // allocations; all are marked 'mutable' as they do not affect the
  // semantic const'ness of the class under its methods.

  // temporaries for regularized solver
  mutable Eigen::MatrixXd _MM;
  mutable Eigen::VectorXd _wx;

  // temporaries for fast pivoting solver
  mutable Eigen::VectorXd _z, _w, _qbas, _qprime;
  mutable Eigen::MatrixXd _Msub, _Mmix, _M;

  // temporaries for Lemke solver
  mutable Eigen::VectorXd _d, _Be, _u, _z0, _x, _dl, _xj, _dj, _wl, _result;
  mutable Eigen::MatrixXd _Bl, _Al, _t1, _t2;

  // Vectors which correspond to indices into other data.
  mutable std::vector<unsigned> _all, _tlist, _bas, _nonbas, _j;

  // temporary for sparse Lemke solver
  mutable Eigen::SparseMatrix<double> _sBl;
  mutable Eigen::SparseMatrix<double> _MMs, _MMx, _eye, _zero, _diag_lambda;
};

}  // end namespace Drake

#endif  // DRAKE_SOLVERS_MOBYLCP_H_
