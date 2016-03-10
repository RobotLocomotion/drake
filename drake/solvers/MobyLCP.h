// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#pragma once

#include <Eigen/SparseCore>

#include "MathematicalProgram.h"

namespace Drake {

class MobyLCPSolver : public MathematicalProgramSolverInterface {

  // TODO DEFECT ggould remove these when eigenification is complete.
  typedef Eigen::VectorXd VectorNd;
  typedef Eigen::MatrixXd MatrixNd;
  typedef Eigen::SparseMatrix<double> SparseMatrixNd;

 public:
  MobyLCPSolver();
  virtual ~MobyLCPSolver() {};

  // TODO ggould some subset of these lcp_* methods can be marked const.
  bool lcp_fast(
      const MatrixNd& M, const VectorNd& q, VectorNd* z,
      double zero_tol = -1.0);
  bool lcp_fast_regularized(
      const MatrixNd& M, const VectorNd& q, VectorNd* z,
      int min_exp = -20, unsigned step_exp = 4, int max_exp = 20,
      double piv_tol = -1.0, double zero_tol = -1.0);
  bool lcp_lemke(
      const MatrixNd& M, const VectorNd& q, VectorNd* z,
      double piv_tol = -1.0, double zero_tol = -1.0);
  bool lcp_lemke_regularized(
      const MatrixNd& M, const VectorNd& q, VectorNd* z,
      int min_exp = -20, unsigned step_exp = 1, int max_exp = 1,
      double piv_tol = -1.0, double zero_tol = -1.0);
  bool lcp_lemke(
      const SparseMatrixNd& M, const VectorNd& q, VectorNd* z,
      double piv_tol = -1.0, double zero_tol = -1.0);
  bool lcp_lemke_regularized(
      const SparseMatrixNd& M, const VectorNd& q, VectorNd* z,
      int min_exp = -20, unsigned step_exp = 4, int max_exp = 20,
      double piv_tol = -1.0, double zero_tol = -1.0);
  bool fast_pivoting(
      const MatrixNd& M, const VectorNd& q, VectorNd& z,
      double eps = std::sqrt(std::numeric_limits<double>::epsilon()));

  virtual bool available() const override { return true; }
  virtual bool solve(OptimizationProblem& prog) const override;

 private:
  unsigned pivots;
  static void log_failure(const MatrixNd& M, const VectorNd& q);
  static void set_basis(unsigned n, unsigned count,
                        std::vector<unsigned>& bas,
                        std::vector<unsigned>& nbas);

  // NOTE:  The temporaries below are stored in the class to minimize
  // allocations; all are marked 'mutable' as they do not affect the
  // semantic const'ness of the class under its methods.

  // temporaries for regularized solver
  mutable MatrixNd _MM;
  mutable VectorNd _wx;

  // temporaries for fast pivoting solver
  mutable VectorNd _z, _w, _qbas, _qprime;
  mutable MatrixNd _Msub, _Mmix, _M;

  // temporaries for Lemke solver
  mutable VectorNd _d, _Be, _u, _z0, _x, _dl, _xj, _dj, _wl, _result;
  mutable VectorNd _restart_z0;
  mutable MatrixNd _Bl, _Al, _t1, _t2;
  mutable std::vector<unsigned> _all, _tlist, _bas, _nonbas, _j, _max_elm;

  // temporary for sparse Lemke solver
  mutable SparseMatrixNd _sBl;
  mutable SparseMatrixNd _MMs, _MMx, _eye, _zero, _diag_lambda;

  // TODO defect sammy take this out
  // linear algebra
  // LinAlgd _LA;
};
};
