// Adapted with permission from code by Evan Drumwright
// (https://github.com/edrumwri).

#pragma once

#include <Eigen/SparseCore>

namespace Drake {

class LCP
{

  // TODO DEFECT ggould remove these when eigenification is complete.
  typedef Eigen::VectorXd VectorNd;
  typedef Eigen::MatrixXd MatrixNd;
  typedef Eigen::SparseMatrix<double> SparseMatrixNd;

 public:
  LCP();

  bool lcp_lemke_regularized(
      const MatrixNd& M, const VectorNd& q, VectorNd& z,
      int min_exp = -20, unsigned step_exp = 1, int max_exp = 1,
      double piv_tol = -1.0, double zero_tol = -1.0);
    bool lcp_lemke_regularized(
        const SparseMatrixNd& M, const VectorNd& q, VectorNd& z,
        int min_exp = -20, unsigned step_exp = 4, int max_exp = 20,
        double piv_tol = -1.0, double zero_tol = -1.0);
    bool lcp_lemke(
        const MatrixNd& M, const VectorNd& q, VectorNd& z,
        double piv_tol = -1.0, double zero_tol = -1.0);
    bool lcp_lemke(
        const SparseMatrixNd& M, const VectorNd& q, VectorNd& z,
        double piv_tol = -1.0, double zero_tol = -1.0);
    bool lcp_fast(
        const MatrixNd& M, const VectorNd& q, VectorNd& z,
        double zero_tol = -1.0);
    bool lcp_fast_regularized(
        const MatrixNd& M, const VectorNd& q, VectorNd& z,
        int min_exp = -20, unsigned step_exp = 4, int max_exp = 20,
        double piv_tol = -1.0, double zero_tol = -1.0);
    bool fast_pivoting(
        const MatrixNd& M, const VectorNd& q, VectorNd& z,
        double eps = std::sqrt(std::numeric_limits<double>::epsilon()));

 private:
  unsigned pivots;
  static void log_failure(const MatrixNd& M, const VectorNd& q);
  static void set_basis(unsigned n, unsigned count,
                        std::vector<unsigned>& bas,
                        std::vector<unsigned>& nbas);
  static unsigned rand_min(const VectorNd& v, double zero_tol);

  // temporaries for regularized solver
  MatrixNd _MM;
  VectorNd _wx;

  // temporaries for fast pivoting solver
  VectorNd _z, _w, _qbas, _qprime;
  MatrixNd _Msub, _Mmix, _M;

  // temporaries for Lemke solver
  VectorNd _d, _Be, _u, _z0, _x, _dl, _xj, _dj, _wl, _result;
  VectorNd _restart_z0;
  MatrixNd _Bl, _Al, _t1, _t2;
  std::vector<unsigned> _all, _tlist, _bas, _nonbas, _j, _max_elm;

  // temporary for sparse Lemke solver
  SparseMatrixNd _sBl;
  SparseMatrixNd _MMs, _MMx, _eye, _zero, _diag_lambda;

  // linear algebra
  LinAlgd _LA;
};
};
