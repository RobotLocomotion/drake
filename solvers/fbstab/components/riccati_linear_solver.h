#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_residual.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

// Forward declaration of testing class to enable a friend declaration.
namespace test {
class MpcComponentUnitTests;
}  // namespace test

/**
 * Implements a Riccati recursion based method for solving linear systems of
 * equations that arise when solving MPC form QPs (see mpc_data.h) using FBstab.
 * The equations are of the form
 *
 *     [Hs  G' A'][dz] = [rz]
 *     [-G  sI 0 ][dl] = [rl]
 *     [-CA 0  D ][dv] = [rv]
 *
 * where s = sigma, C = diag(gamma), D = diag(mu + sigma*gamma).
 * The vectors gamma and mu are defined in (24) of
 * https://arxiv.org/pdf/1901.04046.pdf.
 *
 * In compact form:
 *
 *     V(x,xbar,sigma)*dx = r.
 *
 * The Riccati recursion used by this class is based on the one in:
 *
 * Rao, Christopher V., Stephen J. Wright, and James B. Rawlings.
 * "Application of interior-point methods to model predictive control."
 * Journal of optimization theory and applications 99.3 (1998): 723-757.
 *
 * and is used to perform the factorization efficiently. This class also
 * contains workspace memory and methods for setting up and solving the linear
 * systems.
 *
 * There is an error on page 744 of the paper in the \Delta x_k equation, It's
 * missing a residual term. This class contains mutable members as is thus not
 * thread safe.
 */
class RiccatiLinearSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RiccatiLinearSolver);
  /**
   * Allocates workspace memory.
   *
   * @param[in] N  horizon length
   * @param[in] nx number of states
   * @param[in] nu number of control input
   * @param[in] nc number of constraints per stage
   *
   * Throws an exception if any of the inputs are non-positive.
   */
  RiccatiLinearSolver(int N, int nx, int nu, int nc);

  /**
   * Sets a parameter used in the algorithm, see (19)
   * in https://arxiv.org/pdf/1901.04046.pdf.
   * @param[in] alpha
   */
  void SetAlpha(double alpha) { alpha_ = alpha; }

  /**
   * Computes then factors the matrix V(x,xbar,sigma) using a Riccati
   * recursion.
   *
   * The matrix V is computed as described in
   * Algorithm 4 of https://arxiv.org/pdf/1901.04046.pdf.
   *
   * @param[in]  x       Inner loop iterate
   * @param[in]  xbar    Outer loop iterate
   * @param[in]  sigma   Regularization strength
   * @return             true if factorization succeeds false otherwise.
   *
   * Throws an exception if x and xbar aren't the correct size,
   * sigma is negative or the problem data isn't linked.
   */
  bool Initialize(const MpcVariable& x, const MpcVariable& xbar, double sigma);

  /**
   * Solves the system V*x = r and stores the result in x.
   * This method assumes that the Factor routine was run to
   * compute then factor the matrix V.
   *
   * @param[in]   r   The right hand side vector
   * @param[out]  x   Overwritten with the solution
   * @return        true if the solve succeeds, false otherwise
   *
   * Throws an exception if x and r aren't the correct sizes,
   * if x is null or if the problem data isn't linked.
   */
  bool Solve(const MpcResidual& r, MpcVariable* dx) const;

 private:
  // Workspace matrices.
  std::vector<Eigen::MatrixXd> Q_;
  std::vector<Eigen::MatrixXd> R_;
  std::vector<Eigen::MatrixXd> S_;

  // Storage for the matrix portion of the recursion.
  std::vector<Eigen::MatrixXd> P_;
  std::vector<Eigen::MatrixXd> SG_;
  std::vector<Eigen::MatrixXd> M_;
  std::vector<Eigen::MatrixXd> L_;
  std::vector<Eigen::MatrixXd> SM_;
  std::vector<Eigen::MatrixXd> AM_;

  // Storage for the vector portion of the recursion.
  mutable std::vector<Eigen::VectorXd> h_;
  mutable std::vector<Eigen::VectorXd> th_;

  // Workspace.
  Eigen::VectorXd gamma_;
  Eigen::VectorXd mus_;
  Eigen::MatrixXd Gamma_;

  // Workspace matrices.
  Eigen::MatrixXd Etemp_;
  Eigen::MatrixXd Ltemp_;
  Eigen::MatrixXd Linv_;

  // Workspace vectors.
  mutable Eigen::VectorXd tx_;
  mutable Eigen::VectorXd tl_;
  mutable Eigen::VectorXd tu_;
  mutable Eigen::VectorXd r1_;
  mutable Eigen::VectorXd r2_;
  mutable Eigen::VectorXd r3_;

  int N_ = 0;   // horizon length
  int nx_ = 0;  // number of states
  int nu_ = 0;  // number of controls
  int nc_ = 0;  // constraints per stage
  int nz_ = 0;  // number of primal variables
  int nl_ = 0;  // number of equality duals
  int nv_ = 0;  // number of inequality duals

  const MpcData* data_ = nullptr;
  const double zero_tolerance_ = 1e-13;
  double alpha_ = 0.95;

  // Computes the gradient of the penalized fischer-burmeister (PFB)
  // function, (19) in https://arxiv.org/pdf/1901.04046.pdf.
  Eigen::Vector2d PFBGradient(double a, double b) const;

  friend class test::MpcComponentUnitTests;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
