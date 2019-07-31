#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

// Forward declaration of testing class to enable a friend declaration.
namespace test {
class MPCComponentUnitTests;
}  // namespace test

/**
 * This class computes and stores residuals for MPC QPs. See mpc_data.h
 * for the mathematical description.
 *
 * Residuals have 3 components:
 * - z: Stationarity residual
 * - l: Equality residual
 * - v: Inequality/complimentarity residual
 */
class MPCResidual {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MPCResidual);
  /**
   * Allocates memory for the residual.
   *
   * @param[in] N  horizon length
   * @param[in] nx number of states
   * @param[in] nu number of control input
   * @param[in] nc number of constraints per stage
   */
  MPCResidual(int N, int nx, int nu, int nc);

  /**
   * Sets the value of alpha used in residual computations,
   * see (19) in https://arxiv.org/pdf/1901.04046.pdf.
   * @param[in] alpha
   */
  void SetAlpha(double alpha) { alpha_ = alpha; }

  /**
   * Fills the storage with all a.
   * @param[in] a
   */
  void Fill(double a);

  /**
   * Sets y <- -1*y where y is *this.
   */
  void Negate();

  /**
   * @return Euclidean norm of the residual
   */
  double Norm() const;

  /**
   * Computes the merit function
   * @return 0.5*Norm()^2
   */
  double Merit() const;

  /**
   * Computes the proximal subproblem residual at the point x
   * relative to the reference point xbar with regularization strength sigma.
   * Corresponds to (20) in https://arxiv.org/pdf/1901.04046.pdf
   * Overwrites internal storage
   * @param[in] x     inner iterate for evaluating the residual
   * @param[in] xbar  outer iterate for evaluating the residual
   * @param[in] sigma regularization strength
   */
  void InnerResidual(const MPCVariable& x, const MPCVariable& xbar,
                     double sigma);

  /**
   * Computes the natural residual of the KKT conditions at x.
   * Overwrites internal storage.
   * Corresponds to (23) in https://arxiv.org/pdf/1901.04046.pdf
   * @param[in] x primal-dual point to evaluate the KKT conditions
   */
  void NaturalResidual(const MPCVariable& x);

  /**
   * Computes the penalized natural residual of the KKT conditions at x
   * Overwrites internal storage
   * @param[in] x primal-dual point to evaluate the KKT conditions
   */
  void PenalizedNaturalResidual(const MPCVariable& x);

  /** Accessor for z. */
  Eigen::VectorXd& z() { return z_; }
  /** Accessor for z. */
  const Eigen::VectorXd& z() const { return z_; }

  /** Accessor for l. */
  Eigen::VectorXd& l() { return l_; }
  /** Accessor for l. */
  const Eigen::VectorXd& l() const { return l_; }

  /** Accessor for v. */
  Eigen::VectorXd& v() { return v_; }
  /** Accessor for v. */
  const Eigen::VectorXd& v() const { return v_; }

  /** 2 Norm of stationarity residual. */
  double z_norm() const { return znorm_; }
  /** 2 Norm of equality residual. */
  double l_norm() const { return lnorm_; }
  /** 2 Norm of complimentarity residual. */
  double v_norm() const { return vnorm_; }

 private:
  Eigen::VectorXd z_;  // stationarity residual
  Eigen::VectorXd l_;  // equality residual
  Eigen::VectorXd v_;  // complimentarity residual

  int N_ = 0;   // horizon length
  int nx_ = 0;  // number of states
  int nu_ = 0;  // number of controls
  int nc_ = 0;  // constraints per stage
  int nz_ = 0;  // number of primal variables
  int nl_ = 0;  // number of equality duals
  int nv_ = 0;  // number of inequality duals

  double alpha_ = 0.95;

  double znorm_ = 0.0;  // cached norm of z_
  double lnorm_ = 0.0;  // cached norm of l_
  double vnorm_ = 0.0;  // cached norm of v_

  /*
   * Computes the penalized Fischer-Burmeister function pfb(a,b)
   * Equation (19) of https://arxiv.org/pdf/1901.04046.pdf.
   * @param[in]  a
   * @param[in]  b
   * @param[in]  alpha weighting parameter
   * @return     pfb(a,b)
   */
  static double pfb(double a, double b, double alpha);

  /* Scalar max operator. */
  static double max(double a, double b) { return a > b ? a : b; }
  /* Scalar min operator. */
  static double min(double a, double b) { return a < b ? a : b; }

  friend class RicattiLinearSolver;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
