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
class MpcComponentUnitTests;
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
class MpcResidual {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MpcResidual)
  /**
   * Allocates memory for the residual.
   *
   * @param[in] N  horizon length
   * @param[in] nx number of states
   * @param[in] nu number of control input
   * @param[in] nc number of constraints per stage
   *
   * Throws a runtime_error if any of the inputs
   * are non-positive.
   */
  MpcResidual(int N, int nx, int nu, int nc);

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
   * Sets *this <- -1* *this.
   */
  void Negate();

  /**
   * @return Euclidean norm of the residual.
   */
  double Norm() const;

  /**
   * @return 0.5*Norm()^2
   */
  double Merit() const;

  /**
   * Computes R(x,xbar,sigma), the residual of a proximal subproblem
   * and stores the result internally.
   * R(x,xbar,sigma) = 0 if and only if x = P(xbar,sigma)
   * where P is the proximal operator.
   *
   * See (11) and (20) in https://arxiv.org/pdf/1901.04046.pdf
   * for a mathematical description.
   *
   * @param[in] x      Inner loop variable
   * @param[in] xbar   Outer loop variable
   * @param[in] sigma  Regularization strength > 0
   *
   * Throws a runtime_error if sigma isn't positive,
   * or if x and xbar aren't the same size.
   */
  void InnerResidual(const MpcVariable& x, const MpcVariable& xbar,
                     double sigma);

  /**
   * Computes π(x): the natural residual of the QP
   * at the primal-dual point x and stores the result internally.
   * See (17) in https://arxiv.org/pdf/1901.04046.pdf
   * for a mathematical definition.
   *
   * @param[in] x Evaluation point.
   */
  void NaturalResidual(const MpcVariable& x);

  /**
   * Computes the natural residual function augmented with
   * penalty terms, it is analogous to (18) in
   * https://arxiv.org/pdf/1901.04046.pdf,
   * and stores the result internally.
   *
   * @param[in] x Evaluation point.
   */
  void PenalizedNaturalResidual(const MpcVariable& x);

  /** Accessor for stationarity residual. */
  Eigen::VectorXd& z() { return z_; }
  /** Accessor for stationarity residual. */
  const Eigen::VectorXd& z() const { return z_; }

  /** Accessor for the equality residual. */
  Eigen::VectorXd& l() { return l_; }
  /** Accessor for the equality residual. */
  const Eigen::VectorXd& l() const { return l_; }

  /** Accessor for complementarity residual. */
  Eigen::VectorXd& v() { return v_; }
  /** Accessor for complementarity residual. */
  const Eigen::VectorXd& v() const { return v_; }

  /** Norm of the stationarity residual. */
  double z_norm() const { return znorm_; }
  /** Norm of the equality residual. */
  double l_norm() const { return lnorm_; }
  /** Norm of the complementarity residual. */
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
  double pfb(double a, double b, double alpha);

  friend class RiccatiLinearSolver;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
