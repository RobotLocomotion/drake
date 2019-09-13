#pragma once

#include <stdexcept>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/dense_data.h"
#include "drake/solvers/fbstab/components/dense_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

/**
 * This class computes and stores residuals for inequality constrained dense
 * QPs. See dense_data.h for a description of the QP.
 *
 * Residuals have 2 components:
 * - z: Stationarity residual
 * - v: Complementarity residual
 */
class DenseResidual {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseResidual)
  /**
   * Allocates memory for computing and storing residual vectors.
   * Uses alpha = 0.95 (see (19) in https://arxiv.org/pdf/1901.04046.pdf)
   * by default.
   *
   * @param[in] nz Number of decision variables
   * @param[in] nv Number of inequality constraints
   *
   * Throws an exception if any inputs aren't positive.
   */
  DenseResidual(int nz, int nv);

  /**
   * Performs the operation
   * y <- -1*y (y is this object).
   */
  void Negate();
  // TODO(dliaomcp@umich.edu): Add is_negated_ field

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
   * Throws a runtime_error if problem data isn't linked, sigma isn't positive,
   * or if x and xbar aren't the same size.
   */
  void InnerResidual(const DenseVariable& x, const DenseVariable& xbar,
                     double sigma);

  /**
   * Computes π(x): the natural residual of the QP
   * at the primal-dual point x and stores the result internally.
   * See (17) in https://arxiv.org/pdf/1901.04046.pdf
   * for a mathematical definition.
   *
   * @param[in] x Evaluation point.
   *
   * Throws a runtime_error if problem data isn't linked.
   */
  void NaturalResidual(const DenseVariable& x);

  /**
   * Computes the natural residual function augmented with
   * penalty terms, it is analogous to (18) in
   * https://arxiv.org/pdf/1901.04046.pdf,
   * and stores the result internally.
   *
   * @param[in] x Evaluation point.
   *
   * Throws a runtime_error if problem data isn't linked.
   */
  void PenalizedNaturalResidual(const DenseVariable& x);

  /**
   * Fills the storage with a
   * i.e., r <- a*ones.
   * @param[in] a
   */
  void Fill(double a);

  /**
   * Computes the Euclidean norm of the current stored residuals.
   * @return sqrt(|z|^2 + |v|^2)
   */
  double Norm() const;

  /**
   * Computes the merit function of the current stored residuals.
   * @return 0.5*(|z|^2 + |v|^2)
   */
  double Merit() const;

  /** Accessor for stationarity residual. */
  Eigen::VectorXd& z() { return z_; }
  /** Accessor for stationarity residual. */
  const Eigen::VectorXd& z() const { return z_; }

  /** Accessor for complementarity residual. */
  Eigen::VectorXd& v() { return v_; }
  /** Accessor for complementarity residual. */
  const Eigen::VectorXd& v() const { return v_; }

  /**
   * Sets the alpha parameter defined in (19)
   * of https://arxiv.org/pdf/1901.04046.pdf.
   */
  void SetAlpha(double alpha) { alpha_ = alpha; }

  /** Norm of the stationarity residual. */
  double z_norm() const { return znorm_; }
  /** Norm of the complementarity residual. */
  double v_norm() const { return vnorm_; }

  /**
   * The dense QP we consider has no equality constraints
   * so this methods returns 0.
   * It's needed by the printing routines of the FBstabAlgorithm class.
   */
  double l_norm() const { return 0.0; }

 private:
  int nz_ = 0;         // number of decision variables
  int nv_ = 0;         // number of inequality constraints
  Eigen::VectorXd z_;  // storage for the stationarity residual
  Eigen::VectorXd v_;  // storage for the complementarity residual
  double alpha_ = 0.95;
  double znorm_ = 0.0;
  double vnorm_ = 0.0;

  /*
   * Evaluate the Penalized Fischer-Burmeister (PFB) function
   * (19) in https://arxiv.org/pdf/1901.04046.pdf
   */
  static double pfb(double a, double b, double alpha);

  /* Scalar max function. */
  static double max(double a, double b);  // NOLINT

  /* Scalar min function. */
  static double min(double a, double b);  // NOLINT

  friend class DenseLinearSolver;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
