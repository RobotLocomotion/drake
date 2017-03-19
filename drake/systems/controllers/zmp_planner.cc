#include "drake/systems/controllers/zmp_planner.h"

#include <vector>

#include <unsupported/Eigen/MatrixFunctions>

#include "drake/common/text_logging.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

namespace drake {
namespace systems {

Eigen::Vector2d ZMPPlanner::ComputeOptimalCoMdd(
    double time, const Eigen::Vector4d& x) const {
  DRAKE_DEMAND(planned_);
  // Eq. 20 in [1].
  Eigen::Vector2d yf = zmp_d_.value(zmp_d_.getEndTime());
  Eigen::Vector4d x_bar = x;
  x_bar.head<2>() -= yf;
  return K_ * x_bar + k2_.value(time);
}

bool ZMPPlanner::CheckStationaryEndPoint(
    const PiecewisePolynomial<double>& zmp_d) const {
  PiecewisePolynomial<double> last_segment =
      zmp_d.slice(zmp_d.getNumberOfSegments() - 1, 1);
  PiecewisePolynomial<double> derivative = last_segment.derivative();
  int degree = last_segment.getSegmentPolynomialDegree(0);
  for (int d = degree; d >= 0; d--) {
    if (derivative.value(derivative.getEndTime()).norm() >
        kStationaryThreshold) {
      return false;
    }
    derivative = derivative.derivative();
  }
  return true;
}

void ZMPPlanner::Plan(const PiecewisePolynomial<double>& zmp_d,
                      const Eigen::Vector4d& x0, double height, double gravity,
                      const Eigen::Matrix2d& Qy, const Eigen::Matrix2d& R) {
  // Warn the caller if the last point is not stationary. The math is still
  // correct, and this is an allowable (but dangerous) use case.
  // If the user use the policy / nominal trajectory past the end point, the
  // system diverges exponentially fast.
  if (!CheckStationaryEndPoint(zmp_d)) {
    drake::log()->warn("ZMPPlanner: The desired zmp trajectory does not end "
        "in a stationary condition.");
  }

  int n_segments = zmp_d.getNumberOfSegments();
  int zmp_d_degree = zmp_d.getSegmentPolynomialDegree(0);
  DRAKE_DEMAND(zmp_d_degree >= 0);
  DRAKE_DEMAND(zmp_d.rows() == 2 && zmp_d.cols() == 1);
  DRAKE_DEMAND(height > 0);
  DRAKE_DEMAND(gravity > 0);

  zmp_d_ = zmp_d;

  Qy_ = Qy;
  R_ = R;

  // Eq. 1 and 2 in [1].
  A_.setZero();
  A_.block<2, 2>(0, 2).setIdentity();
  B_.setZero();
  B_.block<2, 2>(2, 0).setIdentity();
  C_.setZero();
  C_.block<2, 2>(0, 0).setIdentity();
  D_ = -height / gravity * Eigen::Matrix2d::Identity();

  // Eq. 9 - 14 in [1].
  Eigen::Matrix<double, 4, 4> Q1 = C_.transpose() * Qy_ * C_;
  Eigen::Matrix<double, 2, 2> R1 = R_ + D_.transpose() * Qy_ * D_;
  Eigen::Matrix<double, 4, 2> N = C_.transpose() * Qy_ * D_;
  Eigen::Matrix<double, 2, 2> R1i = R1.inverse();
  R1i_ = R1i;

  LinearQuadraticRegulatorResult lqr_result =
      LinearQuadraticRegulator(A_, B_, Q1, R1, N);
  S1_ = lqr_result.S;
  K_ = -lqr_result.K;

  // Computes the time varying linear and constant term in the value function
  // and linear policy. Also known as the backward pass.
  Eigen::Matrix<double, 2, 4> NB = (N.transpose() + B_.transpose() * S1_);
  // Eq. 23, 24 in [1].
  Eigen::Matrix<double, 4, 4> A2 =
      NB.transpose() * R1i * B_.transpose() - A_.transpose();
  Eigen::Matrix<double, 4, 2> B2 =
      2 * (C_.transpose() - NB.transpose() * R1i * D_) * Qy_;
  Eigen::Matrix<double, 4, 4> A2i = A2.inverse();

  NB_ = NB;
  A2_ = A2;
  B2_ = B2;

  // Last desired ZMP.
  Eigen::Vector2d zmp_tf = zmp_d.value(zmp_d.getEndTime());
  Eigen::Vector4d tmp4;

  Eigen::MatrixXd alpha = Eigen::MatrixXd::Zero(4, n_segments);
  std::vector<Eigen::MatrixXd> beta(n_segments,
                                    Eigen::MatrixXd::Zero(4, zmp_d_degree + 1));
  std::vector<Eigen::MatrixXd> gamma(
      n_segments, Eigen::MatrixXd::Zero(2, zmp_d_degree + 1));
  std::vector<Eigen::MatrixXd> c(n_segments,
                                 Eigen::MatrixXd::Zero(2, zmp_d_degree + 1));

  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>>
      beta_poly(n_segments);
  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>>
      gamma_poly(n_segments);

  Eigen::VectorXd delta_time_vec(zmp_d_degree + 1);
  delta_time_vec[0] = 1;

  // Algorithm 1 in [1] to solve for parameters of s2 and k2.
  for (int t = n_segments - 1; t >= 0; t--) {
    c[t].setZero();
    c[t].row(0).head(zmp_d_degree + 1) =
        zmp_d.getPolynomial(t, 0, 0).GetCoefficients();
    c[t].row(1).head(zmp_d_degree + 1) =
        zmp_d.getPolynomial(t, 1, 0).GetCoefficients();
    /// switch to zbar coord
    c[t].col(0) -= zmp_tf;

    // degree 4
    beta[t].col(zmp_d_degree) = -A2i * B2 * c[t].col(zmp_d_degree);
    gamma[t].col(zmp_d_degree) =
        R1i * D_ * Qy_ * c[t].col(zmp_d_degree) -
        0.5 * R1i * B_.transpose() * beta[t].col(zmp_d_degree);

    for (int d = zmp_d_degree - 1; d >= 0; d--) {
      beta[t].col(d) = A2i * ((d + 1) * beta[t].col(d + 1) - B2 * c[t].col(d));
      gamma[t].col(d) = R1i * D_ * Qy_ * c[t].col(d) -
                        0.5 * R1i * B_.transpose() * beta[t].col(d);
    }

    if (t == n_segments - 1) {
      tmp4 = Eigen::Vector4d::Zero();
    } else {
      tmp4 = alpha.col(t + 1) + beta[t + 1].col(0);
    }

    double dt = zmp_d.getDuration(t);
    Eigen::Matrix4d A2exp = A2 * dt;
    A2exp = A2exp.exp();
    for (int i = 0; i < zmp_d_degree + 1; i++)
      delta_time_vec[i] = std::pow(dt, i);
    tmp4 = tmp4 - beta[t] * delta_time_vec;

    alpha.col(t) = A2exp.inverse() * tmp4;

    beta_poly[t].resize(4, 1);
    for (int n = 0; n < 4; n++) {
      beta_poly[t](n, 0) = Polynomial<double>(beta[t].row(n));
    }

    gamma_poly[t].resize(2, 1);
    for (int n = 0; n < 2; n++) {
      gamma_poly[t](n, 0) = Polynomial<double>(gamma[t].row(n));
    }
  }

  // Eq. 25 in [1].
  PiecewisePolynomial<double> beta_traj(beta_poly, zmp_d.getSegmentTimes());
  s2_ = ExponentialPlusPiecewisePolynomial<double>(Eigen::Matrix4d::Identity(),
                                                   A2, alpha, beta_traj);

  // Eq. 28 in [1].
  PiecewisePolynomial<double> gamma_traj(gamma_poly, zmp_d.getSegmentTimes());
  k2_ = ExponentialPlusPiecewisePolynomial<double>(-0.5 * R1i * B_.transpose(),
                                                   A2, alpha, gamma_traj);

  // Computes the nominal CoM trajectory. Also known as the forward pass.
  // Eq. 35, 36 in [1].
  Eigen::Matrix<double, 8, 8> Az, Azi;
  Eigen::Matrix<double, 8, 2> Bz;
  Az.block<4, 4>(0, 0) = A_ + B_ * K_;
  Az.block<4, 4>(0, 4) = -0.5 * B_ * R1i * B_.transpose();
  Az.block<4, 4>(4, 0).setZero();
  Az.block<4, 4>(4, 4) = A2;
  Azi = Az.inverse();
  Bz.block<4, 2>(0, 0) = B_ * R1i * D_ * Qy_;
  Bz.block<4, 2>(4, 0) = B2;

  Eigen::MatrixXd a(8, n_segments);
  a.bottomRows<4>() = alpha;
  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>>
      b_poly(n_segments);

  std::vector<Eigen::MatrixXd> b(n_segments,
                                 Eigen::MatrixXd(4, zmp_d_degree + 1));
  Eigen::Matrix<double, 8, 1> tmp81;
  Eigen::Matrix<double, 8, 8> Az_exp;
  Eigen::Matrix<double, 4, 8> I48;
  I48.block<4, 4>(0, 0).setIdentity();
  I48.block<4, 4>(0, 4).setZero();

  Eigen::Vector4d x = x0;
  x.head<2>() -= zmp_tf;

  // Algorithm 2 in [1] to solve for the CoM trajectory.
  // Since s2 is already solved above, we only compute the top half for the
  // CoM trajectory.
  for (int t = 0; t < n_segments; t++) {
    double dt = zmp_d.getDuration(t);
    b[t].col(zmp_d_degree) = -Azi.topRows(4) * Bz * c[t].col(zmp_d_degree);
    for (int d = zmp_d_degree - 1; d >= 0; d--) {
      tmp81.head<4>() = b[t].col(d + 1);
      tmp81.tail<4>() = beta[t].col(d + 1);
      tmp81 = tmp81 * (d + 1);
      b[t].col(d) = Azi.topRows(4) * (tmp81 - Bz * c[t].col(d));
    }

    a.block<4, 1>(0, t) = x - b[t].col(0);

    Az_exp = Az * dt;
    Az_exp = Az_exp.exp();
    for (int i = 0; i < zmp_d_degree + 1; i++)
      delta_time_vec[i] = std::pow(dt, i);
    x = I48 * Az_exp * a.col(t) + b[t] * delta_time_vec;

    b[t].block<2, 1>(0, 0) += zmp_tf;  // Map CoM position back to world frame.

    b_poly[t].resize(2, 1);
    for (int n = 0; n < 2; n++) {
      b_poly[t](n, 0) = Polynomial<double>(b[t].row(n));
    }
  }

  Eigen::Matrix<double, 2, 8> tmp28;
  tmp28.block<2, 2>(0, 0).setIdentity();
  tmp28.block<2, 6>(0, 2).setZero();
  PiecewisePolynomial<double> b_traj(b_poly, zmp_d.getSegmentTimes());

  com_ = ExponentialPlusPiecewisePolynomial<double>(tmp28, Az, a, b_traj);
  comd_ = com_.derivative();
  // ComputeOptimalCoMdd(t, nominal_x) should equal to comdd_.
  comdd_ = comd_.derivative();

  planned_ = true;
}

}  // namespace systems
}  // namespace drake
