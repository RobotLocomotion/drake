#include <unsupported/Eigen/MatrixFunctions>

#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/controllers/zmp_planner.h"

namespace drake {
namespace systems {

Eigen::Vector2d ZMPPlanner::ComputeOptimalCoMdd(
    double time, const Eigen::Vector4d& x) const {
  // Eq. 20 in [1].
  Eigen::Vector2d yf = zmp_d_.value(zmp_d_.getEndTime());
  Eigen::Vector4d x_bar;
  x_bar << com_.value(time) - yf, comd_.value(time);
  return K_ * x_bar + k2_.value(time);
}

void ZMPPlanner::Plan(const PiecewisePolynomial<double>& zmp_d,
                      const Eigen::Vector4d& x0, double height,
                      const Eigen::Matrix2d& Qy, const Eigen::Matrix2d& R) {
  int n_segments = zmp_d.getNumberOfSegments();
  int zmp_d_poly_order = zmp_d.getSegmentPolynomialDegree(0);
  DRAKE_DEMAND(zmp_d_poly_order <= 3);
  DRAKE_DEMAND(zmp_d.rows() == 2 && zmp_d.cols() == 1);

  zmp_d_ = zmp_d;
  zmpd_d_ = zmp_d.derivative();

  Qy_ = Qy;
  R_ = R;

  // Eq. 1 and 2 in [1].
  A_.setZero();
  A_.block<2, 2>(0, 2).setIdentity();
  B_.setZero();
  B_.block<2, 2>(2, 0).setIdentity();
  C_.setZero();
  C_.block<2, 2>(0, 0).setIdentity();
  D_ = -height / 9.81 * Eigen::Matrix2d::Identity();

  // Eq. 9 - 14 in [1].
  Eigen::Matrix<double, 4, 4> Q1 = C_.transpose() * Qy_ * C_;
  Eigen::Matrix<double, 2, 2> R1 = R_ + D_.transpose() * Qy_ * D_;
  Eigen::Matrix<double, 4, 2> N = C_.transpose() * Qy_ * D_;
  Eigen::Matrix<double, 2, 2> R1i = R1.inverse();

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

  // Last desired ZMP.
  Eigen::Vector2d zmp_tf = zmp_d.value(zmp_d.getEndTime());
  Eigen::Vector4d tmp4;

  Eigen::MatrixXd alpha(4, n_segments);
  // Col is number of coefficients. Row is state degree.
  std::vector<Eigen::Matrix<double, 4, 4>> beta(n_segments);
  std::vector<Eigen::Matrix<double, 2, 4>> gamma(n_segments);
  std::vector<Eigen::Matrix<double, 2, 4>> c(n_segments);
  alpha.setZero();

  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>>
      beta_poly(n_segments);
  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>>
      gamma_poly(n_segments);

  // Algorithm 1 in [1] to solve for parameters of s2 and k2.
  for (int t = n_segments - 1; t >= 0; t--) {
    c[t].setZero();
    c[t].row(0).head(zmp_d_poly_order + 1) =
        zmp_d.getPolynomial(t, 0, 0).GetCoefficients();
    c[t].row(1).head(zmp_d_poly_order + 1) =
        zmp_d.getPolynomial(t, 1, 0).GetCoefficients();
    /// switch to zbar coord
    c[t].col(0) -= zmp_tf;

    // degree 4
    beta[t].col(3) = -A2i * B2 * c[t].col(3);
    gamma[t].col(3) = R1i * D_ * Qy_ * c[t].col(3) -
                      0.5 * R1i * B_.transpose() * beta[t].col(3);

    for (int d = 2; d >= 0; d--) {
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
    tmp4 = tmp4 - beta[t] * Eigen::Vector4d(1, dt, dt * dt, dt * dt * dt);

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
  Az.block<4, 4>(4, 0) = Eigen::Matrix<double, 4, 4>::Zero();
  Az.block<4, 4>(4, 4) = A2;
  Azi = Az.inverse();
  Bz.block<4, 2>(0, 0) = B_ * R1i * D_ * Qy_;
  Bz.block<4, 2>(4, 0) = B2;

  Eigen::MatrixXd a(8, n_segments);
  a.bottomRows(4) = alpha;
  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>>
      b_poly(n_segments);

  std::vector<Eigen::Matrix<double, 4, 4>> b(n_segments);
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
    b[t].col(3) = -Azi.topRows(4) * Bz * c[t].col(3);
    for (int d = 2; d >= 0; d--) {
      tmp81.head<4>() = b[t].col(d + 1);
      tmp81.tail<4>() = beta[t].col(d + 1);
      tmp81 = tmp81 * (d + 1);
      b[t].col(d) = Azi.topRows(4) * (tmp81 - Bz * c[t].col(d));
    }

    a.block<4, 1>(0, t) = x - b[t].col(0);

    Az_exp = Az * dt;
    Az_exp = Az_exp.exp();
    x = I48 * Az_exp * a.col(t) +
        b[t] * Eigen::Vector4d(1, dt, dt * dt, dt * dt * dt);

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
}

}  // namespace systems
}  // namespace drake
