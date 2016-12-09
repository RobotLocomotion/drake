#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <fstream>

#include "drake/systems/controllers/zmp_planner.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

namespace drake {
namespace systems {

void ZMPPlanner::Plan(const PiecewisePolynomial<double> &zmp_d, const Eigen::Vector4d &x0, double height) {
  int zmp_d_poly_order = zmp_d.getSegmentPolynomialDegree(0) + 1;
  DRAKE_DEMAND(zmp_d_poly_order <= 4);

  zmp_traj_ = zmp_d;
  zmpd_traj_ = zmp_traj_.derivative();

  A_.setZero();
  A_.block<2, 2>(0, 2).setIdentity();
  B_.setZero();
  B_.block<2, 2>(2, 0).setIdentity();
  C_.setZero();
  C_.block<2, 2>(0, 0).setIdentity();
  D_ = -height / 9.81 * Eigen::Matrix2d::Identity();

  // TODO: take params
  Qy_ = Eigen::Matrix2d::Identity();
  R_.setZero();

  Eigen::Matrix<double, 4, 4> Q1 = C_.transpose() * Qy_ * C_;
  Eigen::Matrix<double, 2, 2> R1 = R_ + D_.transpose() * Qy_ * D_;
  Eigen::Matrix<double, 4, 2> N = C_.transpose() * Qy_ * D_;
  Eigen::Matrix<double, 2, 2> R1i = R1.inverse();

  LinearQuadraticRegulatorResult lqr_result = LinearQuadraticRegulator(A_, B_, Q1, R1, N);
  S_ = lqr_result.S;
  K_ = -lqr_result.K;

  s1_dot_.setZero();
  u0_.setZero();

  // generate s1 traj
  Eigen::Matrix<double, 2, 4> NB = (N.transpose() + B_.transpose() * S_);
  Eigen::Matrix<double, 4, 4> A2 = NB.transpose() * R1i * B_.transpose() - A_.transpose();
  Eigen::Matrix<double, 4, 2> B2 = 2 * (C_.transpose() - NB.transpose() * R1i * D_) * Qy_;
  Eigen::Matrix<double, 4, 4> A2i = A2.inverse();

  int n_segments = zmp_d.getNumberOfSegments();
  Eigen::Vector2d zmp_tf;
  Eigen::Vector4d s1dt;

  zmp_tf = zmp_d.value(zmp_d.getEndTime());

  Eigen::MatrixXd alpha(4, n_segments);
  std::vector<Eigen::Matrix<double, 4, 4>> beta(n_segments);
  std::vector<Eigen::Matrix<double, 2, 4>> gamma(n_segments);
  std::vector<Eigen::Matrix<double, 2, 4>> c(n_segments);
  alpha.setZero();

  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>> beta_poly(n_segments);

  for (int t = n_segments-1; t >= 0; t--) {
    c[t].setZero();
    c[t].row(0).head(zmp_d_poly_order) = zmp_d.getPolynomial(t, 0, 0).GetCoefficients();
    c[t].row(1).head(zmp_d_poly_order) = zmp_d.getPolynomial(t, 1, 0).GetCoefficients();
    /// switch to zbar coord
    c[t].col(0) -= zmp_tf;

    // degree 4
    beta[t].col(3) = -A2i * B2 * c[t].col(3);
    gamma[t].col(3) = R1i * D_ * Qy_ * c[t].col(3) - 0.5 * R1i * B_.transpose() * beta[t].col(3);

    for (int d = 2; d >= 0; d--) {
      beta[t].col(d) = A2i * ((d+1) * beta[t].col(d+1) - B2 * c[t].col(d));
      gamma[t].col(d) = R1i * D_ * Qy_ * c[t].col(d) - 0.5 * R1i * B_.transpose() * beta[t].col(d);
    }

    if (t == n_segments-1) {
      s1dt = Eigen::Vector4d::Zero();
    }
    else {
      s1dt = alpha.col(t+1) + beta[t+1].col(0);
    }

    double dt = zmp_d.getDuration(t);
    Eigen::Matrix4d A2exp = A2 * dt;
    A2exp = A2exp.exp();
    Eigen::Vector4d tmp4 = s1dt - beta[t] * Eigen::Vector4d(1, dt, dt * dt, dt * dt * dt);

    alpha.col(t) = A2exp.inverse() * tmp4;

    // setup the poly part
    beta_poly[t].resize(4,1);
    for (int n = 0; n < 4; n++) {
      beta_poly[t](n, 0) = Polynomial<double>(beta[t].row(n));
    }

  }

  PiecewisePolynomial<double> beta_traj(beta_poly, zmp_d.getSegmentTimes());
  s1_traj_ = ExponentialPlusPiecewisePolynomial<double>(Eigen::Matrix4d::Identity(), A2, alpha, beta_traj);

  // generate com traj
  Eigen::Matrix<double, 8, 8> Ay, Ayi;
  Eigen::Matrix<double, 8, 2> By;
  Ay.block<4, 4>(0, 0) = A_ + B_ * K_;
  Ay.block<4, 4>(0, 4) = -0.5 * B_ * R1i * B_.transpose();
  Ay.block<4, 4>(4, 0) = Eigen::Matrix<double, 4, 4>::Zero();
  Ay.block<4, 4>(4, 4) = A2;
  Ayi = Ay.inverse();
  By.block<4, 2>(0, 0) = B_ * R1i * D_ * Qy_;
  By.block<4, 2>(4, 0) = B2;

  Eigen::MatrixXd a(8, n_segments);
  a.bottomRows(4) = alpha;
  std::vector<Eigen::Matrix<Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic>> b_poly(n_segments);

  // set x0
  Eigen::Vector4d x = x0;
  x.head(2) -= zmp_tf;

  std::vector<Eigen::Matrix<double, 4, 4>> b(n_segments);
  Eigen::Matrix<double, 8, 1> tmp81;
  Eigen::Matrix<double, 8, 8> Ayexp;
  Eigen::Matrix<double, 4, 8> tmp48;
  for (int t = 0; t < n_segments; t++) {
    double dt = zmp_d.getDuration(t);
    b[t].col(3) = -Ayi.topRows(4) * By * c[t].col(3);
    for (int d = 2; d >= 0; d--) {
      tmp81.head(4) = b[t].col(d+1);
      tmp81.tail(4) = beta[t].col(d+1);
      tmp81 = tmp81 * (d+1);
      b[t].col(d) = Ayi.topRows(4) * (tmp81 - By * c[t].col(d));
    }

    a.block<4, 1>(0, t) = x - b[t].col(0);

    Ayexp = Ay * dt;
    Ayexp = Ayexp.exp();
    tmp48.block<4, 4>(0, 0).setIdentity();
    tmp48.block<4, 4>(0, 4).setZero();
    x = tmp48 * Ayexp * a.col(t) + b[t] * Eigen::Vector4d(1, dt, dt * dt, dt * dt * dt);

    b[t].block<2, 1>(0, 0) += zmp_tf; /// back to world coord

    // setup poly
    b_poly[t].resize(2, 1);
    for (int n = 0; n < 2; n++) {
      b_poly[t](n, 0) = Polynomial<double>(b[t].row(n));
    }
  }

  Eigen::Matrix<double, 2, 8> tmp28;
  tmp28.block<2, 2>(0, 0).setIdentity();
  tmp28.block<2, 6>(0, 2).setZero();
  PiecewisePolynomial<double> b_traj(b_poly, zmp_d.getSegmentTimes());

  com_traj_ = ExponentialPlusPiecewisePolynomial<double>(tmp28, Ay, a, b_traj);
  comd_traj_ = com_traj_.derivative();
  comdd_traj_ = comd_traj_.derivative(); // this is essentially the feedforward/nominal control input coming from lqr solution
}

/*
drake::lcmt_zmp_data ZMPPlanner::EncodeZMPData(double plan_time) const {
  drake::lcmt_zmp_data zmp_data_lcm;
  zmp_data_lcm.timestamp = 0;
  eigenToCArrayOfArrays(A_, zmp_data_lcm.A);
  eigenToCArrayOfArrays(B_, zmp_data_lcm.B);
  eigenToCArrayOfArrays(C_, zmp_data_lcm.C);
  eigenToCArrayOfArrays(D_, zmp_data_lcm.D);
  eigenToCArrayOfArrays(u0_, zmp_data_lcm.u0);
  eigenToCArrayOfArrays(R_, zmp_data_lcm.R);
  eigenToCArrayOfArrays(Qy_, zmp_data_lcm.Qy);
  zmp_data_lcm.s2 = 0;  // never used by the controller
  zmp_data_lcm.s2dot = 0;

  Eigen::Vector2d zmp_d_final = zmp_traj_.value(zmp_traj_.getEndTime());
  for (size_t i = 0; i < 2; i++) {
    zmp_data_lcm.x0[i][0] = zmp_d_final[i];
    zmp_data_lcm.x0[i + 2][0] = 0;
  }
  Eigen::Vector2d zmp_d = zmp_traj_.value(plan_time);
  eigenToCArrayOfArrays(zmp_d, zmp_data_lcm.y0);

  // Lyapunov function
  eigenToCArrayOfArrays(S_, zmp_data_lcm.S);
  Eigen::Vector4d s1 = s1_traj_.value(plan_time);
  eigenToCArrayOfArrays(s1, zmp_data_lcm.s1);
  eigenToCArrayOfArrays(s1_dot_, zmp_data_lcm.s1dot);

  // nominal com and comd
  Eigen::Vector2d com_d = com_traj_.value(plan_time);
  Eigen::Vector2d comd_d = comd_traj_.value(plan_time);
  for (int i = 0; i < 2; i++) {
    zmp_data_lcm.com[i][0] = com_d[i];
    zmp_data_lcm.com[i + 2][0] = comd_d[i];
  }

  return zmp_data_lcm;
}
*/

void ZMPPlanner::WriteToFile(const std::string &name, double dt) const {
  std::ofstream out;
  out.open(name);
  for (double t = 0; t < com_traj_.getEndTime(); t += dt)
    out << t << " " << com_traj_.value(t).transpose() << " " << comd_traj_.value(t).transpose() << " " << zmp_traj_.value(t).transpose() << std::endl;
  out.close();
}

}  // namespace systems
}  // namespace drake
