#include "drake/solvers/fbstab/test/ocp_generator.h"

#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/fbstab_mpc.h"

namespace drake {
namespace solvers {
namespace fbstab {
namespace test {

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

// Returns a structure ready to be fed into FBstab.
FBstabMpc::QPData OcpGenerator::GetFBstabInput() const {
  if (!data_populated_) {
    throw std::runtime_error(
        "In OcpGenerator::GetFBstabInput: Call a problem creator method "
        "first.");
  }

  FBstabMpc::QPData s;

  s.Q = &Q_;
  s.R = &R_;
  s.S = &S_;
  s.q = &q_;
  s.r = &r_;
  s.A = &A_;
  s.B = &B_;
  s.c = &c_;
  s.E = &E_;
  s.L = &L_;
  s.d = &d_;
  s.x0 = &x0_;

  return s;
}

OcpGenerator::SimulationInputs OcpGenerator::GetSimulationInputs() const {
  if (!data_populated_) {
    throw std::runtime_error(
        "In OcpGenerator::GetSimulationInputs: Call a problem creator method "
        "first.");
  }
  SimulationInputs out;
  out.x0 = x0_;
  out.A = Asim_;
  out.B = Bsim_;
  out.C = Csim_;
  out.D = Dsim_;
  out.T = T_;

  return out;
}

void OcpGenerator::CopolymerizationReactor(int N) {
  if (N <= 0) {
    throw std::runtime_error(
        "In OcpGenerator::CopolymerizationReactor: N <= 0.");
  }
  /* This model is a modal realization of the following 4 x 5 transfer function.
   * Which is then converted to discrete time using a zero order hold
   * approximation.
   * G(1,1) = 0.34/(0.85*s+1)
   * G(1,2) = 0.21/(0.42*s+1)
   * G(1,3) = 0.5*(0.5*s+1)/(12*s^2+0.4*s+1)
   * G(1,5) = 6.46*(0.9*s+1)/(0.007*s^2 + 0.3*s+ 1)
   * G(2,1) = -0.41/(2.41*s+1)
   * G(2,2) = 0.66/(1.51*s+1)
   * G(2,3) = -0.3/(1.45*s+1)
   * G(2,5) = -0.372/(0.8*s+1)
   * G(3,1) = 0.3/(2.54*s+1)
   * G(3,2) = 0.49/(1.54*s+1)
   * G(3,3) = -0.71/(1.35*s+1)
   * G(3,4) = -0.2/(2.71*s+1)
   * G(3,5) = -4.71/(0.008*s^2 + 0.41*s+1)
   * G(4,5) = 1.02*(0.23*s+1)/(0.07*s^2 + 0.31*s +1);
   */

  MatrixXd A = MatrixXd::Zero(18, 18);
  Eigen::VectorXi i(26);
  i << 1, 2, 3, 4, 5, 6, 7, 8, 7, 8, 9, 10, 11, 12, 13, 12, 13, 14, 15, 16, 15,
      16, 17, 18, 17, 18;

  Eigen::VectorXi j(26);
  j << 1, 2, 3, 4, 5, 6, 7, 7, 8, 8, 9, 10, 11, 12, 12, 13, 13, 14, 15, 15, 16,
      16, 17, 17, 18, 18;
  VectorXd v(26);
  v << 0.55531, 0.81264, 0.82131, 0.30408, 0.71811, 0.72276, 0.97319, 0.12353,
      -0.16471, 0.98966, 0.70834, 0.69048, 0.83152, -0.016569, 0.07277,
      -0.040608, 0.17835, 0.53526, -0.015422, 0.04805, -0.093847, 0.2924,
      -0.22577, 0.43126, -0.38505, 0.2517;

  for (int k = 0; k < i.size(); k++) {
    A(i(k) - 1, j(k) - 1) = v(k);
  }

  MatrixXd B = MatrixXd::Zero(18, 5);

  i.resize(18);
  i << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18;
  j.resize(18);
  j << 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 5, 5, 5, 5, 5, 5, 5;
  v.resize(18);
  v << 0.18899, 0.22577, 0.11347, 0.14614, 0.21282, 0.21347, 0.24707, 0.015512,
      0.21145, 0.41785, 0.11415, 0.14554, 2.9448, 0.1859, 0.04805, 0.36229,
      0.21563, 0.41905;

  for (int k = 0; k < i.size(); k++) {
    B(i(k) - 1, j(k) - 1) = v(k);
  }

  MatrixXd C = MatrixXd::Zero(4, 18);
  C.row(0) << 0.8, 0, 0, 1, 0, 0, 0.0416666666666667, 0.333333333333333, 0, 0,
      0, 25.9553571428571, 1.80245535714286, 0, 0, 0, 0, 0;
  C.row(1) << 0, -0.340248962655602, 0, 0, 0.874172185430464, 0, 0, 0,
      -0.413793103448276, 0, 0, 0, 0, -0.930000000000000, 0, 0, 0, 0;
  C.row(2) << 0, 0, 0.47244, 0, 0, 0.63636, 0, 0, 0, -0.52593, -0.2952, 0, 0, 0,
      0, -9.1992, 0, 0;
  C.row(3) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.6757, 1.8214;

  VectorXd c = VectorXd::Zero(18);

  VectorXd x0 = VectorXd::Zero(18);
  for (int k = 0; k < x0.size(); k++) {
    x0(k) = 0.2 * sin(k + 1);
  }
  // Cost function.
  MatrixXd Q = C.transpose() * C;
  MatrixXd R = 0.1 * MatrixXd::Identity(5, 5);
  MatrixXd S = MatrixXd::Zero(5, 18);
  VectorXd q = VectorXd::Zero(18);
  VectorXd r = VectorXd::Zero(5);

  // Input constraints.
  const double umax = 5.0 / 100.0;
  MatrixXd E = MatrixXd::Zero(10, 18);

  MatrixXd L(10, 5);
  L << MatrixXd::Identity(5, 5), -MatrixXd::Identity(5, 5);

  VectorXd d = -umax * VectorXd::Ones(10);

  CopyOverHorizon(Q, R, S, q, r, A, B, c, E, L, d, x0, N);

  // Simulation data.
  Asim_ = A;
  Bsim_ = B;
  Csim_ = C;
  Dsim_ = MatrixXd::Zero(Csim_.rows(), Bsim_.cols());
  T_ = 200;
}

void OcpGenerator::SpacecraftRelativeMotion(int N) {
  if (N <= 0) {
    throw std::runtime_error(
        "In OcpGenerator::SpacecraftRelativeMotion: N <= 0.");
  }
  // Model parameters.
  const double mu = 398600.4418;  // gravitational constant
  const double Re = 6371;         // radius of the earth
  const double alt = 650;         // orbit altitude

  const double n = sqrt(mu / pow(Re + alt, 3));

  // Construct the continuous time model.
  MatrixXd A(6, 6);
  MatrixXd A21(3, 3);
  MatrixXd A22(3, 3);

  A21 << 2 * n * n, 0, 0, 0, 0, 0, 0, 0, -n * n;
  A22 << 0, 2 * n, 0, -2 * n, 0, 0, 0, 0, 0;
  A << MatrixXd::Zero(3, 3), MatrixXd::Identity(3, 3), A21, A22;

  MatrixXd B(6, 3);
  B << MatrixXd::Zero(3, 3), MatrixXd::Identity(3, 3);

  MatrixXd C = MatrixXd::Identity(6, 6);

  // Convert to discrete time.
  const double ts = 30.0;  // sampling period
  A = (MatrixXd::Identity(6, 6) + ts * A);
  B = ts * B;

  // Form a system with Delta v as the input.
  B = A * B;

  VectorXd c = VectorXd::Zero(6);
  VectorXd x0(6);
  x0 << -2.8, -0.01, -1, 0, 0, 0;

  // Cost function.
  MatrixXd Q = MatrixXd::Zero(6, 6);
  MatrixXd R = MatrixXd::Zero(3, 3);
  MatrixXd S = MatrixXd::Zero(3, 6);

  Q.diagonal() << VectorXd::Ones(3), 1e-3 * VectorXd::Ones(3);
  R.diagonal() << VectorXd::Ones(3);

  VectorXd q = VectorXd::Zero(6);
  VectorXd r = VectorXd::Zero(3);

  // Inputs constraints |u| <= umax and
  // velocity constraints |v| <= vmax.
  MatrixXd E(12, 6);
  MatrixXd L(12, 3);
  const double umax = 1e-3;
  const double vmax = 1e-3;

  E << MatrixXd::Zero(6, 6), MatrixXd::Zero(3, 3), MatrixXd::Identity(3, 3),
      MatrixXd::Zero(3, 3), -MatrixXd::Identity(3, 3);

  L << MatrixXd::Identity(3, 3), -MatrixXd::Identity(3, 3),
      MatrixXd::Zero(6, 3);

  VectorXd d(12);
  d << -umax * VectorXd::Ones(6), -vmax * VectorXd::Ones(6);

  CopyOverHorizon(Q, R, S, q, r, A, B, c, E, L, d, x0, N);

  // Simulation data.
  Asim_ = A;
  Bsim_ = B;
  Csim_ = C;
  Dsim_ = MatrixXd::Zero(Csim_.rows(), Bsim_.cols());
  T_ = 100;
}
void OcpGenerator::ServoMotor(int N) {
  if (N <= 0) {
    throw std::runtime_error("In OcpGenerator::ServoMotor: N <= 0.");
  }
  // Model parameters.
  const double kt = 10.0;
  const double bl = 25.0;
  const double Jm = 0.5;
  const double bm = 0.1;
  const double ktheta = 1280.2;
  const double RR = 20.0;
  const double rho = 20.0;
  const double Jl = 20 * Jm;

  const double umax = 220.0;
  const double ymax = 78.5358;

  // Continuous time matrices for
  // \dot{x} = Ax + Bu, y = Cx.
  MatrixXd A(4, 4);
  MatrixXd B(4, 1);
  A << 0, 1, 0, 0, -ktheta / Jl, -bl / Jl, ktheta / (rho * Jl), 0, 0, 0, 0, 1,
      ktheta / (rho * Jm), 0, -ktheta / (rho * rho * Jm),
      -(bm + kt * kt / RR) / Jm;
  B << 0, 0, 0, kt / (RR * Jm);

  MatrixXd C(2, 4);
  C << 1, 0, 0, 0, ktheta, 0, -ktheta / rho, 0;

  // Convert to discrete time using forward Euler.
  const double ts = 0.05;
  A = (MatrixXd::Identity(4, 4) + ts * A);
  B = ts * B;

  VectorXd c = VectorXd::Zero(4);
  VectorXd x0 = VectorXd::Zero(4);

  // Cost function
  MatrixXd Q = MatrixXd::Zero(4, 4);
  MatrixXd R = MatrixXd::Zero(1, 1);
  MatrixXd S = MatrixXd::Zero(1, 4);
  Q(0, 0) = 1000;
  R(0, 0) = 1e-4;

  constexpr double pi = 3.1415926535897;
  VectorXd xtrg(4);
  xtrg << 30 * pi / 180, 0, 0, 0;
  VectorXd utrg(1);
  utrg << 0;

  VectorXd q = -Q * xtrg;
  VectorXd r = -R * utrg;

  // Constraints
  MatrixXd E(4, 4);
  MatrixXd L(4, 1);
  VectorXd d(4);

  E << C.row(1), -C.row(1), MatrixXd::Zero(2, 4);
  L << 0, 0, 1, -1;
  d << -ymax, -ymax, -umax, -umax;

  CopyOverHorizon(Q, R, S, q, r, A, B, c, E, L, d, x0, N);

  // Simulation data.
  Asim_ = A;
  Bsim_ = B;
  Csim_ = C;
  Dsim_ = MatrixXd::Zero(Csim_.rows(), Bsim_.cols());
  T_ = 40;
}
// Fills internal storage with data
// for a double integrator problem with horizon N.
void OcpGenerator::DoubleIntegrator(int N) {
  if (N <= 0) {
    throw std::runtime_error("In OcpGenerator::DoubleIntegrator: N <= 0.");
  }
  MatrixXd Q(2, 2);
  MatrixXd R(1, 1);
  MatrixXd S(1, 2);
  VectorXd q(2);
  VectorXd r(1);

  MatrixXd A(2, 2);
  MatrixXd B(2, 1);
  VectorXd c(2);

  MatrixXd E(6, 2);
  MatrixXd L(6, 1);
  VectorXd d(6);

  VectorXd x0(2);

  Q << 2, 0, 0, 1;
  S << 1, 0;
  R << 3;
  q << -2, 0;
  r << 0;

  A << 1, 1, 0, 1;
  B << 0, 1;
  c << 0, 0;

  E << -1, 0, 0, -1, 1, 0, 0, 1, 0, 0, 0, 0;
  L << 0, 0, 0, 0, -1, 1;
  d << 0, 0, -2, -2, -1, -1;

  x0 << 0, 0;

  CopyOverHorizon(Q, R, S, q, r, A, B, c, E, L, d, x0, N);

  // Simulation data.
  Asim_ = A;
  Bsim_ = B;
  Csim_ = MatrixXd::Identity(2, 2);
  Dsim_ = MatrixXd::Zero(Csim_.rows(), Bsim_.cols());
  T_ = 40;
}

void OcpGenerator::CopyOverHorizon(const MatrixXd& Q, const MatrixXd& R,
                                   const MatrixXd& S, const VectorXd& q,
                                   const VectorXd& r, const MatrixXd& A,
                                   const MatrixXd& B, const VectorXd& c,
                                   const MatrixXd& E, const MatrixXd& L,
                                   const VectorXd& d, const VectorXd& x0,
                                   int N) {
  // These are indexed from 0 to N.
  for (int i = 0; i < N + 1; i++) {
    Q_.push_back(Q);
    R_.push_back(R);
    S_.push_back(S);
    q_.push_back(q);
    r_.push_back(r);
    if (i == 0) {  // don't impose constraints on x0
      MatrixXd E0 = MatrixXd::Zero(E.rows(), E.cols());
      E_.push_back(E0);
    } else {
      E_.push_back(E);
    }
    L_.push_back(L);
    d_.push_back(d);
  }

  // These are indexed from 0 to N-1.
  for (int i = 0; i < N; i++) {
    A_.push_back(A);
    B_.push_back(B);
    c_.push_back(c);
  }
  x0_ = x0;
  N_ = N;
  nx_ = Q.rows();
  nu_ = R.rows();
  nc_ = E.rows();
  nz_ = (nx_ + nu_) * (N + 1);
  nl_ = nx_ * (N + 1);
  nv_ = nc_ * (N + 1);
  data_populated_ = true;
}

Eigen::Vector4d OcpGenerator::ProblemSize() {
  Eigen::Vector4d out;
  out << N_, nx_, nu_, nc_;
  return out;
}

}  // namespace test
}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
