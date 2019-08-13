#include "drake/solvers/fbstab/components/riccati_linear_solver.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include <Eigen/Dense>

#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_residual.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

RiccatiLinearSolver::RiccatiLinearSolver(int N, int nx, int nu, int nc) {
  if (N <= 0 || nx <= 0 || nu <= 0 || nc <= 0) {
    throw std::runtime_error(
        "In RiccatiLinearSolver::RiccatiLinearSolver: all inputs must be "
        "positive.");
  }
  N_ = N;
  nx_ = nx;
  nu_ = nu;
  nc_ = nc;
  nz_ = (N_ + 1) * (nx_ + nu_);
  nl_ = (N_ + 1) * (nx_);
  nv_ = (N_ + 1) * (nc_);

  Q_.resize(N + 1);
  S_.resize(N + 1);
  R_.resize(N + 1);

  P_.resize(N + 1);
  SG_.resize(N + 1);
  M_.resize(N + 1);
  L_.resize(N + 1);
  SM_.resize(N + 1);
  AM_.resize(N + 1);

  h_.resize(N + 1);
  th_.resize(N + 1);

  for (int i = 0; i < N + 1; i++) {
    Q_[i].resize(nx, nx);
    S_[i].resize(nu, nx);
    R_[i].resize(nu, nu);

    P_[i].resize(nx, nu);
    SG_[i].resize(nu, nu);
    M_[i].resize(nx, nx);
    L_[i].resize(nx, nx);
    SM_[i].resize(nu, nx);
    AM_[i].resize(nx, nx);

    h_[i].resize(nx);
    th_[i].resize(nx);
  }

  gamma_.resize(nv_);
  mus_.resize(nv_);
  Gamma_.resize(nc, N + 1);

  Etemp_.resize(nc, nx);
  Ltemp_.resize(nc, nu);
  Linv_.resize(nx, nx);

  tx_.resize(nx);
  tu_.resize(nu);
  tl_.resize(nx);
  r1_.resize(nz_);
  r2_.resize(nl_);
  r3_.resize(nv_);
}

bool RiccatiLinearSolver::Initialize(const MpcVariable& x,
                                     const MpcVariable& xbar, double sigma) {
  const MpcData* const data = x.data();
  if (xbar.data_ != data) {
    throw std::runtime_error(
        "In RiccatiLinearSolver::Initialize: x and xbar have mismatched "
        "problem data.");
  }
  if (!MpcVariable::SameSize(x, xbar)) {
    throw std::runtime_error(
        "In RiccatiLinearSolver::Initialize: x and xbar are not the same "
        "size.");
  }
  if (sigma <= 0) {
    throw std::runtime_error(
        "In RiccatiLinearSolver::Initialize: sigma must be positive.");
  }

  Eigen::Vector2d temp;
  for (int i = 0; i < nv_; i++) {
    const double ys = x.y()(i) + sigma * (x.v()(i) - xbar.v()(i));
    temp = PFBGradient(ys, x.v()(i));

    gamma_(i) = temp(0);
    mus_(i) = temp(1) + sigma * temp(0);
    Gamma_(i) = gamma_(i) / mus_(i);
  }
  // Compute the barrier augmented Hessian.
  for (int i = 0; i < N_ + 1; i++) {
    const MatrixXd& Ei = (*data->E_)[i];
    const MatrixXd& Li = (*data->L_)[i];

    Q_[i].triangularView<Eigen::Lower>() =
        (*data->Q_)[i] + sigma * MatrixXd::Identity(nx_, nx_);
    R_[i].triangularView<Eigen::Lower>() =
        (*data->R_)[i] + sigma * MatrixXd::Identity(nu_, nu_);
    S_[i] = (*data->S_)[i];

    // Add barriers associated with E(i)x(i) + L(i)u(i) + d(i) <=0
    // Q(i) += E(i)'*diag(Gamma(i))*E(i)
    Etemp_.noalias() = Gamma_.col(i).asDiagonal() * Ei;
    Q_[i].triangularView<Eigen::Lower>() += Ei.transpose() * Etemp_;

    // R(i) += L(i)'*diag(Gamma(i))*L(i)
    Ltemp_.noalias() = Gamma_.col(i).asDiagonal() * Li;
    R_[i].triangularView<Eigen::Lower>() += Li.transpose() * Ltemp_;

    // S(i) += L(i) ' * diag(Gamma(i)) * E(i)
    S_[i].noalias() += Li.transpose() * Etemp_;
  }

  // Begin the matrix potion of the Riccati recursion.
  // Base case: L(0) = chol(sigma*I).
  L_[0] = sqrt(sigma) * MatrixXd::Identity(nx_, nx_);

#define FBSTAB_LLT_CHECK(llt)           \
  {                                     \
    if (llt.info() != Eigen::Success) { \
      return false;                     \
    }                                   \
  }

  for (int i = 0; i < N_; i++) {
    // Compute inv(L(i)) then
    // compute QQ = Q+inv(L*L') = Q + inv(L)'*inv(L)
    // and factor M = chol(QQ) in place.
    Linv_ = MatrixXd::Identity(nx_, nx_);
    L_[i].triangularView<Eigen::Lower>().solveInPlace(Linv_);
    L_[i].triangularView<Eigen::Lower>().transpose().solveInPlace(Linv_);
    M_[i].triangularView<Eigen::Lower>() = Q_[i] + Linv_;
    Eigen::LLT<Eigen::Ref<MatrixXd>> llt1(M_[i]);
    FBSTAB_LLT_CHECK(llt1);

    // Compute AM = A*inv(M)'.
    AM_[i] = (*data->A_)[i];
    M_[i]
        .triangularView<Eigen::Lower>()
        .transpose()
        .solveInPlace<Eigen::OnTheRight>(AM_[i]);

    // Compute SM = S*inv(M)'.
    SM_[i] = S_[i];
    M_[i]
        .triangularView<Eigen::Lower>()
        .transpose()
        .solveInPlace<Eigen::OnTheRight>(SM_[i]);

    // Factor SG = chol(R - SM*SM') in place.
    SG_[i].noalias() = R_[i] - SM_[i] * SM_[i].transpose();
    Eigen::LLT<Eigen::Ref<MatrixXd>> llt2(SG_[i]);
    FBSTAB_LLT_CHECK(llt2);

    // Compute P = (A*inv(QQ)S' - B)*inv(SG)',
    //           = (AM*SM' - B)*inv(SG)'.
    P_[i].noalias() = AM_[i] * SM_[i].transpose();
    P_[i] -= (*data->B_)[i];
    SG_[i]
        .triangularView<Eigen::Lower>()
        .transpose()
        .solveInPlace<Eigen::OnTheRight>(P_[i]);

    // Compute L(i+1) = chol(Pi)
    // where Pi = P*P' + AM*AM' + sigma I.
    L_[i + 1] = sigma * MatrixXd::Identity(nx_, nx_);
    L_[i + 1].noalias() += P_[i] * P_[i].transpose();
    L_[i + 1].noalias() += AM_[i] * AM_[i].transpose();
    Eigen::LLT<Eigen::Ref<MatrixXd>> llt3(L_[i + 1]);
    FBSTAB_LLT_CHECK(llt3);
  }

  // Finish the recursion, i.e., perform the i = N step.
  Linv_ = MatrixXd::Identity(nx_, nx_);
  L_[N_].triangularView<Eigen::Lower>().solveInPlace(Linv_);
  L_[N_].triangularView<Eigen::Lower>().transpose().solveInPlace(Linv_);

  // Compute M = chol(Q + inv(L*L')).
  M_[N_].triangularView<Eigen::Lower>() = Q_[N_] + Linv_;
  Eigen::LLT<Eigen::Ref<MatrixXd>> llt4(M_[N_]);
  FBSTAB_LLT_CHECK(llt4);

  // Compute SM = S*inv(M)'.
  SM_[N_] = S_[N_];
  M_[N_]
      .triangularView<Eigen::Lower>()
      .transpose()
      .solveInPlace<Eigen::OnTheRight>(SM_[N_]);

  // Compute SG = chol(R - SM*SM').
  SG_[N_].noalias() = R_[N_] - SM_[N_] * SM_[N_].transpose();
  Eigen::LLT<Eigen::Ref<MatrixXd>> llt5(SG_[N_]);
  FBSTAB_LLT_CHECK(llt5);

#undef FBSTAB_LLT_CHECK
  return true;
}

bool RiccatiLinearSolver::Solve(const MpcResidual& r, MpcVariable* dx) const {
  const MpcData* const data = dx->data();
  if (r.nz_ != dx->nz_ || r.nl_ != dx->nl_ || r.nv_ != dx->nv_) {
    throw std::runtime_error(
        "In RiccatiLinearSolver::Solve: r and dx size mismatch.");
  }
  // Compute the post-elimination residual,
  // r1 = rz - A'*(rv./mus) and r2 = -rl.
  r1_ = r.z_;
  r3_ = r.v_.cwiseQuotient(mus_);  // r3_ is used as a temp here
  data->gemvAT(r3_, -1.0, 1.0, &r1_);
  r2_ = -r.l_;
  // Get reshaped aliases for r1 and r2.
  Eigen::Map<MatrixXd> r1(r1_.data(), nx_ + nu_, N_ + 1);
  Eigen::Map<MatrixXd> r2(r2_.data(), nx_, N_ + 1);

  // Begin the vector portion of the Riccati recursion.
  // Base case: theta(0) = -rl(0), h(0) = inv(L*L')*theta(0) - rx(0).
  th_[0] = r2.col(0);
  h_[0] = th_[0];
  L_[0].triangularView<Eigen::Lower>().solveInPlace(h_[0]);
  L_[0].triangularView<Eigen::Lower>().transpose().solveInPlace(h_[0]);
  h_[0].noalias() -= r1.block(0, 0, nx_, 1);  // r1(0) = [rx(0);ru(0)]

  // Main loop:
  for (int i = 0; i < N_; i++) {
    // Compute theta(i+1).
    // tx = inv(M)*h
    tx_ = h_[i];
    M_[i].triangularView<Eigen::Lower>().solveInPlace(tx_);

    // tu = inv(SG)*(SM*tx + ru)
    // r1(i) = [rx(i);ru(i)], block extracts ru(i)
    tu_.noalias() = SM_[i] * tx_;
    tu_.noalias() += r1.block(nx_, i, nu_, 1);
    SG_[i].triangularView<Eigen::Lower>().solveInPlace(tu_);

    const auto rlp = r2.col(i + 1);
    th_[i + 1].noalias() = P_[i] * tu_ + AM_[i] * tx_;
    th_[i + 1].noalias() += rlp;

    // Compute h(i+1).
    const auto rxp = r1.block(0, i + 1, nx_, 1);
    h_[i + 1] = th_[i + 1];
    L_[i + 1].triangularView<Eigen::Lower>().solveInPlace(h_[i + 1]);
    L_[i + 1].triangularView<Eigen::Lower>().transpose().solveInPlace(
        h_[i + 1]);
    h_[i + 1].noalias() -= rxp;
  }

  // Begin the backwards recursion for the solution
  // by computing xN,uN, and lN.
  // u(N) = inv(SG*SG')*(SM*inv(M)*h + ru)
  tx_ = h_[N_];
  M_[N_].triangularView<Eigen::Lower>().solveInPlace(tx_);
  tu_.noalias() = SM_[N_] * tx_;
  tu_ += r1.block(nx_, N_, nu_, 1);
  SG_[N_].triangularView<Eigen::Lower>().solveInPlace(tu_);
  SG_[N_].triangularView<Eigen::Lower>().transpose().solveInPlace(tu_);

  // x(N) = -inv(M')*(inv(M)*h + SM'*u(N))
  tx_ = h_[N_];
  M_[N_].triangularView<Eigen::Lower>().solveInPlace(tx_);
  tx_.noalias() += SM_[N_].transpose() * tu_;
  M_[N_].triangularView<Eigen::Lower>().transpose().solveInPlace(tx_);
  tx_ *= -1.0;

  // l(N) = -inv(L*L')* (xN + theta)
  tl_ = tx_ + th_[N_];
  L_[N_].triangularView<Eigen::Lower>().solveInPlace(tl_);
  L_[N_].triangularView<Eigen::Lower>().transpose().solveInPlace(tl_);
  tl_ *= -1.0;

  // Copy these into the solution vector.
  // Using reshaped aliases to make indexing easier.
  Eigen::Map<MatrixXd> dz(dx->z_->data(), nx_ + nu_, N_ + 1);
  Eigen::Map<MatrixXd> dl(dx->l_->data(), nx_, N_ + 1);

  dz.block(0, N_, nx_, 1) = tx_;
  dz.block(nx_, N_, nu_, 1) = tu_;
  dl.col(N_) = tl_;

  // The main backwards recursion loop.
  for (int i = N_ - 1; i >= 0; i--) {
    // Solve SG'*u(i) = inv(SG)*(SM*inv(M)*h + ru) + P'*l(i+1)
    tx_ = h_[i];
    M_[i].triangularView<Eigen::Lower>().solveInPlace(tx_);

    // This is an alias.
    auto ui = dz.block(nx_, i, nu_, 1);  // dz(i) = [xi;ui], extract ui
    ui.noalias() = SM_[i] * tx_;
    ui += r1.block(nx_, i, nu_, 1);  // SM*tx + ru
    SG_[i].triangularView<Eigen::Lower>().solveInPlace(ui);
    ui.noalias() += P_[i].transpose() * dl.col(i + 1);
    SG_[i].triangularView<Eigen::Lower>().transpose().solveInPlace(ui);

    // Solve -M'*x(i) = inv(M)*h + SM'*u(i) + AM'*l(i+1)
    // This is an alias.
    auto xi = dz.block(0, i, nx_, 1);

    xi = h_[i];
    M_[i].triangularView<Eigen::Lower>().solveInPlace(xi);
    xi.noalias() += SM_[i].transpose() * ui;
    xi.noalias() += AM_[i].transpose() * dl.col(i + 1);
    M_[i].triangularView<Eigen::Lower>().transpose().solveInPlace(xi);
    xi *= -1.0;

    // Solve -L*L' * l(i) = theta(i) + x(i).
    auto li = dl.col(i);
    li = th_[i] + xi;
    L_[i].triangularView<Eigen::Lower>().solveInPlace(li);
    L_[i].triangularView<Eigen::Lower>().transpose().solveInPlace(li);
    li *= -1.0;
  }

  // Recover the inequality duals by solving
  // diag(mus)* dv = (rv + diag(gamma)*A*dz).
  VectorXd& dv = dx->v();
  dv = r.v_;
  // r3_ = A*dz, r3_ is being used as a temp.
  data->gemvA(dx->z(), 1.0, 0.0, &r3_);
  dv += gamma_.asDiagonal() * r3_;
  dv = dv.cwiseQuotient(mus_);

  // Compute dy = b - A*dz.
  VectorXd& dy = dx->y();
  data->gemvA(dx->z(), -1.0, 0.0, &dy);
  data->axpyb(1.0, &dy);

  return true;
}

Eigen::Vector2d RiccatiLinearSolver::PFBGradient(double a, double b) const {
  const double r = sqrt(a * a + b * b);
  const double d = 1.0 / sqrt(2.0);

  Eigen::Vector2d v;
  if (r < zero_tolerance_) {
    v(0) = alpha_ * (1.0 - d);
    v(1) = alpha_ * (1.0 - d);

  } else if ((a > 0) && (b > 0)) {
    v(0) = alpha_ * (1.0 - a / r) + (1.0 - alpha_) * b;
    v(1) = alpha_ * (1.0 - b / r) + (1.0 - alpha_) * a;

  } else {
    v(0) = alpha_ * (1.0 - a / r);
    v(1) = alpha_ * (1.0 - b / r);
  }

  return v;
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
