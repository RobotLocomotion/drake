#include "drake/solvers/fbstab/components/mpc_data.h"

#include <iostream>
#include <stdexcept>

#include <Eigen/Dense>

namespace drake {
namespace solvers {
namespace fbstab {

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Map = Eigen::Map<Eigen::MatrixXd>;
using ConstMap = Eigen::Map<const Eigen::MatrixXd>;

MpcData::MpcData(const std::vector<Eigen::MatrixXd>* Q,
                 const std::vector<Eigen::MatrixXd>* R,
                 const std::vector<Eigen::MatrixXd>* S,
                 const std::vector<Eigen::VectorXd>* q,
                 const std::vector<Eigen::VectorXd>* r,
                 const std::vector<Eigen::MatrixXd>* A,
                 const std::vector<Eigen::MatrixXd>* B,
                 const std::vector<Eigen::VectorXd>* c,
                 const std::vector<Eigen::MatrixXd>* E,
                 const std::vector<Eigen::MatrixXd>* L,
                 const std::vector<Eigen::VectorXd>* d,
                 const Eigen::VectorXd* x0) {
  if (Q == nullptr || R == nullptr || S == nullptr || q == nullptr ||
      r == nullptr || A == nullptr || B == nullptr || c == nullptr ||
      E == nullptr || L == nullptr || d == nullptr || x0 == nullptr) {
    throw std::runtime_error("A null pointer was passed to MpcData::MpcData.");
  }

  Q_ = Q;
  R_ = R;
  S_ = S;
  q_ = q;
  r_ = r;
  A_ = A;
  B_ = B;
  c_ = c;
  E_ = E;
  L_ = L;
  d_ = d;
  x0_ = x0;

  validate_length();
  validate_size();

  N_ = B_->size();
  nx_ = B_->at(0).rows();
  nu_ = B_->at(0).cols();
  nc_ = E_->at(0).rows();

  nz_ = (N_ + 1) * (nx_ + nu_);
  nl_ = (N_ + 1) * nx_;
  nv_ = (N_ + 1) * nc_;
}

void MpcData::gemvH(const Eigen::VectorXd& x, double a, double b,
                    Eigen::VectorXd* y) const {
  if (y == nullptr) {
    throw std::runtime_error("In MpcData::gemvH: y input is null.");
  }
  if (x.size() != nz_ || y->size() != nz_) {
    throw std::runtime_error("Size mismatch in MpcData::gemvH.");
  }
  if (b == 0.0) {
    y->fill(0.0);
  } else if (b != 1.0) {
    (*y) *= b;
  }

  // Create reshaped views of input and output vectors.
  Map w(y->data(), nx_ + nu_,
        N_ + 1);  // w = reshape(y, [nx + nu, N + 1]);
  ConstMap v(x.data(), nx_ + nu_,
             N_ + 1);  // v = reshape(x, [nx + nu, N + 1]);
  for (int i = 0; i < N_ + 1; i++) {
    const MatrixXd& Q = Q_->at(i);
    const MatrixXd& S = S_->at(i);
    const MatrixXd& R = R_->at(i);

    // These variables alias w.
    auto yx = w.block(0, i, nx_, 1);
    auto yu = w.block(nx_, i, nu_, 1);

    // These variables alias v.
    const auto vx = v.block(0, i, nx_, 1);
    const auto vu = v.block(nx_, i, nu_, 1);

    // [yx] += a * [Q(i) S(i)'] [vx]
    // [yu]        [S(i) R(i) ] [vu]
    // Using lazyProduct is inefficient so should be avoided when possible.
    if (a == 1.0) {
      yx.noalias() += Q * vx + S.transpose() * vu;
      yu.noalias() += S * vx + R * vu;
    } else if (a == -1.0) {
      yx.noalias() -= Q * vx + S.transpose() * vu;
      yu.noalias() -= S * vx + R * vu;
    } else {
      yx += a * Q.lazyProduct(vx);
      yx += a * S.transpose().lazyProduct(vu);
      yu += a * S.lazyProduct(vx);
      yu += a * R.lazyProduct(vu);
    }
  }
}

void MpcData::gemvA(const Eigen::VectorXd& x, double a, double b,
                    Eigen::VectorXd* y) const {
  if (y == nullptr) {
    throw std::runtime_error("In MpcData::gemvA: y input is null.");
  }
  if (x.size() != nz_ || y->size() != nv_) {
    throw std::runtime_error("Size mismatch in MpcData::gemvA.");
  }
  if (b == 0.0) {
    y->fill(0.0);
  } else if (b != 1.0) {
    (*y) *= b;
  }
  // Create reshaped views of input and output vectors.
  ConstMap z(x.data(), nx_ + nu_, N_ + 1);
  Map w(y->data(), nc_, N_ + 1);

  for (int i = 0; i < N_ + 1; i++) {
    const MatrixXd& E = E_->at(i);
    const MatrixXd& L = L_->at(i);

    // This aliases w.
    auto yi = w.col(i);

    // These alias z.
    const auto xi = z.block(0, i, nx_, 1);
    const auto ui = z.block(nx_, i, nu_, 1);

    // yi += a*(E*vx + L*vu)
    if (a == 1.0) {
      yi.noalias() += E * xi + L * ui;
    } else if (a == -1.0) {
      yi.noalias() -= E * xi + L * ui;
    } else {
      yi += a * E.lazyProduct(xi);
      yi += a * L.lazyProduct(ui);
    }
  }
}

void MpcData::gemvG(const Eigen::VectorXd& x, double a, double b,
                    Eigen::VectorXd* y) const {
  if (y == nullptr) {
    throw std::runtime_error("In MpcData::gemvG: y input is null.");
  }
  if (x.size() != nz_ || y->size() != nl_) {
    throw std::runtime_error("Size mismatch in MpcData::gemvG.");
  }
  if (b == 0.0) {
    y->fill(0.0);
  } else if (b != 1.0) {
    (*y) *= b;
  }
  // Create reshaped views of input and output vectors.
  ConstMap z(x.data(), nx_ + nu_, N_ + 1);
  Map w(y->data(), nx_, N_ + 1);

  w.col(0).noalias() += -a * z.block(0, 0, nx_, 1);

  for (int i = 1; i < N_ + 1; i++) {
    const MatrixXd& A = A_->at(i - 1);
    const MatrixXd& B = B_->at(i - 1);

    // Alias for the output at stage i.
    auto yi = w.col(i);
    // Aliases for the state and control at stage i - 1.
    const auto xm1 = z.block(0, i - 1, nx_, 1);
    const auto um1 = z.block(nx_, i - 1, nu_, 1);
    // Alias for the state at stage i.
    const auto xi = z.block(0, i, nx_, 1);

    // y(i) += a*(A(i-1)*x(i-1) + B(i-1)u(i-1) - x(i))
    if (a == 1.0) {
      yi.noalias() += A * xm1 + B * um1;
      yi.noalias() -= xi;
    } else if (a == -1.0) {
      yi.noalias() -= A * xm1 + B * um1;
      yi.noalias() += xi;
    } else {
      yi += a * A.lazyProduct(xm1);
      yi += a * B.lazyProduct(um1);
      yi -= a * xi;
    }
  }
}

void MpcData::gemvGT(const Eigen::VectorXd& x, double a, double b,
                     Eigen::VectorXd* y) const {
  if (y == nullptr) {
    throw std::runtime_error("In MpcData::gemvGT: y input is null.");
  }
  if (x.size() != nl_ || y->size() != nz_) {
    throw std::runtime_error("Size mismatch in MpcData::gemvGT.");
  }
  if (b == 0.0) {
    y->fill(0.0);
  } else if (b != 1.0) {
    (*y) *= b;
  }

  // Create reshaped views of input and output vectors.
  ConstMap v(x.data(), nx_, N_ + 1);
  Map w(y->data(), nx_ + nu_, N_ + 1);

  for (int i = 0; i < N_; i++) {
    const MatrixXd& A = A_->at(i);
    const MatrixXd& B = B_->at(i);

    // Aliases for the dual variables at stage i and i+1;
    const auto vi = v.col(i);
    const auto vp1 = v.col(i + 1);

    // Aliases for the state and control at stage i.
    auto xi = w.block(0, i, nx_, 1);
    auto ui = w.block(nx_, i, nu_, 1);

    // x(i) += a*(-v(i) + A(i)' * v(i+1))
    // u(i) += a*B(i)' * v(i+1)
    xi.noalias() += -a * vi;
    if (a == 1.0) {
      xi.noalias() += A.transpose() * vp1;
      ui.noalias() += B.transpose() * vp1;
    } else if (a == -1.0) {
      xi.noalias() -= A.transpose() * vp1;
      ui.noalias() -= B.transpose() * vp1;
    } else {
      xi.noalias() += a * A.transpose().lazyProduct(vp1);
    }
  }
  // The i = N step of the recursion.
  w.block(0, N_, nx_, 1).noalias() += -a * v.col(N_);
}

void MpcData::gemvAT(const Eigen::VectorXd& x, double a, double b,
                     Eigen::VectorXd* y) const {
  if (y == nullptr) {
    throw std::runtime_error("In MpcData::gemvAT: y input is null.");
  }
  if (x.size() != nv_ || y->size() != nz_) {
    throw std::runtime_error("Size mismatch in MpcData::gemvAT.");
  }
  if (b == 0.0) {
    y->fill(0.0);
  } else if (b != 1.0) {
    (*y) *= b;
  }
  // Create reshaped views of input and output vectors.
  ConstMap v(x.data(), nc_, N_ + 1);
  Map w(y->data(), nx_ + nu_, N_ + 1);

  for (int i = 0; i < N_ + 1; i++) {
    const MatrixXd& E = E_->at(i);
    const MatrixXd& L = L_->at(i);

    auto xi = w.block(0, i, nx_, 1);
    auto ui = w.block(nx_, i, nu_, 1);

    const auto vi = v.col(i);
    // x(i) += a*E(i)' * v(i)
    // u(i) += a*L(i)' * v(i)
    if (a == 1.0) {
      xi.noalias() += E.transpose() * vi;
      ui.noalias() += L.transpose() * vi;
    } else if (a == -1.0) {
      xi.noalias() -= E.transpose() * vi;
      ui.noalias() -= L.transpose() * vi;
    } else {
      ui.noalias() += a * L.transpose().lazyProduct(vi);
      xi.noalias() += a * E.transpose().lazyProduct(vi);
    }
  }
}

void MpcData::axpyf(double a, Eigen::VectorXd* y) const {
  if (y == nullptr) {
    throw std::runtime_error("In MpcData::axpyf: y input is null.");
  }
  if (y->size() != nz_) {
    throw std::runtime_error("Size mismatch in MpcData::axpyf.");
  }

  // Create reshaped view of the input vector.
  Map w(y->data(), nx_ + nu_, N_ + 1);

  for (int i = 0; i < N_ + 1; i++) {
    auto xi = w.block(0, i, nx_, 1);
    auto ui = w.block(nx_, i, nu_, 1);

    xi.noalias() += a * q_->at(i);
    ui.noalias() += a * r_->at(i);
  }
}

void MpcData::axpyh(double a, Eigen::VectorXd* y) const {
  if (y == nullptr) {
    throw std::runtime_error("In MpcData::axpyh: y input is null.");
  }
  if (y->size() != nl_) {
    throw std::runtime_error("Size mismatch in MpcData::axpyh.");
  }
  // Create reshaped view of the input vector.
  Map w(y->data(), nx_, N_ + 1);
  w.col(0) += -a * (*x0_);

  for (int i = 1; i < N_ + 1; i++) {
    w.col(i) += -a * c_->at(i - 1);
  }
}

void MpcData::axpyb(double a, Eigen::VectorXd* y) const {
  if (y == nullptr) {
    throw std::runtime_error("In MpcData::axpyb: y input is null.");
  }
  if (y->size() != nv_) {
    throw std::runtime_error("Size mismatch in MpcData::axpyb.");
  }
  // Create reshaped view of the input vector.
  Map w(y->data(), nc_, N_ + 1);

  for (int i = 0; i < N_ + 1; i++) {
    w.col(i).noalias() += -a * d_->at(i);
  }
}

void MpcData::validate_length() const {
  bool OK = true;

  const auto N = Q_->size();
  if (N <= 0) {
    throw std::runtime_error("Horizon length must be at least 1.");
  }

  OK = OK && N == R_->size();
  OK = OK && N == S_->size();
  OK = OK && N == q_->size();
  OK = OK && N == r_->size();
  OK = OK && (N - 1) == A_->size();
  OK = OK && (N - 1) == B_->size();
  OK = OK && (N - 1) == c_->size();
  OK = OK && N == E_->size();
  OK = OK && N == L_->size();
  OK = OK && N == d_->size();

  if (!OK) {
    throw std::runtime_error(
        "Sequence length mismatch in input data to MpcData.");
  }
}

void MpcData::validate_size() const {
  const int N = B_->size();

  const int nx = Q_->at(0).rows();
  if (x0_->size() != nx) {
    throw std::runtime_error("Size mismatch in input data to MpcData.");
  }
  for (int i = 0; i < N + 1; i++) {
    if (Q_->at(i).rows() != nx || Q_->at(i).cols() != nx) {
      throw std::runtime_error("Size mismatch in Q input to MpcData.");
    }
    if (S_->at(i).cols() != nx) {
      throw std::runtime_error("Size mismatch in S input to MpcData.");
    }
    if (q_->at(i).size() != nx) {
      throw std::runtime_error("Size mismatch in q input to MpcData.");
    }
    if (E_->at(i).cols() != nx) {
      throw std::runtime_error("Size mismatch in E input to MpcData.");
    }
  }
  for (int i = 0; i < N; i++) {
    if (A_->at(i).rows() != nx || A_->at(i).cols() != nx) {
      throw std::runtime_error("Size mismatch in A input to MpcData.");
    }
    if (B_->at(i).rows() != nx) {
      throw std::runtime_error("Size mismatch in B input to MpcData.");
    }
    if (c_->at(i).size() != nx) {
      throw std::runtime_error("Size mismatch in c input to MpcData.");
    }
  }

  const int nu = R_->at(0).rows();
  for (int i = 0; i < N + 1; i++) {
    if (R_->at(i).rows() != nu || R_->at(i).cols() != nu) {
      throw std::runtime_error("Size mismatch in R input to MpcData.");
    }
    if (S_->at(i).rows() != nu) {
      throw std::runtime_error("Size mismatch in S input to MpcData.");
    }
    if (r_->at(i).size() != nu) {
      throw std::runtime_error("Size mismatch in r input to MpcData.");
    }
    if (L_->at(i).cols() != nu) {
      throw std::runtime_error("Size mismatch in L input to MpcData.");
    }
  }
  for (int i = 0; i < N; i++) {
    if (B_->at(i).cols() != nu) {
      throw std::runtime_error("Size mismatch in B input to MpcData.");
    }
  }

  const int nc = E_->at(0).rows();
  for (int i = 0; i < N + 1; i++) {
    if (E_->at(i).rows() != nc) {
      throw std::runtime_error("Size mismatch in E input to MpcData.");
    }
    if (L_->at(i).rows() != nc) {
      throw std::runtime_error("Size mismatch in L input to MpcData.");
    }
    if (d_->at(i).size() != nc) {
      throw std::runtime_error("Size mismatch in d input to MpcData.");
    }
  }
}

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
