#include "drake/systems/controllers/linear_optimal_control.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/primitives/linear_system.h"

namespace drake {
namespace systems {

std::unique_ptr<systems::LinearSystem<double>> LinearQuadraticRegulator(
    const LinearSystem<double>& system,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  const int num_states = system.B().rows(), num_inputs = system.B().cols();

  const auto& S =
      ContinuousAlgebraicRiccatiEquation(system.A(), system.B(), Q, R);

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  const Eigen::MatrixXd K = R_cholesky.solve(system.B().transpose() * S);

  // Return the controller: u = -Kx.
  return std::make_unique<systems::LinearSystem<double>>(
      Eigen::Matrix<double, 0, 0>::Zero(),   // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -K);                                   // D
}

std::unique_ptr<systems::AffineSystem<double>> LinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  // TODO(russt): accept optional additional argument to return the cost-to-go
  // but note that it will be a full quadratic form (x'S2x + s1'x + s0).

  DRAKE_DEMAND(system.get_num_input_ports() == 1);
  const int num_inputs = system.get_input_port(0).get_size(),
            num_states = context.get_continuous_state()->size();
  DRAKE_DEMAND(num_states > 0);
  // TODO(russt): Confirm behavior if Q is not PSD.

  auto linear_system = Linearize(system, context);

  const auto& S = ContinuousAlgebraicRiccatiEquation(linear_system->A(),
                                                     linear_system->B(), Q, R);

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  const Eigen::MatrixXd K =
      R_cholesky.solve(linear_system->B().transpose() * S);

  const Eigen::VectorXd& x0 =
      context.get_continuous_state_vector().CopyToVector();
  const auto& u0 = system.EvalEigenVectorInput(context, 0);

  // Return the affine controller: u = u0 - K(x-x0).
  return std::make_unique<systems::AffineSystem<double>>(
      Eigen::Matrix<double, 0, 0>::Zero(),   // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // xDot0
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -K,                                    // D
      u0 + K * x0);                          // y0
}

Eigen::MatrixXd ContinuousAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  const Eigen::Index n = B.rows(), m = B.cols();

  DRAKE_DEMAND(A.rows() == n && A.cols() == n);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);
  DRAKE_DEMAND(
      CompareMatrices(Q, Q.transpose(), 1e-10, MatrixCompareType::absolute));
  DRAKE_DEMAND(
      CompareMatrices(R, R.transpose(), 1e-10, MatrixCompareType::absolute));

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);

  Eigen::MatrixXd H(2 * n, 2 * n);
  H << A, B * R_cholesky.solve(B.transpose()), Q, -A.transpose();
  if (R_cholesky.info() != Eigen::Success)
    throw std::runtime_error("R must be positive definite");

  Eigen::MatrixXd Z = H;
  Eigen::MatrixXd Z_old;

  // these could be options
  const double tolerance = 1e-9;
  const double max_iterations = 100;

  double relative_norm;
  size_t iteration = 0;

  const double p = static_cast<double>(Z.rows());

  do {
    Z_old = Z;
    // R. Byers. Solving the algebraic Riccati equation with the matrix sign
    // function. Linear Algebra Appl., 85:267–279, 1987
    // Added determinant scaling to improve convergence (converges in rough half
    // the iterations with this)
    double ck = std::pow(std::abs(Z.determinant()), -1.0 / p);
    Z *= ck;
    Z = Z - 0.5 * (Z - Z.inverse());
    relative_norm = (Z - Z_old).norm();
    iteration++;
  } while (iteration < max_iterations && relative_norm > tolerance);

  Eigen::MatrixXd W11 = Z.block(0, 0, n, n);
  Eigen::MatrixXd W12 = Z.block(0, n, n, n);
  Eigen::MatrixXd W21 = Z.block(n, 0, n, n);
  Eigen::MatrixXd W22 = Z.block(n, n, n, n);

  Eigen::MatrixXd lhs(2 * n, n);
  Eigen::MatrixXd rhs(2 * n, n);
  Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(n, n);
  lhs << W12, W22 + eye;
  rhs << W11 + eye, W21;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);

  return svd.solve(rhs);
}

}  // namespace systems
}  // namespace drake
