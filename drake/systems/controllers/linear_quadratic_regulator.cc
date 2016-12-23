#include "drake/systems/controllers/linear_quadratic_regulator.h"

#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/math/continuous_algebraic_ricatti_equation.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {

LinearQuadraticRegulatorResult LinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
  Eigen::Index n = A.rows(), m = B.cols();
  DRAKE_DEMAND(n > 0 && m > 0);
  DRAKE_DEMAND(B.rows() == n && A.cols() == n);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);
  // N is default to Matrix<double, 0, 0>.
  if (N.rows() != 0) {
    DRAKE_DEMAND(N.rows() == n && N.cols() == m);
  }
  DRAKE_DEMAND(is_approx_equal_abstol(R, R.transpose(), 1e-10));

  LinearQuadraticRegulatorResult ret;

  Eigen::LLT<Eigen::MatrixXd> R_cholesky(R);
  if (R_cholesky.info() != Eigen::Success)
    throw std::runtime_error("R must be positive definite");

  if (N.rows() != 0) {
    Eigen::MatrixXd Q1 = Q - N * R_cholesky.solve(N.transpose());
    Eigen::MatrixXd A1 = A - B * R_cholesky.solve(N.transpose());

    ret.S = math::ContinuousAlgebraicRiccatiEquation(A1, B, Q1, R_cholesky);
    ret.K = R_cholesky.solve(B.transpose() * ret.S + N.transpose());
  } else {
    ret.S = math::ContinuousAlgebraicRiccatiEquation(A, B, Q, R_cholesky);
    ret.K = R_cholesky.solve(B.transpose() * ret.S);
  }
  return ret;
}

std::unique_ptr<systems::LinearSystem<double>> LinearQuadraticRegulator(
    const LinearSystem<double>& system,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
  DRAKE_DEMAND(system.time_period() == 0.0);
  // TODO(russt): Support discrete-time systems.

  const int num_states = system.B().rows(), num_inputs = system.B().cols();

  LinearQuadraticRegulatorResult lqr_result =
      LinearQuadraticRegulator(system.A(), system.B(), Q, R, N);

  // Return the controller: u = -Kx.
  return std::make_unique<systems::LinearSystem<double>>(
      Eigen::Matrix<double, 0, 0>::Zero(),   // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -lqr_result.K);                        // D
}

std::unique_ptr<systems::AffineSystem<double>> LinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
  // TODO(russt): accept optional additional argument to return the cost-to-go
  // but note that it will be a full quadratic form (x'S2x + s1'x + s0).

  DRAKE_DEMAND(system.get_num_input_ports() == 1);
  const int num_inputs = system.get_input_port(0).get_size(),
            num_states = context.get_continuous_state()->size();
  DRAKE_DEMAND(num_states > 0);
  DRAKE_DEMAND(context.has_only_continuous_state());
  // TODO(russt): Confirm behavior if Q is not PSD.

  auto linear_system = Linearize(system, context);

  LinearQuadraticRegulatorResult lqr_result =
      LinearQuadraticRegulator(linear_system->A(), linear_system->B(), Q, R, N);

  const Eigen::VectorXd& x0 =
      context.get_continuous_state_vector().CopyToVector();
  const auto& u0 = system.EvalEigenVectorInput(context, 0);

  // Return the affine controller: u = u0 - K(x-x0).
  return std::make_unique<systems::AffineSystem<double>>(
      Eigen::Matrix<double, 0, 0>::Zero(),   // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // xDot0
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -lqr_result.K,                         // D
      u0 + lqr_result.K * x0);               // y0
}

}  // namespace systems
}  // namespace drake
