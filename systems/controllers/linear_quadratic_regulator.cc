#include "drake/systems/controllers/linear_quadratic_regulator.h"

#include "drake/common/drake_assert.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/math/continuous_algebraic_riccati_equation.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {

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

LinearQuadraticRegulatorResult DiscreteTimeLinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  Eigen::Index n = A.rows(), m = B.cols();
  DRAKE_DEMAND(n > 0 && m > 0);
  DRAKE_DEMAND(B.rows() == n && A.cols() == n);
  DRAKE_DEMAND(Q.rows() == n && Q.cols() == n);
  DRAKE_DEMAND(R.rows() == m && R.cols() == m);
  DRAKE_DEMAND(is_approx_equal_abstol(R, R.transpose(), 1e-10));

  LinearQuadraticRegulatorResult ret;

  ret.S = math::DiscreteAlgebraicRiccatiEquation(A, B, Q, R);

  Eigen::MatrixXd tmp = B.transpose() * ret.S * B + R;
  ret.K = tmp.llt().solve(B.transpose() * ret.S * A);

  return ret;
}

std::unique_ptr<systems::LinearSystem<double>> LinearQuadraticRegulator(
    const LinearSystem<double>& system,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N) {
  // DiscreteTimeLinearQuadraticRegulator does not support N yet.
  DRAKE_DEMAND(system.time_period() == 0.0 || N.rows() == 0);

  const int num_states = system.B().rows(), num_inputs = system.B().cols();

  LinearQuadraticRegulatorResult lqr_result = (system.time_period() == 0.0) ?
    LinearQuadraticRegulator(system.A(), system.B(), Q, R, N) :
    DiscreteTimeLinearQuadraticRegulator(system.A(), system.B(), Q, R);

  // Return the controller: u = -Kx.
  return std::make_unique<systems::LinearSystem<double>>(
      Eigen::Matrix<double, 0, 0>::Zero(),   // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -lqr_result.K,                         // D
      system.time_period());
}

std::unique_ptr<systems::AffineSystem<double>> LinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N,
    int input_port_index) {
  // TODO(russt): accept optional additional argument to return the cost-to-go
  // but note that it will be a full quadratic form (x'S2x + s1'x + s0).

  const int num_inputs = system.get_input_port(input_port_index).size();
  const int num_states = context.num_total_states();
  DRAKE_DEMAND(num_states > 0);
  // The Linearize method call below will verify that the system has either
  // continuous-time OR (only simple) discrete-time dynamics.

  // TODO(russt): Confirm behavior if Q is not PSD.

  // Use specified input and no outputs (the output dynamics are irrelevant for
  // LQR design).
  auto linear_system = Linearize(
      system, context, InputPortIndex{input_port_index},
      OutputPortSelection::kNoOutput);

  // DiscreteTimeLinearQuadraticRegulator does not support N yet.
  DRAKE_DEMAND(linear_system->time_period() == 0.0 || N.rows() == 0);

  LinearQuadraticRegulatorResult lqr_result =
      (linear_system->time_period() == 0.0)
          ? LinearQuadraticRegulator(linear_system->A(), linear_system->B(), Q,
                                     R, N)
          : DiscreteTimeLinearQuadraticRegulator(linear_system->A(),
                                                 linear_system->B(), Q, R);

  const Eigen::VectorXd& x0 =
      (linear_system->time_period() == 0.0)
          ? context.get_continuous_state_vector().CopyToVector()
          : context.get_discrete_state(0).CopyToVector();

  const auto& u0 = system.get_input_port(input_port_index).Eval(context);

  // Return the affine controller: u = u0 - K(x-x0).
  return std::make_unique<systems::AffineSystem<double>>(
      Eigen::Matrix<double, 0, 0>::Zero(),   // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // xDot0
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -lqr_result.K,                         // D
      u0 + lqr_result.K * x0,                // y0
      linear_system->time_period());
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake
