#include "drake/systems/controllers/linear_optimal_control.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/primitives/linear_system.h"
#include "drake/util/drakeUtil.h"

namespace drake {
namespace systems {

std::unique_ptr<systems::AffineSystem<double>> TimeInvariantLqr(
    const System<double>& system, const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R) {
  // TODO(russt): accept optional additional argument to return the cost-to-go
  // but note that it will be a full quadratic form (x'S2x + s1'x + s0).

  DRAKE_DEMAND(system.get_num_input_ports() == 1);
  const int num_inputs = system.get_input_port(0).get_size(),
            num_states = context.get_continuous_state()->size();
  DRAKE_DEMAND(num_states > 0);
  DRAKE_DEMAND(Q.rows() == num_states && Q.cols() == num_states);
  DRAKE_DEMAND(R.rows() == num_inputs && R.cols() == num_inputs);
  DRAKE_ASSERT(
      CompareMatrices(Q, Q.transpose(), 1e-10, MatrixCompareType::absolute));
  DRAKE_ASSERT(
      CompareMatrices(R, R.transpose(), 1e-10, MatrixCompareType::absolute));
  // Note: the lqr method will throw an exception of Q is not PSD or R is not
  // PD.

  auto linear_system = Linearize(system, context);

  Eigen::MatrixXd K(num_inputs, num_states), S(num_states, num_states);
  lqr(linear_system->A(), linear_system->B(), Q, R, K, S);

  const Eigen::VectorXd x0 =
      context.get_continuous_state_vector().CopyToVector();
  auto u0 = system.EvalEigenVectorInput(context, 0);

  // Return the affine controller: u = u0 - K(x-x0).
  return std::make_unique<systems::AffineSystem<double>>(
      Eigen::MatrixXd::Zero(0, 0),           // A
      Eigen::MatrixXd::Zero(0, num_states),  // B
      Eigen::VectorXd::Zero(0),              // xDot0
      Eigen::MatrixXd::Zero(num_inputs, 0),  // C
      -K,                                    // D
      u0 + K * x0);                          // y0
}

}  // namespace systems
}  // namespace drake
