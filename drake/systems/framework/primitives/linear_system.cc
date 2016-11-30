#include "drake/systems/framework/primitives/linear_system.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {

template <typename T>
LinearSystem<T>::LinearSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D)
    : AffineSystem<T>(A, B, Eigen::VectorXd::Zero(A.rows()), C, D,
                      Eigen::VectorXd::Zero(C.rows())) {}
// TODO(naveenoid): Modify constructor to accommodate 0 dimension systems;
// i.e. in initializing xDot0 and y0 with a zero matrix.
template class LinearSystem<double>;
template class LinearSystem<AutoDiffXd>;

std::unique_ptr<LinearSystem<double>> Linearize(
    const System<double>& system, const Context<double>& context,
    const double equilibrium_check_tolerance) {
  DRAKE_ASSERT_VOID(system.CheckValidContext(context));

  // TODO(russt): check if system is continuous time (only)
  // TODO(russt): handle the discrete time case

  DRAKE_DEMAND(system.get_num_input_ports() <= 1);
  DRAKE_DEMAND(system.get_num_output_ports() <= 1);

  const int num_inputs = (system.get_num_input_ports() > 0)
                             ? system.get_input_port(0).get_size()
                             : 0,
            num_outputs = (system.get_num_output_ports() > 0)
                              ? system.get_output_port(0).get_size()
                              : 0;

  // Create an autodiff version of the system.
  std::unique_ptr<System<AutoDiffXd>> autodiff_system =
      drake::systems::System<double>::ToAutoDiffXd(system);

  // Initialize autodiff.
  std::unique_ptr<Context<AutoDiffXd>> autodiff_context =
      autodiff_system->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(context);

  const Eigen::VectorXd x0 =
      context.get_continuous_state_vector().CopyToVector();
  const int num_states = x0.size();

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(num_inputs);
  if (num_inputs > 0) {
    u0 = system.EvalEigenVectorInput(context, 0);
  }

  auto autodiff_args = math::initializeAutoDiffTuple(x0, u0);
  autodiff_context->get_mutable_continuous_state_vector()->SetFromVector(
      std::get<0>(autodiff_args));

  if (num_inputs > 0) {
    auto input_vector = std::make_unique<BasicVector<AutoDiffXd>>(num_inputs);
    input_vector->SetFromVector(std::get<1>(autodiff_args));
    autodiff_context->SetInputPort(
        0, std::make_unique<FreestandingInputPort>(std::move(input_vector)));
  }

  std::unique_ptr<ContinuousState<AutoDiffXd>> autodiff_xdot =
      autodiff_system->AllocateTimeDerivatives();
  autodiff_system->EvalTimeDerivatives(*autodiff_context, autodiff_xdot.get());
  auto autodiff_xdot_vec = autodiff_xdot->CopyToVector();

  // Ensure that xdot0 = f(x0,u0) == 0.
  if (!math::autoDiffToValueMatrix(autodiff_xdot_vec)
           .isZero(equilibrium_check_tolerance)) {
    throw std::runtime_error(
        "The nominal operating point (x0,u0) is not an equilibrium point of "
        "the system.  Without additional information, a time-invariant "
        "linearization of this system is not well defined.");
  }

  Eigen::MatrixXd AB = math::autoDiffToGradientMatrix(autodiff_xdot_vec);
  Eigen::MatrixXd A = AB.leftCols(num_states);
  Eigen::MatrixXd B = AB.rightCols(num_inputs);

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_outputs, num_states);
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_outputs, num_inputs);

  if (num_outputs > 0) {
    std::unique_ptr<SystemOutput<AutoDiffXd>> autodiff_y0 =
        autodiff_system->AllocateOutput(*autodiff_context);
    autodiff_system->EvalOutput(*autodiff_context, autodiff_y0.get());
    auto autodiff_y0_vec = autodiff_y0->get_vector_data(0)->CopyToVector();

    Eigen::MatrixXd CD = math::autoDiffToGradientMatrix(autodiff_y0_vec);
    C = CD.leftCols(num_states);
    D = CD.rightCols(num_inputs);
  }

  return std::make_unique<LinearSystem<double>>(A, B, C, D);
}

}  // namespace systems
}  // namespace drake
