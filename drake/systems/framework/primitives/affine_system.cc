#include "drake/systems/framework/primitives/affine_system.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

using std::make_unique;

template <typename T>
AffineSystem<T>::AffineSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::VectorXd>& xDot0,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              const Eigen::Ref<const Eigen::VectorXd>& y0)
    : A_(A),
      B_(B),
      xDot0_(xDot0),
      C_(C),
      D_(D),
      y0_(y0),
      num_inputs_(D.cols()),
      num_outputs_(D.rows()),
      num_states_(xDot0.size()) {
  DRAKE_DEMAND(num_states_ == A.rows());
  DRAKE_DEMAND(num_states_ == A.cols());
  DRAKE_DEMAND(num_states_ == B.rows());
  DRAKE_DEMAND(num_states_ == C.cols());
  DRAKE_DEMAND(num_inputs_ == B.cols());
  DRAKE_DEMAND(num_inputs_ == D.cols());
  DRAKE_DEMAND(num_outputs_ == C.rows());
  DRAKE_DEMAND(num_outputs_ == D.rows());

  // Declares input port for u.
  this->DeclareInputPort(kVectorValued, num_inputs_, kContinuousSampling);

  // Declares output port for y.
  this->DeclareOutputPort(kVectorValued, num_outputs_, kContinuousSampling);

  // Declares the number of continuous state variables. This is needed for
  // EvalTimeDerivaties() to work.
  this->DeclareContinuousState(num_states_);
}

// setup equivalent system with a different scalar type
template <typename T>
AffineSystem<AutoDiffXd>* AffineSystem<T>::DoToAutoDiffXd() const {
  return new AffineSystem<AutoDiffXd>(A_, B_, xDot0_, C_, D_, y0_);
}

template <typename T>
const SystemPortDescriptor<T>& AffineSystem<T>::get_input_port() const {
  return System<T>::get_input_port(0);
}

template <typename T>
const SystemPortDescriptor<T>& AffineSystem<T>::get_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void AffineSystem<T>::EvalOutput(const Context<T>& context,
                                 SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Evaluates the state output port.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input);
  auto u = input->get_value();

  output_vector->get_mutable_value() = C_ * x + D_ * u + y0_;
}

template <typename T>
void AffineSystem<T>::EvalTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input);
  auto u = input->get_value();

  derivatives->SetFromVector(A_ * x + B_ * u + xDot0_);
}

template class DRAKE_EXPORT AffineSystem<double>;
template class DRAKE_EXPORT AffineSystem<AutoDiffXd>;

std::unique_ptr<AffineSystem<double>> Linearize(
    const System<double>& system, const Context<double>& context) {
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

  // create an autodiff version of the system
  std::unique_ptr<System<AutoDiffXd>> autodiff_system =
      drake::systems::System<double>::ToAutoDiffXd(system);

  // initialize autodiff
  std::unique_ptr<Context<AutoDiffXd>> autodiff_context =
      autodiff_system->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(context);

  const Eigen::VectorXd& x0 =
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

  Eigen::MatrixXd AB = math::autoDiffToGradientMatrix(autodiff_xdot_vec);
  Eigen::MatrixXd A = AB.leftCols(num_states);
  Eigen::MatrixXd B = AB.rightCols(num_inputs);
  Eigen::VectorXd xDot0 = math::autoDiffToValueMatrix(autodiff_xdot_vec);

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_outputs, num_states);
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(num_outputs, num_inputs);
  Eigen::VectorXd y0 = Eigen::VectorXd::Zero(num_outputs);

  if (num_outputs > 0) {
    std::unique_ptr<SystemOutput<AutoDiffXd>> autodiff_y0 =
        autodiff_system->AllocateOutput(*autodiff_context);
    autodiff_system->EvalOutput(*autodiff_context, autodiff_y0.get());
    auto autodiff_y0_vec = autodiff_y0->get_vector_data(0)->CopyToVector();

    Eigen::MatrixXd CD = math::autoDiffToGradientMatrix(autodiff_y0_vec);
    C = CD.leftCols(num_states);
    D = CD.rightCols(num_inputs);
    y0 = math::autoDiffToValueMatrix(autodiff_y0_vec);
  }

  return std::make_unique<AffineSystem<double>>(A, B, xDot0, C, D, y0);
}

}  // namespace systems
}  // namespace drake
