#include "drake/systems/framework/primitives/affine_system.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

#include "drake/math/autodiff.h"

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


std::unique_ptr<AffineSystem<double>> Linearize(const System<double>& system,
                                                 const Context<double>& context) {
  DRAKE_ASSERT_VOID(system.CheckValidContext(context));

  // TODO(russt): check if system is continuous time (only)
  // TODO(russt): handle the discrete time case

  DRAKE_DEMAND(system.get_num_input_ports() <= 1);
  DRAKE_DEMAND(system.get_num_output_ports() <= 1);

  int num_inputs = (system.get_num_input_ports()>0) ?  system.get_input_port(0).get_size() : 0,
      num_outputs = (system.get_num_output_ports()>0) ? system.get_output_port(0).get_size() : 0;

  // create an autodiff version of the system
  auto autodiff_system = system.template ToAutoDiffXd();

  // initialize autodiff
  auto autodiff_context = autodiff_system->CreateDefaultContext();
  autodiff_context->SetTimeStateAndParametersFrom(context);

  const auto& x0 = context.get_continuous_state_vector().CopyToVector();
  int num_states = x0.size();

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(num_inputs);
  if (num_inputs>0) {
    u0 = system.EvalVectorInput(context,0)->CopyToVector();
  }

  auto autodiff_args = math::initializeAutoDiffTuple(x0,u0);
  auto autodiff_x0 = autodiff_context->get_mutable_continuous_state_vector();
  autodiff_x0->SetFromVector(std::get<0>(autodiff_args));

  if (num_inputs>0) {
    auto input_vector = std::make_unique<BasicVector<AutoDiffXd>>(system.get_input_port(0).get_size());
    input_vector->get_mutable_value() = std::get<1>(autodiff_args);
    autodiff_context->SetInputPort(
        0, std::make_unique<FreestandingInputPort>(std::move(input_vector)));
  }

  auto autodiff_xdot = autodiff_system->AllocateTimeDerivatives();
  autodiff_system->EvalTimeDerivatives(*autodiff_context, autodiff_xdot.get());
  auto autodiff_xdot_vec = autodiff_xdot->CopyToVector();

  auto AB = math::autoDiffToGradientMatrix(autodiff_xdot_vec);
  auto A = AB.leftCols(num_states);
  auto B = AB.rightCols(num_inputs);
  auto xDot0 = autodiff_xdot_vec.value();

  return std::move(std::make_unique<AffineSystem<double>>(A,B,autodiff_xdot_vec.value(),Eigen::MatrixXd::Zero(0,A.rows()),Eigen::MatrixXd::Zero(0,B.cols()),Eigen::VectorXd::Zero(A.rows()));
}



}  // namespace systems
}  // namespace drake
