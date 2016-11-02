#include "drake/systems/framework/primitives/affine_system.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

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

}  // namespace systems
}  // namespace drake
