#include "drake/systems/framework/primitives/affine_system_plant.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

using std::make_unique;

template <typename T>
AffineSystemPlant<T>::AffineSystemPlant(
    const Eigen::Ref<const MatrixX<T>>& A,
    const Eigen::Ref<const MatrixX<T>>& B,
    const Eigen::Ref<const VectorX<T>>& xdot0,
    const Eigen::Ref<const MatrixX<T>>& C,
    const Eigen::Ref<const MatrixX<T>>& D,
    const Eigen::Ref<const VectorX<T>>& y0)
    : A_(A), B_(B), C_(C), D_(D), XDot0_(xdot0), Y0_(y0),
      kNumInputs(B.cols()), kNumOutputs(y0.size()), kNumStates(xdot0.size()) {
  DRAKE_ASSERT(kNumStates == A.rows());
  DRAKE_ASSERT(kNumStates == A.cols());
  DRAKE_ASSERT(kNumStates == B.rows());
  DRAKE_ASSERT(kNumStates == C.cols());
  DRAKE_ASSERT(kNumInputs == B.cols());
  DRAKE_ASSERT(kNumOutputs == C.rows());
  DRAKE_ASSERT(kNumOutputs == D.rows());

  // Declares input port.
  this->DeclareInputPort(kVectorValued, kNumInputs,
                         kContinuousSampling);

  // Declares output port for y.
  this->DeclareOutputPort(kVectorValued, kNumOutputs,
                          kContinuousSampling);
}

template <typename T>
const SystemPortDescriptor<T>& AffineSystemPlant<T>::get_input_port() const {
  return System<T>::get_input_port(0);
}

template <typename T>
const SystemPortDescriptor<T>& AffineSystemPlant<T>::get_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void AffineSystemPlant<T>::EvalOutput(const Context<T>& context,
                                      SystemOutput<T>* output) const {
  // TODO(naveenoid): Perhaps cache the output of this function.

  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Evaluates the state output port.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  auto x = dynamic_cast<const BasicVector<T>&>(
      context.get_continuous_state()->get_state()).get_value();

  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input);
  auto u = input->get_value();

  output_vector->get_mutable_value() =
      C_ * x + D_ * u + Y0_;
}

template <typename T>
void AffineSystemPlant<T>::EvalTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {

    DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  auto x = dynamic_cast<const BasicVector<T>&>(
      context.get_continuous_state()->get_state()).get_value();

  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input);
  auto u = input->get_value();

  derivatives->SetFromVector(A_ * x + B_ * u + XDot0_);
}

template <typename T>
std::unique_ptr<ContinuousState<T>> AffineSystemPlant<T>::
    AllocateContinuousState() const {
  // For a first order system.
    return make_unique<ContinuousState<T>>(
        make_unique<BasicVector<T>>(kNumStates));
}

template class DRAKE_EXPORT AffineSystemPlant<double>;
template class DRAKE_EXPORT AffineSystemPlant<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
