#include "drake/systems/plants/affine_linear_system/affine_system_plant.h"

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
    const Eigen::Ref<const VectorX<T>>& xdot0,
    const Eigen::Ref<const MatrixX<T>>& A,
    const Eigen::Ref<const MatrixX<T>>& B,
    const Eigen::Ref<const MatrixX<T>>& C,
    const Eigen::Ref<const MatrixX<T>>& D,
    const Eigen::Ref<const VectorX<T>>& y0)
    : kA(A), kB(B), kC(C), kD(D), kXDot0(xdot0), kY0(y0),
      kNumInputs(B.cols()), kNumOutputs(y0.size()), kNumStates(xdot0.size()) {
  DRAKE_ASSERT(kNumStates == A.rows());
  DRAKE_ASSERT(kNumStates == A.cols());
  DRAKE_ASSERT(kNumStates == B.rows());
  DRAKE_ASSERT(kNumStates == C.cols());
  DRAKE_ASSERT(kNumInputs == B.cols());
  DRAKE_ASSERT(kNumOutputs == C.rows());
  DRAKE_ASSERT(kNumOutputs == D.rows());

  // Declares input port for forcing term.
  this->DeclareInputPort(kVectorValued, kNumInputs,
                         kContinuousSampling);

  // Declares output port for q, qdot, y.
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
void AffineSystemPlant<T>::set_state_vector(
    Context<T>* context, const Eigen::Ref<const VectorX<T>> x) const {
  DRAKE_ASSERT(context != nullptr);
  DRAKE_ASSERT(x.size() == kNumStates);
  context->get_mutable_continuous_state()->get_mutable_state()
      ->SetFromVector(x);
}

template <typename T>
void AffineSystemPlant<T>::EvalOutput(const MyContext& context,
                                      MyOutput* output) const {
  // TODO(naveenoid): Perhaps cache the output of this function.

  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Evaluates the state output port.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  // TODO(naveenoid): Remove this copying of state and input.

  auto x = context.get_continuous_state()->CopyToVector();
  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input);
  auto u = input->get_value();

  output_vector->get_mutable_value() =
      kC * x + kD * u + kY0;
}

template <typename T>
void AffineSystemPlant<T>::EvalTimeDerivatives(
    const MyContext& context, MyContinuousState* derivatives) const {

    DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // TODO(naveenoid): Perhaps the output of this function needs caching.
  auto x = context.get_continuous_state()->CopyToVector();

  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input);
  auto u = input->get_value();

  derivatives->SetFromVector(kA * x + kB * u + kXDot0);
}

template <typename T>
std::unique_ptr<SystemOutput<T>> AffineSystemPlant<T>::AllocateOutput(
    const MyContext &context) const {
  auto output = make_unique<LeafSystemOutput<T>>();
  // Allocates an output for the AffineSystemPlant state (output port 0).
  auto data = make_unique<BasicVector<T>>(kNumStates);
  auto port = make_unique<OutputPort>(move(data));
  output->get_mutable_ports()->push_back(move(port));

  return std::unique_ptr<SystemOutput<T>>(output.release());
}

template <typename T>
std::unique_ptr<ContinuousState<T>> AffineSystemPlant<T>::
    AllocateContinuousState() const {
  // For a first order system.
    return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicVector<T>>(kNumStates));
}

template class DRAKE_EXPORT AffineSystemPlant<double>;
template class DRAKE_EXPORT AffineSystemPlant<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
