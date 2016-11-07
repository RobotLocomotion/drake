#include "drake/automotive/linear_car.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace automotive {

template <typename T>
LinearCar<T>::LinearCar() {
  this->DeclareInputPort(systems::kVectorValued,
                         LinearCarInputIndices::kNumCoordinates,
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued,
                          LinearCarStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);
}

template <typename T>
LinearCar<T>::~LinearCar() {}

template <typename T>
void LinearCar<T>::set_states(
    systems::Context<T>* context,
    const Eigen::Ref<const VectorX<T>>& value) const {
  systems::VectorBase<T>* state_vector =
      context->get_mutable_continuous_state_vector();
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_ASSERT(value.rows() == state_vector->size() && value.cols() == 1);
  state_vector->SetFromVector(value);
}

template <typename T>
bool LinearCar<T>::has_any_direct_feedthrough() const {
  return false;
}
  /*
template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
LinearCar<T>::AllocateContinuousState() 
    const {
  // The integrator's state is first-order; its state vector size is the
  // same as the input (and output) vector size.
  const int size = systems::System<T>::get_output_port(0).get_size();
  DRAKE_ASSERT(systems::System<T>::get_input_port(0).get_size() == size);
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<systems::BasicVector<T>>(size));
}
*/
template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
LinearCar<T>::AllocateContinuousState() const {
  auto state = std::make_unique<LinearCarState<T>>();
  // Define the initial state values.
  state->set_x(T{0.0});
  state->set_v(T{0.0});
  return std::make_unique<systems::ContinuousState<T>>(std::move(state));
}

template <typename T>
void LinearCar<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  //DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  //const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  //derivatives->SetFromVector(input->get_value());

    // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const LinearCarInput<T>* const input =
      dynamic_cast<const LinearCarInput<T>*>(this->EvalVectorInput(context, 0));
  DRAKE_ASSERT(input != nullptr);
  const LinearCarState<T>* const state =
      dynamic_cast<const LinearCarState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const derivatives_state =
      derivatives->get_mutable_vector();
  DRAKE_ASSERT(derivatives_state != nullptr);
  LinearCarState<T>* const new_derivatives =
      dynamic_cast<LinearCarState<T>*>(derivatives_state);
  DRAKE_ASSERT(new_derivatives != nullptr);

  // Declare the state derivatives (consistent with linear_car_state.cc).
  new_derivatives->set_x(state->v());
  new_derivatives->set_v(input->vdot());
}

template <typename T>
void LinearCar<T>::EvalOutput(const systems::Context<T>& context,
                               systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  systems::System<T>::GetMutableOutputVector(output, 0) =
      systems::System<T>::CopyContinuousStateVector(context);
}


// Explicitly instantiates on the most common scalar types.
template class DRAKE_EXPORT LinearCar<double>;
template class DRAKE_EXPORT LinearCar<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
