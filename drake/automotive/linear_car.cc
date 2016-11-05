#include "drake/automotive/linear_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/math/autodiff_overloads.h"
//#include "drake/systems/framework/leaf_context.h"

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
const systems::SystemPortDescriptor<T>& LinearCar<T>::get_input_port() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>& LinearCar<T>::get_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
void LinearCar<T>::EvalOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  output_vector->get_mutable_value() =
      context.get_continuous_state()->CopyToVector();
}

template <typename T>
void LinearCar<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const LinearCarInput<T>* const input =
      dynamic_cast<const LinearCarInput<T>*>(this->EvalVectorInput(context, 0));
  const LinearCarState<T>* const state =
      dynamic_cast<const LinearCarState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
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
std::unique_ptr<systems::ContinuousState<T>>
LinearCar<T>::AllocateContinuousState() const {
  auto state = std::make_unique<LinearCarState<T>>();
  // Define the initial state values.
  state->set_x(T{0.0});
  state->set_v(T{0.0});
  return std::make_unique<systems::ContinuousState<T>>(std::move(state));
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
LinearCar<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<LinearCarState<T>>();
}

// These instantiations must match the API documentation in
// idm_with_trajectory_agent.h.
template class DRAKE_EXPORT LinearCar<double>;
template class DRAKE_EXPORT LinearCar<drake::TaylorVarXd>;
template class DRAKE_EXPORT LinearCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
