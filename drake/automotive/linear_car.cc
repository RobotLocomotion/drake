#include "drake/automotive/linear_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_expression.h"

namespace drake {
namespace automotive {

template <typename T>
LinearCar<T>::LinearCar(const T& x_init, const T& v_init)
    : x_init_(x_init), v_init_(v_init) {
  this->DeclareInputPort(systems::kVectorValued,
                         1,  // Acceleration is the sole input.
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued,
                          2,  // Two outputs: x, v.
                          systems::kContinuousSampling);
  this->DeclareContinuousState(1,   // num_q
                               1,   // num_v
                               0);  // num_z
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
void LinearCar<T>::EvalOutput(const systems::Context<T>& context,
                              systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  this->GetMutableOutputVector(output, 0) =
      this->CopyContinuousStateVector(context);
}

template <typename T>
void LinearCar<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT(derivatives != nullptr);

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const systems::BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_ASSERT(input != nullptr);

  (*derivatives->get_mutable_generalized_position())[0] =
      context_state.GetAtIndex(1);  // x_dot
  (*derivatives->get_mutable_generalized_velocity())[0] =
      input->GetAtIndex(0);  // v_dot
}

template <typename T>
void LinearCar<T>::SetDefaultState(systems::Context<T>* context) const {
  // Obtain mutable references to the contexts for each car.
  DRAKE_DEMAND(context != nullptr);
  systems::ContinuousState<T>* state = context->get_mutable_continuous_state();
  DRAKE_DEMAND(state != nullptr);

  // Set the elements of the state vector to pre-defined values.
  (*state->get_mutable_generalized_position())[0] = x_init_;  // initial x
  (*state->get_mutable_generalized_velocity())[0] = v_init_;  // initial v
}

// These instantiations must match the API documentation in linear_car.h.
template class LinearCar<double>;
template class LinearCar<drake::TaylorVarXd>;
template class LinearCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
