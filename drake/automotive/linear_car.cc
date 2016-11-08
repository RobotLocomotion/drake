#include "drake/automotive/linear_car.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/common/symbolic_expression.h"
#include "drake/math/autodiff_overloads.h"

namespace drake {
namespace automotive {

template <typename T>
LinearCar<T>::LinearCar() {
  this->DeclareInputPort(systems::kVectorValued,
                         1,  // Acceleration is the sole input.
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued,
                          2,  // Two outputs: x, v.
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
void LinearCar<T>::EvalOutput(const systems::Context<T>& context,
                              systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

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
  auto input = this->EvalVectorInput(context, 0);
  DRAKE_ASSERT(input != nullptr);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const derivatives_state =
      derivatives->get_mutable_vector();
  DRAKE_ASSERT(derivatives_state != nullptr);

  derivatives_state->SetAtIndex(0, context_state.GetAtIndex(1));
  derivatives_state->SetAtIndex(1, input->GetAtIndex(0));
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
LinearCar<T>::AllocateContinuousState() const {
  const int size = systems::System<T>::get_output_port(0).get_size();
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<systems::BasicVector<T>>(size));
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> LinearCar<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<systems::BasicVector<T>>(2);
}

// These instantiations must match the API documentation in
// linear_car.h.
template class DRAKE_EXPORT LinearCar<double>;
template class DRAKE_EXPORT LinearCar<drake::TaylorVarXd>;
template class DRAKE_EXPORT LinearCar<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
