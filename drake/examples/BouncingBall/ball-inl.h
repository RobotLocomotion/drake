#pragma once

/// @file
/// Template method implementations for ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/BouncingBall/ball.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace bouncingball {

template <typename T>
Ball<T>::Ball() {
  this->DeclareOutputPort(systems::kVectorValued, 
			  2, 
			  systems::kContinuousSampling);
}

template <typename T>
  void Ball<T>::EvalOutput(const systems::Context<T>& context,
			   systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  output_vector->get_mutable_value() =
      context.get_state().continuous_state->get_state().CopyToVector();
}

template <typename T>
void Ball<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_state().continuous_state->get_state();
  const systems::BasicVector<T>* const state =
    dynamic_cast<const systems::BasicVector<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const derivatives_state =
      derivatives->get_mutable_state();
  DRAKE_ASSERT(derivatives_state != nullptr);
  systems::BasicVector<T>* const new_derivatives =
    dynamic_cast<systems::BasicVector<T>*>(derivatives_state);
  DRAKE_ASSERT(new_derivatives != nullptr);

  const double g{9.81};  // max acceleration.

  new_derivatives->SetAtIndex(0,state->GetAtIndex(1));
  new_derivatives->SetAtIndex(1,T{-g});

 }

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
Ball<T>::AllocateContinuousState() const {
  std::unique_ptr<systems::BasicVector<T>> state(new systems::BasicVector<T>(2));
  state->get_mutable_value() << 0, 0;   // initial state values.
  return std::make_unique<systems::ContinuousState<T>>(std::move(state));
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
Ball<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<systems::BasicVector<T>>(2);
}

}  // namespace bouncingball
}  // namespace drake
