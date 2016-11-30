#pragma once

/// @file
/// Template method implementations for ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/bouncing_ball/ball.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace bouncing_ball {

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
      context.get_continuous_state()->CopyToVector();
}

template <typename T>
void Ball<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the state.
  const systems::VectorBase<T>& state =
      context.get_continuous_state_vector();

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const new_derivatives =
      derivatives->get_mutable_vector();
  DRAKE_ASSERT(new_derivatives != nullptr);

  const double g{9.81};  // gravity.

  new_derivatives->SetAtIndex(0, state.GetAtIndex(1));
  new_derivatives->SetAtIndex(1, T{-g});
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
Ball<T>::AllocateContinuousState() const {
  auto state = std::make_unique<systems::BasicVector<T>>(2);
  state->get_mutable_value() << 10, 0;   // initial state values.
  return std::make_unique<systems::ContinuousState<T>>(std::move(state),
                                                       1,  // num_q
                                                       1,  // num_v
                                                       0);  // num_z
}

}  // namespace bouncing_ball
}  // namespace drake
