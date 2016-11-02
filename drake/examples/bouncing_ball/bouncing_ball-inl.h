#pragma once

/// @file
/// Template method implementations for bouncing_ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/bouncing_ball/bouncing_ball.h"

#include <algorithm>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace bouncing_ball {

template <typename T>
BouncingBall<T>::BouncingBall() {
}

template <typename T>
T BouncingBall<T>::EvalGuard(const systems::Context<T>& context) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Evaluate the guard function.
  const systems::VectorBase<T>& state =
    context.get_continuous_state_vector();

  // The guard is satisfied (returns a non-positive value) when
  // the ball's position is less than or equal to zero and its
  // velocity is non-positive.
  return std::max(state.GetAtIndex(0), state.GetAtIndex(1));
}

template <typename T>
void BouncingBall<T>::PerformReset(systems::Context<T>* context) const {
  DRAKE_ASSERT(context != nullptr);
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(*context));

  // Define a pointer to the continuous state in the context.
  const auto result =
    context->get_mutable_continuous_state_vector();

  // Perform the reset: map the position to itself and negate the
  // velocity and attenuate by the coefficient of restitution.
  auto state = context->get_mutable_continuous_state_vector();
  result->SetAtIndex(1,
     -1.0 * this->restitution_coef_ * state->GetAtIndex(1));
}

}  // namespace bouncing_ball
}  // namespace drake
