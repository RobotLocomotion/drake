#pragma once

/// @file
/// Template method implementations for bouncing_ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/BouncingBall/bouncing_ball.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace bouncingball {

template <typename T>
BouncingBall<T>::BouncingBall() {
}

template <typename T>
T BouncingBall<T>::EvalGuard(const systems::Context<T>& context) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Evaluate the guard condition.
  const systems::VectorBase<T>& context_x =
    context.get_state().continuous_state->get_state();
  const systems::BasicVector<T>* const x =
    dynamic_cast<const systems::BasicVector<T>*>(&context_x);
  DRAKE_ASSERT(x != nullptr);

  // The guard is satisfied (returns a non-positive value) when
  // the ball's position is less than or equal to zero and its
  // velocity is non-positive.
  return std::max(1.0 * x->GetAtIndex(0), 1.0 * x->GetAtIndex(1));
}

template <typename T>
void BouncingBall<T>::PerformReset(systems::Context<T>* context) const {
  DRAKE_ASSERT(context != nullptr);

  // Perform the reset: map the position to itself and negate the
  // velocity and attenuate by the coefficient of restitution.
  context->get_mutable_state()->continuous_state->get_mutable_state()->
    SetAtIndex(1,-1.0 * this->cor * context->get_mutable_state()->
	       continuous_state->get_mutable_state()->GetAtIndex(1));
}

}  // namespace bouncingball
}  // namespace drake
