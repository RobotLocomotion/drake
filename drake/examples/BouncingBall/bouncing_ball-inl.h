#pragma once

/// @file
/// Template method implementations for bouncing_ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

//#include "drake/examples/BouncingBall/ball-inl.h"
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
BouncingBall<T>::BouncingBall() {}

template <typename T>
void BouncingBall<T>::EvalBlah(const systems::Context<T>& context,
			   systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));
}

template <typename T>
T BouncingBall<T>::EvalGuard(const systems::Context<T>& context) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  
  // Evaluate the guard condition.
  //const systems::VectorBase<T>& context_x =
  //  context.get_state().continuous_state->get_state();
  //const systems::BasicVector<T>* const x =
  //  dynamic_cast<const systems::BasicVector<T>*>(&context_x);
  //DRAKE_ASSERT(x != nullptr);

  // The guard is satisfied (returns a non-positive value) when 
  // the ball's position is less than or equal to zero and its 
  // velocity is non-positive.
  //return std::numeric_limits<double>::max(-1 * (x->GetAtIndex(0)),
  //					   x->GetAtIndex(1));
}

template <typename T>
void BouncingBall<T>::PerformReset(const systems::Context<T>* context) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  systems::VectorBase<T>* const context_state =
      context->get_mutable_state();
  DRAKE_ASSERT(context_state != nullptr);
  systems::BasicVector<T>* const new_context =
    dynamic_cast<systems::BasicVector<T>*>(context_state);
  DRAKE_ASSERT(new_context != nullptr);

  // Perform the reset: map the position to itself and negate the
  // velocity and attenuate by the coefficient of restitution.
  new_context->SetAtIndex(1,-1 * cor * new_context->GetAtIndex(1));
}  

}  // namespace bouncingball
}  // namespace drake
