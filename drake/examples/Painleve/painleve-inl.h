#pragma once

/// @file
/// Template method implementations for ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/Painleve/ball.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace painleve {

template <typename T>
Painleve<T>::Painleve() {
  this->DeclareContinuousState(3, 3, 0);
  this->DeclareOutputPort(systems::kVectorValued, 6);
}

template <typename T>
void Painleve<T>::EvalOutput(const systems::Context<T>& context,
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
std::pair<std::pair<T, T>, std::pair<T, T>> Painleve<T>::CalcContactPoint(const 
                                         systems::Context<T>& context) const {

}

template <typename T>
void Painleve<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  using std::sin;
  using std::cos;

  // Get the necessary parts of the state. 
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T x = state->GetAtIndex(0);
  const T y = state->GetAtIndex(1);
  const T theta = state->GetAtIndex(2);
  const T x_dot = state->GetAtIndex(3);
  const T y_dot = state->GetAtIndex(4);
  const T theta_dot = state->GetAtIndex(5);

  // The two points of the rod are located at (x,y) + R(theta)*[0,l/2] and
  // (x,y) + R(theta)*[0,-l/2], where 
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and l is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + cos(theta)*l/2 and y - cos(theta)*l/2.
  const T half_rod_length = rod_length_*0.5;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);

  // Determine which point is lower and use that to set a constant multiplier. 
  const T k = (ctheta < 0.0) ? 1.0 : -1.0;
  
  const T xc = x - k*stheta*half_rod_length;
  const T yc = y + k*ctheta*half_rod_length;
  const T xc_dot = x - k*ctheta*half_rod_length*theta_dot;
  const T yc_dot = y - k*stheta*half_rod_length*theta_dot;

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(new_derivatives != nullptr);

  // First three derivative components are x_dot, y_dot, theta_dot.
  f->SetAtIndex(0, x_dot);
  f->SetAtIndex(1, y_dot);
  f->SetAtIndex(2, theta_dot);

  // Case 1: the rod is not touching the ground (located at y=0).
  if (yc > 0.0) {
    // Second three derivative components are simple: just add in gravitational
    // acceleration.
    f->SetAtIndex(3, T(0.));
    f->SetAtIndex(4, get_gravitational_acceleration());
    f->SetAtIndex(5, T(0.));
  } else {
    // Case 2: the rod is touching the ground (or even embedded in the ground).
    // Constraint stabilization should be used to eliminate embedding, but we
    // perform no such check in the derivative evaluation.

    // We *should* ensure that the rod is not impacting at the contact, but it
    // is not clear how a good tolerance would be assigned.

    // Compute the normal acceleration at the point of contact (yc_ddot),
    // *assuming zero normal force*.
    T yc_ddot = k*half_rod_length*ctheta*theta_dot*theta_dot +
                get_gravitational_acceleration();

    // If this derivative is negative, we must compute the normal force
    // necessary to set it to zero.
    if (yc_ddot < 0.0) { 
      T N = -yc_ddot / (1./mass_ - sqr(half_rod_length_)/rod_inertia_*
                        k*ctheta*(mu_*stheta - ctheta));
      DRAKE_ASSERT(N > 0.0);

      // Now that normal force is computed, set the acceleration.
      f->SetAtIndex(3, mu_*N/mass_);
      f->SetAtIndex(4, N/mass_ + get_gravitational_acceleration());
      f->SetAtIndex(5, half_rod_length_*(mu_*N*stheta - N*ctheta)/rod_inertia_);
    } 
  }
}

template <typename T>
void Painleve<T>::SetDefaultState(systems::Context<T>* context) const {
  Vector2<T> x0;
  x0 << 10.0, 0.0;  // initial state values.
  context->get_mutable_continuous_state_vector()->SetFromVector(x0);
}

}  // namespace painleve
}  // namespace drake
