#pragma once

/// @file
/// Template method implementations for ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/examples/painleve/painleve.h"
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

/// Handles impact using an inelastic impact model with friction.
template <typename T>
void Painleve<T>::HandleImpact(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* new_state) const {
  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T x = state.GetAtIndex(0);
  const T y = state.GetAtIndex(1);
  const T theta = state.GetAtIndex(2);
  const T xdot = state.GetAtIndex(3);
  const T ydot = state.GetAtIndex(4);
  const T thetadot = state.GetAtIndex(5);

  // Get the state vector
  systems::VectorBase<T>* new_statev = new_state->get_mutable_vector();

  // Positional aspects of state do not change.
  new_statev->SetAtIndex(0, x);
  new_statev->SetAtIndex(1, y);
  new_statev->SetAtIndex(2, theta);

  // The two points of the rod are located at (x,y) + R(theta)*[0,l/2] and
  // (x,y) + R(theta)*[0,-l/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and l is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + cos(theta)*l/2 and y - cos(theta)*l/2,
  // or, y + k*cos(theta)*l/2, where k = +/-1.

  // Verify that there is an impact.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const T k = (ctheta < 0.0) ? 1.0 : -1.0;
  const T half_rod_length = rod_length_*0.5;
  const T ycdot = ydot - k*stheta*half_rod_length*thetadot;
  if (ycdot > -std::numeric_limits<double>::epsilon()) {
    new_statev->SetAtIndex(3, xdot);
    new_statev->SetAtIndex(4, ydot);
    new_statev->SetAtIndex(5, thetadot);
    return;
  }

  // Compute the horizontal velocity at the contact point.
  const T xcdot = xdot + k*rod_length_*0.5*stheta;

  // Compute the change in velocities if the point velocity at the contact
  // point is zero. These equations were determined by issuing the following
  // command in Mathematica:
  // Solve[{m*delta_xdot == F,
  //        m*delta_ydot == N,
  //        J*delta_thetadot == (l/2)*(F*stheta - N*ctheta),
  //        0 == (ydot + delta_ydot) +
  //             k*(l/2) * -Stheta*(thetadot + delta_thetadot),
  //        0 == (xdot + delta_xdot) +
  //             k*(l/2)*ctheta*(thetadot + delta_thetadot)},
  //       {delta_xdot, delta_ydot, delta_thetadot, F, N}]
  // which computes the change in velocity and frictional (F) and normal (N)
  // impulses necessary to bring the system to rest at the point of contact.
  // 'm' is the mass, 'l' is the rod length, stheta = sin(theta),
  // and ctheta = cos(theta).
  const T lsq = rod_length_*rod_length_;
  T delta_xdot = -((4*J_*xdot - ctheta*k*k*lsq*mass_*stheta*xdot -
                    ctheta*ctheta*k*k*lsq*mass_*ydot +
                    2*ctheta*J_*k*rod_length_*thetadot)/
                   (4*J_ + ctheta*k*lsq*mass_*stheta -
                    ctheta*k*k*lsq*mass_*stheta));
  T delta_ydot = -((-k*lsq*mass_*stheta*stheta*xdot - 4*J_*ydot -
                    ctheta*k*lsq*mass_*stheta*ydot +
                    2*J_*k*rod_length_*stheta*thetadot)/
                   (-4*J_ - ctheta*k*lsq*mass_*stheta +
                    ctheta*k*k*lsq*mass_*stheta));
  T delta_thetadot = -((rod_length_*(-2*mass_*stheta*xdot -
                                     2*ctheta*k*mass_*ydot -
                                 ctheta*k*rod_length_*mass_*stheta*thetadot +
                                 ctheta*k*k*rod_length_*mass_*stheta*thetadot))/
                       (-4*J_ - ctheta*k*lsq*mass_*stheta +
                        ctheta*k*k*lsq*mass_*stheta));

  T F = -((mass_ * (-4*J_*xdot +
        ctheta*k*k*lsq*mass_*stheta*xdot + ctheta*ctheta*k*k*lsq*mass_*ydot -
        2*ctheta*J_*k*rod_length_*thetadot))/(-4*J_ - ctheta*k*lsq*mass_*stheta+
        ctheta*k*k*lsq*mass_*stheta));
  T N = -((-k*lsq*mass_*mass_*stheta*stheta*xdot -
           4*J_*mass_*ydot - ctheta*k*lsq*mass_*mass_*stheta*ydot +
           2*J_*k*rod_length_*mass_*stheta*thetadot)/
          (-4*J_ - ctheta*k*lsq*mass_*stheta +
           ctheta*k*k*lsq*mass_*stheta));

/*
  T delta_xdot = -((4*J_*xdot + ctheta*k*lsq*mass_*stheta*xdot +
                    ctheta*ctheta*k*lsq*mass_*ydot +
                    2*ctheta*J_*k*rod_length_*thetadot)/
                   (2*(2*J_ + ctheta*k*lsq*mass_*stheta)));
  T delta_ydot = -((k*lsq*mass_*stheta*stheta*xdot + 4*J_*ydot +
                    ctheta*k*lsq*mass_*stheta*ydot -
                    2*J_*k*rod_length_*stheta*thetadot)/
                   (2*(2*J_ + ctheta*k*lsq*mass_*stheta)));
  T delta_thetadot = -((rod_length_*(mass_*stheta*xdot -
                                     ctheta*mass_*ydot +
                                     ctheta*k*rod_length_*mass_*
                                          stheta*thetadot))/
                       (2*J_ + ctheta*k*lsq*mass_*stheta));

  // Compute the normal and frictional force.
  T F = -((mass_ * (4*J_*xdot + ctheta*k*lsq*mass_*stheta*xdot +
                    ctheta*ctheta*k*lsq*mass_*ydot +
                    2*ctheta*J_*k*rod_length_*thetadot))/
          (2*(2*J_ + ctheta*k*lsq*mass_*stheta)));
  T N = -((k*lsq*mass_*mass_*stheta*stheta*xdot + 4*J_*mass_*ydot +
           ctheta*k*lsq*mass_*mass_*stheta*ydot -
           2*J_*k*rod_length_*mass_*stheta*thetadot)/
          (2*(2*J_ + ctheta*k*lsq*mass_*stheta)));
*/
  // Verify that N is non-negative.
  DRAKE_DEMAND(N > 0.0);

  // If F is not within the friction cone, recompute so that F is on the edge
  // of the friction cone. These equations were determined by issuing the
  // following command in Mathematica:
  // Solve[{m*delta_xdot == F,
  //        m*delta_ydot == N,
  //        J*delta_thetadot == (l/2)*(F*stheta + k*N*ctheta),
  //        0 == (ydot + delta_ydot) +
  //             k*(l/2) * -stheta*(thetadot + delta_thetadot),
  //        0 == (xdot + delta_xdot) +
  //             k*(l/2)*ctheta*(thetadot + delta_thetadot)},
  //       {delta_xdot, delta_ydot, delta_thetadot, F, N}]
  if (std::abs(F) > mu_*N) {
    const T ell = rod_length_;
    const T sgn_xcdot = (xcdot > 0.0) ? 1.0 : -1.0;
    delta_xdot = (-4*J_* xdot + ctheta*ell*k*
                  (-2*J_*thetadot + ell*k* mass_*stheta*xdot) +
                  ctheta*ctheta*lsq*k*k*mass_*ydot)/
                 (4*J_ - ctheta*lsq*(-1 + k)*k*mass_*stheta);
    delta_ydot = (-2*ell*J_*k*stheta*thetadot +
                  4*J_*ydot + lsq*k*mass_*stheta*(stheta*xdot + ctheta*ydot))/
                 (-4*J_ + ctheta*lsq*(-1 + k)*k*mass_*stheta);
    delta_thetadot = -((ell*mass_* (-2*stheta*xdot +
                                    ctheta*k*(ell*(-1 + k)*stheta*thetadot -
                                              2*ydot)))/
                      (-4*J_ + ctheta*lsq*(-1 + k)*k*mass_*stheta));
    F = -((2*J_*mass_*mu_*sgn_xcdot *
           (-2*ydot + k*rod_length_*stheta*thetadot))/
          (4*J_ + ctheta*k*lsq*mass_*stheta +
           k*lsq*mass_*mu_*stheta*stheta*sgn_xcdot));
    N = (2*J_*mass_*(-2*ydot + k*rod_length_*stheta*thetadot))/
        (4*J_ + ctheta*k*lsq*mass_*stheta +
         k*lsq*mass_*mu_*stheta*stheta*sgn_xcdot);

    // Verify new result w/in the friction cone and N is non-negative.
    DRAKE_DEMAND(N >= 0.0);
    DRAKE_DEMAND(std::abs(F) <= mu_*N);
  }

  // Verify that the new velocity is sensible.
  DRAKE_DEMAND((ydot + delta_ydot) -
                    k*stheta*half_rod_length*(thetadot + delta_thetadot)
                 > -std::numeric_limits<double>::epsilon());

  // Update the velocity.
  new_statev->SetAtIndex(3, xdot + delta_xdot);
  new_statev->SetAtIndex(4, ydot + delta_ydot);
  new_statev->SetAtIndex(5, thetadot + delta_thetadot);
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
  const T x = state.GetAtIndex(0);
  const T y = state.GetAtIndex(1);
  const T theta = state.GetAtIndex(2);
  const T xdot = state.GetAtIndex(3);
  const T ydot = state.GetAtIndex(4);
  const T thetadot = state.GetAtIndex(5);

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
  const T yc = y + k*ctheta*half_rod_length;
  const T xcdot = x - k*ctheta*half_rod_length*thetadot;

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // First three derivative components are xdot, ydot, thetadot.
  f->SetAtIndex(0, xdot);
  f->SetAtIndex(1, ydot);
  f->SetAtIndex(2, thetadot);

  // Case 1: the rod is not touching the ground (located at y=0).
  if (yc > std::numeric_limits<double>::epsilon()) {
    // Second three derivative components are simple: just add in gravitational
    // acceleration.
    f->SetAtIndex(3, T(0.));
    f->SetAtIndex(4, get_gravitational_acceleration());
    f->SetAtIndex(5, T(0.));
  } else {
    // Case 2: the rod is touching the ground (or even embedded in the ground).
    // Constraint stabilization should be used to eliminate embedding, but we
    // perform no such check in the derivative evaluation.

    // Verify that the rod is not impacting and not separating.
    const T ycdot = ydot - k*stheta*half_rod_length*thetadot;
    DRAKE_DEMAND(ycdot > -std::numeric_limits<double>::epsilon() &&
                 ycdot < std::numeric_limits<double>::epsilon());

    // We *should* ensure that the rod is not impacting at the contact, but it
    // is not clear how a good tolerance would be assigned.

    // Compute the normal acceleration at the point of contact (yc_ddot),
    // *assuming zero normal force*.
    T yc_ddot = get_gravitational_acceleration() -
                k*half_rod_length*ctheta*thetadot*thetadot;

    // If this derivative is negative, we must compute the normal force
    // necessary to set it to zero.
    if (yc_ddot < 0.0) {
      // Look for the case where the tangential velocity is zero.
      if (std::abs(xcdot) < std::numeric_limits<double>::epsilon()) {
// TODO(edrumwri): Finish this implementation.
/*
        // Solve for the case where xddot = 0. These equations were determined
        // by issuing the following command in Mathematica:

        // Sanity check that normal force is non-negative. 
        DRAKE_DEMAND(N >= 0.0);

        // Constraint fF such that it lies on the edge of the friction cone.
        if (std::abs(F) > mu_*N)
          F *= mu_*N/std::abs(F);

        // Now that normal force is computed, set the acceleration.
        f->SetAtIndex(3, F/mass_);
        f->SetAtIndex(4, N/mass_ + get_gravitational_acceleration());
        f->SetAtIndex(5, half_rod_length*(F*stheta + k*N*ctheta)/J_);
*/
      } else { 
        // These equations were determined by issuing the following
        // command in Mathematica:
        // Solve[{0 == yddot - (ell/2)*k*stheta*thetaddot +
        //             k*(ell/2)*ctheta*thetadot*thetadot,
        //        yddot == 1/m*fN + g,
        //        J*thetaddot == (ell/2)*(fF*stheta + k*fN*ctheta),
        //        fF == -sgn_xcdot*mu*fN},
        //       {yddot, fN, fF, thetaddot}]
        // where 'ell' is the length of the rod, stheta = sin(theta),
        // ctheta = cos(theta), m is the mass, fN and fF are normal and
        // frictional forces, respectively, sgn_xcdot = sgn(xcdot), g is the
        // acceleration due to gravity, and (hopefully) all other variables are
        // self-explanatory.
        const T lsq = rod_length_*rod_length_;
        const T sgn_xcdot = (xcdot > 0.0) ? 1.0 : -1.0;
        const T g = get_gravitational_acceleration();
        const T N = ((2*(2*g*J_*mass_ +
                       ctheta*rod_length_*J_*k*mass_*thetadot*thetadot))/
                      (-4*J_ + ctheta*lsq*k*mass_*stheta -
                       lsq*k*mass_*mu_*stheta*stheta*sgn_xcdot));
        if (N < 0.0)
          throw std::runtime_error("Inconsistent configuration detected.");

        // Now that normal force is computed, set the acceleration.
        const T F = -sgn_xcdot*mu_*N;
        f->SetAtIndex(3, F/mass_);
        f->SetAtIndex(4, N/mass_ + get_gravitational_acceleration());
        f->SetAtIndex(5, half_rod_length*(F*stheta + k*N*ctheta)/J_);
      }
    }
  }
}

template <typename T>
void Painleve<T>::SetDefaultState(const systems::Context<T>& context,
                                  systems::State<T>* state) const {
  using std::sqrt;
  const T half_len = get_rod_length()/2;
  const T r22 = sqrt(2)/2;
  VectorX<T> x0(6);
  x0 << half_len*r22, half_len*r22, M_PI/4.0, -1, 0, 0;  // Initial state.
  state->get_mutable_continuous_state()->SetFromVector(x0);
}

}  // namespace painleve
}  // namespace drake
