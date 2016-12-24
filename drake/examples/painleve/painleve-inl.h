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
void Painleve<T>::DoCalcOutput(const systems::Context<T>& context,
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
void Painleve<T>::HandleImpact(const systems::Context<T>& context,
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

  // The two points of the rod are located at (x,y) + R(theta)*[0,ell/2] and
  // (x,y) + R(theta)*[0,-ell/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and ell is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + sin(theta)*l/2 and y - sin(theta)*l/2,
  // or, y + k*sin(theta)*l/2, where k = +/-1.

  // Verify that there is an impact.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const T k = (ctheta < 0.0) ? 1.0 : -1.0;
  const T half_rod_length = rod_length_ * 0.5;

  // Compute the velocity at the point of contact
  const T xc = x + k * ctheta * rod_length_ / 2;
  const T yc = y + k * stheta * rod_length_ / 2;
  const T xcdot = xdot - k * stheta * rod_length_ / 2 * thetadot;
  const T ycdot = ydot + k * ctheta * rod_length_ / 2 * thetadot;

  // If the rod is touching, but separating, don't apply any impact forces.
  if (ycdot > -std::numeric_limits<double>::epsilon()) {
    new_statev->SetAtIndex(3, xdot);
    new_statev->SetAtIndex(4, ydot);
    new_statev->SetAtIndex(5, thetadot);
    return;
  }

  // Compute the change in velocities such that the velocity at the contact
  // point is zero. These equations were determined by issuing the following
  // commands in Mathematica:
  // xc[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
  // yc[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
  // Solve[{mass*delta_xdot == fF, mass*delta_ydot == fN,
  //        J*delta_thetadot == (xc[t] - x)*fN - (yc - y)*fF,
  //        0 == (D[y[t], t] + delta_ydot) +
  //             k*(ell/2) *Cos[theta[t]]*(D[theta[t], t] + delta_thetadot),
  //        0 == (D[x[t], t] + delta_xdot) +
  //             k*(ell/2)*-Cos[theta[t]]*(D[theta[t], t] + delta_thetadot)},
  //       {delta_xdot, delta_ydot, delta_thetadot, fN, fF}]
  // which computes the change in velocity and frictional (fF) and normal (fN)
  // impulses necessary to bring the system to rest at the point of contact,
  // 'ell' is the rod length, theta is the counter-clockwise angle measured
  // with respect to the x-axis; delta_xdot, delta_ydot, and delta_thetadot
  // are the changes in velocity.
  const T ell = rod_length_;
  const T mass = mass_;
  const T J = J_;
  T fN = (2 * (-(ell * J * k * mass * ctheta * thetadot) +
               ell * k * mass * mass * y * ctheta * xdot -
               ell * k * mass * mass * yc * ctheta * xdot -
               2 * J * mass * ydot + ell * k * mass * mass * y * ctheta * ydot -
               ell * k * mass * mass * yc * ctheta * ydot)) /
         (4 * J - 2 * ell * k * mass * x * ctheta -
          2 * ell * k * mass * y * ctheta + 2 * ell * k * mass * yc * ctheta +
          ell * ell * k * k * mass * ctheta * ctheta +
          2 * ell * k * mass * ctheta * x);
  T fF = -((mass * (-2 * ell * J * k * ctheta * thetadot + 4 * J * xdot -
                    2 * ell * k * mass * x * ctheta * xdot +
                    ell * ell * k * k * mass * ctheta * ctheta * xdot +
                    2 * ell * k * mass * ctheta * x * xdot -
                    2 * ell * k * mass * x * ctheta * ydot +
                    ell * ell * k * k * mass * ctheta * ctheta * ydot +
                    2 * ell * k * mass * ctheta * x * ydot)) /
           (4 * J - 2 * ell * k * mass * x * ctheta -
            2 * ell * k * mass * y * ctheta + 2 * ell * k * mass * yc * ctheta +
            ell * ell * k * k * mass * ctheta * ctheta +
            2 * ell * k * mass * ctheta * x));

  // Verify that fN is non-negative.
  DRAKE_DEMAND(fN > 0.0);

  // Compute the change in velocity.
  T delta_xdot = fF / mass;
  T delta_ydot = fN / mass;
  T delta_thetadot = ((xc - x) * fN - (yc - y) * fF) / J;

  // If F is not within the friction cone, recompute so that F is on the edge
  // of the friction cone. These equations were determined by issuing the
  // following commands in Mathematica:
  // xc[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
  // yc[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
  // Solve[{mass*delta_xdot == fF,
  //        mass*delta_ydot == fN,
  //        J*delta_thetadot == (xc[t] - x)*fN - (yc - y)*fF,
  //        0 == (D[y[t], t] + delta_ydot) +
  //              k*(ell/2) *Cos[theta[t]]*(D[theta[t], t] + delta_thetadot),
  //        fF == mu*fN *-sgn_xcdot},
  //       {delta_xdot, delta_ydot, delta_thetadot, fN, fF}]
  // where theta is the counter-clockwise angle the rod makes with the x-axis,
  // fN and fF are contact normal and frictional forces; delta_xdot,
  // delta_ydot, and delta_thetadot represent the changes in velocity,
  // ell is the length of the rod, sgn_xdot is the sign of the tangent
  // velocity (pre-impact), and (hopefully) all other variables are
  // self-explanatory.
  const T mu = mu_;
  if (std::abs(fF) > mu * fN) {
    const T ell = rod_length_;
    const T sgn_xcdot = (xcdot > 0.0) ? 1.0 : -1.0;
    const T mass = mass_;
    const T J = J_;

    // Compute the normal force.
    fN = (J * mass * (-(ell * k * ctheta * thetadot) / 2. - ydot)) /
         (J + (ell * k * mass * mu * (-y + yc) * ctheta * sgn_xcdot) / 2. -
          (ell * k * mass * ctheta * (x - (ell * k * ctheta) / 2. - x)) / 2.);

    // Compute the frictional force.
    fF = -sgn_xcdot * mu * fN;

    // Verify normal force is non-negative.
    DRAKE_DEMAND(fN >= 0.0);

    // Recompute the change in velocity.
    delta_xdot = fF / mass;
    delta_ydot = fN / mass;
    delta_thetadot = ((xc - x) * fN - (yc - y) * fF) / J;
  }

  // Verify that the new velocity is reasonable.
  DRAKE_DEMAND((ydot + delta_ydot) +
                   k * ctheta * half_rod_length * (thetadot + delta_thetadot) >
               -std::numeric_limits<double>::epsilon() * 10);

  // Update the velocity.
  new_statev->SetAtIndex(3, xdot + delta_xdot);
  new_statev->SetAtIndex(4, ydot + delta_ydot);
  new_statev->SetAtIndex(5, thetadot + delta_thetadot);
}

template <typename T>
void Painleve<T>::DoCalcTimeDerivatives(
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

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // The two points of the rod are located at (x,y) + R(theta)*[0,l/2] and
  // (x,y) + R(theta)*[0,-l/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and l is designated as the rod endpoint. Thus, the vertical positions of
  // the rod endpoints are located at y + sin(theta)*l/2 and y - sin(theta)*l/2.
  const T half_rod_length = rod_length_ * 0.5;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const T k = (stheta > 0.0) ? -1.0 : 1.0;
  const T xc = x + k * ctheta * rod_length_ / 2;
  const T yc = y + k * stheta * rod_length_ / 2;

  // Compute the velocity at the point of contact
  const T xcdot = xdot - k * stheta * rod_length_ / 2 * thetadot;
  const T ycdot = ydot + k * ctheta * rod_length_ / 2 * thetadot;

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
    DRAKE_DEMAND(ycdot > -std::numeric_limits<double>::epsilon() &&
                 ycdot < std::numeric_limits<double>::epsilon());

    // Handle the two contact case specially.
    if (std::abs(std::sin(theta)) < std::numeric_limits<T>::epsilon()) {
      // Verify that the normal velocity is zero.
      DRAKE_DEMAND(std::abs(ydot) < std::numeric_limits<T>::epsilon() &&
                   std::abs(thetadot) < std::numeric_limits<T>::epsilon());

      // Look to see whether there is sliding velocity.
      if (std::abs(xdot) < std::numeric_limits<T>::epsilon()) {
        // Set the time derivatives to "resting".
        f->SetAtIndex(3, T(0.));
        f->SetAtIndex(4, T(0.));
        f->SetAtIndex(5, T(0.));
      } else {
        // This code assumes no sliding will occur with contacts at multiple
        // points unless the system is initialized to such a condition. This
        // assumption has been neither proven nor rigorously validated.
        throw std::logic_error("Sliding detected with non-point contact.");
      }

      return;
    }

    // Compute the normal acceleration at the point of contact (yc_ddot),
    // *assuming zero contact force*.
    const T ycddot = get_gravitational_acceleration() -
                     k * half_rod_length * stheta * thetadot * thetadot;

    // If this derivative is negative, we must compute the normal force
    // necessary to set it to zero.
    if (ycddot < 0.0) {
      // Look for the case where the tangential velocity is zero.
      if (std::abs(xcdot) < std::numeric_limits<double>::epsilon()) {
        // Solve for the case where xddot = 0. I've computed these
        // equations by issuing the following command in Mathematica:
        // xc[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
        // yc[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
        // Solve[{0 == D[D[yc[t], t], t],
        //        D[D[y[t], t], t] == fN/mass + g,
        //        D[D[x[t], t], t] == fF/mass,
        //        J*D[D[theta[t], t], t] == (xc[t]-x[t])*fN - (yc[t]-y[t])*fF,
        //        0 == D[D[xc[t], t], t]},
        //       { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t],
        //          D[D[theta[t], t], t] } ]
        // where theta is the counter-clockwise angle the rod makes with the
        // x-axis, fN and fF are contact normal and frictional forces, g is the
        // acceleration due to gravity, and (hopefully) all other variables are
        // self-explanatory.
        const T mu = mu_;
        const T mass = mass_;
        const T ell = rod_length_;
        const T g = get_gravitational_acceleration();
        const T J = J_;
        const T fN =
            (mass *
             (-8 * g * J - 2 * ell * ell * g * k * k * mass * stheta * stheta +
              4 * ell * J * k * stheta * thetadot * thetadot +
              ell * ell * ell * k * k * k * mass * ctheta * ctheta * stheta *
                  thetadot * thetadot +
              ell * ell * ell * k * k * k * mass * stheta * stheta * stheta *
                  thetadot * thetadot)) /
            (2. * (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
                   ell * ell * k * k * mass * stheta * stheta));
        const T fF =
            -(2 * ell * ell * g * k * k * mass * mass * ctheta * stheta -
              4 * ell * J * k * mass * ctheta * thetadot * thetadot -
              ell * ell * ell * k * k * k * mass * mass * ctheta * ctheta *
                  ctheta * thetadot * thetadot -
              ell * ell * ell * k * k * k * mass * mass * ctheta * stheta *
                  stheta * thetadot * thetadot) /
            (2. * (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
                   ell * ell * k * k * mass * stheta * stheta));

        // Sanity check that normal force is non-negative.
        DRAKE_DEMAND(fN >= 0.0);

        // Now that normal force is computed, set the acceleration.
        f->SetAtIndex(3, fF / mass_);
        f->SetAtIndex(4, fN / mass_ + get_gravitational_acceleration());
        f->SetAtIndex(5, ((xc - x) * fN - (yc - y) * fF) / J);

        // Get yddot and xddot and compute xcddot and ycddot.
        const T xddot = f->GetAtIndex(3);
        const T yddot = f->GetAtIndex(4);
        const T thetaddot = f->GetAtIndex(5);

        // Verify that xcddot, ycddot = 0
        const T xcddot =
            xddot +
            ell * k * (-stheta * thetaddot - +ctheta * thetadot * thetadot) / 2;
        const T ycddot =
            yddot +
            ell * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;

        DRAKE_DEMAND(std::abs(xcddot) < std::numeric_limits<double>::epsilon());
        DRAKE_DEMAND(std::abs(ycddot) < std::numeric_limits<double>::epsilon());

        // Constrain F such that it lies on the edge of the friction cone.
        if (std::abs(fN) > mu * fN) {
          // Pick the solution that minimizes the tangential acceleration.
          const int d1 = 1;
          const T fN1 =
              (2 * mass *
               (-2 * g * J + ell * J * k * stheta * thetadot * thetadot)) /
              (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
               ell * ell * k * k * mass * mu * ctheta * d1 * stheta);
          const T fF1 = -d1 * mu * fN1;
          const T d2 = -1;
          const T fN2 =
              (2 * mass *
               (-2 * g * J + ell * J * k * stheta * thetadot * thetadot)) /
              (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
               ell * ell * k * k * mass * mu * ctheta * d2 * stheta);
          const T fF2 = -d2 * mu * fN2;

          // Compute two theta acceleration candidates.
          const T thetaddot1 = ((xc - x) * fN1 - (yc - y) * fF1) / J;
          const T thetaddot2 = ((xc - x) * fN2 - (yc - y) * fF2) / J;

          // Compute two tangential acceleration candidates.
          const T xcddot1 =
              fF1 / mass +
              ell * k * (-stheta * thetaddot1 - +ctheta * thetadot * thetadot) /
                  2;
          const T xcddot2 =
              fF2 / mass +
              ell * k * (-stheta * thetaddot2 - +ctheta * thetadot * thetadot) /
                  2;

          // Pick the one that is smaller in magnitude.
          if (std::abs(xcddot1) < std::abs(xcddot2)) {
            f->SetAtIndex(3, fF1 / mass_);
            f->SetAtIndex(4, fN1 / mass_ + get_gravitational_acceleration());
            f->SetAtIndex(5, ((xc - x) * fN1 - (yc - y) * fF1) / J);
          } else {
            f->SetAtIndex(3, fF2 / mass_);
            f->SetAtIndex(4, fN2 / mass_ + get_gravitational_acceleration());
            f->SetAtIndex(5, ((xc - x) * fN2 - (yc - y) * fF2) / J);
          }
        }
      } else {
        // Rod is sliding at the point of contact.
        // These equations were determined by issuing the following
        // commands in Mathematica:
        // xc[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
        // yc[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
        // Solve[{0 == D[D[yc[t], t], t],
        //        D[D[y[t], t], t] == fN/mass + g,
        //        D[D[x[t], t], t] == fF/mass,
        //        J*D[D[theta[t], t], t] == (xc[t]-x[t])*fN - (yc[t]-y[t])*fF,
        //        fF == -sgn_xcdot*mu*fN},
        // { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t], D[D[theta[t], t], t]}]
        // where theta is the counter-clockwise angle the rod makes with the
        // x-axis, 'ell' is the length of the rod, fN and fF are normal and
        // frictional forces, respectively, sgn_xcdot = sgn(xcdot), g is the
        // acceleration due to gravity, and (hopefully) all other variables are
        // self-explanatory.
        const T J = J_;
        const T mass = mass_;
        const T mu = mu_;
        const T sgn_xcdot = (xcdot > 0.0) ? 1.0 : -1.0;
        const T g = get_gravitational_acceleration();
        const T ell = rod_length_;
        T fN = (2 * mass *
                (-2 * g * J + ell * J * k * stheta * thetadot * thetadot)) /
               (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
                ell * ell * k * k * mass * mu * ctheta * sgn_xcdot * stheta);

        // Check for inconsistent configuration.
        if (fN < 0.0)
          throw std::runtime_error("Inconsistent configuration detected.");

        // Now that normal force is computed, set the acceleration.
        const T fF = -sgn_xcdot * mu_ * fN;
        f->SetAtIndex(3, fF / mass_);
        f->SetAtIndex(4, fN / mass_ + get_gravitational_acceleration());
        f->SetAtIndex(5, ((xc - x) * fN - (yc - y) * fF) / J);

        // Compute the normal acceleration at the contact point (a check).
        const T yddot = f->GetAtIndex(4);
        const T thetaddot = f->GetAtIndex(5);
        const T ycddot =
            yddot +
            ell * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;

        // Lines below currently unused but are occasionally helpful for
        // debugging.
        //        const T xddot = f->GetAtIndex(3);
        //        const T xcddot = xddot + ell*k*(-stheta*thetaddot -
        //                                        +ctheta*thetadot*thetadot)/2;

        // Verify that the normal acceleration is zero.
        DRAKE_DEMAND(std::abs(ycddot) < std::numeric_limits<T>::epsilon() * 10);
      }
    }
  }
}

template <typename T>
void Painleve<T>::SetDefaultState(const systems::Context<T>& context,
                                  systems::State<T>* state) const {
  using std::sqrt;
  const T half_len = get_rod_length() / 2;
  VectorX<T> x0(6);
  const T r22 = sqrt(2) / 2;
  x0 << half_len * r22, half_len * r22, M_PI / 4.0, -1, 0, 0;  // Initial state.
  state->get_mutable_continuous_state()->SetFromVector(x0);
}

}  // namespace painleve
}  // namespace drake
