#pragma once

// @file
// Template method implementations for painleve.h.
// Most users should only include that file, not this one.
// For background, see http://drake.mit.edu/cxx_inl.html.

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
  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // Output port value is just the continuous state.
  output_vector->get_mutable_value() =
      context.get_continuous_state()->CopyToVector();
}

template <typename T>
void Painleve<T>::HandleImpact(const systems::Context<T>& context,
                               systems::ContinuousState<T>* new_state) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Get the state vector.
  systems::VectorBase<T>* new_statev = new_state->get_mutable_vector();

  // Positional aspects of state do not change.
  new_statev->SetAtIndex(0, x);
  new_statev->SetAtIndex(1, y);
  new_statev->SetAtIndex(2, theta);

  // If there is no impact, quit now.
  if (!IsImpacting(context)) {
    new_statev->SetAtIndex(3, xdot);
    new_statev->SetAtIndex(4, ydot);
    new_statev->SetAtIndex(5, thetadot);
    return;
  }

  // The two points of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod endpoint. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2,
  // or, y + k*sin(theta)*r/2, where k = +/-1.

  // Determine the point of contact (cx,cy).
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const double half_rod_length = rod_length_ / 2;
  const T cx = x + k * ctheta * half_rod_length;
  const T cy = y + k * stheta * half_rod_length;

  // Compute the impulses such that the tangential velocity post-collision
  // is zero.
  const Vector2<T> f_sticking = CalcStickingImpactImpulse(context);
  T fN = f_sticking(0);
  T fF = f_sticking(1);

  // See whether it is necessary to recompute the impulses (because the impulse
  // lies outside of the friction cone). In that case, the rod will have
  // non-zero sliding velocity post-impact (i.e., it will be sliding).
  if (mu_*fN < abs(fF)) {
    const Vector2<T> f_sliding = CalcFConeImpactImpulse(context);
    fN = f_sliding(0);
    fF = f_sliding(1);
  }

  // Compute the change in linear velocity.
  const T delta_xdot = fF / mass_;
  const T delta_ydot = fN / mass_;

  // Change in thetadot is equivalent to the third component of:
  // | cx - x |    | fF |
  // | cy - y | ×  | fN | = (cx - x) * fN - (cy - y) * fF)
  // | 0      |    | 0  |
  // divided by the moment of inertia.
  const T delta_thetadot = ((cx - x) * fN - (cy - y) * fF) / J_;

  // Verify that the post-impact velocity is non-negative (to allowable floating
  // point error).
  DRAKE_DEMAND((ydot + delta_ydot) +
                   k * ctheta * half_rod_length * (thetadot + delta_thetadot) >
               -std::numeric_limits<double>::epsilon() * 10);

  // Update the velocity.
  new_statev->SetAtIndex(3, xdot + delta_xdot);
  new_statev->SetAtIndex(4, ydot + delta_ydot);
  new_statev->SetAtIndex(5, thetadot + delta_thetadot);
}

// Computes the impulses such that the vertical velocity at the contact point
// is zero and the frictional impulse lies exactly on the friction cone.
// These equations were determined by issuing the
// following commands in Mathematica:
//
// cx[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
// Solve[{mass*delta_xdot == fF,
//        mass*delta_ydot == fN,
//        J*delta_thetadot == (cx[t] - x)*fN - (cy - y)*fF,
//        0 == (D[y[t], t] + delta_ydot) +
//              k*(ell/2) *Cos[theta[t]]*(D[theta[t], t] + delta_thetadot),
//        fF == mu*fN *-sgn_cxdot},
//       {delta_xdot, delta_ydot, delta_thetadot, fN, fF}]
// where theta is the counter-clockwise angle the rod makes with the x-axis,
// fN and fF are contact normal and frictional forces; delta_xdot,
// delta_ydot, and delta_thetadot represent the changes in velocity,
// ell is the length of the rod, sgn_xdot is the sign of the tangent
// velocity (pre-impact), and (hopefully) all other variables are
// self-explanatory.
//
// The first two equations above are the formula
// for the location of the point of contact. The next two equations
// describe the relationship between the horizontal/vertical change in
// momenta at the center of mass of the rod and the frictional/normal
// contact impulses. The fifth equation yields the moment from
// the contact impulses. The sixth equation specifies that the post-impact
// velocity in the vertical direction be zero. The last equation corresponds
// to the relationship between normal and frictional impulses (dictated by the
// Coulomb friction model).
// @returns a Vector2, with the first element corresponding to the impulse in
//          the normal direction (positive y-axis) and the second element
//          corresponding to the impulse in the tangential direction (positive
//          x-axis).
template <class T>
Vector2<T> Painleve<T>::CalcFConeImpactImpulse(
    const systems::Context<T>& context) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T> &state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // The two points of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod endpoint. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2,
  // or, y + k*sin(theta)*r/2, where k = +/-1.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const double half_rod_length = rod_length_ / 2;
  const T cy = y + k * stheta * half_rod_length;
  const double mu = mu_;
  const double J = J_;
  const double mass = mass_;
  const double ell = rod_length_;

  // Compute the impulses.
  const T cxdot = xdot - k * stheta * half_rod_length * thetadot;
  const int sgn_cxdot = (cxdot > 0) ? 1 : -1;
  const T fN = (J * mass * (-(ell * k * ctheta * thetadot) / 2 - ydot)) /
              (J + (ell * k * mass * mu * (-y + cy) * ctheta * sgn_cxdot) / 2 -
              (ell * k * mass * ctheta * (x - (ell * k * ctheta) / 2 - x)) / 2);
  const T fF = -sgn_cxdot * mu * fN;

  // Verify normal force is non-negative.
  DRAKE_DEMAND(fN >= 0);

  return Vector2<T>(fN, fF);
}

// Computes the impulses such that the velocity at the contact point is zero.
// These equations were determined by issuing the following commands in
// Mathematica:
//
// cx[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
// Solve[{mass*delta_xdot == fF, mass*delta_ydot == fN,
//        J*delta_thetadot == (cx[t] - x)*fN - (cy - y)*fF,
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
//
// The first two equations above are the formula
// for the location of the point of contact. The next two equations
// describe the relationship between the horizontal/vertical change in
// momenta at the center of mass of the rod and the frictional/normal
// contact impulses. The fifth equation yields the moment from
// the contact impulses. The sixth and seventh equations specify that the
// post-impact velocity in the horizontal and vertical directions at the
// point of contact be zero.
// @returns a Vector2, with the first element corresponding to the impulse in
//          the normal direction (positive y-axis) and the second element
//          corresponding to the impulse in the tangential direction (positive
//          x-axis).
template <class T>
Vector2<T> Painleve<T>::CalcStickingImpactImpulse(
    const systems::Context<T>& context) const {
  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // The two points of the rod are located at (x,y) + R(theta)*[0,ell/2] and
  // (x,y) + R(theta)*[0,-ell/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and ell is designated as the rod endpoint. Thus, the heights of
  // the rod endpoints are y + sin(theta)*l/2 and y - sin(theta)*l/2,
  // or, y + k*sin(theta)*l/2, where k = +/-1.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const double half_rod_length = rod_length_ / 2;
  const T cy = y + k * stheta * half_rod_length;

  // Compute the impulses.
  const double ell = rod_length_;
  const double mass = mass_;
  const double J = J_;
  const T fN = (2 * (-(ell * J * k * mass * ctheta * thetadot) +
      ell * k * mass * mass * y * ctheta * xdot -
      ell * k * mass * mass * cy * ctheta * xdot -
      2 * J * mass * ydot + ell * k * mass * mass * y * ctheta * ydot -
      ell * k * mass * mass * cy * ctheta * ydot)) /
      (4 * J - 2 * ell * k * mass * x * ctheta -
          2 * ell * k * mass * y * ctheta + 2 * ell * k * mass * cy * ctheta +
          ell * ell * k * k * mass * ctheta * ctheta +
          2 * ell * k * mass * ctheta * x);
  const T fF = -((mass * (-2 * ell * J * k * ctheta * thetadot + 4 * J * xdot -
      2 * ell * k * mass * x * ctheta * xdot +
      ell * ell * k * k * mass * ctheta * ctheta * xdot +
      2 * ell * k * mass * ctheta * x * xdot -
      2 * ell * k * mass * x * ctheta * ydot +
      ell * ell * k * k * mass * ctheta * ctheta * ydot +
      2 * ell * k * mass * ctheta * x * ydot)) /
      (4 * J - 2 * ell * k * mass * x * ctheta -
          2 * ell * k * mass * y * ctheta + 2 * ell * k * mass * cy * ctheta +
          ell * ell * k * k * mass * ctheta * ctheta +
          2 * ell * k * mass * ctheta * x));

  // Verify that normal impulse is non-negative.
  DRAKE_DEMAND(fN > 0.0);

  return Vector2<T>(fN, fF);
}

// Sets the velocity derivatives for the rod, given contact forces.
template <class T>
void Painleve<T>::SetVelocityDerivatives(const systems::Context<T>& context,
                                         systems::VectorBase<T>* const f,
                                         T fN, T fF, T cx, T cy) const {
  using std::abs;

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Compute the derivatives
  const T xddot = fF / mass_;
  const T yddot = fN / mass_ + get_gravitational_acceleration();
  const T thetaddot = ((cx - x) * fN - (cy - y) * fF) / J_;

  // Set the derivatives.
  f->SetAtIndex(3, xddot);
  f->SetAtIndex(4, yddot);
  f->SetAtIndex(5, thetaddot);

  // Get constants for checking accelerations.
  const double mu = mu_;
  const double ell = rod_length_;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;

  // Verify that the vertical acceleration at the point of contact is zero
  // (i.e., cyddot = 0).
  const T cyddot =
      yddot +
          ell * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;
  DRAKE_DEMAND(abs(cyddot) < std::numeric_limits<double>::epsilon());

  // If the force is within the friction cone, verify that the horizontal
  // acceleration at the point of contact is zero (i.e., cxddot = 0).
  if (fN*mu > abs(fF)) {
    const T cxddot =
        xddot +
            ell * k * (-stheta * thetaddot - ctheta * thetadot * thetadot) / 2;

    DRAKE_DEMAND(abs(cxddot) < std::numeric_limits<double>::epsilon());
  }
}

// Computes the contact forces for the case of zero sliding velocity, assuming
// that the tangential acceleration at the point of contact will be zero
// (i.e., cxddot = 0). This function solves for these forces.
//
// Equations were determined by issuing the following command in Mathematica:
// cx[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
// Solve[{0 == D[D[cy[t], t], t],
//        D[D[y[t], t], t] == fN/mass + g,
//        D[D[x[t], t], t] == fF/mass,
//        J*D[D[theta[t], t], t] == (cx[t]-x[t])*fN - (cy[t]-y[t])*fF,
//        0 == D[D[cx[t], t], t]},
//       { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t],
//          D[D[theta[t], t], t] } ]
// where theta is the counter-clockwise angle the rod makes with the
// x-axis, fN and fF are contact normal and frictional forces, g is the
// acceleration due to gravity, and (hopefully) all other variables are
// self-explanatory.
//
// The first two equations above are the formula
// for the point of contact. The next equation requires that the
// vertical acceleration be zero. The fourth and fifth equations
// describe the horizontal and vertical accelerations at the center
// of mass of the rod. The sixth equation yields the moment from
// the contact forces. The last equation specifies that the horizontal
// acceleration at the point of contact be zero.
// @returns a Vector2 with the first element giving the normal force (along
//          the positive y-direction) and the second element giving the
//          tangential force (along the positive x-direction).
template <class T>
Vector2<T> Painleve<T>::CalcStickingContactForces(
    const systems::Context<T>& context) const {

  // Get necessary state variables.
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Precompute quantities that will be used repeatedly.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;

  // Set named Mathematica constants.
  const double mass = mass_;
  const double ell = rod_length_;
  const double g = get_gravitational_acceleration();
  const double J = J_;
  const T fN =
      (mass *
          (-8 * g * J - 2 * ell * ell * g * k * k * mass * stheta * stheta +
              4 * ell * J * k * stheta * thetadot * thetadot +
              ell * ell * ell * k * k * k * mass * ctheta * ctheta * stheta *
                  thetadot * thetadot +
              ell * ell * ell * k * k * k * mass * stheta * stheta * stheta *
                  thetadot * thetadot)) /
          (2 * (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
              ell * ell * k * k * mass * stheta * stheta));
  const T fF =
      -(2 * ell * ell * g * k * k * mass * mass * ctheta * stheta -
          4 * ell * J * k * mass * ctheta * thetadot * thetadot -
          ell * ell * ell * k * k * k * mass * mass * ctheta * ctheta *
              ctheta * thetadot * thetadot -
          ell * ell * ell * k * k * k * mass * mass * ctheta * stheta *
              stheta * thetadot * thetadot) /
          (2 * (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
              ell * ell * k * k * mass * stheta * stheta));

  return Vector2<T>(fN, fF);
}

// Computes the time derivatives for the case of the rod contacting the
// surface at exactly one point and without any sliding velocity.
template <class T>
void Painleve<T>::CalcTimeDerivativesOneContactNoSliding(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Compute contact point.
  const double half_rod_length = rod_length_ / 2;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const T cx = x + k * ctheta * half_rod_length;
  const T cy = y + k * stheta * half_rod_length;

  // Compute the contact forces, assuming sticking contact.
  Vector2<T> cf = CalcStickingContactForces(context);
  const T fN = cf(0);
  const T fF = cf(1);

  // Sanity check that normal force is non-negative.
  DRAKE_DEMAND(fN >= 0);

  // Recompute fF if it does not lie within the friction cone.
  // Constrain F such that it lies on the edge of the friction cone.
  const double mu = get_mu_coulomb();
  if (abs(fF) > mu * fN) {
    // TODO(edrumwri): Test this once inputs have been added to the system
    //                 in a future PR.

    // Set named Mathematica constants.
    const double mass = get_rod_mass();
    const double ell = get_rod_length();
    const double J = get_rod_moment_of_inertia();
    const double g = get_gravitational_acceleration();

    // Pick the solution that minimizes the tangential acceleration.
    // This solution was obtained by solving for zero normal acceleration
    // with the frictional force pointing either possible direction
    // (positive x-axis and negative x-axis).
    auto calc_force = [=](int d) {
      const T N =
          (2 * mass *
              (-2 * g * J + ell * J * k * stheta * thetadot * thetadot)) /
              (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
                  ell * ell * k * k * mass * mu * ctheta * d * stheta);
      const T F = -d * mu * N;
      return Vector2<T>(N, F);
    };
    Vector2<T> s1 = calc_force(+1);
    Vector2<T> s2 = calc_force(-1);

    // Get the candidate normal and tangent forces.
    const T fN1 = s1(0);
    const T fN2 = s2(0);
    const T fF1 = s1(1);
    const T fF2 = s2(1);

    // Calculate candidate tangential accelerations.
    auto calc_tan_accel = [=](int d, const T N, const T F) {
      const T thetaddot = ((cx - x) * N - (cy - y) * F) / J;
      return F / mass +
          ell * k * (-stheta * thetaddot - ctheta * thetadot * thetadot) / 2;
    };

    // Compute two tangential acceleration candidates.
    const T cxddot1 = calc_tan_accel(+1, fN1, fF1);
    const T cxddot2 = calc_tan_accel(-1, fN2, fF2);

    // Pick the one that is smaller in magnitude.
    if (abs(cxddot1) < abs(cxddot2)) {
      SetVelocityDerivatives(context, f, fN1, fF1, cx, cy);
    } else {
      SetVelocityDerivatives(context, f, fN2, fF2, cx, cy);
    }
  } else {
    // Friction force is within the friction cone.
    SetVelocityDerivatives(context, f, fN, fF, cx, cy);
  }
}

// Computes the time derivatives for the case of the rod contacting the
// surface at more than one point.
template <class T>
void Painleve<T>::CalcTimeDerivativesTwoContact(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& xdot = state.GetAtIndex(3);

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // Look to see whether there is sliding velocity.
  if (abs(xdot) < std::numeric_limits<double>::epsilon()) {
    // Set the time derivatives to "resting".
    f->SetAtIndex(3, T(0));
    f->SetAtIndex(4, T(0));
    f->SetAtIndex(5, T(0));
  } else {
    // This code assumes no sliding will occur with contacts at multiple
    // points unless the system is initialized to such a condition. This
    // assumption has been neither proven nor rigorously validated.
    throw std::logic_error("Sliding detected with non-point contact.");
  }
}

template <class T>
bool Painleve<T>::IsImpacting(const systems::Context<T>& context) const {
  using std::sin;
  using std::cos;

  // Get state data necessary to compute the point of contact.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Get the height of the lower rod endpoint.
  const double half_rod_length = rod_length_ / 2;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const T cy = y + k * stheta * half_rod_length;

  // If rod endpoint is not touching, there is no impact.
  if (cy >= std::numeric_limits<double>::epsilon())
    return false;

  // Compute the velocity at the point of contact.
  const T cydot = ydot + k * ctheta * half_rod_length * thetadot;

  // Verify that the rod is not impacting.
  return (cydot < -std::numeric_limits<double>::epsilon());
}

template <typename T>
void Painleve<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  using std::sin;
  using std::cos;
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();

  // The two endpoints of the rod are located at (x,y) + R(theta)*[0,l/2] and
  // (x,y) + R(theta)*[0,-l/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and l is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*l/2 and y - sin(theta)*l/2.
  const double half_rod_length = rod_length_ / 2;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;

  // Determine the point of contact (cx, cy).
  const T cx = x + k * ctheta * half_rod_length;
  const T cy = y + k * stheta * half_rod_length;

  // Compute the horizontal velocity at the point of contact.
  const T cxdot = xdot - k * stheta * half_rod_length * thetadot;

  // First three derivative components are xdot, ydot, thetadot.
  f->SetAtIndex(0, xdot);
  f->SetAtIndex(1, ydot);
  f->SetAtIndex(2, thetadot);

  // Case 1 (ballistic mode): the rod is not touching the ground
  // (located at y=0).
  if (cy > std::numeric_limits<double>::epsilon()) {
    // Second three derivative components are simple: just add in gravitational
    // acceleration.
    f->SetAtIndex(3, T(0));
    f->SetAtIndex(4, get_gravitational_acceleration());
    f->SetAtIndex(5, T(0));
  } else {
    // Case 2: the rod is touching the ground (or even embedded in the ground).
    // Constraint stabilization should be used to eliminate embedding, but we
    // perform no such check in the derivative evaluation.

    // Handle the case where the rod is both parallel to the halfspace and
    // contacting the halfspace (at the entire length of the rod).
    // TODO(edrumwri): Modify this two-contact point routine to account for
    //                 contacts along the entire length of the rod, assumingly
    //                 only a single coefficient of friction).
    if (abs(sin(theta)) < std::numeric_limits<double>::epsilon()) {
      CalcTimeDerivativesTwoContact(context, derivatives);
      return;
    }

    // At this point, it is known that exactly one endpoint of the rod is
    // touching the halfspace. Compute the normal acceleration at that point of
    // contact (cy_ddot), *assuming zero contact force*.
    T cyddot = get_gravitational_acceleration() -
               k * half_rod_length * stheta * thetadot * thetadot;

    // If this derivative is negative, we must compute the normal force
    // necessary to set it to zero.
    if (cyddot < 0) {
      // Look for the case where the tangential velocity is zero.
      if (abs(cxdot) < std::numeric_limits<double>::epsilon()) {
        CalcTimeDerivativesOneContactNoSliding(context, derivatives);
        return;
      } else {
        // Rod is sliding at the point of contact.
        // These equations were determined by issuing the following
        // commands in Mathematica:
        // cx[t_] := x[t] + k*Cos[theta[t]]*(ell/2)
        // cy[t_] := y[t] + k*Sin[theta[t]]*(ell/2)
        // Solve[{0 == D[D[cy[t], t], t],
        //        D[D[y[t], t], t] == fN/mass + g,
        //        D[D[x[t], t], t] == fF/mass,
        //        J*D[D[theta[t], t], t] == (cx[t]-x[t])*fN - (cy[t]-y[t])*fF,
        //        fF == -sgn_cxdot*mu*fN},
        // { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t], D[D[theta[t], t], t]}]
        // where theta is the counter-clockwise angle the rod makes with the
        // x-axis, 'ell' is the length of the rod, fN and fF are normal and
        // frictional forces, respectively, sgn_cxdot = sgn(cxdot), g is the
        // acceleration due to gravity, and (hopefully) all other variables are
        // self-explanatory. The first two equations above are the formula
        // for the point of contact. The next equation requires that the
        // vertical acceleration be zero. The fourth and fifth equations
        // describe the horizontal and vertical accelerations at the center
        // of mass of the rod. The sixth equation yields the moment from
        // the contact forces. The last equation corresponds to the relationship
        // between normal and frictional forces (dictated by the Coulomb
        // friction model).
        const double J = J_;
        const double mass = mass_;
        const double mu = mu_;
        const int sgn_cxdot = (cxdot > 0) ? 1 : -1;
        const double g = get_gravitational_acceleration();
        const double ell = rod_length_;
        const T fN = (2 * mass *
                   (-2 * g * J + ell * J * k * stheta * thetadot * thetadot)) /
                  (4 * J + ell * ell * k * k * mass * ctheta * ctheta +
                   ell * ell * k * k * mass * mu * ctheta * sgn_cxdot * stheta);

        // Check for inconsistent configuration.
        if (fN < 0)
          throw std::runtime_error("Inconsistent configuration detected.");

        // Now that normal force is computed, set the acceleration.
        const T fF = -sgn_cxdot * mu_ * fN;
        f->SetAtIndex(3, fF / mass_);
        f->SetAtIndex(4, fN / mass_ + get_gravitational_acceleration());
        f->SetAtIndex(5, ((cx - x) * fN - (cy - y) * fF) / J);

        // Compute the normal acceleration at the contact point (a check).
        const T yddot = f->GetAtIndex(4);
        const T thetaddot = f->GetAtIndex(5);
        cyddot =
            yddot +
            ell * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;

        // Lines below currently unused but are occasionally helpful for
        // debugging.
        //        const T xddot = f->GetAtIndex(3);
        //        const T cxddot = xddot + ell*k*(-stheta*thetaddot -
        //                                        +ctheta*thetadot*thetadot)/2;

        // Verify that the normal acceleration is zero.
        DRAKE_DEMAND(abs(cyddot) < std::numeric_limits<double>::epsilon() * 10);
      }
    }
  }
}

/// Sets the rod to a 45 degree angle with the halfspace and positions the rod
/// such that it and the halfspace are touching at exactly one point of contact.
template <typename T>
void Painleve<T>::SetDefaultState(const systems::Context<T>& context,
                                  systems::State<T>* state) const {
  using std::sqrt;

  // Initial state corresponds to an inconsistent configuration.
  const double half_len = get_rod_length() / 2;
  VectorX<T> x0(6);
  const double r22 = sqrt(2) / 2;
  x0 << half_len * r22, half_len * r22, M_PI / 4.0, -1, 0, 0;  // Initial state.
  state->get_mutable_continuous_state()->SetFromVector(x0);
}

}  // namespace painleve
}  // namespace drake
