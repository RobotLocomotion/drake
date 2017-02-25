#pragma once

// @file
// Template method implementations for painleve.h.
// Most users should only include that file, not this one.
// For background, see http://drake.mit.edu/cxx_inl.html.

#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/examples/painleve/painleve.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace examples {
namespace painleve {

template <typename T>
Painleve<T>::Painleve() {
  // Piecewise DAE approach needs six continuous variables.
  this->DeclareContinuousState(3, 3, 0);
  this->DeclareInputPort(systems::kVectorValued, 3);
  this->DeclareOutputPort(systems::kVectorValued, 6);
}

template <typename T>
Painleve<T>::Painleve(double dt) : dt_(dt) {
  // Verify that step size parameter is valid.
  if (dt_ <= 0.0)
    throw std::logic_error("Time stepping system must be constructed using "
                               "positive integration step.");

  // Time stepping approach requires three position variables and
  // three velocity variables.
  this->DeclareDiscreteState(6);
  this->DeclareInputPort(systems::kVectorValued, 3);
  this->DeclareDiscreteUpdatePeriodSec(dt);
  this->DeclareOutputPort(systems::kVectorValued, 6);
}

/// Gets the integer variable 'k' used to determine the point of contact
/// indicated by the current mode.
/// @throws std::logic_error if this is a time-stepping system (implying that
///         modes are unused).
/// @returns the value -1 to indicate the bottom of the rod (when theta = pi/2),
///          +1 to indicate the top of the rod (when theta = pi/2), or 0 to
///          indicate the mode where both endpoints of the rod are contacting
///          the halfspace.
template <class T>
int Painleve<T>::get_k(const systems::Context<T>& context) const {
  if (is_time_stepping_system())
    throw std::logic_error("'k' is not valid for time stepping systems.");
  const int k = context.template get_abstract_state<int>(1);
  DRAKE_DEMAND(std::abs(k) <= 1);
  return k;
}

// Utility method for determining the rod endpoint, given the endpoint
// k = { -1, 0, 1 }.
template <class T>
std::pair<T, T> Painleve<T>::CalcRodEndpoint(const T& x,
                                             const T& y,
                                             const int k,
                                             const T& ctheta,
                                             const T& stheta,
                                             const double half_rod_len) {
  const T cx = x + k * ctheta * half_rod_len;
  const T cy = y + k * stheta * half_rod_len;
  return std::make_pair(cx, cy);
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

/// Integrates the Painleve Paradox example forward in time using a
/// half-explicit time stepping scheme.
template <class T>
void Painleve<T>::DoCalcDiscreteVariableUpdates(
                           const systems::Context<T>& context,
                           systems::DiscreteState<T>* discrete_state) const {
  // Set ERP (error reduction parameter) and CFM (constraint force mixing)
  // to make this problem "mostly rigid" and with rapid stabilization. These
  // parameters are described in the Open Dynamics Engine user manual (see
  // http://ode.org/ode-latest-userguide.html#sec_3_8 titled "Soft constraint
  // and constraint force mixing (CFM)") as well as in a presentation by
  // Erin Catto at the 2011 Game Developers Conference (Soft Constraints:
  // Reinventing the Spring,
  // http://box2d.org/files/GDC2011/GDC2011_Catto_Erin_Soft_Constraints.pdf).
  const double erp = get_erp();
  const double cfm = get_cfm();

  // Get the necessary state variables.
  const systems::BasicVector<T>& state = *context.get_discrete_state(0);
  const auto& q = state.get_value().template segment<3>(0);
  Vector3<T> v = state.get_value().template segment<3>(3);
  const T& x = q(0);
  const T& y = q(1);
  const T& theta = q(2);

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Compute the two rod vertical endpoint locations.
  const T stheta = sin(theta);
  const T ctheta = cos(theta);
  const int k1 = -1.0;
  const int k2 = 1.0;
  const T half_rod_length = rod_length_ / 2;

  // Three generalized coordinates / velocities.
  const int ngc = 3;

  // Two contact points, corresponding to the two rod endpoints, are always
  // used, regardless of whether any part of the rod is in contact with the
  // halfspace. This practice is standard in time stepping approaches with
  // constraint stabilization. See:
  // M. Anitescu and G. Hart. A Constraint-Stabilized Time-Stepping Approach
  // for Rigid Multibody Dynamics with Joints, Contact, and Friction. Intl.
  // J. for Numerical Methods in Engr., 60(14), 2004.
  const int nc = 2;

  // The two contact points are (xep1, yep1) and (xep2, yep2).
  const T xep1 = x + k1 * ctheta * half_rod_length;
  const T yep1 = y + k1 * stheta * half_rod_length;
  const T xep2 = x + k2 * ctheta * half_rod_length;
  const T yep2 = y + k2 * stheta * half_rod_length;

  // Total number of friction directions = number of friction directions
  // per contact * number of contacts. Because this problem is two dimensional,
  // no polygonalization of a friction cone is necessary. However, the LCP
  // variables can only assume positive values, so the negation of the tangent
  // direction permits obtaining the same effect.
  const int nk = 2 * nc;

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Construct the inverse generalized inertia matrix computed about the
  // center of mass of the rod and expressed in the world frame.
  Matrix3<T> iM;
  iM << 1.0/mass_, 0,         0,
        0,         1.0/mass_, 0,
        0,         0,         1.0/J_;

  // Update the generalized velocity vector with discretized external forces
  // (expressed in the world frame).
  const Vector3<T> fgrav(0, mass_ * get_gravitational_acceleration(), 0);
  const Vector3<T> fapplied = input.segment(0, 3);
  v += dt_ * iM * (fgrav + fapplied);

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  Eigen::Matrix<T, nc, ngc> N, F;
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (xep1 - x);
  N(1, 2) = (xep2 - x);
  F(0, 0) = F(1, 0) = 1;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -(yep1 - y);
  F(1, 2) = -(yep2 - y);

  // Construct a matrix similar to E in Anitscu and Potra 1997. This matrix
  // will yield mu*fN - E*fF = 0, or, equivalently:
  // mu*fN₁ - fF₁⁺ - fF₁⁻ ≥ 0
  // mu*fN₂ - fF₂⁺ - fF₂⁻ ≥ 0
  Eigen::Matrix<T, nk, nc> E;
  E.col(0) << 1, 0, 1, 0;
  E.col(1) << 0, 1, 0, 1;

  // Construct the LCP matrix. First do the "normal contact direction" rows.
  Eigen::Matrix<T, 8, 8> MM;
  MM.template block<2, 2>(0, 0) = N * iM * N.transpose();
  MM.template block<2, 2>(0, 2) = N * iM * F.transpose();
  MM.template block<2, 2>(0, 4) = -MM.template block<2, 2>(0, 2);
  MM.template block<2, 2>(0, 6).setZero();

  // Now construct the un-negated tangent contact direction rows (everything
  // but last block column).
  MM.template block<2, 2>(2, 0) = F * iM * N.transpose();
  MM.template block<2, 2>(2, 2) = F * iM * F.transpose();
  MM.template block<2, 2>(2, 4) = -MM.template block<2, 2>(2, 2);

  // Now construct the negated tangent contact direction rows (everything but
  // last block column). These negated tangent contact directions allow the
  // LCP to compute forces applied along the negative x-axis.
  MM.template block<2, 6>(4, 0) = -MM.template block<2, 6>(2, 0);

  // Construct the last block column for the last set of rows (see Anitescu and
  // Potra, 1997).
  MM.template block<4, 2>(2, 6) = E;

  // Construct the last two rows, which provide the friction "cone" constraint.
  MM.template block<2, 2>(6, 0) = Matrix2<T>::Identity() * get_mu_coulomb();
  MM.template block<2, 4>(6, 2) = -E.transpose();
  MM.template block<2, 2>(6, 6).setZero();

  // Construct the LCP vector.
  Eigen::Matrix<T, 8, 1> qq(8);
  qq.segment(0, 2) = N * v;
  qq(0) += erp * yep1/dt_;
  qq(1) += erp * yep2/dt_;
  qq.segment(2, 2) = F * v;
  qq.segment(4, 2) = -qq.segment(2, 2);
  qq.template segment(6, 2).setZero();

  // Regularize the LCP matrix: this is essentially Tikhonov Regularization.
  // Cottle et al. show that any linear complementarity problem is solvable
  // for sufficiently large cfm.
  // R. Cottle, J.-S. Pang, and R. Stone. The Linear Complementarity Problem.
  // Academic Press, 1992.
  MM += Eigen::Matrix<T, 8, 8>::Identity() * cfm;

  // Solve the LCP.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz);
  DRAKE_DEMAND(success);

  // Obtain the normal and frictional contact forces.
  VectorX<T> fN = zz.segment(0, 2);
  VectorX<T> fF_pos = zz.segment(2, 2);
  VectorX<T> fF_neg = zz.segment(4, 2);

  // Compute the new velocity. Note that external forces have already been
  // incorporated into v.
  VectorX<T> vplus = v + iM * (N.transpose()*fN + F.transpose()*fF_pos -
                               F.transpose()*fF_neg);

  // Compute the new position using explicit Euler integration.
  VectorX<T> qplus = q + vplus*dt_;

  // Set the new discrete state.
  systems::BasicVector<T>* new_state = discrete_state->
      get_mutable_discrete_state(0);
  new_state->get_mutable_value().segment(0, 3) = qplus;
  new_state->get_mutable_value().segment(3, 3) = vplus;
}

/// Models impact using an inelastic impact model with friction.
template <typename T>
void Painleve<T>::HandleImpact(const systems::Context<T>& context,
                               systems::State<T>* new_state) const {
  using std::abs;

  // This method is only used for piecewise DAE integration. The time
  // stepping method implicitly incorporates impact into its model.
  DRAKE_DEMAND(!is_time_stepping_system());

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Get the continuous state vector.
  systems::VectorBase<T>* new_statev = new_state->
      get_mutable_continuous_state()->get_mutable_vector();

  // Positional aspects of state do not change.
  new_statev->SetAtIndex(0, x);
  new_statev->SetAtIndex(1, y);
  new_statev->SetAtIndex(2, theta);

  // Copy abstract variables (mode and contact point) by default. Mode
  // may change depending on impact outcome.
  new_state->get_mutable_abstract_state()->CopyFrom(
      *context.get_abstract_state());

  // If there is no impact, quit now.
  if (!IsImpacting(context)) {
    new_statev->SetAtIndex(3, xdot);
    new_statev->SetAtIndex(4, ydot);
    new_statev->SetAtIndex(5, thetadot);
    return;
  }

  // TODO(edrumwri): Handle two-point impact.
  DRAKE_DEMAND(abs(sin(theta)) >= std::numeric_limits<T>::epsilon());

  // The two points of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2,
  // or, y + k*sin(theta)*r/2, where k = +/-1.

  // Determine the point of contact (cx,cy).
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const double half_rod_length = rod_length_ / 2;
  const std::pair<T, T> c = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                                 half_rod_length);
  const T cx = c.first;
  const T cy = c.second;

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

    // Set the mode to sliding.
    new_state->template get_mutable_abstract_state<Mode>(0) =
      Painleve<T>::kSlidingSingleContact;
  } else {
    // Set the mode to sticking.
    new_state->template get_mutable_abstract_state<Mode>(0) =
      Painleve<T>::kStickingSingleContact;
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
// cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
// Solve[{mass*delta_xdot == fF,
//        mass*delta_ydot == fN,
//        J*delta_thetadot == (cx[t] - x)*fN - (cy - y)*fF,
//        0 == (D[y[t], t] + delta_ydot) +
//              k*(r/2) *Cos[theta[t]]*(D[theta[t], t] + delta_thetadot),
//        fF == mu*fN *-sgn_cxdot},
//       {delta_xdot, delta_ydot, delta_thetadot, fN, fF}]
// where theta is the counter-clockwise angle the rod makes with the x-axis,
// fN and fF are contact normal and frictional forces;  delta_xdot,
// delta_ydot, and delta_thetadot represent the changes in velocity,
// r is the length of the rod, sgn_xdot is the sign of the tangent
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
  // and r is designated as the rod length. Thus, the heights of
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
  const double r = rod_length_;

  // Compute the impulses.
  const T cxdot = xdot - k * stheta * half_rod_length * thetadot;
  const int sgn_cxdot = (cxdot > 0) ? 1 : -1;
  const T fN = (J * mass * (-(r * k * ctheta * thetadot) / 2 - ydot)) /
      (J + (r * k * mass * mu * (-y + cy) * ctheta * sgn_cxdot) / 2 -
          (r * k * mass * ctheta * (x - (r * k * ctheta) / 2 - x)) / 2);
  const T fF = -sgn_cxdot * mu * fN;

  // Verify normal force is non-negative.
  DRAKE_DEMAND(fN >= 0);

  return Vector2<T>(fN, fF);
}

// Computes the impulses such that the velocity at the contact point is zero.
// These equations were determined by issuing the following commands in
// Mathematica:
//
// cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
// Solve[{mass*delta_xdot == fF, mass*delta_ydot == fN,
//        J*delta_thetadot == (cx[t] - x)*fN - (cy - y)*fF,
//        0 == (D[y[t], t] + delta_ydot) +
//             k*(r/2) *Cos[theta[t]]*(D[theta[t], t] + delta_thetadot),
//        0 == (D[x[t], t] + delta_xdot) +
//             k*(r/2)*-Cos[theta[t]]*(D[theta[t], t] + delta_thetadot)},
//       {delta_xdot, delta_ydot, delta_thetadot, fN, fF}]
// which computes the change in velocity and frictional (fF) and normal (fN)
// impulses necessary to bring the system to rest at the point of contact,
// 'r' is the rod length, theta is the counter-clockwise angle measured
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

  // The two points of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*l/2 and y - sin(theta)*l/2,
  // or, y + k*sin(theta)*l/2, where k = +/-1.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const double half_rod_length = rod_length_ / 2;
  const T cy = y + k * stheta * half_rod_length;

  // Compute the impulses.
  const double r = rod_length_;
  const double mass = mass_;
  const double J = J_;
  const T fN = (2 * (-(r * J * k * mass * ctheta * thetadot) +
      r * k * mass * mass * y * ctheta * xdot -
      r * k * mass * mass * cy * ctheta * xdot -
      2 * J * mass * ydot + r * k * mass * mass * y * ctheta * ydot -
      r * k * mass * mass * cy * ctheta * ydot)) /
      (4 * J - 2 * r * k * mass * x * ctheta -
          2 * r * k * mass * y * ctheta + 2 * r * k * mass * cy * ctheta +
          r * r * k * k * mass * ctheta * ctheta +
          2 * r * k * mass * ctheta * x);
  const T fF = -((mass * (-2 * r * J * k * ctheta * thetadot + 4 * J * xdot -
      2 * r * k * mass * x * ctheta * xdot +
      r * r * k * k * mass * ctheta * ctheta * xdot +
      2 * r * k * mass * ctheta * x * xdot -
      2 * r * k * mass * x * ctheta * ydot +
      r * r * k * k * mass * ctheta * ctheta * ydot +
      2 * r * k * mass * ctheta * x * ydot)) /
      (4 * J - 2 * r * k * mass * x * ctheta -
          2 * r * k * mass * y * ctheta + 2 * r * k * mass * cy * ctheta +
          r * r * k * k * mass * ctheta * ctheta +
          2 * r * k * mass * ctheta * x));

  // Verify that normal impulse is non-negative.
  DRAKE_DEMAND(fN > 0.0);

  return Vector2<T>(fN, fF);
}

// Sets the velocity derivatives for the rod, given contact forces.
template <class T>
void Painleve<T>::SetAccelerations(const systems::Context<T>& context,
                                   systems::VectorBase<T>* const f,
                                   const T& fN, const T& fF,
                                   const T& cx, const T& cy) const {
  using std::abs;

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Retrieve the external forces.
  const Vector3<T> fapplied = input.segment(0, 3);

  // Compute the derivatives
  const T xddot = (fapplied(0) + fF) / mass_;
  const T yddot = (fapplied(1) + fN) / mass_ + get_gravitational_acceleration();
  const T thetaddot = ((cx - x) * fN - (cy - y) * fF + fapplied(2)) / J_;

  // Set the derivatives.
  f->SetAtIndex(3, xddot);
  f->SetAtIndex(4, yddot);
  f->SetAtIndex(5, thetaddot);

  // Get constants for checking accelerations.
  const double mu = mu_;
  const double r = rod_length_;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = get_k(context);

  // Verify that the vertical acceleration at the point of contact is zero
  // (i.e., cyddot = 0).
  const T cyddot =
      yddot +
          r * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;
  DRAKE_DEMAND(abs(cyddot) < std::numeric_limits<double>::epsilon() * 10);

  // If the force is within the friction cone, verify that the horizontal
  // acceleration at the point of contact is zero (i.e., cxddot = 0).
  if (fN * mu > abs(fF)) {
    const T cxddot =
        xddot +
            r * k * (-stheta * thetaddot - ctheta * thetadot * thetadot) / 2;

    DRAKE_DEMAND(abs(cxddot) < std::numeric_limits<double>::epsilon() * 10);
  }
}

// Computes the contact forces for the case of zero sliding velocity, assuming
// that the tangential acceleration at the point of contact will be zero
// (i.e., cxddot = 0). This function solves for these forces.
//
// Equations were determined by issuing the following command in Mathematica:
// cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
// cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
// Solve[{0 == D[D[cy[t], t], t],
//        D[D[y[t], t], t] == (fN + fY)/mass + g,
//        D[D[x[t], t], t] == (fF + fX)/mass,
//        J*D[D[theta[t], t], t] == (cx[t]-x[t])*fN - (cy[t]-y[t])*fF + tau,
//        0 == D[D[cx[t], t], t]},
//       { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t],
//          D[D[theta[t], t], t] } ]
// where theta is the counter-clockwise angle the rod makes with the
// x-axis; fN and fF are contact normal and frictional forces; [fX fY] are
// arbitrary external forces (expressed in the world frame) applied at the
// center-of-mass of the rod; tau is an arbitrary external torque (expressed
// in the world frame) that should contain any moments due to any forces applied
// away from the center-of-mass plus any pure torques; g is the
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
  const int k = get_k(context);

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Set named Mathematica constants.
  const double mass = mass_;
  const double r = rod_length_;
  const double g = get_gravitational_acceleration();
  const double J = J_;
  const double fX = input(0);
  const double fY = input(1);
  const double tau = input(2);
  const T fN = (-(1/(8*J +
      2 * k * k * mass * r * r)) * (8 * fY * J + 8 * g * J * mass +
      fY * k * k * mass * r * r +  g * k * k * mass * mass * r * r +
      4 * k * mass * r * tau * ctheta -
      fY * k * k * mass * r * r * cos(2 * theta) -
      g * k * k * mass * mass * r * r *cos(2 * theta) +
      fX * k * k * mass * r * r * sin(2 * theta) -
      k * mass * r * (4 * J + k * k * mass * r * r)*
          stheta * thetadot * thetadot));

  const T fF = (-(1/(8 * J + 2 * k * k * mass * r * r)) *
         (8 * fX * J + fX * k * k * mass * r * r +
          fX * k * k * mass* r * r * cos(2 * theta)-
          4 * k * mass * r * tau * stheta +
          fY * k * k * mass * r * r * sin(2 * theta)+
          g * k * k * mass * mass * r * r * sin(2 * theta) -
          k * mass * r * (4 * J + k * k * mass * r * r) *
          ctheta * thetadot * thetadot));

  return Vector2<T>(fN, fF);
}

// Computes the accelerations of the rod center of mass for the case of the rod
// contacting the surface at exactly one point and with sliding velocity.
template <class T>
void Painleve<T>::CalcAccelerationsOneContactSliding(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& thetadot = state.GetAtIndex(5);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // The two endpoints of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2.
  const double half_rod_length = rod_length_ / 2;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const int k = get_k(context);

  // Determine the point of contact (cx, cy).
  const std::pair<T, T> c = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                                 half_rod_length);
  const T cx = c.first;
  const T cy = c.second;

  // Compute the horizontal velocity at the point of contact.
  const T cxdot = xdot - k * stheta * half_rod_length * thetadot;

  // Compute the normal acceleration at the point of contact (cy_ddot),
  // *assuming zero contact force*.
  T cyddot = get_gravitational_acceleration() -
      k * half_rod_length * stheta * thetadot * thetadot;

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);
  const double fX = input(0);
  const double fY = input(1);
  const double tau = input(2);

  // These equations were determined by issuing the following
  // commands in Mathematica:
  // cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
  // cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
  // Solve[{0 == D[D[cy[t], t], t],
  //        D[D[y[t], t], t] == (fN+fY)/mass + g,
  //        D[D[x[t], t], t] == (fF+fX)/mass,
  //        J*D[D[theta[t], t], t] == (cx[t]-x[t])*fN - (cy[t]-y[t])*fF + tau,
  //        fF == -sgn_cxdot*mu*fN},
  // { fN, fF, D[D[y[t], t], t], D[D[x[t], t], t], D[D[theta[t], t], t]}]
  // where theta is the counter-clockwise angle the rod makes with the
  // x-axis; 'r' is the length of the rod; fN and fF are normal and
  // frictional forces, respectively; [fX fY] are arbitrary
  // external forces (expressed in the world frame) applied at the
  // center-of-mass of the rod; tau is an arbitrary external torque (expressed
  // in the world frame) that should contain any moments due to any forces
  // applied away from the center-of-mass plus any pure torques; sgn_cxdot is
  // the sign function applied to the horizontal contact velocity; g is the
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
  const double r = rod_length_;
  const T fN = -((2 * (2 * J * (fY + g * mass) + k * mass * r * tau * ctheta -
       J * k * mass * r * stheta * thetadot * thetadot))/
       (4 * J + k * k * mass * r * r * ctheta * ctheta +
       k * k * mass * mu * r * r * ctheta * sgn_cxdot * stheta));

  // Check for inconsistent configuration.
  if (fN < 0)
    throw std::runtime_error("Inconsistent configuration detected.");

  // Now that normal force is computed, set the acceleration.
  const T fF = -sgn_cxdot * mu_ * fN;
  f->SetAtIndex(3, (fF + fX) / mass_);
  f->SetAtIndex(4, (fN + fY) / mass_ + get_gravitational_acceleration());
  f->SetAtIndex(5, ((cx - x) * fN - (cy - y) * fF + tau) / J);

  // Compute the normal acceleration at the contact point (a check).
  const T yddot = f->GetAtIndex(4);
  const T thetaddot = f->GetAtIndex(5);
  cyddot =
      yddot +
          r * k * (ctheta * thetaddot - stheta * thetadot * thetadot) / 2;

  // Lines below currently unused but are occasionally helpful for
  // debugging.
  //        const T xddot = f->GetAtIndex(3);
  //        const T cxddot = xddot + r*k*(-stheta*thetaddot -
  //                                        +ctheta*thetadot*thetadot)/2;

  // Verify that the normal acceleration is zero.
  DRAKE_DEMAND(abs(cyddot) < std::numeric_limits<double>::epsilon() * 10);
}

// Computes the accelerations of the rod center of mass for the case of the rod
// contacting the surface at exactly one point and without any sliding velocity.
template <class T>
void Painleve<T>::CalcAccelerationsOneContactNoSliding(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();
  DRAKE_ASSERT(f != nullptr);

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Get the contact point.
  const int k = get_k(context);
  const double half_rod_length = rod_length_ / 2;
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  const std::pair<T, T> c = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                                 half_rod_length);
  const T cx = c.first;
  const T cy = c.second;

  // Compute the contact forces, assuming sticking contact.
  Vector2<T> cf = CalcStickingContactForces(context);
  const T fN = cf(0);
  const T fF = cf(1);

  // Sanity check that normal force is non-negative.
  DRAKE_DEMAND(fN >= 0);

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);
  const double fX = input(0);
  const double fY = input(1);
  const double tau = input(2);

  // Recompute fF if it does not lie within the friction cone.
  // Constrain F such that it lies on the edge of the friction cone.
  const double mu = get_mu_coulomb();
  if (abs(fF) > mu * fN) {
    // Set named Mathematica constants.
    const double mass = get_rod_mass();
    const double r = get_rod_length();
    const double J = get_rod_moment_of_inertia();
    const double g = get_gravitational_acceleration();

    // Pick the solution that minimizes the tangential acceleration toward
    // obeying the principle of maximum dissipation, which states that the
    // friction forces should be those that maximize the dot product
    // between slip velocity and frictional force. This particular solution was
    // obtained by solving for zero normal acceleration with the frictional
    // force pointing either possible direction (indicated by d, meaning
    // positive x-axis and negative x-axis).
    // cx[t_] := x[t] + k*Cos[theta[t]]*(r/2)
    // cy[t_] := y[t] + k*Sin[theta[t]]*(r/2)
    // Solve[{0 == D[D[cy[t], t], t],
    //       D[D[y[t], t], t] == (N + fY)/mass + g,
    //       D[D[x[t], t], t] == (F + fX)/mass,
    // J*D[D[theta[t], t], t] == (cx[t] - x[t])*N - (cy[t] - y[t])*F + tau,
    //       F == -d*mu*N},
   // {N, F, D[D[y[t], t], t], D[D[x[t], t], t], D[D[theta[t], t], t]}]
    auto calc_force = [=](int d) {
      const T N = (-2 * (2 * J * (fY + g * mass) + k * mass * r * tau * ctheta -
                   J * k * mass * r * stheta * thetadot * thetadot)/
                   (4 * J + k * k * mass * r * r * ctheta * ctheta +
                    d* k * k * mass * mu * r * r * ctheta * stheta));
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
    DRAKE_ASSERT(fN1 > -10*std::numeric_limits<double>::epsilon());
    DRAKE_ASSERT(fN2 > -10*std::numeric_limits<double>::epsilon());

    // Calculate candidate tangential accelerations.
    auto calc_tan_accel = [=](int d, const T N, const T F) {
      const T thetaddot = ((cx - x) * N - (cy - y) * F + tau) / J;
      return (F + fX) / mass +
          r * k * (-stheta * thetaddot - ctheta * thetadot * thetadot) / 2;
    };

    // Compute two tangential acceleration candidates.
    const T cxddot1 = calc_tan_accel(+1, fN1, fF1);
    const T cxddot2 = calc_tan_accel(-1, fN2, fF2);

    // Pick the one that is smaller in magnitude.
    if (abs(cxddot1) < abs(cxddot2)) {
      SetAccelerations(context, f, fN1, fF1, cx, cy);
    } else {
      SetAccelerations(context, f, fN2, fF2, cx, cy);
    }
  } else {
    // Friction force is within the friction cone.
    SetAccelerations(context, f, fN, fF, cx, cy);
  }
}

// Computes the accelerations of the rod center of mass for the case of the rod
// contacting the surface at more than one point.
template <class T>
void Painleve<T>::CalcAccelerationsTwoContact(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& xdot = state.GetAtIndex(3);

  // Obtain the structure we need to write into.
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

  // Note: we do not consider modes here.

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
  const int k = (stheta > 0) ? -1 : (stheta < 0) ? 1 : 0;
  const T cy = y + k * stheta * half_rod_length;

  // If rod endpoint is not touching, there is no impact.
  if (cy >= std::numeric_limits<double>::epsilon())
    return false;

  // Compute the velocity at the point of contact.
  const T cydot = ydot + k * ctheta * half_rod_length * thetadot;

  // Verify that the rod is not impacting.
  return (cydot < -std::numeric_limits<double>::epsilon());
}

// Computes the accelerations of the rod center of mass for the case of
// ballistic motion.
template <typename T>
void Painleve<T>::CalcAccelerationsBallistic(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Second three derivative components are acceleration due to gravity and
  // external forces.
  f->SetAtIndex(3, input(0)/mass_);
  f->SetAtIndex(4, input(1)/mass_ + get_gravitational_acceleration());
  f->SetAtIndex(5, input(2)/J_);
}

template <typename T>
void Painleve<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::sin;
  using std::cos;
  using std::abs;

  // Don't compute any derivatives if this is the time stepping system.
  if (is_time_stepping_system()) {
    DRAKE_ASSERT(derivatives->size() == 0);
    return;
  }

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>* const f = derivatives->get_mutable_vector();

  // Get the abstract variables that determine the current system mode and
  // the endpoint in contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);

  // First three derivative components are xdot, ydot, thetadot.
  f->SetAtIndex(0, xdot);
  f->SetAtIndex(1, ydot);
  f->SetAtIndex(2, thetadot);

  // Call the proper derivative function (depending on mode type).
  switch (mode) {
    case kBallisticMotion:
      return CalcAccelerationsBallistic(context, derivatives);
    case kSlidingSingleContact:
      return CalcAccelerationsOneContactSliding(context, derivatives);
    case kStickingSingleContact:
      return CalcAccelerationsOneContactNoSliding(context, derivatives);
    case kSlidingTwoContacts:
      return CalcAccelerationsTwoContact(context, derivatives);
    case kStickingTwoContacts:
      return CalcAccelerationsTwoContact(context, derivatives);

    default:
      DRAKE_ABORT_MSG("Invalid mode detected");
  }
}

/// Allocates the abstract state (for piecewise DAE systems).
template <typename T>
std::unique_ptr<systems::AbstractState> Painleve<T>::
  AllocateAbstractState() const {
  if (!is_time_stepping_system()) {
    // Piecewise DAE approach needs two abstract variables (one mode and one
    // contact point indicator).
    std::vector<std::unique_ptr<systems::AbstractValue>> abstract_data;
    abstract_data.push_back(
        std::make_unique<systems::Value<Painleve<T>::Mode>>(
            Painleve<T>::kInvalid));

    // Indicates that the rod is in contact at both points.
    abstract_data.push_back(std::make_unique<systems::Value<int>>(0));
    return std::make_unique<systems::AbstractState>(std::move(abstract_data));
  } else {
    // Time stepping approach needs no abstract variables.
    return std::make_unique<systems::AbstractState>();
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
  if (!is_time_stepping_system()) {
    // DAE mode.
    state->get_mutable_continuous_state()->SetFromVector(x0);

    // Indicate that the rod is in the single contact sliding mode.
    state->get_mutable_abstract_state()->get_mutable_abstract_state(0).
        template GetMutableValue<Painleve<T>::Mode>() =
            Painleve<T>::kSlidingSingleContact;

    // Determine and set the point of contact.
    const double theta = x0(2);
    const int k = (std::sin(theta) > 0) ? -1 : 1;
    state->get_mutable_abstract_state()->get_mutable_abstract_state(1).
        template GetMutableValue<int>() = k;
  } else {
    state->get_mutable_discrete_state()->get_mutable_discrete_state(0)->
        SetFromVector(x0);
  }
}

}  // namespace painleve
}  // namespace examples
}  // namespace drake
