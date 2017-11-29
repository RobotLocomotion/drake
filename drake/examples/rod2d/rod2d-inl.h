#pragma once

// @file
// Template method implementations for rod2d.h.
// Most users should only include that file, not this one.
// For background, see http://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/examples/rod2d/rod2d.h"
/* clang-format on */

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/value.h"

// TODO(edrumwri,sherm1) This code is currently written out mostly in scalars
// but should be done in 2D vectors instead to make it more compact, easier to
// understand, and easier to relate to our 3D code.
namespace drake {
namespace examples {
namespace rod2d {

template <typename T>
Rod2D<T>::Rod2D(SimulationType simulation_type, double dt)
    : simulation_type_(simulation_type), dt_(dt) {
  // Verify that the simulation approach is either piecewise DAE or
  // compliant ODE.
  if (simulation_type == SimulationType::kTimeStepping) {
    if (dt <= 0.0)
      throw std::logic_error(
          "Time stepping approach must be constructed using"
          " strictly positive step size.");

    // Time stepping approach requires three position variables and
    // three velocity variables, all discrete, and periodic update.
    this->DeclarePeriodicDiscreteUpdate(dt);
    this->DeclareDiscreteState(6);
  } else {
    if (dt != 0)
      throw std::logic_error(
          "Piecewise DAE and compliant approaches must be "
          "constructed using zero step size.");

    // Both piecewise DAE and compliant approach require six continuous
    // variables.
    this->DeclareContinuousState(Rod2dStateVector<T>(), 3, 3, 0);  // q, v, z
  }

  this->DeclareInputPort(systems::kVectorValued, 3);
  state_output_port_ = &this->DeclareVectorOutputPort(
      systems::BasicVector<T>(6), &Rod2D::CopyStateOut);
  pose_output_port_ = &this->DeclareVectorOutputPort(&Rod2D::CopyPoseOut);
}

// Computes the external forces on the rod.
// @returns a three dimensional vector corresponding to the forces acting at
//          the center-of-mass of the rod (first two components) and the
//          last corresponding to the torque acting on the rod (final
//          component). All forces and torques are expressed in the world
//          frame.
template <class T>
Vector3<T> Rod2D<T>::ComputeExternalForces(
    const systems::Context<T>& context) const {
  // Compute the external forces (expressed in the world frame).
  const int port_index = 0;
  const VectorX<T> input = this->EvalEigenVectorInput(context, port_index);
  const Vector3<T> fgrav(0, mass_ * get_gravitational_acceleration(), 0);
  const Vector3<T> fapplied = input.segment(0, 3);
  return fgrav + fapplied;
}

template <class T>
T Rod2D<T>::CalcSignedDistance(const systems::Context<T>& context) const {
  using std::sin;
  using std::cos;
  using std::min;

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(get_simulation_type() ==
      Rod2D<T>::SimulationType::kPiecewiseDAE);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);

  // Get the two rod endpoints.
  const T ctheta = cos(theta);
  const T stheta = sin(theta);
  int k1 = 1;
  int k2 = -1;
  const Vector2<T> ep1 = CalcRodEndpoint(x, y, k1, ctheta, stheta,
                                         get_rod_half_length());
  const Vector2<T> ep2 = CalcRodEndpoint(x, y, k2, ctheta, stheta,
                                         get_rod_half_length());

  return min(ep1[1], ep2[1]);
}

template <class T>
T Rod2D<T>::CalcEndpointDistance(const systems::Context<T>& context) const {
  using std::sin;

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(get_simulation_type() ==
      Rod2D<T>::SimulationType::kPiecewiseDAE);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T stheta = sin(theta);

  // Get the abstract variables that determine the current system mode and
  // the endpoint in contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);
  DRAKE_DEMAND(mode == Mode::kSlidingSingleContact ||
               mode == Mode::kStickingSingleContact);
  const int k = get_k(context);

  // Get the vertical position of the other rod endpoint.
  const int k2 = -k;
  return y + k2 * stheta * get_rod_half_length();
}

template <class T>
T Rod2D<T>::CalcNormalAccelWithoutContactForces(const systems::Context<T>&
                                                     context) const {
  DRAKE_ASSERT_VOID(this->CheckValidContext(context));
  using std::sin;
  using std::cos;

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(get_simulation_type() ==
               Rod2D<T>::SimulationType::kPiecewiseDAE);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& theta = state.GetAtIndex(2);
  const T& thetadot = state.GetAtIndex(5);

  // Get the abstract variables that determine the current system mode and
  // the endpoint in contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);
  DRAKE_DEMAND(mode != Mode::kBallisticMotion);
  const int k = get_k(context);
  const T stheta = sin(theta);
  const T ctheta = cos(theta);

  // Get the external force.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);
  const Vector3<T> fapplied = input.segment(0, 3);
  const T& fY = fapplied(1);
  const T& tau = fapplied(2);

  // Compute the normal acceleration at the point of contact (cyddot),
  // *assuming zero contact force*. This equation comes from the kinematics of
  // the rod:
  // cy = y + k * half_rod_length * sin(θ)  [rod endpoint vertical location]
  // dcy/dt = dy/dt + k * half_rod_length * cos(θ) * dθ/dt
  // d²cy/dt² = d²y/dt² - k * half_rod_length * sin(θ) * (dθ/dt)² +
  //            k * half_rod_length * cos(θ) * d²θ/dt²
  const T yddot = get_gravitational_acceleration() + fY/get_rod_mass();
  const T thetaddot = tau/get_rod_moment_of_inertia();
  T cyddot = yddot -
      k * half_length_ * stheta * thetadot * thetadot +
      k * half_length_ * ctheta * thetaddot;

  return cyddot;
}

template <class T>
T Rod2D<T>::CalcSlidingDot(const systems::Context<T>& context) const {
  using std::sin;

  // Verify the system is simulated using piecewise DAE.
  DRAKE_DEMAND(get_simulation_type() ==
      Rod2D<T>::SimulationType::kPiecewiseDAE);

  // Verify rod is undergoing sliding contact.
  const Mode mode = context.template get_abstract_state<Mode>(0);
  DRAKE_DEMAND(mode == Mode::kSlidingSingleContact ||
               mode == Mode::kSlidingTwoContacts);

  // Get the point of contact.
  const int k = get_k(context);

  // Get the relevant parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& thetadot = state.GetAtIndex(5);

  // Compute the velocity at the point of contact
  const T stheta = sin(theta);
  const T half_rod_length = get_rod_half_length();
  const T xcdot = xdot - k * stheta * half_rod_length * thetadot;
  return xcdot;
}

template <class T>
T Rod2D<T>::CalcStickingFrictionForceSlack(const systems::Context<T>& context)
                                               const {
  using std::abs;

  // Compute the contact forces, assuming sticking contact.
  const Vector2<T> cf = CalcStickingContactForces(context);
  const T &fN = cf(0);
  const T &fF = cf(1);

  // Compute the difference between how much force *can* be applied to effect
  // sticking and how much force needs to be applied to effect sticking.
  const double mu = get_mu_coulomb();
  return mu * fN - abs(fF);
}

template <class T>
int Rod2D<T>::DetermineNumWitnessFunctions(const systems::Context<T>&
                                             context) const {
  // No witness functions if this is not a piecewise DAE model.
  if (simulation_type_ != SimulationType::kPiecewiseDAE)
    return 0;

  // Get the abstract variable that determines the current system mode.
  Mode mode = context.template get_abstract_state<Mode>(0);

  switch (mode) {
    case Rod2D::kBallisticMotion:
      // The rod is in ballistic flight, there is just one witness function:
      // the signed distance between the rod and the half-space.
      return 1;

    case Rod2D::kSlidingSingleContact:
      // The rod is undergoing contact without impact and is sliding at a single
      // point of contact. Three witness functions are necessary: one for
      // checking whether the rod is to separate from the half-space, another
      // for checking whether the direction of sliding has changed, and a third
      // for checking for contact between the other rod endpoint and the ground.
      return 3;

    case Rod2D::kStickingSingleContact:
      // The rod is undergoing contact without impact and is sticking at a
      // single point of contact. Three witness functions are necessary: one for
      // checking whether the rod is to separate from the half-space, a
      // second for checking for contact between the other rod endpoint
      // and the ground, and a third for checking for the transition from
      // sticking to sliding.
      return 3;

    case Rod2D::kSlidingTwoContacts:
      // The rod is undergoing sliding contact without impact at two points of
      // contact. Two witness functions are necessary: one for checking
      // whether the rod is to separate from the half-space and one more to
      // check whether the rod is to transition from sliding to sticking.
      // TODO(edrumwri): Add two more witness functions- one to check separation
      //                 from half-space for second contact point and another
      //                 to check transition from sliding to sticking from
      //                 second contact point.
      return 2;

    case Rod2D::kStickingTwoContacts:
      // The rod is undergoing sticking contact without impact at two points of
      // contact. Two witness functions are necessary: one to check whether
      // the rod is to separate from the half-space and one more to check
      // whether the rod is to transition from sticking to sliding.
      // TODO(edrumwri): Add two more witness functions- one to check separation
      //                 from half-space for second contact point and another
      //                 to check transition from sticking to sliding from
      //                 second contact point.
      return 2;

    default:
      DRAKE_ABORT();
  }

  DRAKE_ABORT();
  return 0;
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
int Rod2D<T>::get_k(const systems::Context<T>& context) const {
  if (simulation_type_ != SimulationType::kPiecewiseDAE)
    throw std::logic_error("'k' is only valid for piecewise DAE approach.");
  const int k = context.template get_abstract_state<int>(1);
  DRAKE_DEMAND(std::abs(k) <= 1);
  return k;
}

template <class T>
Vector2<T> Rod2D<T>::CalcRodEndpoint(const T& x, const T& y, int k,
                                     const T& ctheta, const T& stheta,
                                     double half_rod_len) {
  const T cx = x + k * ctheta * half_rod_len;
  const T cy = y + k * stheta * half_rod_len;
  return Vector2<T>(cx, cy);
}

template <class T>
Vector2<T> Rod2D<T>::CalcCoincidentRodPointVelocity(
    const Vector2<T>& p_WRo, const Vector2<T>& v_WRo,
    const T& w_WR,  // aka thetadot
    const Vector2<T>& p_WC) {
  const Vector2<T> p_RoC_W = p_WC - p_WRo;  // Vector from R origin to C, in W.
  const Vector2<T> v_WRc = v_WRo + w_cross_r(w_WR, p_RoC_W);
  return v_WRc;
}

/// Solves MX = B for X, where M is the generalized inertia matrix of the rod
/// and is defined in the following manner:<pre>
/// M = | m 0 0 |
///     | 0 m 0 |
///     | 0 0 J | </pre>
/// where `m` is the mass of the rod and `J` is its moment of inertia.
template <class T>
MatrixX<T> Rod2D<T>::solve_inertia(const MatrixX<T>& B) const {
  const T inv_mass = 1.0 / get_rod_mass();
  const T inv_J = 1.0 / get_rod_moment_of_inertia();
  Matrix3<T> iM;
  iM << inv_mass, 0,        0,
        0,       inv_mass, 0,
        0,       0,        inv_J;
  return iM * B;
}

/// The velocity tolerance for sliding.
template <class T>
T Rod2D<T>::GetSlidingVelocityTolerance() const {
  // TODO(edrumwri): Change this coarse tolerance, which is only guaranteed to
  // be reasonable for rod locations near the world origin and rod velocities
  // of unit magnitude, to a computed tolerance that can account for these
  // assumptions being violated.
  return 100 * std::numeric_limits<double>::epsilon();
}

// Gets the time derivative of a 2D rotation matrix.
template <class T>
Matrix2<T> Rod2D<T>::GetRotationMatrixDerivative(T theta, T thetadot) {
  using std::cos;
  using std::sin;
  const T cth = cos(theta), sth = sin(theta);
  Matrix2<T> Rdot;
  Rdot << -sth, -cth, cth, -sth;
  return Rdot * thetadot;
}

template <class T>
void Rod2D<T>::GetContactPoints(const systems::Context<T>& context,
                                std::vector<Vector2<T>>* points) const {
  using std::cos;
  using std::sin;

  DRAKE_DEMAND(points);
  DRAKE_DEMAND(points->empty());

  const Vector3<T> q = GetRodConfig(context);
  const T& x = q[0];
  const T& y = q[1];
  T cth = cos(q[2]), sth = sin(q[2]);

  // Get the two rod endpoint locations.
  const T half_len = get_rod_half_length();
  const Vector2<T> pa = CalcRodEndpoint(x, y, -1, cth, sth, half_len);
  const Vector2<T> pb = CalcRodEndpoint(x, y, +1, cth, sth, half_len);

  // If an endpoint touches the ground, add it as a contact point.
  if (pa[1] <= 0)
    points->push_back(pa);
  if (pb[1] <= 0)
    points->push_back(pb);
}

template <class T>
void Rod2D<T>::GetContactPointsTangentVelocities(
    const systems::Context<T>& context,
    const std::vector<Vector2<T>>& points, std::vector<T>* vels) const {
  DRAKE_DEMAND(vels);
  const Vector3<T> q = GetRodConfig(context);
  const Vector3<T> v = GetRodVelocity(context);

  // Get necessary quantities.
  const Vector2<T> p_WRo = q.segment(0, 2);
  const Vector2<T> v_WRo = v.segment(0, 2);
  const T& w_WR = v[2];

  vels->resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    (*vels)[i] = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR,
                                                points[i])[0];
  }
}

// Gets the row of a contact Jacobian matrix, given a point of contact, @p p,
// and projection direction (unit vector), @p dir. Aborts if @p dir is not a
// unit vector.
template <class T>
Vector3<T> Rod2D<T>::GetJacobianRow(const systems::Context<T>& context,
                                    const Vector2<T>& p,
                                    const Vector2<T>& dir) const {
  using std::abs;
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  DRAKE_DEMAND(abs(dir.norm() - 1) < eps);

  // Get rod configuration variables.
  const Vector3<T> q = GetRodConfig(context);

  // Compute cross product of the moment arm (expressed in the world frame)
  // and the direction.
  const Vector2<T> r(p[0] - q[0], p[1] - q[1]);
  return Vector3<T>(dir[0], dir[1], cross2(r, dir));
}

// Gets the time derivative of a row of a contact Jacobian matrix, given a
// point of contact, @p p, and projection direction (unit vector), @p dir.
// Aborts if @p dir is not a unit vector.
template <class T>
Vector3<T> Rod2D<T>::GetJacobianDotRow(const systems::Context<T>& context,
                                       const Vector2<T>& p,
                                       const Vector2<T>& dir) const {
  using std::abs;
  const double eps = 10 * std::numeric_limits<double>::epsilon();
  DRAKE_DEMAND(abs(dir.norm() - 1) < eps);

  // Get rod state variables.
  const Vector3<T> q = GetRodConfig(context);
  const Vector3<T> v = GetRodVelocity(context);

  // Get the transformation of vectors from the rod frame to the
  // world frame and its time derivative.
  const T& theta = q[2];
  Eigen::Rotation2D<T> R(theta);

  // Get the vector from the rod center-of-mass to the contact point,
  // expressed in the rod frame.
  const Vector2<T> x = q.segment(0, 2);
  const Vector2<T> u = R.inverse() * (p - x);

  // Compute the translational velocity of the contact point.
  const Vector2<T> xdot = v.segment(0, 2);
  const T& thetadot = v[2];
  const Matrix2<T> Rdot = GetRotationMatrixDerivative(theta, thetadot);
  const Vector2<T> pdot = xdot + Rdot * u;

  // Compute cross product of the time derivative of the moment arm (expressed
  // in the world frame) and the direction.
  const Vector2<T> rdot(pdot[0] - v[0], pdot[1] - v[1]);
  return Vector3<T>(0, 0, cross2(rdot, dir));
}

template <class T>
Matrix2<T> Rod2D<T>::GetSlidingContactFrameToWorldTransform(
    const T& xaxis_velocity) const {
  // Note: the contact normal for the rod with the horizontal plane always
  // points along the world +y axis; the sliding tangent vector points along
  // either the +/-x (world) axis. The << operator populates the matrix row by
  // row, so
  // R_WC = | 0  ±1 |
  //        | 1   0 |
  // indicating that the contact normal direction (the +x axis in the contact
  // frame) is +y in the world frame and the contact tangent direction (more
  // precisely, the direction of sliding, the +y axis in the contact frame) is
  // ±x in the world frame.
  DRAKE_DEMAND(xaxis_velocity != 0);
  Matrix2<T> R_WC;
  // R_WC's elements match how they appear lexically: one row per line.
  R_WC << 0, ((xaxis_velocity > 0) ? 1 : -1),
          1, 0;
  return R_WC;
}

template <class T>
Matrix2<T> Rod2D<T>::GetNonSlidingContactFrameToWorldTransform() const {
  // Note: the contact normal for the rod with the horizontal plane always
  // points along the world +y axis; the non-sliding tangent vector always
  // points along the world +x axis. The << operator populates the matrix row by
  // row, so
  // R_WC = | 0 1 |
  //        | 1 0 |
  // indicating that the contact normal direction is (the +x axis in the contact
  // frame) is +y in the world frame and the contact tangent direction (the +y
  // axis in the contact frame) is +x in the world frame.
  Matrix2<T> R_WC;
  // R_WC's elements match how they appear lexically: one row per line.
  R_WC << 0, 1,
          1, 0;
  return R_WC;
}

template <class T>
void Rod2D<T>::CalcConstraintProblemData(
    const systems::Context<T>& context,
    const std::vector<Vector2<T>>& points,
    const std::vector<T>& tangent_vels,
    multibody::constraint::ConstraintAccelProblemData<T>* data)
    const {
  using std::abs;
  DRAKE_DEMAND(data);
  DRAKE_DEMAND(points.size() == tangent_vels.size());

  // Set the inertia solver.
  data->solve_inertia = [this](const MatrixX<T>& m) {
    return solve_inertia(m);
  };

  // The normal and tangent spanning direction are unique.
  const Matrix2<T> R_wc = GetNonSlidingContactFrameToWorldTransform();
  const Vector2<T> contact_normal = R_wc.col(0);
  const Vector2<T> contact_tangent = R_wc.col(1);

  // Verify contact normal and tangent directions are as we expect.
  DRAKE_ASSERT(abs(contact_normal.dot(Vector2<T>(0, 1)) - 1) <
      std::numeric_limits<double>::epsilon());
  DRAKE_ASSERT(abs(contact_tangent.dot(Vector2<T>(1, 0)) - 1) <
      std::numeric_limits<double>::epsilon());

  // Get the set of contact points.
  const int nc = points.size();

  // Get the set of tangent velocities.
  const T sliding_vel_tol = GetSlidingVelocityTolerance();

  // Set sliding and non-sliding contacts.
  std::vector<int>& non_sliding_contacts = data->non_sliding_contacts;
  std::vector<int>& sliding_contacts = data->sliding_contacts;
  non_sliding_contacts.clear();
  sliding_contacts.clear();
  for (int i = 0; i < nc; ++i) {
    if (abs(tangent_vels[i]) < sliding_vel_tol) {
      non_sliding_contacts.push_back(i);
    } else {
      sliding_contacts.push_back(i);
    }
  }

  // Designate sliding and non-sliding contacts.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();

  // Set sliding and non-sliding friction coefficients.
  data->mu_sliding.setOnes(num_sliding) *= get_mu_coulomb();
  data->mu_non_sliding.setOnes(num_non_sliding) *= get_mu_static();

  // Set spanning friction cone directions (set to unity, because rod is 2D).
  data->r.resize(num_non_sliding);
  for (int i = 0; i < num_non_sliding; ++i)
    data->r[i] = 1;

  // Form the normal contact Jacobian (N).
  const int ngc = 3;        // Number of generalized coordinates / velocities.
  MatrixX<T> N(nc, ngc);
  for (int i = 0; i < nc; ++i)
    N.row(i) = GetJacobianRow(context, points[i], contact_normal);
  data->N_mult = [N](const VectorX<T>& w) -> VectorX<T> { return N * w; };

  // Form Ndot (time derivative of N) and compute Ndot * v.
  MatrixX<T> Ndot(nc, ngc);
  for (int i = 0; i < nc; ++i)
    Ndot.row(i) =  GetJacobianDotRow(context, points[i], contact_normal);
  const Vector3<T> v = GetRodVelocity(context);
  data->kN = Ndot * v;
  data->gammaN.setZero(nc);

  // Form the tangent directions contact Jacobian (F), its time derivative
  // (Fdot), and compute Fdot * v.
  const int nr = std::accumulate(data->r.begin(), data->r.end(), 0);
  MatrixX<T> F = MatrixX<T>::Zero(nr, ngc);
  MatrixX<T> Fdot = MatrixX<T>::Zero(nr, ngc);
  for (int i = 0; i < static_cast<int>(non_sliding_contacts.size()); ++i) {
    const int contact_index = non_sliding_contacts[i];
    F.row(i) = GetJacobianRow(context, points[contact_index], contact_tangent);
    Fdot.row(i) = GetJacobianDotRow(
        context, points[contact_index], contact_tangent);
  }
  data->kF = Fdot * v;
  data->F_mult = [F](const VectorX<T>& w) -> VectorX<T> { return F * w; };
  data->F_transpose_mult = [F](const VectorX<T>& w) -> VectorX<T> {
    return F.transpose() * w;
  };
  data->gammaF.setZero(nr);
  data->gammaE.setZero(non_sliding_contacts.size());

  // Form N - mu*Q (Q is sliding contact direction Jacobian).
  MatrixX<T> N_minus_mu_Q = N;
  Vector3<T> Qrow;
  for (int i = 0; i < static_cast<int>(sliding_contacts.size()); ++i) {
    const int contact_index = sliding_contacts[i];
    Matrix2<T> sliding_contract_frame = GetSlidingContactFrameToWorldTransform(
        tangent_vels[contact_index]);
    const auto& sliding_dir = sliding_contract_frame.col(1);
    Qrow = GetJacobianRow(context, points[contact_index], sliding_dir);
    N_minus_mu_Q.row(contact_index) -= data->mu_sliding[i] * Qrow;
  }
  data->N_minus_muQ_transpose_mult = [N_minus_mu_Q](const VectorX<T>& w) ->
      VectorX<T> { return N_minus_mu_Q.transpose() * w; };

  data->kL.resize(0);
  data->gammaL.resize(0);

  // Set external force vector.
  data->tau = ComputeExternalForces(context);
}

template <class T>
void Rod2D<T>::CalcImpactProblemData(
    const systems::Context<T>& context,
    const std::vector<Vector2<T>>& points,
    multibody::constraint::ConstraintVelProblemData<T>* data) const {
  using std::abs;
  DRAKE_DEMAND(data);

  // Setup the generalized inertia matrix.
  Matrix3<T> M;
  M << mass_, 0,     0,
       0,     mass_, 0,
       0,     0,     J_;

  // Get the generalized velocity.
  data->Mv = M * context.get_continuous_state().get_generalized_velocity().
      CopyToVector();

  // Set the inertia solver.
  data->solve_inertia = [this](const MatrixX<T>& m) {
    return solve_inertia(m);
  };

  // The normal and tangent spanning direction are unique for the rod undergoing
  // impact (i.e., unlike with non-impacting rigid contact equations, the
  // frame does not change depending on sliding velocity direction).
  const Matrix2<T> non_sliding_contact_frame =
      GetNonSlidingContactFrameToWorldTransform();
  const Vector2<T> contact_normal = non_sliding_contact_frame.col(0);
  const Vector2<T> contact_tan = non_sliding_contact_frame.col(1);

  // Get the set of contact points.
  const int num_contacts = points.size();

  // Set sliding and non-sliding friction coefficients.
  data->mu.setOnes(num_contacts) *= get_mu_coulomb();

  // Set spanning friction cone directions (set to unity, because rod is 2D).
  data->r.resize(num_contacts);
  for (int i = 0; i < num_contacts; ++i)
    data->r[i] = 1;

  // Form the normal contact Jacobian (N).
  const int num_generalized_coordinates = 3;
  MatrixX<T> N(num_contacts, num_generalized_coordinates);
  for (int i = 0; i < num_contacts; ++i)
    N.row(i) = GetJacobianRow(context, points[i], contact_normal);
  data->N_mult = [N](const VectorX<T>& w) -> VectorX<T> { return N * w; };
  data->N_transpose_mult = [N](const VectorX<T>& w) -> VectorX<T> {
    return N.transpose() * w; };
  data->kN.setZero(num_contacts);
  data->gammaN.setZero(num_contacts);

  // Form the tangent directions contact Jacobian (F).
  const int nr = std::accumulate(data->r.begin(), data->r.end(), 0);
  MatrixX<T> F(nr, num_generalized_coordinates);
  for (int i = 0; i < num_contacts; ++i) {
    F.row(i) = GetJacobianRow(context, points[i], contact_tan);
  }
  data->F_mult = [F](const VectorX<T>& w) -> VectorX<T> { return F * w; };
  data->F_transpose_mult = [F](const VectorX<T>& w) -> VectorX<T> {
    return F.transpose() * w;
  };
  data->kF.setZero(nr);
  data->gammaF.setZero(nr);
  data->gammaE.setZero(num_contacts);

  data->kL.resize(0);
  data->gammaL.resize(0);
}

template <typename T>
void Rod2D<T>::CopyStateOut(const systems::Context<T>& context,
                            systems::BasicVector<T>* state_port_value) const {
  // Output port value is just the continuous or discrete state.
  const VectorX<T> state = (simulation_type_ == SimulationType::kTimeStepping)
                               ? context.get_discrete_state(0).CopyToVector()
                               : context.get_continuous_state().CopyToVector();
  state_port_value->SetFromVector(state);
}

template <typename T>
void Rod2D<T>::CopyPoseOut(
    const systems::Context<T>& context,
    systems::rendering::PoseVector<T>* pose_port_value) const {
  const VectorX<T> state = (simulation_type_ == SimulationType::kTimeStepping)
                               ? context.get_discrete_state(0).CopyToVector()
                               : context.get_continuous_state().CopyToVector();
  ConvertStateToPose(state, pose_port_value);
}

/// Integrates the Rod 2D example forward in time using a
/// half-explicit time stepping scheme.
template <class T>
void Rod2D<T>::DoCalcDiscreteVariableUpdates(
    const systems::Context<T>& context,
    const std::vector<const systems::DiscreteUpdateEvent<T>*>&,
    systems::DiscreteValues<T>* discrete_state) const {
  using std::sin;
  using std::cos;

  // Determine ERP and CFM.
  const double erp = get_erp();
  const double cfm = get_cfm();

  // Get the necessary state variables.
  const systems::BasicVector<T>& state = context.get_discrete_state(0);
  const auto& q = state.get_value().template segment<3>(0);
  Vector3<T> v = state.get_value().template segment<3>(3);
  const T& x = q(0);
  const T& y = q(1);
  const T& theta = q(2);

  // Construct the problem data.
  const int ngc = 3;      // Number of generalized coords / velocities.
  multibody::constraint::ConstraintVelProblemData<T> problem_data(ngc);

  // Two contact points, corresponding to the two rod endpoints, are always
  // used, regardless of whether any part of the rod is in contact with the
  // halfspace. This practice is standard in time stepping approaches with
  // constraint stabilization. See:
  // M. Anitescu and G. Hart. A Constraint-Stabilized Time-Stepping Approach
  // for Rigid Multibody Dynamics with Joints, Contact, and Friction. Intl.
  // J. for Numerical Methods in Engr., 60(14), 2004.
  const int nc = 2;

  // Find left and right end point locations.
  const T stheta = sin(theta), ctheta = cos(theta);
  const Vector2<T> left =
      CalcRodEndpoint(x, y, -1, ctheta, stheta, half_length_);
  const Vector2<T> right =
      CalcRodEndpoint(x, y,  1, ctheta, stheta, half_length_);

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  Eigen::Matrix<T, nc, ngc> N, F;
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (left[0]  - x);
  N(1, 2) = (right[0] - x);
  F(0, 0) = F(1, 0) = 1;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -(left[1]  - y);
  F(1, 2) = -(right[1] - y);

  // Set the number of tangent directions.
  problem_data.r = { 1, 1 };

  // Get the total number of tangent directions.
  const int nr = std::accumulate(
      problem_data.r.begin(), problem_data.r.end(), 0);

  // Set the coefficients of friction.
  problem_data.mu.setOnes(nc) *= get_mu_coulomb();

  // Set up the operators.
  problem_data.N_mult = [&N](const VectorX<T>& vv) -> VectorX<T> {
    return N * vv;
  };
  problem_data.N_transpose_mult = [&N](const VectorX<T>& vv) -> VectorX<T> {
    return N.transpose() * vv;
  };
  problem_data.F_transpose_mult = [&F](const VectorX<T>& vv) -> VectorX<T> {
    return F.transpose() * vv;
  };
  problem_data.F_mult = [&F](const VectorX<T>& vv) -> VectorX<T> {
    return F * vv;
  };
  problem_data.solve_inertia = [this](const MatrixX<T>& mm) -> MatrixX<T> {
    return GetInverseInertiaMatrix() * mm;
  };

  // Update the generalized velocity vector with discretized external forces
  // (expressed in the world frame).
  const Vector3<T> fext = ComputeExternalForces(context);
  v += dt_ * problem_data.solve_inertia(fext);
  problem_data.Mv = GetInertiaMatrix() * v;

  // Set stabilization parameters.
  problem_data.kN.resize(nc);
  problem_data.kN[0] = erp * left[1] / dt_;
  problem_data.kN[1] = erp * right[1] / dt_;
  problem_data.kF.setZero(nr);

  // Set regularization parameters.
  problem_data.gammaN.setOnes(nc) *= cfm;
  problem_data.gammaF.setOnes(nr) *= cfm;
  problem_data.gammaE.setOnes(nc) *= cfm;

  // Solve the constraint problem.
  VectorX<T> cf;
  solver_.SolveImpactProblem(problem_data, &cf);

  // Compute the updated velocity.
  VectorX<T> delta_v;
  solver_.ComputeGeneralizedVelocityChange(problem_data, cf, &delta_v);

  // Compute the new velocity. Note that external forces have already been
  // incorporated into v.
  VectorX<T> vplus = v + delta_v;

  // Compute the new position using an "explicit" update.
  VectorX<T> qplus = q + vplus * dt_;

  // Set the new discrete state.
  systems::BasicVector<T>& new_state = discrete_state->get_mutable_vector(0);
  new_state.get_mutable_value().segment(0, 3) = qplus;
  new_state.get_mutable_value().segment(3, 3) = vplus;
}

/// Models impact using an inelastic impact model with friction.
template <typename T>
void Rod2D<T>::HandleImpact(const systems::Context<T>& context,
                               systems::State<T>* new_state) const {
  using std::abs;

  // This method is only used for piecewise DAE integration. The time
  // stepping method implicitly incorporates impact into its model.
  DRAKE_DEMAND(simulation_type_ == SimulationType::kPiecewiseDAE);

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Get the continuous state vector.
  systems::VectorBase<T>& new_statev = new_state->
      get_mutable_continuous_state().get_mutable_vector();

  // Positional aspects of state do not change.
  new_statev.SetAtIndex(0, x);
  new_statev.SetAtIndex(1, y);
  new_statev.SetAtIndex(2, theta);

  // Copy abstract variables (mode and contact point) by default. Mode
  // may change depending on impact outcome.
  new_state->get_mutable_abstract_state().CopyFrom(
      context.get_abstract_state());

  // If there is no impact, quit now.
  if (!IsImpacting(context)) {
    new_statev.SetAtIndex(3, xdot);
    new_statev.SetAtIndex(4, ydot);
    new_statev.SetAtIndex(5, thetadot);
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
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const Vector2<T> c = CalcRodEndpoint(x, y, k, ctheta, stheta, half_length_);
  const T cx = c[0];
  const T cy = c[1];

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
      Rod2D<T>::kSlidingSingleContact;
  } else {
    // Set the mode to sticking.
    new_state->template get_mutable_abstract_state<Mode>(0) =
      Rod2D<T>::kStickingSingleContact;
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
                   k * ctheta * half_length_ * (thetadot + delta_thetadot) >
               -10*std::numeric_limits<double>::epsilon());

  // Update the velocity.
  new_statev.SetAtIndex(3, xdot + delta_xdot);
  new_statev.SetAtIndex(4, ydot + delta_ydot);
  new_statev.SetAtIndex(5, thetadot + delta_thetadot);
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
// TODO(@edrumwri) This return vector is backwards -- should be x,y not y,x!
template <class T>
Vector2<T> Rod2D<T>::CalcFConeImpactImpulse(
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
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const T cy = y + k * stheta * half_length_;
  const double mu = mu_;
  const double J = J_;
  const double mass = mass_;
  const double r = 2 * half_length_;

  // Compute the impulses.
  const T cxdot = xdot - k * stheta * half_length_ * thetadot;
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
// TODO(@edrumwri) This return vector is backwards -- should be x,y not y,x!
template <class T>
Vector2<T> Rod2D<T>::CalcStickingImpactImpulse(
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
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : 1;
  const T cy = y + k * stheta * half_length_;

  // Compute the impulses.
  const double r = 2 * half_length_;
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

// Sets the velocity derivatives for the rod, given contact forces at a single
// point of contact.
// @param [out] f the meta vector object containing all derivatives; velocity
//        derivatives *only* will be modified on return.
// @param fN the magnitude of the force applied normal to a single contact.
// @param fF the magnitude of the force applied tangent to a single contact.
// @param c the location of the point of contact.
template <class T>
void Rod2D<T>::SetAccelerations(const systems::Context<T>& context,
                                const T& fN, const T& fF,
                                const Vector2<T>& c,
                                systems::VectorBase<T>* const f) const {
  using std::abs;

  // Get the point of contact.
  const T& cx = c(0);
  const T& cy = c(1);

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Retrieve the external forces.
  const Vector3<T> fext = ComputeExternalForces(context);

  // Compute the velocity derivatives.
  const T xddot = (fext(0) + fF) / mass_;
  const T yddot = (fext(1) + fN) / mass_;
  const T contact_moment = (cx - x) * fN - (cy - y) * fF;
  const T thetaddot = (contact_moment + fext(2)) / J_;

  // Set the derivatives.
  f->SetAtIndex(3, xddot);
  f->SetAtIndex(4, yddot);
  f->SetAtIndex(5, thetaddot);

  // Get constants for checking accelerations.
  const double mu = mu_;
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = get_k(context);
  const double h = half_length_;

  // Verify that the vertical acceleration at the point of contact is zero
  // (i.e., cyddot = 0).
  const T cyddot =
      yddot + h * k * (ctheta * thetaddot - stheta * thetadot * thetadot);

  DRAKE_DEMAND(abs(cyddot) < 10 * std::numeric_limits<double>::epsilon());

  // If the force is within the friction cone, verify that the horizontal
  // acceleration at the point of contact is zero (i.e., cxddot = 0).
  if (fN * mu > abs(fF)) {
    const T cxddot =
        xddot + h * k * (-stheta * thetaddot - ctheta * thetadot * thetadot);

    DRAKE_DEMAND(abs(cxddot) < 10 * std::numeric_limits<double>::epsilon());
  }
}

// Sets the velocity derivatives for the rod, given contact forces at the two
// rod endpoints.
// @param [out] f the meta vector object containing all derivatives; velocity
//        derivatives *only* will be modified on return.
// @param fN the magnitudes of the forces applied normal to the contacts.
// @param fF the magnitudes of the forces applied tangent to the contacts.
// @param ca the location of the point of the first point of contact.
// @param cb the location of the point of the second point of contact.
template <class T>
void Rod2D<T>::SetAccelerations(const systems::Context<T>& context,
                                const Vector2<T>& fN, const Vector2<T>& fF,
                                const Vector2<T>& ca, const Vector2<T>& cb,
                                systems::VectorBase<T>* const f) const {
  using std::abs;

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Retrieve the external forces.
  const Vector3<T> fext = ComputeExternalForces(context);

  // Compute the derivatives
  const T xddot = (fext(0) + fF[0] + fF[1]) / mass_;
  const T yddot = (fext(1) + fN[0] + fN[1]) / mass_;
  const T moment_a = (ca[0] - x) * fN[0] - (ca[1] - y) * fF[0];
  const T moment_b = (cb[0] - x) * fN[1] - (cb[1] - y) * fF[1];
  const T thetaddot = (moment_a + moment_b + fext(2)) / J_;

  // Set the derivatives.
  f->SetAtIndex(3, xddot);
  f->SetAtIndex(4, yddot);
  f->SetAtIndex(5, thetaddot);

  // Get constants for checking accelerations.
  const double mu = mu_;
  const T ctheta = cos(theta), stheta = sin(theta);
  const double h = half_length_;

  // Verify that the vertical acceleration at both points of contact are zero
  // (i.e., cyddot = 0).
  const T cyddot0 =
      yddot + h * 1 * (ctheta * thetaddot - stheta * thetadot * thetadot);
  const T cyddot1 =
      yddot + h * -1 * (ctheta * thetaddot - stheta * thetadot * thetadot);

  const double eps = 250 * std::numeric_limits<double>::epsilon();
  DRAKE_DEMAND(abs(cyddot0) < eps);
  DRAKE_DEMAND(abs(cyddot1) < eps);

  // If the force is within the friction cone, verify that the horizontal
  // acceleration at the point of contact is zero (i.e., cxddot = 0).
  if (fN[0] * mu > abs(fF[0]) && fN[1] * mu > abs(fF[1])) {
    const T cxddot0 =
        xddot + h * 1 * (-stheta * thetaddot - ctheta * thetadot * thetadot);
    const T cxddot1 =
        xddot + h * -1 * (-stheta * thetaddot - ctheta * thetadot * thetadot);

    DRAKE_DEMAND(abs(cxddot0) < eps);
    DRAKE_DEMAND(abs(cxddot1) < eps);
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
Vector2<T> Rod2D<T>::CalcStickingContactForces(
    const systems::Context<T>& context) const {
  // Get necessary state variables.
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Precompute quantities that will be used repeatedly.
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = get_k(context);

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);

  // Set named Mathematica constants.
  const double mass = mass_;
  const double r = 2 * half_length_;
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

// Returns the generalized inertia matrix computed about the
// center of mass of the rod and expressed in the world frame.
template <class T>
Matrix3<T> Rod2D<T>::GetInertiaMatrix() const {
  Matrix3<T> M;
  M << mass_, 0,     0,
       0,     mass_, 0,
       0,     0,     J_;
  return M;
}

// Returns the inverse of the generalized inertia matrix computed about the
// center of mass of the rod and expressed in the world frame.
template <class T>
Matrix3<T> Rod2D<T>::GetInverseInertiaMatrix() const {
  Matrix3<T> M_inv;
  M_inv << 1.0 / mass_, 0,           0,
           0,           1.0 / mass_, 0,
           0,           0,           1.0 / J_;
  return M_inv;
}

// Computes the contact forces for the case of nonzero sliding velocity at
// two points of contact. Equations governing the dynamics in this mode are:
//
// (1) 0 ≤ fN ⊥ N⋅dv/dt + dN/dt⋅v ≥ 0
// (2) M⋅dv/dt = fext + NᵀfN - μFᵀfN
// where M is the 3x3 generalized inertia matrix, v is the generalized velocity
// vector, fext is the generalized external force vector, μ is the coefficient
// of friction, N ∈ ℝⁿˣ³ is the Jacobian matrix transforming generalized
// velocities to velocities along the normal component of the n contact frames,
// and F ∈ ℝⁿˣ³ is the Jacobian matrix transforming generalized velocities to
// velocities along the _directions of sliding_ of the n contact frames. As in
// other functions, we assume that fN ∈ ℝⁿ are the magnitudes of forces applied
// along the contact normals and fF ∈ ℝⁿ are the magnitude of forces applied
// *against the directions of sliding*.
//
// ⊥ is the "complementarity operator" and 0 ≤ a ⊥ b ≥ 0 represents the
// conjunction of three constraints:
// (3) a ≥ 0
// (4) b ≥ 0
// (5) (a⋅b) = 0
// where the last of these is known as the "complementarity constraint".
//
// From Equation 2:
//   M⋅dv/dt = fext + NᵀfN - μFᵀfN
//   dv/dt = M⁻¹(fext + NᵀfN - μFᵀfN)
// Equation 1 then can be reformulated as:
//   0 ≤ fN ⊥ N⋅M⁻¹(fext + NᵀfN - μFᵀfN) + dN/dt⋅v ≥ 0
// Therefore, the LCP matrix is:
//   N⋅M⁻¹⋅(Nᵀ - μFᵀ)
// and the LCP vector is:
//   N⋅M⁻¹⋅fext + dN/dt⋅v
template <class T>
void Rod2D<T>::CalcTwoContactSlidingForces(
    const systems::Context<T>& context, Vector2<T>* fN, Vector2<T>* fF) const {
  // Get the necessary state variables.
  const VectorX<T> state = context.get_continuous_state_vector().
      CopyToVector();
  const auto& q = state.template segment<3>(0);
  const auto& v = state.template segment<3>(3);
  const T& x = q(0);
  const T& y = q(1);
  const T& theta = q(2);
  const T& xdot = v(0);
  const T& ydot = v(1);
  const T& thetadot = v(2);

  // Function name is predicated on two contact points.
  const int nc = 2;

  // Verify arguments.
  DRAKE_DEMAND(fN && fN->size() == nc);
  DRAKE_DEMAND(fF && fF->size() == nc);

  // Get the coefficient of friction.
  const double mu = get_mu_coulomb();

  // Three generalized coordinates / velocities.
  const int ngc = 3;

  // Find left and right end point locations.
  const T stheta = sin(theta), ctheta = cos(theta);
  const Vector2<T> left =
      CalcRodEndpoint(x, y, -1, ctheta, stheta, half_length_);
  const Vector2<T> right =
      CalcRodEndpoint(x, y, 1, ctheta, stheta, half_length_);

  // Compute the velocities at the rod end points.
  const Vector2<T> v_WRo(xdot, ydot);
  const T w_WR(thetadot);
  const Vector2<T> p_WRo(x, y);
  const Vector2<T> leftdot = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR,
                                                            left);
  const Vector2<T> rightdot = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR,
                                                             right);

  // Get the inverse of the generalized inertia matrix.
  const Matrix3<T> M_inv = GetInverseInertiaMatrix();

  // Compute the external forces (expressed in the world frame).
  const Vector3<T> fext = ComputeExternalForces(context);

  // Verify that the two directions of sliding are identical.
  double sliding_sign_left = (leftdot[0] > 0) ? 1 : -1;
  double sliding_sign_right = (rightdot[0] > 0) ? 1 : -1;
  DRAKE_DEMAND(sliding_sign_left * sliding_sign_right > 0);

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  Eigen::Matrix<T, nc, ngc> N, F;
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (left[0] - x);
  N(1, 2) = (right[0] - x);
  F(0, 0) = sliding_sign_left;
  F(1, 0) = sliding_sign_right;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -sliding_sign_left*(left[1] - y);
  F(1, 2) = -sliding_sign_right*(right[1] - y);

  // Compute dN/dt.
  Eigen::Matrix<T, nc, ngc> Ndot;
  Ndot(0, 0) = Ndot(1, 0) = 0;
  Ndot(0, 1) = Ndot(1, 1) = 0;
  Ndot(0, 2) = (leftdot[0] - xdot);
  Ndot(1, 2) = (rightdot[0] - xdot);

  // Form the vector in the 2-dimensional linear complementarity problem.
  Vector2<T> qq;
  qq = N * M_inv * fext + Ndot * v;

  // Form the 2x2 linear complementarity problem matrix.
  Matrix2<T> MM;
  MM = N * M_inv * (N.transpose() - mu * F.transpose());

  // Attempt to solve the LCP. For μ = 0, the LCP is guaranteed to have a
  // solution, and several algorithms (e.g., Dantzig's Principle Pivoting
  // Method) are capable of solving it in expected polynomial time. For μ > 0,
  // the LCP might not possess a solution. We use Lemke's Algorithm, which is
  // capable of solving "harder" LCPs (i.e., more classes of matrices) than
  // Dantzig's Algorithm, and then we verify the solution if success is
  // reported.

  // Set a zero tolerance for solving the LCP.
  const double eps = std::numeric_limits<double>::epsilon();
  const double zero_tol = std::sqrt(eps);

  // Solve the LCP.
  VectorX<T> zz, ww;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, zero_tol);
  ww = MM * zz + qq;

  // If not successful, throw an exception. It is conceivable that Lemke's
  // Algorithm will report success when it is applied to a "harder" than
  // copositive problem without actually finding a solution. Thus, we also
  // check that the LCP solution really is a solution.
  if (!success || zz.minCoeff() < -zero_tol || ww.minCoeff() < -zero_tol ||
      std::abs(zz.dot(ww)) > zero_tol)
    throw std::runtime_error("Unable to solve LCP- it may be unsolvable.");

  // Obtain the normal and frictional contact forces.
  *fN = zz;
  *fF = (*fN) * -mu;
}

// Computes the contact forces for the case of zero sliding velocity at two
// points of contact. Equations governing the dynamics in this mode are:
//
// (1) 0 ≤ fN ⊥ N⋅dv/dt + dN/dt⋅v ≥ 0
// (2) 0 ≤ Eλ + D⋅dv/dt + dD/dt v  ⊥  fD ≥ 0
// (3) 0 ≤ (μ⋅fN - Eᵀ⋅fD)  ⊥  λ ≥ 0
// (4) M⋅dv/dt = fext + NᵀfN + DᵀfD
// where M is the 3x3 generalized inertia matrix, v is the generalized velocity
// vector, fext is the generalized external force vector, μ is the coefficient
// of friction, N ∈ ℝⁿˣ³ is the Jacobian matrix transforming generalized
// velocities to velocities along the normal component of the n contact frames,
// D ∈ ℝ²ⁿˣ³ is the Jacobian matrix transforming generalized velocities to
// velocities along the positive and negative tangent directions at each point
// of contact, E is a block matrix of ones, λ- which can be viewed as a slack
// variable- is roughly interpretable as the magnitudes of remaining tangential
// acceleration at each point of contact after contact forces are applied,
// fN ∈ ℝⁿ are the magnitudes of forces applied along the contact normals, and
// fD ∈ ℝ²ⁿ are the magnitude of frictional forces applied along the positive
// and negative tangent directions at each point of contact.
//
// Equations (1) through (3) will be referred to as "Complementarity Conditions
// (1)-(3)" in the remainder of this function documentation. They use the
// operator ⊥, where 0 ≤ a ⊥ b ≥ 0 denotes the triple of constraints constraints
// a ≥ 0, b ≥ 0, a⋅b = 0.
//
// Complementarity conditions (2) and (3) were inspired directly from
// corresponding conditions in [Anitescu and Potra, 1997]. In the case that
// the frictional force lies strictly within the friction cone, (μ⋅fN - fD) > 0
// in Equation 3, implying both that λ = 0 and, since fD ≥ 0, μ⋅fN = fD and
// D⋅dv/dt + dD/dt v = 0 (i.e., there can be no tangential acceleration). In the
// case that the friction force is insufficient to prevent sliding,
// D⋅dv/dt + dD/dt v ≠ 0, which implies that λ > 0. It
// should also be evident that when the frictional force pushes along +x, any
// remaining tangential acceleration must be pointed along -x (and vice versa).
template <class T>
void Rod2D<T>::CalcTwoContactNoSlidingForces(
    const systems::Context<T>& context, Vector2<T>* fN, Vector2<T>* fF) const {
  // Get the necessary state variables.
  const VectorX<T> state = context.get_continuous_state_vector().
      CopyToVector();
  const auto& q = state.template segment<3>(0);
  const auto& v = state.template segment<3>(3);
  const T& x = q(0);
  const T& y = q(1);
  const T& theta = q(2);
  const T& xdot = v(0);
  const T& ydot = v(1);
  const T& thetadot = v(2);

  // Function name is predicated on two contact points.
  const int nc = 2;

  // Verify arguments.
  DRAKE_DEMAND(fN && fN->size() == nc);
  DRAKE_DEMAND(fF && fF->size() == nc);

  // Three generalized coordinates / velocities.
  const int ngc = 3;

  // Find left and right end point locations.
  const T stheta = sin(theta), ctheta = cos(theta);
  const Vector2<T> left = CalcRodEndpoint(x, y, -1, ctheta, stheta,
                                          half_length_);
  const Vector2<T> right = CalcRodEndpoint(x, y, 1, ctheta, stheta,
                                          half_length_);

  // Compute the velocities at the end points.
  const Vector2<T> v_WRo(xdot, ydot);
  const T w_WR(thetadot);
  const Vector2<T> p_WRo(x, y);
  const Vector2<T> leftdot = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR,
                                                            left);
  const Vector2<T> rightdot = CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR,
                                                             right);

  // Get the inverse of the generalized inertia matrix.
  const Matrix3<T> M_inv = GetInverseInertiaMatrix();

  // Compute the external forces (expressed in the world frame).
  const Vector3<T> fext = ComputeExternalForces(context);

  // Set up the contact normal and tangent (friction) direction Jacobian
  // matrices. These take the form:
  //     | 0 1 n1 |        | 1 0 f1 |
  // N = | 0 1 n2 |    F = | 1 0 f2 |
  // where n1, n2/f1, f2 are the moment arm induced by applying the
  // force at the given contact point along the normal/tangent direction.
  Eigen::Matrix<T, nc, ngc> N, F;
  N(0, 0) = N(1, 0) = 0;
  N(0, 1) = N(1, 1) = 1;
  N(0, 2) = (left[0] - x);
  N(1, 2) = (right[0] - x);
  F(0, 0) = 1;
  F(1, 0) = 1;
  F(0, 1) = F(1, 1) = 0;
  F(0, 2) = -(left[1] - y);
  F(1, 2) = -(right[1] - y);

  // Compute D.
  Eigen::Matrix<T, 2*nc, ngc> D;
  D.template block<nc, ngc>(0, 0) = F;
  D.template block<nc, ngc>(nc, 0) = -F;

  // Compute dN/dt.
  Eigen::Matrix<T, nc, ngc> Ndot;
  Ndot(0, 0) = Ndot(1, 0) = 0;
  Ndot(0, 1) = Ndot(1, 1) = 0;
  Ndot(0, 2) = (leftdot[0] - xdot);
  Ndot(1, 2) = (rightdot[0] - xdot);

  // Compute dF/dt.
  Eigen::Matrix<T, nc, ngc> Fdot;
  Fdot(0, 0) = Fdot(1, 0) = 0;
  Fdot(0, 1) = Fdot(1, 1) = 0;
  Fdot(0, 2) = -(leftdot[1] - ydot);
  Fdot(1, 2) = -(rightdot[1] - ydot);

  // Compute dD/dt.
  Eigen::Matrix<T, 2*nc, ngc> Ddot;
  Ddot.template block<nc, ngc>(0, 0) = Fdot;
  Ddot.template block<nc, ngc>(nc, 0) = -Fdot;

  // Construct E.
  const int nk = 2 * nc;
  Eigen::Matrix<T, nk, nc> E;
  E.col(0) << 1, 0, 1, 0;
  E.col(1) << 0, 1, 0, 1;

  // Form the linear complementarity problem matrix and vector.
  Eigen::Matrix<T, 4*nc, 4*nc> MM;
  Eigen::Matrix<T, 4*nc, 1> qq;

  // First two rows correspond to Complementarity Condition (1).
  MM.template block<2, 2>(0, 0) = N * M_inv * N.transpose();
  MM.template block<2, 4>(0, 2) = N * M_inv * D.transpose();
  MM.template block<2, 2>(0, 6).setZero();
  qq.template segment<2>(0) = N * M_inv * fext + Ndot * v;

  // Next four rows correspond to Complementarity Condition (2).
  MM.template block<4, 2>(2, 0) = D * M_inv * N.transpose();
  MM.template block<4, 4>(2, 2) = D * M_inv * D.transpose();
  MM.template block<4, 2>(2, 6) = E;
  qq.template segment<4>(2) = D * M_inv * fext + Ddot * v;

  // Final two rows correspond to Complementarity Condition (3).
  MM.template block<2, 2>(6, 0) = Matrix2<T>::Identity() * get_mu_coulomb();
  MM.template block<2, 4>(6, 2) = -E.transpose();
  MM.template block<2, 2>(6, 6).setZero();
  qq.template segment<2>(nc*3).setZero();

  // Solve the LCP. It has the same form of that in [Anitescu and Potra, 1997];
  // by following the same proof as in that reference, it is clear that this
  // matrix is copositive and hence provably solvable by Lemke's Algorithm.
  VectorX<T> zz, ww;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz);
  DRAKE_DEMAND(success);

  // Obtain the normal and frictional contact forces.
  *fN = zz.template segment<2>(0);
  *fF = zz.template segment<2>(2) - zz.template segment<2>(4);
}

// This is a smooth approximation to a step function. Input x goes from 0 to 1;
// output goes 0 to 1 but smoothed with an S-shaped quintic with first and
// second derivatives zero at both ends.
template <class T>
T Rod2D<T>::step5(const T& x) {
  DRAKE_ASSERT(0 <= x && x <= 1);
  const T x3 = x * x * x;
  return x3 * (10 + x * (6 * x - 15));  // 10x^3-15x^4+6x^5
}

// This function models Stribeck dry friction as a C² continuous quintic spline
// with 3 segments that is used to calculate an effective coefficient of
// friction mu_stribeck. That is the composite coefficient of friction that we
// use to scale the normal force to produce the friction force.
//
// We are given the friction coefficients mu_s (static) and mu_d (dynamic), and
// a dimensionless slip speed s>=0, then calculate:
//     mu_stribeck =
//      (a) s=0..1: smooth interpolation from 0 to mu_s
//      (b) s=1..3: smooth interpolation from mu_s down to mu_d (Stribeck)
//      (c) s=3..Inf mu_d
//
// s must be non-dimensionalized by taking the actual slip speed and dividing by
// the stiction slip velocity tolerance.
//
// mu_stribeck is zero at s=0 with 1st deriv (slope) zero and 2nd deriv
// (curvature) 0. At large speeds s>>0 the value is mu_d, again with zero slope
// and curvature. That makes mu_stribeck(s) C² continuous for all s>=0.
//
// The resulting curve looks like this:
//
//     mu_s   ***
//           *    *
//           *     *
//           *      *
//     mu_d  *        *  *    *    *    *   slope, curvature = 0 at Inf
//          *
//          *
//         *
//  0  * *   slope, curvature = 0 at 0
//
//     |      |           |
//   s=0      1           3
//
template <class T>
T Rod2D<T>::CalcMuStribeck(const T& mu_s, const T& mu_d, const T& s) {
  DRAKE_ASSERT(mu_s >= 0 && mu_d >= 0 && s >= 0);
  T mu_stribeck;
  if (s >= 3)
    mu_stribeck = mu_d;  // sliding
  else if (s >= 1)
    mu_stribeck = mu_s - (mu_s - mu_d) * step5((s - 1) / 2);  // Stribeck
  else
    mu_stribeck = mu_s * step5(s);  // 0 <= s < 1 (stiction)
  return mu_stribeck;
}

// Finds the locations of the two rod endpoints and generates contact forces at
// either or both end points (depending on contact condition) using a linear
// stiffness model, Hunt and Crossley normal dissipation, and a Stribeck
// friction model. No forces are applied anywhere else along the rod. The force
// is returned as the spatial force F_Ro_W (described in Rod2D class comments),
// represented as (fx,fy,τ).
//
// Note that we are attributing all of the compliance to the *halfspace*, not
// the *rod*, so that forces will be applied to the same points of the rod as is
// done in the rigid contact models. That makes comparison of the models easier.
//
// For each end point, let h be the penetration depth (in the -y direction) and
// v the slip speed along x. Then
//   fK =  k*h                   normal force due to stiffness
//   fD =  fK*d*hdot             normal force due to dissipation
//   fN =  max(fK + fD, 0)       total normal force (>= 0)
//   fF = -mu(|v|)*fN*sign(v)    friction force
//   f  = fN + fF                total force
//
// Here mu(v) is a Stribeck function that is zero at zero slip speed, and
// reaches a maximum mu_s at the stiction speed tolerance. mu_s is the
// static coefficient of friction.
//
// Note: we return the spatial force in a Vector3 but it is not a vector
// quantity.
template <class T>
Vector3<T> Rod2D<T>::CalcCompliantContactForces(
    const systems::Context<T>& context) const {
  // Depends on continuous state being available.
  DRAKE_DEMAND(simulation_type_ == SimulationType::kCompliant);

  using std::abs;
  using std::max;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const Vector2<T> p_WRo(state.GetAtIndex(0), state.GetAtIndex(1));
  const T theta = state.GetAtIndex(2);
  const Vector2<T> v_WRo(state.GetAtIndex(3), state.GetAtIndex(4));
  const T w_WR(state.GetAtIndex(5));

  const T ctheta = cos(theta), stheta = sin(theta);

  // Find left and right end point locations.
  Vector2<T> p_WP[2] = {
      CalcRodEndpoint(p_WRo[0], p_WRo[1], -1, ctheta, stheta, half_length_),
      CalcRodEndpoint(p_WRo[0], p_WRo[1], 1, ctheta, stheta, half_length_)};

  // Calculate the net spatial force to apply at rod origin from the individual
  // contact forces at the endpoints.
  Vector3<T> F_Ro_W(0, 0, 0);  // Accumulate contact forces here.
  for (int i = 0; i < 2; ++i) {
    const Vector2<T>& p_WC = p_WP[i];  // Get contact point C location in World.

    // Calculate penetration depth h along -y; negative means separated.
    const T h = -p_WC[1];
    if (h > 0) {  // This point is in contact.
      const Vector2<T> v_WRc =  // Velocity of rod point coincident with C.
          CalcCoincidentRodPointVelocity(p_WRo, v_WRo, w_WR, p_WC);
      const T hdot = -v_WRc[1];  // Penetration rate in -y.
      const T v = v_WRc[0];      // Slip velocity in +x.
      const int sign_v = v < 0 ? -1 : 1;
      const T fK = get_stiffness() * h;  // See method comment above.
      const T fD = fK * get_dissipation() * hdot;
      const T fN = max(fK + fD, T(0));
      const T mu = CalcMuStribeck(get_mu_static(), get_mu_coulomb(),
                                  abs(v) / get_stiction_speed_tolerance());
      const T fF = -mu * fN * T(sign_v);

      // Find the point Rc of the rod that is coincident with the contact point
      // C, measured from Ro but expressed in W.
      const Vector2<T> p_RRc_W = p_WC - p_WRo;
      const Vector2<T> f_Rc(fF, fN);  // The force to apply at Rc.

      const T t_R = cross2(p_RRc_W, f_Rc);  // τ=r X f.
      F_Ro_W += Vector3<T>(fF, fN, t_R);
    }
  }
  return F_Ro_W;  // A 2-vector & scalar, not really a 3-vector.
}

// Computes the accelerations of the rod center of mass for the case of the rod
// contacting the surface at exactly one point and with sliding velocity.
template <class T>
void Rod2D<T>::CalcAccelerationsOneContactSliding(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;
  using std::max;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);
  const T& thetadot = state.GetAtIndex(5);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>& f = derivatives->get_mutable_vector();

  // The two endpoints of the rod are located at (x,y) + R(theta)*[0,r/2] and
  // (x,y) + R(theta)*[0,-r/2], where
  // R(theta) = | cos(theta) -sin(theta) |
  //            | sin(theta)  cos(theta) |
  // and r is designated as the rod length. Thus, the heights of
  // the rod endpoints are y + sin(theta)*r/2 and y - sin(theta)*r/2.
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = get_k(context);

  // Determine the point of contact (cx, cy).
  const Vector2<T> c = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                       half_length_);
  const T& cx = c[0];
  const T& cy = c[1];

  // Compute the horizontal velocity at the point of contact.
  const T cxdot = xdot - k * stheta * half_length_ * thetadot;

  // Get the inputs.
  const int port_index = 0;
  const auto input = this->EvalEigenVectorInput(context, port_index);
  const double fX = input(0);
  const double fY = input(1);
  const double tau = input(2);

  // Compute the normal acceleration at the point of contact (cyddot),
  // *assuming zero contact force*. This equation comes from the kinematics of
  // the rod:
  // cy = y + k * half_rod_length * sin(θ)  [rod endpoint vertical location]
  // dcy/dt = dy/dt + k * half_rod_length * cos(θ) * dθ/dt
  // d²cy/dt² = d²y/dt² - k * half_rod_length * sin(θ) * (dθ/dt)² +
  //                      k * half_rod_length * cos(θ) * d²θ/dt²
  const T yddot = get_gravitational_acceleration() + fY/mass_;
  const T thetaddot = tau/J_;
  T cyddot = yddot -
      k * half_length_ * stheta * thetadot * thetadot +
      k * half_length_ * ctheta * thetaddot;

  // If the normal acceleration is non-negative, no contact forces need be
  // applied.
  const T zero_tol = 10 * std::numeric_limits<double>::epsilon() *
      max(T(1), thetadot) * max(T(1), thetaddot);
  if (cyddot > -zero_tol) {
    f.SetAtIndex(3, fX / mass_);
    f.SetAtIndex(4, fY / mass_ + g_);
    f.SetAtIndex(5, tau / J_);
    return;
  }

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
  const double r = 2 * half_length_;
  const T fN = -((2 * (2 * J * (fY + g * mass) + k * mass * r * tau * ctheta -
       J * k * mass * r * stheta * thetadot * thetadot))/
       (4 * J + k * k * mass * r * r * ctheta * ctheta +
       k * k * mass * mu * r * r * ctheta * sgn_cxdot * stheta));

  // Check for inconsistent configuration.
  if (fN < 0)
    throw std::runtime_error("Inconsistent configuration detected.");

  // Now that normal force is computed, set the acceleration.
  const T fF = -sgn_cxdot * mu_ * fN;
  f.SetAtIndex(3, (fF + fX) / mass_);
  f.SetAtIndex(4, (fN + fY) / mass_ + get_gravitational_acceleration());
  f.SetAtIndex(5, ((cx - x) * fN - (cy - y) * fF + tau) / J);

  // Lines below currently unused but are occasionally helpful for
  // debugging.
  //        const T xddot = f->GetAtIndex(3);
  //        const T cxddot = xddot + r*k*(-stheta*thetaddot -
  //                                        +ctheta*thetadot*thetadot)/2;
}

// Computes the accelerations of the rod center of mass for the case of the rod
// contacting the surface at exactly one point and without any sliding velocity.
template <class T>
void Rod2D<T>::CalcAccelerationsOneContactNoSliding(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Obtain the structure we need to write into.
  systems::VectorBase<T>& f = derivatives->get_mutable_vector();

  // Get necessary state variables.
  const T& x = context.get_continuous_state_vector().GetAtIndex(0);
  const T& y = context.get_continuous_state_vector().GetAtIndex(1);
  const T& theta = context.get_continuous_state_vector().GetAtIndex(2);
  const T& thetadot = context.get_continuous_state_vector().GetAtIndex(5);

  // Get the contact point.
  const int k = get_k(context);
  const T ctheta = cos(theta), stheta = sin(theta);
  const Vector2<T> c = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                       half_length_);
  const T& cx = c[0];
  const T& cy = c[1];

  // Compute the contact forces, assuming sticking contact.
  const Vector2<T> cf = CalcStickingContactForces(context);
  const T& fN = cf(0);
  const T& fF = cf(1);

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
    const double r = 2 * get_rod_half_length();
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
    auto calc_tan_accel = [=](const T N, const T F) {
      const T thetaddot = ((cx - x) * N - (cy - y) * F + tau) / J;
      return (F + fX) / mass +
          r * k * (-stheta * thetaddot - ctheta * thetadot * thetadot) / 2;
    };

    // Compute two tangential acceleration candidates.
    const T cxddot1 = calc_tan_accel(fN1, fF1);
    const T cxddot2 = calc_tan_accel(fN2, fF2);

    // Pick the one that is smaller in magnitude.
    if (abs(cxddot1) < abs(cxddot2)) {
      SetAccelerations(context, fN1, fF1, c, &f);
    } else {
      SetAccelerations(context, fN2, fF2, c, &f);
    }
  } else {
    // Friction force is within the friction cone.
    SetAccelerations(context, fN, fF, c, &f);
  }
}

// Computes the accelerations of the rod center of mass for the case of the rod
// contacting the surface at more than one point.
template <class T>
void Rod2D<T>::CalcAccelerationsTwoContact(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::abs;

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& x = state.GetAtIndex(0);
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& xdot = state.GetAtIndex(3);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>& f = derivatives->get_mutable_vector();

  // Get the two points of contact.
  const int k = get_k(context);
  const T ctheta = cos(theta), stheta = sin(theta);
  const Vector2<T> c1 = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                        half_length_);
  const Vector2<T> c2 = CalcRodEndpoint(x, y, k, ctheta, stheta,
                                        half_length_);

  // Call the appropriate contact force computation method.
  Vector2<T> fN, fF;
  if (abs(xdot) < std::numeric_limits<double>::epsilon()) {
    CalcTwoContactNoSlidingForces(context, &fN, &fF);
  } else {
    CalcTwoContactSlidingForces(context, &fN, &fF);
  }

  SetAccelerations(context, fN, fF, c1, c2, &f);
}

template <class T>
bool Rod2D<T>::IsImpacting(const systems::Context<T>& context) const {
  using std::sin;
  using std::cos;

  // Note: we do not consider modes here.
  // TODO(edrumwri): Handle two-point contacts.

  // Get state data necessary to compute the point of contact.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& y = state.GetAtIndex(1);
  const T& theta = state.GetAtIndex(2);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Get the height of the lower rod endpoint.
  const T ctheta = cos(theta), stheta = sin(theta);
  const int k = (stheta > 0) ? -1 : (stheta < 0) ? 1 : 0;
  const T cy = y + k * stheta * half_length_;

  // If rod endpoint is not touching, there is no impact.
  if (cy >= 10 * std::numeric_limits<double>::epsilon()) return false;

  // Compute the velocity at the point of contact.
  const T cydot = ydot + k * ctheta * half_length_ * thetadot;

  // Verify that the rod is not impacting.
  return (cydot < -10 * std::numeric_limits<double>::epsilon());
}

// Computes the accelerations of the rod center of mass for the rod, both
// in compliant contact and contact-free.
template <typename T>
void Rod2D<T>::CalcAccelerationsCompliantContactAndBallistic(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the structure we need to write into (ds=d/dt state).
  systems::VectorBase<T>& ds = derivatives->get_mutable_vector();

  // Get external applied force (a spatial force at Ro, in W).
  const auto Fext_Ro_W = ComputeExternalForces(context);

  // Calculate contact forces (also spatial force at Ro, in W).
  const Vector3<T> Fc_Ro_W = CalcCompliantContactForces(context);
  const Vector3<T> F_Ro_W = Fc_Ro_W + Fext_Ro_W;  // Total force.

  // Second three derivative components are acceleration due to gravity,
  // contact forces, and non-gravitational, non-contact external forces.
  ds.SetAtIndex(3, F_Ro_W[0]/mass_);
  ds.SetAtIndex(4, F_Ro_W[1]/mass_);
  ds.SetAtIndex(5, F_Ro_W[2]/J_);
}

// Computes the accelerations of the rod center of mass for the case of
// ballistic motion.
template <typename T>
void Rod2D<T>::CalcAccelerationsBallistic(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the structure we need to write into (ds=d/dt state).
  systems::VectorBase<T>& ds = derivatives->get_mutable_vector();

  const Vector3<T> fext = ComputeExternalForces(context);

  // Second three derivative components are acceleration due to gravity and
  // external forces.
  ds.SetAtIndex(3, fext(0)/mass_);
  ds.SetAtIndex(4, fext(1)/mass_);
  ds.SetAtIndex(5, fext(2)/J_);
}

template <typename T>
void Rod2D<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::sin;
  using std::cos;
  using std::abs;

  // Don't compute any derivatives if this is the time stepping system.
  if (simulation_type_ == SimulationType::kTimeStepping) {
    DRAKE_ASSERT(derivatives->size() == 0);
    return;
  }

  // Get the necessary parts of the state.
  const systems::VectorBase<T>& state = context.get_continuous_state_vector();
  const T& xdot = state.GetAtIndex(3);
  const T& ydot = state.GetAtIndex(4);
  const T& thetadot = state.GetAtIndex(5);

  // Obtain the structure we need to write into.
  systems::VectorBase<T>& f = derivatives->get_mutable_vector();

  // First three derivative components are xdot, ydot, thetadot.
  f.SetAtIndex(0, xdot);
  f.SetAtIndex(1, ydot);
  f.SetAtIndex(2, thetadot);

  // Compute the velocity derivatives (accelerations).
  if (simulation_type_ == SimulationType::kCompliant) {
    return CalcAccelerationsCompliantContactAndBallistic(context, derivatives);
  } else {
    // (Piecewise DAE approach follows).
    // Get the abstract variables that determine the current system mode and
    // the endpoint in contact.
    const Mode mode = context.template get_abstract_state<Mode>(0);

    // Call the proper derivative function (depending on mode type).
    switch (mode) {
      case kBallisticMotion:
        return CalcAccelerationsBallistic(context,
                                          derivatives);
      case kSlidingSingleContact:
        return CalcAccelerationsOneContactSliding(context,
                                                  derivatives);
      case kStickingSingleContact:
        return CalcAccelerationsOneContactNoSliding(context,
                                                    derivatives);
      case kSlidingTwoContacts:
        return CalcAccelerationsTwoContact(context,
                                           derivatives);
      case kStickingTwoContacts:
        return CalcAccelerationsTwoContact(context,
                                           derivatives);

      default:DRAKE_ABORT_MSG("Invalid mode detected");
    }
  }
}

/// Allocates the abstract state (for piecewise DAE systems).
template <typename T>
std::unique_ptr<systems::AbstractValues> Rod2D<T>::AllocateAbstractState()
    const {
  if (simulation_type_ == SimulationType::kPiecewiseDAE) {
    // Piecewise DAE approach needs two abstract variables: one mode and one
    // contact point indicator.
    std::vector<std::unique_ptr<systems::AbstractValue>> abstract_data;
    abstract_data.push_back(
        std::make_unique<systems::Value<Rod2D<T>::Mode>>(
            Rod2D<T>::kInvalid));

    // Indicates that the rod is in contact at both points.
    abstract_data.push_back(std::make_unique<systems::Value<int>>(0));
    return std::make_unique<systems::AbstractValues>(std::move(abstract_data));
  } else {
    // Time stepping and compliant approaches need no abstract variables.
    return std::make_unique<systems::AbstractValues>();
  }
}

/// Sets the rod to a 45 degree angle with the halfspace and positions the rod
/// such that it and the halfspace are touching at exactly one point of contact.
template <typename T>
void Rod2D<T>::SetDefaultState(const systems::Context<T>&,
                                  systems::State<T>* state) const {
  using std::sqrt;
  using std::sin;

  // Initial state corresponds to an inconsistent configuration for piecewise
  // DAE.
  const double half_len = get_rod_half_length();
  VectorX<T> x0(6);
  const double r22 = sqrt(2) / 2;
  x0 << half_len * r22, half_len * r22, M_PI / 4.0, -1, 0, 0;  // Initial state.
  if (simulation_type_ == SimulationType::kTimeStepping) {
    state->get_mutable_discrete_state().get_mutable_vector(0)
        .SetFromVector(x0);
  } else {
    // Continuous variables.
    state->get_mutable_continuous_state().SetFromVector(x0);

    // Set abstract variables for piecewise DAE approach.
    if (simulation_type_ == SimulationType::kPiecewiseDAE) {
      // Indicate that the rod is in the single contact sliding mode.
      state->get_mutable_abstract_state()
          .get_mutable_value(0)
          .template GetMutableValue<Rod2D<T>::Mode>() =
          Rod2D<T>::kSlidingSingleContact;

      // Determine and set the point of contact.
      const double theta = x0(2);
      const int k = (sin(theta) > 0) ? -1 : 1;
      state->get_mutable_abstract_state().get_mutable_value(1)
          .template GetMutableValue<int>() = k;
    }
  }
}

// Converts a state vector to a rendering PoseVector.
template <class T>
void Rod2D<T>::ConvertStateToPose(const VectorX<T>& state,
                                  systems::rendering::PoseVector<T>* pose) {
  // Converts the configuration of the rod to a pose, accounting for both
  // the change to a y+ up coordinate system and the fact that Drake's cylinder
  // up-direction defaults to +z.
  const T theta = state[2] + M_PI_2;
  pose->set_translation(Eigen::Translation<T, 3>(state[0], 0, state[1]));
  const Vector3<T> y_axis{0, 1, 0};
  const Eigen::AngleAxis<T> rotation(theta, y_axis);
  pose->set_rotation(Eigen::Quaternion<T>(rotation));
}

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
