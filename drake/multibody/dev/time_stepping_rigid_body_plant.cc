#include "drake/multibody/dev/time_stepping_rigid_body_plant.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/constraint/constraint_problem_data.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/solvers/mathematical_program.h"

using std::make_unique;
using std::move;
using std::runtime_error;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::multibody::collision::ElementId;
using drake::multibody::constraint::ConstraintSolver;

namespace drake {
namespace systems {

template <typename T>
TimeSteppingRigidBodyPlant<T>::TimeSteppingRigidBodyPlant(
    std::unique_ptr<const RigidBodyTree<T>> tree, double timestep)
    : RigidBodyPlant<T>(std::move(tree), timestep) {

  // Verify that the time-step is strictly positive.
  if (timestep <= 0.0) {
    throw std::logic_error("TimeSteppingRigidBodyPlant requires a positive "
                               "integration time step.");
  }

  // Schedule the time stepping.
  this->DeclarePeriodicDiscreteUpdate(timestep);
}

// Gets A's translational velocity relative to B's translational velocity at a
// point common to the two rigid bodies.
// @param p_W The point of contact (defined in the world frame).
// @returns the relative velocity at p_W expressed in the world frame.
template <class T>
Vector3<T> TimeSteppingRigidBodyPlant<T>::CalcRelTranslationalVelocity(
    const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
    const Vector3<T>& p_W) const {
  DRAKE_DEMAND("CalcRelTranslationalVelocity() not yet implemented.");
}

// Updates a generalized force from a force of f (expressed in the world frame)
// applied at point p (defined in the global frame).
template <class T>
void TimeSteppingRigidBodyPlant<T>::UpdateGeneralizedForce(
    const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
    const Vector3<T>& p_W, const Vector3<T>& f, VectorX<T>* gf) const {
  DRAKE_DEMAND("UpdateGeneralizedForce() not yet implemented.");
}

// Evaluates the relative velocities between two bodies projected along the
// contact normals.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::N_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const VectorX<T>& q,
    const VectorX<T>& v) const {
  DRAKE_DEMAND("N_mult() not yet implemented.");
  return VectorX<T>::Zero(contacts.size());
}

// Applies forces along the contact normals at the contact points and gets the
// effect out on the generalized forces.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::N_transpose_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const VectorX<T>& q,
    const VectorX<T>& v,
    const VectorX<T>& f) const {
  DRAKE_DEMAND("N_transpose_mult() not yet implemented.");
  return VectorX<T>::Zero(v.size());
}

// Evaluates the relative velocities between two bodies projected along the
// contact tangent directions.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::F_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const VectorX<T>& q,
    const VectorX<T>& v) const {
  DRAKE_DEMAND("F_mult() not yet implemented.");
  return VectorX<T>::Zero(contacts.size() * half_cone_edges_);
}

// Applies a force at the contact spanning directions at all contacts and gets
// the effect out on the generalized forces.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::F_transpose_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const VectorX<T>& q,
    const VectorX<T>& v,
    const VectorX<T>& f) const {
  DRAKE_DEMAND("F_transpose_mult() not yet implemented.");
  return VectorX<T>::Zero(v.size());
}

template <typename T>
void TimeSteppingRigidBodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  using std::abs;
  std::vector<JointLimit> limits;

  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");

  // Get the time step.
  double dt = this->get_time_step();
  DRAKE_DEMAND(dt > 0.0);

  VectorX<T> u = this->EvaluateActuatorInputs(context);

  const int nq = this->get_num_positions();
  const int nv = this->get_num_velocities();
  const int num_actuators = this->get_num_actuators();

  // Initialize the velocity problem data.
  drake::multibody::constraint::ConstraintVelProblemData<T> data(nv);

  // Get the rigid body tree.
  const auto& tree = this->get_rigid_body_tree();

  // Get the system state.
  auto x = context.get_discrete_state(0)->get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kcache = tree.doKinematics(q, v);

  // Get the generalized inertia matrix and set up the inertia solve function.
  auto H = tree.massMatrix(kcache);

  // Compute the LDLT factorizations, which will be used by the solver.
  Eigen::LDLT<MatrixX<T>> ldlt(H);
  DRAKE_DEMAND(ldlt.info() == Eigen::Success);

  // Set the inertia matrix solver.
  data.solve_inertia = [this, &ldlt](const MatrixX<T>& m) {
    return ldlt.solve(m);
  };

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  // right_hand_side is the right hand side of the system's equations:
  //   right_hand_side = B*u - C(q,v)
  VectorX<T> right_hand_side =
      -tree.dynamicsBiasTerm(kcache, no_external_wrenches);
  if (num_actuators > 0) right_hand_side += tree.B * u;

  // Determine the set of contact points corresponding to the current q.
  std::vector<drake::multibody::collision::PointPair> contacts =
      const_cast<RigidBodyTree<T>*>(&tree)->ComputeMaximumDepthCollisionPoints(
          kcache, true);

  // Verify the friction directions are set correctly.
  DRAKE_DEMAND(half_cone_edges_ >= 2);

  // Set the coefficients of friction.
  data.mu.resize(contacts.size());
  data.r.resize(contacts.size());
  for (int i = 0; i < data.mu.rows(); ++i) {
    // TODO(edrumwri): Replace this with parsed mu once #7016 lands. 
    data.mu[i] = mu_;
    data.r[i] = half_cone_edges_;
  }
  const int total_friction_cone_edges = std::accumulate(
      data.r.begin(), data.r.end(), 0);

  // Set the joint range of motion limits.
  for (auto const& b : tree.bodies) {
    if (!b->has_parent_body()) continue;
    auto const& joint = b->getJoint();

    // Joint limit forces are only implemented for single-axis joints.
    if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
      const T qmin = joint.getJointLimitMin()(0);
      const T qmax = joint.getJointLimitMax()(0);
      DRAKE_DEMAND(qmin < qmax);

      // Get the current joint position and velocity.
      const T& qjoint = q(b->get_position_start_index());
      const T& vjoint = v(b->get_velocity_start_index());

      // See whether the joint is beyond its limit or whether the *current*
      // joint velocity might lead to a limit violation.
      if (qjoint < qmin || qjoint + vjoint * dt < qmin) {
        // Institute a lower limit.
        limits.push_back(JointLimit());
        limits.back().v_index = b->get_velocity_start_index();
        limits.back().error = (qjoint - qmin);
        limits.back().lower_limit = true;
      }
      if (qjoint > qmax || qjoint + vjoint * dt > qmax) {
        // Institute an upper limit.
        limits.push_back(JointLimit());
        limits.back().v_index = b->get_velocity_start_index();
        limits.back().error = (qmax - qjoint);
        limits.back().lower_limit = false;
      }
    }
  }

  // Set the number of generic unilateral constraint functions.
  data.num_limit_constraints = limits.size();

  // Set up the N multiplication operator (projected velocity along the contact
  // normals).
  data.N_mult = [this, &contacts, &q](const VectorX<T>& w) -> VectorX<T> {
    return N_mult(contacts, q, w);
  };

  // Set up the N' multiplication operator (effect of contact normal forces
  // on generalized forces).
  data.N_transpose_mult = [this, &contacts, &q, &v](const VectorX<T>& f) ->
      VectorX<T> {
    return N_transpose_mult(contacts, q, v, f);
  };

  // Set up the F multiplication operator (projected velocity along the contact
  // tangent directions).
  data.F_mult = [this, &contacts, &q](const VectorX<T>& w) -> VectorX<T> {
    return F_mult(contacts, q, w);
  };

  // Set up the F' multiplication operator (effect of contact frictional forces
  // on generalized forces).
  data.F_transpose_mult = [this, &contacts, &q, &v](const VectorX<T>& f) ->
      VectorX<T> {
    return F_transpose_mult(contacts, q, v, f);
  };

  // Set the unilateral constraint operator (for evaluating joint velocities
  // at joint limit constraints).
  data.L_mult = [this, &limits](const VectorX<T>& w) -> VectorX<T> {
    VectorX<T> result(limits.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[i] = (limits[i].lower_limit) ? w[index] : -w[index];
    }
    return result;
  };

  // Set the unilateral constraint operator (for evaluating effect of forces 
  // at joint limits on generalized forces).
  data.L_transpose_mult = [this, &v, &limits](const VectorX<T>& lambda) {
    VectorX<T> result = VectorX<T>::Zero(v.size());
    for (int i = 0; static_cast<size_t>(i) < limits.size(); ++i) {
      const int index = limits[i].v_index;
      result[index] = (limits[i].lower_limit) ? lambda[i] : -lambda[i];
    }
    return result;
  };

  // Set the stabilization and softening terms for contact normal direction 
  // (kN and gammaN), contact tangent directions (kF and gammaF, gammaE),
  // joint limit constraints (kL and gammaL).
  data.kN.resize(contacts.size());
  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    double stiffness, damping;
    CalcStiffnessAndDamping(contacts[i], &stiffness, &damping);
    const double denom = dt * stiffness + damping;
    double contact_cfm = 1.0 / denom;
    double contact_erp = dt * stiffness / denom;
    data.kN[i] = contact_erp * contacts[i].distance / dt;
  }
  data.kF.setZero(total_friction_cone_edges);
  data.kL.resize(limits.size());
  for (int i = 0; i < static_cast<int>(limits.size()); ++i)
    data.kL[i] = erp_ * limits[i].error / dt;

  // Integrate the forces into the velocity.
  data.v = v + data.solve_inertia(right_hand_side) * dt;

  // Solve the rigid impact problem.
  VectorX<T> vnew, cf;
  constraint_solver_.SolveImpactProblem(cfm_, data, &cf);
  constraint_solver_.ComputeGeneralizedVelocityChange(data, cf, &vnew);
  vnew += data.v;

  // qn = q + h*qdot.
  VectorX<T> xn(this->get_num_states());
  xn << q + dt * tree.transformVelocityToQDot(kcache, vnew), vnew;
  updates->get_mutable_vector(0)->SetFromVector(xn);
}

// Explicitly instantiates on the most common scalar types.
template class TimeSteppingRigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
