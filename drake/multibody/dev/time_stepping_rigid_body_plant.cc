#include "drake/multibody/dev/time_stepping_rigid_body_plant.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
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

  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");

  // Verify that the time-step is strictly positive.
  if (timestep <= 0.0) {
    throw std::logic_error("TimeSteppingRigidBodyPlant requires a positive "
                               "integration time step.");
  }

  // Schedule the time stepping.
  this->DeclarePeriodicDiscreteUpdate(timestep);
}

// Retrieves contact stiffness and damping coefficients for a pair of
// geometries.
template <class T>
void TimeSteppingRigidBodyPlant<T>::CalcContactStiffnessAndDamping(
    const drake::multibody::collision::PointPair&,
    double*,
    double*) const {
  DRAKE_ABORT_MSG("CalcContactStiffnessAndDamping() not yet implemented.");
}

// Gets A's translational velocity relative to B's translational velocity at a
// point common to the two rigid bodies.
// @param p_W The point of contact (defined in the world frame).
// @returns the relative velocity at p_W expressed in the world frame.
template <class T>
Vector3<T> TimeSteppingRigidBodyPlant<T>::CalcRelTranslationalVelocity(
    const KinematicsCache<T>&, int, int,
    const Vector3<T>&) const {
  DRAKE_ABORT_MSG("CalcRelTranslationalVelocity() not yet implemented.");
  return Vector3<T>::Zero();
}

// Updates a generalized force from a force of f (expressed in the world frame)
// applied at point p (defined in the global frame).
template <class T>
void TimeSteppingRigidBodyPlant<T>::UpdateGeneralizedForce(
    const KinematicsCache<T>&, int, int,
    const Vector3<T>&, const Vector3<T>&, VectorX<T>*) const {
  DRAKE_ABORT_MSG("UpdateGeneralizedForce() not yet implemented.");
}

// Evaluates the relative velocities between two bodies projected along the
// contact normals.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::N_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>&,
    const VectorX<T>&) const {
  if (!contacts.empty())
    DRAKE_ABORT_MSG("N_mult() not yet implemented.");
  return VectorX<T>::Zero(contacts.size());
}

// Applies forces along the contact normals at the contact points and gets the
// effect out on the generalized forces.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::N_transpose_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kcache,
    const VectorX<T>&) const {
  if (!contacts.empty())
    DRAKE_ABORT_MSG("N_transpose_mult() not yet implemented.");
  return VectorX<T>::Zero(kcache.getV().size());
}

// Evaluates the relative velocities between two bodies projected along the
// contact tangent directions.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::F_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>&,
    const VectorX<T>&) const {
  if (!contacts.empty())
    DRAKE_ABORT_MSG("F_mult() not yet implemented.");
  return VectorX<T>::Zero(contacts.size() * half_cone_edges_);
}

// Applies a force at the contact spanning directions at all contacts and gets
// the effect out on the generalized forces.
template <class T>
VectorX<T> TimeSteppingRigidBodyPlant<T>::F_transpose_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kcache,
    const VectorX<T>&) const {
  if (!contacts.empty())
    DRAKE_ABORT_MSG("F_transpose_mult() not yet implemented.");
  return VectorX<T>::Zero(kcache.getV().size());
}

template <typename T>
void TimeSteppingRigidBodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  using std::abs;

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
  auto x = context.get_discrete_state(0).get_value();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kcache = tree.doKinematics(q, v);

  // Get the generalized inertia matrix and set up the inertia solve function.
  auto H = tree.massMatrix(kcache);

  // Compute the LDLT factorizations, which will be used by the solver.
  Eigen::LDLT<MatrixX<T>> ldlt(H);
  DRAKE_DEMAND(ldlt.info() == Eigen::Success);

  // Set the inertia matrix solver.
  data.solve_inertia = [&ldlt](const MatrixX<T>& m) {
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

  // TODO(edrumwri): Relax this assumption to allow contact constraints.
  DRAKE_DEMAND(contacts.empty());

  // Verify the friction directions are set correctly.
  DRAKE_DEMAND(half_cone_edges_ >= 2);

  // Set up the N multiplication operator (projected velocity along the contact
  // normals).
  data.N_mult = [this, &contacts, &kcache](const VectorX<T>& w) -> VectorX<T> {
    return N_mult(contacts, kcache, w);
  };

  // Set up the N' multiplication operator (effect of contact normal forces
  // on generalized forces).
  data.N_transpose_mult = [this, &contacts, &kcache](const VectorX<T>& f) ->
      VectorX<T> {
    return N_transpose_mult(contacts, kcache, f);
  };

  // Set up the F multiplication operator (projected velocity along the contact
  // tangent directions).
  data.F_mult = [this, &contacts, &kcache](const VectorX<T>& w) -> VectorX<T> {
    return F_mult(contacts, kcache, w);
  };

  // Set up the F' multiplication operator (effect of contact frictional forces
  // on generalized forces).
  data.F_transpose_mult = [this, &contacts, &kcache](const VectorX<T>& f) ->
      VectorX<T> {
    return F_transpose_mult(contacts, kcache, f);
  };

  // 1. Set the stabilization term for contact normal direction (kN)
  data.kN.resize(contacts.size());
  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    double stiffness, damping;
    CalcContactStiffnessAndDamping(contacts[i], &stiffness, &damping);
    // TODO(edrumwri): Use this to set cfm and erp parameters for contacts.
  }

  // 2. Set the stabilization term for contact tangent directions (kF).
  // TODO(edrumwri): Update 'total_friction_cone_edges' once contact constraints
  // supported.
  const int total_friction_cone_edges = 0;
  data.kF.setZero(total_friction_cone_edges);

  // 3. Set the stabilization term for joint limit constraints (kL).
  data.kL.resize(0);

  // Integrate the forces into the velocity.
  const VectorX<T> vprime = v + data.solve_inertia(right_hand_side) * dt;
  data.Mv = H * vprime;

  // Solve the rigid impact problem.
  VectorX<T> vnew, cf;
  constraint_solver_.SolveImpactProblem(data, &cf);
  constraint_solver_.ComputeGeneralizedVelocityChange(data, cf, &vnew);
  vnew += vprime;

  // qn = q + dt*qdot.
  VectorX<T> xn(this->get_num_states());
  xn << q + dt * tree.transformVelocityToQDot(kcache, vnew), vnew;
  updates->get_mutable_vector(0).SetFromVector(xn);
}

// Explicitly instantiates on the most common scalar types.
template class TimeSteppingRigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
