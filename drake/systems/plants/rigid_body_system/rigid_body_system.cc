#include "drake/systems/plants/rigid_body_system/rigid_body_system.h"

#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/solvers/optimization.h"
#include "drake/systems/framework/basic_state_vector.h"
// TODO(amcastro-tri): parsers are not "plants" and should therefore be moved
// somewhere else. Maybe inside "multibody_dynamics/parsers" when that exists.
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/common/eigen_autodiff_types.h"

using std::move;
using std::string;
using std::vector;

using drake::parsers::ModelInstanceIdTable;

namespace drake {
namespace systems {

template <typename T>
RigidBodySystem<T>::RigidBodySystem(std::unique_ptr<const RigidBodyTree> mbd_world) :
    mbd_world_(move(mbd_world)) {

  // The input to the system is the generalized forces on the actuators.
  System<T>::DeclareInputPort(
      kVectorValued, get_num_actuators(), kContinuousSampling);
  // The output to the system is the state vector.
  System<T>::DeclareOutputPort(
      kVectorValued, get_num_states(), kContinuousSampling);

}

template <typename T>
RigidBodySystem<T>::~RigidBodySystem() { }

template <typename T>
bool RigidBodySystem<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
const RigidBodyTree& RigidBodySystem<T>::get_multibody_world() const {
  return *mbd_world_.get();
}

template <typename T>
int RigidBodySystem<T>::get_num_generalized_positions() const {
  return mbd_world_->number_of_positions();
}

template <typename T>
int RigidBodySystem<T>::get_num_generalized_velocities() const {
  return mbd_world_->number_of_velocities();
}

template <typename T>
int RigidBodySystem<T>::get_num_states() const {
  return get_num_generalized_positions() + get_num_generalized_velocities();
}

template <typename T>
int RigidBodySystem<T>::get_num_actuators() const {
  return mbd_world_->actuators.size();
}

template <typename T>
int RigidBodySystem<T>::get_num_inputs() const {
  return get_num_actuators();
}

template <typename T>
int RigidBodySystem<T>::get_num_outputs() const {
  return get_num_states();
}

template <typename T>
std::unique_ptr<ContinuousState<T>>
RigidBodySystem<T>::AllocateContinuousState() const {
  // The state is second-order.
  DRAKE_ASSERT(System<T>::get_input_port(0).get_size() == get_num_actuators());
  // TODO(amcastro-tri): add z state to track energy conservation.
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicStateVector<T>>(get_num_states()),
      get_num_generalized_positions() /* num_q */,
      get_num_generalized_velocities() /* num_v */, 0 /* num_z */);
}

template <typename T>
void RigidBodySystem<T>::EvalOutput(const ContextBase<T>& context,
                                    SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  VectorBase<T>* output_vector = output->GetMutableVectorData(0);

  // TODO(amcastro-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  output_vector->get_mutable_value() = context.get_xc().CopyToVector();
}

template <typename T>
void RigidBodySystem<T>::EvalTimeDerivatives(
    const ContextBase<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  const VectorBase<T>* input = context.get_vector_input(0);

  // The input vector of actuation values.
  auto u = input->get_value();

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x = dynamic_cast<const BasicStateVector<T>&>(
      context.get_state().continuous_state->get_state()).get_vector();

  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  int nq = get_num_generalized_positions();
  int nv = get_num_generalized_velocities();
  int num_actuators = get_num_actuators();
  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block which
  // is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinsol = mbd_world_->doKinematics(q, v);

  // TODO(amcastro-tri): preallocate the optimization problem and constraints,
  // and simply update them then solve on each function eval.
  // How to place something like this in the context?
  drake::solvers::OptimizationProblem prog;
  auto const& vdot = prog.AddContinuousVariables(nv, "vdot");

  auto H = mbd_world_->massMatrix(kinsol);
  Eigen::MatrixXd H_and_neg_JT = H;

  // f_ext here has zero size but is a required argument in dynamicsBiasTerm.
  eigen_aligned_unordered_map<const RigidBody*, Vector6<T>> f_ext;
  VectorX<T> C = mbd_world_->dynamicsBiasTerm(kinsol, f_ext);
  if (num_actuators > 0) C -= mbd_world_->B * u;

  // Applies joint limit forces.
  // TODO(amcastro-tri): Maybe move to
  // RBT::ComputeGeneralizedJointLimitForces(C)?
  {
    for (auto const& b : mbd_world_->bodies) {
      if (!b->hasParent()) continue;
      auto const& joint = b->getJoint();
      if (joint.getNumPositions() == 1 &&
          joint.getNumVelocities() ==
              1) {  // taking advantage of only single-axis joints having joint
        // limits makes things easier/faster here
        T qmin = joint.getJointLimitMin()(0),
            qmax = joint.getJointLimitMax()(0);
        // tau = k*(qlimit-q) - b(qdot)
        if (q(b->get_position_start_index()) < qmin)
          C(b->get_velocity_start_index()) -=
              penetration_stiffness_ * (qmin - q(b->get_position_start_index()))
                  - penetration_damping_ * v(b->get_velocity_start_index());
        else if (q(b->get_position_start_index()) > qmax)
          C(b->get_velocity_start_index()) -=
              penetration_stiffness_ * (qmax - q(b->get_position_start_index()))
                  - penetration_damping_ * v(b->get_velocity_start_index());
      }
    }
  }

  // Applies contact forces.
  // TODO(amcastro-tri): Maybe move to RBT::ComputeGeneralizedContactForces(C)?
  {
    VectorX<T> phi;
    Matrix3X<T> normal, xA, xB;
    vector<int> bodyA_idx, bodyB_idx;

    // TODO(amcastro-tri): get rid of this const_cast.
    // Unfortunately collisionDetect modifies the collision model in the RBT
    // when updating the collision element poses.
    const_cast<RigidBodyTree*>(mbd_world_.get())->collisionDetect(
        kinsol, phi, normal, xA, xB, bodyA_idx, bodyB_idx);

    for (int i = 0; i < phi.rows(); i++) {
      if (phi(i) < 0.0) {  // then i have contact
        // todo: move this entire block to a shared an updated "contactJacobian"
        // method in RigidBodyTree
        auto JA = mbd_world_->transformPointsJacobian(
            kinsol, xA.col(i), bodyA_idx[i], 0, false);
        auto JB = mbd_world_->transformPointsJacobian(
            kinsol, xB.col(i), bodyB_idx[i], 0, false);
        Vector3<T> this_normal = normal.col(i);

        // Computes the surface tangent basis
        Vector3<T> tangent1;
        if (1.0 - this_normal(2) < EPSILON) {  // handle the unit-normal case
          // (since it's unit length, just
          // check z)
          tangent1 << 1.0, 0.0, 0.0;
        } else if (1 + this_normal(2) < EPSILON) {
          tangent1 << -1.0, 0.0, 0.0;  // same for the reflected case
        } else {                       // now the general case
          tangent1 << this_normal(1), -this_normal(0), 0.0;
          tangent1 /= sqrt(this_normal(1) * this_normal(1) +
              this_normal(0) * this_normal(0));
        }
        Vector3<T> tangent2 = this_normal.cross(tangent1);
        Matrix3<T> R;  // rotation into normal coordinates
        R.row(0) = tangent1;
        R.row(1) = tangent2;
        R.row(2) = this_normal;
        auto J = R * (JA - JB);          // J = [ D1; D2; n ]
        auto relative_velocity = J * v;  // [ tangent1dot; tangent2dot; phidot ]

        {
          // spring law for normal force:  fA_normal = -k*phi - b*phidot
          // and damping for tangential force:  fA_tangent = -b*tangentdot
          // (bounded by the friction cone)
          Vector3<T> fA;
          fA(2) =
              std::max<T>(-penetration_stiffness_ * phi(i) -
                                penetration_damping_ * relative_velocity(2),
                               0.0);
          fA.head(2) =
              -std::min<T>(
                  penetration_damping_,
                  friction_coefficient_ * fA(2) /
                      (relative_velocity.head(2).norm() + EPSILON)) *
                  relative_velocity.head(2);  // epsilon to avoid divide by zero

          // equal and opposite: fB = -fA.
          // tau = (R*JA)^T fA + (R*JB)^T fB = J^T fA
          C -= J.transpose() * fA;
        }
      }
    }
  }

  if (mbd_world_->getNumPositionConstraints()) {
    size_t nc = mbd_world_->getNumPositionConstraints();
    const T alpha = 5.0;  // 1/time constant of position constraint
    // satisfaction (see my latex rigid body notes)

    prog.AddContinuousVariables(
        nc, "position constraint force");

    // then compute the constraint force
    auto phi = mbd_world_->positionConstraints(kinsol);
    auto J = mbd_world_->positionConstraintsJacobian(kinsol, false);
    auto Jdotv = mbd_world_->positionConstraintsJacDotTimesV(kinsol);

    // phiddot = -2 alpha phidot - alpha^2 phi  (0 + critically damped
    // stabilization term)
    prog.AddLinearEqualityConstraint(
        J, -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi), {vdot});
    H_and_neg_JT.conservativeResize(
        Eigen::NoChange, H_and_neg_JT.cols() + J.rows());
    H_and_neg_JT.rightCols(J.rows()) = -J.transpose();
  }

  // add [H,-J^T]*[vdot;f] = -C
  prog.AddLinearEqualityConstraint(H_and_neg_JT, -C);

  prog.Solve();

  VectorX<T> xdot(get_num_states());
  xdot << kinsol.transformPositionDotMappingToVelocityMapping(
      MatrixX<T>::Identity(nq, nq)) * v, vdot.value();

  derivatives->get_mutable_state()->SetFromVector(xdot);
}

// Explicitly instantiates on the most common scalar types.
template class DRAKE_RBS_EXPORT RigidBodySystem<double>;
//template class DRAKESYSTEMFRAMEWORK_EXPORT Gain<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
