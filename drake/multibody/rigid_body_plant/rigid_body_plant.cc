#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_plant/contact_resultant_force_calculator.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/collision/element.h"

using std::make_unique;
using std::move;
using std::string;
using std::unique_ptr;
using std::vector;

using DrakeCollision::ElementId;

namespace drake {
namespace systems {

template <typename T>
RigidBodyPlant<T>::RigidBodyPlant(std::unique_ptr<const RigidBodyTree<T>> tree)
    : tree_(move(tree)) {
  DRAKE_DEMAND(tree_ != nullptr);

  // The input to this system are the generalized forces commanded on the
  // actuators.
  // TODO(amcastro-tri): add separate input ports for each model_instance_id.
  System<T>::DeclareInputPort(kVectorValued, get_num_actuators(),
                              kContinuousSampling);
  // The output of the system is the state vector.
  // TODO(amcastro-tri): add separate output ports for each model_id.
  state_output_port_id_ =
      this->DeclareOutputPort(kVectorValued, get_num_states(),
                              kContinuousSampling)
          .get_index();
  // Declares an abstract valued port for kinematics results.
  kinematics_output_port_id_ =
      this->DeclareAbstractOutputPort(kInheritedSampling).get_index();
  contact_output_port_id_ =
      this->DeclareAbstractOutputPort(kInheritedSampling).get_index();
}

template <typename T>
RigidBodyPlant<T>::~RigidBodyPlant() {}

// TODO(liang.fok) Remove this method once a more advanced contact modeling
// framework is available.
template <typename T>
void RigidBodyPlant<T>::set_contact_parameters(double penetration_stiffness,
                                               double penetration_damping,
                                               double friction_coefficient) {
  penetration_stiffness_ = penetration_stiffness;
  penetration_damping_ = penetration_damping;
  friction_coefficient_ = friction_coefficient;
}

template <typename T>
bool RigidBodyPlant<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
const RigidBodyTree<T>& RigidBodyPlant<T>::get_rigid_body_tree() const {
  return *tree_.get();
}

template <typename T>
int RigidBodyPlant<T>::get_num_bodies() const {
  return tree_->get_num_bodies();
}

template <typename T>
int RigidBodyPlant<T>::get_num_positions() const {
  return tree_->get_num_positions();
}

template <typename T>
int RigidBodyPlant<T>::get_num_velocities() const {
  return tree_->get_num_velocities();
}

template <typename T>
int RigidBodyPlant<T>::get_num_states() const {
  return get_num_positions() + get_num_velocities();
}

template <typename T>
int RigidBodyPlant<T>::get_num_actuators() const {
  return tree_->actuators.size();
}

template <typename T>
int RigidBodyPlant<T>::get_input_size() const {
  return get_num_actuators();
}

template <typename T>
int RigidBodyPlant<T>::get_output_size() const {
  return get_num_states();
}

template <typename T>
void RigidBodyPlant<T>::set_position(Context<T>* context, int position_index,
                                     T position) const {
  DRAKE_ASSERT(context != nullptr);
  context->get_mutable_continuous_state()
      ->get_mutable_generalized_position()
      ->SetAtIndex(position_index, position);
}

template <typename T>
void RigidBodyPlant<T>::set_velocity(Context<T>* context, int velocity_index,
                                     T velocity) const {
  DRAKE_ASSERT(context != nullptr);
  context->get_mutable_continuous_state()
      ->get_mutable_generalized_velocity()
      ->SetAtIndex(velocity_index, velocity);
}

template <typename T>
void RigidBodyPlant<T>::set_state_vector(
    Context<T>* context, const Eigen::Ref<const VectorX<T>> x) const {
  DRAKE_ASSERT(context != nullptr);
  DRAKE_ASSERT(x.size() == get_num_states());
  context->get_mutable_continuous_state_vector()->SetFromVector(x);
}

template <typename T>
std::unique_ptr<SystemOutput<T>> RigidBodyPlant<T>::AllocateOutput(
    const Context<T>& context) const {
  auto output = make_unique<LeafSystemOutput<T>>();
  // Allocates an output for the RigidBodyPlant state (output port 0).
  {
    auto data = make_unique<BasicVector<T>>(get_num_states());
    auto port = make_unique<OutputPort>(move(data));
    output->get_mutable_ports()->push_back(move(port));
  }

  // Allocates an output for the RigidBodyPlant kinematics results
  // (output port 1).
  {
    auto kinematics_results = make_unique<Value<KinematicsResults<T>>>(
        KinematicsResults<T>(tree_.get()));
    output->add_port(move(kinematics_results));
  }

  // Allocates an output for the RigidBodyPlant contact results
  // (output port 2).
  {
    auto contact_results =
        make_unique<Value<ContactResults<T>>>(ContactResults<T>());
    output->add_port(move(contact_results));
  }

  return std::unique_ptr<SystemOutput<T>>(output.release());
}

template <typename T>
std::unique_ptr<ContinuousState<T>> RigidBodyPlant<T>::AllocateContinuousState()
    const {
  // The state is second-order.
  DRAKE_ASSERT(System<T>::get_input_port(0).get_size() == get_num_actuators());
  // TODO(amcastro-tri): add z state to track energy conservation.
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicVector<T>>(get_num_states()),
      get_num_positions() /* num_q */, get_num_velocities() /* num_v */,
      0 /* num_z */);
}

template <typename T>
void RigidBodyPlant<T>::EvalOutput(const Context<T>& context,
                                   SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Evaluates the state output port.
  BasicVector<T>* output_vector =
      output->GetMutableVectorData(state_output_port_id_);
  // TODO(amcastro-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  output_vector->get_mutable_value() =
      context.get_continuous_state()->CopyToVector();

  // Evaluates the kinematics results output port.
  auto& kinematics_results =
      output->GetMutableData(kinematics_output_port_id_)
          ->template GetMutableValue<KinematicsResults<T>>();
  kinematics_results.UpdateFromContext(context);

  // Evaluates the contact results output port.
  auto& contact_results = output->GetMutableData(contact_output_port_id_)
                              ->template GetMutableValue<ContactResults<T>>();
  ComputeContactResults(context, &contact_results);
}

template <typename T>
void RigidBodyPlant<T>::EvalTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  DRAKE_DEMAND(derivatives != nullptr);
  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(input);

  // The input vector of actuation values.
  auto u = input->get_value();

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int num_actuators = get_num_actuators();
  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block which
  // is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_->doKinematics(q, v);

  // TODO(amcastro-tri): preallocate the optimization problem and constraints,
  // and simply update them then solve on each function eval.
  // How to place something like this in the context?
  drake::solvers::MathematicalProgram prog;
  auto const& vdot = prog.AddContinuousVariables(nv, "vdot");

  auto H = tree_->massMatrix(kinsol);
  Eigen::MatrixXd H_and_neg_JT = H;

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm.
  // TODO(amcastro-tri): external_wrenches should be made an optional parameter
  // of dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
  // right_hand_side is the right hand side of the system's equations:
  // [H, -J^T] * [vdot; f] = -right_hand_side.
  VectorX<T> right_hand_side =
      tree_->dynamicsBiasTerm(kinsol, no_external_wrenches);
  if (num_actuators > 0) right_hand_side -= tree_->B * u;

  // Applies joint limit forces.
  // TODO(amcastro-tri): Maybe move to
  // RBT::ComputeGeneralizedJointLimitForces(C)?
  {
    for (auto const& b : tree_->bodies) {
      if (!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();
      // Joint limit forces are only implemented for single-axis joints.
      if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
        const T limit_force =
            JointLimitForce(joint, q(b->get_position_start_index()),
                            v(b->get_velocity_start_index()));
        right_hand_side(b->get_velocity_start_index()) -= limit_force;
      }
    }
  }

  right_hand_side -= ComputeContactForce(kinsol);

  if (tree_->getNumPositionConstraints()) {
    size_t nc = tree_->getNumPositionConstraints();
    // 1/time constant of position constraint satisfaction.
    const T alpha = 5.0;

    prog.AddContinuousVariables(nc, "position constraint force");

    auto phi = tree_->positionConstraints(kinsol);
    auto J = tree_->positionConstraintsJacobian(kinsol, false);
    auto Jdotv = tree_->positionConstraintsJacDotTimesV(kinsol);

    // Critically damped stabilization term.
    // phiddot = -2 * alpha * phidot - alpha^2 * phi.
    prog.AddLinearEqualityConstraint(
        J, -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi), {vdot});
    H_and_neg_JT.conservativeResize(Eigen::NoChange,
                                    H_and_neg_JT.cols() + J.rows());
    H_and_neg_JT.rightCols(J.rows()) = -J.transpose();
  }

  // Adds [H,-J^T] * [vdot;f] = -C.
  prog.AddLinearEqualityConstraint(H_and_neg_JT, -right_hand_side);

  prog.Solve();

  VectorX<T> xdot(get_num_states());
  xdot << kinsol.transformQDotMappingToVelocityMapping(
              MatrixX<T>::Identity(nq, nq)) *
              v,
      vdot.value();

  derivatives->SetFromVector(xdot);
}

template <typename T>
void RigidBodyPlant<T>::DoMapQDotToVelocity(
    const Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& configuration_dot,
    VectorBase<T>* generalized_velocity) const {
  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int nstates = get_num_states();

  DRAKE_ASSERT(configuration_dot.size() == nq);
  DRAKE_ASSERT(generalized_velocity->size() == nv);
  DRAKE_ASSERT(x.size() == nstates);

  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block which
  // is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);

  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_->doKinematics(q);

  generalized_velocity->SetFromVector(
      kinsol.transformQDotMappingToVelocityMapping(
          configuration_dot.transpose()));
}

template <typename T>
void RigidBodyPlant<T>::DoMapVelocityToQDot(
    const Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    VectorBase<T>* configuration_dot) const {
  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int nstates = get_num_states();

  DRAKE_ASSERT(configuration_dot->size() == nq);
  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_ASSERT(x.size() == nstates);

  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block which
  // is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = generalized_velocity;

  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_->doKinematics(q, v);

  configuration_dot->SetFromVector(
      kinsol.transformVelocityMappingToQDotMapping(v.transpose()));
}

template <typename T>
T RigidBodyPlant<T>::JointLimitForce(const DrakeJoint& joint, const T& position,
                                     const T& velocity) {
  const T qmin = joint.getJointLimitMin()(0);
  const T qmax = joint.getJointLimitMax()(0);
  DRAKE_DEMAND(qmin < qmax);
  const T joint_stiffness = joint.get_joint_limit_stiffness()(0);
  DRAKE_DEMAND(joint_stiffness >= 0);
  const T joint_dissipation = joint.get_joint_limit_dissipation()(0);
  DRAKE_DEMAND(joint_dissipation >= 0);
  if (position > qmax) {
    const T violation = position - qmax;
    const T limit_force =
        (-joint_stiffness * violation * (1 + joint_dissipation * velocity));
    return std::min(limit_force, 0.);
  } else if (position < qmin) {
    const T violation = position - qmin;
    const T limit_force =
        (-joint_stiffness * violation * (1 - joint_dissipation * velocity));
    return std::max(limit_force, 0.);
  }
  return 0;
}

template <typename T>
void RigidBodyPlant<T>::ComputeContactResults(
    const Context<T>& context, ContactResults<T>* contacts) const {
  DRAKE_ASSERT(contacts != nullptr);
  contacts->Clear();

  // TODO(SeanCurtis-TRI): This is horribly redundant code that only exists
  // because the data is not properly accessible in the cache.  This is
  // boilerplate drawn from EvalDerivatives.  See that code for further
  // comments
  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();
  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinsol = tree_->doKinematics(q, v);

  ComputeContactForce(kinsol, contacts);
}

template <typename T>
VectorX<T> RigidBodyPlant<T>::ComputeContactForce(
    const KinematicsCache<T>& kinsol, ContactResults<T>* contacts) const {
  std::vector<DrakeCollision::PointPair> pairs;

  // TODO(amcastro-tri): get rid of this const_cast.
  // Unfortunately collisionDetect() modifies the collision model in the RBT
  // when updating the collision element poses.
  const_cast<RigidBodyTree<T>*>(tree_.get())
      ->AllPairsClosestPoints(kinsol, &pairs);

  VectorX<T> contact_force(kinsol.getV().rows(), 1);
  contact_force.setZero();
  // TODO(SeanCurtis-TRI): Determine if a distance of zero should be reported
  //  as a zero-force contact.
  for (const auto& pair : pairs) {
    if (pair.distance < 0.0) {  // There is contact.
      int body_a_index = pair.elementA->get_body()->get_body_index();
      int body_b_index = pair.elementB->get_body()->get_body_index();
      auto JA = tree_->transformPointsJacobian(kinsol, pair.ptA, body_a_index,
                                               0, false);
      auto JB = tree_->transformPointsJacobian(kinsol, pair.ptB, body_b_index,
                                               0, false);
      Vector3<T> this_normal = pair.normal;

      // Computes a local surface coordinate frame with the local z axis
      // aligned with the surface's normal. The other two axes are arbitrarily
      // chosen to complete a right handed triplet.
      Vector3<T> tangent1;
      if (1.0 - this_normal(2) < EPSILON) {
        // Handles the unit-normal case. Since it's unit length, just check z.
        tangent1 << 1.0, 0.0, 0.0;
      } else if (1 + this_normal(2) < EPSILON) {
        tangent1 << -1.0, 0.0, 0.0;  // Same for the reflected case.
      } else {                       // Now the general case.
        tangent1 << this_normal(1), -this_normal(0), 0.0;
        tangent1 /= sqrt(this_normal(1) * this_normal(1) +
                         this_normal(0) * this_normal(0));
      }
      Vector3<T> tangent2 = this_normal.cross(tangent1);
      // Transformation from world frame to local surface frame.
      Matrix3<T> R;
      R.row(0) = tangent1;
      R.row(1) = tangent2;
      R.row(2) = this_normal;
      auto J = R * (JA - JB);  // J = [ D1; D2; n ]
      // [ tangent1dot; tangent2dot; phidot ]
      auto relative_velocity = J * kinsol.getV();

      {
        // Spring law for normal force:  fA_normal = -k * phi - b * phidot
        // and damping for tangential force:  fA_tangent = -b * tangentdot
        // (bounded by the friction cone).
        Vector3<T> fA;
        fA(2) = std::max<T>(-penetration_stiffness_ * pair.distance -
                                penetration_damping_ * relative_velocity(2),
                            0.0);
        fA.head(2) =
            -std::min<T>(penetration_damping_,
                         friction_coefficient_ * fA(2) /
                             (relative_velocity.head(2).norm() + EPSILON)) *
            relative_velocity.head(2);  // Epsilon avoids divide by zero.

        // fB is equal and opposite to fA: fB = -fA.
        // Therefore the generalized forces tau_c due to contact are:
        // tau_c = (R * JA)^T * fA + (R * JB)^T * fB = J^T * fA.
        // With J computed as above: J = R * (JA - JB).
        // Since right_hand_side has a negative sign when on the RHS of the
        // system of equations ([H,-J^T] * [vdot;f] + right_hand_side = 0),
        // this term needs to be subtracted.
        contact_force += J.transpose() * fA;
        if (contacts != nullptr) {
          Vector3<T> pt_a_world =
              kinsol.getElement(*pair.elementA->get_body()).transform_to_world *
              pair.ptA;
          Vector3<T> pt_b_world =
              kinsol.getElement(*pair.elementB->get_body()).transform_to_world *
              pair.ptB;
          Vector3<T> point = (pt_a_world + pt_b_world) * 0.5;

          ContactInfo<T>& contact_info = contacts->AddContact(
              pair.elementA->getId(), pair.elementB->getId());

          // TODO(SeanCurtis-TRI): Future feature: test against user-set flag
          // for whether the details should generally be captured or not and
          // make this function dependent.
          std::vector<unique_ptr<ContactDetail<T>>> details;
          ContactResultantForceCalculator<T> calculator(&details);

          // This contact model produces responses that only have a force
          // component (i.e., the torque portion of the wrench is zero.)
          // In contrast, other models (e.g., torsional friction model) can
          // also introduce a "pure torque" component to the wrench.
          Vector3<T> force = R.transpose() * fA;
          Vector3<T> normal = R.transpose().template block<3, 1>(0, 2);

          calculator.AddForce(point, normal, force);

          contact_info.set_resultant_force(calculator.ComputeResultant());
          // TODO(SeanCurtis-TRI): As with previous note, this line depends
          // on the eventual instantiation of the user-set flag for accumulating
          // contact details.
          contact_info.set_contact_details(move(details));
        }
      }
    }
  }
  return contact_force;
}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
