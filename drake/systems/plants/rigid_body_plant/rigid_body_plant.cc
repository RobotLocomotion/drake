#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/solvers/mathematical_program.h"
// TODO(amcastro-tri): parsers are not "plants" and should therefore be moved
// somewhere else. Maybe inside "multibody_dynamics/parsers" when that exists.
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/common/eigen_autodiff_types.h"

using std::make_unique;
using std::move;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::parsers::ModelInstanceIdTable;

namespace drake {
namespace systems {

template <typename T>
RbpPosesVector<T>::RbpPosesVector(int num_bodies) :
    BasicVector<T>(7 * num_bodies) {}

template <typename T>
RbpPosesVector<T>::~RbpPosesVector() {}

template <typename T>
int RbpPosesVector<T>::get_num_bodies() const {
  return this->size()/7;
}

// TODO(amcastro-tri): Output a quaternion map referencing the actual memory in
// BasicVector. However this is not possible right now given that Drake does not
// use the same memory layout as Eigen does. See issue #???? which needs to be
// resolved before we can fix this todo.
template <typename T>
Quaternion<T> RbpPosesVector<T>::get_body_orientation(int body_index) const {
  const int body_start = 7 * body_index;
  return Quaternion<T>(this->GetAtIndex(body_start + 3),
                       this->GetAtIndex(body_start + 0),
                       this->GetAtIndex(body_start + 1),
                       this->GetAtIndex(body_start + 2));
}

template <typename T>
Vector3<T> RbpPosesVector<T>::get_body_position(int body_index) const {
  const int body_start = 7 * body_index + 4;
  return Vector3<T>(this->GetAtIndex(body_start + 0),
                    this->GetAtIndex(body_start + 1),
                    this->GetAtIndex(body_start + 2));
}

template <typename T>
void RbpPosesVector<T>::set_body_orientation(int body_index,
                                             const Quaternion<T>& q) {
  const int body_start = 7 * body_index;
  this->get_mutable_value().template segment<4>(body_start) = q.coeffs();
}

template <typename T>
void RbpPosesVector<T>::set_body_position(int body_index,
                                          const Vector3<T>& p) {
  const int body_start = 7 * body_index + 4;
  this->get_mutable_value().template segment<3>(body_start) = p;
}

template <typename T>
RbpPosesVector<T>* RbpPosesVector<T>::DoClone() const {
  auto poses = new RbpPosesVector<T>(get_num_bodies());
  poses->get_mutable_value() = this->get_value();
  return poses;
}

template <typename T>
RigidBodyPlant<T>::RigidBodyPlant(std::unique_ptr<const RigidBodyTree> tree) :
    tree_(move(tree)) {
  // The input to the system is the generalized forces on the actuators.
  // TODO(amcastro-tri): add separate input ports for each model_id.
  System<T>::DeclareInputPort(
      kVectorValued, get_num_actuators(), kContinuousSampling);
  // The output to the system is the state vector.
  // TODO(amcastro-tri): add separate output ports for each model_id.
  System<T>::DeclareOutputPort(
      kVectorValued, get_num_states(), kContinuousSampling);
  // Declares an vector valued vector port for each rigid body pose.
  // A semantically richer vector of type RbpPosesVector is allocated by the
  // RigidBodyPlant<T>::AllocateOutput method.
  System<T>::DeclareOutputPort(
      kVectorValued, 7 * get_num_bodies(), kContinuousSampling);
}

template <typename T>
RigidBodyPlant<T>::~RigidBodyPlant() { }

template <typename T>
bool RigidBodyPlant<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
const RigidBodyTree& RigidBodyPlant<T>::get_multibody_world() const {
  return *tree_.get();
}

template <typename T>
int RigidBodyPlant<T>::get_num_bodies() const {
  return tree_->get_number_of_bodies();
}

template <typename T>
int RigidBodyPlant<T>::get_num_positions() const {
  return tree_->number_of_positions();
}

template <typename T>
int RigidBodyPlant<T>::get_num_velocities() const {
  return tree_->number_of_velocities();
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
void RigidBodyPlant<T>::set_position(Context<T>* context,
                                     int position_index, T position) const {
  DRAKE_ASSERT(context != nullptr);
  context->get_mutable_state()->continuous_state->
      get_mutable_generalized_position()->SetAtIndex(position_index, position);
}

template <typename T>
void RigidBodyPlant<T>::set_velocity(Context<T>* context,
                                     int velocity_index, T velocity) const {
  DRAKE_ASSERT(context != nullptr);
  context->get_mutable_state()->continuous_state->
      get_mutable_generalized_velocity()->SetAtIndex(velocity_index, velocity);
}

template <typename T>
void RigidBodyPlant<T>::set_state_vector(
    Context<T>* context, const Eigen::Ref<const VectorX<T>> x) const {
  DRAKE_ASSERT(context != nullptr);
  DRAKE_ASSERT(x.size() == get_num_states());
  context->get_mutable_state()->continuous_state->
      get_mutable_state()->SetFromVector(x);
}

template <typename T>
std::unique_ptr<SystemOutput<T>> RigidBodyPlant<T>::AllocateOutput(
    const Context<T>& context) const {
  auto output = make_unique<LeafSystemOutput<T>>();
  // Allocates output for the RigidBodyPlant state (output port 0).
  {
    auto data = make_unique<BasicVector<T>>(get_num_states());
    auto port = make_unique<OutputPort>(move(data));
    output->get_mutable_ports()->push_back(move(port));
  }

  // Allocates output for the RigidBodyPlant poses (output port 1).
  {
    std::unique_ptr<BasicVector<T>> data(
        new RbpPosesVector<T>(get_num_bodies()));
    std::unique_ptr<OutputPort> port(new OutputPort(move(data)));
    output->get_mutable_ports()->push_back(move(port));
  }
  return std::unique_ptr<SystemOutput<T>>(output.release());
}

template <typename T>
std::unique_ptr<ContinuousState<T>>
RigidBodyPlant<T>::AllocateContinuousState() const {
  // The state is second-order.
  DRAKE_ASSERT(System<T>::get_input_port(0).get_size() == get_num_actuators());
  // TODO(amcastro-tri): add z state to track energy conservation.
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicVector<T>>(get_num_states()),
      get_num_positions() /* num_q */,
      get_num_velocities() /* num_v */, 0 /* num_z */);
}

template <typename T>
void RigidBodyPlant<T>::EvalOutput(const Context<T>& context,
                                   SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // Evaluates port 0 to be the state of the system.
  BasicVector<T>* output_vector = output->GetMutableVectorData(0);
  // TODO(amcastro-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  output_vector->get_mutable_value() =
      context.get_continuous_state().CopyToVector();

  // Evaluates port 1 to be the poses for each rigid body.
  auto poses_vector =
      dynamic_cast<RbpPosesVector<T>*>(output->GetMutableVectorData(1));
  auto cache = InstantiateKinematicsCache(context);

  for (int ibody = 0; ibody < get_num_bodies(); ++ibody) {
    Isometry3<T> pose = tree_->relativeTransform(cache, 0, ibody);
    Vector4<T> quat_vector = drake::math::rotmat2quat(pose.linear());
    Vector3<T> position = pose.translation();
    poses_vector->set_body_position(ibody, position);
    poses_vector->set_body_orientation(ibody,
        Quaternion<T>(
            quat_vector[0], quat_vector[1], quat_vector[2], quat_vector[3]));
  }
}

template <typename T>
KinematicsCache<T> RigidBodyPlant<T>::InstantiateKinematicsCache(
    const Context<T> &context) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x = dynamic_cast<const BasicVector<T> &>(
      context.get_state().continuous_state->get_state()).get_value();

  const int nq = get_num_positions();
  const int nv = get_num_velocities();

  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block which
  // is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  return tree_->doKinematics(q, v);
}


template <typename T>
void RigidBodyPlant<T>::EvalTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  DRAKE_DEMAND(derivatives != nullptr);
  const BasicVector<T>* input = context.get_vector_input(0);

  // The input vector of actuation values.
  auto u = input->get_value();

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x = dynamic_cast<const BasicVector<T>&>(
      context.get_state().continuous_state->get_state()).get_value();

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
  const RigidBodyTree::BodyToWrenchMap<T> no_external_wrenches;
  // right_hand_side is the right hand side of the system's equations:
  // [H, -J^T] * [vdot; f] = -right_hand_side.
  VectorX<T> right_hand_side = tree_->dynamicsBiasTerm(kinsol,
                                                       no_external_wrenches);
  if (num_actuators > 0) right_hand_side -= tree_->B * u;

  // Applies joint limit forces.
  // TODO(amcastro-tri): Maybe move to
  // RBT::ComputeGeneralizedJointLimitForces(C)?
  {
    for (auto const& b : tree_->bodies) {
      if (!b->has_parent_body()) continue;
      auto const& joint = b->getJoint();
      // Only for single-axis joints.
      if (joint.getNumPositions() == 1 && joint.getNumVelocities() == 1) {
        // Limits makes things easier/faster here.
        T qmin = joint.getJointLimitMin()(0),
            qmax = joint.getJointLimitMax()(0);
        // tau = k * (qlimit-q) - b(qdot)
        if (q(b->get_position_start_index()) < qmin)
          right_hand_side(b->get_velocity_start_index()) -=
              penetration_stiffness_ * (qmin - q(b->get_position_start_index()))
                  - penetration_damping_ * v(b->get_velocity_start_index());
        else if (q(b->get_position_start_index()) > qmax)
          right_hand_side(b->get_velocity_start_index()) -=
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
    // Unfortunately collisionDetect() modifies the collision model in the RBT
    // when updating the collision element poses.
    const_cast<RigidBodyTree*>(tree_.get())->collisionDetect(
        kinsol, phi, normal, xA, xB, bodyA_idx, bodyB_idx);

    for (int i = 0; i < phi.rows(); i++) {
      if (phi(i) < 0.0) {  // There is contact.
        auto JA = tree_->transformPointsJacobian(
            kinsol, xA.col(i), bodyA_idx[i], 0, false);
        auto JB = tree_->transformPointsJacobian(
            kinsol, xB.col(i), bodyB_idx[i], 0, false);
        Vector3<T> this_normal = normal.col(i);

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
        auto J = R * (JA - JB);          // J = [ D1; D2; n ]
        auto relative_velocity = J * v;  // [ tangent1dot; tangent2dot; phidot ]

        {
          // Spring law for normal force:  fA_normal = -k * phi - b * phidot
          // and damping for tangential force:  fA_tangent = -b * tangentdot
          // (bounded by the friction cone).
          Vector3<T> fA;
          fA(2) = std::max<T>(
              -penetration_stiffness_ * phi(i) -
               penetration_damping_ * relative_velocity(2), 0.0);
          fA.head(2) =
              -std::min<T>(
                  penetration_damping_,
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
          right_hand_side -= J.transpose() * fA;
        }
      }
    }
  }

  if (tree_->getNumPositionConstraints()) {
    size_t nc = tree_->getNumPositionConstraints();
    // 1/time constant of position constraint satisfaction.
    const T alpha = 5.0;

    prog.AddContinuousVariables(
        nc, "position constraint force");

    auto phi = tree_->positionConstraints(kinsol);
    auto J = tree_->positionConstraintsJacobian(kinsol, false);
    auto Jdotv = tree_->positionConstraintsJacDotTimesV(kinsol);

    // Critically damped stabilization term.
    // phiddot = -2 * alpha * phidot - alpha^2 * phi.
    prog.AddLinearEqualityConstraint(
        J, -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi), {vdot});
    H_and_neg_JT.conservativeResize(
        Eigen::NoChange, H_and_neg_JT.cols() + J.rows());
    H_and_neg_JT.rightCols(J.rows()) = -J.transpose();
  }

  // Adds [H,-J^T] * [vdot;f] = -C.
  prog.AddLinearEqualityConstraint(H_and_neg_JT, -right_hand_side);

  prog.Solve();

  VectorX<T> xdot(get_num_states());
  xdot << kinsol.transformPositionDotMappingToVelocityMapping(
      MatrixX<T>::Identity(nq, nq)) * v, vdot.value();

  derivatives->get_mutable_state()->SetFromVector(xdot);
}

template <typename T>
void RigidBodyPlant<T>::MapVelocityToConfigurationDerivatives(
    const Context<T>& context,
    const VectorBase<T>& generalized_velocity,
    VectorBase<T>* positions_derivative) const {
  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x = dynamic_cast<const BasicVector<T>&>(
      context.get_state().continuous_state->get_state()).get_value();

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int nstates = get_num_states();

  DRAKE_ASSERT(positions_derivative->size() == nq);
  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_ASSERT(x.size() == nstates);

  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block which
  // is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = generalized_velocity.CopyToVector();

  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_->doKinematics(q, v);

  positions_derivative->SetFromVector(
      kinsol.transformPositionDotMappingToVelocityMapping(
          MatrixX<T>::Identity(nq, nq)) * v);
}

// Explicitly instantiates on the most common scalar types.
template class DRAKE_RBP_EXPORT RigidBodyPlant<double>;
template class DRAKE_RBP_EXPORT RbpPosesVector<double>;

}  // namespace systems
}  // namespace drake
