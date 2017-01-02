#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_plant/contact_resultant_force_calculator.h"
#include "drake/solvers/mathematical_program.h"

using std::make_unique;
using std::move;
using std::string;
using std::unique_ptr;
using std::vector;

using DrakeCollision::ElementId;

namespace drake {
namespace systems {
namespace {
// Constant used to indicate that a model instance doesn't have an
// input/output port associated with it.
const int kInvalidPortIdentifier = -1;
}  // namespace

template <typename T>
RigidBodyPlant<T>::RigidBodyPlant(std::unique_ptr<const RigidBodyTree<T>> tree)
    : tree_(move(tree)) {
  DRAKE_DEMAND(tree_ != nullptr);

  // The first input to this system is a vector containing the
  // generalized forces commanded on the actuators.
  System<T>::DeclareInputPort(kVectorValued, get_num_actuators());
  // The first output of the system is the state vector.
  state_output_port_id_ =
      this->DeclareOutputPort(kVectorValued, get_num_states()).get_index();
  // Declares an abstract valued port for kinematics results.
  kinematics_output_port_id_ = this->DeclareAbstractOutputPort().get_index();
  contact_output_port_id_ = this->DeclareAbstractOutputPort().get_index();

  const int num_instances = tree_->get_num_model_instances();
  const std::pair<int, int> default_entry =
      std::pair<int, int>(kInvalidPortIdentifier, 0);
  input_map_.resize(num_instances, kInvalidPortIdentifier);
  actuator_map_.resize(num_instances, default_entry);
  output_map_.resize(num_instances, kInvalidPortIdentifier);
  position_map_.resize(num_instances, default_entry);
  velocity_map_.resize(num_instances, default_entry);

  if (num_instances != 0) {
    // Figure out which actuators belong to which model instance, and
    // create the appropriate maps and input ports.  We demand that
    // the tree be constructed such that all actuator, positions, and
    // velocity indices are contiguous for each model instance.
    for (int actuator_index = 0; actuator_index < tree_->get_num_actuators();
         ++actuator_index) {
      const RigidBody<double>* body = tree_->actuators[actuator_index].body_;
      const int instance_id = body->get_model_instance_id();
      if (actuator_map_[instance_id].first == kInvalidPortIdentifier) {
        actuator_map_[instance_id] = std::pair<int, int>(actuator_index, 1);
      } else {
        std::pair<int, int> map_entry = actuator_map_[instance_id];
        DRAKE_DEMAND(actuator_index == map_entry.first + map_entry.second);
        ++map_entry.second;
        actuator_map_[instance_id] = map_entry;
      }
    }

    for (int i = 0; i < num_instances; ++i) {
      if (get_num_actuators(i) == 0) { continue; }
      input_map_[i] = this->DeclareInputPort(
          kVectorValued, actuator_map_[i].second).get_index();
    }

    // Now create the appropriate maps for the position and velocity
    // components.
    for (const auto& body : tree_->bodies) {
      if (!body->has_parent_body()) { continue; }
      const int instance_id = body->get_model_instance_id();
      const int position_start_index = body->get_position_start_index();
      const int num_positions = body->getJoint().get_num_positions();
      if (num_positions) {
        if (position_map_[instance_id].first == kInvalidPortIdentifier) {
          position_map_[instance_id] =
              std::pair<int, int>(position_start_index, num_positions);
        } else {
          std::pair<int, int> map_entry = position_map_[instance_id];
          DRAKE_DEMAND(position_start_index ==
                       map_entry.first + map_entry.second);
          map_entry.second += num_positions;
          position_map_[instance_id] = map_entry;
        }
      }

      const int velocity_start_index = body->get_velocity_start_index();
      const int num_velocities = body->getJoint().get_num_velocities();
      if (num_velocities) {
        if (velocity_map_[instance_id].first == kInvalidPortIdentifier) {
          velocity_map_[instance_id] =
              std::pair<int, int>(velocity_start_index, num_velocities);
        } else {
          std::pair<int, int> map_entry = velocity_map_[instance_id];
          DRAKE_DEMAND(velocity_start_index ==
                       map_entry.first + map_entry.second);
          map_entry.second += num_velocities;
          velocity_map_[instance_id] = map_entry;
        }
      }
    }

    for (int i = 0; i < num_instances; ++i) {
      if (get_num_states(i) == 0) { continue; }
      output_map_[i] = this->DeclareOutputPort(
          kVectorValued, get_num_states(i)).get_index();
    }
  }
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
int RigidBodyPlant<T>::get_num_positions(int model_instance_id) const {
  DRAKE_ASSERT(get_num_model_instances() > model_instance_id);
  return position_map_[model_instance_id].second;
}

template <typename T>
int RigidBodyPlant<T>::get_num_velocities() const {
  return tree_->get_num_velocities();
}

template <typename T>
int RigidBodyPlant<T>::get_num_velocities(int model_instance_id) const {
  DRAKE_ASSERT(get_num_model_instances() > model_instance_id);
  return velocity_map_[model_instance_id].second;
}

template <typename T>
int RigidBodyPlant<T>::get_num_states() const {
  return get_num_positions() + get_num_velocities();
}

template <typename T>
int RigidBodyPlant<T>::get_num_states(int model_instance_id) const {
  return get_num_positions(model_instance_id) +
      get_num_velocities(model_instance_id);
}

template <typename T>
int RigidBodyPlant<T>::get_num_actuators() const {
  return tree_->get_num_actuators();
}

template <typename T>
int RigidBodyPlant<T>::get_num_actuators(int model_instance_id) const {
  DRAKE_ASSERT(get_num_model_instances() > model_instance_id);
  return actuator_map_[model_instance_id].second;
}

template <typename T>
int RigidBodyPlant<T>::get_num_model_instances() const {
  return tree_->get_num_model_instances();
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
  set_state_vector(context->get_mutable_state(), x);
}

template <typename T>
void RigidBodyPlant<T>::set_state_vector(
    State<T>* state, const Eigen::Ref<const VectorX<T>> x) const {
  DRAKE_ASSERT(state != nullptr);
  DRAKE_ASSERT(x.size() == get_num_states());
  state->get_mutable_continuous_state()->SetFromVector(x);
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

  // Allocates an output for each of the per-instance output ports.
  for (int instance_id = 0;
       instance_id < get_num_model_instances(); ++instance_id) {
    if (output_map_[instance_id] == kInvalidPortIdentifier) { continue; }
    auto data = make_unique<BasicVector<T>>(get_num_states(instance_id));
    auto port = make_unique<OutputPort>(move(data));
    output->get_mutable_ports()->push_back(move(port));
  }

  return std::unique_ptr<SystemOutput<T>>(output.release());
}

template <typename T>
std::unique_ptr<ContinuousState<T>> RigidBodyPlant<T>::AllocateContinuousState()
    const {
  // The state is second-order.
  DRAKE_ASSERT(System<T>::get_input_port(0).size() == get_num_actuators());
  // TODO(amcastro-tri): add z state to track energy conservation.
  return std::make_unique<ContinuousState<T>>(
      std::make_unique<BasicVector<T>>(get_num_states()),
      get_num_positions() /* num_q */, get_num_velocities() /* num_v */,
      0 /* num_z */);
}

template <typename T>
void RigidBodyPlant<T>::DoCalcOutput(const Context<T>& context,
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
  const auto state_vector = output_vector->get_value();

  for (int instance_id = 0;
       instance_id < get_num_model_instances(); ++instance_id) {
    if (output_map_[instance_id] == kInvalidPortIdentifier) { continue; }
    BasicVector<T>* instance_output =
        output->GetMutableVectorData(output_map_[instance_id]);
    auto values = instance_output->get_mutable_value();
    const auto& instance_positions = position_map_[instance_id];
    values.head(instance_positions.second) =
        state_vector.segment(instance_positions.first,
                             instance_positions.second);

    const auto& instance_velocities = velocity_map_[instance_id];
    values.tail(instance_velocities.second) =
        state_vector.segment(instance_velocities.first + get_num_positions(),
                             instance_velocities.second);
  }

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

/*
 * TODO(hongkai.dai): This only works for templates on double, it does not
 * work for autodiff yet, I will add the code to compute the gradient of vdot
 * w.r.t. q and v. See issue
 * https://github.com/RobotLocomotion/drake/issues/4267.
 */
template <typename T>
void RigidBodyPlant<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  DRAKE_DEMAND(derivatives != nullptr);

  // The input vector of actuation values.
  VectorX<T> u;
  const BasicVector<T>* input = this->EvalVectorInput(context, 0);
  if (input != nullptr) {
    for (int instance_id = 0;
         instance_id < get_num_model_instances(); ++instance_id) {
      if (input_map_[instance_id] == kInvalidPortIdentifier) { continue; }
      DRAKE_ASSERT(
          this->EvalVectorInput(context, input_map_[instance_id]) == nullptr);
    }
    u = input->get_value();
  } else {
    u.resize(get_num_actuators());
    u.fill(0.);
    for (int instance_id = 0;
         instance_id < get_num_model_instances(); ++instance_id) {
      if (input_map_[instance_id] == kInvalidPortIdentifier) { continue; }
      const BasicVector<T>* instance_input =
          this->EvalVectorInput(context, input_map_[instance_id]);
      if (instance_input == nullptr) { continue; }

      const auto& instance_actuators = actuator_map_[instance_id];
      DRAKE_ASSERT(instance_actuators.first != kInvalidPortIdentifier);
      u.segment(instance_actuators.first, instance_actuators.second) =
          instance_input->get_value();
    }
  }

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
  drake::solvers::DecisionVariableVectorX vdot =
      prog.AddContinuousVariables(nv, "vdot");

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

  /*
   * TODO(hongkai.dai): This only works for templates on double, it does not
   * work for autodiff yet, I will add the code to compute the gradient of vdot
   * w.r.t. q and v. See issue
   * https://github.com/RobotLocomotion/drake/issues/4267.
   */
  // TODO(amcastro-tri): Remove .eval() below once RigidBodyTree is fully
  // templatized.
  const auto& vdot_value =
      drake::solvers::GetSolution(vdot);
  xdot << tree_->transformQDotMappingToVelocityMapping(
      kinsol, MatrixX<T>::Identity(nq, nq).eval()) * v, vdot_value;

  derivatives->SetFromVector(xdot);
}

template <typename T>
int RigidBodyPlant<T>::FindInstancePositionIndexFromWorldIndex(
    int model_instance_id, int world_position_index) {
  DRAKE_ASSERT(get_num_model_instances() > model_instance_id);
  const auto& instance_positions = position_map_[model_instance_id];
  if (world_position_index >=
      (instance_positions.first + instance_positions.second)) {
    throw std::runtime_error(
        "Unable to find position index in model instance.");
  }
  return world_position_index - instance_positions.first;
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

  // TODO(amcastro-tri): Remove .eval() below once RigidBodyTree is fully
  // templatized.
  generalized_velocity->SetFromVector(
      tree_->transformQDotMappingToVelocityMapping(
          kinsol, configuration_dot.transpose().eval()).transpose());
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

  // TODO(amcastro-tri): Remove .eval() below once RigidBodyTree is fully
  // templatized.
  configuration_dot->SetFromVector(
      tree_->transformVelocityMappingToQDotMapping(
          kinsol, v.transpose().eval()).transpose());
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
Matrix3<T> RigidBodyPlant<T>::ComputeBasisFromZ(const Vector3<T>& z_axis_W) {
  // Projects the z-axis into the first quadrant in order to identify the
  // *smallest* component of the normal.
  const Vector3<T> u(z_axis_W.cwiseAbs());
  int minAxis;
  u.minCoeff(&minAxis);
  // The world axis corresponding to the smallest component of the local z-axis
  // will be *most* perpendicular.
  Vector3<T> perpAxis;
  perpAxis << (minAxis == 0 ? 1 : 0), (minAxis == 1 ? 1 : 0),
      (minAxis == 2 ? 1 : 0);
  // Now define x- and y-axes.
  Vector3<T> x_axis_W = z_axis_W.cross(perpAxis).normalized();
  Vector3<T> y_axis_W = z_axis_W.cross(x_axis_W);
  // Transformation from world frame to local frame.
  Matrix3<T> R_WL;
  R_WL.col(0) = x_axis_W;
  R_WL.col(1) = y_axis_W;
  R_WL.col(2) = z_axis_W;
  return R_WL;
}

template <typename T>
VectorX<T> RigidBodyPlant<T>::ComputeContactForce(
    const KinematicsCache<T>& kinsol, ContactResults<T>* contacts) const {
  std::vector<DrakeCollision::PointPair> pairs;

  // TODO(amcastro-tri): get rid of this const_cast.
  // Unfortunately collisionDetect() modifies the collision model in the RBT
  // when updating the collision element poses.
  pairs = const_cast<RigidBodyTree<T>*>(tree_.get())
      ->ComputeMaximumDepthCollisionPoints(kinsol, true);

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

      // R_WC is a left-multiplied rotation matrix to transform a vector from
      // contact frame (C) to world (W), e.g., v_W = R_WC * v_C.
      Matrix3<T> R_WC = ComputeBasisFromZ(this_normal);
      auto J = R_WC.transpose() * (JA - JB);  // J = [ D1; D2; n ]

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
        // tau_c = (R_CW * JA)^T * fA + (R_CW * JB)^T * fB = J^T * fA.
        // With J computed as above: J = R_CW * (JA - JB).
        // Since right_hand_side has a negative sign when on the RHS of the
        // system of equations ([H,-J^T] * [vdot;f] + right_hand_side = 0),
        // this term needs to be subtracted.
        contact_force += J.transpose() * fA;
        if (contacts != nullptr) {
          Vector3<T> pt_a_world =
              kinsol.get_element(
                  pair.elementA->get_body()->
                      get_body_index()).transform_to_world * pair.ptA;
          Vector3<T> pt_b_world =
              kinsol.get_element(
                  pair.elementB->get_body()->
                      get_body_index()).transform_to_world * pair.ptB;
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
          Vector3<T> force = R_WC * fA;
          Vector3<T> normal = R_WC.template block<3, 1>(0, 2);

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
