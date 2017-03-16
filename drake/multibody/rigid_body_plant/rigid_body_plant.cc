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
using std::runtime_error;
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
RigidBodyPlant<T>::RigidBodyPlant(std::unique_ptr<const RigidBodyTree<T>> tree,
                                  double timestep)
    : tree_(move(tree)), timestep_(timestep) {
  DRAKE_DEMAND(tree_ != nullptr);
  state_output_port_index_ =
      this->DeclareOutputPort(kVectorValued, get_num_states()).get_index();
  ExportModelInstanceCentricPorts();
  // Declares an abstract valued output port for kinematics results.
  kinematics_output_port_index_ = this->DeclareAbstractOutputPort(
      Value<KinematicsResults<T>>(tree_.get())).get_index();
  // Declares an abstract valued output port for contact information.
  contact_output_port_index_ = this->DeclareAbstractOutputPort(
      Value<ContactResults<T>>()).get_index();
}

template <typename T>
void RigidBodyPlant<T>::ExportModelInstanceCentricPorts() {
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
      if (get_num_actuators(i) == 0) {
        continue;
      }
      input_map_[i] =
          this->DeclareInputPort(kVectorValued, actuator_map_[i].second)
              .get_index();
    }

    // Now create the appropriate maps for the position and velocity
    // components.
    for (const auto& body : tree_->bodies) {
      if (!body->has_parent_body()) {
        continue;
      }
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
      if (get_num_states(i) == 0) {
        continue;
      }
      output_map_[i] =
          this->DeclareOutputPort(kVectorValued, get_num_states(i)).get_index();
    }
  }
}

template <typename T>
RigidBodyPlant<T>::~RigidBodyPlant() {}

// TODO(liang.fok) Remove these methods once a more advanced contact modeling
// framework is available.
template <typename T>
void RigidBodyPlant<T>::set_normal_contact_parameters(
    double penetration_stiffness, double dissipation) {
  DRAKE_DEMAND(penetration_stiffness >= 0);
  DRAKE_DEMAND(dissipation >= 0);
  penetration_stiffness_ = penetration_stiffness;
  dissipation_ = dissipation;
}

template <typename T>
void RigidBodyPlant<T>::set_friction_contact_parameters(
    double static_friction_coef, double dynamic_friction_coef,
    double v_stiction_tolerance) {
  DRAKE_DEMAND(dynamic_friction_coef >= 0);
  DRAKE_DEMAND(static_friction_coef >= dynamic_friction_coef);
  DRAKE_DEMAND(v_stiction_tolerance > 0);
  dynamic_friction_ceof_ = dynamic_friction_coef;
  static_friction_coef_ = static_friction_coef;
  inv_v_stiction_tolerance_ = 1.0 / v_stiction_tolerance;
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
  if (timestep_ > 0.0) {
    context->get_mutable_discrete_state(0)->SetAtIndex(position_index,
                                                       position);
  } else {
    context->get_mutable_continuous_state()
        ->get_mutable_generalized_position()
        ->SetAtIndex(position_index, position);
  }
}

template <typename T>
void RigidBodyPlant<T>::set_velocity(Context<T>* context, int velocity_index,
                                     T velocity) const {
  DRAKE_ASSERT(context != nullptr);
  if (timestep_ > 0.0) {
    context->get_mutable_discrete_state(0)->SetAtIndex(
        get_num_positions() + velocity_index, velocity);
  } else {
    context->get_mutable_continuous_state()
        ->get_mutable_generalized_velocity()
        ->SetAtIndex(velocity_index, velocity);
  }
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
  if (timestep_ > 0.0) {
    state->get_mutable_discrete_state()
        ->get_mutable_discrete_state(0)
        ->SetFromVector(x);
  } else {
    state->get_mutable_continuous_state()->SetFromVector(x);
  }
}

template <typename T>
const OutputPortDescriptor<T>&
RigidBodyPlant<T>::model_instance_state_output_port(
    int model_instance_id) const {
  if (model_instance_id >= static_cast<int>(output_map_.size())) {
    throw std::runtime_error(
        "RigidBodyPlant::model_state_output_port(): "
        "ERROR: Model instance with ID " +
        std::to_string(model_instance_id) + " does not exist! Maximum ID is " +
        std::to_string(output_map_.size() - 1) + ".");
  }
  if (output_map_.at(model_instance_id) == kInvalidPortIdentifier) {
    throw std::runtime_error(
        "RigidBodyPlant::model_state_output_port(): "
        "ERROR: Model instance with ID " +
        std::to_string(model_instance_id) +
        " does not have any state output port!");
  }
  return System<T>::get_output_port(output_map_.at(model_instance_id));
}

template <typename T>
std::unique_ptr<ContinuousState<T>> RigidBodyPlant<T>::AllocateContinuousState()
    const {
  // TODO(amcastro-tri): add z state to track energy conservation.
  if (timestep_ > 0.0) {
    return std::make_unique<ContinuousState<T>>();
  }

  return make_unique<ContinuousState<T>>(
      make_unique<BasicVector<T>>(get_num_states()),
      get_num_positions() /* num_q */, get_num_velocities() /* num_v */,
      0 /* num_z */);
}

template <typename T>
std::unique_ptr<DiscreteState<T>> RigidBodyPlant<T>::AllocateDiscreteState()
    const {
  if (timestep_ == 0.0) {
    return std::make_unique<DiscreteState<T>>();
  }
  return make_unique<DiscreteState<T>>(
      make_unique<BasicVector<T>>(get_num_states()));
}

template <typename T>
bool RigidBodyPlant<T>::model_instance_has_actuators(
    int model_instance_id) const {
  DRAKE_ASSERT(static_cast<int>(input_map_.size()) ==
               get_num_model_instances());
  if (model_instance_id >= get_num_model_instances()) {
    throw std::runtime_error(
        "RigidBodyPlant::model_instance_has_actuators(): ERROR: provided "
        "model_instance_id of " +
        std::to_string(model_instance_id) +
        " does not exist. Maximum model_instance_id is " +
        std::to_string(get_num_model_instances()));
  }
  return input_map_.at(model_instance_id) != kInvalidPortIdentifier;
}

template <typename T>
const InputPortDescriptor<T>&
RigidBodyPlant<T>::model_instance_actuator_command_input_port(
    int model_instance_id) const {
  if (input_map_.at(model_instance_id) == kInvalidPortIdentifier) {
    throw std::runtime_error(
        "RigidBodyPlant::"
        "model_instance_actuator_command_input_port(): ERROR model instance "
        "with ID " +
        std::to_string(model_instance_id) +
        " does not have "
        "an actuator command input ports because it does not have any "
        "actuators.");
  }
  return System<T>::get_input_port(input_map_.at(model_instance_id));
}

template <typename T>
void RigidBodyPlant<T>::DoCalcOutput(const Context<T>& context,
                                     SystemOutput<T>* output) const {
  // TODO(amcastro-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  const VectorX<T> state_vector =
      (timestep_ > 0.0) ? context.get_discrete_state(0)->CopyToVector()
                        : context.get_continuous_state()->CopyToVector();

  // Evaluates the state output port.
  BasicVector<T>* state_output_vector =
      output->GetMutableVectorData(state_output_port_index_);
  state_output_vector->get_mutable_value() = state_vector;

  // Updates the model-instance-centric state output ports.
  for (int instance_id = 0; instance_id < get_num_model_instances();
       ++instance_id) {
    if (output_map_[instance_id] == kInvalidPortIdentifier) {
      continue;
    }
    BasicVector<T>* instance_output =
        output->GetMutableVectorData(output_map_[instance_id]);
    auto values = instance_output->get_mutable_value();
    const auto& instance_positions = position_map_[instance_id];
    values.head(instance_positions.second) = state_vector.segment(
        instance_positions.first, instance_positions.second);

    const auto& instance_velocities = velocity_map_[instance_id];
    values.tail(instance_velocities.second) =
        state_vector.segment(instance_velocities.first + get_num_positions(),
                             instance_velocities.second);
  }

  // Updates the kinematics results output port.
  auto& kinematics_results =
      output->GetMutableData(kinematics_output_port_index_)
          ->template GetMutableValue<KinematicsResults<T>>();
  kinematics_results.UpdateFromContext(context);

  // Updates the contact results output port.
  auto& contact_results = output->GetMutableData(contact_output_port_index_)
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
  if (timestep_ > 0.0) return;

  VectorX<T> u = EvaluateActuatorInputs(context);

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int num_actuators = get_num_actuators();
  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block
  // which
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
  drake::solvers::VectorXDecisionVariable vdot =
      prog.NewContinuousVariables(nv, "vdot");

  auto H = tree_->massMatrix(kinsol);
  Eigen::MatrixXd H_and_neg_JT = H;

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm.
  // TODO(amcastro-tri): external_wrenches should be made an optional
  // parameter
  // of dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;
  // right_hand_side is the right hand side of the system's equations:
  // H*vdot -J^T*f = right_hand_side.
  VectorX<T> right_hand_side =
      -tree_->dynamicsBiasTerm(kinsol, no_external_wrenches);
  if (num_actuators > 0) right_hand_side += tree_->B * u;

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
        right_hand_side(b->get_velocity_start_index()) += limit_force;
      }
    }
  }

  right_hand_side += ComputeContactForce(kinsol);

  solvers::VectorXDecisionVariable position_force{};

  if (tree_->getNumPositionConstraints()) {
    size_t nc = tree_->getNumPositionConstraints();
    // 1/time constant of position constraint satisfaction.
    const T alpha = 5.0;

    position_force =
        prog.NewContinuousVariables(nc, "position constraint force");

    auto phi = tree_->positionConstraints(kinsol);
    auto J = tree_->positionConstraintsJacobian(kinsol, false);
    auto Jdotv = tree_->positionConstraintsJacDotTimesV(kinsol);

    // Critically damped stabilization term.
    // phiddot = -2 * alpha * phidot - alpha^2 * phi.
    prog.AddLinearEqualityConstraint(
        J, -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi), vdot);
    H_and_neg_JT.conservativeResize(Eigen::NoChange,
                                    H_and_neg_JT.cols() + J.rows());
    H_and_neg_JT.rightCols(J.rows()) = -J.transpose();
  }

  // Adds [H,-J^T] * [vdot;f] = -C.
  prog.AddLinearEqualityConstraint(H_and_neg_JT, right_hand_side,
                                   {vdot, position_force});

  prog.Solve();

  VectorX<T> xdot(get_num_states());
  const auto& vdot_value = prog.GetSolution(vdot);
  xdot << tree_->transformVelocityToQDot(kinsol, v), vdot_value;
  derivatives->SetFromVector(xdot);
}

template <typename T>
void RigidBodyPlant<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    drake::systems::DiscreteState<T>* updates) const {
  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");
  if (timestep_ == 0.0) return;

  VectorX<T> u = EvaluateActuatorInputs(context);

  auto x = context.get_discrete_state(0)->get_value();

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int num_actuators = get_num_actuators();

  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinsol = tree_->doKinematics(q, v);

  drake::solvers::MathematicalProgram prog;
  drake::solvers::VectorXDecisionVariable vn =
      prog.NewContinuousVariables(nv, "vn");

  auto H = tree_->massMatrix(kinsol);

  // There are no external wrenches, but it is a required argument in
  // dynamicsBiasTerm().
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  // right_hand_side is the right hand side of the system's equations:
  //   right_hand_side = B*u - C(q,v)
  VectorX<T> right_hand_side =
      -tree_->dynamicsBiasTerm(kinsol, no_external_wrenches);
  if (num_actuators > 0) right_hand_side += tree_->B * u;

  // TODO(russt): Handle joint limits.
  // TODO(russt): Handle contact constraints.

  // Add H*(vn - v)/h = right_hand_side
  prog.AddLinearEqualityConstraint(H / timestep_,
                                   H * v / timestep_ + right_hand_side, vn);

  prog.Solve();

  VectorX<T> xn(get_num_states());
  const auto& vn_sol = prog.GetSolution(vn);

  // qn = q + h*qdn.
  xn << q + timestep_ * tree_->transformVelocityToQDot(kinsol, vn_sol), vn_sol;
  updates->get_mutable_discrete_state(0)->SetFromVector(xn);
}

template <typename T>
int RigidBodyPlant<T>::FindInstancePositionIndexFromWorldIndex(
    int model_instance_id, int world_position_index) {
  DRAKE_ASSERT(get_num_model_instances() > model_instance_id);
  const auto& instance_positions = position_map_[model_instance_id];
  if (world_position_index >=
      (instance_positions.first + instance_positions.second)) {
    throw runtime_error("Unable to find position index in model instance.");
  }
  return world_position_index - instance_positions.first;
}

template <typename T>
void RigidBodyPlant<T>::DoMapQDotToVelocity(
    const Context<T>& context, const Eigen::Ref<const VectorX<T>>& qdot,
    VectorBase<T>* generalized_velocity) const {
  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  const int nstates = get_num_states();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_ASSERT(generalized_velocity->size() == nv);
  DRAKE_ASSERT(x.size() == nstates);

  // TODO(amcastro-tri): we would like to compile here with `auto` instead of
  // `VectorX<T>`. However it seems we get some sort of block from a block
  // that is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);

  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_->doKinematics(q);

  // TODO(amcastro-tri): Remove .eval() below once RigidBodyTree is fully
  // templatized.
  generalized_velocity->SetFromVector(
      tree_->transformQDotToVelocity(kinsol, qdot));
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
  // `VectorX<T>`. However it seems we get some sort of block from a block
  // that is not instantiated in drakeRBM.
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = generalized_velocity;

  // TODO(amcastro-tri): place kinematics cache in the context so it can be
  // reused.
  auto kinsol = tree_->doKinematics(q, v);

  configuration_dot->SetFromVector(tree_->transformVelocityToQDot(kinsol, v));
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
    using std::min;  // Needed for ADL.
    return min(limit_force, 0.);
  } else if (position < qmin) {
    const T violation = position - qmin;
    const T limit_force =
        (-joint_stiffness * violation * (1 - joint_dissipation * velocity));
    using std::max;  // Needed for ADL.
    return max(limit_force, 0.);
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
  // The world axis corresponding to the smallest component of the local
  // z-axis will be *most* perpendicular.
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
  using std::sqrt;

  // TODO(amcastro-tri): get rid of this const_cast.
  // Unfortunately collisionDetect() modifies the collision model in the RBT
  // when updating the collision element poses.
  std::vector<DrakeCollision::PointPair> pairs =
      const_cast<RigidBodyTree<T>*>(tree_.get())
          ->ComputeMaximumDepthCollisionPoints(kinsol, true);

  VectorX<T> contact_force(kinsol.getV().rows(), 1);
  contact_force.setZero();
  // TODO(SeanCurtis-TRI): Determine if a distance of zero should be reported
  //  as a zero-force contact.
  for (const auto& pair : pairs) {
    if (pair.distance < 0.0) {  // There is contact.
      // Define the contact point: the pair contains points on the *surfaces* of
      // bodies A and B, given as location vectors measured and expressed in the
      // respective body frames.  For penetration, these points will *not* be
      // coincident. We must define a common contact point at which relative
      // velocity is defined and the force is applied.

      const int body_a_index = pair.elementA->get_body()->get_body_index();
      const int body_b_index = pair.elementB->get_body()->get_body_index();
      // The reported point on A's surface (As) in the world frame (W).
      const Vector3<T> p_WAs =
          kinsol.get_element(body_a_index).transform_to_world * pair.ptA;
      // The reported point on B's surface (Bs) in the world frame (W).
      const Vector3<T> p_WBs =
          kinsol.get_element(body_b_index).transform_to_world * pair.ptB;
      // The point of contact in the world frame.  Without better information,
      // the point is arbitrarily selected to be halfway between the two
      // surface points.
      const Vector3<T> p_WC = (p_WAs + p_WBs) / 2;

      // The contact point in A's frame.
      const auto X_AW = kinsol.get_element(body_a_index)
          .transform_to_world.inverse(Eigen::Isometry);
      const Vector3<T> p_AAc = X_AW * p_WC;
      // The contact point in B's frame.
      const auto X_BW = kinsol.get_element(body_b_index)
          .transform_to_world.inverse(Eigen::Isometry);
      const Vector3<T> p_BBc = X_BW * p_WC;

      const auto JA = tree_->transformPointsJacobian(kinsol, p_AAc,
                                                     body_a_index, 0, false);
      const auto JB = tree_->transformPointsJacobian(kinsol, p_BBc,
                                                     body_b_index, 0, false);
      // This normal points *from* element B *to* element A.
      const Vector3<T> this_normal = pair.normal;

      // R_WC is a left-multiplied rotation matrix to transform a vector from
      // contact frame (C) to world (W), e.g., v_W = R_WC * v_C.
      const Matrix3<T> R_WC = ComputeBasisFromZ(this_normal);
      const auto J = R_WC.transpose() * (JA - JB);  // J = [ D1; D2; n ]

      // TODO(SeanCurtis-TRI): Coordinate with Paul Mitiguy to standardize this
      // notation.
      // The *relative* velocity of the contact point in A relative to that in
      // B, expressed in the contact frame, C.
      const auto v_CBcAc_C = J * kinsol.getV();

      // TODO(SeanCurtis-TRI): Move this documentation to the larger doxygen
      // discussion and simply reference it here.

      // See contact_model_doxygen.h for the details of this contact model.
      // Normal force fN = kx(1 + dẋ).  We map the equation to the
      // local variables as follows:
      //  x = -pair.distance -- penetration depth.
      //  ̇ẋ = -v_CBcAc_C(2)  -- change of penetration (in normal direction).
      //  fK = kx -- force due to stiffness.
      //  fD = fk dẋ -- force due to dissipation.
      //  fN = max(0, fK + fD ) -- total normal force; (skipped if fN < 0).
      //  fF = mu(v) * fN  - friction force magnitude.

      const T x = T(-pair.distance);
      const T x_dot = -v_CBcAc_C(2);

      const T fK = penetration_stiffness_ * x;
      const T fD = fK * dissipation_ * x_dot;
      const T fN = fK + fD;
      if (fN <= 0) continue;

      Vector3<T> fA;
      fA(2) = fN;
      // Friction force
      const auto slip_vector = v_CBcAc_C.template head<2>();
      T slip_speed_squared = slip_vector.squaredNorm();
      // Consider a value indistinguishable from zero if it is smaller
      // then 1e-14 and test against that value squared.
      const T kNonZeroSqd = T(1e-14 * 1e-14);
      if (slip_speed_squared > kNonZeroSqd) {
        const T slip_speed = sqrt(slip_speed_squared);
        const T friction_coefficient = ComputeFrictionCoefficient(slip_speed);
        const T fF = friction_coefficient * fN;
        fA.template head<2>() = -(fF / slip_speed) *
            slip_vector;
      } else {
        fA.template head<2>() << 0, 0;
      }

      // fB is equal and opposite to fA: fB = -fA.
      // Therefore the generalized forces tau_c due to contact are:
      // tau_c = (R_CW * JA)^T * fA + (R_CW * JB)^T * fB = J^T * fA.
      // With J computed as above: J = R_CW * (JA - JB).
      // Since right_hand_side has a negative sign when on the RHS of the
      // system of equations ([H,-J^T] * [vdot;f] + right_hand_side = 0),
      // this term needs to be subtracted.
      contact_force += J.transpose() * fA;
      if (contacts != nullptr) {
        ContactInfo<T>& contact_info = contacts->AddContact(
            pair.elementA->getId(), pair.elementB->getId());

        // TODO(SeanCurtis-TRI): Future feature: test against user-set flag
        // for whether the details should generally be captured or not and
        // make this function dependent.
        vector<unique_ptr<ContactDetail<T>>> details;
        ContactResultantForceCalculator<T> calculator(&details);

        // This contact model produces responses that only have a force
        // component (i.e., the torque portion of the wrench is zero.)
        // In contrast, other models (e.g., torsional friction model) can
        // also introduce a "pure torque" component to the wrench.
        const Vector3<T> force = R_WC * fA;
        const Vector3<T> normal = R_WC.template block<3, 1>(0, 2);

        calculator.AddForce(p_WC, normal, force);

        contact_info.set_resultant_force(calculator.ComputeResultant());
        // TODO(SeanCurtis-TRI): As with previous note, this line depends
        // on the eventual instantiation of the user-set flag for accumulating
        // contact details.
        contact_info.set_contact_details(move(details));
      }
    }
  }
  return contact_force;
}

template <typename T>
VectorX<T> RigidBodyPlant<T>::EvaluateActuatorInputs(
    const Context<T>& context) const {
  VectorX<T> u;  // The plant-centric input vector of actuation values.
  u.resize(get_num_actuators());
  u.fill(0.);

  if (get_num_actuators() > 0) {
    for (int instance_id = 0; instance_id < get_num_model_instances();
         ++instance_id) {
      if (input_map_[instance_id] == kInvalidPortIdentifier) {
        continue;
      }
      if (this->EvalVectorInput(context, input_map_[instance_id]) == nullptr) {
        throw runtime_error(
            "RigidBodyPlant::EvaluateActuatorInputs(): ERROR: "
                "Actuator command input port for model instance " +
                std::to_string(instance_id) + " is not connected. All " +
                std::to_string(get_num_model_instances()) +
                " actuator command input ports must be connected.");
      }
    }

    for (int instance_id = 0; instance_id < get_num_model_instances();
         ++instance_id) {
      if (input_map_[instance_id] == kInvalidPortIdentifier) {
        continue;
      }
      const BasicVector<T> *instance_input =
          this->EvalVectorInput(context, input_map_[instance_id]);
      if (instance_input == nullptr) {
        continue;
      }

      const auto &instance_actuators = actuator_map_[instance_id];
      DRAKE_ASSERT(instance_actuators.first != kInvalidPortIdentifier);
      u.segment(instance_actuators.first, instance_actuators.second) =
          instance_input->get_value();
    }
  }
  return u;
}

template <typename T>
T RigidBodyPlant<T>::ComputeFrictionCoefficient(T v_tangent_BAc) const {
  DRAKE_ASSERT(v_tangent_BAc >= 0);
  const T v = v_tangent_BAc * inv_v_stiction_tolerance_;
  if (v >= 3) {
    return dynamic_friction_ceof_;
  } else if (v >= 1) {
    return static_friction_coef_ -
           (static_friction_coef_ - dynamic_friction_ceof_) *
               step5((v - 1) / 2);
  } else {
    return static_friction_coef_ * step5(v);
  }
}

template <typename T>
T RigidBodyPlant<T>::step5(T x) {
  DRAKE_ASSERT(0 <= x && x <= 1);
  const T x3 = x * x * x;
  return x3 * (10 + x * (6 * x - 15));  // 10x³ - 15x⁴ + 6x⁵
}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
