#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"
#include "drake/multibody/rigid_body_plant/compliant_material.h"
#include "drake/solvers/mathematical_program.h"

using std::make_unique;
using std::move;
using std::runtime_error;
using std::string;
using std::unique_ptr;
using std::vector;

using drake::multibody::collision::ElementId;

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
    : tree_(move(tree)), timestep_(timestep), compliant_contact_model_(
    std::make_unique<CompliantContactModel<T>>()) {
  DRAKE_DEMAND(tree_ != nullptr);
  state_output_port_index_ =
      this->DeclareVectorOutputPort(BasicVector<T>(get_num_states()),
                                    &RigidBodyPlant::CopyStateToOutput)
          .get_index();
  ExportModelInstanceCentricPorts();
  // Declares an abstract valued output port for kinematics results.
  kinematics_output_port_index_ =
      this->DeclareAbstractOutputPort(
              KinematicsResults<T>(tree_.get()),
              &RigidBodyPlant::CalcKinematicsResultsOutput)
          .get_index();

  // Declares an abstract valued output port for contact information.
  contact_output_port_index_ = DeclareContactResultsOutputPort();

  // Schedule time stepping update.
  if (timestep > 0.0)
    this->DeclarePeriodicDiscreteUpdate(timestep);
}

template <class T>
OutputPortIndex RigidBodyPlant<T>::DeclareContactResultsOutputPort() {
  return this->DeclareAbstractOutputPort(
      ContactResults<T>(),
      &RigidBodyPlant::CalcContactResultsOutput).get_index();
}

template <class T>
Eigen::VectorBlock<const VectorX<T>> RigidBodyPlant<T>::GetStateVector(
    const Context<T>& context) const {
  if (is_state_discrete()) {
    return dynamic_cast<const BasicVector<T>&>(
        context.get_discrete_state_vector()).get_value();
  } else {
    return dynamic_cast<const BasicVector<T>&>(
        context.get_continuous_state_vector()).get_value();
  }
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
          this->DeclareVectorOutputPort(BasicVector<T>(get_num_states(i)),
          [this, i](const Context<T>& context, BasicVector<T>* output) {
            this->CalcInstanceOutput(i, context, output);
          }).get_index();
    }
  }
}

template <typename T>
RigidBodyPlant<T>::~RigidBodyPlant() {}

template <typename T>
void RigidBodyPlant<T>::set_contact_model_parameters(
    const CompliantContactModelParameters& parameters) {
  compliant_contact_model_->set_model_parameters(parameters);
}

template <typename T>
void RigidBodyPlant<T>::set_default_compliant_material(
    const CompliantMaterial& material) {
  compliant_contact_model_->set_default_material(material);
}

template <typename T>
optional<bool> RigidBodyPlant<T>::DoHasDirectFeedthrough(int, int) const {
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
  if (is_state_discrete()) {
    context->get_mutable_discrete_state(0).SetAtIndex(position_index,
                                                       position);
  } else {
    context->get_mutable_continuous_state()
        .get_mutable_generalized_position()
        .SetAtIndex(position_index, position);
  }
}

template <typename T>
void RigidBodyPlant<T>::set_velocity(Context<T>* context, int velocity_index,
                                     T velocity) const {
  DRAKE_ASSERT(context != nullptr);
  if (is_state_discrete()) {
    context->get_mutable_discrete_state(0).SetAtIndex(
        get_num_positions() + velocity_index, velocity);
  } else {
    context->get_mutable_continuous_state()
        .get_mutable_generalized_velocity()
        .SetAtIndex(velocity_index, velocity);
  }
}

template <typename T>
void RigidBodyPlant<T>::set_state_vector(
    Context<T>* context, const Eigen::Ref<const VectorX<T>> x) const {
  DRAKE_ASSERT(context != nullptr);
  set_state_vector(&context->get_mutable_state(), x);
}

template <typename T>
void RigidBodyPlant<T>::set_state_vector(
    State<T>* state, const Eigen::Ref<const VectorX<T>> x) const {
  DRAKE_ASSERT(state != nullptr);
  DRAKE_ASSERT(x.size() == get_num_states());
  if (is_state_discrete()) {
    auto& xd = state->get_mutable_discrete_state();
    xd.get_mutable_vector(0).SetFromVector(x);
  } else {
    state->get_mutable_continuous_state().SetFromVector(x);
  }
}

template <typename T>
const OutputPort<T>&
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
  if (is_state_discrete()) {
    // Return an empty continuous state if the plant state is discrete.
    return std::make_unique<ContinuousState<T>>();
  }

  // TODO(amcastro-tri): add z state to track energy conservation.
  return make_unique<ContinuousState<T>>(
      make_unique<BasicVector<T>>(get_num_states()),
      get_num_positions() /* num_q */, get_num_velocities() /* num_v */,
      0 /* num_z */);
}

template <typename T>
std::unique_ptr<DiscreteValues<T>> RigidBodyPlant<T>::AllocateDiscreteState()
    const {
  if (!is_state_discrete()) {
    // State of the plant is continuous- return an empty discrete state.
    return std::make_unique<DiscreteValues<T>>();
  }
  return make_unique<DiscreteValues<T>>(
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

// Updates the state output port.
template <typename T>
void RigidBodyPlant<T>::CopyStateToOutput(const Context<T>& context,
                       BasicVector<T>* state_output_vector) const {
  // TODO(amcastro-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  const VectorX<T> state_vector =
      (is_state_discrete()) ? context.get_discrete_state(0).CopyToVector()
                            : context.get_continuous_state().CopyToVector();

  state_output_vector->get_mutable_value() = state_vector;
}

// Updates one model-instance-centric state output port.
template <typename T>
void RigidBodyPlant<T>::CalcInstanceOutput(
    int instance_id, const Context<T>& context,
    BasicVector<T>* instance_output) const {
  // TODO(sherm1) Should reference state rather than copy it here.
  const VectorX<T> state_vector =
      (is_state_discrete()) ? context.get_discrete_state(0).CopyToVector()
                            : context.get_continuous_state().CopyToVector();

  auto values = instance_output->get_mutable_value();
  const auto& instance_positions = position_map_[instance_id];
  values.head(instance_positions.second) =
      state_vector.segment(instance_positions.first, instance_positions.second);

  const auto& instance_velocities = velocity_map_[instance_id];
  values.tail(instance_velocities.second) =
      state_vector.segment(instance_velocities.first + get_num_positions(),
                           instance_velocities.second);
}

// Updates the kinematics results output port.
template <typename T>
void RigidBodyPlant<T>::CalcKinematicsResultsOutput(
    const Context<T>& context, KinematicsResults<T>* kinematics_results) const {
  kinematics_results->UpdateFromContext(context);
}

// Computes the stiffness, damping, and friction coefficient for a contact (if
// it exists).
template <typename T>
void RigidBodyPlant<T>::CalcContactStiffnessDampingMuAndNumHalfConeEdges(
      const drake::multibody::collision::PointPair& contact,
      double* stiffness,
      double* damping,
      double* mu,
      int* num_half_cone_edges) const {
  DRAKE_DEMAND(stiffness);
  DRAKE_DEMAND(damping);
  DRAKE_DEMAND(mu);
  DRAKE_DEMAND(num_half_cone_edges);

  // Get the compliant material parameters.
  CompliantMaterial material;
  compliant_contact_model_->CalcContactParameters(
      *contact.elementA, *contact.elementB, &material);

  // Get the stiffness.
  *stiffness = material.youngs_modulus();

  // Get the dissipation value.
  *damping = material.dissipation();

  // Get the coefficient of friction.
  *mu = material.static_friction();

  // TODO(edrumwri): The number of half-cone edges should be able to be set on
  // a per-geometry pair basis. For now, just set the value to pyramidal
  // friction.
  *num_half_cone_edges = 2;

  // Verify the friction directions are set correctly.
  DRAKE_DEMAND(*num_half_cone_edges >= 2);
}

// Gets A's translational velocity relative to B's translational velocity at a
// point common to the two rigid bodies.
// @param p_W The point of contact (defined in the world frame).
// @returns the relative velocity at p_W expressed in the world frame.
template <class T>
Vector3<T> RigidBodyPlant<T>::CalcRelTranslationalVelocity(
    const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
    const Vector3<T>& p_W) const {
  const auto& tree = this->get_rigid_body_tree();

  // TODO(edrumwri): Convert this method to avoid Jacobian computation using
  // RigidBodyTree::CalcBodySpatialVelocityInWorldFrame().

  // The contact point in A's frame.
  const auto X_AW = kcache.get_element(body_a_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_A = X_AW * p_W;

  // The contact point in B's frame.
  const auto X_BW = kcache.get_element(body_b_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_B = X_BW * p_W;

  // Get the Jacobian matrices.
  const auto JA =
      tree.transformPointsJacobian(kcache, p_A, body_a_index, 0, false);
  const auto JB =
      tree.transformPointsJacobian(kcache, p_B, body_b_index, 0, false);

  // Compute the relative velocity in the world frame.
  return (JA - JB) * kcache.getV();
}

// Updates a generalized force from a force of f (expressed in the world frame)
// applied at point p (defined in the global frame).
template <class T>
void RigidBodyPlant<T>::UpdateGeneralizedForce(
    const KinematicsCache<T>& kcache, int body_a_index, int body_b_index,
    const Vector3<T>& p_W, const Vector3<T>& f, VectorX<T>* gf) const {
  const auto& tree = this->get_rigid_body_tree();

  // TODO(edrumwri): Convert this method to avoid Jacobian computation using
  // RigidBodyTree::dynamicsBiasTerm().

  // The contact point in A's frame.
  const auto X_AW = kcache.get_element(body_a_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_A = X_AW * p_W;

  // The contact point in B's frame.
  const auto X_BW = kcache.get_element(body_b_index)
      .transform_to_world.inverse(Eigen::Isometry);
  const Vector3<T> p_B = X_BW * p_W;

  // Get the Jacobian matrices.
  const auto JA =
      tree.transformPointsJacobian(kcache, p_A, body_a_index, 0, false);
  const auto JB =
      tree.transformPointsJacobian(kcache, p_B, body_b_index, 0, false);

  // Compute the Jacobian transpose times the force, and use it to update gf.
  (*gf) += (JA - JB).transpose() * f;
}

// Evaluates the relative velocities between two bodies projected along the
// contact normals.
template <class T>
VectorX<T> RigidBodyPlant<T>::N_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const VectorX<T>& q, const VectorX<T>& v) const {
  const auto& tree = this->get_rigid_body_tree();
  auto kcache = tree.doKinematics(q, v);

  // Create a result vector.
  VectorX<T> result(contacts.size());

  // Loop through all contacts.
  for (int i = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kcache.get_element(body_a_index).transform_to_world * contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kcache.get_element(body_b_index).transform_to_world * contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // The *relative* velocity of the contact point in A relative to that in
    // B.
    const auto v_W = CalcRelTranslationalVelocity(kcache, body_a_index,
                                                  body_b_index, p_W);

    // Get the projected normal velocity
    result[i] = v_W.dot(contacts[i].normal);
  }

  return result;
}

// Applies forces along the contact normals at the contact points and gets the
// effect out on the generalized forces.
template <class T>
VectorX<T> RigidBodyPlant<T>::N_transpose_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kcache,
    const VectorX<T>& f) const {
  // Create a result vector.
  VectorX<T> result = VectorX<T>::Zero(kcache.getV().size());

  // Loop through all contacts.
  for (int i = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kcache.get_element(body_a_index).transform_to_world * contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kcache.get_element(body_b_index).transform_to_world * contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // Get the contribution to the generalized force from a force of the
    // specified normal applied at this point.
    UpdateGeneralizedForce(kcache, body_a_index, body_b_index, p_W,
                           contacts[i].normal * f[i], &result);
  }

  return result;
}

// Evaluates the relative velocities between two bodies projected along the
// contact tangent directions.
template <class T>
VectorX<T> RigidBodyPlant<T>::F_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const VectorX<T>& q, const VectorX<T>& v,
    const std::vector<int>& half_num_cone_edges) const {
  using std::cos;
  using std::sin;
  std::vector<Vector3<T>> basis_vecs;
  const auto& tree = this->get_rigid_body_tree();
  auto kcache = tree.doKinematics(q, v);

  // Get the total number of edges.
  const int total_edges = std::accumulate(
      half_num_cone_edges.begin(), half_num_cone_edges.end(), 0);

  // Create a result vector.
  VectorX<T> result(total_edges);

  // Loop through all contacts.
  for (int i = 0, k = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3<T> p_WAs =
        kcache.get_element(body_a_index).transform_to_world * contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3<T> p_WBs =
        kcache.get_element(body_b_index).transform_to_world * contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3<T> p_W = (p_WAs + p_WBs) * 0.5;

    // The *relative* velocity of the contact point in A relative to that in
    // B.
    const auto v_W = CalcRelTranslationalVelocity(kcache, body_a_index,
                                                  body_b_index, p_W);

    // Compute an orthonormal basis.
    const int kXAxisIndex = 0, kYAxisIndex = 1, kZAxisIndex = 2;
    auto R_WC = math::ComputeBasisFromAxis(kXAxisIndex, contacts[i].normal);
    const Vector3<T> tan1_dir = R_WC.col(kYAxisIndex);
    const Vector3<T> tan2_dir = R_WC.col(kZAxisIndex);

    // Set spanning tangent directions.
    basis_vecs.resize(half_num_cone_edges[i]);
    if (half_num_cone_edges[i] == 2) {
      // Special case: pyramid friction.
      basis_vecs.front() = tan1_dir;
      basis_vecs.back() = tan2_dir;
    } else {
      for (int j = 0; j < half_num_cone_edges[i]; ++j) {
        double theta = M_PI * j / 
            (static_cast<double>(half_num_cone_edges[i]) - 1);
        basis_vecs[j] = tan1_dir * cos(theta) + tan2_dir * sin(theta);
      }
    }

    // Loop over the spanning tangent directions.
    for (int j = 0; j < static_cast<int>(basis_vecs.size()); ++j) {
      // Get the projected tangent velocity.
      result[k++] = v_W.dot(basis_vecs[j]);
    }
  }

  return result;
}

// Applies a force at the contact spanning directions at all contacts and gets
// the effect out on the generalized forces.
template <class T>
VectorX<T> RigidBodyPlant<T>::F_transpose_mult(
    const std::vector<drake::multibody::collision::PointPair>& contacts,
    const KinematicsCache<T>& kcache,
    const VectorX<T>& f,
    const std::vector<int>& half_num_cone_edges) const {
  std::vector<Vector3<T>> basis_vecs;

  // Create a result vector.
  VectorX<T> result = VectorX<T>::Zero(kcache.getV().size());

  // Loop through all contacts.
  for (int i = 0, k = 0; static_cast<size_t>(i) < contacts.size(); ++i) {
    // Get the two body indices.
    const int body_a_index = contacts[i].elementA->get_body()->get_body_index();
    const int body_b_index = contacts[i].elementB->get_body()->get_body_index();

    // The reported point on A's surface (As) in the world frame (W).
    const Vector3 <T> p_WAs =
        kcache.get_element(body_a_index).transform_to_world * contacts[i].ptA;

    // The reported point on B's surface (Bs) in the world frame (W).
    const Vector3 <T> p_WBs =
        kcache.get_element(body_b_index).transform_to_world * contacts[i].ptB;

    // Get the point of contact in the world frame.
    const Vector3 <T> p_W = (p_WAs + p_WBs) * 0.5;

    // Compute an orthonormal basis.
    const int kXAxisIndex = 0, kYAxisIndex = 1, kZAxisIndex = 2;
    auto R_WC = math::ComputeBasisFromAxis(kXAxisIndex, contacts[i].normal);
    const Vector3<T> tan1_dir = R_WC.col(kYAxisIndex);
    const Vector3<T> tan2_dir = R_WC.col(kZAxisIndex);

    // Set spanning tangent directions.
    basis_vecs.resize(half_num_cone_edges[i]);
    if (half_num_cone_edges[i] == 2) {
      // Special case: pyramid friction.
      basis_vecs.front() = tan1_dir;
      basis_vecs.back() = tan2_dir;
    } else {
      for (int j = 0; j < half_num_cone_edges[i]; ++j) {
        double theta = M_PI * j /
            (static_cast<double>(half_num_cone_edges[i]) - 1);
        basis_vecs[j] = tan1_dir * cos(theta) + tan2_dir * sin(theta);
      }
    }

    // Get the contribution to the generalized force from a force of the
    // specified normal applied at this point.
    for (int j = 0; j < static_cast<int>(basis_vecs.size()); ++j) {
      UpdateGeneralizedForce(kcache, body_a_index, body_b_index, p_W,
                             basis_vecs[j] * f[k++], &result);
    }
  }

  return result;
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

  // No derivatives to compute if state is discrete.
  if (is_state_discrete()) return;

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

  right_hand_side += compliant_contact_model_->ComputeContactForce(
      *tree_.get(), kinsol);

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
    const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  using std::abs;

  static_assert(std::is_same<double, T>::value,
                "Only support templating on double for now");

  // If plant state is continuous, no discrete state to update.
  if (!is_state_discrete()) return;

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

  // Set up the N multiplication operator (projected velocity along the contact
  // normals).
  data.N_mult = [this, &contacts, &q](const VectorX<T>& w) -> VectorX<T> {
    return N_mult(contacts, q, w);
  };

  // Set up the N' multiplication operator (effect of contact normal forces
  // on generalized forces).
  data.N_transpose_mult = [this, &contacts, &kcache](const VectorX<T>& f) ->
      VectorX<T> {
    return N_transpose_mult(contacts, kcache, f);
  };

  // Set up the F multiplication operator (projected velocity along the contact
  // tangent directions).
  data.F_mult = [this, &contacts, &q, &data](const VectorX<T>& w) ->
      VectorX<T> {
    return F_mult(contacts, q, w, data.r);
  };

  // Set up the F' multiplication operator (effect of contact frictional forces
  // on generalized forces).
  data.F_transpose_mult = [this, &contacts, &kcache, &data](const VectorX<T>& f)
      -> VectorX<T> {
    return F_transpose_mult(contacts, kcache, f, data.r);
  };

  // 1. Set the stabilization term for contact normal direction (kN). Also,
  // determine the friction coefficients and (half) the number of friction cone
  // edges.
  data.gammaN.resize(contacts.size());
  data.kN.resize(contacts.size());
  data.mu.resize(contacts.size());
  data.r.resize(contacts.size());
  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    double stiffness, damping, mu;
    int half_friction_cone_edges;
    CalcContactStiffnessDampingMuAndNumHalfConeEdges(
        contacts[i], &stiffness, &damping, &mu, &half_friction_cone_edges);
    data.mu[i] = mu;
    data.r[i] = half_friction_cone_edges;

    // Set cfm and erp parameters for contacts.
    const double denom = dt * stiffness + damping;
    const double cfm = 1.0 / denom;
    const double erp = (dt * stiffness) / denom;
    data.gammaN[i] = cfm;
    data.kN[i] = erp * contacts[i].distance / dt;
  }

  // 2. Set the stabilization term for contact tangent directions (kF).
  const int total_friction_cone_edges = std::accumulate(
      data.r.begin(), data.r.end(), 0);
  data.kF.setZero(total_friction_cone_edges);

  // 3. Set the stabilization term for joint limit constraints (kL).
  data.kL.resize(0);

  // Integrate the forces into the velocity.
  data.Mv = H * v + right_hand_side * dt;

  // Solve the rigid impact problem.
  VectorX<T> vnew, cf;
  constraint_solver_.SolveImpactProblem(data, &cf);
  constraint_solver_.ComputeGeneralizedVelocityChange(data, cf, &vnew);
  vnew += data.solve_inertia(data.Mv);

  // qn = q + dt*qdot.
  VectorX<T> xn(this->get_num_states());
  xn << q + dt * tree.transformVelocityToQDot(kcache, vnew), vnew;
  updates->get_mutable_vector(0).SetFromVector(xn);
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
  // Discrete state does not use this method, since there are no continuous
  // qdot variables. Verify that, then return silently in this case.
  if (is_state_discrete()) {
    DRAKE_DEMAND(qdot.size() == 0);
    return;
  }

  // TODO(amcastro-tri): provide nicer accessor to an Eigen representation for
  // LeafSystems.
  auto x = GetStateVector(context);
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
  // Discrete state does not use this method, since there are no continuous
  // generalized velocity variables. Verify that, then return silently in this
  // case.
  if (is_state_discrete()) {
    DRAKE_DEMAND(generalized_velocity.size() == 0);
    return;
  }

  auto x = GetStateVector(context);
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

// Calculates the value of the contact results output port.
template <typename T>
void RigidBodyPlant<T>::CalcContactResultsOutput(
    const Context<T>& context, ContactResults<T>* contacts) const {
  DRAKE_ASSERT(contacts != nullptr);
  contacts->Clear();

  // This code should do nothing if the state is discrete because the compliant
  // contact model will not be used to compute contact forces.
  if (is_state_discrete())
    return;

  // TODO(SeanCurtis-TRI): This is horribly redundant code that only exists
  // because the data is not properly accessible in the cache.  This is
  // boilerplate drawn from EvalDerivatives.  See that code for further
  // comments
  auto x = GetStateVector(context);
  const int nq = get_num_positions();
  const int nv = get_num_velocities();
  VectorX<T> q = x.topRows(nq);
  VectorX<T> v = x.bottomRows(nv);
  auto kinsol = tree_->doKinematics(q, v);

  compliant_contact_model_->ComputeContactForce(*tree_.get(), kinsol, contacts);
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

// Explicitly instantiates on the most common scalar types.
template class RigidBodyPlant<double>;

}  // namespace systems
}  // namespace drake
