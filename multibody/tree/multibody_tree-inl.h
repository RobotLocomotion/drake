#pragma once

#include <algorithm>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/pointer_cast.h"
#include "drake/common/random.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/body_node.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/model_instance.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/multibody/tree/position_kinematics_cache.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/velocity_kinematics_cache.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
template <template <typename Scalar> class BodyType>
const BodyType<T>& MultibodyTree<T>::AddBody(
    std::unique_ptr<BodyType<T>> body) {
  static_assert(std::is_convertible_v<BodyType<T>*, Body<T>*>,
                "BodyType must be a sub-class of Body<T>.");
  if (topology_is_valid()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. "
        "Therefore adding more bodies is not allowed. "
        "See documentation for Finalize() for details.");
  }
  if (body == nullptr) {
    throw std::logic_error("Input body is a nullptr.");
  }
  BodyIndex body_index(0);
  FrameIndex body_frame_index(0);
  std::tie(body_index, body_frame_index) = topology_.add_body();
  // These tests MUST be performed BEFORE frames_.push_back() and
  // owned_bodies_.push_back() below. Do not move them around!
  DRAKE_DEMAND(body_index == num_bodies());
  DRAKE_DEMAND(body_frame_index == num_frames());
  DRAKE_DEMAND(body->model_instance().is_valid());

  // TODO(amcastro-tri): consider not depending on setting this pointer at
  // all. Consider also removing MultibodyElement altogether.
  body->set_parent_tree(this, body_index);
  // MultibodyTree can access selected private methods in Body through its
  // BodyAttorney.
  // - Register body frame.
  Frame<T>* body_frame =
      &internal::BodyAttorney<T>::get_mutable_body_frame(body.get());
  body_frame->set_parent_tree(this, body_frame_index);
  DRAKE_ASSERT(body_frame->name() == body->name());
  this->SetElementIndex(body_frame->name(), body_frame_index,
                        &frame_name_to_index_);
  frames_.push_back(body_frame);
  // - Register body.
  BodyType<T>* raw_body_ptr = body.get();
  this->SetElementIndex(body->name(), body->index(), &body_name_to_index_);
  owned_bodies_.push_back(std::move(body));
  return *raw_body_ptr;
}

template <typename T>
template<template<typename Scalar> class BodyType, typename... Args>
const BodyType<T>& MultibodyTree<T>::AddBody(Args&&... args) {
  static_assert(std::is_convertible_v<BodyType<T>*, Body<T>*>,
                "BodyType must be a sub-class of Body<T>.");
  return AddBody(std::make_unique<BodyType<T>>(std::forward<Args>(args)...));
}

template <typename T>
const RigidBody<T>& MultibodyTree<T>::AddRigidBody(
    const std::string& name, ModelInstanceIndex model_instance,
    const SpatialInertia<double>& M_BBo_B) {
  if (model_instance >= num_model_instances()) {
    throw std::logic_error("Invalid model instance specified.");
  }

  if (HasBodyNamed(name, model_instance)) {
    throw std::logic_error(
        "Model instance '" + instance_index_to_name_.at(model_instance) +
            "' already contains a body named '" + name + "'. " +
            "Body names must be unique within a given model.");
  }

  const RigidBody<T>& body =
      this->template AddBody<RigidBody>(name, model_instance, M_BBo_B);
  return body;
}

template <typename T>
const RigidBody<T>& MultibodyTree<T>::AddRigidBody(
    const std::string& name, const SpatialInertia<double>& M_BBo_B) {
  if (num_model_instances() != 2) {
    throw std::logic_error(
        "This model has more model instances than the default.  Please "
        "call AddRigidBody with an explicit model instance.");
  }

  return AddRigidBody(name, default_model_instance(), M_BBo_B);
}

template <typename T>
template <template <typename Scalar> class FrameType>
const FrameType<T>& MultibodyTree<T>::AddFrame(
    std::unique_ptr<FrameType<T>> frame) {
  static_assert(std::is_convertible_v<FrameType<T>*, Frame<T>*>,
                "FrameType must be a sub-class of Frame<T>.");
  if (topology_is_valid()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. "
        "Therefore adding more frames is not allowed. "
        "See documentation for Finalize() for details.");
  }
  if (frame == nullptr) {
    throw std::logic_error("Input frame is a nullptr.");
  }
  FrameIndex frame_index = topology_.add_frame(frame->body().index());
  // This test MUST be performed BEFORE frames_.push_back() and
  // owned_frames_.push_back() below. Do not move it around!
  DRAKE_DEMAND(frame_index == num_frames());
  DRAKE_DEMAND(frame->model_instance().is_valid());

  // TODO(amcastro-tri): consider not depending on setting this pointer at
  // all. Consider also removing MultibodyElement altogether.
  frame->set_parent_tree(this, frame_index);
  FrameType<T>* raw_frame_ptr = frame.get();
  frames_.push_back(raw_frame_ptr);
  this->SetElementIndex(frame->name(), frame_index, &frame_name_to_index_);
  owned_frames_.push_back(std::move(frame));
  return *raw_frame_ptr;
}

template <typename T>
template<template<typename Scalar> class FrameType, typename... Args>
const FrameType<T>& MultibodyTree<T>::AddFrame(Args&&... args) {
  static_assert(std::is_convertible_v<FrameType<T>*, Frame<T>*>,
                "FrameType must be a sub-class of Frame<T>.");
  return AddFrame(
      std::make_unique<FrameType<T>>(std::forward<Args>(args)...));
}

template <typename T>
template <template<typename Scalar> class MobilizerType>
const MobilizerType<T>& MultibodyTree<T>::AddMobilizer(
    std::unique_ptr<MobilizerType<T>> mobilizer) {
  static_assert(std::is_convertible_v<MobilizerType<T>*, Mobilizer<T>*>,
                "MobilizerType must be a sub-class of mobilizer<T>.");
  if (topology_is_valid()) {
    throw std::logic_error("This MultibodyTree is finalized already. "
                           "Therefore adding more mobilizers is not allowed. "
                           "See documentation for Finalize() for details.");
  }
  if (mobilizer == nullptr) {
    throw std::logic_error("Input mobilizer is a nullptr.");
  }
  // Verifies that the inboard/outboard frames provided by the user do belong
  // to this tree. This is a pathological case, but in theory nothing
  // (but this test) stops a user from adding frames to a tree1 and attempting
  // later to define mobilizers between those frames in a second tree2.
  mobilizer->inboard_frame().HasThisParentTreeOrThrow(this);
  mobilizer->outboard_frame().HasThisParentTreeOrThrow(this);
  const int num_positions = mobilizer->num_positions();
  const int num_velocities = mobilizer->num_velocities();
  MobilizerIndex mobilizer_index = topology_.add_mobilizer(
      mobilizer->inboard_frame().index(),
      mobilizer->outboard_frame().index(),
      num_positions, num_velocities);

  // This DRAKE_ASSERT MUST be performed BEFORE owned_mobilizers_.push_back()
  // below. Do not move it around!
  DRAKE_ASSERT(mobilizer_index == num_mobilizers());

  // TODO(sammy-tri) This effectively means that there's no way to
  // programmatically add mobilizers from outside of MultibodyTree
  // itself with multiple model instances.  I'm not convinced that
  // this is a problem.
  if (!mobilizer->model_instance().is_valid()) {
    mobilizer->set_model_instance(default_model_instance());
  }

  // TODO(amcastro-tri): consider not depending on setting this pointer at
  // all. Consider also removing MultibodyElement altogether.
  mobilizer->set_parent_tree(this, mobilizer_index);

  // Mark free bodies as needed.
  const BodyIndex outboard_body_index = mobilizer->outboard_body().index();
  topology_.get_mutable_body(outboard_body_index).is_floating =
      mobilizer->is_floating();
  topology_.get_mutable_body(outboard_body_index).has_quaternion_dofs =
      mobilizer->has_quaternion_dofs();

  MobilizerType<T>* raw_mobilizer_ptr = mobilizer.get();
  owned_mobilizers_.push_back(std::move(mobilizer));
  return *raw_mobilizer_ptr;
}

template <typename T>
template<template<typename Scalar> class MobilizerType, typename... Args>
const MobilizerType<T>& MultibodyTree<T>::AddMobilizer(Args&&... args) {
  static_assert(std::is_base_of_v<Mobilizer<T>, MobilizerType<T>>,
                "MobilizerType must be a sub-class of Mobilizer<T>.");
  return AddMobilizer(
      std::make_unique<MobilizerType<T>>(std::forward<Args>(args)...));
}

template <typename T>
template <template<typename Scalar> class ForceElementType>
const ForceElementType<T>& MultibodyTree<T>::AddForceElement(
    std::unique_ptr<ForceElementType<T>> force_element) {
  static_assert(
      std::is_convertible_v<ForceElementType<T>*, ForceElement<T>*>,
      "ForceElementType<T> must be a sub-class of ForceElement<T>.");
  if (topology_is_valid()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. Therefore adding more "
        "force elements is not allowed. "
        "See documentation for Finalize() for details.");
  }
  if (force_element == nullptr) {
    throw std::logic_error("Input force element is a nullptr.");
  }

  auto gravity_element = dynamic_cast<UniformGravityFieldElement<T>*>(
      force_element.get());
  if (gravity_element) {
    if (gravity_field_) {
      throw std::runtime_error(
          "This model already contains a gravity field element. "
          "Only one gravity field element is allowed per model.");
    }
    gravity_field_ = gravity_element;
  }

  ForceElementIndex force_element_index = topology_.add_force_element();
  // This test MUST be performed BEFORE owned_force_elements_.push_back()
  // below. Do not move it around!
  DRAKE_DEMAND(force_element_index == num_force_elements());
  DRAKE_DEMAND(force_element->model_instance().is_valid());
  force_element->set_parent_tree(this, force_element_index);

  ForceElementType<T>* raw_force_element_ptr = force_element.get();
  owned_force_elements_.push_back(std::move(force_element));
  return *raw_force_element_ptr;
}

template <typename T>
template<template<typename Scalar> class ForceElementType, typename... Args>
const ForceElementType<T>&
MultibodyTree<T>::AddForceElement(Args&&... args) {
  static_assert(std::is_base_of_v<ForceElement<T>, ForceElementType<T>>,
                "ForceElementType<T> must be a sub-class of "
                "ForceElement<T>.");
  return AddForceElement(
      std::make_unique<ForceElementType<T>>(std::forward<Args>(args)...));
}

template <typename T>
template <template<typename Scalar> class JointType>
const JointType<T>& MultibodyTree<T>::AddJoint(
    std::unique_ptr<JointType<T>> joint) {
  static_assert(std::is_convertible_v<JointType<T>*, Joint<T>*>,
                "JointType must be a sub-class of Joint<T>.");

  if (HasJointNamed(joint->name(), joint->model_instance())) {
    throw std::logic_error(
        "Model instance '" +
            instance_index_to_name_.at(joint->model_instance()) +
            "' already contains a joint named '" + joint->name() + "'. " +
            "Joint names must be unique within a given model.");
  }

  if (topology_is_valid()) {
    throw std::logic_error("This MultibodyTree is finalized already. "
                           "Therefore adding more joints is not allowed. "
                           "See documentation for Finalize() for details.");
  }
  if (joint == nullptr) {
    throw std::logic_error("Input joint is a nullptr.");
  }
  const JointIndex joint_index(owned_joints_.size());
  joint->set_parent_tree(this, joint_index);
  JointType<T>* raw_joint_ptr = joint.get();
  this->SetElementIndex(joint->name(), joint->index(), &joint_name_to_index_);
  owned_joints_.push_back(std::move(joint));
  return *raw_joint_ptr;
}

template <typename T>
template<template<typename> class JointType, typename... Args>
const JointType<T>& MultibodyTree<T>::AddJoint(
    const std::string& name,
    const Body<T>& parent,
    const std::optional<math::RigidTransform<double>>& X_PF,
    const Body<T>& child,
    const std::optional<math::RigidTransform<double>>& X_BM,
    Args&&... args) {
  static_assert(std::is_base_of_v<Joint<T>, JointType<T>>,
                "JointType<T> must be a sub-class of Joint<T>.");

  const Frame<T>* frame_on_parent{nullptr};
  if (X_PF) {
    frame_on_parent = &this->AddFrame<FixedOffsetFrame>(parent, *X_PF);
  } else {
    frame_on_parent = &parent.body_frame();
  }

  const Frame<T>* frame_on_child{nullptr};
  if (X_BM) {
    frame_on_child = &this->AddFrame<FixedOffsetFrame>(child, *X_BM);
  } else {
    frame_on_child = &child.body_frame();
  }

  const JointType<T>& joint = AddJoint(
      std::make_unique<JointType<T>>(
          name,
          *frame_on_parent, *frame_on_child,
          std::forward<Args>(args)...));
  return joint;
}

template <typename T>
const JointActuator<T>& MultibodyTree<T>::AddJointActuator(
    const std::string& name, const Joint<T>& joint, double effort_limit) {
  if (HasJointActuatorNamed(name, joint.model_instance())) {
    throw std::logic_error(
        "Model instance '" +
            instance_index_to_name_.at(joint.model_instance()) +
            "' already contains a joint actuator named '" + name + "'. " +
            "Joint actuator names must be unique within a given model.");
  }

  if (topology_is_valid()) {
    throw std::logic_error("This MultibodyTree is finalized already. "
                           "Therefore adding more actuators is not allowed. "
                           "See documentation for Finalize() for details.");
  }

  const JointActuatorIndex actuator_index =
      topology_.add_joint_actuator(joint.num_velocities());
  owned_actuators_.push_back(
      std::make_unique<JointActuator<T>>(name, joint, effort_limit));
  JointActuator<T>* actuator = owned_actuators_.back().get();
  actuator->set_parent_tree(this, actuator_index);
  this->SetElementIndex(name, actuator_index, &actuator_name_to_index_);
  return *actuator;
}

template <typename T>
ModelInstanceIndex MultibodyTree<T>::AddModelInstance(const std::string& name) {
  if (HasModelInstanceNamed(name)) {
    throw std::logic_error(
        "This model already contains a model instance named '" + name +
            "'. Model instance names must be unique within a given model.");
  }

  if (topology_is_valid()) {
    throw std::logic_error("This MultibodyTree is finalized already. "
                           "Therefore adding more model instances is not "
                           "allowed. See documentation for Finalize() for "
                           "details.");
  }
  const ModelInstanceIndex index(num_model_instances());
  this->SetElementIndex(name, index, &instance_name_to_index_);
  instance_index_to_name_[index] = name;
  return index;
}

template <typename T>
template <typename FromScalar>
Frame<T>* MultibodyTree<T>::CloneFrameAndAdd(const Frame<FromScalar>& frame) {
  FrameIndex frame_index = frame.index();

  auto frame_clone = frame.CloneToScalar(*this);
  frame_clone->set_parent_tree(this, frame_index);
  frame_clone->set_model_instance(frame.model_instance());

  Frame<T>* raw_frame_clone_ptr = frame_clone.get();
  // The order in which frames are added into frames_ is important to keep the
  // topology invariant. Therefore we index new clones according to the
  // original frame_index.
  frames_[frame_index] = raw_frame_clone_ptr;
  // The order within owned_frames_ does not matter.
  owned_frames_.push_back(std::move(frame_clone));
  return raw_frame_clone_ptr;
}

template <typename T>
template <typename FromScalar>
Body<T>* MultibodyTree<T>::CloneBodyAndAdd(const Body<FromScalar>& body) {
  const BodyIndex body_index = body.index();
  const FrameIndex body_frame_index = body.body_frame().index();

  auto body_clone = body.CloneToScalar(*this);
  body_clone->set_parent_tree(this, body_index);
  body_clone->set_model_instance(body.model_instance());
  // MultibodyTree can access selected private methods in Body through its
  // BodyAttorney.
  Frame<T>* body_frame_clone =
      &internal::BodyAttorney<T>::get_mutable_body_frame(body_clone.get());
  body_frame_clone->set_parent_tree(this, body_frame_index);
  body_frame_clone->set_model_instance(body.model_instance());

  // The order in which frames are added into frames_ is important to keep the
  // topology invariant. Therefore we index new clones according to the
  // original body_frame_index.
  frames_[body_frame_index] = body_frame_clone;
  Body<T>* raw_body_clone_ptr = body_clone.get();
  // The order in which bodies are added into owned_bodies_ is important to
  // keep the topology invariant. Therefore this method is called from
  // MultibodyTree::CloneToScalar() within a loop by original body_index.
  DRAKE_DEMAND(static_cast<int>(owned_bodies_.size()) == body_index);
  owned_bodies_.push_back(std::move(body_clone));
  return raw_body_clone_ptr;
}

template <typename T>
template <typename FromScalar>
Mobilizer<T>* MultibodyTree<T>::CloneMobilizerAndAdd(
    const Mobilizer<FromScalar>& mobilizer) {
  MobilizerIndex mobilizer_index = mobilizer.index();
  auto mobilizer_clone = mobilizer.CloneToScalar(*this);
  mobilizer_clone->set_parent_tree(this, mobilizer_index);
  mobilizer_clone->set_model_instance(mobilizer.model_instance());
  Mobilizer<T>* raw_mobilizer_clone_ptr = mobilizer_clone.get();
  owned_mobilizers_.push_back(std::move(mobilizer_clone));
  return raw_mobilizer_clone_ptr;
}

template <typename T>
template <typename FromScalar>
void MultibodyTree<T>::CloneForceElementAndAdd(
    const ForceElement<FromScalar>& force_element) {
  ForceElementIndex force_element_index = force_element.index();
  auto force_element_clone = force_element.CloneToScalar(*this);
  force_element_clone->set_parent_tree(this, force_element_index);
  force_element_clone->set_model_instance(force_element.model_instance());
  owned_force_elements_.push_back(std::move(force_element_clone));
}

template <typename T>
template <typename FromScalar>
Joint<T>* MultibodyTree<T>::CloneJointAndAdd(const Joint<FromScalar>& joint) {
  JointIndex joint_index = joint.index();
  auto joint_clone = joint.CloneToScalar(this);
  joint_clone->set_parent_tree(this, joint_index);
  joint_clone->set_model_instance(joint.model_instance());
  owned_joints_.push_back(std::move(joint_clone));
  return owned_joints_.back().get();
}

template <typename T>
template <typename FromScalar>
void MultibodyTree<T>::CloneActuatorAndAdd(
    const JointActuator<FromScalar>& actuator) {
  JointActuatorIndex actuator_index = actuator.index();
  std::unique_ptr<JointActuator<T>> actuator_clone =
      actuator.CloneToScalar(*this);
  actuator_clone->set_parent_tree(this, actuator_index);
  actuator_clone->set_model_instance(actuator.model_instance());
  owned_actuators_.push_back(std::move(actuator_clone));
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>>
MultibodyTree<T>::get_positions_and_velocities(
    const systems::Context<T>& context) const {

  // Note that we can't currently count on MultibodyPlant's API to have
  // validated this Context. The extract_qv_from_continuous() call
  // below depends on this being a LeafContext. We'll just verify that this
  // Context belongs to the owning System, since we know that's a LeafSystem.
  tree_system().ValidateContext(context);

  if (is_state_discrete()) {
    return get_discrete_state_vector(context);
  }
  const systems::VectorBase<T>& continuous_qvz_base =
      context.get_continuous_state().get_vector();
  return extract_qv_from_continuous(continuous_qvz_base);
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> MultibodyTree<T>::get_positions(
    const systems::Context<T>& context) const {
  Eigen::VectorBlock<const VectorX<T>> qv =
      get_positions_and_velocities(context);
  return make_block_segment(qv, 0, num_positions());
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> MultibodyTree<T>::get_velocities(
    const systems::Context<T>& context) const {
  Eigen::VectorBlock<const VectorX<T>> qv =
      get_positions_and_velocities(context);
  return make_block_segment(qv, num_positions(), num_velocities());
}

template <typename T>
Eigen::VectorBlock<VectorX<T>>
MultibodyTree<T>::GetMutablePositionsAndVelocities(
    systems::Context<T>* context) const {
  DRAKE_ASSERT(context != nullptr);

  // Note that we can't currently count on MultibodyPlant's API to have
  // validated this Context. The extract_mutable_qv_from_continuous() call
  // below depends on this being a LeafContext. We'll just verify that this
  // Context belongs to the owning System, since we know that's a LeafSystem.
  tree_system().ValidateContext(*context);

  if (is_state_discrete()) {
    return get_mutable_discrete_state_vector(context);
  }
  systems::VectorBase<T>& continuous_qvz_base =
      context->get_mutable_continuous_state().get_mutable_vector();
  return extract_mutable_qv_from_continuous(&continuous_qvz_base);
}

template <typename T>
Eigen::VectorBlock<VectorX<T>>
MultibodyTree<T>::get_mutable_positions_and_velocities(
    systems::State<T>* state) const {
  DRAKE_ASSERT(state != nullptr);

  // Note that we can't currently count on MultibodyPlant's API to have
  // validated this State. The extract_mutable_qv_from_continuous() call
  // below depends on this being the State of a LeafContext. We'll just verify
  // that this State was created by the owning System, since we know that's
  // a LeafSystem.
  tree_system().ValidateCreatedForThisSystem(*state);

  if (is_state_discrete()) {
    return get_mutable_discrete_state_vector(state);
  }
  systems::VectorBase<T>& continuous_qvz_base =
      state->get_mutable_continuous_state().get_mutable_vector();
  return extract_mutable_qv_from_continuous(&continuous_qvz_base);
}

// Must be implemented carefully to avoid invalidating more cache entries than
// are necessary.
// TODO(sherm1) Currently we can only get q and v together so have no way to
// invalidate just q-dependent or v-dependent cache entries.
template <typename T>
Eigen::VectorBlock<VectorX<T>> MultibodyTree<T>::GetMutablePositions(
    systems::Context<T>* context) const {
  Eigen::VectorBlock<VectorX<T>> qv = GetMutablePositionsAndVelocities(context);
  return make_mutable_block_segment(&qv, 0, num_positions());
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> MultibodyTree<T>::GetMutableVelocities(
    systems::Context<T>* context) const {
  Eigen::VectorBlock<VectorX<T>> qv = GetMutablePositionsAndVelocities(context);
  return make_mutable_block_segment(&qv, num_positions(), num_velocities());
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> MultibodyTree<T>::get_mutable_positions(
    systems::State<T>* state) const {
  Eigen::VectorBlock<VectorX<T>> qv =
      get_mutable_positions_and_velocities(state);
  return make_mutable_block_segment(&qv, 0, num_positions());
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> MultibodyTree<T>::get_mutable_velocities(
    systems::State<T>* state) const {
  Eigen::VectorBlock<VectorX<T>> qv =
      get_mutable_positions_and_velocities(state);
  return make_mutable_block_segment(&qv, num_positions(), num_velocities());
}

// Although this private method could return a const VectorX<T>& since we
// currently have only qv in the discrete state, it is used in
// get_positions_and_velocities() where we are choosing between a qv block of
// the continuous qvz state or the discrete qv state so must return a block.
// (If we add z variables to MbP in the future this would need to return a block
// anyway.)
template <typename T>
Eigen::VectorBlock<const VectorX<T>>
MultibodyTree<T>::get_discrete_state_vector(
    const systems::Context<T>& context) const {
  DRAKE_ASSERT(is_state_discrete());
  DRAKE_ASSERT(discrete_state_index_.is_valid());
  // Only q and v.
  const systems::BasicVector<T>& discrete_state_vector =
      context.get_discrete_state(discrete_state_index_);
  DRAKE_ASSERT(discrete_state_vector.size() ==
               num_positions() + num_velocities());
  return discrete_state_vector.value().head(discrete_state_vector.size());
}

template <typename T>
Eigen::VectorBlock<VectorX<T>>
MultibodyTree<T>::get_mutable_discrete_state_vector(
    systems::Context<T>* context) const {
  DRAKE_ASSERT(context != nullptr);
  DRAKE_ASSERT(is_state_discrete());
  DRAKE_ASSERT(discrete_state_index_.is_valid());
  // Only q and v.
  systems::BasicVector<T>& discrete_state_vector =
      context->get_mutable_discrete_state(discrete_state_index_);
  DRAKE_ASSERT(discrete_state_vector.size() ==
               num_positions() + num_velocities());
  return discrete_state_vector.get_mutable_value();
}

template <typename T>
Eigen::VectorBlock<VectorX<T>>
MultibodyTree<T>::get_mutable_discrete_state_vector(
    systems::State<T>* state) const {
  DRAKE_ASSERT(state != nullptr);
  DRAKE_ASSERT(is_state_discrete());
  DRAKE_ASSERT(discrete_state_index_.is_valid());
  // Only q and v.
  systems::BasicVector<T>& discrete_state_vector =
      state->get_mutable_discrete_state(discrete_state_index_);
  DRAKE_ASSERT(discrete_state_vector.size() ==
      num_positions() + num_velocities());
  return discrete_state_vector.get_mutable_value();
}

// State is continuous. This might have x = [q, v, z] -- we only want q and v.

// These next two private methods are used only in the above implementations for
// digging continuous State out of a Context, which results in a VectorBase.
// Profiling showed that it is too expensive to do a dynamic_cast for simple
// state access, which is VERY frequent. However a static_cast is safe here as
// long as the supplied VectorBase comes from the State in a LeafContext,
// because LeafContext continuous state variables are BasicVectors. We're
// depending on all callers to verify that precondition.
template <typename T>
Eigen::VectorBlock<const VectorX<T>>
MultibodyTree<T>::extract_qv_from_continuous(
    const systems::VectorBase<T>& continuous_qvz_base) const {
  DRAKE_ASSERT(!is_state_discrete());
  DRAKE_ASSERT(dynamic_cast<const systems::BasicVector<T>*>(
                   &continuous_qvz_base) != nullptr);
  const int num_qv = num_positions() + num_velocities();

  const systems::BasicVector<T>& continuous_qvz =
      static_cast<const systems::BasicVector<T>&>(continuous_qvz_base);
  DRAKE_ASSERT(continuous_qvz.size() >= num_qv);
  const VectorX<T>& qvz = continuous_qvz.value();
  return qvz.segment(0, num_qv);
}

template <typename T>
Eigen::VectorBlock<VectorX<T>>
MultibodyTree<T>::extract_mutable_qv_from_continuous(
    systems::VectorBase<T>* continuous_qvz_base) const {
  DRAKE_ASSERT(!is_state_discrete());
  DRAKE_ASSERT(continuous_qvz_base != nullptr);
  DRAKE_ASSERT(dynamic_cast<systems::BasicVector<T>*>(continuous_qvz_base) !=
               nullptr);
  const int num_qv = num_positions() + num_velocities();

  systems::BasicVector<T>& continuous_qvz =
      static_cast<systems::BasicVector<T>&>(*continuous_qvz_base);
  DRAKE_ASSERT(continuous_qvz.size() >= num_qv);
  Eigen::VectorBlock<VectorX<T>> qvz = continuous_qvz.get_mutable_value();
  return make_mutable_block_segment(&qvz, 0, num_qv);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
