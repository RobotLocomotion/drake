#include "drake/multibody/tree/multibody_tree.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include <fmt/ranges.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/body_node_world.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/rpy_floating_joint.h"
#include "drake/multibody/tree/rpy_floating_mobilizer.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/multibody/tree/weld_mobilizer.h"

namespace drake {
namespace multibody {
namespace internal {

using internal::BodyNode;
using internal::BodyNodeWorld;
using math::RigidTransform;
using math::RotationMatrix;

// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBT_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBT_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // Adds a "world" body to MultibodyTree having a NaN SpatialInertia.
  ModelInstanceIndex world_instance = AddModelInstance("WorldModelInstance");

  // `world_model_instance()` hardcodes the returned index.  Make sure it's
  // correct.
  DRAKE_DEMAND(world_instance == world_model_instance());

  world_rigid_body_ = &AddRigidBody("world", world_model_instance(),
                                    SpatialInertia<double>::NaN());

  // `default_model_instance()` hardcodes the returned index.  Make sure it's
  // correct.
  ModelInstanceIndex default_instance =
      AddModelInstance("DefaultModelInstance");
  DRAKE_DEMAND(default_instance == default_model_instance());

  const ForceElement<T>& new_field =
      AddForceElement<UniformGravityFieldElement>();
  DRAKE_DEMAND(num_force_elements() == 1);
  DRAKE_DEMAND(force_elements_[0].get() == &new_field);
}

// Registers a joint in the graph. Also register its type if that wasn't
// already done.
template <typename T>
void MultibodyTree<T>::RegisterJointAndMaybeJointTypeInGraph(
    const Joint<T>& joint) {
  const std::string type_name = joint.type_name();
  if (!link_joint_graph_.IsJointTypeRegistered(type_name)) {
    // TODO(sherm1) The Joint interface should say whether we're expecting
    //  a quaternion. Then get rid of this hack.
    const bool has_quaternion =
        type_name.find("quaternion") != std::string::npos;
    link_joint_graph_.RegisterJointType(type_name, joint.num_positions(),
                                        joint.num_velocities(), has_quaternion);
  }
  // Note changes in the graph.
  link_joint_graph_.AddJoint(joint.name(), joint.model_instance(), type_name,
                             joint.parent_body().index(),
                             joint.child_body().index());
}

template <typename T>
MultibodyTree<T>::~MultibodyTree() = default;

template <typename T>
const RigidBody<T>& MultibodyTree<T>::AddRigidBody(
    const std::string& name, ModelInstanceIndex model_instance,
    const SpatialInertia<double>& M_BBo_B) {
  if (model_instance >= num_model_instances()) {
    throw std::logic_error("Invalid model instance specified.");
  }

  if (HasBodyNamed(name, model_instance)) {
    throw std::logic_error(fmt::format(
        "Model instance '{}' already contains a body named '{}'. Body names "
        "must be unique within a given model.",
        model_instances_.get_element(model_instance).name(), name));
  }

  const RigidBody<T>& body = this->AddRigidBodyImpl(
      std::make_unique<RigidBody<T>>(name, model_instance, M_BBo_B));
  return body;
}

template <typename T>
const RigidBody<T>& MultibodyTree<T>::AddRigidBody(
    const std::string& name, const SpatialInertia<double>& M_BBo_B) {
  if (num_model_instances() != 2) {
    throw std::logic_error(
        "This model has more model instances than the default.  Please "
        "call AddRigidBody() with an explicit model instance.");
  }

  return AddRigidBody(name, default_model_instance(), M_BBo_B);
}

template <typename T>
void MultibodyTree<T>::MaybeSetUniformGravityFieldElement(
    ForceElement<T>* force_element) {
  if (auto gravity_element =
          dynamic_cast<UniformGravityFieldElement<T>*>(force_element)) {
    if (gravity_field_ != nullptr) {
      throw std::runtime_error(
          "This model already contains a gravity field element. "
          "Only one gravity field element is allowed per model.");
    }
    gravity_field_ = gravity_element;
  }
}

template <typename T>
void MultibodyTree<T>::RemoveJoint(const Joint<T>& joint) {
  DRAKE_MBT_THROW_IF_FINALIZED();
  joint.HasThisParentTreeOrThrow(this);
  JointIndex joint_index = joint.index();
  joints_.Remove(joint_index);
  link_joint_graph_.RemoveJoint(joint_index);

  // Update the ordinals for all joints with higher indices than the
  // one being removed.
  for (JointIndex index : joints_.indices()) {
    if (index > joint_index) {
      Joint<T>& mutable_joint = joints_.get_mutable_element(index);
      mutable_joint.set_ordinal(mutable_joint.ordinal() - 1);
    }
  }
}

template <typename T>
void MultibodyTree<T>::RemoveJointActuator(const JointActuator<T>& actuator) {
  DRAKE_MBT_THROW_IF_FINALIZED();
  actuator.HasThisParentTreeOrThrow(this);
  const int num_dofs_removed = actuator.num_inputs();
  JointActuatorIndex actuator_index = actuator.index();
  actuators_.Remove(actuator_index);
  num_actuated_dofs_ -= num_dofs_removed;

  // We have to adjust the dof-start allocation for all the higher-index
  // actuators.
  for (JointActuatorIndex i(actuator_index); i < actuators_.next_index(); ++i) {
    if (!actuators_.has_element(i)) continue;  // Skip removed actuators.
    JointActuator<T>& fixup_actuator = actuators_.get_mutable_element(i);
    fixup_actuator.set_actuator_dof_start(fixup_actuator.input_start() -
                                          num_dofs_removed);
  }
}

template <typename T>
const std::string& MultibodyTree<T>::GetModelInstanceName(
    ModelInstanceIndex model_instance) const {
  if (!model_instances_.has_element(model_instance)) {
    throw std::logic_error(
        fmt::format("There is no model instance id {} in the model.",
                    std::to_string(model_instance)));
  }
  return model_instances_.get_element_unchecked(model_instance).name();
}

template <typename T>
bool MultibodyTree<T>::HasUniqueFloatingBaseBodyImpl(
    ModelInstanceIndex model_instance) const {
  std::optional<BodyIndex> base_body_index =
      MaybeGetUniqueBaseBodyIndex(model_instance);
  return base_body_index.has_value() &&
         rigid_bodies_.get_element(base_body_index.value())
             .is_floating_base_body();
}

template <typename T>
const RigidBody<T>& MultibodyTree<T>::GetUniqueFloatingBaseBodyOrThrowImpl(
    ModelInstanceIndex model_instance) const {
  std::optional<BodyIndex> base_body_index =
      MaybeGetUniqueBaseBodyIndex(model_instance);
  if (!base_body_index.has_value()) {
    throw std::logic_error(
        fmt::format("Model {} does not have a unique base body.",
                    model_instances_.get_element(model_instance).name()));
  }
  const RigidBody<T>& result =
      rigid_bodies_.get_element(base_body_index.value());
  if (!result.is_floating_base_body()) {
    throw std::logic_error(fmt::format(
        "Model {} has a unique base body, but it is not a floating base body.",
        model_instances_.get_element(model_instance).name()));
  }
  return result;
}

namespace {

// Given an ElementIndex type (e.g., BodyIndex) as a template argument, returns
// the name of the corresponding class (e.g., "Body").
template <typename ElementIndex>
std::string_view GetElementClassname() {
  if constexpr (std::is_same_v<ElementIndex, BodyIndex>) {
    return "RigidBody";
  }
  if constexpr (std::is_same_v<ElementIndex, FrameIndex>) {
    return "Frame";
  }
  if constexpr (std::is_same_v<ElementIndex, JointIndex>) {
    return "Joint";
  }
  if constexpr (std::is_same_v<ElementIndex, JointActuatorIndex>) {
    return "JointActuator";
  }
  DRAKE_UNREACHABLE();
}

// Given a tree and element index, returns the corresponding `const Element&`.
template <typename T, typename ElementIndex>
const auto& GetElementByIndex(const MultibodyTree<T>& tree,
                              const ElementIndex index) {
  if constexpr (std::is_same_v<ElementIndex, BodyIndex>) {
    return tree.get_body(index);
  }
  if constexpr (std::is_same_v<ElementIndex, FrameIndex>) {
    return tree.get_frame(index);
  }
  if constexpr (std::is_same_v<ElementIndex, JointIndex>) {
    return tree.get_joint(index);
  }
  if constexpr (std::is_same_v<ElementIndex, JointActuatorIndex>) {
    return tree.get_joint_actuator(index);
  }
  DRAKE_UNREACHABLE();
}

// Shorthand type name for our name_to_index multimaps.
template <typename ElementIndex>
using NameToIndex = string_unordered_multimap<ElementIndex>;

// In service of error messages, returns a string listing the names of all
// model instances that contain an element of the given name.
template <typename T, typename ElementIndex>
std::string GetElementModelInstancesByName(
    const MultibodyTree<T>& tree, std::string_view name,
    const NameToIndex<ElementIndex>& name_to_index) {
  // Grab all of the model instance indices.
  std::vector<ModelInstanceIndex> model_instances;
  const auto [lower, upper] = name_to_index.equal_range(name);
  for (auto it = lower; it != upper; ++it) {
    const ElementIndex index = it->second;
    const auto& element = GetElementByIndex(tree, index);
    model_instances.push_back(element.model_instance());
  }
  // Sort them, because map iteration order is non-deterministic.
  std::sort(model_instances.begin(), model_instances.end());
  // Convert to indices to strings.
  std::vector<std::string_view> names;
  for (const auto& model_instance : model_instances) {
    names.push_back(tree.GetModelInstanceName(model_instance));
  }
  // Concatenate and return.
  return fmt::format("{}", fmt::join(names, ", "));
}

// Common implementation for HasBodyNamed, HasFrameNamed, etc.
template <typename T, typename ElementIndex>
bool HasElementNamed(const MultibodyTree<T>& tree, std::string_view name,
                     std::optional<ModelInstanceIndex> model_instance,
                     const NameToIndex<ElementIndex>& name_to_index) {
  // Find all elements with a matching name.
  const auto [lower, upper] = name_to_index.equal_range(name);

  // Filter for the requested model_instance, if one was provided.
  if (model_instance) {
    // Use the name lookup for its side-effect of throwing on an invalid index.
    unused(tree.GetModelInstanceName(*model_instance));
    // Search linearly on the assumption that we won't often have lots of
    // elements with the same name in different model instances.  If this
    // turns out to be incorrect, we can switch to a different data structure.
    for (auto it = lower; it != upper; ++it) {
      const ElementIndex index = it->second;
      const auto& element = GetElementByIndex(tree, index);
      if (element.model_instance() == *model_instance) {
        return true;
      }
    }
    // No (filtered) match.
    return false;
  }

  // No match.
  if (lower == upper) {
    return false;
  }

  // With no model instance requested, ensure the name is globally unique.
  if (std::next(lower) != upper) {
    const std::string_view element_classname =
        GetElementClassname<ElementIndex>();
    const std::string known_instances =
        GetElementModelInstancesByName(tree, name, name_to_index);
    throw std::logic_error(fmt::format(
        "Has{}Named(): A {} named '{}' appears in multiple model instances"
        " ({}); you must provide a model_instance argument to disambiguate.",
        element_classname, element_classname, name, known_instances));
  }

  return true;
}

// Common implementation for GetRigidBodyByName, GetFrameByName, etc.
template <typename T, typename ElementIndex>
const auto& GetElementByName(const MultibodyTree<T>& tree,
                             std::string_view name,
                             std::optional<ModelInstanceIndex> model_instance,
                             const NameToIndex<ElementIndex>& name_to_index) {
  // We fetch the model instance name as the first operation in this function
  // (even though we'll only need it for error messages) because it throws an
  // exception when the model_instance index is invalid.
  const std::string empty_name;
  const std::string& model_instance_name =
      model_instance ? tree.GetModelInstanceName(*model_instance) : empty_name;
  const std::string_view element_classname =
      GetElementClassname<ElementIndex>();

  // Find all elements with a matching name.
  auto [lower, upper] = name_to_index.equal_range(name);

  // If the name is non-existent, then say so, whether or not a specific model
  // instance was requested.
  if (lower == upper) {
    std::string message = fmt::format(
        "Get{}ByName(): There is no {} named '{}' anywhere in the model ",
        element_classname, element_classname, name);
    // We use a std::map of vectors here instead of multimap to sort in place
    // below. Note that model instances with no elements are not included in
    // the map.
    std::map<ModelInstanceIndex, std::vector<std::string_view>>
        names_by_model_instance;
    for (const auto& [element_name, element_index] : name_to_index) {
      const auto& element = GetElementByIndex(tree, element_index);
      names_by_model_instance[element.model_instance()].push_back(element_name);
    }
    if (names_by_model_instance.empty()) {
      message =
          fmt::format("Get{}ByName(): There are no {}s defined in the model",
                      element_classname, element_classname);
    } else {
      std::vector<std::string> instance_messages;
      instance_messages.reserve(names_by_model_instance.size());
      for (auto& [model_instance_index, element_names] :
           names_by_model_instance) {
        std::sort(element_names.begin(), element_names.end());
        instance_messages.push_back(
            fmt::format("valid names in model instance '{}' are: {}",
                        tree.GetModelInstanceName(model_instance_index),
                        fmt::join(element_names, ", ")));
      }
      message += fmt::format("({})", fmt::join(instance_messages, "; "));
    }
    throw std::logic_error(message);
  }

  // Filter for the requested model_instance, if one was provided.
  if (model_instance) {
    for (auto it = lower; it != upper; ++it) {
      const ElementIndex index = it->second;
      const auto& element = GetElementByIndex(tree, index);
      if (element.model_instance() == *model_instance) {
        return element;
      }
    }
    const std::string known_instances =
        GetElementModelInstancesByName(tree, name, name_to_index);
    throw std::logic_error(fmt::format(
        "Get{}ByName(): There is no {} named '{}' in the model instance named"
        " '{}', but one does exist in other model instances ({}).",
        element_classname, element_classname, name, model_instance_name,
        known_instances));
  }

  // With no model instance requested, ensure the name is globally unique.
  if (std::next(lower) != upper) {
    const std::string known_instances =
        GetElementModelInstancesByName(tree, name, name_to_index);
    throw std::logic_error(fmt::format(
        "Get{}ByName(): A {} named '{}' appears in multiple model instances"
        " ({}); you must provide a model_instance argument to disambiguate.",
        element_classname, element_classname, name, known_instances));
  }

  // Success.
  return GetElementByIndex(tree, lower->second);
}

}  // namespace

template <typename T>
int MultibodyTree<T>::NumBodiesWithName(std::string_view name) const {
  return static_cast<int>(rigid_bodies_.names_map().count(name));
}

template <typename T>
bool MultibodyTree<T>::HasBodyNamed(std::string_view name) const {
  return HasElementNamed(*this, name, std::nullopt, rigid_bodies_.names_map());
}

template <typename T>
bool MultibodyTree<T>::HasBodyNamed(std::string_view name,
                                    ModelInstanceIndex model_instance) const {
  return HasElementNamed(*this, name, model_instance,
                         rigid_bodies_.names_map());
}

template <typename T>
bool MultibodyTree<T>::HasFrameNamed(std::string_view name) const {
  return HasElementNamed(*this, name, std::nullopt, frames_.names_map());
}

template <typename T>
bool MultibodyTree<T>::HasFrameNamed(std::string_view name,
                                     ModelInstanceIndex model_instance) const {
  return HasElementNamed(*this, name, model_instance, frames_.names_map());
}

template <typename T>
bool MultibodyTree<T>::HasJointNamed(std::string_view name) const {
  return HasElementNamed(*this, name, std::nullopt, joints_.names_map());
}

template <typename T>
bool MultibodyTree<T>::HasJointNamed(std::string_view name,
                                     ModelInstanceIndex model_instance) const {
  return HasElementNamed(*this, name, model_instance, joints_.names_map());
}

template <typename T>
bool MultibodyTree<T>::HasJointActuatorNamed(std::string_view name) const {
  return HasElementNamed(*this, name, std::nullopt, actuators_.names_map());
}

template <typename T>
bool MultibodyTree<T>::HasJointActuatorNamed(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return HasElementNamed(*this, name, model_instance, actuators_.names_map());
}

template <typename T>
bool MultibodyTree<T>::HasModelInstanceNamed(std::string_view name) const {
  return model_instances_.names_map().contains(name);
}

template <typename T>
std::vector<BodyIndex> MultibodyTree<T>::GetBodyIndices(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instances_.has_element(model_instance));
  std::vector<BodyIndex> indices;
  for (const Body<T>* body : rigid_bodies_.elements()) {
    if (body->model_instance() == model_instance) {
      indices.emplace_back(body->index());
    }
  }
  return indices;
}

template <typename T>
std::vector<JointIndex> MultibodyTree<T>::GetJointIndices(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instances_.has_element(model_instance));
  std::vector<JointIndex> indices;
  for (const Joint<T>* joint : joints_.elements()) {
    if (joint->model_instance() == model_instance) {
      indices.emplace_back(joint->index());
    }
  }
  return indices;
}

template <typename T>
std::vector<JointActuatorIndex> MultibodyTree<T>::GetJointActuatorIndices(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instances_.has_element(model_instance));
  return model_instances_.get_element(model_instance).GetJointActuatorIndices();
}

template <typename T>
std::vector<JointIndex> MultibodyTree<T>::GetActuatedJointIndices(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instances_.has_element(model_instance));
  return model_instances_.get_element(model_instance).GetActuatedJointIndices();
}

template <typename T>
std::vector<FrameIndex> MultibodyTree<T>::GetFrameIndices(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instances_.has_element(model_instance));
  std::vector<FrameIndex> indices;
  for (const Frame<T>* frame : frames_.elements()) {
    if (frame->model_instance() == model_instance) {
      indices.emplace_back(frame->index());
    }
  }
  return indices;
}

template <typename T>
const Frame<T>& MultibodyTree<T>::GetFrameByName(std::string_view name) const {
  return GetElementByName(*this, name, std::nullopt, frames_.names_map());
}

template <typename T>
const Frame<T>& MultibodyTree<T>::GetFrameByName(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return GetElementByName(*this, name, model_instance, frames_.names_map());
}

template <typename T>
const RigidBody<T>& MultibodyTree<T>::GetRigidBodyByName(
    std::string_view name) const {
  return GetElementByName(*this, name, std::nullopt, rigid_bodies_.names_map());
}

template <typename T>
const RigidBody<T>& MultibodyTree<T>::GetRigidBodyByName(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return GetElementByName(*this, name, model_instance,
                          rigid_bodies_.names_map());
}

template <typename T>
const RigidBody<T>& MultibodyTree<T>::AddRigidBodyImpl(
    std::unique_ptr<RigidBody<T>> body) {
  if (is_finalized()) {
    throw std::logic_error(
        "This MultibodyTree is finalized already. "
        "Therefore adding more bodies is not allowed. "
        "See documentation for Finalize() for details.");
  }
  if (body == nullptr) {
    throw std::logic_error("Input body is a nullptr.");
  }

  DRAKE_DEMAND(body->model_instance().is_valid());

  const BodyIndex body_index(num_bodies());

  if (body_index == 0) {
    // We're adding the first RigidBody -- must be World!
    DRAKE_DEMAND(body->name() == "world");
    DRAKE_DEMAND(body->model_instance() == world_model_instance());
    // The LinkJointGraph should already contain only World.
    DRAKE_DEMAND(ssize(link_joint_graph_.links()) == 1);
    DRAKE_DEMAND(link_joint_graph_.link_by_index(body_index).name() == "world");
  } else {
    // Make note in the graph of the new rigid body.
    link_joint_graph_.AddLink(body->name(), body->model_instance());
  }

  body->set_parent_tree(this, body_index);
  // MultibodyTree can access selected private methods in RigidBody through its
  // RigidBodyAttorney.
  // - Register body frame.
  Frame<T>* body_frame =
      &internal::RigidBodyAttorney<T>::get_mutable_body_frame(body.get());
  const FrameIndex body_frame_index(num_frames());
  body_frame->set_parent_tree(this, body_frame_index);
  DRAKE_DEMAND(body_frame->name() == body->name());
  frames_.AddBorrowed(body_frame);
  // - Register body.
  return rigid_bodies_.Add(std::move(body));
}

template <typename T>
const Joint<T>& MultibodyTree<T>::GetJointByNameImpl(
    std::string_view name,
    std::optional<ModelInstanceIndex> model_instance) const {
  return GetElementByName(*this, name, model_instance, joints_.names_map());
}

template <typename T>
void MultibodyTree<T>::ThrowJointSubtypeMismatch(
    const Joint<T>& joint, std::string_view desired_type) const {
  throw std::logic_error(fmt::format(
      "GetJointByName(): Joint '{}' in model instance '{}' is not of type {} "
      "but of type {}.",
      joint.name(), model_instances_.get_element(joint.model_instance()).name(),
      desired_type, NiceTypeName::Get(joint)));
}

template <typename T>
const JointActuator<T>& MultibodyTree<T>::GetJointActuatorByName(
    std::string_view name) const {
  return GetElementByName(*this, name, std::nullopt, actuators_.names_map());
}

template <typename T>
const JointActuator<T>& MultibodyTree<T>::GetJointActuatorByName(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return GetElementByName(*this, name, model_instance, actuators_.names_map());
}

template <typename T>
ModelInstanceIndex MultibodyTree<T>::GetModelInstanceByName(
    std::string_view name) const {
  auto& names = model_instances_.names_map();
  const auto it = names.find(name);
  if (it == names.end()) {
    std::vector<std::string_view> valid_names;
    valid_names.reserve(names.size());
    for (const auto& [valid_name, _] : names) {
      valid_names.push_back(valid_name);
    }
    std::sort(valid_names.begin(), valid_names.end());
    throw std::logic_error(fmt::format(
        "GetModelInstanceByName(): There is no model instance named '{}'. The "
        "current model instances are '{}'.",
        name, fmt::join(valid_names, "', '")));
  }
  return it->second;
}

template <typename T>
std::set<BodyIndex> MultibodyTree<T>::GetBodiesKinematicallyAffectedBy(
    const std::vector<JointIndex>& joint_indexes) const {
  // For each Joint, find its implementing Mobod and collect the RigidBodies
  // in the subtree rooted by that Mobod. Duplicates are weeded out
  // and the returned BodyIndexes are sorted.
  std::set<BodyIndex> links;
  for (const JointIndex& joint_index : joint_indexes) {
    if (get_joint(joint_index).num_velocities() == 0) continue;  // Skip welds.
    const MobodIndex mobod_index =
        graph().joint_by_index(joint_index).mobod_index();
    DRAKE_DEMAND(mobod_index.is_valid());
    const std::vector<BodyIndex> subtree_links =
        forest().FindSubtreeLinks(mobod_index);
    links.insert(subtree_links.cbegin(), subtree_links.cend());
  }
  return links;
}

template <typename T>
std::set<BodyIndex> MultibodyTree<T>::GetBodiesOutboardOfBodies(
    const std::vector<BodyIndex>& body_indexes) const {
  // For each given rigid body, find the Mobod it follows and collect the bodies
  // in the subtree rooted by that Mobod. Duplicates are weeded out and the
  // returned BodyIndexes are sorted.
  std::set<BodyIndex> bodies;
  for (const BodyIndex& body_index : body_indexes) {
    const MobodIndex mobod_index =
        graph().link_by_index(body_index).mobod_index();
    DRAKE_DEMAND(mobod_index.is_valid());
    const std::vector<BodyIndex> subtree_bodies =
        forest().FindSubtreeLinks(mobod_index);
    bodies.insert(subtree_bodies.cbegin(), subtree_bodies.cend());
  }
  return bodies;
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetActuationFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& u) const {
  return model_instances_.get_element(model_instance).GetActuationFromArray(u);
}

template <typename T>
void MultibodyTree<T>::SetActuationInArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& u_instance,
    EigenPtr<VectorX<T>> u) const {
  model_instances_.get_element(model_instance)
      .SetActuationInArray(u_instance, u);
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetPositionsFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& q) const {
  return model_instances_.get_element(model_instance).GetPositionsFromArray(q);
}

template <typename T>
void MultibodyTree<T>::GetPositionsFromArray(
    ModelInstanceIndex model_instance, const Eigen::Ref<const VectorX<T>>& q,
    drake::EigenPtr<VectorX<T>> q_out) const {
  model_instances_.get_element(model_instance).GetPositionsFromArray(q, q_out);
}

template <class T>
void MultibodyTree<T>::SetPositionsInArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& q_instance,
    EigenPtr<VectorX<T>> q) const {
  model_instances_.get_element(model_instance)
      .SetPositionsInArray(q_instance, q);
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetVelocitiesFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& v) const {
  return model_instances_.get_element(model_instance).GetVelocitiesFromArray(v);
}

template <typename T>
void MultibodyTree<T>::GetVelocitiesFromArray(
    ModelInstanceIndex model_instance, const Eigen::Ref<const VectorX<T>>& v,
    drake::EigenPtr<VectorX<T>> v_out) const {
  model_instances_.get_element(model_instance).GetVelocitiesFromArray(v, v_out);
}

template <class T>
void MultibodyTree<T>::SetVelocitiesInArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& v_instance,
    EigenPtr<VectorX<T>> v) const {
  model_instances_.get_element(model_instance)
      .SetVelocitiesInArray(v_instance, v);
}

/* Create Joint implementations from the already-built SpanningForest. Joints
are implemented with a Mobilizer (either forward or reversed), unless they are
welds between Links that were merged onto a single Mobod. We visit the forest's
mobilized bodies (Mobods) in depth-first order and create Mobilizers in the same
order as Mobods. We make a stub 0th Mobilizer for World so that Mobilizers and
Mobods are numbered identically. (Think of it as a weld of World to the
universe.) Joints that are interior to optimized WeldedLinksAssemblies
(composite bodies) won't get modeled at all since they don't appear in the
forest. */
template <typename T>
void MultibodyTree<T>::CreateJointImplementations() {
  DRAKE_DEMAND(!is_finalized());

  // These are the Joint type names for the joints that are currently
  // reversible.
  const std::set<std::string> reversible{PrismaticJoint<double>::kTypeName,
                                         RevoluteJoint<double>::kTypeName,
                                         WeldJoint<double>::kTypeName};

  // Mobods are in depth-first order, starting with World.
  for (const auto& mobod : forest().mobods()) {
    if (mobod.is_world()) {
      // No associated Joint but we do want a stub "weld" Mobilizer so that
      // Mobods, BodyNodes, and Mobilizers have identical numbering.
      auto dummy_weld = std::make_unique<internal::WeldMobilizer<T>>(
          mobod, world_frame(), world_frame());
      dummy_weld->set_model_instance(world_model_instance());
      dummy_weld->set_parent_tree(this, MobodIndex(0));
      mobilizers_.push_back(std::move(dummy_weld));
      continue;
    }

    const JointIndex joint_index =
        forest().joints(mobod.joint_ordinal()).index();
    Joint<T>& joint = joints_.get_mutable_element(joint_index);

    // We allow reversed mobilizers only for a subset of joint types.
    if (mobod.is_reversed() && !reversible.contains(joint.type_name())) {
      throw std::runtime_error(fmt::format(
          "MultibodyPlant::Finalize(): parent/child ordering for "
          "{} joint {} in model instance {} would have to be reversed "
          "to make a tree-structured model for this system. "
          "Currently Drake does not support reversed {} joints. The joints "
          "that can be reversed are: {}. Reverse the ordering in your joint "
          "definition so that its parent body is closer to World in the tree.",
          joint.type_name(), joint.name(),
          GetModelInstanceName(joint.model_instance()), joint.type_name(),
          fmt::join(reversible, ", ")));
    }

    std::unique_ptr<Mobilizer<T>> owned_mobilizer = joint.Build(mobod, this);
    Mobilizer<T>* mobilizer = owned_mobilizer.get();
    mobilizer->set_model_instance(joint.model_instance());
    mobilizer->set_is_ephemeral(joint.is_ephemeral());

    // Mark floating base bodies as needed. Note the strict definition:
    // (1) the inboard joint must have six degrees of freedom, and
    // (2) that joint must be ephemeral (added automatically).
    bool is_floating_base_mobilizer =
        mobilizer->has_six_dofs() && mobilizer->is_ephemeral();
    mobilizer->set_is_floating_base_mobilizer(is_floating_base_mobilizer);

    AddMobilizer(std::move(owned_mobilizer));  // ownership->tree
    DRAKE_DEMAND(mobilizer->index() == mobod.index());
    // Record the joint to mobilizer map.
    joint_to_mobilizer_[joint_index] = mobilizer->index();
  }
}

template <typename T>
const Mobilizer<T>& MultibodyTree<T>::GetFreeBodyMobilizerOrThrow(
    const RigidBody<T>& body) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  DRAKE_DEMAND(body.index() != world_index());
  const LinkJointGraph::Link& link = forest().link_by_index(body.index());
  const Mobilizer<T>& mobilizer = get_mobilizer(link.mobod_index());
  if (!mobilizer.has_six_dofs()) {
    throw std::logic_error("Body '" + body.name() + "' is not a free body.");
  }
  return mobilizer;
}

template <typename T>
const Frame<T>& MultibodyTree<T>::AddOrGetJointFrame(
    const RigidBody<T>& body,
    const std::optional<math::RigidTransform<double>>& X_BF,
    ModelInstanceIndex joint_instance, std::string_view joint_name,
    std::string_view frame_suffix) {
  if (X_BF.has_value()) {
    return this->AddFrame<FixedOffsetFrame>(
        fmt::format("{}_{}", joint_name, frame_suffix), body.body_frame(),
        *X_BF, joint_instance);
  }
  return body.body_frame();
}

template <typename T>
void MultibodyTree<T>::FinalizeInternals() {
  DRAKE_DEMAND(!is_finalized());
  DRAKE_DEMAND(graph().forest_is_valid());

  // Give different multibody elements the chance to perform any finalize-time
  // setup.
  for (const auto& body_index : rigid_bodies_.indices()) {
    // This sets the body's is_floating_base_body() flag appropriately.
    rigid_bodies_.get_mutable_element(body_index).SetTopology();
  }
  for (const auto& frame_index : frames_.indices()) {
    // We (re)set the topology on all frames. The rigid body frames' topologies
    // will already have been set in the body loop immediately above, but it
    // doesn't hurt to set them again. The important thing is that the non-body
    // frames are being set here for the first time.
    frames_.get_mutable_element(frame_index).SetTopology();
  }
  for (const auto& mobilizer : mobilizers_) {
    mobilizer->SetTopology();
  }
  for (const auto& force_element : force_elements_) {
    force_element->SetTopology();
  }
  for (const auto& actuator_index : actuators_.indices()) {
    actuators_.get_mutable_element(actuator_index).SetTopology();
  }

  // TODO(sherm1) Consider building the level collection while building the
  //  spanning forest.
  body_node_levels_.resize(forest().height());
  for (MobodIndex mobod_index(1); mobod_index < forest().num_mobods();
       ++mobod_index) {
    const SpanningForest::Mobod& mobod = forest().mobods(mobod_index);
    body_node_levels_[mobod.level()].push_back(mobod_index);
  }

  // Creates BodyNodes:
  // This recursion order ensures that a BodyNode's parent is created before the
  // node itself, since BodyNode objects are in Depth First Traversal order.
  for (MobodIndex mobod_index(0); mobod_index < forest().num_mobods();
       ++mobod_index) {
    CreateBodyNode(mobod_index);
  }

  FinalizeModelInstances();

  // For each floating base body, transfer its default pose to its newly-added
  // ephemeral floating joint and route its future default pose setting &
  // querying through its joint representation.
  for (JointIndex i : GetJointIndices()) {
    auto& joint = joints_.get_mutable_element(i);
    const RigidBody<T>& body = joint.child_body();
    if (RigidBodyAttorney<T>::is_floating_base_body_pre_finalize(body)) {
      DRAKE_DEMAND(joint.is_ephemeral());
      const auto [quaternion, translation] =
          GetDefaultFloatingBaseBodyPoseAsQuaternionVec3Pair(body);
      joint.SetDefaultPosePair(quaternion, translation);
      default_body_poses_[body.index()] = joint.index();
    }
  }

  is_finalized_ = true;
}

template <typename T>
void MultibodyTree<T>::Finalize() {
  DRAKE_MBT_THROW_IF_FINALIZED();

  /* Given the user-defined directed graph of Links and Joints, decide how we're
  going to model this using a spanning forest comprised of
    - bodies and their mobilizers, paired as "mobilized bodies" (mobods) and
      directed by inboard/outboard edges, and
    - added constraints where needed to close kinematic loops in the graph.
  The modeler sorts the mobilized bodies into depth-first order. Note that
  welded-together Links (in a WeldedLinksAssembly) may be optimized into a
  single mobilized body so there can be more Links than Mobods.

  Every Link will be modeled with one "primary" body and possibly several
  "shadow" bodies. Every non-Weld Joint will be modeled with a mobilizer, with
  the Joint's parent/child connections mapped to the mobilizer's
  inboard/outboard connection or to the reverse, as necessary for the mobilizers
  to form properly-directed trees. Every body in a tree must have a path in
  the inboard direction connecting it to World. If necessary, additional
  "ephemeral" floating (6 dof) or weld (0 dof) joints are added to make the
  final connection to World.

  During the modeling process, the LinkJointGraph is augmented with "ephemeral
  elements" to provide a uniform interface to the additional elements that were
  required to build the model. Below, we will augment the MultibodyPlant
  elements to match, so that advanced users can use the familiar Plant API to
  access and control these ephemeral elements. The process of modeling Joints
  with available Mobilizers may introduce ephemeral Frames as well. All
  ephemeral elements must be marked as such in the base MultibodyElement class;
  public API element.is_ephemeral() is available to check. */
  link_joint_graph_.BuildForest();
  const LinkJointGraph& graph = link_joint_graph_;

  /* Add Links, Joints, and Constraints that were created during the modeling
  process (BuildForest()), which augmented the graph with them. We call those
  "ephemeral" elements. */

  // TODO(sherm1) Add shadow links and loop constraints.
  if (!graph.loop_constraints().empty()) {
    link_joint_graph_.InvalidateForest();
    throw std::runtime_error(fmt::format(
        "The bodies and joints of this system form one or "
        "more loops in the system graph. Drake currently does not "
        "support automatic modeling of such systems; however, they "
        "can be modeled with some input changes. See "
        "https://drake.mit.edu/troubleshooting.html"
        "#mbp-loops-in-graph for advice on how to model systems with loops."));
  }

  /* Add the ephemeral Joints. */
  for (JointOrdinal i(graph.num_user_joints()); i < graph.num_joints(); ++i) {
    const LinkJointGraph::Joint& added_joint = graph.joints(i);
    DRAKE_DEMAND(added_joint.parent_link_index() == BodyIndex(0));

    const Joint<T>& new_joint = [this, &added_joint]() -> const Joint<T>& {
      if (added_joint.traits_index() ==
          LinkJointGraph::quaternion_floating_joint_traits_index()) {
        return AddEphemeralJoint<QuaternionFloatingJoint>(
            added_joint.name(), world_body(),
            get_body(added_joint.child_link_index()));
      }
      if (added_joint.traits_index() ==
          LinkJointGraph::rpy_floating_joint_traits_index()) {
        return AddEphemeralJoint<RpyFloatingJoint>(
            added_joint.name(), world_body(),
            get_body(added_joint.child_link_index()));
      }
      if (added_joint.traits_index() ==
          LinkJointGraph::weld_joint_traits_index()) {
        return AddEphemeralJoint<WeldJoint>(
            added_joint.name(), world_body(),
            get_body(added_joint.child_link_index()),
            math::RigidTransform<double>());
      }
      DRAKE_UNREACHABLE();
    }();
    DRAKE_DEMAND(new_joint.index() == added_joint.index());
  }

  // Model each Joint with an appropriate Mobilizer.
  CreateJointImplementations();

  // Perform final setup of all the MultibodyTree internals. This step is
  // required both here and after cloning of a MultibodyTree. Actions:
  // - invoke SetTopology() on all elements to give them a chance to
  //   perform any finalize-time setup,
  // - create BodyNodes corresponding to each Mobilizer and Mobod,
  // - set default poses for each floating base body,
  // - populate the ModelInstance objects with their mobilizers and actuators,
  // - set the "is_finalized" flag to true.
  FinalizeInternals();
}

template <typename T>
void MultibodyTree<T>::CreateBodyNode(MobodIndex mobod_index) {
  const SpanningForest::Mobod& mobod = forest().mobods(mobod_index);
  const LinkJointGraph::Link& active_link =
      forest().links(mobod.link_ordinal());
  const BodyIndex body_index = active_link.index();

  const RigidBody<T>& body = rigid_bodies_.get_element(body_index);

  std::unique_ptr<BodyNode<T>> body_node;
  const Mobilizer<T>* const mobilizer = mobilizers_[mobod_index].get();
  if (body_index == world_index()) {
    body_node = std::make_unique<BodyNodeWorld<T>>(&world_body(), mobilizer);
  } else {
    BodyNode<T>* parent_node = body_nodes_[mobod.inboard()].get();

    // Only the mobilizer knows how to create a BodyNode with compile-time
    // fixed sizes.
    body_node = mobilizer->CreateBodyNode(parent_node, &body, mobilizer);
    parent_node->add_child_node(body_node.get());
  }
  body_node->set_parent_tree(this, mobod_index);
  body_node->SetTopology();

  body_nodes_.push_back(std::move(body_node));
}

template <typename T>
void MultibodyTree<T>::FinalizeModelInstances() {
  // Add all of our mobilizers and joint actuators to the appropriate instance.
  for (const auto& mobilizer : mobilizers_) {
    model_instances_.get_mutable_element(mobilizer->model_instance())
        .add_mobilizer(mobilizer.get());
  }

  // N.B. The result of the code below is that actuators are sorted by
  // JointActuatorIndex within each model instance. If this was not true,
  // ModelInstance::add_joint_actuator() would throw.
  for (const JointActuator<T>* actuator : actuators_.elements()) {
    model_instances_.get_mutable_element(actuator->model_instance())
        .add_joint_actuator(actuator);
  }
}

template <typename T>
std::unique_ptr<systems::LeafContext<T>>
MultibodyTree<T>::CreateDefaultContext() const {
  if (tree_system_ == nullptr) {
    throw std::runtime_error(
        "MultibodyTree::CreateDefaultContext(): can only be called from a "
        "MultibodyTree that is owned by a MultibodyPlant / "
        "MultibodyTreeSystem");
  }
  return dynamic_pointer_cast<systems::LeafContext<T>>(
      tree_system_->CreateDefaultContext());
}

template <typename T>
void MultibodyTree<T>::SetDefaultState(const systems::Context<T>& context,
                                       systems::State<T>* state) const {
  for (const auto& mobilizer : mobilizers_) {
    mobilizer->set_default_state(context, state);
  }
}

template <typename T>
void MultibodyTree<T>::SetRandomState(const systems::Context<T>& context,
                                      systems::State<T>* state,
                                      RandomGenerator* generator) const {
  for (const auto& mobilizer : mobilizers_) {
    mobilizer->set_random_state(context, state, generator);
  }
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetPositionsAndVelocities(
    const systems::Context<T>& context,
    ModelInstanceIndex model_instance) const {
  VectorX<T> instance_state_vector(num_states(model_instance));

  GetPositionsAndVelocities(context, model_instance, &instance_state_vector);

  return instance_state_vector;
}

template <typename T>
void MultibodyTree<T>::GetPositionsAndVelocities(
    const systems::Context<T>& context, ModelInstanceIndex model_instance,
    EigenPtr<VectorX<T>> qv_out) const {
  DRAKE_DEMAND(qv_out != nullptr);

  Eigen::VectorBlock<const VectorX<T>> state_vector =
      get_positions_and_velocities(context);

  if (qv_out->size() !=
      num_positions(model_instance) + num_velocities(model_instance))
    throw std::logic_error("Output array is not properly sized.");

  auto qv_out_head = qv_out->head(num_positions(model_instance));
  auto qv_out_tail = qv_out->tail(num_velocities(model_instance));

  GetPositionsFromArray(model_instance, state_vector.head(num_positions()),
                        &qv_out_head);
  GetVelocitiesFromArray(model_instance, state_vector.tail(num_velocities()),
                         &qv_out_tail);
}

template <typename T>
void MultibodyTree<T>::SetPositionsAndVelocities(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& instance_state,
    systems::Context<T>* context) const {
  Eigen::VectorBlock<VectorX<T>> state_vector =
      GetMutablePositionsAndVelocities(context);
  Eigen::VectorBlock<VectorX<T>> q =
      make_mutable_block_segment(&state_vector, 0, num_positions());
  Eigen::VectorBlock<VectorX<T>> v = make_mutable_block_segment(
      &state_vector, num_positions(), num_velocities());
  SetPositionsInArray(model_instance,
                      instance_state.head(num_positions(model_instance)), &q);
  SetVelocitiesInArray(model_instance,
                       instance_state.tail(num_velocities(model_instance)), &v);
}

template <typename T>
RigidTransform<T> MultibodyTree<T>::GetFreeBodyPoseOrThrow(
    const systems::Context<T>& context, const RigidBody<T>& body) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  const Mobilizer<T>& mobilizer = GetFreeBodyMobilizerOrThrow(body);
  return mobilizer.CalcAcrossMobilizerTransform(context);
}

template <typename T>
void MultibodyTree<T>::SetDefaultFloatingBaseBodyPose(
    const RigidBody<T>& body, const RigidTransform<double>& X_WB) {
  if (!default_body_poses_.contains(body.index()) ||
      std::holds_alternative<
          std::pair<Eigen::Quaternion<double>, Vector3<double>>>(
          default_body_poses_.at(body.index()))) {
    default_body_poses_[body.index()] =
        std::make_pair(X_WB.rotation().ToQuaternion(), X_WB.translation());
    return;
  }
  auto& joint = joints_.get_mutable_element(
      std::get<JointIndex>(default_body_poses_.at(body.index())));
  joint.SetDefaultPose(X_WB);
}

template <typename T>
RigidTransform<double> MultibodyTree<T>::GetDefaultFloatingBaseBodyPose(
    const RigidBody<T>& body) const {
  const std::pair<Eigen::Quaternion<double>, Vector3<double>> pose =
      GetDefaultFloatingBaseBodyPoseAsQuaternionVec3Pair(body);
  return RigidTransform<double>(pose.first, pose.second);
}

template <typename T>
std::pair<Eigen::Quaternion<double>, Vector3<double>>
MultibodyTree<T>::GetDefaultFloatingBaseBodyPoseAsQuaternionVec3Pair(
    const RigidBody<T>& body) const {
  if (!default_body_poses_.contains(body.index())) {
    return std::make_pair(Eigen::Quaternion<double>::Identity(),
                          Vector3<double>::Zero());
  }
  const auto& default_body_pose = default_body_poses_.at(body.index());
  if (std::holds_alternative<JointIndex>(default_body_pose)) {
    const auto& joint =
        joints_.get_element(std::get<JointIndex>(default_body_pose));
    return joint.GetDefaultPosePair();
  }
  return std::get<std::pair<Eigen::Quaternion<double>, Vector3<double>>>(
      default_body_pose);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyPoseOrThrow(
    const RigidBody<T>& body, const RigidTransform<T>& X_WB,
    systems::Context<T>* context) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  SetFreeBodyPoseOrThrow(body, X_WB, *context, &context->get_mutable_state());
}

template <typename T>
void MultibodyTree<T>::SetFreeBodySpatialVelocityOrThrow(
    const RigidBody<T>& body, const SpatialVelocity<T>& V_WB,
    systems::Context<T>* context) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  SetFreeBodySpatialVelocityOrThrow(body, V_WB, *context,
                                    &context->get_mutable_state());
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyPoseOrThrow(
    const RigidBody<T>& body, const RigidTransform<T>& X_WB,
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  const Mobilizer<T>& mobilizer = GetFreeBodyMobilizerOrThrow(body);
  const RotationMatrix<T>& R_WB = X_WB.rotation();
  mobilizer.SetPosePair(context, R_WB.ToQuaternion(), X_WB.translation(),
                        state);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodySpatialVelocityOrThrow(
    const RigidBody<T>& body, const SpatialVelocity<T>& V_WB,
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  const Mobilizer<T>& mobilizer = GetFreeBodyMobilizerOrThrow(body);
  mobilizer.SetSpatialVelocity(context, V_WB, state);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyRandomTranslationDistributionOrThrow(
    const RigidBody<T>& body,
    const Vector3<symbolic::Expression>& translation) {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  Mobilizer<T>& mobilizer =
      get_mutable_variant(GetFreeBodyMobilizerOrThrow(body));
  QuaternionFloatingMobilizer<T>* maybe_quaternion_mobilizer =
      dynamic_cast<QuaternionFloatingMobilizer<T>*>(&mobilizer);
  if (maybe_quaternion_mobilizer != nullptr) {
    maybe_quaternion_mobilizer->set_random_translation_distribution(
        translation);
    return;
  }
  RpyFloatingMobilizer<T>* maybe_rpy_mobilizer =
      dynamic_cast<RpyFloatingMobilizer<T>*>(&mobilizer);
  if (maybe_rpy_mobilizer != nullptr) {
    maybe_rpy_mobilizer->set_random_translation_distribution(translation);
    return;
  }

  DRAKE_UNREACHABLE();  // Only the two floating joints are possible.
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyRandomRotationDistributionOrThrow(
    const RigidBody<T>& body,
    const Eigen::Quaternion<symbolic::Expression>& rotation) {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  Mobilizer<T>& mobilizer =
      get_mutable_variant(GetFreeBodyMobilizerOrThrow(body));
  QuaternionFloatingMobilizer<T>* maybe_quaternion_mobilizer =
      dynamic_cast<QuaternionFloatingMobilizer<T>*>(&mobilizer);
  if (maybe_quaternion_mobilizer == nullptr) {
    // Note the (likely reasonable) assumption that a free body uses either
    // a quaternion floating joint or an rpy floating joint.
    throw std::logic_error(fmt::format(
        "{}(): Requires a {} joint but free body {} uses an {} joint. "
        "Use SetFreeBodyRandomAnglesDistribution() instead.",
        __func__, QuaternionFloatingJoint<T>::kTypeName, body.name(),
        RpyFloatingJoint<T>::kTypeName));
  }
  maybe_quaternion_mobilizer->set_random_quaternion_distribution(rotation);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyRandomAnglesDistributionOrThrow(
    const RigidBody<T>& body,
    const math::RollPitchYaw<symbolic::Expression>& angles) {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  Mobilizer<T>& mobilizer =
      get_mutable_variant(GetFreeBodyMobilizerOrThrow(body));
  RpyFloatingMobilizer<T>* maybe_rpy_mobilizer =
      dynamic_cast<RpyFloatingMobilizer<T>*>(&mobilizer);
  if (maybe_rpy_mobilizer == nullptr) {
    // Note the (likely reasonable) assumption that a free body uses either
    // a quaternion floating joint or an rpy floating joint.
    throw std::logic_error(fmt::format(
        "{}(): Requires an {} joint but free body {} uses a {} joint. "
        "Use SetFreeBodyRandomRotationDistribution() instead.",
        __func__, RpyFloatingJoint<T>::kTypeName, body.name(),
        QuaternionFloatingJoint<T>::kTypeName));
  }
  maybe_rpy_mobilizer->set_random_angles_distribution(angles.vector());
}

// Note that the result is indexed by BodyIndex, not MobodIndex.
template <typename T>
void MultibodyTree<T>::CalcAllBodyPosesInWorld(
    const systems::Context<T>& context,
    std::vector<RigidTransform<T>>* X_WB) const {
  DRAKE_THROW_UNLESS(X_WB != nullptr);
  if (ssize(*X_WB) != num_bodies()) {
    X_WB->resize(num_bodies(), RigidTransform<T>::Identity());
  }
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const MobodIndex mobod_index = get_body(body_index).mobod_index();
    X_WB->at(body_index) = pc.get_X_WB(mobod_index);
  }
}

// Note that the result is indexed by BodyIndex, not MobodIndex.
template <typename T>
void MultibodyTree<T>::CalcAllBodySpatialVelocitiesInWorld(
    const systems::Context<T>& context,
    std::vector<SpatialVelocity<T>>* V_WB) const {
  DRAKE_THROW_UNLESS(V_WB != nullptr);
  if (ssize(*V_WB) != num_bodies()) {
    V_WB->resize(num_bodies(), SpatialVelocity<T>::Zero());
  }
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const MobodIndex mobod_index = get_body(body_index).mobod_index();
    V_WB->at(body_index) = vc.get_V_WB(mobod_index);
  }
}

template <typename T>
void MultibodyTree<T>::CalcPositionKinematicsCache(
    const systems::Context<T>& context, PositionKinematicsCache<T>* pc) const {
  DRAKE_DEMAND(pc != nullptr);

  const FrameBodyPoseCache<T>& frame_body_pose_cache =
      EvalFrameBodyPoses(context);

  const Eigen::VectorBlock<const VectorX<T>> q_block = get_positions(context);
  const T* q = q_block.data();

  // With the kinematics information across mobilizers and the kinematics
  // information for each body, we are now in position to perform a base-to-tip
  // recursion to update world positions and parent to child body transforms.
  // This skips the world, level = 0.
  // Performs a base-to-tip recursion computing body poses.
  // Skip the World which is mobod_index(0).
  for (MobodIndex mobod_index(1); mobod_index < num_mobods(); ++mobod_index) {
    const BodyNode<T>& node = *body_nodes_[mobod_index];
    DRAKE_ASSERT(node.mobod_index() == mobod_index);

    // Update per-node kinematics.
    node.CalcPositionKinematicsCache_BaseToTip(frame_body_pose_cache, q, pc);
  }
}

template <typename T>
void MultibodyTree<T>::CalcBlockSystemJacobianCache(
    const systems::Context<T>& context,
    BlockSystemJacobianCache<T>* sjc) const {
  DRAKE_DEMAND(sjc != nullptr);

  const PositionKinematicsCache<T>& pc = this->EvalPositionKinematics(context);
  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);
  std::vector<Eigen::MatrixX<T>>& tree_jacobians =
      sjc->mutable_block_system_jacobian();

  // The iᵗʰ tree (treeᵢ) generates an nᵢ x mᵢ block. Note that we are relying
  // on the BlockSystemJacobianCache initialization to have zeroed out the
  // entire Jacobian so that it is safe for us to fill in only the pieces that
  // we know might be non-zero. The fill-in structure is always the same once
  // allocated so we only need to rewrite the non-zero pieces.
  for (const SpanningForest::Tree& tree : forest().trees()) {
    const int n = 6 * tree.num_mobods();
    const int m = tree.nv();
    MatrixX<T>& J = tree_jacobians[tree.index()];  // J_V_WB_W(tree)
    DRAKE_DEMAND(J.rows() == n && J.cols() == m);

    // The J(0,0) element for this tree is the Jacobian of the base body
    // w.r.t. to its first mobility (velocity coordinate). Both the bodies
    // (rows) and mobilities (columns) are numbered consecutively from there.
    const MobodIndex base_index = tree.base_mobod();  // row 0
    const int base_v_start = tree.v_start();          // column 0

    // Each mobod Bi fills in a 6xm row of the Jacobian. All the columns
    // outboard of Bi are left zero. Those inboard of Bi are the same as the
    // parent's corresponding row except shifted to Bi's body origin Bio.
    // Note: Tree mobods never include World.
    for (const SpanningForest::Mobod& mobod : tree) {
      const MobodIndex index_B = mobod.index();
      const MobodIndex index_P = mobod.inboard();
      const SpanningForest::Mobod& parent = forest().mobods(index_P);
      const Vector3<T>& p_PoBo_W = pc.get_p_PoBo_W(index_B);
      const int row_B = 6 * (index_B - base_index);
      const int row_P = 6 * (index_P - base_index);

      // Run through all the parent's non-zero Jacobian entries, shift to Bio
      // and put the result in the same column of the new row. We are always
      // shifting the _parent_'s entries; we're running down the ancestors
      // just to find all the mobilities that affect the parent.
      const SpanningForest::Mobod* ancestor = &parent;
      while (!ancestor->is_world()) {
        for (int i = 0; i < ancestor->nv(); ++i) {
          const int avi = ancestor->v_start() + i;
          const int col = avi - base_v_start;
          const auto Jvi_V_WP = J.template block<6, 1>(row_P, col);
          const Vector3<T> w_WP = Jvi_V_WP.template head<3>();
          const Vector3<T> v_WP = Jvi_V_WP.template tail<3>();
          auto Jvi_V_WB = J.template block<6, 1>(row_B, col);
          Jvi_V_WB.template head<3>() = w_WP;
          Jvi_V_WB.template tail<3>() = v_WP + w_WP.cross(p_PoBo_W);
        }
        ancestor = &forest().mobods(ancestor->inboard());
      }
      // Parent contributions are done, just need to fill in the local
      // contribution from H_PB_W.
      for (int i = 0; i < mobod.nv(); ++i) {
        const int vi = mobod.v_start() + i;
        const int col = vi - base_v_start;
        const Vector6<T>& Hi_PB_W = H_PB_W_cache[vi];
        auto Jvi_V_WB = J.template block<6, 1>(row_B, col);
        Jvi_V_WB = Hi_PB_W;
      }
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcVelocityKinematicsCache(
    const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
    VelocityKinematicsCache<T>* vc) const {
  DRAKE_DEMAND(vc != nullptr);

  // If the model has zero dofs we simply set all spatial velocities to zero and
  // return since there is no work to be done.
  if (num_velocities() == 0) {
    vc->InitializeToZero();
    return;
  }

  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);

  const T* positions = get_positions(context).data();
  const T* velocities = get_velocities(context).data();

  // Performs a base-to-tip recursion computing body velocities.
  // Skip the World which is mobod_index(0).
  for (MobodIndex mobod_index(1); mobod_index < num_mobods(); ++mobod_index) {
    const BodyNode<T>& node = *body_nodes_[mobod_index];
    DRAKE_ASSERT(node.mobod_index() == mobod_index);

    // Update per-mobod kinematics.
    node.CalcVelocityKinematicsCache_BaseToTip(positions, pc, H_PB_W_cache,
                                               velocities, vc);
  }
}

// Result is indexed by MobodIndex, not BodyIndex.
template <typename T>
void MultibodyTree<T>::CalcSpatialInertiasInWorld(
    const systems::Context<T>& context,
    std::vector<SpatialInertia<T>>* M_B_W_all) const {
  DRAKE_THROW_UNLESS(M_B_W_all != nullptr);
  DRAKE_THROW_UNLESS(ssize(*M_B_W_all) == num_mobods());

  const FrameBodyPoseCache<T>& frame_body_pose_cache =
      EvalFrameBodyPoses(context);
  const PositionKinematicsCache<T>& pc = this->EvalPositionKinematics(context);

  // Skip the world.
  // TODO(joemasterjohn): Consider an optimization to avoid calculating spatial
  //  inertias for locked floating bodies.
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    const RigidTransform<T>& X_WB = pc.get_X_WB(body.mobod_index());

    // Orientation of B in W.
    const RotationMatrix<T>& R_WB = X_WB.rotation();

    // Spatial inertia of body B about Bo and expressed in the body frame B.
    const SpatialInertia<T>& M_BBo_B =
        frame_body_pose_cache.get_M_BBo_B(body.mobod_index());
    // Re-express body B's spatial inertia in the world frame W.
    SpatialInertia<T>& M_BBo_W = (*M_B_W_all)[body.mobod_index()];
    M_BBo_W = M_BBo_B;               // Wrong frame.
    M_BBo_W.ReExpressInPlace(R_WB);  // Fixed.
  }
}

template <typename T>
void MultibodyTree<T>::CalcReflectedInertia(
    const systems::Context<T>& context, VectorX<T>* reflected_inertia) const {
  DRAKE_THROW_UNLESS(reflected_inertia != nullptr);
  DRAKE_THROW_UNLESS(ssize(*reflected_inertia) == num_velocities());

  // See JointActuator::reflected_inertia().
  reflected_inertia->setZero();

  for (const JointActuator<T>* actuator : actuators_.elements()) {
    const int joint_velocity_index =
        actuator->joint().velocity_start();  // within v
    (*reflected_inertia)(joint_velocity_index) =
        actuator->calc_reflected_inertia(context);
  }
}

template <typename T>
void MultibodyTree<T>::CalcJointDamping(const systems::Context<T>& context,
                                        VectorX<T>* joint_damping) const {
  DRAKE_THROW_UNLESS(joint_damping != nullptr);
  DRAKE_THROW_UNLESS(ssize(*joint_damping) == num_velocities());

  for (const Joint<T>* joint : joints_.elements()) {
    joint_damping->segment(joint->velocity_start(), joint->num_velocities()) =
        joint->GetDampingVector(context);
  }
}

template <typename T>
void MultibodyTree<T>::CalcFrameBodyPoses(
    const systems::Context<T>& context,
    FrameBodyPoseCache<T>* frame_body_poses) const {
  DRAKE_DEMAND(frame_body_poses != nullptr);

  // All RigidBodyFrames share this entry which is set once and forever.
  DRAKE_ASSERT(frame_body_poses->get_X_BF(0).IsExactlyIdentity());
  DRAKE_ASSERT(frame_body_poses->get_X_FB(0).IsExactlyIdentity());

  // The first pass locates each frame with respect to the body frame B
  // of the body to which it is fixed.
  for (const Frame<T>* frame : frames_.elements()) {
    const int body_pose_index_in_cache = frame->get_body_pose_index_in_cache();
    if (frame->is_body_frame()) {
      DRAKE_DEMAND(body_pose_index_in_cache == 0);
      continue;
    }
    // TODO(sherm1) Note that we're unnecessarily recalculating the parent
    //  and ancestor poses. Likely OK since we expect short sequences and
    //  the whole computation is done only when parameters change. But if
    //  there is a performance issue, consider doing this in topological
    //  order (or memoizing) so we don't have to recalculate.
    frame_body_poses->SetX_BF(body_pose_index_in_cache,
                              frame->CalcPoseInBodyFrame(context));
  }

  // For every mobilized body, precalculate its body-frame spatial inertia
  // M_BBo_B from the parameterization of that inertia.
  for (const SpanningForest::Mobod& mobod : forest().mobods()) {
    if (mobod.is_world()) continue;
    // TODO(sherm1) Can't handle optimized WeldedLinksAssemblies yet.
    DRAKE_DEMAND(ssize(mobod.follower_link_ordinals()) == 1);
    const Mobilizer<T>& mobilizer = get_mobilizer(mobod.index());

    // Get the parameterized spatial inertia.
    const RigidBody<T>& body = mobilizer.outboard_body();
    const SpatialInertia<T> M_BBo_B =
        body.CalcSpatialInertiaInBodyFrame(context);
    frame_body_poses->SetM_BBo_B(mobod.index(), M_BBo_B);
  }
}

template <typename T>
void MultibodyTree<T>::CalcCompositeBodyInertiasInWorld(
    const systems::Context<T>& context,
    std::vector<SpatialInertia<T>>* K_BBo_W_all) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const std::vector<SpatialInertia<T>>& M_BBo_W_all =
      EvalSpatialInertiaInWorldCache(context);

  // Perform tip-to-base recursion for each composite body, skipping the world.
  for (MobodIndex mobod_index(num_mobods() - 1); mobod_index > 0;
       --mobod_index) {
    // Node corresponding to the base of composite body C. We'll add in
    // everything outboard of this node.
    const BodyNode<T>& composite_node = *body_nodes_[mobod_index];

    composite_node.CalcCompositeBodyInertiaInWorld_TipToBase(pc, M_BBo_W_all,
                                                             &*K_BBo_W_all);
  }
}

template <typename T>
void MultibodyTree<T>::CalcSpatialAccelerationBias(
    const systems::Context<T>& context,
    std::vector<SpatialAcceleration<T>>* Ab_WB_all) const {
  const FrameBodyPoseCache<T>& frame_body_pose_cache =
      EvalFrameBodyPoses(context);
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  const T* const positions = get_positions(context).data();
  const T* const velocities = get_velocities(context).data();

  // This skips the world mobilized body, mobod_index 0.
  // For world we opted for leaving Ab_WB initialized to NaN so that
  // an accidental usage (most likely indicating unnecessary math) in code would
  // immediately trigger a trail of NaNs that we can track to the source.
  // TODO(joemasterjohn): Consider an optimization where we avoid computing
  //  `Ab_WB` for locked floating bodies.
  (*Ab_WB_all)[world_mobod_index()].SetNaN();
  for (MobodIndex mobod_index(1); mobod_index < num_mobods(); ++mobod_index) {
    const BodyNode<T>& node = *body_nodes_[mobod_index];
    node.CalcSpatialAccelerationBias(frame_body_pose_cache, positions, pc,
                                     velocities, vc, &*Ab_WB_all);
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyForceBias(
    const systems::Context<T>& context,
    const ArticulatedBodyInertiaCache<T>& abic,
    std::vector<SpatialForce<T>>* Zb_Bo_W_all) const {
  DRAKE_THROW_UNLESS(Zb_Bo_W_all != nullptr);
  DRAKE_THROW_UNLESS(ssize(*Zb_Bo_W_all) == num_mobods());
  const std::vector<SpatialAcceleration<T>>& Ab_WB_cache =
      EvalSpatialAccelerationBiasCache(context);

  // This skips the world mobilized body, mobod_index 0.
  // For the world we opted for leaving Zb_Bo_W initialized to NaN so that
  // an accidental usage (most likely indicating unnecessary math) in code would
  // immediately trigger a trail of NaNs that we can track to the source.
  // TODO(joemasterjohn): Consider an optimization to avoid computing `Zb_Bo_W`
  //  for locked floating bodies.
  (*Zb_Bo_W_all)[world_mobod_index()].SetNaN();
  for (MobodIndex mobod_index(1); mobod_index < num_mobods(); ++mobod_index) {
    const ArticulatedBodyInertia<T>& Pplus_PB_W =
        abic.get_Pplus_PB_W(mobod_index);
    const SpatialAcceleration<T>& Ab_WB = Ab_WB_cache[mobod_index];
    SpatialForce<T>& Zb_Bo_W = (*Zb_Bo_W_all)[mobod_index];
    Zb_Bo_W = Pplus_PB_W * Ab_WB;
  }
}

// Result is indexed by MobodIndex.
template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyForceBias(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* Zb_Bo_W_all) const {
  DRAKE_THROW_UNLESS(Zb_Bo_W_all != nullptr);
  DRAKE_THROW_UNLESS(ssize(*Zb_Bo_W_all) == num_mobods());
  const ArticulatedBodyInertiaCache<T>& abic =
      EvalArticulatedBodyInertiaCache(context);
  CalcArticulatedBodyForceBias(context, abic, Zb_Bo_W_all);
}

// Result is indexed by MobodIndex.
template <typename T>
void MultibodyTree<T>::CalcDynamicBiasForces(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* Fb_Bo_W_all) const {
  DRAKE_THROW_UNLESS(Fb_Bo_W_all != nullptr);
  DRAKE_THROW_UNLESS(ssize(*Fb_Bo_W_all) == num_mobods());

  const std::vector<SpatialInertia<T>>& spatial_inertia_in_world_cache =
      EvalSpatialInertiaInWorldCache(context);

  const VelocityKinematicsCache<T>& vc = this->EvalVelocityKinematics(context);

  // Skip the world.
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);

    const SpatialInertia<T>& M_B_W =
        spatial_inertia_in_world_cache[body.mobod_index()];

    const T& mass = M_B_W.get_mass();
    // B's center of mass measured in B and expressed in W.
    const Vector3<T>& p_BoBcm_W = M_B_W.get_com();
    // B's unit rotational inertia about Bo, expressed in W.
    const UnitInertia<T>& G_B_W = M_B_W.get_unit_inertia();

    // Gyroscopic spatial force Fb_Bo_W(q, v) on body B about Bo, expressed in
    // W.
    const SpatialVelocity<T>& V_WB = vc.get_V_WB(body.mobod_index());
    const Vector3<T>& w_WB = V_WB.rotational();
    SpatialForce<T>& Fb_Bo_W = (*Fb_Bo_W_all)[body.mobod_index()];
    Fb_Bo_W = mass * SpatialForce<T>(
                         w_WB.cross(G_B_W * w_WB), /* rotational */
                         w_WB.cross(w_WB.cross(p_BoBcm_W)) /* translational */);
  }
}

template <typename T>
void MultibodyTree<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context, const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&, const VectorX<T>& known_vdot,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  const bool ignore_velocities = false;
  CalcSpatialAccelerationsFromVdot(context, known_vdot, ignore_velocities,
                                   &*A_WB_array);
}

template <typename T>
void MultibodyTree<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context, const VectorX<T>& known_vdot,
    bool ignore_velocities,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  DRAKE_DEMAND(A_WB_array != nullptr);
  DRAKE_DEMAND(ssize(*A_WB_array) == num_mobods());

  DRAKE_DEMAND(known_vdot.size() == forest().num_velocities());

  const auto& frame_body_pose_cache = EvalFrameBodyPoses(context);
  const auto& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>* vc =
      ignore_velocities ? nullptr : &EvalVelocityKinematics(context);

  // The world's spatial acceleration is always zero.
  A_WB_array->at(world_mobod_index()) = SpatialAcceleration<T>::Zero();

  const T* const positions = get_positions(context).data();
  const T* const velocities =
      ignore_velocities ? nullptr : get_velocities(context).data();
  const T* const accelerations = known_vdot.data();

  // Performs a base-to-tip recursion computing body accelerations. World was
  // handled above so is skipped here.
  for (MobodIndex mobod_index{1}; mobod_index < num_mobods(); ++mobod_index) {
    const BodyNode<T>& node = *body_nodes_[mobod_index];
    DRAKE_ASSERT(node.mobod_index() == mobod_index);

    // Update per-node kinematics.
    node.CalcSpatialAcceleration_BaseToTip(frame_body_pose_cache, positions, pc,
                                           velocities, vc, accelerations,
                                           &*A_WB_array);
  }
}

template <typename T>
void MultibodyTree<T>::CalcAccelerationKinematicsCache(
    const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc, const VectorX<T>& known_vdot,
    AccelerationKinematicsCache<T>* ac) const {
  DRAKE_DEMAND(ac != nullptr);
  DRAKE_DEMAND(known_vdot.size() == forest().num_velocities());

  std::vector<SpatialAcceleration<T>>& A_WB_array = ac->get_mutable_A_WB_pool();

  CalcSpatialAccelerationsFromVdot(context, pc, vc, known_vdot, &A_WB_array);
}

template <typename T>
VectorX<T> MultibodyTree<T>::CalcInverseDynamics(
    const systems::Context<T>& context, const VectorX<T>& known_vdot,
    const MultibodyForces<T>& external_forces) const {
  // Temporary storage used in the computation of inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W(num_bodies());
  VectorX<T> tau(num_velocities());
  CalcInverseDynamics(context, known_vdot, external_forces.body_forces(),
                      external_forces.generalized_forces(), &A_WB, &F_BMo_W,
                      &tau);
  return tau;
}

// All argument vectors are indexed by MobodIndex.
template <typename T>
void MultibodyTree<T>::CalcInverseDynamics(
    const systems::Context<T>& context, const VectorX<T>& known_vdot,
    const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
    const Eigen::Ref<const VectorX<T>>& tau_applied_array,
    std::vector<SpatialAcceleration<T>>* A_WB_array,
    std::vector<SpatialForce<T>>* F_BMo_W_array,
    EigenPtr<VectorX<T>> tau_array) const {
  const bool ignore_velocities = false;
  CalcInverseDynamics(context, known_vdot, Fapplied_Bo_W_array,
                      tau_applied_array, ignore_velocities, A_WB_array,
                      F_BMo_W_array, tau_array);
}

// All argument vectors are indexed by MobodIndex. Note that we permit (perhaps
// unwisely) the output arguments F_BMo_W_array and tau_array to use the same
// memory as the input arguments Fapplied_Bo_W_array & tau_applied_array.
// (There are internal usages that take advantage of that.)
template <typename T>
void MultibodyTree<T>::CalcInverseDynamics(
    const systems::Context<T>& context, const VectorX<T>& known_vdot,
    const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
    const Eigen::Ref<const VectorX<T>>& tau_applied_array,
    bool ignore_velocities, std::vector<SpatialAcceleration<T>>* A_WB_array,
    std::vector<SpatialForce<T>>* F_BMo_W_array,
    EigenPtr<VectorX<T>> tau_array) const {
  DRAKE_DEMAND(known_vdot.size() == num_velocities());
  DRAKE_DEMAND(ssize(Fapplied_Bo_W_array) == 0 ||
               ssize(Fapplied_Bo_W_array) == num_mobods());
  DRAKE_DEMAND(ssize(tau_applied_array) == 0 ||
               ssize(tau_applied_array) == num_velocities());
  DRAKE_DEMAND(A_WB_array != nullptr && ssize(*A_WB_array) == num_mobods());
  DRAKE_DEMAND(F_BMo_W_array != nullptr &&
               ssize(*F_BMo_W_array) == num_mobods());
  DRAKE_DEMAND(tau_array != nullptr && ssize(*tau_array) == num_velocities());

  // Compute body spatial accelerations given the generalized accelerations are
  // known.
  CalcSpatialAccelerationsFromVdot(context, known_vdot, ignore_velocities,
                                   &*A_WB_array);

  const FrameBodyPoseCache<T>& frame_body_pose_cache =
      EvalFrameBodyPoses(context);
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  // Eval M_Bo_W(q).
  const std::vector<SpatialInertia<T>>& spatial_inertia_in_world_cache =
      EvalSpatialInertiaInWorldCache(context);

  // Eval Fb_Bo_W(q, v). Fb_Bo_W = 0 if v = 0.
  const std::vector<SpatialForce<T>>* dynamic_bias_cache =
      ignore_velocities ? nullptr : &EvalDynamicBiasCache(context);

  const T* const positions = get_positions(context).data();

  // Performs a tip-to-base recursion computing the total spatial force F_BMo_W
  // acting on body B, about point Mo, expressed in the world frame W.
  // This includes the world (depth = 0) so that
  // F_BMo_W_array[world_mobod_index()] contains the total force of the bodies
  // connected to the world by a mobilizer.
  for (int level = forest_height() - 1; level >= 0; --level) {
    for (MobodIndex mobod_index : body_node_levels_[level]) {
      const BodyNode<T>& node = *body_nodes_[mobod_index];

      DRAKE_ASSERT(node.mobod().level() == level);
      DRAKE_ASSERT(node.mobod_index() == mobod_index);

      // Compute F_BMo_W for the body associated with this node and project it
      // onto the space of generalized forces tau for the associated mobilizer.
      node.CalcInverseDynamics_TipToBase(
          frame_body_pose_cache, positions, pc, spatial_inertia_in_world_cache,
          dynamic_bias_cache,  // null if ignoring velocities
          *A_WB_array,
          Fapplied_Bo_W_array,        // null if no applied spatial forces
          tau_applied_array,          // null if no applied generalized forces
          F_BMo_W_array, tau_array);  // outputs
    }
  }

  // Add the effect of reflected inertias.
  // See JointActuator::reflected_inertia().
  const VectorX<T>& reflected_inertia = EvalReflectedInertiaCache(context);
  for (int i = 0; i < num_velocities(); ++i) {
    (*tau_array)(i) += reflected_inertia(i) * known_vdot(i);
  }
}

template <typename T>
void MultibodyTree<T>::CalcForceElementsContribution(
    const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc, MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(*this));

  forces->SetZero();
  // Add contributions from force elements.
  for (const auto& force_element : force_elements_) {
    force_element->CalcAndAddForceContribution(context, pc, vc, forces);
  }

  // TODO(amcastro-tri): Remove this call once damping is implemented in terms
  //  of force elements.
  AddJointDampingForces(context, forces);
}

template <typename T>
void MultibodyTree<T>::AddJointDampingForces(const systems::Context<T>& context,
                                             MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  for (const Joint<T>* joint : joints_.elements()) {
    joint->AddInDamping(context, forces);
  }
}

template <typename T>
bool MultibodyTree<T>::IsVelocityEqualToQDot() const {
  if (num_positions() != num_velocities()) {
    return false;
  }
  for (const auto& mobilizer : mobilizers_) {
    if (!mobilizer->is_velocity_equal_to_qdot()) {
      return false;
    }
  }
  return true;
}

template <typename T>
void MultibodyTree<T>::MapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot, EigenPtr<VectorX<T>> v) const {
  DRAKE_DEMAND(qdot.size() == num_positions());
  DRAKE_DEMAND(v != nullptr);
  DRAKE_DEMAND(v->size() == num_velocities());

  VectorUpTo6<T> v_mobilizer;
  for (const auto& mobilizer : mobilizers_) {
    const auto qdot_mobilizer = mobilizer->get_positions_from_array(qdot);
    v_mobilizer.resize(mobilizer->num_velocities());
    mobilizer->MapQDotToVelocity(context, qdot_mobilizer, &v_mobilizer);
    mobilizer->get_mutable_velocities_from_array(v) = v_mobilizer;
  }
}

template <typename T>
void MultibodyTree<T>::MapVelocityToQDot(const systems::Context<T>& context,
                                         const Eigen::Ref<const VectorX<T>>& v,
                                         EigenPtr<VectorX<T>> qdot) const {
  DRAKE_DEMAND(v.size() == num_velocities());
  DRAKE_DEMAND(qdot != nullptr);
  DRAKE_DEMAND(qdot->size() == num_positions());

  const int kMaxQdot = 7;
  // qdot_mobilizer is a dynamic sized vector of max size equal to seven.
  Eigen::Matrix<T, Eigen::Dynamic, 1, 0, kMaxQdot, 1> qdot_mobilizer;
  for (const auto& mobilizer : mobilizers_) {
    const auto v_mobilizer = mobilizer->get_velocities_from_array(v);
    DRAKE_DEMAND(mobilizer->num_positions() <= kMaxQdot);
    qdot_mobilizer.resize(mobilizer->num_positions());
    mobilizer->MapVelocityToQDot(context, v_mobilizer, &qdot_mobilizer);
    mobilizer->get_mutable_positions_from_array(qdot) = qdot_mobilizer;
  }
}

template <typename T>
Eigen::SparseMatrix<T> MultibodyTree<T>::MakeVelocityToQDotMap(
    const systems::Context<T>& context) const {
  Eigen::SparseMatrix<T> N(num_positions(), num_velocities());
  if (IsVelocityEqualToQDot()) {
    N.setIdentity();
    return N;
  }

  // TODO(russt): Consider updating Mobilizer::CalcNMatrix to populate the
  //  SparseMatrix directly. But SparseMatrix does not support block writing
  //  operations, so we will likely need to pass the entire matrix, and each
  //  mobilizer will need to populate according to position_start_in_q() and
  //  velocity_start_in_v().
  std::vector<Eigen::Triplet<T>> triplet_list;
  // Note: We don't reserve storage for the triplet_list, because we don't have
  // a useful estimate of the size in general.
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0, 7, 6> N_mobilizer;
  for (const auto& mobilizer : mobilizers_) {
    N_mobilizer.resize(mobilizer->num_positions(), mobilizer->num_velocities());
    mobilizer->CalcNMatrix(context, &N_mobilizer);
    for (int i = 0; i < mobilizer->num_positions(); ++i) {
      for (int j = 0; j < mobilizer->num_velocities(); ++j) {
        if (N_mobilizer(i, j) != 0) {
          triplet_list.push_back(Eigen::Triplet<T>(
              mobilizer->position_start_in_q() + i,
              mobilizer->velocity_start_in_v() + j, N_mobilizer(i, j)));
        }
      }
    }
  }
  N.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return N;
}

template <typename T>
Eigen::SparseMatrix<T> MultibodyTree<T>::MakeQDotToVelocityMap(
    const systems::Context<T>& context) const {
  Eigen::SparseMatrix<T> Nplus(num_velocities(), num_positions());
  if (IsVelocityEqualToQDot()) {
    Nplus.setIdentity();
    return Nplus;
  }

  std::vector<Eigen::Triplet<T>> triplet_list;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 7> Nplus_mobilizer;
  for (const auto& mobilizer : mobilizers_) {
    Nplus_mobilizer.resize(mobilizer->num_velocities(),
                           mobilizer->num_positions());
    mobilizer->CalcNplusMatrix(context, &Nplus_mobilizer);
    for (int i = 0; i < mobilizer->num_velocities(); ++i) {
      for (int j = 0; j < mobilizer->num_positions(); ++j) {
        if (Nplus_mobilizer(i, j) != 0) {
          triplet_list.push_back(Eigen::Triplet<T>(
              mobilizer->velocity_start_in_v() + i,
              mobilizer->position_start_in_q() + j, Nplus_mobilizer(i, j)));
        }
      }
    }
  }
  Nplus.setFromTriplets(triplet_list.begin(), triplet_list.end());
  return Nplus;
}

template <typename T>
void MultibodyTree<T>::CalcMassMatrixViaInverseDynamics(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> M) const {
  DRAKE_DEMAND(M != nullptr);
  DRAKE_DEMAND(M->rows() == num_velocities());
  DRAKE_DEMAND(M->cols() == num_velocities());

  // Compute one column of the mass matrix via inverse dynamics at a time.
  const int nv = num_velocities();
  VectorX<T> vdot(nv);
  VectorX<T> tau(nv);
  // Auxiliary arrays used by inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_mobods());
  std::vector<SpatialForce<T>> F_BMo_W_array(num_mobods());

  // The mass matrix is only a function of configuration q. Therefore velocity
  // terms are not considered.
  const bool ignore_velocities = true;
  for (int j = 0; j < nv; ++j) {
    // N.B. VectorX<T>::Unit() does not perform any heap allocation but rather
    // returns a functor-like object that fills the entries in vdot.
    vdot = VectorX<T>::Unit(nv, j);
    tau.setZero();
    CalcInverseDynamics(context, vdot, {}, VectorX<T>(), ignore_velocities,
                        &A_WB_array, &F_BMo_W_array, &tau);
    M->col(j) = tau;
  }
}

// TODO(sherm1) This needs to be reworked so the node-specific code is
//  processed by the templatized class.
template <typename T>
void MultibodyTree<T>::CalcMassMatrix(const systems::Context<T>& context,
                                      EigenPtr<MatrixX<T>> M) const {
  DRAKE_DEMAND(M != nullptr);
  DRAKE_DEMAND(M->rows() == num_velocities());
  DRAKE_DEMAND(M->cols() == num_velocities());

  // This method implements algorithm 9.3 in [Jain 2010]. We use slightly
  // different notation conventions:
  // - Rigid shift operators A and Φ are implemented in SpatialInertia::Shift()
  //   and SpatialForce::Shift(), respectively.
  // - We use the symbol F instead of X, to highlight the physical
  //   interpretation of the algorithm in terms of composite bodies.
  // - Even though we use H for the "hinge matrix" as in Jain's book, our hinge
  //   matrix is the transpose of that used by Jain. Therefore our hinge matrix
  //   is the across-mobilizer Jacobian such that we can write
  //   V_PB_W = H_PB_W * v_B, with v_B the generalized velocities of body B's
  //   mobilizer.
  // - In code we use the monogram notation K_BBo_W to denote the spatial
  //   inertia of composite body B, about it's frame origin Bo, and expressed in
  //   the world frame W.
  //
  // - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
  //               algorithms. Springer Science & Business Media, pp. 123-130.

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const std::vector<SpatialInertia<T>>& K_BBo_W_cache =
      EvalCompositeBodyInertiaInWorldCache(context);
  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);
  const VectorX<T>& reflected_inertia = EvalReflectedInertiaCache(context);

  // The algorithm below does not recurse zero entries and therefore these must
  // be set a priori.
  M->setZero();

  // Perform tip-to-base recursion for each composite body, skipping the world.
  for (MobodIndex mobod_index(num_mobods() - 1); mobod_index > 0;
       --mobod_index) {
    // Node corresponding to the composite body C.
    const BodyNode<T>& composite_node = *body_nodes_[mobod_index];

    composite_node.CalcMassMatrixContributionViaWorld_TipToBase(
        pc, K_BBo_W_cache, H_PB_W_cache, M);
  }

  // Account for reflected inertia.
  M->diagonal() += reflected_inertia;
}

template <typename T>
void MultibodyTree<T>::CalcBiasTerm(const systems::Context<T>& context,
                                    EigenPtr<VectorX<T>> Cv) const {
  DRAKE_DEMAND(Cv != nullptr);
  DRAKE_DEMAND(Cv->rows() == num_velocities());
  DRAKE_DEMAND(Cv->cols() == 1);
  const int nv = num_velocities();
  const VectorX<T> vdot = VectorX<T>::Zero(nv);
  // Auxiliary arrays used by inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_mobods());
  std::vector<SpatialForce<T>> F_BMo_W_array(num_mobods());
  // TODO(amcastro-tri): provide specific API for when vdot = 0.
  CalcInverseDynamics(context, vdot, {}, VectorX<T>(), &A_WB_array,
                      &F_BMo_W_array, Cv);
}

template <typename T>
VectorX<T> MultibodyTree<T>::CalcGravityGeneralizedForces(
    const systems::Context<T>& context) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  if (gravity_field_) {
    return gravity_field_->CalcGravityGeneralizedForces(context);
  }
  return VectorX<T>::Zero(num_velocities());
}

template <typename T>
RigidTransform<T> MultibodyTree<T>::CalcRelativeTransform(
    const systems::Context<T>& context, const Frame<T>& frame_F,
    const Frame<T>& frame_G) const {
  // Shortcut: Efficiently return identity transform if frame_F == frame_G.
  if (frame_F.index() == frame_G.index()) return RigidTransform<T>::Identity();

  const RigidBody<T>& A = frame_F.body();
  const RigidBody<T>& B = frame_G.body();

  // Find each Frame's pose on its own body (F on A, G on B).
  const FrameBodyPoseCache<T>& frame_body_pose_cache =
      EvalFrameBodyPoses(context);
  const math::RigidTransform<T>& X_AF =
      frame_F.get_X_BF(frame_body_pose_cache);  // B==A
  const math::RigidTransform<T>& X_BG =
      frame_G.get_X_BF(frame_body_pose_cache);  // F==G

  // Find each body's pose in World.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const RigidTransform<T>& X_WA = pc.get_X_WB(A.mobod_index());  // B==A
  const RigidTransform<T>& X_WB = pc.get_X_WB(B.mobod_index());

  const RigidTransform<T> X_WF = X_WA * X_AF;
  const RigidTransform<T> X_WG = X_WB * X_BG;
  return X_WF.InvertAndCompose(X_WG);  // X_FG = X_FW * X_WG;
}

template <typename T>
RotationMatrix<T> MultibodyTree<T>::CalcRelativeRotationMatrix(
    const systems::Context<T>& context, const Frame<T>& frame_F,
    const Frame<T>& frame_G) const {
  // Shortcut: Efficiently return identity matrix if frame_F == frame_G.
  if (frame_F.index() == frame_G.index()) return RotationMatrix<T>::Identity();

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const RigidBody<T>& A = frame_F.body();
  const RigidBody<T>& B = frame_G.body();
  const RotationMatrix<T>& R_WA = pc.get_R_WB(A.mobod_index());
  const RotationMatrix<T>& R_WB = pc.get_R_WB(B.mobod_index());
  const RotationMatrix<T> R_AF = frame_F.CalcRotationMatrixInBodyFrame(context);
  const RotationMatrix<T> R_BG = frame_G.CalcRotationMatrixInBodyFrame(context);
  const RotationMatrix<T> R_WF = R_WA * R_AF;
  const RotationMatrix<T> R_WG = R_WB * R_BG;
  return R_WF.InvertAndCompose(R_WG);  // R_FG = R_FW * R_WG;
}

template <typename T>
void MultibodyTree<T>::CalcPointsPositions(
    const systems::Context<T>& context, const Frame<T>& frame_B,
    const Eigen::Ref<const MatrixX<T>>& p_BQi, const Frame<T>& frame_A,
    EigenPtr<MatrixX<T>> p_AQi) const {
  DRAKE_THROW_UNLESS(p_BQi.rows() == 3);
  DRAKE_THROW_UNLESS(p_AQi != nullptr);
  DRAKE_THROW_UNLESS(p_AQi->rows() == 3);
  DRAKE_THROW_UNLESS(p_AQi->cols() == p_BQi.cols());
  const RigidTransform<T> X_AB =
      CalcRelativeTransform(context, frame_A, frame_B);
  p_AQi->template topRows<3>() = X_AB * p_BQi.template topRows<3>();
}

template <typename T>
void MultibodyTree<T>::CalcPointsVelocities(
    const systems::Context<T>& context, const Frame<T>& frame_B,
    const Eigen::Ref<const MatrixX<T>>& p_BoQi_B, const Frame<T>& frame_A,
    const Frame<T>& frame_E, EigenPtr<MatrixX<T>> v_AQi_E) const {
  DRAKE_THROW_UNLESS(p_BoQi_B.rows() == 3);
  DRAKE_THROW_UNLESS(v_AQi_E != nullptr);
  DRAKE_THROW_UNLESS(v_AQi_E->rows() == 3);
  DRAKE_THROW_UNLESS(v_AQi_E->cols() == p_BoQi_B.cols());

  // The calculation used below is: v_AQi_E = v_ABo_E + ω_AB_E x p_BoQi_E.
  // The R_EB rotation matrix is used to express p_BoQi_B in terms of frame E.
  const SpatialVelocity<T> V_ABo_E =
      frame_B.CalcSpatialVelocity(context, frame_A, frame_E);
  const RotationMatrix<T> R_EB =
      CalcRelativeRotationMatrix(context, frame_E, frame_B);
  for (int col = 0; col < p_BoQi_B.cols(); ++col) {
    const Vector3<T> p_BoQi_E = R_EB * p_BoQi_B.col(col);
    v_AQi_E->col(col) = V_ABo_E.Shift(p_BoQi_E).translational();
  }

  // TODO(Mitiguy) A few possible optimizations are below. Implement as useful.
  // If ω_AB_E is zero, v_AQi_E = v_ABo_E + (ω_AB_E = 0) x p_BoQi_E = v_ABo_E.
  // If R_EB = [I₃₃], form v_AQi_E = v_ABo_E + ω_AB_E x p_BoQi_B
  // rather than as        v_AQi_E = v_ABo_E + ω_AB_E x p_BoQi_E.
}

template <typename T>
T MultibodyTree<T>::CalcTotalMass(const systems::Context<T>& context) const {
  T total_mass = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    const T& body_mass = body.get_mass(context);
    total_mass += body_mass;
  }
  return total_mass;
}

template <typename T>
T MultibodyTree<T>::CalcTotalMass(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances) const {
  T total_mass = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      const T& body_mass = body.get_mass(context);
      total_mass += body_mass;
    }
  }
  return total_mass;
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassPositionInWorld(
    const systems::Context<T>& context) const {
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Vector3<T> sum_mi_pi = Vector3<T>::Zero();

  // Sum over all the bodies except the 0th body (which is the world body).
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);

    // total_mass = ∑ mᵢ.
    const T& body_mass = body.get_mass(context);
    total_mass += body_mass;

    // sum_mi_pi = ∑ mᵢ * pi_WoBcm_W.
    const Vector3<T> pi_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
    const Vector3<T> pi_WoBcm_W = body.EvalPoseInWorld(context) * pi_BoBcm_B;
    sum_mi_pi += body_mass * pi_WoBcm_W;
  }

  if (total_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  return sum_mi_pi / total_mass;
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassPositionInWorld(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances) const {
  // Reminder: MultibodyTree always declares a world body and 2 model instances
  // "world" and "default" so num_model_instances() should always be >= 2.
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Vector3<T> sum_mi_pi = Vector3<T>::Zero();

  // Sum over all the bodies that are in model_instances except for the 0th body
  // (which is the world body), and count each body's contribution only once.
  // Reminder: Although it is not possible for a body to belong to multiple
  // model instances [as RigidBody::model_instance() returns a body's unique
  // model instance], it is possible for the same model instance to be added
  // multiple times to std::vector<ModelInstanceIndex>& model_instances). The
  // code below ensures a body's contribution to the sum occurs only once.
  // Duplicate model_instances in std::vector are ignored.
  int number_of_non_world_bodies_processed = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      const T& body_mass = body.get_mass(context);
      total_mass += body_mass;  // total_mass = ∑ mᵢ.
      ++number_of_non_world_bodies_processed;

      // sum_mi_pi = ∑ mᵢ * pi_WoBcm_W.
      const Vector3<T> pi_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
      const Vector3<T> pi_WoBcm_W = body.EvalPoseInWorld(context) * pi_BoBcm_B;
      sum_mi_pi += body_mass * pi_WoBcm_W;
    }
  }

  // Throw an exception if there are zero non-world bodies in model_instances.
  if (number_of_non_world_bodies_processed == 0) {
    std::string message = fmt::format(
        "{}(): There must be at least one "
        "non-world body contained in model_instances.",
        __func__);
    throw std::logic_error(message);
  }

  if (total_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  return sum_mi_pi / total_mass;
}

template <typename T>
SpatialInertia<T> MultibodyTree<T>::CalcSpatialInertia(
    const systems::Context<T>& context, const Frame<T>& frame_F,
    const std::vector<BodyIndex>& body_indexes) const {
  // Check if there are repeated BodyIndex in body_indexes by converting the
  // vector to a set (to eliminate duplicates) and see if their sizes differ.
  const std::set<BodyIndex> without_duplicate_bodies(body_indexes.begin(),
                                                     body_indexes.end());
  if (body_indexes.size() != without_duplicate_bodies.size()) {
    throw std::logic_error(
        "CalcSpatialInertia(): contains a repeated BodyIndex.");
  }

  // For the set S of bodies contained in body_indexes, return S's
  // spatial inertia about Fo (frame_F's origin), expressed in frame F.
  // For efficiency, evaluate all bodies' spatial inertia and pose.
  const std::vector<SpatialInertia<T>>& M_Bi_W =
      EvalSpatialInertiaInWorldCache(context);
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  // Add each body's spatial inertia in the world frame W to this system
  // S's spatial inertia in W about Wo (the origin of W), expressed in W.
  // TODO(Mitiguy) Create SpatialInertia<T>::Zero() and use it below.
  SpatialInertia<T> M_SWo_W(0., Vector3<T>::Zero(),
                            UnitInertia<T>::TriaxiallySymmetric(0));

  for (BodyIndex body_index : body_indexes) {
    if (body_index == 0) continue;  // No contribution from the world body.

    // Ensure MultibodyPlant method contains a valid body_index.
    if (body_index >= num_bodies()) {
      throw std::logic_error(
          "CalcSpatialInertia(): contains an invalid BodyIndex.");
    }

    // Get the current body B's spatial inertia about Bo (body B's origin),
    // expressed in the world frame W.
    const MobodIndex mobod_index = get_body(body_index).mobod_index();
    const SpatialInertia<T>& M_BBo_W = M_Bi_W[mobod_index];

    // Shift M_BBo_W from about-point Bo to about-point Wo and add to the sum.
    const RigidTransform<T>& X_WB = pc.get_X_WB(mobod_index);
    const Vector3<T>& p_WoBo_W = X_WB.translation();
    M_SWo_W += M_BBo_W.Shift(-p_WoBo_W);  // Shift from Bo to Wo by p_BoWo_W.
  }

  // If frame_F is the world frame W, return now.
  if (frame_F.is_world_frame()) return M_SWo_W;

  // Otherwise, shift from Wo (world origin) to Fo (frame_F's origin).
  const RigidTransform<T> X_WF = frame_F.CalcPoseInWorld(context);
  const Vector3<T>& p_WoFo_W = X_WF.translation();
  const SpatialInertia<T> M_SFo_W = M_SWo_W.Shift(p_WoFo_W);

  // Re-express spatial inertia from frame W to frame F.
  const RotationMatrix<T> R_FW = (X_WF.rotation()).inverse();
  return M_SFo_W.ReExpress(R_FW);  // Returns M_SFo_F.
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassTranslationalVelocityInWorld(
    const systems::Context<T>& context) const {
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Vector3<T> sum_mi_vi = Vector3<T>::Zero();

  // Sum over all the bodies except the 0th body (which is the world body).
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);

    // total_mass = ∑ mᵢ.
    const T& body_mass = body.get_mass(context);
    total_mass += body_mass;

    // sum_mi_vi = ∑ mᵢ * vi_WBcm_W.
    const Vector3<T> vi_WBcm_W =
        body.CalcCenterOfMassTranslationalVelocityInWorld(context);
    sum_mi_vi += body_mass * vi_WBcm_W;
  }

  if (total_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  // For a system S with center of mass Scm, Scm's translational velocity in the
  // world frame W is calculated as v_WScm_W = ∑ (mᵢ vᵢ)  / mₛ, where mₛ = ∑ mᵢ,
  // mᵢ is the mass of the  iᵗʰ body, and vᵢ is Bᵢcm's velocity in world frame W
  // (Bᵢcm is the center of mass of the iᵗʰ body).
  return sum_mi_vi / total_mass;
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassTranslationalVelocityInWorld(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances) const {
  // Reminder: MultibodyTree always declares a world body and 2 model instances
  // "world" and "default" so num_model_instances() should always be >= 2.
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Vector3<T> sum_mi_vi = Vector3<T>::Zero();

  // Sum over all the bodies that are in model_instances except for the 0th body
  // (which is the world body), and count each body's contribution only once.
  // Reminder: Although it is not possible for a body to belong to multiple
  // model instances [as RigidBody::model_instance() returns a body's unique
  // model instance], it is possible for the same model instance to be added
  // multiple times to std::vector<ModelInstanceIndex>& model_instances). The
  // code below ensures a body's contribution to the sum occurs only once.
  // Duplicate model_instances in std::vector are ignored.
  int number_of_non_world_bodies_processed = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      const T& body_mass = body.get_mass(context);
      total_mass += body_mass;  // total_mass = ∑ mᵢ.
      ++number_of_non_world_bodies_processed;

      // sum_mi_vi = ∑ mᵢ * vi_WBcm_W.
      const Vector3<T> vi_WBcm_W =
          body.CalcCenterOfMassTranslationalVelocityInWorld(context);
      sum_mi_vi += body_mass * vi_WBcm_W;
    }
  }

  // Throw an exception if there are zero non-world bodies in model_instances.
  if (number_of_non_world_bodies_processed == 0) {
    std::string message = fmt::format(
        "{}(): There must be at least one "
        "non-world body contained in model_instances.",
        __func__);
    throw std::logic_error(message);
  }

  if (total_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  return sum_mi_vi / total_mass;
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassTranslationalAccelerationInWorld(
    const systems::Context<T>& context) const {
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  // To ensure a sensible exception message is issued if the system's mass is
  // zero or negative, check if ∑ mᵢ ≤ 0 _before_ calculating ∑ mᵢ * aᵢ.
  // Why? Acceleration calculations may require a dynamic analysis that will
  // issue a significantly less helpful exception message.
  // Sum over all the bodies except the 0th body (which is the world body).
  T total_mass = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    const T& body_mass = body.get_mass(context);
    total_mass += body_mass;  // total_mass = ∑ mᵢ.
  }

  if (total_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  // Sum over all the bodies except the 0th body (which is the world body).
  Vector3<T> sum_mi_ai = Vector3<T>::Zero();
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    const T& body_mass = body.get_mass(context);
    const Vector3<T> ai_WBcm_W =
        body.CalcCenterOfMassTranslationalAccelerationInWorld(context);
    sum_mi_ai += body_mass * ai_WBcm_W;  // sum_mi_ai = ∑ mᵢ * ai_WBcm_W.
  }

  // For a system S with center of mass Scm, Scm's translational acceleration in
  // the world W is calculated as a_WScm_W = ∑ (mᵢ aᵢ) / mₛ, where mₛ = ∑ mᵢ,
  // mᵢ is the mass of the  iᵗʰ body, and aᵢ is Bᵢcm's acceleration in world W
  // (Bᵢcm is the center of mass of the iᵗʰ body).
  return sum_mi_ai / total_mass;
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassTranslationalAccelerationInWorld(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances) const {
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  // To ensure a sensible exception message is issued if the system's mass is
  // zero or negative, check if ∑ mᵢ ≤ 0 _before_ calculating ∑ mᵢ * aᵢ.
  // Why? Acceleration calculations may require a dynamic analysis that will
  // issue a significantly less helpful exception message.
  // Sum over all the bodies in model_instances except the 0th body (which is
  // the world body). Each body is counted only once even if its model instance
  // is listed multiple times.
  T total_mass = 0;
  int number_of_non_world_bodies_processed = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      const T& body_mass = body.get_mass(context);
      total_mass += body_mass;  // total_mass = ∑ mᵢ.
      ++number_of_non_world_bodies_processed;
    }
  }

  // Throw an exception if there are zero non-world bodies in model_instances.
  if (number_of_non_world_bodies_processed == 0) {
    std::string message = fmt::format(
        "{}(): There must be at least one "
        "non-world body contained in model_instances.",
        __func__);
    throw std::logic_error(message);
  }

  if (total_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  // Sum over all the bodies that are in model_instances except for the 0th body
  // (which is the world body), and count each body's contribution only once.
  // Reminder: Although it is not possible for a body to belong to multiple
  // model instances [as RigidBody::model_instance() returns a body's unique
  // model instance], it is possible for the same model instance to be added
  // multiple times to std::vector<ModelInstanceIndex>& model_instances).
  // The code below ensures a body's contribution to the sum occurs only once.
  // Duplicate model_instances in std::vector are ignored.
  Vector3<T> sum_mi_ai = Vector3<T>::Zero();
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      const T& body_mass = body.get_mass(context);
      const Vector3<T> ai_WBcm_W =
          body.CalcCenterOfMassTranslationalAccelerationInWorld(context);
      sum_mi_ai += body_mass * ai_WBcm_W;  // sum_mi_ai = ∑ mᵢ * ai_WBcm_W.
    }
  }

  return sum_mi_ai / total_mass;
}

template <typename T>
SpatialMomentum<T> MultibodyTree<T>::CalcSpatialMomentumInWorldAboutPoint(
    const systems::Context<T>& context, const Vector3<T>& p_WoP_W) const {
  // Assemble a list of ModelInstanceIndex.
  // Skip model_instance_index(0) which always contains the "world" body -- the
  // spatial momentum of the world body measured in the world is always zero.
  std::vector<ModelInstanceIndex> model_instances;
  for (ModelInstanceIndex model_instance_index(1);
       model_instance_index < num_model_instances(); ++model_instance_index)
    model_instances.push_back(model_instance_index);

  return CalcSpatialMomentumInWorldAboutPoint(context, model_instances,
                                              p_WoP_W);
}

template <typename T>
SpatialMomentum<T> MultibodyTree<T>::CalcSpatialMomentumInWorldAboutPoint(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances,
    const Vector3<T>& p_WoP_W) const {
  // Assemble a list of BodyIndex.
  std::vector<BodyIndex> body_indexes;
  for (auto model_instance : model_instances) {
    // If invalid model_instance, throw an exception with a helpful message.
    if (!model_instances_.has_element(model_instance)) {
      throw std::logic_error(
          "CalcSpatialMomentumInWorldAboutPoint(): This MultibodyPlant method"
          " contains an invalid model_instance.");
    }

    const std::vector<BodyIndex> body_index_in_instance =
        GetBodyIndices(model_instance);
    for (BodyIndex body_index : body_index_in_instance)
      body_indexes.push_back(body_index);
  }

  // Form spatial momentum about Wo (origin of world frame W), expressed in W.
  const SpatialMomentum<T> L_WS_W =
      CalcBodiesSpatialMomentumInWorldAboutWo(context, body_indexes);

  // Shift the spatial momentum from Wo to point P.
  return L_WS_W.Shift(p_WoP_W);
}

template <typename T>
SpatialMomentum<T> MultibodyTree<T>::CalcBodiesSpatialMomentumInWorldAboutWo(
    const systems::Context<T>& context,
    const std::vector<BodyIndex>& body_indexes) const {
  // For efficiency, evaluate all bodies' spatial inertia, velocities, and pose.
  const std::vector<SpatialInertia<T>>& M_Bi_W =
      EvalSpatialInertiaInWorldCache(context);
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Accumulate each body's spatial momentum in the world frame W to this system
  // S's spatial momentum in W about Wo (the origin of W), expressed in W.
  SpatialMomentum<T> L_WS_W = SpatialMomentum<T>::Zero();

  // Add contributions from each body Bi.
  for (BodyIndex body_index : body_indexes) {
    if (body_index == 0) continue;  // No contribution from the world body.

    // Ensure MultibodyPlant method contains a valid body_index.
    DRAKE_DEMAND(body_index < num_bodies());

    // Form the current body's spatial momentum in W about Bo, expressed in W.
    const MobodIndex mobod_index = get_body(body_index).mobod_index();
    const SpatialInertia<T>& M_BBo_W = M_Bi_W[mobod_index];
    const SpatialVelocity<T>& V_WBo_W = vc.get_V_WB(mobod_index);
    SpatialMomentum<T> L_WBo_W = M_BBo_W * V_WBo_W;

    // Shift L_WBo_W from about Bo to about Wo and accumulate the sum.
    const RigidTransform<T>& X_WB = pc.get_X_WB(mobod_index);
    const Vector3<T>& p_WoBo_W = X_WB.translation();
    // After ShiftInPlace, L_WBo_W is changed to L_WBWo_W, which is B's
    // spatial momentum about point Wo, measured and expressed in frame W.
    L_WBo_W.ShiftInPlace(-p_WoBo_W);  // After this, L_WBo_W is now L_WBWo_W.
    L_WS_W += L_WBo_W;                // Actually is `L_WS_W += L_WBWo_W`.
  }

  return L_WS_W;
}

template <typename T>
const RigidTransform<T>& MultibodyTree<T>::EvalBodyPoseInWorld(
    const systems::Context<T>& context, const RigidBody<T>& body_B) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  body_B.HasThisParentTreeOrThrow(this);
  return EvalPositionKinematics(context).get_X_WB(body_B.mobod_index());
}

template <typename T>
const SpatialVelocity<T>& MultibodyTree<T>::EvalBodySpatialVelocityInWorld(
    const systems::Context<T>& context, const RigidBody<T>& body_B) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  body_B.HasThisParentTreeOrThrow(this);
  return EvalVelocityKinematics(context).get_V_WB(body_B.mobod_index());
}

template <typename T>
const SpatialAcceleration<T>&
MultibodyTree<T>::EvalBodySpatialAccelerationInWorld(
    const systems::Context<T>& context, const RigidBody<T>& body_B) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  body_B.HasThisParentTreeOrThrow(this);
  return EvalAccelerationKinematics(context).get_A_WB(body_B.mobod_index());
}

template <typename T>
void MultibodyTree<T>::CalcAcrossNodeJacobianWrtVExpressedInWorld(
    const systems::Context<T>& context, const PositionKinematicsCache<T>& pc,
    std::vector<Vector6<T>>* H_PB_W_cache) const {
  DRAKE_DEMAND(H_PB_W_cache != nullptr);
  DRAKE_DEMAND(static_cast<int>(H_PB_W_cache->size()) == num_velocities());

  // Quick return on nv = 0. Nothing to compute.
  if (num_velocities() == 0) return;

  const T* positions = get_positions(context).data();

  const FrameBodyPoseCache<T>& frame_body_pose_cache =
      EvalFrameBodyPoses(context);

  // TODO(joemasterjohn): Consider an optimization where we avoid computing
  //  `H_PB_W` for locked floating bodies.
  for (MobodIndex mobod_index(1); mobod_index < num_mobods(); ++mobod_index) {
    const BodyNode<T>& node = *body_nodes_[mobod_index];

    node.CalcAcrossNodeJacobianWrtVExpressedInWorld(
        frame_body_pose_cache, positions, pc, H_PB_W_cache);
  }
}

template <typename T>
void MultibodyTree<T>::CalcAllBodyBiasSpatialAccelerationsInWorld(
    const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
    std::vector<SpatialAcceleration<T>>* AsBias_WB_all) const {
  // TODO(mitiguy) Allow with_respect_to be JacobianWrtVariable::kQDot.
  // TODO(mitiguy) Per issue #13560, cache bias acceleration computation.
  DRAKE_THROW_UNLESS(with_respect_to == JacobianWrtVariable::kV);

  // Ensure AsBias_WB_all is a not nullptr and is properly sized.
  DRAKE_THROW_UNLESS(AsBias_WB_all != nullptr);
  DRAKE_THROW_UNLESS(static_cast<int>(AsBias_WB_all->size()) == num_bodies());

  // To calculate a generic body A's spatial acceleration bias in world W,
  // note that body A's spatial velocity in world W is
  //     V_WA = J𝑠_V_WA ⋅ 𝑠
  // which upon vector differentiation in W gives A's spatial acceleration in W
  //     A_WA = J𝑠_V_WA ⋅ 𝑠̇  +  J̇𝑠_V_WA ⋅ 𝑠
  // Since A𝑠Bias_WA can be defined as the term in A_WA that does not include 𝑠̇,
  //     A𝑠Bias_WA = J̇𝑠_V_WA ⋅ 𝑠  =  A_WA − J𝑠_V_WA ⋅ 𝑠̇
  // One way to calculate A𝑠Bias_WA is to evaluate A_WA with 𝑠̇ = 0.  Hence, set
  // 𝑠̇ = 0 to calculate all bodies' spatial acceleration biases in world W.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  const VectorX<T> vdot = VectorX<T>::Zero(num_velocities());
  CalcSpatialAccelerationsFromVdot(context, pc, vc, vdot, AsBias_WB_all);
}

template <typename T>
SpatialAcceleration<T> MultibodyTree<T>::CalcBiasSpatialAcceleration(
    const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
    const Frame<T>& frame_A, const Frame<T>& frame_E) const {
  // TODO(mitiguy) Allow with_respect_to be JacobianWrtVariable::kQDot.
  DRAKE_THROW_UNLESS(with_respect_to == JacobianWrtVariable::kV);

  // Reserve room to store all the bodies' spatial acceleration bias in world W.
  // TODO(Mitiguy) Inefficient use of heap. Per issue #13560, implement caching.
  std::vector<SpatialAcceleration<T>> AsBias_WB_all(num_bodies());
  CalcAllBodyBiasSpatialAccelerationsInWorld(context, with_respect_to,
                                             &AsBias_WB_all);

  // Frame_B is regarded as fixed/welded to a body, herein named body_B.
  // Extract body_B's spatial acceleration bias in W from AsBias_WB_all.
  const RigidBody<T>& body_B = frame_B.body();
  const SpatialAcceleration<T> AsBias_WBodyB_W =
      AsBias_WB_all[body_B.mobod_index()];

  // Frame_A is regarded as fixed/welded to a body herein named body_A.
  // Extract body_A's spatial acceleration bias in W from AsBias_WB_all.
  const RigidBody<T>& body_A = frame_A.body();
  const SpatialAcceleration<T> AsBias_WBodyA_W =
      AsBias_WB_all[body_A.mobod_index()];

  // Calculate Bp's spatial acceleration bias in body_A, expressed in frame_E.
  return CalcSpatialAccelerationHelper(context, frame_B, p_BoBp_B, body_A,
                                       frame_E, AsBias_WBodyB_W,
                                       AsBias_WBodyA_W);
}

template <typename T>
SpatialAcceleration<T> MultibodyTree<T>::CalcSpatialAccelerationHelper(
    const systems::Context<T>& context, const Frame<T>& frame_F,
    const Eigen::Ref<const Vector3<T>>& p_FoFp_F, const RigidBody<T>& body_A,
    const Frame<T>& frame_E, const SpatialAcceleration<T>& A_WB_W,
    const SpatialAcceleration<T>& A_WA_W) const {
  // For a frame Fp that is fixed/welded to a frame_F, one way to calculate
  // A_AFp (Fp's spatial acceleration in body_A) is by rearranging formulas
  // for the angular acceleration and translational acceleration parts of
  // Fp's spatial acceleration in the world frame W.
  //
  // Since frame Fp is regarded as fixed/welded to both frame_F and a body_B,
  // Fp's angular acceleration in body_A is equal to body_B's angular
  // acceleration in body_A, and hence can be denoted α_AB and can be
  // calculated by rearranging the "angular acceleration addition theorem"
  // (from eqn (12) in SpatialAcceleration::ComposeWithMovingFrameAcceleration()
  // or Chap 8, Angular velocity/acceleration [Mitiguy 2019], reference below).
  //   (1)  α_WB = α_WA + α_AB + w_WA x w_AB   is rearranged to
  //   (2)  α_AB = α_WB - α_WA - w_WA x w_AB,  where
  // α_AB is body B's angular acceleration in body A,
  // α_WB is body B's angular acceleration in frame W (world),
  // α_WA is body A's angular acceleration in frame W (world),
  // w_WA is body A's angular velocity in frame W, and
  // w_AB is body B's angular velocity in body A.
  //
  // The translational acceleration part of A_AFp is denoted a_AFp and can be
  // calculated by rearranging the "one point moving on a rigid frame formula"
  // (from eqn (13) in SpatialAcceleration::ComposeWithMovingFrameAcceleration()
  // or from Chapter 10, Points: Velocity and acceleration [Mitiguy 2019]
  // or from section 2.8, page 39 [Kane & Levinson 1985], references below)
  //   (3)  a_WFp = a_WAp + a_AFp + 2 w_WA x v_AFp    is rearranged to
  //   (4)  a_AFp = a_WFp - a_WAp - 2 w_WA x v_AFp,  where
  // point Ap is the point fixed to body_A that is coincident with Fp,
  // a_AFp is Fp's translational acceleration in body_A,
  // a_WFp is Fp's translational acceleration in frame W (world),
  // a_WAp is point Ap's acceleration in frame W (calculated as shown below),
  // w_WA is body A's angular velocity in frame W,
  // v_AFp is Fp's translational velocity in body_A.
  //
  // The previous equations also apply to bias acceleration, so eqns (2) and (4)
  // apply to bias angular acceleration and bias translational acceleration as
  //   (5)  αBias_AB = αBias_WB - αBias_WA - w_WA x w_AB
  //   (6)  aBias_AFp = aBias_WFp - aBias_WAp - 2 w_WA x v_AFp
  //
  // - [Mitiguy, 2019]: "Advanced Dynamics and Motion Simulation,
  //   For professional engineers and scientists," Prodigy Press, Sunnyvale CA,
  //   Available at www.MotionGenesis.com
  // - [Kane & Levinson 1985] "Dynamics, Theory and Applications," McGraw-Hill.
  //    Available for free .pdf download: https://hdl.handle.net/1813/638
  // Shift spatial acceleration from body_B's origin to point Fp of frame_F.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  const SpatialAcceleration<T> A_WFp_W =
      ShiftSpatialAccelerationInWorld(frame_F, p_FoFp_F, A_WB_W, pc, vc);

  // Calculations are simpler if body_A (the "measured-in" frame) is the world
  // frame W.  Otherwise, extra calculations are needed.
  SpatialAcceleration<T> A_AFp_W;
  const Frame<T>& frame_W = world_frame();
  const Frame<T>& frame_A = body_A.body_frame();
  if (frame_A.is_world_frame()) {
    A_AFp_W = A_WFp_W;
  } else {
    // Point Ap is the point of (fixed to) body_A that is coincident with
    // point Fp. Calculate the position vector from Ao (body_A's origin) to Ap.
    const RigidTransform<T> X_AF = frame_F.CalcPose(context, frame_A);
    const Vector3<T> p_AoAp_A = X_AF * p_FoFp_F;  // Note: p_AoAp = p_AoFp

    // Shift spatial acceleration from body_A's origin to point Ap of body_A.
    // Note: Since Ap is regarded as fixed to body_A, Ap's translational
    // acceleration in the world frame W is calculated as
    //   a_WAp = a_WAo + α_WA x p_AoAp + w_WA x (w_WA x p_AoAp)
    // Reminder: p_AoAp is an "instantaneous" position vector, so differentation
    // of p_AoAp or a_WAp may produce a result different than you might expect.
    const SpatialAcceleration<T> A_WAp_W =
        ShiftSpatialAccelerationInWorld(frame_A, p_AoAp_A, A_WA_W, pc, vc);

    // Implement part of the formula from equations (5) and (6) above.
    // TODO(Mitiguy) Investigate whether it is more accurate and/or efficient
    //  to calculate (A_WFp_W - A_WAp_W) via a least common ancestor.
    // Discussion with reviewers (Sherm and Alejandro) included the following
    // thoughts about using a least common ancestor.
    // * There may be simulations in which using a least common ancestor is
    //   important for speed or avoiding loss of precision from cancellations.
    // * Code for operating in the ancestor frame requires conversions for
    //   quantities that were already available in World; there is some cost to
    //   that both in execution time and programming effort.
    // * In Simbody, Sherm used the least common ancestor for all constraint
    //   equations and grew to regret it. It was surprisingly complicated and
    //   the extra transformations made the code (including caching of results)
    //   complicated, ultimately with questionable saving of computation time.
    // * For Jacobians (one of Drake's fastest recursive calculations), it is
    //   unclear whether typical non-World frame relative accelerations would
    //   involve near-ancestors rather than far-ancestors. If the latter,
    //   then the extra iterations from World wouldn't matter much.
    A_AFp_W = A_WFp_W - A_WAp_W;  // Calculation of A_AFp_W is unfinished here.

    // Equation (5) is  α_AB = α_WB - α_WA - w_WA x w_AB,
    // hence calculate A's angular velocity in W and B's angular velocity in A.
    const Vector3<T> w_WA_W =
        body_A.EvalSpatialVelocityInWorld(context).rotational();
    SpatialVelocity<T> V_AF_W =
        frame_F.CalcSpatialVelocity(context, frame_A, frame_W);
    const Vector3<T> w_AF_W = V_AF_W.rotational();  // Frame F is welded to B.
    A_AFp_W.rotational() -= w_WA_W.cross(w_AF_W);

    // Equation (6) is  a_AFp = a_WFp - a_WAp - 2 w_WA x v_AFp,  hence calculate
    // Fp's velocity in A to form the "Coriolis acceleration" 2 w_WA x v_AFp.
    const RotationMatrix<T> R_WF = frame_F.CalcRotationMatrixInWorld(context);
    const Vector3<T> p_FoFp_W = R_WF * p_FoFp_F;
    const Vector3<T> v_AFp_W = V_AF_W.Shift(p_FoFp_W).translational();
    const Vector3<T> coriolis_acceleration = 2 * w_WA_W.cross(v_AFp_W);
    A_AFp_W.translational() -= coriolis_acceleration;
  }

  // If necessary, re-express the results in frame_E.
  if (frame_E.is_world_frame()) return A_AFp_W;
  const RotationMatrix<T> R_EW =
      frame_E.CalcRotationMatrixInWorld(context).inverse();
  const SpatialAcceleration<T> A_AFp_E = R_EW * A_AFp_W;
  return A_AFp_E;
}

template <typename T>
SpatialAcceleration<T> MultibodyTree<T>::ShiftSpatialAccelerationInWorld(
    const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
    const SpatialAcceleration<T>& A_WA_W, const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
  // frame_B is fixed/welded to body_A.
  const RigidBody<T>& body_A = frame_B.body();

  // Optimize for the common case that frame_B is a body frame.
  Vector3<T> p_AoBp_A;
  if (frame_B.is_body_frame()) {
    p_AoBp_A = p_BoBp_B;
  } else {
    // Form the position from Ao (body_A's origin) to Bp, expressed in body_A.
    const RigidTransform<T> X_AB = frame_B.GetFixedPoseInBodyFrame();
    p_AoBp_A = X_AB * p_BoBp_B;
  }

  // Form the position vector from Ao to Bp, expressed in the world frame W.
  const RotationMatrix<T>& R_WA = pc.get_R_WB(body_A.mobod_index());
  const Vector3<T> p_AoBp_W = R_WA * p_AoBp_A;

  // Shift spatial acceleration from body_A to frame_Bp.
  // Note: Since frame_B is assumed to be fixed to body_A, w_WB = w_WA.
  const Vector3<T>& w_WA_W = vc.get_V_WB(body_A.mobod_index()).rotational();
  SpatialAcceleration<T> A_WBp_W = A_WA_W.Shift(p_AoBp_W, w_WA_W);
  return A_WBp_W;
}

template <typename T>
Matrix3X<T> MultibodyTree<T>::CalcBiasTranslationalAcceleration(
    const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B, const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
    const Frame<T>& frame_A, const Frame<T>& frame_E) const {
  // TODO(mitiguy) Allow with_respect_to be JacobianWrtVariable::kQDot.
  DRAKE_THROW_UNLESS(with_respect_to == JacobianWrtVariable::kV);

  // Form frame_B's bias spatial acceleration in frame_A, expressed in frame_E.
  const SpatialAcceleration<T> AsBias_ABo_E = CalcBiasSpatialAcceleration(
      context, with_respect_to, frame_B, Vector3<T>::Zero(), frame_A, frame_E);

  // Get R_EB (rotation matrix relating frame_E to frame_B).
  const RotationMatrix<T> R_EB =
      CalcRelativeRotationMatrix(context, frame_E, frame_B);

  // Form w_AB_E (B's angular velocity in frame A, measured in frame_E).
  const Vector3<T> w_AB_E =
      frame_B.CalcSpatialVelocity(context, frame_A, frame_E).rotational();

  // Allocate the output vector.
  const int num_points = p_BoBi_B.cols();
  Matrix3X<T> asBias_ABi_E_array(3, num_points);

  // Fill the output vector with bias translational accelerations.
  for (int ipoint = 0; ipoint < num_points; ++ipoint) {
    // Express the position vector from Bo (frame_B's origin) to point Bp (the
    // ith point in the position vector list in p_BoBi_B) in frame_E.
    const Vector3<T> p_BoBp_E = R_EB * p_BoBi_B.col(ipoint);

    // Shift bias translational acceleration from Bo (frame_B's origin) to Bp.
    const SpatialAcceleration<T> AsBias_ABp_E =
        AsBias_ABo_E.Shift(p_BoBp_E, w_AB_E);

    // Store only the translational bias acceleration component in the results.
    asBias_ABi_E_array.col(ipoint) = AsBias_ABp_E.translational();
  }
  return asBias_ABi_E_array;
}

template <typename T>
void MultibodyTree<T>::CalcJacobianSpatialVelocity(
    const systems::Context<T>& context,
    const JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
    const Eigen::Ref<const Vector3<T>>& p_BP, const Frame<T>& frame_A,
    const Frame<T>& frame_E, EigenPtr<MatrixX<T>> Js_V_ABp_E) const {
  DRAKE_THROW_UNLESS(Js_V_ABp_E != nullptr);
  DRAKE_THROW_UNLESS(Js_V_ABp_E->rows() == 6);

  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot)
                              ? num_positions()
                              : num_velocities();
  DRAKE_THROW_UNLESS(Js_V_ABp_E->cols() == num_columns);

  // The spatial velocity V_WBp can be obtained by composing the spatial
  // velocities V_WAp and V_ABp. Expressed in the world frame W this composition
  // is V_WBp_W = V_WAp_W + V_ABp_W
  // Therefore, V_ABp_W = (Js_V_WBp - Js_V_WAp) ⋅ s.
  //
  // If with_respect_to = JacobianWrtVariable::kQDot, s = q̇ and
  // Js_V_W{Ap,Bp} = Jq_V_W{Ap,Bp},
  // If with_respect_to == JacobianWrtVariable::kV,  s = v and
  // Js_V_W{Ap,Bp} = Jv_V_W{Ap,Bp}.
  //
  // Expressed in frame E, this becomes
  //   V_ABp_E = R_EW⋅(Js_V_WBp - Js_V_WAp) ⋅ s.
  // Thus, Js_V_ABp_E = R_EW⋅(Js_V_WBp - Js_V_WAp).

  Vector3<T> p_WP;
  CalcPointsPositions(context, frame_B, p_BP, /* From frame B */
                      world_frame(), &p_WP);  /* To world frame W */

  // TODO(amcastro-tri): When performance becomes an issue, implement this
  // method so that we only consider the kinematic path from A to B.

  Matrix6X<T> Js_V_WAp(6, num_columns);
  auto Js_w_WAp = Js_V_WAp.template topRows<3>();     // rotational part.
  auto Js_v_WAp = Js_V_WAp.template bottomRows<3>();  // translational part.
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(
      context, with_respect_to, frame_A, p_WP, &Js_w_WAp, &Js_v_WAp);

  Matrix6X<T> Js_V_WBp(6, num_columns);
  auto Js_w_WBp = Js_V_WBp.template topRows<3>();     // rotational part.
  auto Js_v_WBp = Js_V_WBp.template bottomRows<3>();  // translational part.
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(
      context, with_respect_to, frame_B, p_WP, &Js_w_WBp, &Js_v_WBp);

  // Jacobian Js_V_ABp_W when E is the world frame W.
  Js_V_ABp_E->template topRows<3>() = Js_w_WBp - Js_w_WAp;
  Js_V_ABp_E->template bottomRows<3>() = Js_v_WBp - Js_v_WAp;

  // If the expressed-in frame E is not the world frame, we need to perform
  // an additional operation.
  if (frame_E.index() != world_frame().index()) {
    const RotationMatrix<T> R_EW =
        CalcRelativeRotationMatrix(context, frame_E, world_frame());
    Js_V_ABp_E->template topRows<3>() =
        R_EW * Js_V_ABp_E->template topRows<3>();
    Js_V_ABp_E->template bottomRows<3>() =
        R_EW * Js_V_ABp_E->template bottomRows<3>();
  }
}

template <typename T>
void MultibodyTree<T>::CalcJacobianAngularVelocity(
    const systems::Context<T>& context,
    const JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
    const Frame<T>& frame_A, const Frame<T>& frame_E,
    EigenPtr<Matrix3X<T>> Js_w_AB_E) const {
  DRAKE_THROW_UNLESS(Js_w_AB_E != nullptr);
  DRAKE_THROW_UNLESS(Js_w_AB_E->rows() == 3);
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot)
                              ? num_positions()
                              : num_velocities();
  DRAKE_THROW_UNLESS(Js_w_AB_E->cols() == num_columns);

  // The angular velocity addition theorem, gives w_WB = w_WA + w_AB, where
  // w_WB is frame B's angular velocity in world W,
  // w_WA is frame A's angular velocity in world W, and
  // w_AB is frame B's angular velocity in frame A.
  // Rearrange to calculate B's angular velocity in A as w_AB = w_WB - w_WA.
  // So B's angular velocity Jacobian in A, expressed in frame E is
  // Js_w_AB_E = R_EW * (Js_w_WB_W - Js_w_WA_W).

  // TODO(Mitiguy): When performance becomes an issue, optimize this method by
  //  only using the kinematics path from A to B.

  // Create dummy position list for signature requirements of next method.
  const Eigen::Matrix<T, 3, 0> empty_position_list;

  // TODO(Mitiguy) One way to avoid memory allocation and speed this up is to
  //  be clever and use the input argument as follows:
  //  Eigen::Ref<MatrixX<T>> Js_w_WA_W = *Js_w_AB_E;
  //  Also modify CalcJacobianAngularAndOrTranslationalVelocityInWorld() so
  //  it can add or subtract to the Jacobian that is passed to it.
  Matrix3X<T> Js_w_WA_W(3, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(
      context, with_respect_to, frame_A, empty_position_list, &Js_w_WA_W,
      nullptr);

  Matrix3X<T> Js_w_WB_W(3, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(
      context, with_respect_to, frame_B, empty_position_list, &Js_w_WB_W,
      nullptr);

  const Frame<T>& frame_W = world_frame();
  if (frame_E.index() == frame_W.index()) {
    // Calculate B's angular velocity Jacobian in A, expressed in W.
    *Js_w_AB_E = Js_w_WB_W - Js_w_WA_W;  // This calculates Js_w_AB_W.
  } else {
    // When frame E is not the world frame:
    // 1. Calculate B's angular velocity Jacobian in A, expressed in W.
    // 2. Re-express that Jacobian in frame_E (rather than frame_W).
    const RotationMatrix<T> R_EW =
        CalcRelativeRotationMatrix(context, frame_E, frame_W);
    *Js_w_AB_E = R_EW * (Js_w_WB_W - Js_w_WA_W);
  }
}

template <typename T>
void MultibodyTree<T>::CalcJacobianTranslationalVelocityHelper(
    const systems::Context<T>& context,
    const JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
    const Eigen::Ref<const Matrix3X<T>>& p_WoBi_W, const Frame<T>& frame_A,
    EigenPtr<MatrixX<T>> Js_v_ABi_W) const {
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot)
                              ? num_positions()
                              : num_velocities();
  const int num_points = p_WoBi_W.cols();
  DRAKE_THROW_UNLESS(num_points > 0);
  DRAKE_THROW_UNLESS(Js_v_ABi_W != nullptr);
  DRAKE_THROW_UNLESS(Js_v_ABi_W->rows() == 3 * num_points);
  DRAKE_THROW_UNLESS(Js_v_ABi_W->cols() == num_columns);

  // Bi's velocity in W can be calculated v_WBi = v_WAi + v_ABi, where
  // v_WBi is point Bi's translational velocity in world W,
  // v_WAi is point Ai's translational velocity in world W
  //         (where Ai is the point of A coincident with Bi),
  // v_ABi is point Bi's translational velocity in frame A.
  // Rearrange to calculate Bi's velocity in A as v_ABi = v_WBi - v_WAi.

  // TODO(Mitiguy): When performance becomes an issue, optimize this method by
  //  only using the kinematics path from A to B.

  // Calculate each point Bi's translational velocity Jacobian in world W.
  // The result is Js_v_WBi_W, but we store into Js_v_ABi_W for performance.
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(
      context, with_respect_to, frame_B, p_WoBi_W, nullptr, Js_v_ABi_W);

  // For the common special case in which frame A is the world W, optimize as
  // Js_v_ABi_W = Js_v_WBi_W
  if (frame_A.index() == world_frame().index()) return;

  // Calculate each point Ai's translational velocity Jacobian in world W.
  MatrixX<T> Js_v_WAi_W(3 * num_points, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(
      context, with_respect_to, frame_A, p_WoBi_W, nullptr, &Js_v_WAi_W);

  // Calculate each point Bi's translational velocity Jacobian in frame A,
  // expressed in world W. Note, again, that before this line Js_v_ABi_W
  // is actually storing Js_v_WBi_W.
  *Js_v_ABi_W -= Js_v_WAi_W;  // This calculates Js_v_ABi_W.
}

template <typename T>
void MultibodyTree<T>::CalcJacobianTranslationalVelocity(
    const systems::Context<T>& context,
    const JacobianWrtVariable with_respect_to, const Frame<T>& frame_B,
    const Frame<T>& frame_F, const Eigen::Ref<const Matrix3X<T>>& p_FoBi_F,
    const Frame<T>& frame_A, const Frame<T>& frame_E,
    EigenPtr<MatrixX<T>> Js_v_ABi_E) const {
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot)
                              ? num_positions()
                              : num_velocities();
  const int num_points = p_FoBi_F.cols();
  DRAKE_THROW_UNLESS(num_points > 0);
  DRAKE_THROW_UNLESS(p_FoBi_F.rows() == 3);
  DRAKE_THROW_UNLESS(Js_v_ABi_E != nullptr);
  DRAKE_THROW_UNLESS(Js_v_ABi_E->rows() == 3 * num_points);
  DRAKE_THROW_UNLESS(Js_v_ABi_E->cols() == num_columns);

  // If frame_F == frame_W (World), then just call helper method.
  // The helper method returns each point Bi's translational velocity Jacobian
  // in frame A, expressed in world W, i.e., helper method returns Js_v_ABi_W.
  const Frame<T>& frame_W = world_frame();
  if (&frame_F == &frame_W) {
    CalcJacobianTranslationalVelocityHelper(context, with_respect_to, frame_B,
                                            p_FoBi_F, frame_A, Js_v_ABi_E);
  } else {
    // If frame_F != frame_W, then for each point Bi, determine Bi's position
    // from Wo (World origin), expressed in world W and call helper method.
    Matrix3X<T> p_WoBi_W(3, num_points);
    CalcPointsPositions(context, frame_F, p_FoBi_F, /* From frame F */
                        world_frame(), &p_WoBi_W);  /* To world frame W */
    CalcJacobianTranslationalVelocityHelper(context, with_respect_to, frame_B,
                                            p_WoBi_W, frame_A, Js_v_ABi_E);
  }

  // If frame_E is not the world frame, re-express Js_v_ABi_W in frame_E as
  // Js_v_ABi_E = R_EW * (Js_v_WBi_W - Js_v_WAi_W).
  if (&frame_E != &frame_W) {
    const RotationMatrix<T> R_EW =
        CalcRelativeRotationMatrix(context, frame_E, frame_W);
    // Extract the 3 x num_columns block that starts at row = 3 * i, column = 0.
    for (int i = 0; i < num_points; ++i) {
      Js_v_ABi_E->template block<3, Eigen::Dynamic>(3 * i, 0, 3, num_columns) =
          R_EW * Js_v_ABi_E->template block<3, Eigen::Dynamic>(3 * i, 0, 3,
                                                               num_columns);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcJacobianAngularAndOrTranslationalVelocityInWorld(
    const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_F, const Eigen::Ref<const Matrix3X<T>>& p_WoFpi_W,
    EigenPtr<Matrix3X<T>> Js_w_WF_W, EigenPtr<MatrixX<T>> Js_v_WFpi_W) const {
  // At least one of the Jacobian output terms must be nullptr.
  DRAKE_THROW_UNLESS(Js_w_WF_W != nullptr || Js_v_WFpi_W != nullptr);

  const bool is_wrt_qdot = (with_respect_to == JacobianWrtVariable::kQDot);
  const int num_columns = is_wrt_qdot ? num_positions() : num_velocities();
  const int num_points = p_WoFpi_W.cols();

  // If non-nullptr, check the proper size of the output Jacobian matrices and
  // initialize the contents to zero.
  if (Js_w_WF_W) {
    DRAKE_THROW_UNLESS(Js_w_WF_W->rows() == 3);
    DRAKE_THROW_UNLESS(Js_w_WF_W->cols() == num_columns);
    Js_w_WF_W->setZero();
  }
  if (Js_v_WFpi_W) {
    DRAKE_THROW_UNLESS(Js_v_WFpi_W->rows() == 3 * num_points);
    DRAKE_THROW_UNLESS(Js_v_WFpi_W->cols() == num_columns);
    Js_v_WFpi_W->setZero();
  }

  // RigidBody to which frame_F is welded/attached.
  const RigidBody<T>& body_F = frame_F.body();

  // Return zero Jacobians for bodies anchored to the world, since for anchored
  // bodies, w_wF = Js_w_WF * v = 0  and  v_WFpi = Js_v_WFpi * v = 0.
  if (body_F.index() == world_index()) return;

  // Form kinematic path from World to body_F.
  std::vector<MobodIndex> path_from_world =
      forest().FindPathFromWorld(body_F.mobod_index());

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);

  // A stack allocated matrix with a maximum number of rows and columns.
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 7> Nplus;

  // For all bodies in the kinematic path from the world to body_F, compute
  // each node's contribution to the Jacobians.
  // Skip the world (level = 0).
  for (size_t level = 1; level < path_from_world.size(); ++level) {
    const MobodIndex mobod_index = path_from_world[level];
    const BodyNode<T>& node = *body_nodes_[mobod_index];
    const SpanningForest::Mobod& mobod = node.mobod();
    const int start_index_in_v = mobod.v_start();
    const int start_index_in_q = mobod.q_start();
    const int mobilizer_num_velocities = mobod.nv();
    const int mobilizer_num_positions = mobod.nq();

    const int start_index = is_wrt_qdot ? start_index_in_q : start_index_in_v;
    const int mobilizer_jacobian_ncols =
        is_wrt_qdot ? mobilizer_num_positions : mobilizer_num_velocities;

    // No contribution to the Jacobian from this mobilizer. We skip it.
    // N.B. This avoids working with zero sized Eigen blocks; see drake#17113.
    if (mobilizer_jacobian_ncols == 0) continue;

    // "Hinge matrix" H for across-node Jacobian.
    // Herein P designates the inboard (parent) body frame P.
    // B designates the current outboard body in this outward sweep.
    Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
        node.GetJacobianFromArray(H_PB_W_cache);

    // Aliases to angular and translational components in H_PB_W.
    const auto Hw_PB_W = H_PB_W.template topRows<3>();
    const auto Hv_PB_W = H_PB_W.template bottomRows<3>();

    // Mapping defined by v = N⁺(q)⋅q̇.
    if (is_wrt_qdot) {
      // TODO(amcastro-tri): consider using an operator version instead only
      //  if/when the computational cost of multiplying with Nplus from the
      //  right becomes a bottleneck.
      // Nplus is stack allocated above so this isn't a memory allocation.
      Nplus.resize(mobilizer_num_velocities, mobilizer_num_positions);
      node.get_mobilizer().CalcNplusMatrix(context, &Nplus);
    }

    // The Jacobian angular velocity term is the same for all points Fpi since
    // all these are points of (fixed/welded to) the same body_F.
    if (Js_w_WF_W) {
      // Get memory address in the output Jacobian angular velocity Js_w_WF_W
      // corresponding to the contribution of the mobilities in level ilevel.
      auto Js_w_PB_W =
          Js_w_WF_W->block(0, start_index, 3, mobilizer_jacobian_ncols);
      if (is_wrt_qdot) {
        Js_w_PB_W = Hw_PB_W * Nplus;
      } else {
        Js_w_PB_W = Hw_PB_W;
      }
    }

    if (Js_v_WFpi_W) {
      // Get memory address in the output block Jacobian translational velocity
      // Js_v_PFpi_W corresponding to the contribution of the mobilities in
      // level ilevel.  This address corresponds to point Fpi's Jacobian
      // translational velocity in the inboard (parent) body frame P, expressed
      // in world frame W.  That is, v_PFpi_W = Js_v_PFpi_W * v(B), where v(B)
      // are the mobilities that correspond to the current node.
      auto Js_v_PFpi_W = Js_v_WFpi_W->block(0, start_index, 3 * num_points,
                                            mobilizer_jacobian_ncols);

      // Position from Wo (world origin) to Bo (origin of body associated with
      // node at level ilevel), expressed in world frame W.
      const Vector3<T>& p_WoBo = pc.get_X_WB(node.mobod_index()).translation();

      for (int ipoint = 0; ipoint < num_points; ++ipoint) {
        // Position from Wo to Fp (ith point of Fpi), expressed in world W.
        const Vector3<T> p_WoFp = p_WoFpi_W.col(ipoint);

        // Position from Bo to Fp, expressed in world W.
        const Vector3<T> p_BoFp_W = p_WoFp - p_WoBo;

        // Point Fp's Jacobian translational velocity is placed in the output
        // memory block in the same order input points Fpi are listed on input.
        // Get a mutable alias into Js_v_PFpi_W for the Jacobian translational
        // velocity term for the currently indexed (ipoint) point.
        const int ipoint_row = 3 * ipoint;
        auto Hv_PFpi_W =
            Js_v_PFpi_W.block(ipoint_row, 0, 3, mobilizer_jacobian_ncols);

        // Now "shift" Hv_PB_W to Hv_PFqi_W one column at a time.
        // Reminder: frame_F is fixed/welded to body_F so its angular velocity
        // in world W is the same as body_F's angular velocity in W.
        if (is_wrt_qdot) {
          Hv_PFpi_W = (Hv_PB_W + Hw_PB_W.colwise().cross(p_BoFp_W)) * Nplus;
        } else {
          Hv_PFpi_W = Hv_PB_W + Hw_PB_W.colwise().cross(p_BoFp_W);
        }
      }
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcJacobianCenterOfMassTranslationalVelocity(
    const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_A, const Frame<T>& frame_E,
    EigenPtr<Matrix3X<T>> Js_v_AScm_E) const {
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot)
                              ? num_positions()
                              : num_velocities();
  DRAKE_THROW_UNLESS(Js_v_AScm_E != nullptr);
  DRAKE_THROW_UNLESS(Js_v_AScm_E->cols() == num_columns);

  // Reminder: MultibodyTree always declares a world body (0ᵗʰ body).
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  Js_v_AScm_E->setZero();
  T composite_mass = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    const Vector3<T> pi_BoBcm = body.CalcCenterOfMassInBodyFrame(context);
    MatrixX<T> Jsi_v_ABcm_E(3, num_columns);
    CalcJacobianTranslationalVelocity(
        context, with_respect_to, body.body_frame(), body.body_frame(),
        pi_BoBcm, frame_A, frame_E, &Jsi_v_ABcm_E);
    const T& body_mass = body.get_mass(context);
    *Js_v_AScm_E += body_mass * Jsi_v_ABcm_E;
    composite_mass += body_mass;
  }

  if (composite_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  *Js_v_AScm_E /= composite_mass;
}

template <typename T>
void MultibodyTree<T>::CalcJacobianCenterOfMassTranslationalVelocity(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances,
    JacobianWrtVariable with_respect_to, const Frame<T>& frame_A,
    const Frame<T>& frame_E, EigenPtr<Matrix3X<T>> Js_v_AScm_E) const {
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot)
                              ? num_positions()
                              : num_velocities();
  DRAKE_THROW_UNLESS(Js_v_AScm_E != nullptr);
  DRAKE_THROW_UNLESS(Js_v_AScm_E->cols() == num_columns);

  // Reminder: MultibodyTree always declares a world body.
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Js_v_AScm_E->setZero();

  // Sum over all bodies contained in model_instances except for the 0th body
  // (which is the world body), and count each body's contribution only once.
  // Reminder: Although it is not possible for a body to belong to multiple
  // model instances [as RigidBody::model_instance() returns a body's unique
  // model instance], it is possible for the same model instance to be added
  // multiple times to std::vector<ModelInstanceIndex>& model_instances). The
  // code below ensures a body's contribution to the sum occurs only once.
  // Duplicate model_instances in std::vector are ignored.
  int number_of_non_world_bodies_processed = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      const T& body_mass = body.get_mass(context);
      total_mass += body_mass;  // total_mass = ∑ mᵢ.
      ++number_of_non_world_bodies_processed;

      // sum_mi_Ji = ∑ (mᵢ Jᵢ), where mᵢ is the mass of the iᵗʰ body and
      // Jᵢ is Bᵢcm's translational velocity Jacobian in frame A, expressed in
      // frame E (Bᵢcm is the center of mass of the iᵗʰ body).
      const Vector3<T> pi_BoBcm = body.CalcCenterOfMassInBodyFrame(context);
      MatrixX<T> Jsi_v_ABcm_E(3, num_columns);
      CalcJacobianTranslationalVelocity(
          context, with_respect_to, body.body_frame(), body.body_frame(),
          pi_BoBcm, frame_A, frame_E, &Jsi_v_ABcm_E);
      *Js_v_AScm_E += body_mass * Jsi_v_ABcm_E;
    }
  }

  // Throw an exception if there are zero non-world bodies in model_instances.
  if (number_of_non_world_bodies_processed == 0) {
    std::string message = fmt::format(
        "{}(): There must be at least one "
        "non-world body contained in model_instances.",
        __func__);
    throw std::logic_error(message);
  }

  if (total_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  /// J𝑠_v_AScm_ = ∑ (mᵢ Jᵢ) / mₛ, where mₛ = ∑ mᵢ.
  *Js_v_AScm_E /= total_mass;
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcBiasCenterOfMassTranslationalAcceleration(
    const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_A, const Frame<T>& frame_E) const {
  // Reminder: MultibodyTree always declares a world body (0ᵗʰ body).
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  T composite_mass = 0;
  Vector3<T> asBias_AScm_E = Vector3<T>::Zero();
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    const Vector3<T> pi_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
    const Frame<T>& frame_B = body.body_frame();
    const SpatialAcceleration<T> AsBiasi_AScm_E = CalcBiasSpatialAcceleration(
        context, with_respect_to, frame_B, pi_BoBcm_B, frame_A, frame_E);
    const T& body_mass = body.get_mass(context);
    asBias_AScm_E += body_mass * AsBiasi_AScm_E.translational();
    composite_mass += body_mass;
  }

  if (composite_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }
  asBias_AScm_E /= composite_mass;
  return asBias_AScm_E;
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcBiasCenterOfMassTranslationalAcceleration(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances,
    JacobianWrtVariable with_respect_to, const Frame<T>& frame_A,
    const Frame<T>& frame_E) const {
  // Reminder: MultibodyTree always declares a world body (0ᵗʰ body).
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Vector3<T> sum_mi_aBiasi = Vector3<T>::Zero();
  // Sum over all bodies contained in model_instances except for the 0th body
  // (which is the world body), and count each body's contribution only once.
  // Reminder: Although it is not possible for a body to belong to multiple
  // model instances [as RigidBody::model_instance() returns a body's unique
  // model instance], it is possible for the same model instance to be added
  // multiple times to std::vector<ModelInstanceIndex>& model_instances). The
  // code below ensures a body's contribution to the sum occurs only once.
  // Duplicate model_instances in std::vector are ignored.
  int number_of_non_world_bodies_processed = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      const T& body_mass = body.get_mass(context);
      total_mass += body_mass;  // total_mass = ∑ mᵢ.
      ++number_of_non_world_bodies_processed;

      // sum_mi_aBiasi = ∑ (mᵢ aBiasᵢ), where mᵢ is the mass of the iᵗʰ body and
      // aBiasᵢ is Bᵢcm's bias translational acceleration in frame A, expressed
      // in frame E (Bᵢcm is the center of mass of the iᵗʰ body).
      const Frame<T>& frame_B = body.body_frame();
      const Vector3<T> pi_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
      const Vector3<T> aBiasi_ABcm_E = CalcBiasTranslationalAcceleration(
          context, with_respect_to, frame_B, pi_BoBcm_B, frame_A, frame_E);
      sum_mi_aBiasi += body_mass * aBiasi_ABcm_E;
    }
  }

  // Throw an exception if there are zero non-world bodies in model_instances.
  if (number_of_non_world_bodies_processed == 0) {
    std::string message = fmt::format(
        "{}(): There must be at least one "
        "non-world body contained in model_instances.",
        __func__);
    throw std::logic_error(message);
  }

  if (total_mass <= 0) {
    std::string message = fmt::format(
        "{}(): The system's total mass must be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  /// aBias_AScm_E = ∑ (mᵢ sum_mi_aBiasi) / mₛ, where mₛ = ∑ mᵢ.
  const Vector3<T> aBias_AScm_E = sum_mi_aBiasi / total_mass;
  return aBias_AScm_E;
}

template <typename T>
T MultibodyTree<T>::CalcPotentialEnergy(
    const systems::Context<T>& context) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  T potential_energy = 0.0;
  // Add contributions from force elements.
  for (const auto& force_element : force_elements_) {
    potential_energy += force_element->CalcPotentialEnergy(context, pc);
  }
  return potential_energy;
}

template <typename T>
T MultibodyTree<T>::CalcKineticEnergy(
    const systems::Context<T>& context) const {
  const std::vector<SpatialInertia<T>>& M_Bi_W =
      EvalSpatialInertiaInWorldCache(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  const VectorX<T>& reflected_inertia = EvalReflectedInertiaCache(context);
  T twice_kinetic_energy_W = 0.0;
  // Add contributions from each body (except World).
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const MobodIndex mobod_index = get_body(body_index).mobod_index();
    const SpatialInertia<T>& M_B_W = M_Bi_W[mobod_index];
    const SpatialVelocity<T>& V_WB = vc.get_V_WB(mobod_index);
    const SpatialMomentum<T> L_WB = M_B_W * V_WB;

    twice_kinetic_energy_W += L_WB.dot(V_WB);
  }

  // Account for reflected inertia.
  // See JointActuator::reflected_inertia().
  const Eigen::VectorBlock<const VectorX<T>> v = get_velocities(context);

  twice_kinetic_energy_W +=
      (v.array() * reflected_inertia.array() * v.array()).sum();

  return twice_kinetic_energy_W / 2.;
}

template <typename T>
T MultibodyTree<T>::CalcConservativePower(
    const systems::Context<T>& context) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  T conservative_power = 0.0;
  // Add contributions from force elements.
  for (const auto& force_element : force_elements_) {
    conservative_power += force_element->CalcConservativePower(context, pc, vc);
  }
  return conservative_power;
}

template <typename T>
T MultibodyTree<T>::CalcNonConservativePower(
    const systems::Context<T>& context) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  T non_conservative_power = 0.0;

  // Add contributions from force elements.
  for (const auto& force_element : force_elements_) {
    non_conservative_power +=
        force_element->CalcNonConservativePower(context, pc, vc);
  }

  // TODO(sherm1) Add contributions from joint dampers, joint actuators, force
  //              input ports, contact forces, etc. as enumerated in #12942.

  return non_conservative_power;
}

template <typename T>
void MultibodyTree<T>::ThrowIfFinalized(const char* source_method) const {
  if (is_finalized()) {
    throw std::logic_error(
        "Post-finalize calls to '" + std::string(source_method) +
        "()' are "
        "not allowed; calls to this method must happen before Finalize().");
  }
}

template <typename T>
void MultibodyTree<T>::ThrowIfNotFinalized(const char* source_method) const {
  if (!is_finalized()) {
    throw std::logic_error("Pre-finalize calls to '" +
                           std::string(source_method) +
                           "()' are "
                           "not allowed; you must call Finalize() first.");
  }
}

template <typename T>
void MultibodyTree<T>::ThrowDefaultMassInertiaError() const {
  ThrowIfNotFinalized(__func__);

  // First look through all the WeldedMobods groups. We'll do the non-welded
  // Mobods afterwards.
  for (const auto& welded_mobods_group : forest().welded_mobods()) {
    const SpanningForest::Mobod& active_mobod =
        forest().mobods(welded_mobods_group[0]);
    if (active_mobod.is_world()) continue;  // All anchored bodies are OK.
    DRAKE_DEMAND(!active_mobod.is_weld());  // That wouldn't be active!
    if (active_mobod.nq_outboard() > 0) continue;  // Not a terminal group.

    // At this point we're looking at a non-World, terminal WeldedMobods group.
    // Find the matching WeldedLinksAssembly that carries the mass properties.
    const std::optional<WeldedLinksAssemblyIndex> link_assembly_index =
        graph().links(active_mobod.link_ordinal()).welded_links_assembly();
    DRAKE_DEMAND(link_assembly_index.has_value());  // Should be an assembly!
    const auto& link_assembly =
        graph().welded_links_assemblies(*link_assembly_index);
    DRAKE_DEMAND(link_assembly.links()[0] ==
                 graph().links(active_mobod.link_ordinal()).index());

    ThrowIfTerminalBodyHasBadDefaultMassProperties(link_assembly.links(),
                                                   active_mobod.index());
  }

  // Now the non-welded (singleton) mobilized bodies.
  for (const auto& mobod : forest().mobods()) {
    if (mobod.welded_mobods_group().has_value()) continue;  // Already done.
    if (!mobod.is_leaf_mobod()) continue;  // An interior Mobod.

    ThrowIfTerminalBodyHasBadDefaultMassProperties(
        {graph().links(mobod.link_ordinal()).index()}, mobod.index());
  }
}

// This is just a private helper for the above function; the arguments are
// assumed to have the expected properties.
template <typename T>
void MultibodyTree<T>::ThrowIfTerminalBodyHasBadDefaultMassProperties(
    const std::vector<BodyIndex>& link_assembly,  // might be just a Link
    MobodIndex active_mobilizer_index) const {
  DRAKE_DEMAND(!link_assembly.empty());
  const bool is_assembly = ssize(link_assembly) > 1;
  const Mobilizer<T>& active_mobilizer = get_mobilizer(active_mobilizer_index);
  const bool can_rotate = active_mobilizer.can_rotate();
  const bool can_translate = active_mobilizer.can_translate();

  const std::string& active_link_name = get_body(link_assembly[0]).name();
  const char* description = is_assembly
                                ? "the active body for a terminal assembly"
                                : "a terminal body";

  if (can_translate && (CalcTotalDefaultMass(link_assembly) == 0)) {
    throw std::logic_error(
        fmt::format("Body {} is {} that is massless, but its joint has a "
                    "translational degree of freedom.",
                    active_link_name, description));
  }

  if (can_rotate && IsAnyDefaultRotationalInertiaNaN(link_assembly)) {
    throw std::logic_error(fmt::format(
        "Body {} is {} that has a NaN rotational inertia, but its joint has a "
        "rotational degree of freedom.",
        active_link_name, description));
  }

  if (can_rotate && AreAllDefaultRotationalInertiaZero(link_assembly)) {
    throw std::logic_error(fmt::format(
        "Body {} is {} that has zero rotational inertia, but its joint has a "
        "rotational degree of freedom.",
        active_link_name, description));
  }
}

// N.B. Ignores NaN masses.
template <typename T>
double MultibodyTree<T>::CalcTotalDefaultMass(
    const std::vector<BodyIndex>& body_indexes) const {
  double total_mass = 0;
  for (BodyIndex body_index : body_indexes) {
    const RigidBody<T>& body_B = get_body(body_index);
    const double mass_B = body_B.default_mass();
    if (!std::isnan(mass_B)) total_mass += mass_B;
  }
  return total_mass;
}

template <typename T>
bool MultibodyTree<T>::IsAnyDefaultRotationalInertiaNaN(
    const std::vector<BodyIndex>& body_indexes) const {
  for (BodyIndex body_index : body_indexes) {
    const RigidBody<T>& body_B = get_body(body_index);
    const RotationalInertia<double> I_BBo_B =
        body_B.default_rotational_inertia();
    if (I_BBo_B.IsNaN()) return true;
  }
  return false;
}

template <typename T>
bool MultibodyTree<T>::AreAllDefaultRotationalInertiaZero(
    const std::vector<BodyIndex>& body_indexes) const {
  for (BodyIndex body_index : body_indexes) {
    const RigidBody<T>& body_B = get_body(body_index);
    const RotationalInertia<double> I_BBo_B =
        body_B.default_rotational_inertia();
    if (!I_BBo_B.IsZero()) return false;
  }
  return true;
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyInertiaCache(
    const systems::Context<T>& context,
    ArticulatedBodyInertiaCache<T>* abic) const {
  const VectorX<T>& reflected_inertia = EvalReflectedInertiaCache(context);
  CalcArticulatedBodyInertiaCache(context, reflected_inertia, abic);
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyInertiaCache(
    const systems::Context<T>& context, const VectorX<T>& diagonal_inertias,
    ArticulatedBodyInertiaCache<T>* abic) const {
  DRAKE_DEMAND(abic != nullptr);

  // TODO(amcastro-tri): consider combining these to improve memory access
  //  pattern into a single position kinematics pass.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);
  const std::vector<SpatialInertia<T>>& spatial_inertia_in_world_cache =
      EvalSpatialInertiaInWorldCache(context);

  // Perform tip-to-base recursion, skipping the world.
  for (int depth = forest_height() - 1; depth > 0; --depth) {
    for (MobodIndex mobod_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[mobod_index];

      // Get hinge matrix and spatial inertia for this node.
      Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
          node.GetJacobianFromArray(H_PB_W_cache);
      const SpatialInertia<T>& M_B_W =
          spatial_inertia_in_world_cache[mobod_index];

      node.CalcArticulatedBodyInertiaCache_TipToBase(context, pc, H_PB_W, M_B_W,
                                                     diagonal_inertias, abic);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyForceCache(
    const systems::Context<T>& context,
    const ArticulatedBodyInertiaCache<T>& abic,
    const std::vector<SpatialForce<T>>& Zb_Bo_W_cache,
    const MultibodyForces<T>& forces,
    ArticulatedBodyForceCache<T>* aba_force_cache) const {
  DRAKE_DEMAND(aba_force_cache != nullptr);
  DRAKE_DEMAND(forces.CheckHasRightSizeForModel(*this));

  // Get position and velocity kinematics from cache.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Extract generalized forces and body forces.
  const VectorX<T>& generalized_forces = forces.generalized_forces();
  const std::vector<SpatialForce<T>>& body_forces = forces.body_forces();

  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);

  // Eval spatial inertia M_B_W(q) and force bias Fb_B_W(q, v) as they appear on
  // the Newton-Euler equation: M_B_W * A_WB + Fb_B_W = Fapp_B_W.
  const std::vector<SpatialForce<T>>& dynamic_bias_cache =
      EvalDynamicBiasCache(context);

  // Perform tip-to-base recursion, skipping the world.
  for (int depth = forest_height() - 1; depth > 0; --depth) {
    for (MobodIndex mobod_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[mobod_index];

      // Get generalized force and body force for this node.
      Eigen::Ref<const VectorX<T>> tau_applied =
          node.get_mobilizer().get_generalized_forces_from_array(
              generalized_forces);
      const SpatialForce<T>& Fapplied_Bo_W = body_forces[mobod_index];

      // Get references to the hinge matrix and force bias for this node.
      Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
          node.GetJacobianFromArray(H_PB_W_cache);
      const SpatialForce<T>& Fb_B_W = dynamic_bias_cache[mobod_index];
      const SpatialForce<T>& Zb_Bo_W = Zb_Bo_W_cache[mobod_index];

      node.CalcArticulatedBodyForceCache_TipToBase(
          context, pc, &vc, Fb_B_W, abic, Zb_Bo_W, Fapplied_Bo_W, tau_applied,
          H_PB_W, aba_force_cache);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyForceCache(
    const systems::Context<T>& context, const MultibodyForces<T>& forces,
    ArticulatedBodyForceCache<T>* aba_force_cache) const {
  // Get configuration dependent articulated body inertia cache.
  const ArticulatedBodyInertiaCache<T>& abic =
      EvalArticulatedBodyInertiaCache(context);
  // We evaluate the kinematics dependent articulated body force bias Zb_Bo_W =
  // Pplus_PB_W * Ab_WB. When cached, this corresponds to a significant
  // computational gain when performing ABA with the same context (storing the
  // same q and v) but different applied `forces`.
  const std::vector<SpatialForce<T>>& Zb_Bo_W_cache =
      EvalArticulatedBodyForceBiasCache(context);
  CalcArticulatedBodyForceCache(context, abic, Zb_Bo_W_cache, forces,
                                aba_force_cache);
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyAccelerations(
    const systems::Context<T>& context,
    const ArticulatedBodyInertiaCache<T>& abic,
    const ArticulatedBodyForceCache<T>& aba_force_cache,
    AccelerationKinematicsCache<T>* ac) const {
  DRAKE_DEMAND(ac != nullptr);
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);
  const std::vector<SpatialAcceleration<T>>& Ab_WB_cache =
      EvalSpatialAccelerationBiasCache(context);

  // Perform base-to-tip recursion, skipping the world.
  for (int level = 1; level < forest_height(); ++level) {
    for (MobodIndex mobod_index : body_node_levels_[level]) {
      const BodyNode<T>& node = *body_nodes_[mobod_index];

      const SpatialAcceleration<T>& Ab_WB = Ab_WB_cache[mobod_index];

      // Get reference to the hinge mapping matrix.
      Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
          node.GetJacobianFromArray(H_PB_W_cache);

      node.CalcArticulatedBodyAccelerations_BaseToTip(
          context, pc, abic, aba_force_cache, H_PB_W, Ab_WB, ac);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyAccelerations(
    const systems::Context<T>& context,
    const ArticulatedBodyForceCache<T>& aba_force_cache,
    AccelerationKinematicsCache<T>* ac) const {
  const ArticulatedBodyInertiaCache<T>& abic =
      EvalArticulatedBodyInertiaCache(context);
  CalcArticulatedBodyAccelerations(context, abic, aba_force_cache, ac);
}

template <typename T>
MatrixX<double> MultibodyTree<T>::MakeStateSelectorMatrix(
    const std::vector<JointIndex>& user_to_joint_index_map) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  // We create a set in order to verify that joint indexes appear only once.
  std::unordered_set<JointIndex> already_selected_joints;
  for (const auto& joint_index : user_to_joint_index_map) {
    const bool inserted = already_selected_joints.insert(joint_index).second;
    if (!inserted) {
      throw std::logic_error("Joint named '" + get_joint(joint_index).name() +
                             "' is repeated multiple times.");
    }
  }

  // Determine the size of the vector of "selected" states xₛ.
  int num_selected_positions = 0;
  int num_selected_velocities = 0;
  for (JointIndex joint_index : user_to_joint_index_map) {
    num_selected_positions += get_joint(joint_index).num_positions();
    num_selected_velocities += get_joint(joint_index).num_velocities();
  }
  const int num_selected_states =
      num_selected_positions + num_selected_velocities;

  // With state x of size n and selected state xₛ of size nₛ, Sx has size
  // nₛ x n so that xₛ = Sx⋅x.
  MatrixX<double> Sx = MatrixX<double>::Zero(num_selected_states, num_states());

  const int nq = num_positions();
  // We place all selected positions first, followed by all the selected
  // velocities, as in the original state x.
  int selected_positions_index = 0;
  int selected_velocities_index = num_selected_positions;
  for (JointIndex joint_index : user_to_joint_index_map) {
    const auto& joint = get_joint(joint_index);

    const int pos_start = joint.position_start();
    const int num_pos = joint.num_positions();
    const int vel_start = joint.velocity_start();  // within v
    const int num_vel = joint.num_velocities();

    Sx.block(selected_positions_index, pos_start, num_pos, num_pos) =
        MatrixX<double>::Identity(num_pos, num_pos);

    Sx.block(selected_velocities_index, nq + vel_start, num_vel, num_vel) =
        MatrixX<double>::Identity(num_vel, num_vel);

    selected_positions_index += num_pos;
    selected_velocities_index += num_vel;
  }

  return Sx;
}

template <typename T>
MatrixX<double> MultibodyTree<T>::MakeStateSelectorMatrixFromJointNames(
    const std::vector<std::string>& selected_joints) const {
  std::vector<JointIndex> selected_joints_indexes;
  for (const auto& joint_name : selected_joints) {
    selected_joints_indexes.push_back(GetJointByName(joint_name).index());
  }
  return MakeStateSelectorMatrix(selected_joints_indexes);
}

template <typename T>
MatrixX<double> MultibodyTree<T>::MakeActuatorSelectorMatrix(
    const std::vector<JointActuatorIndex>& user_to_actuator_index_map) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  const int num_selected_actuators = user_to_actuator_index_map.size();

  // The actuation selector matrix maps the vector of "selected" actuators to
  // the full vector of actuators: u = Sᵤ⋅uₛ.
  MatrixX<double> Su =
      MatrixX<double>::Zero(num_actuated_dofs(), num_selected_actuators);
  int user_index = 0;
  for (JointActuatorIndex actuator_index : user_to_actuator_index_map) {
    Su(get_joint_actuator(actuator_index).input_start(), user_index) = 1.0;
    ++user_index;
  }

  return Su;
}

template <typename T>
MatrixX<double> MultibodyTree<T>::MakeActuatorSelectorMatrix(
    const std::vector<JointIndex>& user_to_joint_index_map) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  // Map of joint ordinal to actuator index for all actuators.
  std::vector<std::optional<JointActuatorIndex>> joint_to_actuator_index(
      num_joints());
  for (JointActuatorIndex actuator_index : GetJointActuatorIndices()) {
    const auto& actuator = get_joint_actuator(actuator_index);
    joint_to_actuator_index[actuator.joint().ordinal()] = actuator_index;
  }

  // Build a list of actuators in the order given by user_to_joint_index_map,
  // which must contain actuated joints. We verify this.
  std::vector<JointActuatorIndex> user_to_actuator_index_map;
  for (JointIndex joint_index : user_to_joint_index_map) {
    const auto& joint = get_joint(joint_index);

    // If the joint does not have an actuator.
    if (!joint_to_actuator_index.at(joint.ordinal()).has_value()) {
      throw std::logic_error("Joint '" + joint.name() +
                             "' does not have an actuator.");
    }

    user_to_actuator_index_map.push_back(
        joint_to_actuator_index.at(joint.ordinal()).value());
  }

  return MakeActuatorSelectorMatrix(user_to_actuator_index_map);
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetPositionLowerLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd q_lower = Eigen::VectorXd::Constant(
      num_positions(), -std::numeric_limits<double>::infinity());
  for (JointIndex i : GetJointIndices()) {
    const auto& joint = get_joint(i);
    q_lower.segment(joint.position_start(), joint.num_positions()) =
        joint.position_lower_limits();
  }
  return q_lower;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetPositionUpperLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd q_upper = Eigen::VectorXd::Constant(
      num_positions(), std::numeric_limits<double>::infinity());
  for (JointIndex i : GetJointIndices()) {
    const auto& joint = get_joint(i);
    q_upper.segment(joint.position_start(), joint.num_positions()) =
        joint.position_upper_limits();
  }
  return q_upper;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetVelocityLowerLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd v_lower = Eigen::VectorXd::Constant(
      num_velocities(), -std::numeric_limits<double>::infinity());
  for (JointIndex i : GetJointIndices()) {
    const auto& joint = get_joint(i);
    v_lower.segment(joint.velocity_start(), joint.num_velocities()) =
        joint.velocity_lower_limits();
  }
  return v_lower;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetVelocityUpperLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd v_upper = Eigen::VectorXd::Constant(
      num_velocities(), std::numeric_limits<double>::infinity());
  for (JointIndex i : GetJointIndices()) {
    const auto& joint = get_joint(i);
    v_upper.segment(joint.velocity_start(), joint.num_velocities()) =
        joint.velocity_upper_limits();
  }
  return v_upper;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetAccelerationLowerLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd vd_lower = Eigen::VectorXd::Constant(
      num_velocities(), -std::numeric_limits<double>::infinity());
  for (JointIndex i : GetJointIndices()) {
    const auto& joint = get_joint(i);
    vd_lower.segment(joint.velocity_start(), joint.num_velocities()) =
        joint.acceleration_lower_limits();
  }
  return vd_lower;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetAccelerationUpperLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd vd_upper = Eigen::VectorXd::Constant(
      num_velocities(), std::numeric_limits<double>::infinity());
  for (JointIndex i : GetJointIndices()) {
    const auto& joint = get_joint(i);
    vd_upper.segment(joint.velocity_start(), joint.num_velocities()) =
        joint.acceleration_upper_limits();
  }
  return vd_upper;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetEffortLowerLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd lower = Eigen::VectorXd::Constant(
      num_actuated_dofs(), -std::numeric_limits<double>::infinity());
  for (JointActuatorIndex i : GetJointActuatorIndices()) {
    const auto& actuator = get_joint_actuator(i);
    for (int j = actuator.input_start();
         j < actuator.input_start() + actuator.num_inputs(); ++j) {
      DRAKE_ASSERT(j < num_actuated_dofs());
      lower[j] = -actuator.effort_limit();
    }
  }
  return lower;
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetEffortUpperLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd upper = Eigen::VectorXd::Constant(
      num_actuated_dofs(), std::numeric_limits<double>::infinity());
  for (JointActuatorIndex i : GetJointActuatorIndices()) {
    const auto& actuator = get_joint_actuator(i);
    for (int j = actuator.input_start();
         j < actuator.input_start() + actuator.num_inputs(); ++j) {
      DRAKE_ASSERT(j < num_actuated_dofs());
      upper[j] = actuator.effort_limit();
    }
  }
  return upper;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<MultibodyTree<ToScalar>> MultibodyTree<T>::CloneToScalar()
    const {
  if (!is_finalized()) {
    throw std::logic_error(
        "Attempting to clone a MultibodyTree that has not been finalized."
        " MultibodyTree::Finalize() must be called before attempting to clone"
        " a MultibodyTree.");
  }
  auto tree_clone = std::make_unique<MultibodyTree<ToScalar>>();

  // The graph and its forest model are scalar type-independent.
  tree_clone->link_joint_graph_ = this->link_joint_graph_;

  // Fill the `frame_` collection with nulls. We'll be cloning the frames out
  // of order, so we can't just append them to the end like we do with the
  // other kinds of elements.
  tree_clone->frames_.ResizeToMatch(frames_);

  // Skipping the world body at body_index = 0.
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const RigidBody<T>& body = get_body(body_index);
    tree_clone->CloneBodyAndAdd(body);
  }

  // Skip the world (0) and default (1) instances.
  for (ModelInstanceIndex index(2); index < num_model_instances(); ++index) {
    tree_clone->AddModelInstance(model_instances_.get_element(index).name());
  }

  // Frames are cloned in their index order, that is, in the exact same order
  // they were added to the original tree. Since the Frame API enforces the
  // creation of the parent frame first, this traversal guarantees that parent
  // body frames are created before their child frames.
  for (const Frame<T>* frame : frames_.elements()) {
    // If the frame was a RigidBodyFrame then it will already have been set
    // in `frames_`. We should only clone frames that don't exist yet.
    if (!tree_clone->frames_.has_element(frame->index())) {
      tree_clone->CloneFrameAndAdd(*frame);
    }
  }

  for (const auto& mobilizer : mobilizers_) {
    // This call assumes that tree_clone already contains all the cloned
    // frames.
    tree_clone->CloneMobilizerAndAdd(*mobilizer);
  }

  // Throw away the default constructed gravity element.
  tree_clone->force_elements_.clear();
  tree_clone->gravity_field_ = nullptr;
  for (const auto& force_element : force_elements_) {
    tree_clone->CloneForceElementAndAdd(*force_element);
  }

  DRAKE_DEMAND(tree_clone->num_force_elements() > 0);
  tree_clone->gravity_field_ =
      dynamic_cast<UniformGravityFieldElement<ToScalar>*>(
          tree_clone->force_elements_[0].get());
  DRAKE_DEMAND(tree_clone->gravity_field_ != nullptr);

  // Fill the `joints_` collection with nulls. This is to preserve the
  // removed index structure of the ElementCollection when adding the clone's
  // joints.
  tree_clone->joints_.ResizeToMatch(joints_);

  // Since Joint<T> objects are implemented from basic element objects like
  // RigidBody, Mobilizer, ForceElement and Constraint, they are cloned last
  // so that the clones of their dependencies are guaranteed to be available.
  // DO NOT change this order!!!
  for (const Joint<T>* joint : joints_.elements()) {
    tree_clone->CloneJointAndAdd(*joint);
  }

  // Fill the `actuators_` collection with nulls. This is to preserve the
  // removed index structure of the ElementCollection when adding the cloned
  // actuators.
  tree_clone->actuators_.ResizeToMatch(actuators_);

  for (const JointActuator<T>* actuator : actuators_.elements()) {
    tree_clone->CloneActuatorAndAdd(*actuator);
  }

  tree_clone->num_actuated_dofs_ = this->num_actuated_dofs_;

  // We can safely make a deep copy here since the original multibody tree is
  // required to be finalized.
  tree_clone->joint_to_mobilizer_ = this->joint_to_mobilizer_;
  tree_clone->discrete_state_index_ = this->discrete_state_index_;

  // All other internals templated on T are created with the following call to
  // FinalizeInternals(), which also sets the "is_finalized" flag to true.
  tree_clone->FinalizeInternals();
  return tree_clone;
}

template <typename T>
template <typename FromScalar>
Frame<T>* MultibodyTree<T>::CloneFrameAndAdd(const Frame<FromScalar>& frame) {
  const FrameIndex frame_index = frame.index();

  auto frame_clone = frame.CloneToScalar(*this);
  frame_clone->set_parent_tree(this, frame_index);
  frame_clone->set_model_instance(frame.model_instance());
  Frame<T>* result = frame_clone.get();
  frames_.Add(std::move(frame_clone));
  return result;
}

template <typename T>
template <typename FromScalar>
RigidBody<T>* MultibodyTree<T>::CloneBodyAndAdd(
    const RigidBody<FromScalar>& body) {
  const BodyIndex body_index = body.index();
  const FrameIndex body_frame_index = body.body_frame().index();

  auto body_clone = body.CloneToScalar(*this);
  body_clone->set_parent_tree(this, body_index);
  body_clone->set_model_instance(body.model_instance());
  // MultibodyTree can access selected private methods in RigidBody through its
  // RigidBodyAttorney.
  Frame<T>* body_frame_clone =
      &internal::RigidBodyAttorney<T>::get_mutable_body_frame(body_clone.get());
  body_frame_clone->set_parent_tree(this, body_frame_index);
  body_frame_clone->set_model_instance(body.model_instance());

  // The order in which frames are added into frames_ is important to keep the
  // topology invariant. Therefore we index new clones according to the
  // original body_frame_index.
  frames_.AddBorrowed(body_frame_clone);
  // The order in which bodies are added into owned_bodies_ is important to keep
  // the topology invariant. Therefore this method is called from
  // MultibodyTree::CloneToScalar() within a loop by original body_index.
  return &rigid_bodies_.Add(std::move(body_clone));
}

template <typename T>
template <typename FromScalar>
Mobilizer<T>* MultibodyTree<T>::CloneMobilizerAndAdd(
    const Mobilizer<FromScalar>& mobilizer) {
  const MobodIndex mobilizer_index = mobilizer.index();
  auto mobilizer_clone = mobilizer.CloneToScalar(*this);
  mobilizer_clone->set_parent_tree(this, mobilizer_index);
  mobilizer_clone->set_model_instance(mobilizer.model_instance());
  mobilizer_clone->set_is_floating_base_mobilizer(
      mobilizer.is_floating_base_mobilizer());
  Mobilizer<T>* raw_mobilizer_clone_ptr = mobilizer_clone.get();
  mobilizers_.push_back(std::move(mobilizer_clone));
  return raw_mobilizer_clone_ptr;
}

template <typename T>
template <typename FromScalar>
void MultibodyTree<T>::CloneForceElementAndAdd(
    const ForceElement<FromScalar>& force_element) {
  const ForceElementIndex force_element_index = force_element.index();
  auto force_element_clone = force_element.CloneToScalar(*this);
  force_element_clone->set_parent_tree(this, force_element_index);
  force_element_clone->set_model_instance(force_element.model_instance());
  force_elements_.push_back(std::move(force_element_clone));
}

template <typename T>
template <typename FromScalar>
Joint<T>* MultibodyTree<T>::CloneJointAndAdd(const Joint<FromScalar>& joint) {
  auto joint_clone = joint.CloneToScalar(this);
  joint_clone->set_parent_tree(this, joint.index());
  joint_clone->set_ordinal(joint.ordinal());
  joint_clone->set_model_instance(joint.model_instance());
  return &joints_.Add(std::move(joint_clone));
}

template <typename T>
template <typename FromScalar>
void MultibodyTree<T>::CloneActuatorAndAdd(
    const JointActuator<FromScalar>& actuator) {
  std::unique_ptr<JointActuator<T>> actuator_clone =
      actuator.CloneToScalar(*this);
  actuator_clone->set_parent_tree(this, actuator.index());
  actuator_clone->set_model_instance(actuator.model_instance());
  actuators_.Add(std::move(actuator_clone));
}

template <typename T>
std::optional<BodyIndex> MultibodyTree<T>::MaybeGetUniqueBaseBodyIndex(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instances_.has_element(model_instance));
  if (model_instance == world_model_instance()) return std::nullopt;

  // We need only look at World's outboard mobods since those are the base
  // bodies of each tree in the forest. Each of those mobods has an
  // associated Link (the active link in case of WeldedLinksAssemblies). We're
  // only interested in links in the given model instance, and there should be
  // just one of those.
  const SpanningForest::Mobod& world = forest().world_mobod();
  std::optional<BodyIndex> base_body_index{};
  for (const MobodIndex& base_mobod_index : world.outboards()) {
    const SpanningForest::Mobod& base_mobod = forest().mobods(base_mobod_index);
    DRAKE_DEMAND(base_mobod.level() == 1);
    const LinkJointGraph::Link& active_link =
        forest().links(base_mobod.link_ordinal());
    if (active_link.model_instance() != model_instance) continue;
    if (base_body_index.has_value())  // Not unique if already set.
      return std::nullopt;
    base_body_index = active_link.index();
  }
  return base_body_index;
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &MultibodyTree<T>::template CloneToScalar<U>
));
// clang-format on

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MultibodyTree);
