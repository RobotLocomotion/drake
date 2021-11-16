#include "drake/multibody/tree/multibody_tree.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/body_node_welded.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace internal {

using internal::BodyNode;
using internal::BodyNodeWelded;
using math::RigidTransform;
using math::RotationMatrix;

// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBT_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBT_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)

template <typename T>
class JointImplementationBuilder {
 public:
  JointImplementationBuilder() = delete;
  static std::vector<Mobilizer<T>*> Build(
      Joint<T>* joint, MultibodyTree<T>* tree) {
    std::vector<Mobilizer<T>*> mobilizers;
    std::unique_ptr<JointBluePrint> blue_print =
        joint->MakeImplementationBlueprint();
    auto implementation = std::make_unique<JointImplementation>(*blue_print);
    DRAKE_DEMAND(implementation->num_mobilizers() != 0);
    for (auto& mobilizer : blue_print->mobilizers_) {
      mobilizers.push_back(mobilizer.get());
      tree->AddMobilizer(std::move(mobilizer));
    }
    // TODO(amcastro-tri): add force elements, bodies, constraints, etc.
    joint->OwnImplementation(std::move(implementation));
    return mobilizers;
  }

 private:
  typedef typename Joint<T>::BluePrint JointBluePrint;
  typedef typename Joint<T>::JointImplementation JointImplementation;
};

template <typename T>
MultibodyTree<T>::MultibodyTree() {
  // Adds a "world" body to MultibodyTree having a NaN SpatialInertia.
  ModelInstanceIndex world_instance = AddModelInstance("WorldModelInstance");

  // `world_model_instance()` hardcodes the returned index.  Make sure it's
  // correct.
  DRAKE_DEMAND(world_instance == world_model_instance());
  world_body_ = &AddRigidBody("WorldBody", world_model_instance(),
                              SpatialInertia<double>());

  // `default_model_instance()` hardcodes the returned index.  Make sure it's
  // correct.
  ModelInstanceIndex default_instance =
      AddModelInstance("DefaultModelInstance");
  DRAKE_DEMAND(default_instance == default_model_instance());

  const ForceElement<T>& new_field =
      AddForceElement<UniformGravityFieldElement>();
  DRAKE_DEMAND(num_force_elements() == 1);
  DRAKE_DEMAND(owned_force_elements_[0].get() == &new_field);
}

template <typename T>
const std::string& MultibodyTree<T>::GetModelInstanceName(
    ModelInstanceIndex model_instance) const {
  const auto it = instance_index_to_name_.find(model_instance);
  if (it == instance_index_to_name_.end()) {
    throw std::logic_error(
        fmt::format("There is no model instance id {} in the model.",
                    std::to_string(model_instance)));
  }
  return it->second;
}

template <typename T>
bool MultibodyTree<T>::HasUniqueFreeBaseBodyImpl(
    ModelInstanceIndex model_instance) const {
  std::optional<BodyIndex> base_body_index =
      MaybeGetUniqueBaseBodyIndex(model_instance);
  return base_body_index.has_value() &&
         owned_bodies_[base_body_index.value()]->is_floating();
}

template <typename T>
const Body<T>& MultibodyTree<T>::GetUniqueFreeBaseBodyOrThrowImpl(
    ModelInstanceIndex model_instance) const {
  std::optional<BodyIndex> base_body_index =
      MaybeGetUniqueBaseBodyIndex(model_instance);
  if (!base_body_index.has_value()) {
    throw std::logic_error("Model " +
                           instance_index_to_name_.at(model_instance) +
                           " does not have a unique base body.");
  }
  if (!owned_bodies_[base_body_index.value()]->is_floating()) {
    throw std::logic_error("Model " +
                           instance_index_to_name_.at(model_instance) +
                           " has a unique base body, but it is not free.");
  }
  return *owned_bodies_[base_body_index.value()];
}

namespace {

// Given an ElementIndex type (e.g., BodyIndex) as a template argument, returns
// the name of the corresponding class (e.g., "Body").
template <typename ElementIndex>
std::string_view GetElementClassname() {
  if constexpr (std::is_same_v<ElementIndex, BodyIndex>) {
    return "Body";
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
const auto& GetElementByIndex(
    const MultibodyTree<T>& tree,
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
using NameToIndex = std::unordered_multimap<StringViewMapKey, ElementIndex>;

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
bool HasElementNamed(
    const MultibodyTree<T>& tree, std::string_view name,
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

// Common implementation for GetBodyByName, GetFrameByName, etc.
template <typename T, typename ElementIndex>
const auto& GetElementByName(
    const MultibodyTree<T>& tree, std::string_view name,
    std::optional<ModelInstanceIndex> model_instance,
    const NameToIndex<ElementIndex>& name_to_index) {
  // We fetch the model instance name as the first operation in this function
  // (even though we'll only need it for error messages) because it throws an
  // exception when the model_instance index is invalid.
  const std::string& model_instance_name =
      model_instance ? tree.GetModelInstanceName(*model_instance)
                     : std::string();
  const std::string_view element_classname =
      GetElementClassname<ElementIndex>();

  // Find all elements with a matching name.
  auto [lower, upper] = name_to_index.equal_range(name);

  // If the name is non-existent, then say so, whether or not a specific model
  // instance was requested.
  if (lower == upper) {
    throw std::logic_error(fmt::format(
        "Get{}ByName(): There is no {} named '{}' anywhere in the model.",
        element_classname, element_classname, name));
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
  return static_cast<int>(body_name_to_index_.count(name));
}

template <typename T>
bool MultibodyTree<T>::HasBodyNamed(std::string_view name) const {
  return HasElementNamed(*this, name, std::nullopt, body_name_to_index_);
}

template <typename T>
bool MultibodyTree<T>::HasBodyNamed(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return HasElementNamed(*this, name, model_instance, body_name_to_index_);
}

template <typename T>
bool MultibodyTree<T>::HasFrameNamed(std::string_view name) const {
  return HasElementNamed(*this, name, std::nullopt, frame_name_to_index_);
}

template <typename T>
bool MultibodyTree<T>::HasFrameNamed(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return HasElementNamed(*this, name, model_instance, frame_name_to_index_);
}

template <typename T>
bool MultibodyTree<T>::HasJointNamed(std::string_view name) const {
  return HasElementNamed(*this, name, std::nullopt, joint_name_to_index_);
}

template <typename T>
bool MultibodyTree<T>::HasJointNamed(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return HasElementNamed(*this, name, model_instance, joint_name_to_index_);
}

template <typename T>
bool MultibodyTree<T>::HasJointActuatorNamed(std::string_view name) const {
  return HasElementNamed(*this, name, std::nullopt, actuator_name_to_index_);
}

template <typename T>
bool MultibodyTree<T>::HasJointActuatorNamed(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return HasElementNamed(*this, name, model_instance, actuator_name_to_index_);
}

template <typename T>
bool MultibodyTree<T>::HasModelInstanceNamed(std::string_view name) const {
  return instance_name_to_index_.find(name) != instance_name_to_index_.end();
}

template <typename T>
const Body<T>& MultibodyTree<T>::GetBodyByName(std::string_view name) const {
  return GetElementByName(*this, name, std::nullopt, body_name_to_index_);
}

template <typename T>
const Body<T>& MultibodyTree<T>::GetBodyByName(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return GetElementByName(*this, name, model_instance, body_name_to_index_);
}

template <typename T>
std::vector<BodyIndex> MultibodyTree<T>::GetBodyIndices(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
  std::vector<BodyIndex> indices;
  for (auto& body : owned_bodies_) {
    if (body->model_instance() == model_instance) {
      indices.emplace_back(body->index());
    }
  }
  return indices;
}

template <typename T>
std::vector<JointIndex> MultibodyTree<T>::GetJointIndices(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
  std::vector<JointIndex> indices;
  for (auto& joint : owned_joints_) {
    if (joint->model_instance() == model_instance) {
      indices.emplace_back(joint->index());
    }
  }
  return indices;
}

template <typename T>
std::vector<FrameIndex> MultibodyTree<T>::GetFrameIndices(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
  std::vector<FrameIndex> indices;
  for (auto& frame : frames_) {
    if (frame->model_instance() == model_instance) {
      indices.emplace_back(frame->index());
    }
  }
  return indices;
}

template <typename T>
const Frame<T>& MultibodyTree<T>::GetFrameByName(std::string_view name) const {
  return GetElementByName(*this, name, std::nullopt, frame_name_to_index_);
}

template <typename T>
const Frame<T>& MultibodyTree<T>::GetFrameByName(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return GetElementByName(*this, name, model_instance, frame_name_to_index_);
}

template <typename T>
const RigidBody<T>& MultibodyTree<T>::GetRigidBodyByName(
    std::string_view name) const {
  const RigidBody<T>* body =
      dynamic_cast<const RigidBody<T>*>(&GetBodyByName(name));
  if (body == nullptr) {
    throw std::logic_error(
        fmt::format("Body '{}' is not a RigidBody.", name));
  }
  return *body;
}

template <typename T>
const RigidBody<T>& MultibodyTree<T>::GetRigidBodyByName(
    std::string_view name, ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
  const RigidBody<T>* body =
      dynamic_cast<const RigidBody<T>*>(&GetBodyByName(name, model_instance));
  if (body == nullptr) {
    throw std::logic_error(
        fmt::format("Body '{}' in model instance '{}' is not a RigidBody.",
                    name, instance_index_to_name_.at(model_instance)));
  }
  return *body;
}

template <typename T>
const Joint<T>& MultibodyTree<T>::GetJointByNameImpl(
    std::string_view name,
    std::optional<ModelInstanceIndex> model_instance) const {
  return GetElementByName(*this, name, model_instance, joint_name_to_index_);
}

template <typename T>
void MultibodyTree<T>::ThrowJointSubtypeMismatch(
    const Joint<T>& joint, std::string_view desired_type) const {
  throw std::logic_error(fmt::format(
      "GetJointByName(): Joint '{}' in model instance '{}' is not of type {} "
      "but of type {}.",
      joint.name(), instance_index_to_name_.at(joint.model_instance()),
      desired_type, NiceTypeName::Get(joint)));
}

template <typename T>
const JointActuator<T>& MultibodyTree<T>::GetJointActuatorByName(
    std::string_view name) const {
  return GetElementByName(*this, name, std::nullopt, actuator_name_to_index_);
}

template <typename T>
const JointActuator<T>& MultibodyTree<T>::GetJointActuatorByName(
    std::string_view name, ModelInstanceIndex model_instance) const {
  return GetElementByName(*this, name, model_instance, actuator_name_to_index_);
}

template <typename T>
ModelInstanceIndex MultibodyTree<T>::GetModelInstanceByName(
    std::string_view name) const {
  const auto it = instance_name_to_index_.find(name);
  if (it == instance_name_to_index_.end()) {
    throw std::logic_error(fmt::format(
        "GetModelInstanceByName(): There is no model instance named '{}'.",
        name));
  }
  return it->second;
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetActuationFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& u) const {
  return model_instances_.at(model_instance)->GetActuationFromArray(u);
}

template <typename T>
void MultibodyTree<T>::SetActuationInArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& u_instance,
    EigenPtr<VectorX<T>> u) const {
  model_instances_.at(model_instance)->SetActuationInArray(u_instance, u);
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetPositionsFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& q) const {
  return model_instances_.at(model_instance)->GetPositionsFromArray(q);
}

template <typename T>
void MultibodyTree<T>::GetPositionsFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& q,
    drake::EigenPtr<VectorX<T>> q_out) const {
  model_instances_.at(model_instance)->GetPositionsFromArray(q, q_out);
}

template <class T>
void MultibodyTree<T>::SetPositionsInArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& q_instance,
    EigenPtr<VectorX<T>> q) const {
  model_instances_.at(model_instance)->SetPositionsInArray(q_instance, q);
}

template <typename T>
VectorX<T> MultibodyTree<T>::GetVelocitiesFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& v) const {
  return model_instances_.at(model_instance)->GetVelocitiesFromArray(v);
}

template <typename T>
void MultibodyTree<T>::GetVelocitiesFromArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& v,
    drake::EigenPtr<VectorX<T>> v_out) const {
  model_instances_.at(model_instance)->GetVelocitiesFromArray(v, v_out);
}

template <class T>
void MultibodyTree<T>::SetVelocitiesInArray(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& v_instance,
    EigenPtr<VectorX<T>> v) const {
  model_instances_.at(model_instance)->SetVelocitiesInArray(v_instance, v);
}

template <typename T>
void MultibodyTree<T>::AddQuaternionFreeMobilizerToAllBodiesWithNoMobilizer() {
  DRAKE_DEMAND(!topology_is_valid());
  // Skip the world.
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    const BodyTopology& body_topology =
        get_topology().get_body(body.index());
    if (!body_topology.inboard_mobilizer.is_valid()) {
      std::unique_ptr<QuaternionFloatingMobilizer<T>> mobilizer =
          std::make_unique<QuaternionFloatingMobilizer<T>>(
              world_body().body_frame(), body.body_frame());
      mobilizer->set_model_instance(body.model_instance());
      this->AddMobilizer(std::move(mobilizer));
    }
  }
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
MultibodyTree<T>::GetFreeBodyMobilizerOrThrow(
    const Body<T>& body) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  DRAKE_DEMAND(body.index() != world_index());
  const BodyTopology& body_topology = get_topology().get_body(body.index());
  const QuaternionFloatingMobilizer<T>* mobilizer =
      dynamic_cast<const QuaternionFloatingMobilizer<T>*>(
          &get_mobilizer(body_topology.inboard_mobilizer));
  if (mobilizer == nullptr) {
    throw std::logic_error(
        "Body '" + body.name() + "' is not a free floating body.");
  }
  return *mobilizer;
}

template <typename T>
void MultibodyTree<T>::FinalizeTopology() {
  // If the topology is valid it means that this MultibodyTree was already
  // finalized. Re-compilation is not allowed.
  if (topology_is_valid()) {
    throw std::logic_error(
        "Attempting to call MultibodyTree::FinalizeTopology() on a tree with"
        " an already finalized topology.");
  }

  // Before performing any setup that depends on the scalar type <T>, compile
  // all the type-T independent topological information.
  topology_.Finalize();
}

template <typename T>
void MultibodyTree<T>::FinalizeInternals() {
  if (!topology_is_valid()) {
    throw std::logic_error(
        "MultibodyTree::FinalizeTopology() must be called before "
        "MultibodyTree::FinalizeInternals().");
  }

  // Give different multiobody elements the chance to perform any finalize-time
  // setup.
  for (const auto& body : owned_bodies_) {
    body->SetTopology(topology_);
  }
  for (const auto& frame : owned_frames_) {
    frame->SetTopology(topology_);
  }
  for (const auto& mobilizer : owned_mobilizers_) {
    mobilizer->SetTopology(topology_);
  }
  for (const auto& force_element : owned_force_elements_) {
    force_element->SetTopology(topology_);
  }
  for (const auto& actuator : owned_actuators_) {
    actuator->SetTopology(topology_);
  }

  body_node_levels_.resize(topology_.tree_height());
  for (BodyNodeIndex body_node_index(1);
       body_node_index < topology_.get_num_body_nodes(); ++body_node_index) {
    const BodyNodeTopology& node_topology =
        topology_.get_body_node(body_node_index);
    body_node_levels_[node_topology.level].push_back(body_node_index);
  }

  // Creates BodyNode's:
  // This recursion order ensures that a BodyNode's parent is created before the
  // node itself, since BodyNode objects are in Depth First Traversal order.
  for (BodyNodeIndex body_node_index(0);
       body_node_index < topology_.get_num_body_nodes(); ++body_node_index) {
    CreateBodyNode(body_node_index);
  }

  CreateModelInstances();
}

template <typename T>
void MultibodyTree<T>::Finalize() {
  DRAKE_MBT_THROW_IF_FINALIZED();
  // Create Joint objects's implementation. Joints are implemented using a
  // combination of MultibodyTree's building blocks such as Body, Mobilizer,
  // ForceElement and Constraint. For a same physical Joint, several
  // implementations could be created (for instance, a Constraint instead of a
  // Mobilizer). The decision on what implementation to create is performed by
  // MultibodyTree at Finalize() time. Then, JointImplementationBuilder below
  // can request MultibodyTree for these choices when building the Joint
  // implementation. Since a Joint's implementation is built upon
  // MultibodyTree's building blocks, notice that creating a Joint's
  // implementation will therefore change the tree topology. Since topology
  // changes are NOT allowed after Finalize(), joint implementations MUST be
  // assembled BEFORE the tree's topology is finalized.
  joint_to_mobilizer_.resize(num_joints());
  for (auto& joint : owned_joints_) {
    std::vector<Mobilizer<T>*> mobilizers =
        internal::JointImplementationBuilder<T>::Build(joint.get(), this);
    // Below we assume a single mobilizer per joint, the sane thing to do.
    // TODO(amcastro-tri): clean up the JointImplementationBuilder so that this
    // assumption (one mobilizer per joint) is set in stone once and for all.
    DRAKE_DEMAND(mobilizers.size() == 1);
    for (Mobilizer<T>* mobilizer : mobilizers) {
      mobilizer->set_model_instance(joint->model_instance());
      // Record the joint to mobilizer map.
      joint_to_mobilizer_[joint->index()] = mobilizer->index();
    }
  }
  // It is VERY important to add quaternions if needed only AFTER joints had a
  // chance to get implemented with mobilizers. This is because joints's
  // implementations change the topology of the tree. Therefore, do not change
  // this order!
  AddQuaternionFreeMobilizerToAllBodiesWithNoMobilizer();
  FinalizeTopology();
  FinalizeInternals();
}

template <typename T>
void MultibodyTree<T>::CreateBodyNode(BodyNodeIndex body_node_index) {
  const BodyNodeTopology& node_topology =
      topology_.get_body_node(body_node_index);
  const BodyIndex body_index = node_topology.body;

  const Body<T>* body = owned_bodies_[node_topology.body].get();

  std::unique_ptr<BodyNode<T>> body_node;
  if (body_index == world_index()) {
    body_node = std::make_unique<BodyNodeWelded<T>>(&world_body());
  } else {
    // The mobilizer should be valid if not at the root (the world).
    DRAKE_ASSERT(node_topology.mobilizer.is_valid());
    const Mobilizer<T>* mobilizer =
        owned_mobilizers_[node_topology.mobilizer].get();

    BodyNode<T>* parent_node =
        body_nodes_[node_topology.parent_body_node].get();

    // Only the mobilizer knows how to create a body node with compile-time
    // fixed sizes.
    body_node = mobilizer->CreateBodyNode(parent_node, body, mobilizer);
    parent_node->add_child_node(body_node.get());
  }
  body_node->set_parent_tree(this, body_node_index);
  body_node->SetTopology(topology_);

  body_nodes_.push_back(std::move(body_node));
}

template <typename T>
void MultibodyTree<T>::CreateModelInstances() {
  DRAKE_ASSERT(model_instances_.empty());

  // First create the pool of instances.
  for (ModelInstanceIndex model_instance_index(0);
       model_instance_index < num_model_instances(); ++model_instance_index) {
    std::unique_ptr<internal::ModelInstance<T>> model_instance =
        std::make_unique<internal::ModelInstance<T>>(model_instance_index);
    model_instance->set_parent_tree(this, model_instance_index);
    model_instances_.push_back(std::move(model_instance));
  }

  // Add all of our mobilizers and joint actuators to the appropriate instance.
  // The order of the mobilizers should match the order in which the bodies were
  // added to the tree, which may not be the order in which the mobilizers were
  // added, so we get the mobilizer through the BodyNode.
  for (const auto& body_node : body_nodes_) {
    if (body_node->get_num_mobilizer_positions() > 0 ||
        body_node->get_num_mobilizer_velocities() > 0) {
      model_instances_.at(body_node->model_instance())->add_mobilizer(
          &body_node->get_mobilizer());
    }
  }

  for (const auto& joint_actuator : owned_actuators_) {
    model_instances_.at(joint_actuator->model_instance())->add_joint_actuator(
        joint_actuator.get());
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
void MultibodyTree<T>::SetDefaultState(
    const systems::Context<T>& context, systems::State<T>* state) const {
  for (const auto& mobilizer : owned_mobilizers_) {
    mobilizer->set_default_state(context, state);
  }
}

template <typename T>
void MultibodyTree<T>::SetRandomState(const systems::Context<T>& context,
                                      systems::State<T>* state,
                                      RandomGenerator* generator) const {
  for (const auto& mobilizer : owned_mobilizers_) {
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
    const systems::Context<T>& context,
    ModelInstanceIndex model_instance,
    EigenPtr<VectorX<T>> qv_out) const {
  DRAKE_DEMAND(qv_out != nullptr);

  Eigen::VectorBlock<const VectorX<T>> state_vector =
      get_positions_and_velocities(context);

  if (qv_out->size() != num_positions(model_instance) +
      num_velocities(model_instance))
    throw std::logic_error("Output array is not properly sized.");

  auto qv_out_head = qv_out->head(num_positions(model_instance));
  auto qv_out_tail = qv_out->tail(num_velocities(model_instance));

  GetPositionsFromArray(
      model_instance, state_vector.head(num_positions()), &qv_out_head);
  GetVelocitiesFromArray(
      model_instance, state_vector.tail(num_velocities()), &qv_out_tail);
}

template <typename T>
void MultibodyTree<T>::SetPositionsAndVelocities(
    ModelInstanceIndex model_instance,
    const Eigen::Ref<const VectorX<T>>& instance_state,
    systems::Context<T>* context) const {
  Eigen::VectorBlock<VectorX<T>> state_vector =
      GetMutablePositionsAndVelocities(context);
  Eigen::VectorBlock<VectorX<T>> q = make_mutable_block_segment(&state_vector,
      0, num_positions());
  Eigen::VectorBlock<VectorX<T>> v = make_mutable_block_segment(&state_vector,
      num_positions(), num_velocities());
  SetPositionsInArray(model_instance,
                      instance_state.head(num_positions(model_instance)), &q);
  SetVelocitiesInArray(model_instance,
                       instance_state.tail(num_velocities(model_instance)), &v);
}

template <typename T>
RigidTransform<T> MultibodyTree<T>::GetFreeBodyPoseOrThrow(
    const systems::Context<T>& context, const Body<T>& body) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  const QuaternionFloatingMobilizer<T>& mobilizer =
      GetFreeBodyMobilizerOrThrow(body);
  return RigidTransform<T>(mobilizer.get_quaternion(context),
                                 mobilizer.get_position(context));
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyPoseOrThrow(
    const Body<T>& body, const RigidTransform<T>& X_WB,
    systems::Context<T>* context) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  SetFreeBodyPoseOrThrow(body, X_WB, *context, &context->get_mutable_state());
}

template <typename T>
void MultibodyTree<T>::SetFreeBodySpatialVelocityOrThrow(
    const Body<T>& body, const SpatialVelocity<T>& V_WB,
    systems::Context<T>* context) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  SetFreeBodySpatialVelocityOrThrow(
      body, V_WB, *context, &context->get_mutable_state());
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyPoseOrThrow(
    const Body<T>& body, const RigidTransform<T>& X_WB,
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  const QuaternionFloatingMobilizer<T>& mobilizer =
      GetFreeBodyMobilizerOrThrow(body);
  const RotationMatrix<T>& R_WB = X_WB.rotation();
  mobilizer.set_quaternion(context, R_WB.ToQuaternion(), state);
  mobilizer.set_position(context, X_WB.translation(), state);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodySpatialVelocityOrThrow(
    const Body<T>& body, const SpatialVelocity<T>& V_WB,
    const systems::Context<T>& context, systems::State<T>* state) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  const QuaternionFloatingMobilizer<T>& mobilizer =
      GetFreeBodyMobilizerOrThrow(body);
  mobilizer.set_angular_velocity(context, V_WB.rotational(), state);
  mobilizer.set_translational_velocity(context, V_WB.translational(), state);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyRandomPositionDistributionOrThrow(
    const Body<T>& body, const Vector3<symbolic::Expression>& position) {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  QuaternionFloatingMobilizer<T>& mobilizer =
      get_mutable_variant(GetFreeBodyMobilizerOrThrow(body));
  mobilizer.set_random_position_distribution(position);
}

template <typename T>
void MultibodyTree<T>::SetFreeBodyRandomRotationDistributionOrThrow(
    const Body<T>& body,
    const Eigen::Quaternion<symbolic::Expression>& rotation) {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  QuaternionFloatingMobilizer<T>& mobilizer =
      get_mutable_variant(GetFreeBodyMobilizerOrThrow(body));
  mobilizer.set_random_quaternion_distribution(rotation);
}

template <typename T>
void MultibodyTree<T>::CalcAllBodyPosesInWorld(
    const systems::Context<T>& context,
    std::vector<RigidTransform<T>>* X_WB) const {
  DRAKE_THROW_UNLESS(X_WB != nullptr);
  if (static_cast<int>(X_WB->size()) != num_bodies()) {
    X_WB->resize(num_bodies(), RigidTransform<T>::Identity());
  }
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const BodyNodeIndex node_index = get_body(body_index).node_index();
    X_WB->at(body_index) = pc.get_X_WB(node_index);
  }
}

template <typename T>
void MultibodyTree<T>::CalcAllBodySpatialVelocitiesInWorld(
    const systems::Context<T>& context,
    std::vector<SpatialVelocity<T>>* V_WB) const {
  DRAKE_THROW_UNLESS(V_WB != nullptr);
  if (static_cast<int>(V_WB->size()) != num_bodies()) {
    V_WB->resize(num_bodies(), SpatialVelocity<T>::Zero());
  }
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);
  for (BodyIndex body_index(0); body_index < num_bodies(); ++body_index) {
    const BodyNodeIndex node_index = get_body(body_index).node_index();
    V_WB->at(body_index) = vc.get_V_WB(node_index);
  }
}

template <typename T>
void MultibodyTree<T>::CalcPositionKinematicsCache(
    const systems::Context<T>& context,
    PositionKinematicsCache<T>* pc) const {
  DRAKE_DEMAND(pc != nullptr);

  // TODO(amcastro-tri): Loop over bodies to update their position dependent
  // kinematics. This gives the chance to flexible bodies to update the pose
  // X_BQ(qb_B) of each frame Q that is attached to the body.
  // Notice this loop can be performed in any order and each X_BQ(qf_B) is
  // independent of all others. This could even be performed in parallel.

  // With the kinematics information across mobilizer's and the kinematics
  // information for each body, we are now in position to perform a base-to-tip
  // recursion to update world positions and parent to child body transforms.
  // This skips the world, level = 0.
  for (int level = 1; level < tree_height(); ++level) {
    for (BodyNodeIndex body_node_index : body_node_levels_[level]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == level);
      DRAKE_ASSERT(node.index() == body_node_index);

      // Update per-node kinematics.
      node.CalcPositionKinematicsCache_BaseToTip(context, pc);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcVelocityKinematicsCache(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    VelocityKinematicsCache<T>* vc) const {
  DRAKE_DEMAND(vc != nullptr);

  // TODO(amcastro-tri): Loop over bodies to compute velocity kinematics updates
  // corresponding to flexible bodies.

  // If the model has zero dofs we simply set all spatial velocities to zero and
  // return since there is no work to be done.
  if (num_velocities() == 0) {
    vc->InitializeToZero();
    return;
  }

  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);

  // Performs a base-to-tip recursion computing body velocities.
  // This skips the world, depth = 0.
  for (int depth = 1; depth < tree_height(); ++depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.index() == body_node_index);

      // Hinge matrix for this node. H_PB_W ∈ ℝ⁶ˣⁿᵐ with nm ∈ [0; 6] the
      // number of mobilities for this node. Therefore, the return is a
      // MatrixUpTo6 since the number of columns generally changes with the
      // node.  It is returned as an Eigen::Map to the memory allocated in the
      // std::vector H_PB_W_cache so that we can work with H_PB_W as with any
      // other Eigen matrix object.
      Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
          node.GetJacobianFromArray(H_PB_W_cache);

      // Update per-node kinematics.
      node.CalcVelocityKinematicsCache_BaseToTip(context, pc, H_PB_W, vc);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcSpatialInertiasInWorld(
    const systems::Context<T>& context,
    std::vector<SpatialInertia<T>>* M_B_W_all) const {
  DRAKE_THROW_UNLESS(M_B_W_all != nullptr);
  DRAKE_THROW_UNLESS(static_cast<int>(M_B_W_all->size()) == num_bodies());

  const PositionKinematicsCache<T>& pc = this->EvalPositionKinematics(context);

  // Skip the world.
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    const RigidTransform<T>& X_WB = pc.get_X_WB(body.node_index());

    // Orientation of B in W.
    const RotationMatrix<T>& R_WB = X_WB.rotation();

    // Spatial inertia of body B about Bo and expressed in the body frame B.
    // This call has zero cost for rigid bodies.
    const SpatialInertia<T> M_B = body.CalcSpatialInertiaInBodyFrame(context);
    // Re-express body B's spatial inertia in the world frame W.
    SpatialInertia<T>& M_B_W = (*M_B_W_all)[body.node_index()];
    M_B_W = M_B.ReExpress(R_WB);
  }
}

template <typename T>
void MultibodyTree<T>::CalcReflectedInertia(
    const systems::Context<T>& context, VectorX<T>* reflected_inertia) const {
  DRAKE_THROW_UNLESS(reflected_inertia != nullptr);
  DRAKE_THROW_UNLESS(static_cast<int>(reflected_inertia->size()) ==
                     num_velocities());

  // See JointActuator::reflected_inertia().
  *reflected_inertia = VectorX<double>::Zero(num_velocities());
  for (const auto& actuator : owned_actuators_) {
    const int joint_velocity_index = actuator->joint().velocity_start();
    (*reflected_inertia)(joint_velocity_index) =
        actuator->calc_reflected_inertia(context);
  }
}

template <typename T>
void MultibodyTree<T>::CalcCompositeBodyInertiasInWorld(
    const systems::Context<T>& context,
    std::vector<SpatialInertia<T>>* Mc_B_W_all) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const std::vector<SpatialInertia<T>>& M_B_W_all =
      EvalSpatialInertiaInWorldCache(context);

  // Perform tip-to-base recursion for each composite body, skipping the world.
  for (int depth = tree_height() - 1; depth > 0; --depth) {
    for (BodyNodeIndex composite_node_index : body_node_levels_[depth]) {
      // Node corresponding to the composite body C.
      const BodyNode<T>& composite_node = *body_nodes_[composite_node_index];

      // This node's spatial inertia.
      const SpatialInertia<T>& M_C_W = M_B_W_all[composite_node_index];

      // Compute the spatial inertia Mc_C_W of the composite body C
      // corresponding to the node with index composite_node_index. Computed
      // about C's origin Co and expressed in the world frame W.
      SpatialInertia<T>& Mc_C_W = (*Mc_B_W_all)[composite_node_index];
      composite_node.CalcCompositeBodyInertia_TipToBase(M_C_W, pc, *Mc_B_W_all,
                                                        &Mc_C_W);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcSpatialAccelerationBias(
    const systems::Context<T>& context,
    std::vector<SpatialAcceleration<T>>* Ab_WB_all)
    const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // This skips the world, body_node_index = 0.
  // For the world body we opted for leaving Ab_WB initialized to NaN so that
  // an accidental usage (most likely indicating unnecessary math) in code would
  // immediately trigger a trail of NaNs that we can track to the source.
  (*Ab_WB_all)[world_index()].SetNaN();
  for (BodyNodeIndex body_node_index(1); body_node_index < num_bodies();
       ++body_node_index) {
    const BodyNode<T>& node = *body_nodes_[body_node_index];
    SpatialAcceleration<T>& Ab_WB = (*Ab_WB_all)[body_node_index];
    node.CalcSpatialAccelerationBias(context, pc, vc, &Ab_WB);
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyForceBias(
  const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* Zb_Bo_W_all) const {
  DRAKE_THROW_UNLESS(Zb_Bo_W_all != nullptr);
  DRAKE_THROW_UNLESS(static_cast<int>(Zb_Bo_W_all->size()) == num_bodies());
  const ArticulatedBodyInertiaCache<T>& abic =
      EvalArticulatedBodyInertiaCache(context);
  const std::vector<SpatialAcceleration<T>>& Ab_WB_cache =
      EvalSpatialAccelerationBiasCache(context);

  // This skips the world, body_node_index = 0.
  // For the world body we opted for leaving Zb_Bo_W initialized to NaN so that
  // an accidental usage (most likely indicating unnecessary math) in code would
  // immediately trigger a trail of NaNs that we can track to the source.
  (*Zb_Bo_W_all)[world_index()].SetNaN();
  for (BodyNodeIndex body_node_index(1); body_node_index < num_bodies();
       ++body_node_index) {
    const ArticulatedBodyInertia<T>& Pplus_PB_W =
        abic.get_Pplus_PB_W(body_node_index);
    const SpatialAcceleration<T>& Ab_WB = Ab_WB_cache[body_node_index];
    SpatialForce<T>& Zb_Bo_W = (*Zb_Bo_W_all)[body_node_index];
    Zb_Bo_W = Pplus_PB_W * Ab_WB;
  }
}

template <typename T>
void MultibodyTree<T>::CalcDynamicBiasForces(
    const systems::Context<T>& context,
    std::vector<SpatialForce<T>>* Fb_Bo_W_all) const {
  DRAKE_THROW_UNLESS(Fb_Bo_W_all != nullptr);
  DRAKE_THROW_UNLESS(static_cast<int>(Fb_Bo_W_all->size()) == num_bodies());

  const std::vector<SpatialInertia<T>>& spatial_inertia_in_world_cache =
      EvalSpatialInertiaInWorldCache(context);

  const VelocityKinematicsCache<T>& vc = this->EvalVelocityKinematics(context);

  // Skip the world.
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);

    const SpatialInertia<T>& M_B_W =
        spatial_inertia_in_world_cache[body.node_index()];

    const T& mass = M_B_W.get_mass();
    // B's center of mass measured in B and expressed in W.
    const Vector3<T>& p_BoBcm_W = M_B_W.get_com();
    // B's unit rotational inertia about Bo, expressed in W.
    const UnitInertia<T>& G_B_W = M_B_W.get_unit_inertia();

    // Gyroscopic spatial force Fb_Bo_W(q, v) on body B about Bo, expressed in
    // W.
    const SpatialVelocity<T>& V_WB = vc.get_V_WB(body.node_index());
    const Vector3<T>& w_WB = V_WB.rotational();
    SpatialForce<T>& Fb_Bo_W = (*Fb_Bo_W_all)[body.node_index()];
    Fb_Bo_W = mass * SpatialForce<T>(
                        w_WB.cross(G_B_W * w_WB), /* rotational */
                        w_WB.cross(w_WB.cross(p_BoBcm_W)) /* translational */);
  }
}

template <typename T>
void MultibodyTree<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>&,
    const VelocityKinematicsCache<T>&,
    const VectorX<T>& known_vdot,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  const bool ignore_velocities = false;
  CalcSpatialAccelerationsFromVdot(context, known_vdot, ignore_velocities,
                                   A_WB_array);
}

template <typename T>
void MultibodyTree<T>::CalcSpatialAccelerationsFromVdot(
    const systems::Context<T>& context,
    const VectorX<T>& known_vdot,
    bool ignore_velocities,
    std::vector<SpatialAcceleration<T>>* A_WB_array) const {
  DRAKE_DEMAND(A_WB_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(A_WB_array->size()) == num_bodies());

  DRAKE_DEMAND(known_vdot.size() == topology_.num_velocities());

  const auto& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>* vc =
      ignore_velocities ? nullptr : &EvalVelocityKinematics(context);

  // TODO(amcastro-tri): Loop over bodies to compute acceleration kinematics
  // updates corresponding to flexible bodies.

  // The world's spatial acceleration is always zero.
  A_WB_array->at(world_index()) = SpatialAcceleration<T>::Zero();

  // Performs a base-to-tip recursion computing body accelerations.
  // This skips the world, depth = 0.
  for (int depth = 1; depth < tree_height(); ++depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.index() == body_node_index);

      // Update per-node kinematics.
      node.CalcSpatialAcceleration_BaseToTip(
          context, pc, vc, known_vdot, A_WB_array);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcAccelerationKinematicsCache(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    const VectorX<T>& known_vdot,
    AccelerationKinematicsCache<T>* ac) const {
  DRAKE_DEMAND(ac != nullptr);
  DRAKE_DEMAND(known_vdot.size() == topology_.num_velocities());

  // TODO(amcastro-tri): Loop over bodies to compute velocity kinematics updates
  // corresponding to flexible bodies.

  std::vector<SpatialAcceleration<T>>& A_WB_array = ac->get_mutable_A_WB_pool();

  CalcSpatialAccelerationsFromVdot(context, pc, vc, known_vdot, &A_WB_array);
}

template <typename T>
VectorX<T> MultibodyTree<T>::CalcInverseDynamics(
    const systems::Context<T>& context,
    const VectorX<T>& known_vdot,
    const MultibodyForces<T>& external_forces) const {
  // Temporary storage used in the computation of inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W(num_bodies());
  VectorX<T> tau(num_velocities());
  CalcInverseDynamics(
      context, known_vdot,
      external_forces.body_forces(), external_forces.generalized_forces(),
      &A_WB, &F_BMo_W, &tau);
  return tau;
}

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

template <typename T>
void MultibodyTree<T>::CalcInverseDynamics(
    const systems::Context<T>& context,
    const VectorX<T>& known_vdot,
    const std::vector<SpatialForce<T>>& Fapplied_Bo_W_array,
    const Eigen::Ref<const VectorX<T>>& tau_applied_array,
    bool ignore_velocities,
    std::vector<SpatialAcceleration<T>>* A_WB_array,
    std::vector<SpatialForce<T>>* F_BMo_W_array,
    EigenPtr<VectorX<T>> tau_array) const {
  DRAKE_DEMAND(known_vdot.size() == num_velocities());
  const int Fapplied_size = static_cast<int>(Fapplied_Bo_W_array.size());
  DRAKE_DEMAND(Fapplied_size == num_bodies() || Fapplied_size == 0);
  const int tau_applied_size = tau_applied_array.size();
  DRAKE_DEMAND(
      tau_applied_size == num_velocities() || tau_applied_size == 0);

  DRAKE_DEMAND(A_WB_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(A_WB_array->size()) == num_bodies());

  DRAKE_DEMAND(F_BMo_W_array != nullptr);
  DRAKE_DEMAND(static_cast<int>(F_BMo_W_array->size()) == num_bodies());

  DRAKE_DEMAND(tau_array->size() == num_velocities());

  // Compute body spatial accelerations given the generalized accelerations are
  // known.
  CalcSpatialAccelerationsFromVdot(context, known_vdot, ignore_velocities,
                                   A_WB_array);

  // Vector of generalized forces per mobilizer.
  // It has zero size if no forces are applied.
  VectorUpTo6<T> tau_applied_mobilizer(0);

  // Spatial force applied on B at Bo.
  // It is left initialized to zero if no forces are applied.
  SpatialForce<T> Fapplied_Bo_W = SpatialForce<T>::Zero();

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  const VectorX<T>& reflected_inertia = EvalReflectedInertiaCache(context);

  // Eval M_Bo_W(q).
  const std::vector<SpatialInertia<T>>& spatial_inertia_in_world_cache =
      EvalSpatialInertiaInWorldCache(context);

  // Eval Fb_Bo_W(q, v). Fb_Bo_W = 0 if v = 0.
  const std::vector<SpatialForce<T>>* dynamic_bias_cache =
      ignore_velocities ? nullptr : &EvalDynamicBiasCache(context);

  // Performs a tip-to-base recursion computing the total spatial force F_BMo_W
  // acting on body B, about point Mo, expressed in the world frame W.
  // This includes the world (depth = 0) so that F_BMo_W_array[world_index()]
  // contains the total force of the bodies connected to the world by a
  // mobilizer.
  for (int depth = tree_height() - 1; depth >= 0; --depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      DRAKE_ASSERT(node.get_topology().level == depth);
      DRAKE_ASSERT(node.index() == body_node_index);

      // Make a copy to the total applied forces since the call to
      // CalcInverseDynamics_TipToBase() below could overwrite the entry for the
      // current body node if the input applied forces arrays are the same
      // in-memory object as the output arrays.
      // This allows users to specify the same input and output arrays if
      // desired to minimize memory footprint.
      // Leave them initialized to zero if no applied forces were provided.
      if (tau_applied_size != 0) {
        tau_applied_mobilizer =
            node.get_mobilizer().get_generalized_forces_from_array(
                tau_applied_array);
      }
      if (Fapplied_size != 0) {
        Fapplied_Bo_W = Fapplied_Bo_W_array[body_node_index];
      }

      // Compute F_BMo_W for the body associated with this node and project it
      // onto the space of generalized forces for the associated mobilizer.
      node.CalcInverseDynamics_TipToBase(
          context, pc, spatial_inertia_in_world_cache, dynamic_bias_cache,
          *A_WB_array, Fapplied_Bo_W, tau_applied_mobilizer, F_BMo_W_array,
          tau_array);
    }
  }

  // Add the effect of reflected inertias.
  // See JointActuator::reflected_inertia().
  for (int i = 0; i < num_velocities(); ++i) {
    (*tau_array)(i) += reflected_inertia(i) * known_vdot(i);
  }
}

template <typename T>
void MultibodyTree<T>::CalcForceElementsContribution(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc,
    MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  DRAKE_DEMAND(forces->CheckHasRightSizeForModel(*this));

  forces->SetZero();
  // Add contributions from force elements.
  for (const auto& force_element : owned_force_elements_) {
    force_element->CalcAndAddForceContribution(context, pc, vc, forces);
  }

  // TODO(amcastro-tri): Remove this call once damping is implemented in terms
  // of force elements.
  AddJointDampingForces(context, forces);
}

template<typename T>
void MultibodyTree<T>::AddJointDampingForces(
    const systems::Context<T>& context, MultibodyForces<T>* forces) const {
  DRAKE_DEMAND(forces != nullptr);
  for (const auto& joint : owned_joints_) {
    joint->AddInDamping(context, forces);
  }
}

template <typename T>
void MultibodyTree<T>::MapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_DEMAND(qdot.size() == num_positions());
  DRAKE_DEMAND(v != nullptr);
  DRAKE_DEMAND(v->size() == num_velocities());

  VectorUpTo6<T> v_mobilizer;
  for (const auto& mobilizer : owned_mobilizers_) {
    const auto qdot_mobilizer = mobilizer->get_positions_from_array(qdot);
    v_mobilizer.resize(mobilizer->num_velocities());
    mobilizer->MapQDotToVelocity(context, qdot_mobilizer, &v_mobilizer);
    mobilizer->get_mutable_velocities_from_array(v) = v_mobilizer;
  }
}

template <typename T>
void MultibodyTree<T>::MapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_DEMAND(v.size() == num_velocities());
  DRAKE_DEMAND(qdot != nullptr);
  DRAKE_DEMAND(qdot->size() == num_positions());

  const int kMaxQdot = 7;
  // qdot_mobilizer is a dynamic sized vector of max size equal to seven.
  Eigen::Matrix<T, Eigen::Dynamic, 1, 0, kMaxQdot, 1> qdot_mobilizer;
  for (const auto& mobilizer : owned_mobilizers_) {
    const auto v_mobilizer = mobilizer->get_velocities_from_array(v);
    DRAKE_DEMAND(mobilizer->num_positions() <= kMaxQdot);
    qdot_mobilizer.resize(mobilizer->num_positions());
    mobilizer->MapVelocityToQDot(context, v_mobilizer, &qdot_mobilizer);
    mobilizer->get_mutable_positions_from_array(qdot) = qdot_mobilizer;
  }
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
  std::vector<SpatialAcceleration<T>> A_WB_array(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(num_bodies());

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
  // - In code we use the monogram notation Mc_C_W to denote the spatial inertia
  //   of composite body C, about it's frame origin Co, and expressed in the
  //   world frame W.
  //
  // - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
  //               algorithms. Springer Science & Business Media, pp. 123-130.

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const std::vector<SpatialInertia<T>>& Mc_B_W_cache =
      EvalCompositeBodyInertiaInWorldCache(context);
  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);
  const VectorX<T>& reflected_inertia = EvalReflectedInertiaCache(context);

  // The algorithm below does not recurse zero entries and therefore these must
  // be set a priori.
  // In addition, we initialize diagonal entries to include the effect of rotor
  // reflected inertia. See JointActuator::reflected_inertia().
  (*M) = reflected_inertia.asDiagonal();

  // Perform tip-to-base recursion for each composite body, skipping the world.
  for (int depth = tree_height() - 1; depth > 0; --depth) {
    for (BodyNodeIndex composite_node_index : body_node_levels_[depth]) {
      // Node corresponding to the composite body C.
      const BodyNode<T>& composite_node = *body_nodes_[composite_node_index];
      const int cnv = composite_node.get_num_mobilizer_velocities();

      if (cnv == 0) continue;  // Weld has no generalized coordinates, so skip.

      // This node's 6x6 composite body inertia.
      const SpatialInertia<T>& Mc_C_W = Mc_B_W_cache[composite_node_index];

      // Across-mobilizer 6 x cnv hinge matrix, from C's parent Cp to C.
      Eigen::Map<const MatrixUpTo6<T>> H_CpC_W =
          composite_node.GetJacobianFromArray(H_PB_W_cache);

      // The composite body algorithm considers the system at rest, when
      // generalized velocities are zero.
      // Now if we consider this node's generalized accelerations as the matrix
      // vm_dot = Iₘ, the identity matrix in ℝᵐˣᵐ, the spatial acceleration A_WC
      // is in ℝ⁶ˣᵐ. That is, we are considering each case in which all
      // generalized accelerations are zero but the m-th generalized
      // acceleration for this node equals one.
      // This node's spatial acceleration can be written as:
      //   A_WC = Φᵀ(p_CpC) * A_WCp + Ac_WC + Ab_CpC_W + H_CpC_W * vm_dot
      // where A_WCp is the spatial acceleration of the parent node's body Cp,
      // Ac_WC include the centrifugal and Coriolis terms, and Ab_CpC_W is the
      // spatial acceleration bias of the hinge Jacobian matrix H_CpC_W.
      // Now, since all generalized accelerations but vm_dot are zero, then
      // A_WCp is zero.  Since the system is at rest, Ac_WC and Ab_CpC_W are
      // zero.
      // Therefore, for vm_dot = Iₘ, we have that A_WC = H_CpC_W.
      const auto& A_WC = H_CpC_W;  // 6 x cnv

      // If we consider the closed system composed of the composite body held by
      // its mobilizer, the Newton-Euler equations state:
      //   Fm_CCo_W = Mc_C_W * A_WC + Fb_C_W
      // where Fm_CCo_W is the spatial force at this node's mobilizer.
      // Since the system is at rest, we have Fb_C_W = 0 and thus:
      const Matrix6xUpTo6<T> Fm_CCo_W = Mc_C_W * A_WC;  // 6 x cnv.

      const int composite_start = composite_node.velocity_start();

      // Diagonal block corresponding to current node (composite_node_index).
      M->block(composite_start, composite_start, cnv, cnv) +=
          H_CpC_W.transpose() * Fm_CCo_W;

      // We recurse the tree inwards from C all the way to the root. We define
      // the frames:
      //  - B:  the frame for the current node, body_node.
      //  - Bc: B's child node frame, child_node.
      //  - P:  B's parent node frame.
      const BodyNode<T>* child_node =
          &composite_node;  // Child starts at frame C.
      const BodyNode<T>* body_node = child_node->parent_body_node();
      Matrix6xUpTo6<T> Fm_CBo_W = Fm_CCo_W;  // 6 x cnv
      while (body_node) {
        const Vector3<T>& p_BcBo_W = -pc.get_p_PoBo_W(child_node->index());
        // In place rigid shift of the spatial force in each column of
        // Fm_CBo_W, from Bc to Bo. Before this computation, Fm_CBo_W actually
        // stores Fm_CBc_W from the previous recursion. At the end of this
        // computation, Fm_CBo_W stores the spatial force on composite body C,
        // shifted to Bo, and expressed in the world W. That is, we are doing
        // Fm_CBo_W = Fm_CBc_W.Shift(p_BcB_W).
        SpatialForce<T>::ShiftInPlace(&Fm_CBo_W, p_BcBo_W);

        // The shift alone is sufficient for a weld joint.
        const int bnv = body_node->get_num_mobilizer_velocities();
        if (bnv > 0) {
          // Across mobilizer 6 x bnv Jacobian between body_node B and
          // its parent P.
          const Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
              body_node->GetJacobianFromArray(H_PB_W_cache);

          // Compute the corresponding bnv x cnv block.
          const MatrixUpTo6<T> HtFm = H_PB_W.transpose() * Fm_CBo_W;
          const int body_start = body_node->velocity_start();
          M->block(body_start, composite_start, bnv, cnv) += HtFm;

          // And copy to its symmetric block.
          M->block(composite_start, body_start, cnv, bnv) += HtFm.transpose();
        }

        child_node = body_node;                      // Update child node Bc.
        body_node = child_node->parent_body_node();  // Update node B.
      }
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcBiasTerm(
    const systems::Context<T>& context, EigenPtr<VectorX<T>> Cv) const {
  DRAKE_DEMAND(Cv != nullptr);
  DRAKE_DEMAND(Cv->rows() == num_velocities());
  DRAKE_DEMAND(Cv->cols() == 1);
  const int nv = num_velocities();
  const VectorX<T> vdot = VectorX<T>::Zero(nv);
  // Auxiliary arrays used by inverse dynamics.
  std::vector<SpatialAcceleration<T>> A_WB_array(num_bodies());
  std::vector<SpatialForce<T>> F_BMo_W_array(num_bodies());
  // TODO(amcastro-tri): provide specific API for when vdot = 0.
  CalcInverseDynamics(context, vdot, {}, VectorX<T>(),
                      &A_WB_array, &F_BMo_W_array, Cv);
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
    const systems::Context<T>& context,
    const Frame<T>& frame_F,
    const Frame<T>& frame_G) const {
  // Shortcut: Efficiently return identity transform if frame_F == frame_G.
  if (frame_F.index() == frame_G.index()) return RigidTransform<T>::Identity();

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const Body<T>& A = frame_F.body();
  const Body<T>& B = frame_G.body();
  const RigidTransform<T>& X_WA = pc.get_X_WB(A.node_index());
  const RigidTransform<T>& X_WB = pc.get_X_WB(B.node_index());
  const RigidTransform<T> X_WF = X_WA * frame_F.CalcPoseInBodyFrame(context);
  const RigidTransform<T> X_WG = X_WB * frame_G.CalcPoseInBodyFrame(context);
  return X_WF.InvertAndCompose(X_WG);  // X_FG = X_FW * X_WG;
}

template <typename T>
RotationMatrix<T> MultibodyTree<T>::CalcRelativeRotationMatrix(
    const systems::Context<T>& context,
    const Frame<T>& frame_F,
    const Frame<T>& frame_G) const {
  // Shortcut: Efficiently return identity matrix if frame_F == frame_G.
  if (frame_F.index() == frame_G.index()) return RotationMatrix<T>::Identity();

  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const Body<T>& A = frame_F.body();
  const Body<T>& B = frame_G.body();
  const RotationMatrix<T>& R_WA = pc.get_R_WB(A.node_index());
  const RotationMatrix<T>& R_WB = pc.get_R_WB(B.node_index());
  const RotationMatrix<T> R_AF = frame_F.CalcRotationMatrixInBodyFrame(context);
  const RotationMatrix<T> R_BG = frame_G.CalcRotationMatrixInBodyFrame(context);
  const RotationMatrix<T> R_WF = R_WA * R_AF;
  const RotationMatrix<T> R_WG = R_WB * R_BG;
  return R_WF.InvertAndCompose(R_WG);  // R_FG = R_FW * R_WG;
}

template <typename T>
void MultibodyTree<T>::CalcPointsPositions(
    const systems::Context<T>& context,
    const Frame<T>& frame_B,
    const Eigen::Ref<const MatrixX<T>>& p_BQi,
    const Frame<T>& frame_A,
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
T MultibodyTree<T>::CalcTotalMass(const systems::Context<T>& context) const {
  T total_mass = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
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
    const Body<T>& body = get_body(body_index);
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
    std::string message = fmt::format("{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.", __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Vector3<T> sum_mi_pi = Vector3<T>::Zero();

  // Sum over all the bodies except the 0th body (which is the world body).
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);

    // total mass = ∑ mᵢ.
    const T& body_mass = body.get_mass(context);
    total_mass += body_mass;

    // sum_mi_pi = ∑ mᵢ * pi_WoBcm_W.
    const Vector3<T> pi_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
    const Vector3<T> pi_WoBcm_W = body.EvalPoseInWorld(context) * pi_BoBcm_B;
    sum_mi_pi += body_mass * pi_WoBcm_W;
  }

  if (total_mass <= 0) {
    std::string message = fmt::format("{}(): The system's total mass must "
                                      "be greater than zero.", __func__);
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
    std::string message = fmt::format("{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.", __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Vector3<T> sum_mi_pi = Vector3<T>::Zero();

  // Sum over all the bodies that are in model_instances except for the 0th body
  // (which is the world body), and count each body's contribution only once.
  // Reminder: Although it is not possible for a body to belong to multiple
  // model instances [as Body::model_instance() returns a body's unique model
  // instance], it is possible for the same model instance to be added multiple
  // times to std::vector<ModelInstanceIndex>& model_instances).  The code below
  // ensures a body's contribution to the sum occurs only once.  Duplicate
  // model_instances in std::vector are considered an upstream user error.
  int number_of_non_world_bodies_processed = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      // total mass = ∑ mᵢ.
      const T& body_mass = body.get_mass(context);
      total_mass += body_mass;
      ++number_of_non_world_bodies_processed;

      // sum_mi_pi = ∑ mᵢ * pi_WoBcm_W.
      const Vector3<T> pi_BoBcm_B = body.CalcCenterOfMassInBodyFrame(context);
      const Vector3<T> pi_WoBcm_W = body.EvalPoseInWorld(context) * pi_BoBcm_B;
      sum_mi_pi += body_mass * pi_WoBcm_W;
    }
  }

  // Throw an exception if there are zero non-world bodies in model_instances.
  if (number_of_non_world_bodies_processed == 0) {
    std::string message = fmt::format("{}(): There must be at least one "
        "non-world body contained in model_instances.", __func__);
    throw std::logic_error(message);
  }

  if (total_mass <= 0) {
    std::string message = fmt::format("{}(): The system's total mass must "
                                      "be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  return sum_mi_pi / total_mass;
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassTranslationalVelocityInWorld(
    const systems::Context<T>& context) const {
  if (num_bodies() <= 1) {
    std::string message = fmt::format("{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.", __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Vector3<T> sum_mi_vi = Vector3<T>::Zero();

  // Sum over all the bodies except the 0th body (which is the world body).
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);

    // total mass = ∑ mᵢ.
    const T& body_mass = body.get_mass(context);
    total_mass += body_mass;

    // sum_mi_vi = ∑ mᵢ * vi_WBcm_W.
    const Vector3<T> vi_WBcm_W =
        body.CalcCenterOfMassTranslationalVelocityInWorld(context);
    sum_mi_vi += body_mass * vi_WBcm_W;
  }

  if (total_mass <= 0) {
    std::string message = fmt::format("{}(): The system's total mass must "
                                      "be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  // For a system S with center of mass Scm, Scm's translational velocity in the
  // world frame W is calculated as v_WScm_W = ∑ (mᵢ vᵢ)  / mₛ, where mₛ = ∑ mᵢ,
  // mᵢ is the mass of the  iᵗʰ body, and vᵢ is Bcm's velocity in world frame W
  // (Bcm is the center of mass of the iᵗʰ body).
  return sum_mi_vi / total_mass;
}

template <typename T>
Vector3<T> MultibodyTree<T>::CalcCenterOfMassTranslationalVelocityInWorld(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances) const {
  // Reminder: MultibodyTree always declares a world body and 2 model instances
  // "world" and "default" so num_model_instances() should always be >= 2.
  if (num_bodies() <= 1) {
    std::string message = fmt::format("{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.", __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Vector3<T> sum_mi_vi = Vector3<T>::Zero();

  // Sum over all the bodies that are in model_instances except for the 0th body
  // (which is the world body), and count each body's contribution only once.
  // Reminder: Although it is not possible for a body to belong to multiple
  // model instances [as Body::model_instance() returns a body's unique model
  // instance], it is possible for the same model instance to be added multiple
  // times to std::vector<ModelInstanceIndex>& model_instances).  The code below
  // ensures a body's contribution to the sum occurs only once.  Duplicate
  // model_instances in std::vector are considered an upstream user error.
  int number_of_non_world_bodies_processed = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      // total mass = ∑ mᵢ.
      const T& body_mass = body.get_mass(context);
      total_mass += body_mass;
      ++number_of_non_world_bodies_processed;

      // sum_mi_vi = ∑ mᵢ * vi_WBcm_W.
      const Vector3<T> vi_WBcm_W =
        body.CalcCenterOfMassTranslationalVelocityInWorld(context);
      sum_mi_vi += body_mass * vi_WBcm_W;
    }
  }

  // Throw an exception if there are zero non-world bodies in model_instances.
  if (number_of_non_world_bodies_processed == 0) {
    std::string message = fmt::format("{}(): There must be at least one "
        "non-world body contained in model_instances.", __func__);
    throw std::logic_error(message);
  }

  if (total_mass <= 0) {
    std::string message = fmt::format("{}(): The system's total mass must "
                                      "be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  return sum_mi_vi / total_mass;
}

template <typename T>
SpatialMomentum<T> MultibodyTree<T>::CalcSpatialMomentumInWorldAboutPoint(
    const systems::Context<T>& context,
    const Vector3<T>& p_WoP_W) const {
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
    if (model_instance >= instance_name_to_index_.size()) {
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
  SpatialMomentum<T> L_WS_W =
      CalcBodiesSpatialMomentumInWorldAboutWo(context, body_indexes);

  // Shift the spatial momentum from Wo to point P.
  return L_WS_W.ShiftInPlace(p_WoP_W);
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
    const BodyNodeIndex body_node_index = get_body(body_index).node_index();
    const SpatialInertia<T>& M_BBo_W = M_Bi_W[body_node_index];
    const SpatialVelocity<T>& V_WBo_W = vc.get_V_WB(body_node_index);
    SpatialMomentum<T> L_WBo_W = M_BBo_W * V_WBo_W;

    // Shift L_WBo_W from about Bo to about Wo and accumulate the sum.
    const RigidTransform<T>& X_WB = pc.get_X_WB(body_node_index);
    const Vector3<T>& p_WoBo_W = X_WB.translation();
    L_WS_W += L_WBo_W.ShiftInPlace(-p_WoBo_W);
  }

  return L_WS_W;
}

template <typename T>
const RigidTransform<T>& MultibodyTree<T>::EvalBodyPoseInWorld(
    const systems::Context<T>& context,
    const Body<T>& body_B) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  body_B.HasThisParentTreeOrThrow(this);
  return EvalPositionKinematics(context).get_X_WB(body_B.node_index());
}

template <typename T>
const SpatialVelocity<T>& MultibodyTree<T>::EvalBodySpatialVelocityInWorld(
    const systems::Context<T>& context,
    const Body<T>& body_B) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  body_B.HasThisParentTreeOrThrow(this);
  return EvalVelocityKinematics(context).get_V_WB(body_B.node_index());
}

template <typename T>
void MultibodyTree<T>::CalcAcrossNodeJacobianWrtVExpressedInWorld(
    const systems::Context<T>& context,
    const PositionKinematicsCache<T>& pc,
    std::vector<Vector6<T>>* H_PB_W_cache) const {
  DRAKE_DEMAND(H_PB_W_cache != nullptr);
  DRAKE_DEMAND(static_cast<int>(H_PB_W_cache->size()) == num_velocities());

  // Quick return on nv = 0. Nothing to compute.
  if (num_velocities() == 0) return;

  for (BodyNodeIndex node_index(1);
       node_index < num_bodies(); ++node_index) {
    const BodyNode<T>& node = *body_nodes_[node_index];

    // The body-node hinge matrix is H_PB_W ∈ ℝ⁶ˣⁿᵐ, with nm ∈ [0; 6] the number
    // of mobilities for this node.
    // Therefore, the return is a MatrixUpTo6 since the number of columns
    // generally changes with the node.  It is returned as an Eigen::Map to the
    // memory allocated in the std::vector H_PB_W_cache so that we can work
    // with H_PB_W as with any other Eigen matrix object.
    Eigen::Map<MatrixUpTo6<T>> H_PB_W =
        node.GetMutableJacobianFromArray(H_PB_W_cache);

    node.CalcAcrossNodeJacobianWrtVExpressedInWorld(context, pc, &H_PB_W);
  }
}

template <typename T>
void MultibodyTree<T>::CalcAllBodyBiasSpatialAccelerationsInWorld(
    const systems::Context<T>& context,
    JacobianWrtVariable with_respect_to,
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
    const systems::Context<T>& context,
    JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E) const {
  // TODO(mitiguy) Allow with_respect_to be JacobianWrtVariable::kQDot.
  DRAKE_THROW_UNLESS(with_respect_to == JacobianWrtVariable::kV);

  // Reserve room to store all the bodies' spatial acceleration bias in world W.
  // TODO(Mitiguy) Inefficient use of heap. Per issue #13560, implement caching.
  std::vector<SpatialAcceleration<T>> AsBias_WB_all(num_bodies());
  CalcAllBodyBiasSpatialAccelerationsInWorld(context, with_respect_to,
                                             &AsBias_WB_all);

  // Frame_B is regarded as fixed/welded to a body, herein named body_B.
  // Extract body_B's spatial acceleration bias in W from AsBias_WB_all.
  const Body<T>& body_B = frame_B.body();
  const SpatialAcceleration<T> AsBias_WBodyB_W =
      AsBias_WB_all[body_B.node_index()];

  // Frame_A is regarded as fixed/welded to a body herein named body_A.
  // Extract body_A's spatial acceleration bias in W from AsBias_WB_all.
  const Body<T>& body_A = frame_A.body();
  const SpatialAcceleration<T> AsBias_WBodyA_W =
      AsBias_WB_all[body_A.node_index()];

  // Calculate Bp's spatial acceleration bias in body_A, expressed in frame_E.
  return CalcSpatialAccelerationHelper(context, frame_B, p_BoBp_B, body_A,
      frame_E, AsBias_WBodyB_W, AsBias_WBodyA_W);
}

template <typename T>
SpatialAcceleration<T> MultibodyTree<T>::CalcSpatialAccelerationHelper(
    const systems::Context<T>& context,
    const Frame<T>& frame_F,
    const Eigen::Ref<const Vector3<T>>& p_FoFp_F,
    const Body<T>& body_A,
    const Frame<T>& frame_E,
    const SpatialAcceleration<T>& A_WB_W,
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
  const SpatialAcceleration<T> A_WFp_W = ShiftSpatialAccelerationInWorld(
      frame_F, p_FoFp_F, A_WB_W, pc, vc);

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
    const SpatialAcceleration<T> A_WAp_W = ShiftSpatialAccelerationInWorld(
        frame_A, p_AoAp_A, A_WA_W, pc, vc);

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
    const Frame<T>& frame_B,
    const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
    const SpatialAcceleration<T>& A_WA_W,
    const PositionKinematicsCache<T>& pc,
    const VelocityKinematicsCache<T>& vc) const {
  // frame_B is fixed/welded to body_A.
  const Body<T>& body_A = frame_B.body();

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
  const RotationMatrix<T>& R_WA = pc.get_R_WB(body_A.node_index());
  const Vector3<T> p_AoBp_W = R_WA * p_AoBp_A;

  // Shift spatial acceleration from body_A to frame_Bp.
  // Note: Since frame_B is assumed to be fixed to body_A, w_WB = w_WA.
  const Vector3<T>& w_WA_W = vc.get_V_WB(body_A.node_index()).rotational();
  SpatialAcceleration<T> A_WBp_W = A_WA_W.Shift(p_AoBp_W, w_WA_W);
  return A_WBp_W;
}

template <typename T>
Matrix3X<T> MultibodyTree<T>::CalcBiasTranslationalAcceleration(
    const systems::Context<T>& context,
    JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Eigen::Ref<const Matrix3X<T>>& p_BoBi_B,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E) const {
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
    const JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Eigen::Ref<const Vector3<T>>& p_BP,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E,
    EigenPtr<MatrixX<T>> Js_V_ABp_E) const {
  DRAKE_THROW_UNLESS(Js_V_ABp_E != nullptr);
  DRAKE_THROW_UNLESS(Js_V_ABp_E->rows() == 6);

  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                           num_positions() : num_velocities();
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
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
      with_respect_to, frame_A,  p_WP,  &Js_w_WAp, &Js_v_WAp);

  Matrix6X<T> Js_V_WBp(6, num_columns);
  auto Js_w_WBp = Js_V_WBp.template topRows<3>();     // rotational part.
  auto Js_v_WBp = Js_V_WBp.template bottomRows<3>();  // translational part.
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
      with_respect_to, frame_B,  p_WP,  &Js_w_WBp, &Js_v_WBp);

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
    const JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E,
    EigenPtr<Matrix3X<T>> Js_w_AB_E) const {
  DRAKE_THROW_UNLESS(Js_w_AB_E != nullptr);
  DRAKE_THROW_UNLESS(Js_w_AB_E->rows() == 3);
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                          num_positions() : num_velocities();
  DRAKE_THROW_UNLESS(Js_w_AB_E->cols() == num_columns);

  // The angular velocity addition theorem, gives w_WB = w_WA + w_AB, where
  // w_WB is frame B's angular velocity in world W,
  // w_WA is frame A's angular velocity in world W, and
  // w_AB is frame B's angular velocity in frame A.
  // Rearrange to calculate B's angular velocity in A as w_AB = w_WB - w_WA.
  // So B's angular velocity Jacobian in A, expressed in frame E is
  // Js_w_AB_E = R_EW * (Js_w_WB_W - Js_w_WA_W).

  // TODO(Mitiguy): When performance becomes an issue, optimize this method by
  // only using the kinematics path from A to B.

  // Create dummy position list for signature requirements of next method.
  const Eigen::Matrix<T, 3, 0> empty_position_list;

  // TODO(Mitiguy) One way to avoid memory allocation and speed this up is to
  // be clever and use the input argument as follows:
  // Eigen::Ref<MatrixX<T>> Js_w_WA_W = *Js_w_AB_E;
  // Also modify CalcJacobianAngularAndOrTranslationalVelocityInWorld() so
  // it can add or subtract to the Jacobian that is passed to it.
  Matrix3X<T> Js_w_WA_W(3, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
      with_respect_to, frame_A, empty_position_list, &Js_w_WA_W, nullptr);

  Matrix3X<T> Js_w_WB_W(3, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
      with_respect_to, frame_B, empty_position_list, &Js_w_WB_W, nullptr);

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
    const JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Eigen::Ref<const Matrix3X<T>>& p_WoBi_W,
    const Frame<T>& frame_A,
    EigenPtr<MatrixX<T>> Js_v_ABi_W) const {
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                          num_positions() : num_velocities();
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
  // only using the kinematics path from A to B.

  // Calculate each point Bi's translational velocity Jacobian in world W.
  // The result is Js_v_WBi_W, but we store into Js_v_ABi_W for performance.
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
    with_respect_to, frame_B, p_WoBi_W, nullptr, Js_v_ABi_W);

  // For the common special case in which frame A is the world W, optimize as
  // Js_v_ABi_W = Js_v_WBi_W
  if (frame_A.index() == world_frame().index() ) return;

  // Calculate each point Ai's translational velocity Jacobian in world W.
  MatrixX<T> Js_v_WAi_W(3 * num_points, num_columns);
  CalcJacobianAngularAndOrTranslationalVelocityInWorld(context,
    with_respect_to, frame_A, p_WoBi_W, nullptr, &Js_v_WAi_W);

  // Calculate each point Bi's translational velocity Jacobian in frame A,
  // expressed in world W. Note, again, that before this line Js_v_ABi_W
  // is actually storing Js_v_WBi_W.
  *Js_v_ABi_W -= Js_v_WAi_W;  // This calculates Js_v_ABi_W.
}

template <typename T>
void MultibodyTree<T>::CalcJacobianTranslationalVelocity(
    const systems::Context<T>& context,
    const JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_B,
    const Frame<T>& frame_F,
    const Eigen::Ref<const Matrix3X<T>>& p_FoBi_F,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E,
    EigenPtr<MatrixX<T>> Js_v_ABi_E) const {
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                          num_positions() : num_velocities();
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
    CalcPointsPositions(context, frame_F, p_FoBi_F,  /* From frame F */
                        world_frame(), &p_WoBi_W);   /* To world frame W */
    CalcJacobianTranslationalVelocityHelper(context, with_respect_to, frame_B,
                                            p_WoBi_W, frame_A, Js_v_ABi_E);
  }

  // If frame_E is not the world frame, re-express Js_v_ABi_W in frame_E as
  // Js_v_ABi_E = R_EW * (Js_v_WBi_W - Js_v_WAi_W).
  if (&frame_E != &frame_W) {
    const RotationMatrix<T> R_EW =
        CalcRelativeRotationMatrix(context, frame_E, frame_W);
    // Extract the 3 x num_columns block that starts at row = 3 * i, column = 0.
    for (int i = 0;  i < num_points; ++i) {
      Js_v_ABi_E->template block<3, Eigen::Dynamic>(3 * i, 0, 3, num_columns) =
      R_EW *
      Js_v_ABi_E->template block<3, Eigen::Dynamic>(3 * i, 0, 3, num_columns);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcJacobianAngularAndOrTranslationalVelocityInWorld(
    const systems::Context<T>& context,
    JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_F,
    const Eigen::Ref<const Matrix3X<T>>& p_WoFpi_W,
    EigenPtr<Matrix3X<T>> Js_w_WF_W,
    EigenPtr<MatrixX<T>> Js_v_WFpi_W) const {
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

  // Body to which frame_F is welded/attached.
  const Body<T>& body_F = frame_F.body();

  // Return zero Jacobians for bodies anchored to the world, since for anchored
  // bodies, w_wF = Js_w_WF * v = 0  and  v_WFpi = Js_v_WFpi * v = 0.
  if (body_F.index() == world_index()) return;

  // Form kinematic path from body_F to the world.
  std::vector<BodyNodeIndex> path_to_world;
  topology_.GetKinematicPathToWorld(body_F.node_index(), &path_to_world);
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);

  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);

  // A statically allocated matrix with a maximum number of rows and columns.
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0, 6, 7> Nplus;

  // For all bodies in the kinematic path from the world to body_F, compute
  // each node's contribution to the Jacobians.
  // Skip the world (ilevel = 0).
  for (size_t ilevel = 1; ilevel < path_to_world.size(); ++ilevel) {
    const BodyNodeIndex body_node_index = path_to_world[ilevel];
    const BodyNode<T>& node = *body_nodes_[body_node_index];
    const BodyNodeTopology& node_topology = node.get_topology();
    const Mobilizer<T>& mobilizer = node.get_mobilizer();
    const int start_index_in_v = node_topology.mobilizer_velocities_start_in_v;
    const int start_index_in_q = node_topology.mobilizer_positions_start;
    const int mobilizer_num_velocities =
        node_topology.num_mobilizer_velocities;
    const int mobilizer_num_positions =
        node_topology.num_mobilizer_positions;

    // "Hinge matrix" H for across-node Jacobian.
    // Herein P designates the inboard (parent) body frame P.
    // B designates the current outboard body in this outward sweep.
    Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
        node.GetJacobianFromArray(H_PB_W_cache);

    // Aliases to angular and translational components in H_PB_W.
    const auto Hw_PB_W = H_PB_W.template topRows<3>();
    const auto Hv_PB_W = H_PB_W.template bottomRows<3>();

    const int start_index = is_wrt_qdot ? start_index_in_q : start_index_in_v;
    const int mobilizer_jacobian_ncols =
        is_wrt_qdot ? mobilizer_num_positions : mobilizer_num_velocities;

    // Mapping defined by v = N⁺(q)⋅q̇.
    if (is_wrt_qdot) {
      // TODO(amcastro-tri): consider using an operator version instead only
      // if/when the computational cost of multiplying with Nplus from the
      // right becomes a bottleneck.
      // TODO(amcastro-tri): cache Nplus to avoid memory allocations.
      Nplus.resize(mobilizer_num_velocities, mobilizer_num_positions);
      mobilizer.CalcNplusMatrix(context, &Nplus);
    }

    // The Jacobian angular velocity term is the same for all points Fpi since
    // all these are points of (fixed/welded to) the same body_F.
    if (Js_w_WF_W) {
      // Get memory address in the output Jacobian angular velocity Js_w_WF_W
      // corresponding to the contribution of the mobilities in level ilevel.
      auto Js_w_PB_W = Js_w_WF_W->block(0, start_index, 3,
                                        mobilizer_jacobian_ncols);
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
      const Vector3<T>& p_WoBo = pc.get_X_WB(node.index()).translation();

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
      }  // ipoint.
    }
  }  // body_node_index
}

template <typename T>
void MultibodyTree<T>::CalcJacobianCenterOfMassTranslationalVelocity(
    const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_A, const Frame<T>& frame_E,
    EigenPtr<Matrix3X<T>> Js_v_ACcm_E) const {

  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                          num_positions() : num_velocities();
  DRAKE_THROW_UNLESS(Js_v_ACcm_E != nullptr);
  DRAKE_THROW_UNLESS(Js_v_ACcm_E->cols() == num_columns);
  if (num_bodies() <= 1) {
    throw std::runtime_error(
        "CalcJacobianCenterOfMassTranslationalVelocity(): this "
        "MultibodyPlant contains only world_body() so its center of mass "
        "is undefined.");
  }

  Js_v_ACcm_E->setZero();
  T composite_mass = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    const Vector3<T> pi_BoBcm = body.CalcCenterOfMassInBodyFrame(context);
    MatrixX<T> Jsi_v_ABcm_E(3, num_columns);
    CalcJacobianTranslationalVelocity(
        context, with_respect_to, body.body_frame(),
        body.body_frame(), pi_BoBcm, frame_A, frame_E, &Jsi_v_ABcm_E);
    const T& body_mass = body.get_mass(context);
    *Js_v_ACcm_E += body_mass * Jsi_v_ABcm_E;
    composite_mass += body_mass;
  }

  if (composite_mass <= 0) {
    throw std::logic_error(
        "CalcJacobianCenterOfMassTranslationalVelocity(): The "
        "system's total mass must be greater than zero.");
  }

  *Js_v_ACcm_E /= composite_mass;
}

template<typename T>
void MultibodyTree<T>::CalcJacobianCenterOfMassTranslationalVelocity(
    const systems::Context<T>& context,
    const std::vector<ModelInstanceIndex>& model_instances,
    JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_A,
    const Frame<T>& frame_E,
    EigenPtr<Matrix3X<T>> Js_v_ACcm_E) const {
  const int num_columns = (with_respect_to == JacobianWrtVariable::kQDot) ?
                          num_positions() : num_velocities();
  DRAKE_THROW_UNLESS(Js_v_ACcm_E != nullptr);
  DRAKE_THROW_UNLESS(Js_v_ACcm_E->cols() == num_columns);

  // Reminder: MultibodyTree always declares a world body.
  if (num_bodies() <= 1) {
    std::string message = fmt::format(
        "{}(): This MultibodyPlant only contains "
        "the world_body() so its center of mass is undefined.",
        __func__);
    throw std::logic_error(message);
  }

  T total_mass = 0;
  Js_v_ACcm_E->setZero();

  // Sum over all bodies contained in model_instances except for the 0th body
  // (which is the world body), and count each body's contribution only once.
  // Reminder: Although it is not possible for a body to belong to multiple
  // model instances [as Body::model_instance() returns a body's unique model
  // instance], it is possible for the same model instance to be added multiple
  // times to std::vector<ModelInstanceIndex>& model_instances).  The code below
  // ensures a body's contribution to the sum occurs only once.  Duplicate
  // model_instances in std::vector are considered an upstream user error.
  int number_of_non_world_bodies_processed = 0;
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    if (std::find(model_instances.begin(), model_instances.end(),
                  body.model_instance()) != model_instances.end()) {
      // total mass = ∑ mᵢ.
      const T& body_mass = body.get_mass(context);
      total_mass += body_mass;
      ++number_of_non_world_bodies_processed;

      // sum_mi_Ji = ∑ (mᵢ Jᵢ), where mᵢ is the mass of the iᵗʰ body and
      // Jᵢ is Bcm's translational velocity Jacobian in frame A, expressed in
      // frame E (Bcm is the center of mass of the iᵗʰ body).
      const Vector3<T> pi_BoBcm = body.CalcCenterOfMassInBodyFrame(context);
      MatrixX<T> Jsi_v_ABcm_E(3, num_columns);
      CalcJacobianTranslationalVelocity(context,
                                        with_respect_to,
                                        body.body_frame(),
                                        body.body_frame(),
                                        pi_BoBcm,
                                        frame_A,
                                        frame_E,
                                        &Jsi_v_ABcm_E);
      *Js_v_ACcm_E += body_mass * Jsi_v_ABcm_E;
    }
  }

  // Throw an exception if there are zero non-world bodies in model_instances.
  if (number_of_non_world_bodies_processed == 0) {
    std::string message = fmt::format("{}(): There must be at least one "
        "non-world body contained in model_instances.",  __func__);
    throw std::logic_error(message);
  }

  if (total_mass <= 0) {
    std::string message = fmt::format("{}(): The system's total mass must "
                                      "be greater than zero.", __func__);
    throw std::logic_error(message);
  }

  /// J𝑠_v_ACcm_ = ∑ (mᵢ Jᵢ) / mₛ, where mₛ = ∑ mᵢ.
  *Js_v_ACcm_E /= total_mass;
}

template <typename T>
Vector3<T>
MultibodyTree<T>::CalcBiasCenterOfMassTranslationalAcceleration(
    const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
    const Frame<T>& frame_A, const Frame<T>& frame_E) const {
  DRAKE_THROW_UNLESS(&frame_A == &world_frame());

  if (num_bodies() <= 1) {
    throw std::runtime_error(
        "CalcBiasCenterOfMassTranslationalAcceleration(): this "
        "MultibodyPlant contains only world_body() so its center of mass "
        "is undefined.");
  }

  T composite_mass = 0;
  Vector3<T> asBias_ACcm_E = Vector3<T>::Zero();
  for (BodyIndex body_index(1); body_index < num_bodies(); ++body_index) {
    const Body<T>& body = get_body(body_index);
    const Vector3<T> pi_BoBcm = body.CalcCenterOfMassInBodyFrame(context);
    const SpatialAcceleration<T> AsBiasi_ACcm_E = CalcBiasSpatialAcceleration(
       context, with_respect_to, body.body_frame(), pi_BoBcm, frame_A, frame_E);
    const T& body_mass = body.get_mass(context);
    asBias_ACcm_E += body_mass * AsBiasi_ACcm_E.translational();
    composite_mass += body_mass;
  }

  if (composite_mass <= 0) {
    throw std::logic_error(
        "CalcBiasCenterOfMassTranslationalAcceleration(): The "
        "system's total mass must be greater than zero.");
  }
  asBias_ACcm_E /= composite_mass;
  return asBias_ACcm_E;
}

template <typename T>
T MultibodyTree<T>::CalcPotentialEnergy(
    const systems::Context<T>& context) const {
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  T potential_energy = 0.0;
  // Add contributions from force elements.
  for (const auto& force_element : owned_force_elements_) {
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
    const BodyNodeIndex node_index = get_body(body_index).node_index();
    const SpatialInertia<T>& M_B_W = M_Bi_W[node_index];
    const SpatialVelocity<T>& V_WB = vc.get_V_WB(node_index);
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
  for (const auto& force_element : owned_force_elements_) {
    conservative_power +=
        force_element->CalcConservativePower(context, pc, vc);
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
  for (const auto& force_element : owned_force_elements_) {
    non_conservative_power +=
        force_element->CalcNonConservativePower(context, pc, vc);
  }

  // TODO(sherm1) Add contributions from joint dampers, joint actuators, force
  //              input ports, contact forces, etc. as enumerated in #12942.

  return non_conservative_power;
}

template <typename T>
void MultibodyTree<T>::ThrowIfFinalized(const char* source_method) const {
  if (topology_is_valid()) {
    throw std::logic_error(
        "Post-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; calls to this method must happen before Finalize().");
  }
}

template <typename T>
void MultibodyTree<T>::ThrowIfNotFinalized(const char* source_method) const {
  if (!topology_is_valid()) {
    throw std::logic_error(
        "Pre-finalize calls to '" + std::string(source_method) + "()' are "
        "not allowed; you must call Finalize() first.");
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyInertiaCache(
    const systems::Context<T>& context,
    ArticulatedBodyInertiaCache<T>* abic) const {
  DRAKE_DEMAND(abic != nullptr);

  // TODO(amcastro-tri): consider combining these to improve memory access
  // pattern into a single position kinematics pass.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);
  const std::vector<SpatialInertia<T>>& spatial_inertia_in_world_cache =
      EvalSpatialInertiaInWorldCache(context);
  const VectorX<T>& reflected_inertia = EvalReflectedInertiaCache(context);

  // Perform tip-to-base recursion, skipping the world.
  for (int depth = tree_height() - 1; depth > 0; --depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      // Get hinge matrix and spatial inertia for this node.
      Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
          node.GetJacobianFromArray(H_PB_W_cache);
      const SpatialInertia<T>& M_B_W =
          spatial_inertia_in_world_cache[body_node_index];

      node.CalcArticulatedBodyInertiaCache_TipToBase(
          context, pc, H_PB_W, M_B_W, reflected_inertia, abic);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyForceCache(
    const systems::Context<T>& context, const MultibodyForces<T>& forces,
    ArticulatedBodyForceCache<T>* aba_force_cache) const {
  DRAKE_DEMAND(aba_force_cache != nullptr);
  DRAKE_DEMAND(forces.CheckHasRightSizeForModel(*this));

  // Get position and velocity kinematics from cache.
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const VelocityKinematicsCache<T>& vc = EvalVelocityKinematics(context);

  // Get configuration dependent articulated body inertia cache.
  const ArticulatedBodyInertiaCache<T>& abic =
      EvalArticulatedBodyInertiaCache(context);

  // Extract generalized forces and body forces.
  const VectorX<T>& generalized_forces = forces.generalized_forces();
  const std::vector<SpatialForce<T>>& body_forces = forces.body_forces();

  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);

  // Eval spatial inertia M_B_W(q) and force bias Fb_B_W(q, v) as they appear on
  // the Newton-Euler equation: M_B_W * A_WB + Fb_B_W = Fapp_B_W.
  const std::vector<SpatialForce<T>>& dynamic_bias_cache =
      EvalDynamicBiasCache(context);

  // We evaluate the kinematics dependent articulated body force bias Zb_Bo_W =
  // Pplus_PB_W * Ab_WB. When cached, this corresponds to a significant
  // computational gain when performing ABA with the same context (storing the
  // same q and v) but different applied `forces`.
  const std::vector<SpatialForce<T>>& Zb_Bo_W_cache =
      EvalArticulatedBodyForceBiasCache(context);

  // Perform tip-to-base recursion, skipping the world.
  for (int depth = tree_height() - 1; depth > 0; --depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      // Get generalized force and body force for this node.
      Eigen::Ref<const VectorX<T>> tau_applied =
          node.get_mobilizer().get_generalized_forces_from_array(
              generalized_forces);
      const SpatialForce<T>& Fapplied_Bo_W = body_forces[body_node_index];

      // Get references to the hinge matrix and force bias for this node.
      Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
          node.GetJacobianFromArray(H_PB_W_cache);
      const SpatialForce<T>& Fb_B_W = dynamic_bias_cache[body_node_index];
      const SpatialForce<T>& Zb_Bo_W = Zb_Bo_W_cache[body_node_index];

      node.CalcArticulatedBodyForceCache_TipToBase(
          context, pc, &vc, Fb_B_W, abic, Zb_Bo_W, Fapplied_Bo_W, tau_applied,
          H_PB_W, aba_force_cache);
    }
  }
}

template <typename T>
void MultibodyTree<T>::CalcArticulatedBodyAccelerations(
    const systems::Context<T>& context,
    const ArticulatedBodyForceCache<T>& aba_force_cache,
    AccelerationKinematicsCache<T>* ac) const {
  DRAKE_DEMAND(ac != nullptr);
  const PositionKinematicsCache<T>& pc = EvalPositionKinematics(context);
  const std::vector<Vector6<T>>& H_PB_W_cache =
      EvalAcrossNodeJacobianWrtVExpressedInWorld(context);
  const ArticulatedBodyInertiaCache<T>& abic =
      EvalArticulatedBodyInertiaCache(context);
  const std::vector<SpatialAcceleration<T>>& Ab_WB_cache =
      EvalSpatialAccelerationBiasCache(context);

  // Perform base-to-tip recursion, skipping the world.
  for (int depth = 1; depth < tree_height(); ++depth) {
    for (BodyNodeIndex body_node_index : body_node_levels_[depth]) {
      const BodyNode<T>& node = *body_nodes_[body_node_index];

      const SpatialAcceleration<T>& Ab_WB = Ab_WB_cache[body_node_index];

      // Get reference to the hinge mapping matrix.
      Eigen::Map<const MatrixUpTo6<T>> H_PB_W =
          node.GetJacobianFromArray(H_PB_W_cache);

      node.CalcArticulatedBodyAccelerations_BaseToTip(
          context, pc, abic, aba_force_cache, H_PB_W, Ab_WB, ac);
    }
  }
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
      throw std::logic_error(
          "Joint named '" + get_joint(joint_index).name() +
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
  MatrixX<double> Sx =
      MatrixX<double>::Zero(num_selected_states, num_states());

  const int nq = num_positions();
  // We place all selected positions first, followed by all the selected
  // velocities, as in the original state x.
  int selected_positions_index = 0;
  int selected_velocities_index = num_selected_positions;
  for (JointIndex joint_index : user_to_joint_index_map) {
    const auto& joint = get_joint(joint_index);

    const int pos_start = joint.position_start();
    const int num_pos = joint.num_positions();
    const int vel_start = joint.velocity_start();
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
    Su(int{actuator_index}, user_index) = 1.0;
    ++user_index;
  }

  return Su;
}

template <typename T>
MatrixX<double> MultibodyTree<T>::MakeActuatorSelectorMatrix(
    const std::vector<JointIndex>& user_to_joint_index_map) const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();

  std::vector<JointActuatorIndex> joint_to_actuator_index(num_joints());
  for (JointActuatorIndex actuator_index(0);
       actuator_index < num_actuators(); ++actuator_index) {
    const auto& actuator = get_joint_actuator(actuator_index);
    joint_to_actuator_index[actuator.joint().index()] = actuator_index;
  }

  // Build a list of actuators in the order given by user_to_joint_index_map,
  // which must contain actuated joints. We verify this.
  std::vector<JointActuatorIndex> user_to_actuator_index_map;
  for (JointIndex joint_index : user_to_joint_index_map) {
    const auto& joint = get_joint(joint_index);

    // If the map has an invalid index then this joint does not have an
    // actuator.
    if (!joint_to_actuator_index[joint_index].is_valid()) {
      throw std::logic_error(
          "Joint '" + joint.name() + "' does not have an actuator.");
    }

    user_to_actuator_index_map.push_back(joint_to_actuator_index[joint_index]);
  }

  return MakeActuatorSelectorMatrix(user_to_actuator_index_map);
}

template <typename T>
VectorX<double> MultibodyTree<T>::GetPositionLowerLimits() const {
  DRAKE_MBT_THROW_IF_NOT_FINALIZED();
  Eigen::VectorXd q_lower = Eigen::VectorXd::Constant(
      num_positions(), -std::numeric_limits<double>::infinity());
  for (JointIndex i{0}; i < num_joints(); ++i) {
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
  for (JointIndex i{0}; i < num_joints(); ++i) {
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
  for (JointIndex i{0}; i < num_joints(); ++i) {
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
  for (JointIndex i{0}; i < num_joints(); ++i) {
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
  for (JointIndex i{0}; i < num_joints(); ++i) {
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
  for (JointIndex i{0}; i < num_joints(); ++i) {
    const auto& joint = get_joint(i);
    vd_upper.segment(joint.velocity_start(), joint.num_velocities()) =
        joint.acceleration_upper_limits();
  }
  return vd_upper;
}

template <typename T>
std::optional<BodyIndex> MultibodyTree<T>::MaybeGetUniqueBaseBodyIndex(
    ModelInstanceIndex model_instance) const {
  DRAKE_THROW_UNLESS(model_instance < instance_name_to_index_.size());
  if (model_instance == world_model_instance()) {
    return std::nullopt;
  }
  std::optional<BodyIndex> base_body_index{};
  for (const auto& body : owned_bodies_) {
    if (body->model_instance() == model_instance &&
        (topology_.get_body(body->index()).parent_body == world_index())) {
      if (base_body_index.has_value()) {
        // More than one base body associated with this model.
        return std::nullopt;
      }
      base_body_index = body->index();
    }
  }
  return base_body_index;
}
}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MultibodyTree)
