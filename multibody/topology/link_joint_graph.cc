// NOLINTNEXTLINE(build/include): prevent complaint re link_joint_graph.h
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/string_unordered_map.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/topology/graph.h"

namespace drake {
namespace multibody {
namespace internal {

LinkJointGraph::LinkJointGraph() {
  Clear();
}

LinkJointGraph::LinkJointGraph(const LinkJointGraph& source)
    : data_(source.data_) {
  data_.forest->SetNewOwner(this);
}

LinkJointGraph::LinkJointGraph(LinkJointGraph&& source)
    : data_(std::move(source.data_)) {
  data_.forest->SetNewOwner(this);
  source.Clear();
}

LinkJointGraph& LinkJointGraph::operator=(const LinkJointGraph& source) {
  if (&source != this) {
    data_ = source.data_;
    data_.forest->SetNewOwner(this);
  }
  return *this;
}

LinkJointGraph& LinkJointGraph::operator=(LinkJointGraph&& source) {
  if (&source != this) {
    data_ = std::move(source.data_);
    data_.forest->SetNewOwner(this);
    source.Clear();
  }
  return *this;
}

void LinkJointGraph::Clear() {
  // Preserve the SpanningForest if we have one already; we'll strip
  // the contents below.
  SpanningForest* saved_forest = data_.forest.release();

  // Clear everything (marks the forest invalid).
  data_ = Data{};

  if (saved_forest != nullptr) {
    saved_forest->Clear();        // Nothing useful there now.
    data_.forest = saved_forest;  // Keep using the old object.
  } else {
    // Constructor is private so we can't use make_unique.
    data_.forest = std::unique_ptr<SpanningForest>(new SpanningForest(this));
  }

  // Now take actions required by default construction.

  // Joint type names used here must match the kTypeName members of the
  // matching Drake Joint types. Order matters here so we match the
  // predefined joint type indices.
  DRAKE_DEMAND(RegisterJointType("weld", 0, 0) == weld_joint_traits_index());
  DRAKE_DEMAND(RegisterJointType("quaternion_floating", 7, 6, true) ==
               quaternion_floating_joint_traits_index());
  DRAKE_DEMAND(RegisterJointType("rpy_floating", 6, 6) ==
               rpy_floating_joint_traits_index());

  // Define the World Link.
  const LinkIndex world_index = AddLink("world", world_model_instance());
  DRAKE_DEMAND(world_index == LinkIndex(0));
}

void LinkJointGraph::SetGlobalForestBuildingOptions(
    ForestBuildingOptions global_options) {
  InvalidateForest();
  data_.global_forest_building_options = global_options;
}

// Note that we're implicitly assuming that model instance indices will be
// reasonably small integers. They don't have to be contiguous but we're
// allocating an array big enough to hold everything up to the largest index
// we've seen. This enables O(1) access to an element's options during forest
// building.
void LinkJointGraph::SetForestBuildingOptions(ModelInstanceIndex instance_index,
                                              ForestBuildingOptions options) {
  InvalidateForest();
  if (instance_index >= ssize(data_.model_instance_forest_building_options)) {
    data_.model_instance_forest_building_options.resize(instance_index + 1,
                                                        std::nullopt);
  }
  data_.model_instance_forest_building_options[instance_index] = options;
}

void LinkJointGraph::ResetForestBuildingOptions() {
  InvalidateForest();
  data_.global_forest_building_options = ForestBuildingOptions::kDefault;
  data_.model_instance_forest_building_options.clear();
}

bool LinkJointGraph::BuildForest() {
  InvalidateForest();
  const bool dynamics_ok = data_.forest->BuildForest();  // (Re)build
  data_.forest_is_valid = true;
  return dynamics_ok;
}

void LinkJointGraph::InvalidateForest() {
  if (!forest_is_valid()) {
    // Nothing to do but a few sanity checks.
    DRAKE_DEMAND(ssize(data_.link_name_to_index) == data_.num_user_links);
    DRAKE_DEMAND(ssize(data_.joint_name_to_index) == data_.num_user_joints);
    DRAKE_DEMAND(data_.ephemeral_link_name_to_index.empty());
    DRAKE_DEMAND(data_.ephemeral_joint_name_to_index.empty());
    DRAKE_DEMAND(data_.welded_links_assemblies.empty());
    DRAKE_DEMAND(data_.num_user_links == ssize(data_.links));
    DRAKE_DEMAND(data_.num_user_joints == ssize(data_.joints));
    return;
  }

  // Gut the forest so it doesn't look valid when it's not.
  DRAKE_DEMAND(data_.forest != nullptr);
  data_.forest->Clear();
  data_.forest_is_valid = false;

  // Remove any ephemeral elements from the graph. The corresponding indices
  // will appear contiguously at the end of the index_to_ordinal vectors.

  if (ssize(data_.links) > num_user_links()) {
    data_.link_index_to_ordinal.erase(
        data_.link_index_to_ordinal.begin() + (data_.max_user_link_index + 1),
        data_.link_index_to_ordinal.end());
    data_.ephemeral_link_name_to_index.clear();
    data_.links.erase(data_.links.begin() + data_.num_user_links,
                      data_.links.end());
    DRAKE_DEMAND(num_links() == data_.num_user_links);
  }

  if (ssize(data_.joints) > num_user_joints()) {
    data_.joint_index_to_ordinal.erase(
        data_.joint_index_to_ordinal.begin() + (data_.max_user_joint_index + 1),
        data_.joint_index_to_ordinal.end());
    data_.ephemeral_joint_name_to_index.clear();
    data_.joints.erase(data_.joints.begin() + data_.num_user_joints,
                       data_.joints.end());
    DRAKE_DEMAND(ssize(joints()) == data_.num_user_joints);
  }

  data_.loop_constraints.clear();  // All ephemeral.

  // Remove all as-modeled information from the user's graph.
  for (auto& link : data_.links) link.ClearModel(data_.max_user_joint_index);
  for (auto& joint : data_.joints) joint.ClearModel();
  data_.welded_links_assemblies.clear();
}

const LinkJointGraph::Link& LinkJointGraph::world_link() const {
  DRAKE_DEMAND(!links().empty());  // World is predefined at construction.
  return links(LinkOrdinal(0));
}

LinkIndex LinkJointGraph::AddLink(const std::string& link_name,
                                  ModelInstanceIndex model_instance,
                                  LinkFlags flags) {
  DRAKE_DEMAND(model_instance.is_valid());

  // Reject use of world model instance for any Link other than World.
  if (num_links() > 0 && model_instance == world_model_instance()) {
    throw std::logic_error(fmt::format(
        "{}(): Model instance index {} is reserved for the World link. "
        " World is always predefined and is named '{}'.",
        __func__, world_model_instance(), world_link().name()));
  }

  // Reject duplicate Link name.
  if (HasLinkNamed(link_name, model_instance)) {
    throw std::logic_error(
        fmt::format("{}(): There is already a link named '{}' in "
                    "the model instance with index {}.",
                    __func__, link_name, model_instance));
  }

  // Reject addition of a Shadow link this way; that can only be set during
  // forest building.
  if (static_cast<bool>(flags & LinkFlags::kShadow)) {
    throw std::logic_error(
        fmt::format("{}(): Can't add link '{}' with the kShadow flag "
                    "set. That can only be set by BuildForest().",
                    __func__, link_name));
  }

  // If we have a SpanningForest, it's no good now.
  InvalidateForest();

  const LinkIndex link_index(num_link_indexes());
  const LinkOrdinal link_ordinal(num_links());
  data_.link_index_to_ordinal.push_back(link_ordinal);

  // provide fast name lookup
  data_.link_name_to_index.insert({link_name, link_index});

  data_.links.emplace_back(
      Link(link_index, link_ordinal, link_name, model_instance, flags));
  data_.num_user_links = num_links();
  data_.max_user_link_index = link_index;

  Link& new_link = data_.links.back();

  // These lists allow for efficient forest building but aren't otherwise
  // useful. Note that if a link goes on the static list it must be a base
  // body so we don't add it separately to the must_be_base_body list.
  if (new_link.is_static_flag_set()) {
    data_.static_link_indexes.push_back(link_index);
  } else if (new_link.must_be_base_body()) {
    data_.non_static_must_be_base_body_link_indexes.push_back(link_index);
  }

  std::vector<LinkIndex>& links_in_instance =
      data_.model_instance_to_link_indexes[model_instance];
  links_in_instance.push_back(link_index);

  return link_index;
}

bool LinkJointGraph::HasLinkNamed(
    std::string_view name, ModelInstanceIndex model_instance_index) const {
  DRAKE_DEMAND(model_instance_index.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // Links with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = data_.link_name_to_index.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (link_by_index(it->second).model_instance() == model_instance_index)
      return true;
  }

  const auto model_range = data_.ephemeral_link_name_to_index.equal_range(name);
  for (auto it = model_range.first; it != model_range.second; ++it) {
    if (link_by_index(it->second).model_instance() == model_instance_index)
      return true;
  }

  return false;
}

bool LinkJointGraph::HasJointNamed(
    std::string_view name, ModelInstanceIndex model_instance_index) const {
  DRAKE_DEMAND(model_instance_index.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // joints with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = data_.joint_name_to_index.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (joint_by_index(it->second).model_instance() == model_instance_index) {
      return true;
    }
  }

  const auto model_range =
      data_.ephemeral_joint_name_to_index.equal_range(name);
  for (auto it = model_range.first; it != model_range.second; ++it) {
    if (joint_by_index(it->second).model_instance() == model_instance_index) {
      return true;
    }
  }

  return false;
}

std::optional<JointIndex> LinkJointGraph::MaybeGetJointBetween(
    LinkIndex link1_index, LinkIndex link2_index) const {
  // Work with the Link that has the fewest joints. (If one of these is World
  // it is probably the other one!)
  const Link& link1 = link_by_index(link1_index);
  const Link& link2 = link_by_index(link2_index);
  const auto [joint_list_ptr, link_to_look_for] =
      ssize(link1.joints()) <= ssize(link2.joints())
          ? std::make_pair(&link1.joints(), link2_index)
          : std::make_pair(&link2.joints(), link1_index);

  // We're doing a linear search under the assumption that at least one of
  // these Links won't have very many Joints.
  for (JointIndex joint_index : *joint_list_ptr) {
    const Joint& joint = joint_by_index(joint_index);
    if (joint.connects(link_to_look_for)) return joint_index;
  }
  return std::nullopt;
}

JointIndex LinkJointGraph::AddJoint(const std::string& name,
                                    ModelInstanceIndex model_instance_index,
                                    const std::string& type,
                                    LinkIndex parent_link_index,
                                    LinkIndex child_link_index,
                                    JointFlags flags) {
  DRAKE_DEMAND(model_instance_index.is_valid());
  DRAKE_DEMAND(parent_link_index.is_valid());
  DRAKE_DEMAND(child_link_index.is_valid());

  if (!has_link(parent_link_index) || link_is_ephemeral(parent_link_index)) {
    throw std::range_error(fmt::format(
        "{}(): parent link index {} for joint '{}' refers to a non-existent "
        "or ephemeral link.",
        __func__, parent_link_index, name));
  }
  if (!has_link(child_link_index) || link_is_ephemeral(child_link_index)) {
    throw std::range_error(fmt::format(
        "{}(): child link index {} for joint '{}' refers to a non-existent "
        "or ephemeral link.",
        __func__, child_link_index, name));
  }

  if (parent_link_index == child_link_index) {
    throw std::logic_error(fmt::format(
        "{}(): Joint '{}' (model instance {}) would connect link '{}' "
        "to itself.",
        __func__, name, model_instance_index,
        link_by_index(parent_link_index).name()));
  }

  if (HasJointNamed(name, model_instance_index)) {
    throw std::logic_error(
        fmt::format("{}(): There is already a joint named '{}' in "
                    "the model instance with index {}.",
                    __func__, name, model_instance_index));
  }

  const std::optional<JointTraitsIndex> type_index = GetJointTraitsIndex(type);
  if (!type_index.has_value()) {
    throw std::logic_error(fmt::format(
        "{}(): Unrecognized type '{}' for joint '{}' (model instance {}).",
        __func__, type, name, model_instance_index));
  }

  // Static Links are implicitly welded to World. We'll permit an explicit
  // joint only if it is a weld.
  const Link& new_parent = link_by_index(parent_link_index);
  const Link& new_child = link_by_index(child_link_index);
  const bool is_static = (new_parent.is_world() && link_is_static(new_child)) ||
                         (new_child.is_world() && link_is_static(new_parent));
  if (is_static && type_index != weld_joint_traits_index()) {
    const std::string static_link_name =
        new_parent.is_world() ? new_child.name() : new_parent.name();
    throw std::logic_error(fmt::format(
        "{}(): can't connect static link '{}' to World using a {} joint; only "
        "a weld is permitted. (Joint '{}' in model instance {}.)",
        __func__, static_link_name, type, name, model_instance_index));
  }

  // We only allow one Joint between any given pair of Links.
  if (std::optional<JointIndex> existing_joint_index =
          MaybeGetJointBetween(parent_link_index, child_link_index)) {
    const Joint& existing_joint = joint_by_index(*existing_joint_index);
    const Link& existing_parent =
        link_by_index(existing_joint.parent_link_index());
    const Link& existing_child =
        link_by_index(existing_joint.child_link_index());

    throw std::logic_error(fmt::format(
        "{}(): This LinkJointGraph already has joint '{}' (model instance {}) "
        "connecting link '{}' to link '{}'. Therefore adding joint '{}' "
        "(model instance {}) connecting link '{}' to link '{}' is not allowed.",
        __func__, existing_joint.name(), existing_joint.model_instance(),
        existing_parent.name(), existing_child.name(), name,
        model_instance_index, new_parent.name(), new_child.name()));
  }

  // If we have a SpanningForest, it's no good now.
  InvalidateForest();

  const JointIndex joint_index(num_joint_indexes());  // next available index
  const JointOrdinal joint_ordinal(ssize(joints()));
  data_.joint_index_to_ordinal.push_back(joint_ordinal);
  data_.joint_name_to_index.insert({name, joint_index});  // fast name lookup
  data_.joints.emplace_back(Joint(joint_index, joint_ordinal, name,
                                  model_instance_index, *type_index,
                                  parent_link_index, child_link_index, flags));
  data_.num_user_joints = ssize(joints());
  data_.max_user_joint_index = joint_index;

  // Links need to know their joints.
  mutable_link(index_to_ordinal(parent_link_index))
      .add_joint_as_parent(joint_index);
  mutable_link(index_to_ordinal(child_link_index))
      .add_joint_as_child(joint_index);

  return joint_index;
}

void LinkJointGraph::RemoveJoint(JointIndex doomed_joint_index) {
  DRAKE_DEMAND(doomed_joint_index.is_valid());

  if (doomed_joint_index >= ssize(data_.joint_index_to_ordinal)) {
    throw std::logic_error(fmt::format("{}(): Joint index {} is out of range.",
                                       __func__, doomed_joint_index));
  }
  const std::optional<JointOrdinal> doomed_ordinal =
      data_.joint_index_to_ordinal[doomed_joint_index];

  if (!doomed_ordinal.has_value()) {
    throw std::logic_error(
        fmt::format("{}(): The joint that had index {} was already removed.",
                    __func__, doomed_joint_index));
  }

  if (doomed_ordinal >= num_user_joints()) {
    throw std::logic_error(fmt::format(
        "{}(): Joint {} with index {} is an ephemeral joint (added during "
        "forest modeling). You didn't add this joint and you can't remove it. "
        "It will be removed by any change made to the graph that invalidates "
        "the forest.",
        __func__, joints(*doomed_ordinal).name(), doomed_joint_index));
  }

  // We're actually going to remove this joint so the forest is no good now.
  InvalidateForest();

  // Remove references to this Joint in the Links it connected.
  const Joint& doomed_joint = joints(*doomed_ordinal);
  const LinkOrdinal parent_ordinal =
      index_to_ordinal(doomed_joint.parent_link_index());
  const LinkOrdinal child_ordinal =
      index_to_ordinal(doomed_joint.child_link_index());
  mutable_link(parent_ordinal).RemoveJointReferences(doomed_joint_index);
  mutable_link(child_ordinal).RemoveJointReferences(doomed_joint_index);

  // Forget the index and the name, then erase the joint.
  data_.joint_index_to_ordinal[doomed_joint_index] = std::nullopt;
  std::erase_if(data_.joint_name_to_index, [&doomed_joint](const auto& item) {
    const auto& [name, index] = item;
    return index == doomed_joint.index() && name == doomed_joint.name();
  });
  data_.joints.erase(data_.joints.begin() + *doomed_ordinal);

  // Update the ordinals for the joints following this one.
  for (JointOrdinal new_ordinal = *doomed_ordinal;
       new_ordinal < ssize(joints()); ++new_ordinal) {
    Joint& joint = mutable_joint(new_ordinal);
    DRAKE_DEMAND(joint.ordinal() == new_ordinal + 1);
    joint.ordinal_ = new_ordinal;
    data_.joint_index_to_ordinal[joint.index()] = new_ordinal;
  }
  data_.num_user_joints = ssize(joints());
}

JointTraitsIndex LinkJointGraph::RegisterJointType(
    const std::string& joint_type_name, int nq, int nv, bool has_quaternion) {
  // Reject duplicate type name.
  const auto it = data_.joint_type_name_to_index.find(joint_type_name);
  if (it != data_.joint_type_name_to_index.end()) {
    throw std::logic_error(fmt::format("{}(): Duplicate joint type: '{}'.",
                                       __func__, joint_type_name));
  }

  // Sanity check the joint parameters.
  DRAKE_DEMAND(0 <= nq && nq <= 7 && 0 <= nv && nv <= 6 && nv <= nq);
  DRAKE_DEMAND(!has_quaternion || nq >= 4);

  const JointTraitsIndex joint_traits_index(data_.joint_traits.size());
  data_.joint_traits.push_back({.name = joint_type_name,
                                .nq = nq,
                                .nv = nv,
                                .has_quaternion = has_quaternion});
  data_.joint_type_name_to_index[joint_type_name] = joint_traits_index;
  DRAKE_DEMAND(data_.joint_type_name_to_index.size() ==
               data_.joint_traits.size());
  return joint_traits_index;
}

bool LinkJointGraph::IsJointTypeRegistered(
    const std::string& joint_type_name) const {
  const auto it = data_.joint_type_name_to_index.find(joint_type_name);
  return it != data_.joint_type_name_to_index.end();
}

void LinkJointGraph::CreateWorldWeldedLinksAssembly() {
  DRAKE_DEMAND(!links().empty());  // Better be at least World!
  DRAKE_DEMAND(data_.welded_links_assemblies.empty());
  Link& world_link = data_.links[LinkOrdinal(0)];
  DRAKE_DEMAND(!world_link.welded_links_assembly_index_.has_value());
  WeldedLinksAssembly& assembly = data_.welded_links_assemblies.emplace_back();
  assembly.links_.push_back(world_link.index());
  assembly.is_massless_ = false;
  world_link.welded_links_assembly_index_ = WeldedLinksAssemblyIndex(0);
}

LoopConstraintIndex LinkJointGraph::AddLoopClosingWeldConstraint(
    LinkOrdinal primary_link_ordinal, LinkOrdinal shadow_link_ordinal) {
  DRAKE_DEMAND(primary_link_ordinal.is_valid() &&
               shadow_link_ordinal.is_valid());
  DRAKE_DEMAND(primary_link_ordinal != shadow_link_ordinal);
  Link& primary_link = mutable_link(primary_link_ordinal);
  Link& shadow_link = mutable_link(shadow_link_ordinal);
  DRAKE_DEMAND(primary_link.model_instance() == shadow_link.model_instance());
  const LoopConstraintIndex index(ssize(loop_constraints()));
  // Use the shadow Link's name as the constraint name also.
  data_.loop_constraints.emplace_back(index, shadow_link.name(),
                                      shadow_link.model_instance(),
                                      primary_link.index(),  // parent
                                      shadow_link.index());  // child
  primary_link.add_loop_constraint(index);
  shadow_link.add_loop_constraint(index);
  return index;
}

std::optional<JointTraitsIndex> LinkJointGraph::GetJointTraitsIndex(
    const std::string& joint_type_name) const {
  const auto it = data_.joint_type_name_to_index.find(joint_type_name);
  if (it == data_.joint_type_name_to_index.end()) return std::nullopt;
  return it->second;
}

void LinkJointGraph::ChangeLinkFlags(LinkIndex link_index, LinkFlags flags) {
  InvalidateForest();
  mutable_link(index_to_ordinal(link_index)).set_flags(flags);
}

void LinkJointGraph::ChangeJointFlags(JointIndex joint_index,
                                      JointFlags flags) {
  InvalidateForest();
  mutable_joint(index_to_ordinal(joint_index)).set_flags(flags);
}

void LinkJointGraph::ChangeJointType(JointIndex existing_joint_index,
                                     const std::string& name_of_new_type) {
  DRAKE_DEMAND(existing_joint_index.is_valid());
  DRAKE_DEMAND(has_joint(existing_joint_index));
  const std::optional<JointTraitsIndex> new_traits_index =
      GetJointTraitsIndex(name_of_new_type);
  DRAKE_DEMAND(new_traits_index.has_value());

  const JointOrdinal joint_ordinal = index_to_ordinal(existing_joint_index);
  const Joint& joint = joints(joint_ordinal);

  if (joint_is_ephemeral(existing_joint_index)) {
    throw std::logic_error(
        fmt::format("{}(): can't change the type of ephemeral joint {}; only "
                    "user-defined joints are changeable.",
                    __func__, joint.name()));
  }

  // If this is a joint between a static link and world, it can only be a
  // weld (see AddJoint()).
  const Link& parent_link = link_by_index(joint.parent_link_index());
  const Link& child_link = link_by_index(joint.child_link_index());
  const bool is_static =
      (parent_link.is_world() && link_is_static(child_link)) ||
      (child_link.is_world() && link_is_static(parent_link));
  if (is_static && new_traits_index != weld_joint_traits_index()) {
    const std::string static_link_name =
        parent_link.is_world() ? child_link.name() : parent_link.name();
    throw std::logic_error(fmt::format(
        "{}(): can't change type of joint {} (in model instance {}) from {} to "
        "{} because it connects static link {} to World; only a weld is "
        "permitted for a static link.",
        __func__, joint.name(), joint.model_instance(),
        joint_traits(joint.traits_index()).name, name_of_new_type,
        static_link_name));
  }

  InvalidateForest();
  mutable_joint(joint_ordinal).traits_index_ = *new_traits_index;
}

JointIndex LinkJointGraph::AddEphemeralJointToWorld(
    JointTraitsIndex traits_index, LinkOrdinal child_link_ordinal) {
  const LinkJointGraph::Link& child = links(child_link_ordinal);
  const JointIndex new_joint_index(num_joint_indexes());
  const JointOrdinal new_joint_ordinal(ssize(joints()));
  const ModelInstanceIndex model_instance = child.model_instance();

  /* We'll try to name the new Joint the same as the base body it mobilizes.
  This can fail if there is already a Joint in this model instance with
  that name (unlikely). In that case we prepend "_" to the body name until
  the name is unique. This must terminate since there are a finite number
  of Joints in the graph. */
  std::string joint_name = child.name();
  while (HasJointNamed(joint_name, model_instance))
    joint_name = "_" + joint_name;

  // TODO(sherm1) Extract this code that is common with AddJoint().
  data_.ephemeral_joint_name_to_index.insert({joint_name, new_joint_index});
  data_.joint_index_to_ordinal.push_back(new_joint_ordinal);
  data_.joints.emplace_back(
      Joint(new_joint_index, new_joint_ordinal, joint_name, model_instance,
            traits_index, LinkIndex(0), child.index(), JointFlags::kDefault));
  // Links need to know their joints.
  mutable_link(LinkOrdinal(0)).add_joint_as_parent(new_joint_index);
  mutable_link(child_link_ordinal).add_joint_as_child(new_joint_index);

  return new_joint_index;
}

WeldedLinksAssemblyIndex LinkJointGraph::AddToWeldedLinksAssembly(
    LinkOrdinal maybe_assembly_link_ordinal, LinkOrdinal new_link_ordinal,
    JointOrdinal weld_joint_ordinal) {
  DRAKE_ASSERT(maybe_assembly_link_ordinal.is_valid() &&
               new_link_ordinal.is_valid());
  Link& maybe_assembly_link = mutable_link(maybe_assembly_link_ordinal);
  Link& new_link = mutable_link(new_link_ordinal);
  DRAKE_DEMAND(!new_link.is_world());

  std::optional<WeldedLinksAssemblyIndex> existing_assembly_index =
      maybe_assembly_link.welded_links_assembly_index_;
  if (!existing_assembly_index.has_value()) {
    // We're starting a new WeldedLinksAssembly. This must be the "active link"
    // for this Assembly because we saw it first while building the Forest.
    existing_assembly_index = maybe_assembly_link.welded_links_assembly_index_ =
        WeldedLinksAssemblyIndex(ssize(data_.welded_links_assemblies));
    WeldedLinksAssembly& assembly =
        data_.welded_links_assemblies.emplace_back();
    assembly.links_.push_back(maybe_assembly_link.index());
    assembly.is_massless_ = maybe_assembly_link.is_massless();
  }

  WeldedLinksAssembly& existing_assembly =
      data_.welded_links_assemblies[*existing_assembly_index];
  existing_assembly.links_.push_back(new_link.index());
  existing_assembly.joints_.push_back(joints(weld_joint_ordinal).index());
  // For the assembly to be massless, _all_ its links must be massless.
  if (!new_link.is_massless()) {
    existing_assembly.is_massless_ = false;
  }
  new_link.welded_links_assembly_index_ = existing_assembly_index;

  return *existing_assembly_index;
}

void LinkJointGraph::NoteUnmodeledJointInWeldedLinksAssembly(
    JointOrdinal unmodeled_joint_ordinal,
    WeldedLinksAssemblyIndex welded_links_assembly_index) {
  Joint& joint = mutable_joint(unmodeled_joint_ordinal);
  DRAKE_DEMAND(joint.is_weld());
  joint.how_modeled_ = welded_links_assembly_index;
}

void LinkJointGraph::AddUnmodeledJointToWeldedLinksAssembly(
    JointOrdinal unmodeled_joint_ordinal,
    WeldedLinksAssemblyIndex welded_links_assembly_index) {
  WeldedLinksAssembly& assembly =
      data_.welded_links_assemblies[welded_links_assembly_index];
  assembly.joints_.push_back(joints(unmodeled_joint_ordinal).index());
  NoteUnmodeledJointInWeldedLinksAssembly(unmodeled_joint_ordinal,
                                          welded_links_assembly_index);
}

LinkOrdinal LinkJointGraph::AddShadowLink(LinkOrdinal primary_link_ordinal,
                                          JointOrdinal shadow_joint_ordinal,
                                          bool shadow_is_parent) {
  /* Caution: this Link reference will be invalid after the emplace. */
  const Link& primary_link = links(primary_link_ordinal);
  const LinkIndex primary_link_index = primary_link.index();
  const int shadow_num = primary_link.num_shadows() + 1;
  /* Name should be <primary_name>$<shadow_num> (unique within primary's model
  instance). In the unlikely event that a user has names like this, we'll keep
  prepending "_" to the link name until this one is unique. Nothing much depends
  on the details of this name. */
  std::string shadow_link_name =
      fmt::format("{}${}", primary_link.name(), shadow_num);
  while (HasLinkNamed(shadow_link_name, primary_link.model_instance()))
    shadow_link_name = "_" + shadow_link_name;
  const LinkIndex shadow_link_index(num_link_indexes());
  const LinkOrdinal shadow_link_ordinal(num_links());
  DRAKE_DEMAND(shadow_link_ordinal >= num_user_links());  // A sanity check.
  data_.link_index_to_ordinal.push_back(shadow_link_ordinal);
  data_.ephemeral_link_name_to_index.insert(
      {shadow_link_name, shadow_link_index});
  data_.links.emplace_back(Link(shadow_link_index, shadow_link_ordinal,
                                shadow_link_name, primary_link.model_instance(),
                                LinkFlags::kShadow));
  /* Caution: primary_link reference is invalid now -- don't use it! */
  Link& shadow_link = data_.links.back();
  shadow_link.primary_link_ = primary_link_index;
  Joint& shadow_joint = mutable_joint(shadow_joint_ordinal);
  if (shadow_is_parent) {
    shadow_link.add_joint_as_parent(shadow_joint.index());
    shadow_joint.set_effective_parent_link(shadow_link_index);
  } else {
    shadow_link.add_joint_as_child(shadow_joint.index());
    shadow_joint.set_effective_child_link(shadow_link_index);
  }

  Link& mutable_primary_link = mutable_link(primary_link_ordinal);
  mutable_primary_link.shadow_links_.push_back(shadow_link_index);
  mutable_primary_link.NoteRetargetedJoint(shadow_joint.index());

  return shadow_link_ordinal;
}

void LinkJointGraph::RenumberMobodIndexes(
    const std::vector<MobodIndex>& old_to_new) {
  for (auto& link : data_.links) link.renumber_mobod_indexes(old_to_new);
  for (auto& joint : data_.joints) joint.renumber_mobod_indexes(old_to_new);
}

std::tuple<LinkOrdinal, LinkOrdinal, bool>
LinkJointGraph::FindInboardOutboardLinks(MobodIndex inboard_mobod_index,
                                         JointOrdinal joint_ordinal) const {
  const Joint& joint = joints(joint_ordinal);
  const LinkOrdinal parent_link_ordinal =
      index_to_ordinal(joint.parent_link_index());
  const LinkOrdinal child_link_ordinal =
      index_to_ordinal(joint.child_link_index());
  const Link& parent_link = links(parent_link_ordinal);
  if (parent_link.mobod_index().is_valid() &&
      parent_link.mobod_index() == inboard_mobod_index) {
    return std::make_tuple(parent_link_ordinal, child_link_ordinal, false);
  }
  const Link& child_link = links(child_link_ordinal);
  DRAKE_DEMAND(child_link.mobod_index().is_valid() &&
               child_link.mobod_index() == inboard_mobod_index);
  return std::make_tuple(child_link_ordinal, parent_link_ordinal, true);
}

bool LinkJointGraph::link_is_static(const Link& link) const {
  if (link.is_static_flag_set()) return true;
  return static_cast<bool>(
      get_forest_building_options_in_use(link.model_instance()) &
      ForestBuildingOptions::kStatic);
}

/* Runs through the Mobods in the model but records the (active) Link
indexes rather than the Mobod indexes. */
std::vector<LinkIndex> LinkJointGraph::FindPathFromWorld(
    LinkIndex link_index) const {
  ThrowIfForestNotBuiltYet(__func__);
  const SpanningForest::Mobod* mobod =
      &forest().mobods()[link_to_mobod(link_index)];
  std::vector<LinkIndex> path(mobod->level() + 1);
  while (mobod->inboard().is_valid()) {
    const Link& link = links(mobod->link_ordinal());
    path[mobod->level()] = link.index();  // Active Link if optimized assembly.
    mobod = &forest().mobods(mobod->inboard());
  }
  DRAKE_DEMAND(mobod->is_world());
  path[0] = LinkIndex(0);
  return path;
}

LinkIndex LinkJointGraph::FindFirstCommonAncestor(LinkIndex link1_index,
                                                  LinkIndex link2_index) const {
  ThrowIfForestNotBuiltYet(__func__);
  const MobodIndex mobod_ancestor = forest().FindFirstCommonAncestor(
      link_to_mobod(link1_index), link_to_mobod(link2_index));
  const Link& ancestor_link =
      links(forest().mobod_to_link_ordinal(mobod_ancestor));
  return ancestor_link.index();
}

std::vector<LinkIndex> LinkJointGraph::FindSubtreeLinks(
    LinkIndex link_index) const {
  ThrowIfForestNotBuiltYet(__func__);
  const MobodIndex root_mobod_index = link_to_mobod(link_index);
  return forest().FindSubtreeLinks(root_mobod_index);
}

// Our welded_links_assemblies collection doesn't include lone Links that aren't
// welded to anything. The return from this function must include every Link,
// with the World link in the first set (even if nothing is welded to it).
std::vector<std::set<LinkIndex>> LinkJointGraph::GetSubgraphsOfWeldedLinks()
    const {
  ThrowIfForestNotBuiltYet(__func__);

  std::vector<std::set<LinkIndex>> subgraphs;

  // First, collect all the precomputed WeldedLinksAssemblies. World is always
  // the first one, even if nothing is welded to it.
  for (const WeldedLinksAssembly& assembly : welded_links_assemblies()) {
    subgraphs.emplace_back(
        std::set<LinkIndex>(assembly.links_.cbegin(), assembly.links_.cend()));
  }

  // Finally, make one-Link subgraphs for Links that aren't in any assembly.
  for (const Link& link : links()) {
    if (link.welded_links_assembly().has_value()) continue;
    subgraphs.emplace_back(std::set<LinkIndex>{link.index()});
  }

  return subgraphs;
}

// Strategy here is to make repeated use of CalcLinksWeldedTo(), separating
// the singleton sets from the actually-welded sets, and then move the
// singletons to the end to match what GetSubgraphsOfWeldedLinks() does.
std::vector<std::set<LinkIndex>> LinkJointGraph::CalcSubgraphsOfWeldedLinks()
    const {
  // Work with ordinals rather than indexes.
  std::vector<bool> visited(num_user_links(), false);

  // World always comes first, even if it is alone.
  std::vector<std::set<LinkIndex>> subgraphs{CalcLinksWeldedTo(LinkIndex(0))};
  for (LinkIndex index : subgraphs[0]) visited[index_to_ordinal(index)] = true;

  std::vector<std::set<LinkIndex>> singletons;
  // If a Forest was already built, there may be shadow links added to
  // the graph -- don't process those here.
  for (LinkOrdinal link_ordinal(1); link_ordinal < num_user_links();
       ++link_ordinal) {
    const Link& link = links(link_ordinal);
    if (link.is_shadow() || visited[link_ordinal]) continue;
    std::set<LinkIndex> welded_links = CalcLinksWeldedTo(link.index());
    for (LinkIndex index : welded_links)
      visited[index_to_ordinal(index)] = true;
    if (ssize(welded_links) == 1) {
      singletons.emplace_back(std::move(welded_links));
    } else {
      subgraphs.emplace_back(std::move(welded_links));
    }
  }

  // Now move all the singletons onto the end of the subgraphs list.
  for (auto& singleton : singletons)
    subgraphs.emplace_back(std::move(singleton));

  return subgraphs;
}

// If the Link isn't part of a WeldedLinksAssembly just return the Link.
// Otherwise, return all the Links in its WeldedLinksAssembly.
std::set<LinkIndex> LinkJointGraph::GetLinksWeldedTo(
    LinkIndex link_index) const {
  ThrowIfForestNotBuiltYet(__func__);
  DRAKE_DEMAND(link_index.is_valid());
  DRAKE_THROW_UNLESS(has_link(link_index));
  const Link& link = link_by_index(link_index);
  const std::optional<WeldedLinksAssemblyIndex> assembly_index =
      link.welded_links_assembly();
  if (!assembly_index.has_value()) return std::set<LinkIndex>{link_index};
  const std::vector<LinkIndex>& welded_links =
      welded_links_assemblies(*assembly_index).links();
  return std::set<LinkIndex>(welded_links.cbegin(), welded_links.cend());
}

// Without a Forest we don't have WeldedLinksAssemblies available so recursively
// chase Weld joints instead.
std::set<LinkIndex> LinkJointGraph::CalcLinksWeldedTo(
    LinkIndex link_index) const {
  std::set<LinkIndex> result;
  AppendLinksWeldedTo(link_index, &result);
  return result;
}

void LinkJointGraph::AppendLinksWeldedTo(LinkIndex link_index,
                                         std::set<LinkIndex>* result) const {
  DRAKE_DEMAND(result != nullptr);
  DRAKE_DEMAND(link_index.is_valid());
  DRAKE_THROW_UNLESS(has_link(link_index));
  DRAKE_DEMAND(!result->contains(link_index));

  const Link& link = link_by_index(link_index);

  // A Link is always considered welded to itself.
  result->insert(link_index);

  // For World we have to look for static links and pretend they are welded to
  // World. (Links might have been explicitly flagged as static or part of a
  // static model instance.)
  if (link.is_world()) {
    for (const Link& maybe_static : links()) {
      if (result->contains(maybe_static.index())) continue;
      if (link_is_static(maybe_static))
        AppendLinksWeldedTo(maybe_static.index(), &*result);
    }
  }

  // Now run through all the actual joints, looking for welds.
  for (auto joint_index : link.joints()) {
    const Joint& joint = joint_by_index(joint_index);
    if (joint.traits_index() != weld_joint_traits_index()) continue;
    const LinkIndex welded_link_index = joint.other_link_index(link_index);
    // Don't reprocess if we already did the other end.
    if (!result->contains(welded_link_index))
      AppendLinksWeldedTo(welded_link_index, &*result);
  }
}

void LinkJointGraph::ThrowIfForestNotBuiltYet(const char* func) const {
  if (!forest_is_valid()) {
    throw std::logic_error(
        fmt::format("{}(): no SpanningForest available. Call BuildForest() "
                    "before calling this function.",
                    func));
  }
}

void LinkJointGraph::ThrowLinkWasRemoved(const char* func,
                                         LinkIndex link_index) const {
  throw std::logic_error(fmt::format(
      "{}(): An attempt was made to access a link with index {} but that "
      "link was removed.",
      func, link_index));
}

void LinkJointGraph::ThrowJointWasRemoved(const char* func,
                                          JointIndex joint_index) const {
  throw std::logic_error(
      fmt::format("{}(): An attempt was made to access a joint with index {} "
                  "but that joint was removed.",
                  func, joint_index));
}

LinkJointGraph::Data::Data() = default;
LinkJointGraph::Data::Data(const Data&) = default;
LinkJointGraph::Data::Data(Data&&) = default;
LinkJointGraph::Data::~Data() = default;
auto LinkJointGraph::Data::operator=(const Data&) -> Data& = default;
auto LinkJointGraph::Data::operator=(Data&&) -> Data& = default;

LinkJointGraph::Link::Link(LinkIndex index, LinkOrdinal ordinal,
                           std::string name, ModelInstanceIndex model_instance,
                           LinkFlags flags)
    : index_(index),
      ordinal_(ordinal),
      name_(std::move(name)),
      model_instance_(model_instance),
      flags_(flags) {
  DRAKE_DEMAND(index_.is_valid() && !name_.empty() &&
               model_instance_.is_valid());
  DRAKE_DEMAND(ordinal_ <= static_cast<int>(index_));
  // Shadow links overwrite this with their actual primary; everyone else
  // is just a self-primary.
  primary_link_ = index_;
}

void LinkJointGraph::Link::ClearModel(JointIndex max_user_joint_index) {
  DRAKE_DEMAND(!is_shadow());  // Those should already have been removed.
  DRAKE_DEMAND(primary_link_ == index_);  // True for any user link.

  auto remove_ephemeral_joints =
      [max_user_joint_index](std::vector<JointIndex>& joints) {
        while (!joints.empty() && joints.back() > max_user_joint_index)
          joints.pop_back();
      };

  remove_ephemeral_joints(joints_as_parent_);
  remove_ephemeral_joints(joints_as_child_);
  remove_ephemeral_joints(joints_);

  loop_constraints_.clear();  // Always ephemeral.
  mobod_ = {};
  joint_ = {};
  shadow_links_.clear();
  joints_moved_to_shadow_links_.clear();
  welded_links_assembly_index_ = {};
}

LinkJointGraph::Joint::Joint(JointIndex index, JointOrdinal ordinal,
                             std::string name,
                             ModelInstanceIndex model_instance,
                             JointTraitsIndex joint_traits_index,
                             LinkIndex parent_link_index,
                             LinkIndex child_link_index, JointFlags flags)
    : index_(index),
      ordinal_(ordinal),
      name_(std::move(name)),
      model_instance_(model_instance),
      flags_(flags),
      traits_index_(joint_traits_index),
      parent_link_index_(parent_link_index),
      child_link_index_(child_link_index) {
  DRAKE_DEMAND(index_.is_valid() && !name_.empty() &&
               model_instance_.is_valid());
  DRAKE_DEMAND(traits_index_.is_valid() && parent_link_index_.is_valid() &&
               child_link_index_.is_valid());
  DRAKE_DEMAND(parent_link_index_ != child_link_index_);
  DRAKE_DEMAND(ordinal_ <= static_cast<int>(index_));
  effective_parent_link_index_ = parent_link_index_;
  effective_child_link_index_ = child_link_index_;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
