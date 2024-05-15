// NOLINTNEXTLINE(build/include): prevent complaint re link_joint_graph.h
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
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
  const BodyIndex world_index = AddLink("world", world_model_instance());
  DRAKE_DEMAND(world_index == BodyIndex(0));
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
    DRAKE_DEMAND(data_.link_composites.empty());
    return;
  }

  // Gut the forest so it doesn't look valid when it's not.
  DRAKE_DEMAND(data_.forest != nullptr);
  data_.forest->Clear();
  data_.forest_is_valid = false;

  // Remove any ephemeral elements from the graph.
  data_.links.erase(data_.links.begin() + data_.num_user_links,
                    data_.links.end());
  data_.joints.erase(data_.joints.begin() + data_.num_user_joints,
                     data_.joints.end());
  data_.loop_constraints.clear();  // All ephemeral.

  data_.ephemeral_link_name_to_index.clear();
  data_.ephemeral_joint_name_to_index.clear();

  // Remove all as-modeled information from the user's graph.
  for (auto& link : data_.links) link.ClearModel(data_.num_user_joints);
  for (auto& joint : data_.joints) joint.ClearModel();
  data_.link_composites.clear();
}

const LinkJointGraph::Link& LinkJointGraph::world_link() const {
  DRAKE_DEMAND(!links().empty());  // World is predefined at construction.
  return links(BodyIndex(0));
}

BodyIndex LinkJointGraph::AddLink(const std::string& link_name,
                                  ModelInstanceIndex model_instance,
                                  LinkFlags flags) {
  DRAKE_DEMAND(model_instance.is_valid());

  // Reject use of world model instance for any Link other than World.
  if (ssize(links()) > 0 && model_instance == world_model_instance()) {
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

  const BodyIndex link_index(ssize(links()));  // next available
  // provide fast name lookup
  data_.link_name_to_index.insert({link_name, link_index});

  data_.links.emplace_back(Link(link_index, link_name, model_instance, flags));
  data_.num_user_links = ssize(links());

  Link& new_link = data_.links.back();

  // These lists allow for efficient forest building but aren't otherwise
  // useful. Note that if a link goes on the static list it must be a base
  // body so we don't add it separately to the must_be_base_body list.
  if (new_link.is_static()) {
    data_.static_links.push_back(link_index);
  } else if (new_link.must_be_base_body()) {
    data_.non_static_must_be_base_body_links.push_back(link_index);
  }

  std::vector<BodyIndex>& links = data_.model_instance_to_links[model_instance];
  links.push_back(link_index);

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
    if (links(it->second).model_instance() == model_instance_index) return true;
  }

  const auto model_range = data_.ephemeral_link_name_to_index.equal_range(name);
  for (auto it = model_range.first; it != model_range.second; ++it) {
    if (links(it->second).model_instance() == model_instance_index) return true;
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
    if (joints(it->second).model_instance() == model_instance_index) {
      return true;
    }
  }

  const auto model_range =
      data_.ephemeral_joint_name_to_index.equal_range(name);
  for (auto it = model_range.first; it != model_range.second; ++it) {
    if (joints(it->second).model_instance() == model_instance_index) {
      return true;
    }
  }

  return false;
}

std::optional<JointIndex> LinkJointGraph::MaybeGetJointBetween(
    BodyIndex link1_index, BodyIndex link2_index) const {
  // Work with the Link that has the fewest joints. (If one of these is World
  // it is probably the other one!)
  const Link& link1 = links(link1_index);
  const Link& link2 = links(link2_index);
  const auto [joint_list_ptr, link_to_look_for] =
      ssize(link1.joints()) <= ssize(link2.joints())
          ? std::make_pair(&link1.joints(), link2_index)
          : std::make_pair(&link2.joints(), link1_index);

  // We're doing a linear search under the assumption that at least one of
  // these Links won't have very many Joints.
  for (JointIndex joint_index : *joint_list_ptr) {
    const Joint& joint = joints(joint_index);
    if (joint.connects(link_to_look_for)) return joint_index;
  }
  return std::nullopt;
}

JointIndex LinkJointGraph::AddJoint(const std::string& name,
                                    ModelInstanceIndex model_instance_index,
                                    const std::string& type,
                                    BodyIndex parent_link_index,
                                    BodyIndex child_link_index,
                                    JointFlags flags) {
  DRAKE_DEMAND(model_instance_index.is_valid());
  DRAKE_DEMAND(parent_link_index.is_valid());
  DRAKE_DEMAND(child_link_index.is_valid());

  if (parent_link_index >= ssize(links())) {
    throw std::range_error(fmt::format(
        "{}(): parent link index {} for joint '{}' is out of range.", __func__,
        parent_link_index, name));
  }
  if (child_link_index >= ssize(links())) {
    throw std::range_error(
        fmt::format("{}(): child link index {} for joint '{}' is out of range.",
                    __func__, child_link_index, name));
  }

  if (parent_link_index == child_link_index) {
    throw std::logic_error(fmt::format(
        "{}(): Joint '{}' (model instance {}) would connect link '{}' "
        "to itself.",
        __func__, name, model_instance_index, links(parent_link_index).name()));
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
  const Link& new_parent = links(parent_link_index);
  const Link& new_child = links(child_link_index);
  const bool is_static = (new_parent.is_world() && link_is_static(new_child)) ||
                         (new_child.is_world() && link_is_static(new_parent));
  if (is_static && type_index != weld_joint_traits_index()) {
    const std::string static_link_name =
        new_parent.is_world() ? new_child.name() : new_parent.name();
    throw std::logic_error(fmt::format(
        "{}(): can't connect static link '{}' to World "
        "using a {} joint; only a weld is permitted. "
        "(Joint '{}' in model instance {}.)",
        __func__, static_link_name, type, name, model_instance_index));
  }

  // We only allow one Joint between any given pair of Links.
  if (std::optional<JointIndex> existing_joint_index =
          MaybeGetJointBetween(parent_link_index, child_link_index)) {
    const Joint& existing_joint = joints(*existing_joint_index);
    const Link& existing_parent = links(existing_joint.parent_link());
    const Link& existing_child = links(existing_joint.child_link());

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

  const JointIndex joint_index(ssize(joints()));  // next available index
  data_.joints.emplace_back(Joint(joint_index, name, model_instance_index,
                                  *type_index, parent_link_index,
                                  child_link_index, flags));
  data_.num_user_joints = ssize(joints());
  data_.joint_name_to_index.insert({name, joint_index});  // fast name lookup

  // Links need to know their joints.
  mutable_link(parent_link_index).add_joint_as_parent(joint_index);
  mutable_link(child_link_index).add_joint_as_child(joint_index);

  return joint_index;
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

void LinkJointGraph::CreateWorldLinkComposite() {
  DRAKE_DEMAND(link_composites().empty() && !links().empty());
  Link& world_link = data_.links[BodyIndex(0)];
  DRAKE_DEMAND(!world_link.link_composite_index_.has_value());
  data_.link_composites.emplace_back(std::vector{BodyIndex(0)});
  world_link.link_composite_index_ = LinkCompositeIndex(0);
}

LoopConstraintIndex LinkJointGraph::AddLoopClosingWeldConstraint(
    BodyIndex primary_link_index, BodyIndex shadow_link_index) {
  DRAKE_DEMAND(primary_link_index.is_valid() && shadow_link_index.is_valid());
  DRAKE_DEMAND(primary_link_index != shadow_link_index);
  Link& primary_link = mutable_link(primary_link_index);
  Link& shadow_link = mutable_link(shadow_link_index);
  DRAKE_DEMAND(primary_link.model_instance() == shadow_link.model_instance());
  const LoopConstraintIndex index(ssize(loop_constraints()));
  // Use the shadow Link's name as the constraint name also.
  data_.loop_constraints.emplace_back(index, shadow_link.name(),
                                      shadow_link.model_instance(),
                                      primary_link_index,  // parent
                                      shadow_link_index);  // child
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

void LinkJointGraph::ChangeLinkFlags(BodyIndex link_index, LinkFlags flags) {
  InvalidateForest();
  mutable_link(link_index).set_flags(flags);
}

void LinkJointGraph::ChangeJointFlags(JointIndex joint_index,
                                      JointFlags flags) {
  InvalidateForest();
  mutable_joint(joint_index).set_flags(flags);
}

void LinkJointGraph::ChangeJointType(JointIndex existing_joint_index,
                                     const std::string& name_of_new_type) {
  DRAKE_DEMAND(existing_joint_index.is_valid() &&
               existing_joint_index < ssize(joints()));
  const std::optional<JointTraitsIndex> new_traits_index =
      GetJointTraitsIndex(name_of_new_type);
  DRAKE_DEMAND(new_traits_index.has_value());

  const Joint& joint = joints(existing_joint_index);

  if (existing_joint_index >= num_user_joints()) {
    throw std::logic_error(
        fmt::format("{}(): can't change the type of ephemeral joint {}; only "
                    "user-defined joints are changeable.",
                    __func__, joint.name()));
  }

  // If this is a joint between a static link and world, it can only be a
  // weld (see AddJoint()).
  const Link& parent_link = links(joint.parent_link());
  const Link& child_link = links(joint.child_link());
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
  mutable_joint(existing_joint_index).traits_index_ = *new_traits_index;
}

JointIndex LinkJointGraph::AddEphemeralJointToWorld(
    JointTraitsIndex type_index, BodyIndex child_link_index) {
  const LinkJointGraph::Link& child = links(child_link_index);
  const JointIndex new_joint_index(ssize(joints()));
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
  data_.joints.emplace_back(Joint(new_joint_index, joint_name, model_instance,
                                  type_index, BodyIndex(0), child_link_index,
                                  JointFlags::kDefault));
  // Links need to know their joints.
  mutable_link(BodyIndex(0)).add_joint_as_parent(new_joint_index);
  mutable_link(child_link_index).add_joint_as_child(new_joint_index);

  return new_joint_index;
}

LinkCompositeIndex LinkJointGraph::AddToLinkComposite(
    BodyIndex maybe_composite_link_index, BodyIndex new_link_index) {
  DRAKE_ASSERT(maybe_composite_link_index.is_valid() &&
               new_link_index.is_valid());
  Link& maybe_composite_link = mutable_link(maybe_composite_link_index);
  Link& new_link = mutable_link(new_link_index);
  DRAKE_DEMAND(!new_link.is_world());

  std::optional<LinkCompositeIndex> existing_composite =
      maybe_composite_link.link_composite_index_;
  if (!existing_composite.has_value()) {
    // We're starting a new LinkComposite. This must be the "active link"
    // for this Composite because we saw it first while building the Forest.
    existing_composite = maybe_composite_link.link_composite_index_ =
        LinkCompositeIndex(ssize(data_.link_composites));
    data_.link_composites.emplace_back(
        std::vector<BodyIndex>{maybe_composite_link_index});
  }
  data_.link_composites[*existing_composite].push_back(new_link_index);
  new_link.link_composite_index_ = existing_composite;

  return *existing_composite;
}

BodyIndex LinkJointGraph::AddShadowLink(BodyIndex primary_link_index,
                                        JointIndex shadow_joint_index,
                                        bool shadow_is_parent) {
  /* Caution: this Link reference will be invalid after the emplace. */
  const Link& primary_link = links(primary_link_index);
  const int shadow_num = primary_link.num_shadows() + 1;
  /* Name should be <primary_name>$<shadow_num> (unique within primary's model
  instance). In the unlikely event that a user has names like this, we'll keep
  prepending "_" to the body name until this one is unique. Nothing much depends
  on the details of this name. */
  std::string shadow_link_name =
      fmt::format("{}${}", primary_link.name(), shadow_num);
  while (HasLinkNamed(shadow_link_name, primary_link.model_instance()))
    shadow_link_name = "_" + shadow_link_name;
  const BodyIndex shadow_link_index(ssize(links()));
  DRAKE_DEMAND(shadow_link_index >= num_user_links());  // A sanity check.
  data_.ephemeral_link_name_to_index.insert(
      {shadow_link_name, shadow_link_index});
  data_.links.emplace_back(Link(shadow_link_index, shadow_link_name,
                                primary_link.model_instance(),
                                LinkFlags::kShadow));
  Link& shadow_link = data_.links.back();
  shadow_link.primary_link_ = primary_link_index;
  if (shadow_is_parent) {
    shadow_link.add_joint_as_parent(shadow_joint_index);
  } else {
    shadow_link.add_joint_as_child(shadow_joint_index);
  }
  mutable_link(primary_link_index).shadow_links_.push_back(shadow_link_index);

  return shadow_link.index();
}

void LinkJointGraph::RenumberMobodIndexes(
    const std::vector<MobodIndex>& old_to_new) {
  for (auto& link : data_.links) link.renumber_mobod_indexes(old_to_new);
  for (auto& joint : data_.joints) joint.renumber_mobod_indexes(old_to_new);
}

std::tuple<BodyIndex, BodyIndex, bool> LinkJointGraph::FindInboardOutboardLinks(
    MobodIndex inboard_mobod_index, JointIndex joint_index) const {
  const Joint& joint = joints(joint_index);
  const Link& parent_link = links(joint.parent_link());
  if (parent_link.mobod_index().is_valid() &&
      parent_link.mobod_index() == inboard_mobod_index) {
    return std::make_tuple(joint.parent_link(), joint.child_link(), false);
  }
  const Link& child_link = links(joint.child_link());
  DRAKE_DEMAND(child_link.mobod_index().is_valid() &&
               child_link.mobod_index() == inboard_mobod_index);
  return std::make_tuple(joint.child_link(), joint.parent_link(), true);
}

bool LinkJointGraph::link_is_static(const Link& link) const {
  if (link.is_static()) return true;  // The flag is set.
  return static_cast<bool>(
      get_forest_building_options_in_use(link.model_instance()) &
      ForestBuildingOptions::kStatic);
}

LinkJointGraph::Data::Data() = default;
LinkJointGraph::Data::Data(const Data&) = default;
LinkJointGraph::Data::Data(Data&&) = default;
LinkJointGraph::Data::~Data() = default;
auto LinkJointGraph::Data::operator=(const Data&) -> Data& = default;
auto LinkJointGraph::Data::operator=(Data&&) -> Data& = default;

LinkJointGraph::Link::Link(BodyIndex index, std::string name,
                           ModelInstanceIndex model_instance, LinkFlags flags)
    : index_(index),
      name_(std::move(name)),
      model_instance_(model_instance),
      flags_(flags) {
  DRAKE_DEMAND(index_.is_valid() && !name_.empty() &&
               model_instance_.is_valid());
  // Shadow links overwrite this with their actual primary; everyone else
  // is just a self-primary.
  primary_link_ = index_;
}

void LinkJointGraph::Link::ClearModel(int num_user_joints) {
  DRAKE_DEMAND(!is_shadow());  // Those should already have been removed.
  DRAKE_DEMAND(primary_link_ == index_);  // True for any user link.

  auto remove_ephemeral_joints =
      [num_user_joints](std::vector<JointIndex>& joints) {
        while (!joints.empty() && joints.back() >= num_user_joints)
          joints.pop_back();
      };

  remove_ephemeral_joints(joints_as_parent_);
  remove_ephemeral_joints(joints_as_child_);
  remove_ephemeral_joints(joints_);

  loop_constraints_.clear();  // Always ephemeral.
  mobod_ = {};
  joint_ = {};
  shadow_links_.clear();
  link_composite_index_ = {};
}

LinkJointGraph::Joint::Joint(JointIndex index, std::string name,
                             ModelInstanceIndex model_instance,
                             JointTraitsIndex joint_traits_index,
                             BodyIndex parent_link_index,
                             BodyIndex child_link_index, JointFlags flags)
    : index_(index),
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
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
