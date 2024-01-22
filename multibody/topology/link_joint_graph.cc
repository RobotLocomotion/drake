// NOLINTNEXTLINE(build/include): prevent complaint re link_joint_graph.h
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
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
  DRAKE_DEMAND(RegisterJointType("weld", 0, 0) == weld_joint_type_index());
  DRAKE_DEMAND(RegisterJointType("quaternion_floating", 7, 6, true) ==
               quaternion_floating_joint_type_index());
  DRAKE_DEMAND(RegisterJointType("rpy_floating", 6, 6) ==
               rpy_floating_joint_type_index());

  // Define the World Link.
  const BodyIndex world_index = AddLink("world", world_model_instance());
  DRAKE_DEMAND(world_index == BodyIndex(0));
}

void LinkJointGraph::BuildForest() {
  InvalidateForest();
  data_.forest->BuildForest();  // (Re)build
  data_.forest_is_valid = true;
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

  data_.ephemeral_link_name_to_index.clear();
  data_.ephemeral_joint_name_to_index.clear();

  // Remove all as-modeled information from the graph.
  for (auto& link : data_.links) link.clear_model(data_.num_user_joints);
  for (auto& joint : data_.joints) joint.clear_model();
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

bool LinkJointGraph::HasLinkNamed(const std::string& name,
                                  ModelInstanceIndex model_instance) const {
  DRAKE_DEMAND(model_instance.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // Links with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = data_.link_name_to_index.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (links(it->second).model_instance() == model_instance) return true;
  }

  const auto model_range = data_.ephemeral_link_name_to_index.equal_range(name);
  for (auto it = model_range.first; it != model_range.second; ++it) {
    if (links(it->second).model_instance() == model_instance) return true;
  }

  return false;
}

JointTypeIndex LinkJointGraph::RegisterJointType(
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

  const JointTypeIndex joint_type_index(data_.joint_types.size());
  data_.joint_types.push_back({.type_name = joint_type_name,
                               .nq = nq,
                               .nv = nv,
                               .has_quaternion = has_quaternion});
  data_.joint_type_name_to_index[joint_type_name] = joint_type_index;
  DRAKE_DEMAND(data_.joint_type_name_to_index.size() ==
               data_.joint_types.size());
  return joint_type_index;
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
}

LinkJointGraph::Joint::Joint(JointIndex index, std::string name,
                             ModelInstanceIndex model_instance,
                             JointTypeIndex joint_type_index,
                             BodyIndex parent_link_index,
                             BodyIndex child_link_index, JointFlags flags)
    : index_(index),
      name_(std::move(name)),
      model_instance_(model_instance),
      flags_(flags),
      type_index_(joint_type_index),
      parent_link_index_(parent_link_index),
      child_link_index_(child_link_index) {
  DRAKE_DEMAND(index_.is_valid() && !name_.empty() &&
               model_instance_.is_valid());
  DRAKE_DEMAND(type_index_.is_valid() && parent_link_index_.is_valid() &&
               child_link_index_.is_valid());
  DRAKE_DEMAND(parent_link_index_ != child_link_index_);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
