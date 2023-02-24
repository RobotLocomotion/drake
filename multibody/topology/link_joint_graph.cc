#include "drake/multibody/topology/link_joint_graph.h"

#include <iostream>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/multibody/topology/spanning_forest_model.h"

namespace drake {
namespace multibody {
namespace internal {

LinkJointGraph::LinkJointGraph() {
  DefaultConstruct();
}

LinkJointGraph::LinkJointGraph(const LinkJointGraph& source)
    : data_(source.data_) {
  data_.model->SetNewOwner(this);
}

LinkJointGraph::LinkJointGraph(LinkJointGraph&& source)
    : data_(std::move(source.data_)) {
  data_.model->SetNewOwner(this);
  source.DefaultConstruct();
}

LinkJointGraph& LinkJointGraph::operator=(const LinkJointGraph& source) {
  if (&source != this) {
    data_ = source.data_;
    data_.model->SetNewOwner(this);
  }
  return *this;
}

LinkJointGraph& LinkJointGraph::operator=(LinkJointGraph&& source) {
  if (&source != this) {
    data_ = std::move(source.data_);
    data_.model->SetNewOwner(this);
    source.DefaultConstruct();
  }
  return *this;
}

void LinkJointGraph::DefaultConstruct() {
  // First clear everything.
  data_ = Data{};

  // Then take actions required by default construction.

  // Joint type names used here must match the kTypeName members of the
  // matching Drake Joint types.
  data_.weld_type_index = RegisterJointType("weld", 0, 0);
  data_.rpy_floating_type_index = RegisterJointType("rpy_floating", 6, 6);
  data_.quaternion_floating_type_index =
      RegisterJointType("quaternion_floating", 7, 6, true);

  // Constructor is private so we can't use make_unique.
  data_.model =
      std::unique_ptr<SpanningForestModel>(new SpanningForestModel(this));
}

const SpanningForestModel& LinkJointGraph::BuildModel(
    ModelingOptions global_options,
    std::map<ModelInstanceIndex, ModelingOptions> instance_options) {
  ClearModel();  // Remove ephemeral Links, Joints, and Constraints.
  data_.model->BuildForest(global_options, instance_options);  // (Re)model
  data_.model_is_valid = true;
  return *data_.model;
}

void LinkJointGraph::ClearModel(bool keep_modeling_additions) {
  if (keep_modeling_additions) {
    // Promote modeling additions to user-supplied.
    data_.num_user_links = ssize(data_.links);
    data_.num_user_joints = ssize(data_.joints);
    data_.num_user_constraints = ssize(data_.constraints);

    data_.link_name_to_index.insert(data_.model_link_name_to_index.begin(),
                                    data_.model_link_name_to_index.end());

    data_.joint_name_to_index.insert(data_.model_joint_name_to_index.begin(),
                                     data_.model_joint_name_to_index.end());
  } else {
    data_.links.erase(data_.links.begin() + data_.num_user_links,
                      data_.links.end());
    data_.joints.erase(data_.joints.begin() + data_.num_user_joints,
                       data_.joints.end());
    data_.constraints.erase(
        data_.constraints.begin() + data_.num_user_constraints,
        data_.constraints.end());
  }

  DRAKE_DEMAND(ssize(data_.link_name_to_index) == data_.num_user_links);
  DRAKE_DEMAND(ssize(data_.joint_name_to_index) == data_.num_user_joints);

  for (auto& link : data_.links) link.clear_model(data_.num_user_joints);
  for (auto& joint : data_.joints) joint.clear_model();

  data_.model_is_valid = false;
  data_.model_link_name_to_index.clear();
  data_.model_joint_name_to_index.clear();
  data_.composite_links.clear();
}

LinkIndex LinkJointGraph::AddLink(const std::string& link_name,
                                  ModelInstanceIndex model_instance,
                                  LinkFlags flags) {
  DRAKE_DEMAND(model_instance.is_valid());

  DRAKE_DEMAND(ssize(links()) > 0 || model_instance == world_model_instance());

  // Reject use of "world" model instance for any Link other than World.
  if (ssize(links()) > 0 && model_instance == world_model_instance()) {
    const std::string msg = fmt::format(
        "AddLink(): Model instance index = {} is reserved for the world Link. "
        " link_index = 0, named '{}'",
        world_model_instance(), world_link().name());
    throw std::runtime_error(msg);
  }

  // Reject duplicate Link name.
  if (HasLinkNamed(link_name, model_instance)) {
    throw std::runtime_error("AddLink(): Duplicate link name '" + link_name +
                             "'");
  }

  // If we have a spanning forest model, it's no good now.
  ClearModel();

  const LinkIndex link_index(ssize(links()));  // next available
  // provide fast name lookup
  data_.link_name_to_index.insert({link_name, link_index});

  data_.links.emplace_back(link_index, link_name, model_instance, flags);
  data_.num_user_links = ssize(links());

  Link& new_link = data_.links.back();

  // These lists allow for efficient forest building but aren't otherwise
  // useful. Note that if a link goes on the static list it must be a base
  // body so we don't add it separately to the must_be_base_body list.
  if (new_link.is_static()) {
    data_.static_links.push_back(link_index);
  } else if (new_link.must_be_base_body()) {
    data_.must_be_base_body_links.push_back(link_index);
  }

  std::vector<LinkIndex>& links = data_.model_instance_to_links[model_instance];
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
    if (links(it->second).model_instance() == model_instance) {
      return true;
    }
  }

  const auto model_range = data_.model_link_name_to_index.equal_range(name);
  for (auto it = model_range.first; it != model_range.second; ++it) {
    if (links(it->second).model_instance() == model_instance) {
      return true;
    }
  }

  return false;
}

bool LinkJointGraph::HasJointNamed(const std::string& name,
                                   ModelInstanceIndex model_instance) const {
  DRAKE_DEMAND(model_instance.is_valid());

  // Search linearly on the assumption that we won't often have lots of
  // joints with the same name in different model instances.  If this turns
  // out to be incorrect we can switch to a different data structure.
  const auto range = data_.joint_name_to_index.equal_range(name);
  for (auto it = range.first; it != range.second; ++it) {
    if (joints(it->second).model_instance() == model_instance) {
      return true;
    }
  }

  const auto model_range = data_.model_joint_name_to_index.equal_range(name);
  for (auto it = model_range.first; it != model_range.second; ++it) {
    if (joints(it->second).model_instance() == model_instance) {
      return true;
    }
  }

  return false;
}

JointIndex LinkJointGraph::MaybeGetJointBetween(LinkIndex link1_index,
                                                LinkIndex link2_index) const {
  // Work with the Link that has the fewest joints. (If one of these is World
  // it is probably the other one!)
  const auto& [joint_list, link_to_look_for] = [&]() {
    const Link& link1 = links(link1_index);
    const Link& link2 = links(link2_index);
    return ssize(link1.joints()) <= ssize(link2.joints())
               ? std::make_pair(link1.joints(), link2_index)
               : std::make_pair(link2.joints(), link1_index);
  }();

  // We're doing a linear search under the assumption that at least one of
  // these links won't have very many Joints.
  for (JointIndex joint_index : joint_list) {
    const Joint& joint = joints(joint_index);
    if (joint.connects(link_to_look_for)) return joint_index;
  }
  return JointIndex{};
}

const LinkJointGraph::Link& LinkJointGraph::world_link() const {
  if (links().empty())
    throw std::runtime_error(
        "world_link(): you can't call this until you have called "
        "AddLink() at least once -- the first link is 'world'.");
  return links(LinkIndex(0));
}

JointIndex LinkJointGraph::AddJoint(const std::string& name,
                                    ModelInstanceIndex model_instance,
                                    const std::string& type,
                                    LinkIndex parent_link_index,
                                    LinkIndex child_link_index,
                                    JointFlags flags) {
  DRAKE_DEMAND(model_instance.is_valid());
  DRAKE_DEMAND(parent_link_index.is_valid() && child_link_index.is_valid());

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
    throw std::logic_error(
        fmt::format("{}(): Can't add a joint from link '{}' to itself.",
                    __func__, links(parent_link_index).name()));
  }

  if (HasJointNamed(name, model_instance)) {
    throw std::logic_error(
        fmt::format("{}(): Duplicate joint name '{}'.", __func__, name));
  }

  const JointTypeIndex type_index = GetJointTypeIndex(type);
  if (!type_index.is_valid()) {
    throw std::logic_error(fmt::format(
        "{}(): Unrecognized type '{}' for joint '{}'.", __func__, type, name));
  }

  // Static Links are implicitly welded to World. We'll permit an explicit
  // joint only if it is a weld.
  const Link& new_parent = links(parent_link_index);
  const Link& new_child = links(child_link_index);
  const bool anchoring = (new_parent.is_world() && new_child.is_static()) ||
                         (new_child.is_world() && new_parent.is_static());
  if (anchoring && type_index != weld_type_index()) {
    const std::string static_link_name =
        new_parent.is_world() ? new_child.name() : new_parent.name();
    throw std::runtime_error(
        fmt::format("{}(): can't connect static link '{}' to World "
                    "using a {} joint; only a weld is permitted.",
                    __func__, static_link_name, type));
  }

  // We only allow one Joint between any given pair of Links.
  if (JointIndex existing_joint_index =
          MaybeGetJointBetween(parent_link_index, child_link_index);
      existing_joint_index.is_valid()) {
    const Joint& existing_joint = joints(existing_joint_index);
    const Link& existing_parent = links(existing_joint.parent_link());
    const Link& existing_child = links(existing_joint.child_link());

    throw std::logic_error(fmt::format(
        "{}(): This LinkJointGraph already has a joint '{}' "
        "connecting '{}' to '{}'. Therefore adding joint '{}' "
        "connecting '{}' to '{}' is not allowed.",
        __func__, existing_joint.name(), existing_parent.name(),
        existing_child.name(), name, new_parent.name(), new_child.name()));
  }

  // If we have a spanning forest model, it's no good now.
  ClearModel();

  const JointIndex joint_index(ssize(joints()));  // next available index
  data_.joints.emplace_back(joint_index, name, model_instance, type_index,
                       parent_link_index, child_link_index, flags);
  data_.num_user_joints = ssize(joints());
  data_.joint_name_to_index.insert({name, joint_index});  // fast name lookup

  // Links need to know their joints.
  mutable_link(parent_link_index).add_joint_as_parent(joint_index);
  mutable_link(child_link_index).add_joint_as_child(joint_index);

  return joint_index;
}

int LinkJointGraph::num_joint_types() const {
  return ssize(data_.joint_types);
}

JointTypeIndex LinkJointGraph::RegisterJointType(
    const std::string& joint_type_name, int nq, int nv, bool has_quaternion) {
  // Reject duplicate type name.
  const auto it = data_.joint_type_name_to_index.find(joint_type_name);
  if (it != data_.joint_type_name_to_index.end())
    throw std::logic_error(fmt::format("{}(): Duplicate joint type: '{}'.",
                                       __func__, joint_type_name));

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

bool LinkJointGraph::IsJointTypeRegistered(
    const std::string& joint_type_name) const {
  const auto it = data_.joint_type_name_to_index.find(joint_type_name);
  return it != data_.joint_type_name_to_index.end();
}

JointIndex LinkJointGraph::AddModelingJointToWorld(JointTypeIndex type_index,
                                                   LinkIndex child_link_index) {
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
  data_.model_joint_name_to_index.insert({joint_name, new_joint_index});
  data_.joints.emplace_back(new_joint_index, joint_name, model_instance,
                            type_index, LinkIndex(0), child_link_index,
                            JointFlags::Default);
  // Links need to know their joints.
  mutable_link(LinkIndex(0)).add_joint_as_parent(new_joint_index);
  mutable_link(child_link_index).add_joint_as_child(new_joint_index);

  return new_joint_index;
}

CompositeLinkIndex LinkJointGraph::AddToCompositeLink(
    LinkIndex existing_link_index, LinkIndex new_link_index) {
  DRAKE_ASSERT(existing_link_index.is_valid() && new_link_index.is_valid());
  Link& existing_link = data_.links[existing_link_index];
  Link& new_link = data_.links[new_link_index];
  DRAKE_DEMAND(!new_link.is_world());
  CompositeLinkIndex existing_composite = existing_link.composite_link_index_;
  if (!existing_composite.is_valid()) {
    // We're starting a new Composite Link.
    existing_composite = existing_link.composite_link_index_ =
        CompositeLinkIndex(ssize(data_.composite_links));
    data_.composite_links.emplace_back(std::vector{existing_link_index});
  }
  data_.composite_links[existing_composite].push_back(new_link_index);
  new_link.composite_link_index_ = existing_composite;

  return existing_composite;
}

void LinkJointGraph::AddUnmodeledJointToComposite(
    JointIndex unmodeled_joint_index, CompositeLinkIndex composite_link_index) {
  Joint& joint = data_.joints[unmodeled_joint_index];
  DRAKE_DEMAND(joint.type_index() == weld_type_index());
  joint.how_modeled_ = composite_link_index;
}

JointTypeIndex LinkJointGraph::GetJointTypeIndex(
    const std::string& joint_type_name) const {
  const auto it = data_.joint_type_name_to_index.find(joint_type_name);
  return it == data_.joint_type_name_to_index.end() ? JointTypeIndex()
                                                    : it->second;
}

/* Runs through the Mobods in the model but records the Link indexes rather than
the Mobod indexes. */
std::vector<LinkIndex> LinkJointGraph::FindPathToWorld(
    LinkIndex link_index) const {
  ThrowIfModelNotBuiltYet(__func__);
  std::vector<LinkIndex> path;
  const SpanningForestModel::Mobod* mobod =
      &model().mobods()[link_to_mobod(link_index)];
  const size_t path_length(mobod->level() + 1);
  path.reserve(path_length);
  path.push_back(link_index);
  while (!mobod->is_world()) {
    mobod = &model().mobods()[mobod->inboard()];
    path.push_back(mobod->link());
  }
  DRAKE_DEMAND(path.size() == path_length);
  return path;
}

LinkIndex LinkJointGraph::FindFirstCommonAncestor(
    LinkIndex link1_index, LinkIndex link2_index) const {
  ThrowIfModelNotBuiltYet(__func__);
  const MobodIndex mobod_ancestor = model().FindFirstCommonAncestor(
      link_to_mobod(link1_index), link_to_mobod(link2_index));
  return model().mobod_to_link(mobod_ancestor);
}

// Note that this algorithm works with LinkJointGraph as an undirected graph.
// We don't care which Link is the parent or child; just that there is a
// weld Joint connecting two Links.
// TODO(sherm1) Reimplement using already-built Composites.
std::vector<std::set<LinkIndex>> LinkJointGraph::FindSubgraphsOfWeldedLinks()
    const {
  std::vector<bool> visited(ssize(links()), false);
  std::vector<std::set<LinkIndex>> subgraphs;

  // Reserve the maximum possible number of subgraphs (that is, when each Link
  // forms its own subgraph) in advance in order to avoid reallocation in the
  // std::vector "subgraphs" which would cause the invalidation of references as
  // we recursively fill it in.
  subgraphs.reserve(ssize(links()));

  // The first Link visited is the "world" (link_index = 0), and therefore
  // subgraphs[0] corresponds to the subgraphs of all Links welded to the
  // world.
  for (const auto& link : links()) {
    if (!visited[link.index()]) {
      // If `link` was not visited yet, we create a subgraph for it.
      subgraphs.push_back(std::set<LinkIndex>{link.index()});

      // We build the subgraph to which `link` belongs by recursively traversing
      // the sub-graph it belongs to.
      std::set<LinkIndex>& link_subgraph = subgraphs.back();

      // Thus far `link` forms its own subgraph. Find if other Links belong to
      // this subgraph by recursively traversing the sub-graph of welded joints
      // connected to `link`.
      FindSubgraphsOfWeldedLinksRecurse(link, &link_subgraph, &subgraphs,
                                        &visited);
    }
  }
  return subgraphs;
}


// As mentioned above, the use of "parent" here does not imply anything about
// the Parent/Child Joint connection. It is really just the root of a subgraph.
// TODO(sherm1) Reimplement using already-built Composites.
void LinkJointGraph::FindSubgraphsOfWeldedLinksRecurse(
    const Link& parent_link, std::set<LinkIndex>* parent_subgraph,
    std::vector<std::set<LinkIndex>>* subgraphs,
    std::vector<bool>* visited) const {
  // Mark parent_link as visited in order to detect loops.
  visited->at(parent_link.index()) = true;

  // Scan each sibling Link.
  for (JointIndex joint_index : parent_link.joints()) {
    const Joint& joint = joints(joint_index);
    const LinkIndex sibling_index = joint.parent_link() == parent_link.index()
                                        ? joint.child_link()
                                        : joint.parent_link();

    // If already visited continue with the next joint.
    if (visited->at(sibling_index)) continue;

    const Link& sibling = links(sibling_index);
    if (joint.type_index() == weld_type_index()) {
      // Welded to parent_link, add it to parent_subgraph.
      parent_subgraph->insert(sibling_index);
      FindSubgraphsOfWeldedLinksRecurse(sibling, parent_subgraph, subgraphs,
                                        visited);
    } else {
      // Disconnected (non-welded) from parent_subgraph. Create its own new
      // subgraph and continue the recursion from "sibling" with its new
      // subgraph "sibling_subgraph".
      subgraphs->push_back(std::set<LinkIndex>{sibling_index});
      std::set<LinkIndex>& sibling_subgraph = subgraphs->back();
      FindSubgraphsOfWeldedLinksRecurse(sibling, &sibling_subgraph, subgraphs,
                                        visited);
    }
  }
}

// TODO(sherm1) Reimplement using already-built Composites.
std::set<LinkIndex> LinkJointGraph::FindLinksWeldedTo(
    LinkIndex link_index) const {
  DRAKE_THROW_UNLESS(link_index.is_valid() && link_index < ssize(links()));

  // TODO(amcastro-tri): Notice that "subgraphs" will get computed with every
  // call to FindLinksWeldedTo(). Consider storing this for subsequent calls if
  // it becomes a performance bottleneck.
  const std::vector<std::set<LinkIndex>> subgraphs =
      FindSubgraphsOfWeldedLinks();

  // Find subgraph that contains this link_index.
  // TODO(amcastro-tri): Consider storing within Link the subgraph it belongs to
  //  if performance becomes an issue.
  auto predicate = [link_index](auto& subgraph) {
    return subgraph.count(link_index) > 0;
  };
  auto subgraph_iter =
      std::find_if(subgraphs.begin(), subgraphs.end(), predicate);

  // If link_index is a valid index to a body in this graph, then it MUST
  // belong to one of the subgraphs. We verify this explicitly.
  DRAKE_DEMAND(subgraph_iter != subgraphs.end());

  return *subgraph_iter;
}

void LinkJointGraph::DumpGraph(std::string title) const {
  std::cout << fmt::format("\n\n**** GRAPH {} ****\n", title);
  std::cout << "JointTypes:\n";
  for (JointTypeIndex i(0); i < ssize(data_.joint_types); ++i) {
    const JointType& type = data_.joint_types[i];
    std::cout << fmt::format("  {}: {} nq={} nv={} has_quat={}\n",
     i, type.type_name, type.nq, type.nv, type.has_quaternion);
  }
  std::cout << "User Links:\n";
  for (const auto& link : links()) {
    if (link.index() == num_user_links()) std::cout << "Model Links:\n";
    std::cout << fmt::format(
        "  {}: {} inst={}{}{}{}{}{}{}",
        link.index(), link.name(), link.model_instance(),
        link.is_world() ? " World" : "",
        link.is_static() ? " static" : "",
        link.is_anchored() ? " anchored" : "",
        link.must_be_base_body() ? " must_be_base" : "",
        link.treat_as_massless() ? " massless" : "",
        link.is_shadow() ? " shadow" : "");
    if (link.mobod_index().is_valid()) {
      std::cout << fmt::format(" mobod={}", link.mobod_index());
    }
    if (link.inboard_joint_index().is_valid()) {
      std::cout << fmt::format(" joint={}", link.inboard_joint_index());
    }
    if (link.composite().is_valid()) {
      std::cout << fmt::format(" composite={}", link.composite());
    }
    std::cout << "\n";

    std::cout << "      joints as_parent=";
    for (const JointIndex& i : link.joints_as_parent()) std::cout << i << " ";
    std::cout << " as_child=";
    for (const JointIndex& i : link.joints_as_child()) std::cout << i << " ";
    if (!link.constraints().empty()) {
      std::cout << " constraints=";
      for (const ConstraintIndex& i : link.constraints()) std::cout << i << " ";
    }
    std::cout << "\n";
  }
  if (ssize(links()) == num_user_links())
    std::cout << "(No added Model Links)\n";

  std::cout << "Composite Links:\n";
  for (CompositeLinkIndex i(0); i < ssize(composite_links()); ++i) {
    std::cout << fmt::format("  {}: ", i);
    for (const LinkIndex& li : composite_links()[i])
      std::cout << fmt::format("{} ", li);
    std::cout << "\n";
  }
  std::cout << "User Joints:\n";
  for (const Joint& joint : joints()) {
    if (joint.index() == num_user_joints())
      std::cout << "Model Joints:\n";
    std::cout << fmt::format("  {}: {} inst={} {} parent={} child={}\n",
                             joint.index(), joint.name(),
                             joint.model_instance(),
                             get_joint_type(joint.type_index()).type_name,
                             joint.parent_link(), joint.child_link());
  }
  if (ssize(joints()) == num_user_joints())
    std::cout << "(No added Model Joints)\n";

  std::cout << (num_user_constraints() > 0 ? "User Constraints:"
                                           : "(No User Constraints)") << "\n";
  for (const Constraint& constraint : constraints()) {
    if (constraint.index() == num_user_constraints())
      std::cout << "Model Constraints:\n";
    std::cout << fmt::format("  {}: {} inst={} parent={} child={}\n",
                             constraint.index(), constraint.name(),
                             constraint.model_instance(),
                             constraint.parent_link(), constraint.child_link());
  }
  if (ssize(joints()) == num_user_joints())
    std::cout << "(No added Model Joints)\n";
}

void LinkJointGraph::ThrowIfModelNotBuiltYet(const char* func) const {
  if (!is_model_valid()) {
    throw std::logic_error(
        fmt::format("{}(): no model available. Call BuildModel() before "
                    "calling this function.", func));
  }
}

void LinkJointGraph::RenumberMobodIndexes(
    const std::vector<MobodIndex>& old_to_new) {
  for (auto& link : data_.links) link.renumber_mobod_indexes(old_to_new);
  for (auto& joint : data_.joints) joint.renumber_mobod_indexes(old_to_new);
}

std::tuple<LinkIndex, LinkIndex, bool> LinkJointGraph::FindInboardOutboardLinks(
    MobodIndex mobod_index, JointIndex joint_index) const {
  const Joint& joint = joints()[joint_index];
  const Link& parent_link = links()[joint.parent_link()];
  if (parent_link.mobod_index().is_valid() &&
      parent_link.mobod_index() == mobod_index) {
    return std::make_tuple(joint.parent_link(), joint.child_link(), false);
  }
  const LinkJointGraph::Link& child_link = links()[joint.child_link()];
  DRAKE_DEMAND(child_link.mobod_index().is_valid() &&
               child_link.mobod_index() == mobod_index);
  return std::make_tuple(joint.child_link(), joint.parent_link(), true);
}

// While modeling, add a shadow link
LinkIndex LinkJointGraph::AddShadowLink(LinkIndex primary_link_index,
                                        JointIndex shadow_joint_index) {
  // Caution: this Link reference will be invalid after the emplace.
  const Link& primary_link = links(primary_link_index);
  const int shadow_num = primary_link.num_shadows() + 1;
  const std::string shadow_link_name =
      fmt::format("{}${}", primary_link.name(), shadow_num);
  // TODO(sherm1) Consider whether to modify name until unique.
  DRAKE_DEMAND(!HasLinkNamed(shadow_link_name, primary_link.model_instance()));
  const LinkIndex shadow_link_index(ssize(links()));
  data_.model_link_name_to_index.insert({shadow_link_name, shadow_link_index});
  data_.links.emplace_back(shadow_link_index, shadow_link_name,
                           primary_link.model_instance(), LinkFlags::Shadow);
  Link& shadow_link = data_.links.back();
  shadow_link.primary_link_ = primary_link_index;
  shadow_link.add_joint_as_child(shadow_joint_index);  // Always a child.
  mutable_link(primary_link_index).shadow_links_.push_back(shadow_link_index);

  return shadow_link.index();
}

ConstraintIndex LinkJointGraph::AddLoopClosingWeldConstraint(
    LinkIndex primary_link_index, LinkIndex shadow_link_index) {
  Link& primary_link = mutable_link(primary_link_index);
  Link& shadow_link = mutable_link(shadow_link_index);
  // Use the shadow Link's name as the constraint name also.
  DRAKE_DEMAND(primary_link.model_instance() == shadow_link.model_instance());
  const ConstraintIndex index(ssize(constraints()));
  data_.constraints.emplace_back(index, shadow_link.name(),
                                 shadow_link.model_instance(),
                                 primary_link_index,  // parent
                                 shadow_link_index);  // child
  primary_link.add_constraint(index);
  shadow_link.add_constraint(index);

  return index;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
