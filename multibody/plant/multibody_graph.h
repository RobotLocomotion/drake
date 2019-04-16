#pragma once

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

/** Type used to identify links by index in a MultibodyGraph. */
using LinkIndex = TypeSafeIndex<class LinkTag>;

/** Type used to identify joint types. */
using JointTypeIndex = TypeSafeIndex<class JointTypeTag>;

/** Defines a multibody graph consisting of links interconnected by joints.
The graph is defined by a sequence of calls to AddLink() and AddJoint(). Anytime
during the lifetime of the graph, a user can ask graph specific questions such
as how links are connected, by which joints or even perform more complex queries
such as what set of bodies are welded together. */
class MultibodyGraph {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyGraph)

  // Local classes.
  class Link;
  class Joint;

  MultibodyGraph();

  /** Add a new Link to the graph.
  @param[in] name
    The unique name of the new link in the particular `model_instance`. Several
    links can have the same name within a %MultibodyGraph however, their name
    within their model instance must be unique.
  @param[in] model_instance
    The model instance to which this link belongs, see @ref model_instance.
  @returns The unique LinkIndex for the added joint in the graph. */
  LinkIndex AddLink(const std::string& name, ModelInstanceIndex model_instance);

  /** Add a new Joint to the graph.
  @param[in] name
    The unique name of the new Joint in the particular `model_instance`. Several
    joints can have the same name within a %MultibodyGraph however, their name
    within their model instance must be unique.
  @param[in] model_instance
    The model instance to which this joint belongs, see @ref model_instance.
  @param[in] type
    A string designating the type of this joint, such as "revolute" or
    "ball". This must be chosen from the set of joint types previously
    registered with calls to RegisterJointType().
  @param[in] parent_link_index
    This must be the index of a link previously obtained with a call to
    AddLink(), or it must be the designated name for the World body
    (world_link_name()).
  @param[in] child_link_index
    This must be the index of a link previously obtained with a call to
    AddLink(), or it must be the designated name for the World body
    (world_link_name()).
  @returns The unique JointIndex for the added joint in the graph. */
  JointIndex AddJoint(const std::string& name,
                      ModelInstanceIndex model_instance,
                      const std::string& type, LinkIndex parent_link_index,
                      LinkIndex child_link_index);

  /** Returns the name we recognize as the World (or Ground) link. This is
  the name that was provided in the first AddLink() call.
  In Drake, MultibodyPlant names it the "WorldBody". */
  const std::string& world_link_name() const;

  /** Returns the model instance index we use for the "world" link. This is
  model instance index that was provided in the first AddLink() call.
  In Drake, MultibodyPlant reserves ModelInstanceIndex(0) for the world, see
  @ref model_instance. */
  ModelInstanceIndex world_model_instance() const;

  /** Returns the link number for the World link (always zero). */
  static LinkIndex world_link_index() { return LinkIndex(0); }

  /** Returns the unique index that identifies the "weld" joint type (always
  zero). */
  static JointTypeIndex weld_type_index() { return JointTypeIndex(0); }

  /** Returns the unique name reserved to identify weld joints (always "weld").
   */
  static std::string weld_type_name() { return "weld"; }

  /** Register a joint type by name.
  MultibodyGraph() registers "weld" at construction as Drake reserves this name
  to identify weld joints. "weld" joint type has index `weld_type_index()`.
  @param[in] joint_type_name
    A unique string identifying a joint type, such as "pin" or "prismatic".
  @throws std::runtime_error if `joint_type_name` already identifies a
  previously registered joint type.
  @retval JointTypeIndex Index uniquely identifying the new joint type. */
  JointTypeIndex RegisterJointType(const std::string& joint_type_name);

  /** @returns `true` iff the given `joint_type_name` was previously registered
  via a call to RegisterJointType(), or iff it equals weld_type_name(). */
  bool IsJointTypeRegistered(const std::string& joint_type_name) const;

  /** Returns the number of registered joint types. */
  int num_joint_types() const;

  /** Returns the number of links, including all added links, and the world
  link.
  @see AddLink(), world_link_index(), world_link_name(). */
  int num_links() const { return static_cast<int>(links_.size()); }

  /** Returns the number joints added with AddJoint(). */
  int num_joints() const { return static_cast<int>(joints_.size()); }

  /** Gets a Link by index. The world link has index world_link_index(). */
  const Link& get_link(LinkIndex index) const {
    DRAKE_THROW_UNLESS(index < num_links());
    return links_[index];
  }

  /** Gets a Joint by index. */
  const Joint& get_joint(JointIndex index) const {
    DRAKE_THROW_UNLESS(index < num_joints());
    return joints_[index];
  }

  /** This method partitions the %MultibodyGraph into islands such that two
  links are in the same island if there is a path between them which
  includes only weld joints (see weld_type_name()).
  Each island of welded links is represented as a set of link indices.
  By definition, these islands will be disconnected by any non-weld
  joint between two bodies. The first island will have all of the links
  welded the world; all subsequent islands will be in no particular order.
  A few more notes:
    - Each link in the %MultibodyGraph is included in one set and one set only.
    - The maximum size of the list equals the number of links in the graph
      (num_links()). This corresponds to a graph with no weld joints.
    - The world link is also included in a set of welded links, and this set is
      element zero in the returned vector.
    - The minimum size of the list is one. This corresponds to a graph with
      all links welded to the world. */
  std::vector<std::set<LinkIndex>> FindIslandsOfWeldedLinks() const;

  /** Returns all links that are transitively welded, or rigidly affixed, to
  `link_index`, per these two definitions:
    1. A link is always considered welded to itself.
    2. Two unique links are considered welded together exclusively by the
       presence of a weld joint, not by other constructs such as constraints.
  Therefore, if `link_index` is a valid index to a Link in this graph, then the
  return vector will always contain at least one entry storing `link_index`. */
  std::set<LinkIndex> FindLinksWeldedTo(LinkIndex link_index) const;

 private:
  // @returns `true` if a body named `name` was added to `model_instance`.
  // @see AddRigidBody().
  // @throws std::exception if `model_instance` is not a valid index.
  // TODO(amcastro-tri): consider making public.
  bool HasLinkNamed(const std::string& name,
                    ModelInstanceIndex model_instance) const;

  // @returns `true` if a joint named `name` was added to `model_instance`.
  // @see AddRigidBody().
  // @throws std::exception if `model_instance` is not a valid index.
  // TODO(amcastro-tri): consider making public.
  bool HasJointNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const;

  // Finds the assigned index for a joint type from the type name. Returns an
  // invalid index if `joint_type_name` was not previously registered with a
  // call to RegisterJointType().
  JointTypeIndex GetJointTypeIndex(const std::string& joint_type_name) const;

  Link& get_mutable_link(LinkIndex link_index) { return links_[link_index]; }

  // Recursive helper method for FindIslandsOfWeldedLinks().
  // The first think this helper does is to mark `parent_link` as "visited" in
  // the output array `visited`.
  // Next, it scans the sibling links connected to `parent_index` by a joint.
  // If the sibling was already visited, it moves on to the next sibling, if
  // any. For a sibling visited for the first time there are two options:
  //   1) if it is connected by a weld joint, it gets added to parent_island.
  //      Recursion continues starting with parent_index = sibling and the
  //      parent island. Otherwise,
  //   2) a new island is created for the sibling and gets added to the
  //      list of all `islands`. Recursion continues starting with parent_index
  //      = sibling and the new island for this sibling.
  void FindIslandsOfWeldedLinksRecurse(
      const Link& parent_link, std::set<LinkIndex>* parent_island,
      std::vector<std::set<LinkIndex>>* islands,
      std::vector<bool>* visited) const;

  // links_ includes the world link at world_link_index() with name
  // world_link_name().
  std::vector<Link> links_;
  std::vector<Joint> joints_;

  std::unordered_map<std::string, JointTypeIndex> joint_type_name_to_index_;

  // The xxx_name_to_index_ structures are multimaps because
  // bodies/joints/actuators/etc may appear with the same name in different
  // model instances. The index values are still unique across the graph.
  std::unordered_multimap<std::string, LinkIndex> link_name_to_index_;
  std::unordered_multimap<std::string, JointIndex> joint_name_to_index_;
};

class MultibodyGraph::Link {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Link)

  LinkIndex index() const { return index_; }

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  /// Returns the number of joints that specify this link as their parent
  /// link. This refers to the input parent/child designation _not_ necessarily
  /// the inboard/outboard relationship in the generated graph.
  int num_children() const;

  /// Returns the number of joints that specify this link as their child
  /// link.
  int num_parents() const;

  /// Returns the total number of joints that specify this body as either
  /// parent or child (this is num_children() + num_parents()). */
  int num_joints() const;

  const std::vector<JointIndex>& joints() const { return joints_; }

 private:
  // Restrict construction and modification to only MultibodyGraph.
  friend class MultibodyGraph;

  Link(LinkIndex index, const std::string& name,
       ModelInstanceIndex model_instance)
      : index_(index), name_(name), model_instance_(model_instance) {}

  // Notes that this link is connected by `joint`.
  void add_joint(JointIndex joint) { joints_.push_back(joint); }

  LinkIndex index_;
  std::string name_;
  ModelInstanceIndex model_instance_;

  // How this link appears in joints (input and added).
  std::vector<JointIndex> joints_as_child_;   // where this link is the child
  std::vector<JointIndex> joints_as_parent_;  // where this link is the parent
  // All joints connecting this link. The union (in no particular order) of
  // joints_as_parent_ and joints_as_child_.
  std::vector<JointIndex> joints_;
};

class MultibodyGraph::Joint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Joint)

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  LinkIndex parent_link() const { return parent_link_index_; }
  LinkIndex child_link() const { return child_link_index_; }

  JointTypeIndex type_index() const { return type_index_; }

 private:
  // Restrict construction and modification to only MultibodyGraph.
  friend class MultibodyGraph;

  Joint(const std::string& name, ModelInstanceIndex model_instance,
        JointTypeIndex joint_type_index, LinkIndex parent_link_index,
        LinkIndex child_link_index)
      : name_(name),
        model_instance_(model_instance),
        type_index_(joint_type_index),
        parent_link_index_(parent_link_index),
        child_link_index_(child_link_index) {}

  std::string name_;
  ModelInstanceIndex model_instance_;
  JointTypeIndex type_index_;
  LinkIndex parent_link_index_;
  LinkIndex child_link_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
