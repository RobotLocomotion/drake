#pragma once

#include <set>
#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

/** Type used to identify links by index in a MultibodyGraph. */
using LinkIndex = TypeSafeIndex<class LinkTag>;

/** Type used to identify joint types. */
using JointTypeIndex = TypeSafeIndex<class JointTypeTag>;

// Forward declarations.
class Link;
class Joint;

/** A link has a unique name within a given model instance. However, links can
have the same name across model instance.
Therefore a link is uniquely identified given the pair
(model_instance, link_name). This class represents this pair. */
class LinkIdentifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinkIdentifier)

  LinkIdentifier(ModelInstanceIndex model_instance,
                 const std::string& link_name)
      : model_instance_(model_instance), name_(link_name) {}

  ModelInstanceIndex model_instance() const { return model_instance_; }

  const std::string& name() const { return name_; }

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const LinkIdentifier& id) noexcept {
    using drake::hash_append;
    hash_append(hasher, id.model_instance_);
    hash_append(hasher, id.name_);
  }

 private:
  ModelInstanceIndex model_instance_;
  std::string name_;
};

class MultibodyGraph {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyGraph)

  MultibodyGraph();

  LinkIndex AddLink(const std::string& name,
               ModelInstanceIndex model_instance = default_model_instance());

  JointIndex AddJoint(const std::string& name,
                      ModelInstanceIndex model_instance,
                      const std::string& type, LinkIndex parent_link_index,
                      LinkIndex child_link_index);

  /** @returns `true` if a body named `name` was added to `model_instance`.
  @see AddRigidBody().
  @throws std::exception if `model_instance` is not a valid index. */
  bool HasLinkNamed(const std::string& name,
                    ModelInstanceIndex model_instance) const;

  //const Link& GetLinkByName(const std::string& name) const;

  /** @returns `true` if a joint named `name` was added to `model_instance`.
  @see AddRigidBody().
  @throws std::exception if `model_instance` is not a valid index. */
  bool HasJointNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const;

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
  LinkIndex world_link_index() const { return LinkIndex(0); }

  /** Returns the unique index identifying the "weld" joint type (always zero).
   */
  JointTypeIndex weld_type_index() const { return JointTypeIndex(0); }

  static std::string weld_type_name() { return "weld"; }

  /** Register a joint type by name. In Drake "weld" is reserved to identify
  weld joints constraining two frames to move as one rigid composite. "weld"
  joint type has index `weld_type_index()`.
  @param[in] joint_type_name
    A unique string identifying a joint type, such as "pin" or "prismatic".
  @throws std::runtime_error if `joint_type_name` already identifies a
  previously registered joint type.
  @retval JointTypeIndex Index uniquely identifying the new joint type. */
  JointTypeIndex RegisterJointType(const std::string& joint_type_name);

  /** Returns the number of registered joint types. */
  int num_joint_types() const;

  /** Finds the assigned index for a joint type from the type name. Returns an
   * invalid index if `joint_type_name` was not previously registered with a
   * call to RegisterJointType(). */
  JointTypeIndex GetJointTypeIndex(const std::string& joint_type_name) const;

#if 0
  /// @returns `true` if this graph contains a link with the given name in any
  /// of the model instances.
  /// @see AddLink().
  bool HasLinkNamed(const std::string& name) const {
    return link_name_to_index_.count(name) > 0;
  }

#endif

  int num_links() const { return static_cast<int>(links_.size()); }

  int num_joints() const { return static_cast<int>(joints_.size()); }

  const Link& get_link(LinkIndex index) const {
    DRAKE_THROW_UNLESS(index < num_links());
    return links_[index];
  }

  const Joint& get_joint(JointIndex index) const {
    DRAKE_THROW_UNLESS(index < num_joints());
    return joints_[index];
  }

  std::vector<std::set<LinkIndex>> FindIslandsOfWeldedLinks() const;

 private:
  Link& get_mutable_link(LinkIndex link_index) { return links_[link_index]; }

  void FindIslandsOfWeldedLinksRecurse(
      const Link& parent_link, std::set<LinkIndex>* parent_island,
      std::vector<std::set<LinkIndex>>* islands,
      std::vector<bool>* visited) const;

  std::vector<Link> links_;  // World is NOT included. Only physical links.
  std::vector<Joint> joints_;

  std::unordered_map<std::string, JointTypeIndex> joint_type_name_to_index_;

  // The xxx_name_to_index_ structures are multimaps because
  // bodies/joints/actuators/etc may appear with the same name in different
  // model instances. The index values are still unique across the graph.
  std::unordered_multimap<std::string, LinkIndex> link_name_to_index_;
  std::unordered_multimap<std::string, JointIndex> joint_name_to_index_;
};

class Link {
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

  /// Returns the list of joints for which this link is the child link.
  const std::vector<JointIndex>& joints_as_child() const;

  /// Returns the list of joints for which this link is the parent link.
  const std::vector<JointIndex>& joints_as_parent() const;

  const std::vector<JointIndex>& joints() const { return joints_; }

 private:
  // Restrict construction and modification to only MultibodyGraph.
  friend class MultibodyGraph;

  Link(LinkIndex index, const std::string& name,
       ModelInstanceIndex model_instance)
      : index_(index), name_(name), model_instance_(model_instance) {}

  // Notes that this link serves as the parent link for the given joint.
  void add_joint_as_parent(JointIndex joint) {
    joints_as_parent_.push_back(joint);  // "as parent" list.
    joints_.push_back(joint);  // All joints list.
  }

  // Notes that this link serves as the child link for the given joint.
  void add_joint_as_child(JointIndex joint) {
    joints_as_child_.push_back(joint);  // "as child" list.
    joints_.push_back(joint);  // All joints list.
  }

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

class Joint {
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
