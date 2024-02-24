#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/ssize.h"
#include "drake/multibody/topology/link_joint_graph_defs.h"

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

// TODO(sherm1) The class comment describes the complete functionality of
//  PR #20225; only part of that code is actually here.

/** Represents a graph consisting of Links (user-defined rigid bodies)
interconnected by Joints.

Terminology note: for clarity we use "Link" here to mean what MultibodyPlant
calls a "RigidBody" (or just "Body"), that is, what a user inputs as an sdf or
urdf "link", as a MuJoCo "body", or using the AddRigidBody() call in
MultibodyPlant's API. (It would have been preferable to use Link in
MultibodyPlant as well, but that ship has sailed and there is less chance of
confusion there.) The spanning forest we generate uses "mobilized bodies"
(Mobods). A single Mobod may represent multiple Links (e.g., when multiple
links are welded together). Conversely, a single Link may be split to create
multiple Mobods (to break cycles in the graph). As there is not necessarily a
one-to-one mapping, we're careful not to mix the two terms.

%LinkJointGraph is a directed, possibly cyclic, graph whose nodes are Links and
whose edges are Joints, defined by a sequence of calls to AddLink() and
AddJoint(). At any point, a user can ask graph specific questions such as how
Links are connected, by which Joints, or perform more complex queries such as
what sets of Links are interconnected by Weld Joints.

LinkJointGraph is intended to represent the user-specified elements and
connectivity of a multibody system, such as might be supplied in an sdf file.
For efficient computation in Drake (using joint-space coordinates) we need to
create a model of this graph as a forest of spanning trees plus loop-closing
constraints. We call that a SpanningForest, one of which is owned by the graph.
The forest consists of "Mobods" (mobilized bodies), each representing a node of
the forest and that node's unique root-directed edge. A Mobod represents (1) a
rigid body (modeling one or more Links) and (2) that rigid body's unique inboard
mobilizer (modeling a Joint). We say a Link "follows" a Mobod if it is one of
the Links represented by that Mobod.

When we build a SpanningForest for a LinkJointGraph, the process may add a
limited set of _ephemeral_ "as built" Links, Joints, and Constraints to the
graph. Those are kept distinct from _user_ Links and Joints so we can easily
restore the graph to its original condition. These additions are limited to
those needed for breaking loops (by splitting a Link) and ensuring that every
body's mobility path leads to World (by adding floating or weld joints).

In general during SpanningForest building:
  - A user-specified Link may get split into a "primary" Link and one or more
    "shadow" Links in order to break loops. Each of those has its own Mobod, so
    a user Link can generate multiple Mobods. (Geometry should remain attached
    to the primary Link.)
  - A primary Link and its shadows must be welded together by Weld Constraints
    which will be added to this Graph during modeling.
  - Building the forest may require additional Joints to mobilize free bodies or
    immobilize static bodies. Each Joint maps to at most one Mobod; some Joints
    may instead be represented as Constraints or (for welds) implicitly as
    part of a composite rigid body.
  - Composite bodies (Links welded together) can be represented by a single
    Mobod, so many Links may follow one Mobod.
  - We never delete any of the user's Links or Joints; we may add new ones
    during forest building but those are distinct from the user's.

@note Links are indexed using MultibodyPlant's BodyIndex type; there is no
separate LinkIndex type since these are necessarily the same. */
class LinkJointGraph {
 public:
  class Link;  // Defined in separate headers.
  class Joint;

  struct JointType;  // Defined below.

  // This Doxygen text is intended to mimic the text generated for
  // DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN. There is some extra information
  // but the formatting is consistent.

  // clang-format off
  // NOLINTNEXTLINE(whitespace/line_length)
  /** @name Implements CopyConstructible, CopyAssignable, MoveConstructible, MoveAssignable */
  // clang-format on
  /**@{*/
  /** Copies both the graph and the forest if there is one. */
  LinkJointGraph(const LinkJointGraph&);
  /** Assigns both the graph and the forest if there is one. */
  LinkJointGraph& operator=(const LinkJointGraph&);
  /** Move construction leaves the source as though it had just been
  default constructed. */
  LinkJointGraph(LinkJointGraph&&);
  /** Move assignment leaves the source as though it had just been
  default constructed. */
  LinkJointGraph& operator=(LinkJointGraph&&);
  /**@}*/

  /** Default construction defines well-known joint types and World. */
  LinkJointGraph();

  /** Restores this graph to its default-constructed state. The contained
  SpanningForest object is preserved but now invalid. */
  void Clear();

  /** Sets the forest building options to be used for elements of any model
  instance that does not have its own forest building options set. Invalidates
  the current forest.
  @see SetForestBuildingOptions() */
  void SetGlobalForestBuildingOptions(ForestBuildingOptions global_options) {
    InvalidateForest();
    data_.global_forest_building_options = global_options;
  }

  /** Gets the forest building options to be used for elements of any model
  instance that does not have its own forest building options set. If no call
  has yet been made to SetGlobalForestBuildingOptions(), returns
  ForestBuildingOptions::kDefault. */
  ForestBuildingOptions get_global_forest_building_options() const {
    return data_.global_forest_building_options;
  }

  /** Sets the forest building options to be used for elements of the given
  `model_instance`, completely overriding (not blending with) global options.
  Invalidates the current forest. Model instances do not have to
  be pre-declared; we will simply record the given `options` with the given
  index and apply them if any elements are tagged with that index.
  @note MultibodyPlant predefines model instances 0 and 1 as the
  world_model_instance() and default_model_instance() respectively.
  @see SetGlobalForestBuildingOptions() */
  void SetForestBuildingOptions(ModelInstanceIndex model_instance,
                                ForestBuildingOptions options) {
    InvalidateForest();
    data_.model_instance_forest_building_options[model_instance] = options;
  }

  /** Gets the forest building options to be used for elements of the given
  `model_instance`. Returns the options set explicitly for this
  `model_instance`, if any. Otherwise, returns what
  get_global_forest_building_options() returns. */
  ForestBuildingOptions get_forest_building_options_in_use(
      ModelInstanceIndex model_instance) const {
    auto instance_options =
        data_.model_instance_forest_building_options.find(model_instance);
    return instance_options ==
                   data_.model_instance_forest_building_options.end()
               ? get_global_forest_building_options()  // Use global default.
               : instance_options->second;
  }

  /** Resets the global forest building options to kDefault and removes any
  per-model instance options that were set. */
  void ResetForestBuildingOptions() {
    InvalidateForest();
    data_.global_forest_building_options = ForestBuildingOptions::kDefault;
    data_.model_instance_forest_building_options.clear();
  }

  /** Models this LinkJointGraph as a spanning forest, records the result
  into this graph's SpanningForest object, and marks it valid. The currently-set
  forest building options are used. This is done unconditionally; if there
  was already a valid forest it is cleared and then rebuilt. If you don't want
  to rebuild unnecessarily, call forest_is_valid() first to check.
  @see forest_is_valid() to check whether the SpanningForest is already up
    to date with the graph's contents.
  @see forest() for access to the SpanningForest object.
  @see InvalidateForest() */
  void BuildForest();

  /** Invalidates this LinkJointGraph's SpanningForest and removes ephemeral
  "as-built" Link, Joint, and Constraint elements that were added to the graph
  during forest building. After this call the graph will contain only
  user-provided elements and forest_is_valid() will return `false`. A
  subsequent call to BuildForest() is required to rebuild the forest.
  @note Any change to graph contents or forest building options will also
    invalidate the forest as a side effect. */
  void InvalidateForest();

  /** Has the SpanningForest been rebuilt with BuildForest() since the most
  recent change to this LinkJointGraph or to the forest building options? If
  this returns `false` then a call to BuildForest() is required before you can
  make use of the SpanningForest. */
  bool forest_is_valid() const { return data_.forest_is_valid; }

  /** Returns a reference to this LinkJointGraph's SpanningForest
  object (without rebuilding). The forest is a data member of LinkJointGraph and
  the _reference_ remains unchanged across repeated graph modifications and
  calls to BuildForest(), though the _contents_ will change. You can call the
  graph's forest_is_valid() or the forest's `is_valid()` function to check
  whether the forest is currently up to date with respect to the graph's
  contents. Use BuildForest() to bring the forest up to date if necessary.
  @warning If you retain this reference be aware that the contents are only
    meaningful when `forest_is_valid()` returns true. We do not promise
    anything about the contents otherwise. */
  const SpanningForest& forest() const { return *data_.forest; }

  // TODO(sherm1) Add editing functions like Remove{Link,Joint} and maybe
  //  ReplaceJoint().

  /** Adds a new Link to the graph. Invalidates the SpanningForest; call
  BuildForest() to rebuild.
  @param[in] name
    A unique name for the new Link (rigid body) in the given `model_instance`.
    Several Links can have the same name within a %LinkJointGraph as long as
    they are unique within their model instances.
  @param[in] model_instance
    The model instance to which this Link belongs.
  @param[in] flags
    Any special handling required for this Link.
  @note The World Link is always predefined with name "world" (case sensitive),
    model instance world_model_instance(), and index 0.
  @returns The unique BodyIndex for the added Link in the graph.
  @throws std::exception if `name` is duplicated within `model_instance`.
  @throws std::exception if an attempt is made to add a link into the World's
    model instance
  @throws std::exception if the internal-only Shadow flag is set. */
  BodyIndex AddLink(const std::string& name, ModelInstanceIndex model_instance,
                    LinkFlags flags = LinkFlags::kDefault);

  /** Returns the Link that corresponds to World (always predefined). */
  const Link& world_link() const;

  /** Registers a joint type by name.
  @param[in] joint_type_name
    A unique string identifying a joint type, such as "revolute" or
    "prismatic".
  @param[in] nq
    Number of generalized position coordinates q needed for implementation
    of this type of Joint.
  @param[in] nv
    Number of generalized velocity coordinates v needed for implementation
    of this type of Joint.
  @param[in] has_quaternion
    Whether the first four q values represent a quaternion (in w[xyz] order).
  @retval joint_type_index
    Unique index assigned to the new joint type.
  @throws std::exception if `joint_type_name` already identifies a
    previously registered joint type.
  @pre 0 ≤ nq ≤ 7, 0 ≤ nv ≤ 6, nv ≤ nq, !has_quaternion or nq ≥ 4.

  @note LinkJointGraph is preloaded with Drake-specific Joint types it needs to
  know about including "weld", "quaternion_floating", and "rpy_floating". */
  JointTypeIndex RegisterJointType(const std::string& joint_type_name, int nq,
                                   int nv, bool has_quaternion = false);

  /** Returns a reference to the vector of known and registered joint types. */
  const std::vector<JointType>& joint_types() const {
    return data_.joint_types;
  }

  /** Returns a reference to a particular JointType using one of the
  predefined indices or an index returned by RegisterJointType(). */
  const JointType& joint_types(JointTypeIndex index) const {
    return joint_types()[index];
  }

  /** Returns a reference to the vector of Link objects. World is always the
  first entry and is always present. */
  const std::vector<Link>& links() const { return data_.links; }

  /** Returns a reference to a particular Link. The World Link is index 0,
  others use the index returned by AddLink(). Links added by BuildForest()
  are indexed last. */
  inline const Link& links(BodyIndex link_index) const;

  /** Returns a reference to the vector of Joint objects. */
  const std::vector<Joint>& joints() const { return data_.joints; }

  /** Returns a reference to a particular Joint using the index returned by
  AddJoint(). Joints added by BuildForest() are indexed last. */
  inline const Joint& joints(JointIndex joint_index) const;

  /** @returns `true` if a Link named `name` was added to `model_instance`.
  @see AddLink().
  @throws std::exception if `model_instance` is not a valid index. */
  bool HasLinkNamed(const std::string& name,
                    ModelInstanceIndex model_instance) const;

  // Forest building requires these joint types so they are predefined.

  /** The predefined index for the "weld" joint type. */
  static JointTypeIndex weld_joint_type_index() { return JointTypeIndex(0); }

  /** The predefined index for the "quaternion_floating" joint type. */
  static JointTypeIndex quaternion_floating_joint_type_index() {
    return JointTypeIndex(1);
  }

  /** The predefined index for the "rpy_floating" joint type. */
  static JointTypeIndex rpy_floating_joint_type_index() {
    return JointTypeIndex(2);
  }

  /** This is all we need to know about joint types for topological purposes. */
  struct JointType {
    std::string type_name;
    int nq{-1};
    int nv{-1};
    bool has_quaternion;  // If so, the first 4 qs are wxyz.
  };

 private:
  friend class SpanningForest;
  friend class LinkJointGraphTester;

  // Finds the assigned index for a joint type from the type name. Returns an
  // invalid index if `joint_type_name` was not previously registered with a
  // call to RegisterJointType().
  JointTypeIndex GetJointTypeIndex(const std::string& joint_type_name) const;

  // Links that were explicitly flagged LinkFlags::kStatic in AddLink().
  const std::vector<BodyIndex>& static_links() const {
    return data_.static_links;
  }

  // Links that were flagged LinkFlags::kMustBeBaseBody in AddLink() but were
  // not also flagged kStatic (those are inherently base bodies since by
  // definition they are welded to World).
  const std::vector<BodyIndex>& non_static_must_be_base_body_links() const {
    return data_.non_static_must_be_base_body_links;
  }

  // Group data members so we can have the compiler generate most of the
  // copy/move/assign methods for us while still permitting pointer fixup.
  struct Data {
    // These are all default but definitions deferred to .cc file so
    // that all the local classes are defined (Mac requirement).
    Data();
    Data(const Data&);
    Data(Data&&);
    ~Data();
    Data& operator=(const Data&);
    Data& operator=(Data&&);

    std::vector<JointType> joint_types;

    // The first entry in links is the World Link with BodyIndex(0) and name
    // world_link().name().
    std::vector<Link> links;
    std::vector<Joint> joints;

    // The first num_user_links etc. elements are user-supplied; the rest were
    // added during modeling.
    int num_user_links{0};
    int num_user_joints{0};

    std::vector<BodyIndex> static_links;
    std::vector<BodyIndex> non_static_must_be_base_body_links;

    // Every user Link, organized by Model Instance.
    std::map<ModelInstanceIndex, std::vector<BodyIndex>>
        model_instance_to_links;

    // This must always have the same number of entries as joint_types_.
    std::unordered_map<std::string, JointTypeIndex> joint_type_name_to_index;

    // The xxx_name_to_index_ structures are multimaps because
    // links/joints/actuators/etc may appear with the same name in different
    // model instances. The index values are still unique across the graph.
    // These include only user-supplied links and joints.
    std::unordered_multimap<std::string, BodyIndex> link_name_to_index;
    std::unordered_multimap<std::string, JointIndex> joint_name_to_index;

    // The 0th composite always exists and contains World (listed first)
    // and any links welded (recursively) to World. The other composites are
    // only present if there are at least two Links welded together. The first
    // Link in the composite is the active Link.
    std::vector<std::vector<BodyIndex>> link_composites;

    bool forest_is_valid{false};  // set false whenever changes are made

    // These parallel {link,joint}_name_to_index except they hold mappings
    // for ephemeral links and joints added during forest-building. We keep
    // them separately so they are easy to get rid of.
    std::unordered_multimap<std::string, BodyIndex>
        ephemeral_link_name_to_index;
    std::unordered_multimap<std::string, JointIndex>
        ephemeral_joint_name_to_index;

    ForestBuildingOptions global_forest_building_options{
        ForestBuildingOptions::kDefault};
    std::map<ModelInstanceIndex, ForestBuildingOptions>
        model_instance_forest_building_options;

    // Contains a back pointer to the graph so needs fixup post copy or move.
    copyable_unique_ptr<SpanningForest> forest;
  } data_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
