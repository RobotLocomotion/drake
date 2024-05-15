#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/ssize.h"
#include "drake/common/string_unordered_map.h"
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
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(LinkJointGraph)

  class Link;  // Defined in separate headers.
  class Joint;
  class LoopConstraint;

  /** This is all we need to know about joint types for topological purposes. */
  struct JointTraits {
    std::string name;
    int nq{-1};
    int nv{-1};
    bool has_quaternion{false};  // If so, the first 4 qs are wxyz.
  };

  /** Default construction defines well-known joint types and World. */
  LinkJointGraph();

  /** Restores this graph to its default-constructed state. The contained
  SpanningForest object is preserved but now invalid. */
  void Clear();

  /** Sets the forest building options to be used for elements of any model
  instance that does not have its own forest building options set. Invalidates
  the current forest.
  @see SetForestBuildingOptions() */
  void SetGlobalForestBuildingOptions(ForestBuildingOptions global_options);

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
                                ForestBuildingOptions options);

  /** Gets the forest building options to be used for elements of the given
  model instance. Returns the options set explicitly for this model instance,
  if any. Otherwise, returns what get_global_forest_building_options() returns.
  @pre `instance_index` is any valid index */
  ForestBuildingOptions get_forest_building_options_in_use(
      ModelInstanceIndex instance_index) const {
    return get_model_instance_options(instance_index)
        .value_or(get_global_forest_building_options());
  }

  /** Resets the global forest building options to kDefault and removes any
  per-model instance options that were set. */
  void ResetForestBuildingOptions();

  /** Models this LinkJointGraph as a spanning forest, records the result
  into this graph's SpanningForest object, and marks it valid. The currently-set
  forest building options are used. This is done unconditionally; if there
  was already a valid forest it is cleared and then rebuilt. If you don't want
  to rebuild unnecessarily, call forest_is_valid() first to check.

  If there are massless links in the graph, it is possible that one of them
  ends as the terminal link in a branch. In that case the resulting system will
  have a singular mass matrix and be unsuited for dynamics. However, it can
  still be used for kinematics, visualization, and some other analyses so the
  forest is still considered valid. In that case we return `false` here and
  a human-readable message explaining what happened is available in case you
  want to report it.

  @returns true if the resulting forest can be used for dynamics.
  @see forest_is_valid() to check whether the SpanningForest is already up
    to date with the graph's contents.
  @see forest() for access to the SpanningForest object.
  @see forest().why_no_dynamics() for a human-readable explanation if
    BuildForest() returned `false`.
  @see InvalidateForest() */
  bool BuildForest();

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
    Optional LinkFlags requesting special handling for this Link. Don't set
    any flags marked "internal use only" (currently just `Shadow`).
  @note The World Link is always predefined with name "world" (case sensitive),
    model instance world_model_instance(), and index 0.
  @returns The unique BodyIndex for the added Link in the graph.
  @throws std::exception if `name` is duplicated within `model_instance`.
  @throws std::exception if an attempt is made to add a link into the World's
    model instance
  @throws std::exception if the `flags` parameter has any internal-only flags
    set (currently just `Shadow`). */
  BodyIndex AddLink(const std::string& name, ModelInstanceIndex model_instance,
                    LinkFlags flags = LinkFlags::kDefault);

  /** Adds a new Joint to the graph.
  @param[in] name
    A unique name for the new Joint in the given `model_instance`. Several
    Joints can have the same name within a %LinkJointGraph as long as they are
    unique within their model instances.
  @param[in] model_instance_index
    The index of the model instance to which this Joint belongs, see
    @ref model_instance.
  @param[in] type
    A string designating the type of this Joint, such as "revolute" or
    "ball". This must be chosen from the set of Joint types previously
    registered with calls to RegisterJointType().
  @param[in] parent_link_index
    The index of a Link previously obtained with a call to AddLink(), or
    BodyIndex(0) for the World Link.
  @param[in] child_link_index
    The index of a Link previously obtained with a call to AddLink(), or
    BodyIndex(0) for the World Link.
  @param[in] flags
    Optional JointFlags requesting special handling for this Joint.
  @returns The unique JointIndex for the added Joint in the graph.
  @throws std::exception if `name` is duplicated within `model_instance`.
  @throws std::exception if `type` has not been registered with
      RegisterJointType().
  @throws std::exception if `parent_link_index` or `child_link_index` are
      not valid Link indices for this graph, if they are the same index,
      or if there is already a Joint between them.
  @throws std::exception if a static Link is connected to World by anything
      other than a "weld" Joint. */
  JointIndex AddJoint(const std::string& name,
                      ModelInstanceIndex model_instance_index,
                      const std::string& type, BodyIndex parent_link_index,
                      BodyIndex child_link_index,
                      JointFlags flags = JointFlags::kDefault);

  /** Returns the Link that corresponds to World (always predefined). */
  const Link& world_link() const;

  /** Registers a joint type by name and provides the joint traits for it.
  @param[in] joint_type_name
    A unique string identifying a joint type, such as "revolute" or
    "prismatic". Should match the MultibodyPlant name for the Joint it
    represents.
  @param[in] nq
    Number of generalized position coordinates q needed for implementation
    of this type of Joint.
  @param[in] nv
    Number of generalized velocity coordinates v needed for implementation
    of this type of Joint.
  @param[in] has_quaternion
    Whether the first four q values represent a quaternion (in w[xyz] order).
  @retval joint_traits_index
    Unique index assigned to the new joint type.
  @throws std::exception if `joint_type_name` was already used for a
    previously registered joint type.
  @pre 0 ≤ nq ≤ 7, 0 ≤ nv ≤ 6, nv ≤ nq, !has_quaternion or nq ≥ 4.

  @note LinkJointGraph is preloaded with the traits for Drake-specific Joint
  types it needs to know about including "weld", "quaternion_floating", and
  "rpy_floating". */
  JointTraitsIndex RegisterJointType(const std::string& joint_type_name, int nq,
                                     int nv, bool has_quaternion = false);

  /** Returns `true` if the given `joint_type_name` was previously registered
  via a call to RegisterJointType(), or is one of the pre-registered names. */
  bool IsJointTypeRegistered(const std::string& joint_type_name) const;

  /** Returns a reference to the vector of traits for the known and registered
  Joint types. */
  const std::vector<JointTraits>& joint_traits() const {
    return data_.joint_traits;
  }

  /** Returns a reference to a particular JointTraits object using one of the
  predefined indices or an index returned by RegisterJointType(). Requires a
  JointTraitsIndex, not a plain integer.*/
  const JointTraits& joint_traits(JointTraitsIndex index) const {
    return joint_traits().at(index);
  }

  /** Returns a reference to the vector of Link objects. World is always the
  first entry and is always present. */
  const std::vector<Link>& links() const { return data_.links; }

  /** Returns a reference to a particular Link. The World Link is BodyIndex(0),
  others use the index returned by AddLink(). Ephemeral links added by
  BuildForest() are indexed last. Requires a BodyIndex, not a plain integer. */
  inline const Link& links(BodyIndex link_index) const;

  /** Returns a reference to the vector of Joint objects. */
  const std::vector<Joint>& joints() const { return data_.joints; }

  /** Returns a reference to a particular Joint using the index returned by
  AddJoint(). Ephemeral joints added by BuildForest() are indexed last. Requires
  a JointIndex, not a plain integer. */
  inline const Joint& joints(JointIndex joint_index) const;

  /** Returns a reference to the vector of LoopConstraint objects. These are
  always "ephemeral" (added during forest-building) so this will return empty
  if there is no valid Forest. See the class comment for more information. */
  const std::vector<LoopConstraint>& loop_constraints() const {
    return data_.loop_constraints;
  }

  /** Returns a reference to a particular LoopConstraint. Requires a
  LoopConstraintIndex, not a plain integer.*/
  inline const LoopConstraint& loop_constraints(
      LoopConstraintIndex constraint_index) const;

  /** Links with this index or higher are "ephemeral" (added during
  forest-building). See the class comment for more information. */
  int num_user_links() const { return data_.num_user_links; }

  /** Joints with this index or higher are "ephemeral" (added during
  forest-building). See the class comment for more information. */
  int num_user_joints() const { return data_.num_user_joints; }

  /** After the SpanningForest has been built, returns the mobilized body
  (Mobod) followed by this Link. If the Link is part of a composite, this will
  be the mobilized body for the whole composite. If the Link was split into a
  primary and shadows, this is the mobilized body followed by the primary. If
  there is no valid Forest, the returned index will be invalid. */
  MobodIndex link_to_mobod(BodyIndex index) const;

  /** After the SpanningForest has been built, returns groups of Links that are
  welded together, which we call "LinkComposites". Each group may be modeled
  in the Forest with a single mobilized body or multiple mobilized bodies
  depending on modeling options, but that doesn't change anything here. The
  first entry in each LinkComposite is the "active link", the one whose
  (non-weld) Joint moves the whole LinkComposite due to its modeling in the
  SpanningForest. The rest of the Links in the composite are listed in no
  particular order.

  The 0th LinkComposite is always present (if there is a valid SpanningForest)
  and its first entry is World (Link 0), even if nothing else is welded to
  World. Otherwise, composites are present here if they contain two or more
  welded Links; Links that aren't welded to any other Links are not included
  here at all. LinkComposites are discovered as a side effect of
  forest-building; there is no cost to accessing them here.

  If there is no valid Forest, the returned vector is empty. */
  const std::vector<std::vector<BodyIndex>>& link_composites() const {
    return data_.link_composites;
  }

  /** Returns a reference to a particular LinkComposite. Requires a
  LinkCompositeIndex, not a plain integer.*/
  const std::vector<BodyIndex>& link_composites(
      LinkCompositeIndex composite_link_index) const {
    return link_composites().at(composite_link_index);
  }

  /** @returns `true` if a Link named `name` was added to `model_instance`.
  @see AddLink().
  @throws std::exception if `model_instance_index` is not a valid index. */
  bool HasLinkNamed(std::string_view name,
                    ModelInstanceIndex model_instance_index) const;

  /** @returns `true` if a Joint named `name` was added to `model_instance`.
  @see AddJoint().
  @throws std::exception if `model_instance_index` is not a valid index. */
  bool HasJointNamed(std::string_view name,
                     ModelInstanceIndex model_instance_index) const;

  /** If there is a Joint connecting the given Links, returns its index. You can
  call this any time and it will work with whatever Joints have been defined.
  But note that Links may be split and additional Joints added during Forest
  building, so you may get a different answer before and after modeling. Cost is
  O(j) where j=min(j₁,j₂) with jᵢ the number of Joints attached to linkᵢ. */
  std::optional<JointIndex> MaybeGetJointBetween(BodyIndex link1_index,
                                                 BodyIndex link2_index) const;

  /** Returns true if the given Link should be treated as massless. That
  requires that the Link was marked TreatAsMassless and is not connected by
  a Weld Joint to a massful Link or Composite. */
  bool must_treat_as_massless(BodyIndex link_index) const;

  /** (Internal use only) For testing -- invalidates the Forest. */
  void ChangeLinkFlags(BodyIndex link_index, LinkFlags flags);

  /** (Internal use only) For testing -- invalidates the Forest. */
  void ChangeJointFlags(JointIndex joint_index, JointFlags flags);

  /** (Internal use only) Changes the type of an existing user-defined Joint
  without making any other changes. Invalidates the Forest, even if the new
  type is the same as the current type.
  @throws std::exception if called on an ephemeral (added) joint.
  @throws std::exception if an attempt is made to change a static link's joint
   to anything other than a weld.
  @pre the joint index is in range and the type name is of a registered or
    predefined joint type. */
  void ChangeJointType(JointIndex existing_joint_index,
                       const std::string& name_of_new_type);

  // Forest building requires these joint types so they are predefined.

  /** The predefined index for the "weld" joint type's traits. */
  static JointTraitsIndex weld_joint_traits_index() {
    return JointTraitsIndex(0);
  }

  /** The predefined index for the "quaternion_floating" joint type's traits. */
  static JointTraitsIndex quaternion_floating_joint_traits_index() {
    return JointTraitsIndex(1);
  }

  /** The predefined index for the "rpy_floating" joint type's traits. */
  static JointTraitsIndex rpy_floating_joint_traits_index() {
    return JointTraitsIndex(2);
  }

 private:
  friend class SpanningForest;
  friend class LinkJointGraphTester;

  inline Link& mutable_link(BodyIndex link_index);

  inline Joint& mutable_joint(JointIndex joint_index);

  // Tells this Link which Mobod it follows and which Joint corresponds to
  // that Mobod's inboard mobilizer.
  void set_primary_mobod_for_link(BodyIndex link_index,
                                  MobodIndex primary_mobod_index,
                                  JointIndex primary_joint_index);

  // Tells this currently-unmodeled Joint that the given Mobod models it.
  void set_mobod_for_joint(JointIndex joint_index, MobodIndex mobod_index);

  // The World Link must already be in the graph but there are no Link
  // Composites yet. This creates the 0th LinkComposite and puts World in it.
  void CreateWorldLinkComposite();

  // Registers a loop-closing weld constraint between these Links and updates
  // the Links to know about it.
  LoopConstraintIndex AddLoopClosingWeldConstraint(BodyIndex primary_link_index,
                                                   BodyIndex shadow_link_index);

  // Delegates to each MobodIndex-keeping element to renumber those indices
  // using the given map.
  // @pre old_to_new is a permutation of [0, ..., num_mobods-1].
  void RenumberMobodIndexes(const std::vector<MobodIndex>& old_to_new);

  // The Forest already has the given (inboard) Mobod, and we want to add a new
  // Mobod outboard of that one to model the given Joint. At least one of the
  // Joint's two Links must already be following the inboard Mobod. That one
  // will be the inboard Link (that is, the Link following the more-inboard
  // Mobod). If that's the Joint's parent Link then parent->child and
  // inboard->outboard will match, otherwise the mobilizer must be reversed.
  // Returned tuple is: inboard link, outboard link, is_reversed.
  std::tuple<BodyIndex, BodyIndex, bool> FindInboardOutboardLinks(
      MobodIndex inboard_mobod_index, JointIndex joint_index) const;

  // Adds the ephemeral Joint for a floating or fixed base Link to mirror a
  // mobilizer added during BuildForest(). World is the parent and the given
  // base Link is the child for the new Joint.
  JointIndex AddEphemeralJointToWorld(JointTraitsIndex type_index,
                                      BodyIndex child_link_index);

  // Adds the new Link to the LinkComposite of which maybe_composite_link is a
  // member. If maybe_composite_link is not a member of any LinkComposite, then
  // we create a new LinkComposite with maybe_composite_link as the first
  // (and hence "active") Link.
  LinkCompositeIndex AddToLinkComposite(BodyIndex maybe_composite_link_index,
                                        BodyIndex new_link_index);

  // While building the Forest, adds a new Shadow link to the given Primary
  // link, with the Shadow mobilized by the given joint. We'll derive a name for
  // the Shadow from the Primary, create the link with appropriate bookkeeping,
  // and add a LoopConstraint (a weld) between the Primary and Shadow. The
  // Shadow link's Mobod is always terminal in the forest but the Shadow link
  // may serve as the parent link for the joint if the sense of the joint is
  // reversed from the implementing mobilizer. Note: A Drake weld constraint
  // connects a "parent" link to a "child" link; that ordering determines the
  // sign convention for its multipliers (forces). We always make the Primary
  // the weld's parent link, and the Shadow its child.
  BodyIndex AddShadowLink(BodyIndex primary_link_index,
                          JointIndex shadow_joint_index, bool shadow_is_parent);

  // Finds the assigned index for a joint's traits from the joint's type name.
  // Returns nullopt if `joint_type_name` was not previously registered
  // with a call to RegisterJointType().
  std::optional<JointTraitsIndex> GetJointTraitsIndex(
      const std::string& joint_type_name) const;

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

  // A link is static if it is in a static model instance or if it has been
  // explicitly marked static.
  bool link_is_static(const Link& link) const;

  // For any valid index: if in range, returns the stored model instance options
  // (which will be nullopt if never set), otherwise nullopt.
  std::optional<ForestBuildingOptions> get_model_instance_options(
      ModelInstanceIndex instance_index) const {
    DRAKE_ASSERT(instance_index.is_valid());
    return instance_index < ssize(data_.model_instance_forest_building_options)
               ? data_.model_instance_forest_building_options[instance_index]
               : std::nullopt;
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

    std::vector<JointTraits> joint_traits;

    // The first entry in links is the World Link with BodyIndex(0) and name
    // world_link().name(). Ephemeral "as-built" links and joints are placed
    // at the end of these lists, following the user-supplied ones.
    std::vector<Link> links;
    std::vector<Joint> joints;

    // The first num_user_links etc. elements are user-supplied; the rest were
    // added during modeling.
    int num_user_links{0};
    int num_user_joints{0};

    // Loop Weld Constraints are only added during forest building; we don't
    // record any user constraints in the LinkJointGraph because they don't
    // affect how we build the forest.
    std::vector<LoopConstraint> loop_constraints;

    std::vector<BodyIndex> static_links;
    std::vector<BodyIndex> non_static_must_be_base_body_links;

    // Every user Link, organized by Model Instance.
    std::map<ModelInstanceIndex, std::vector<BodyIndex>>
        model_instance_to_links;

    // This must always have the same number of entries as joint_traits_.
    string_unordered_map<JointTraitsIndex> joint_type_name_to_index;

    // The xxx_name_to_index_ structures are multimaps because
    // links/joints/actuators/etc may appear with the same name in different
    // model instances. The index values are still unique across the graph.
    // These include only user-supplied links and joints.
    string_unordered_multimap<BodyIndex> link_name_to_index;
    string_unordered_multimap<JointIndex> joint_name_to_index;

    // The 0th composite always exists and contains World (listed first)
    // and any links welded (recursively) to World. The other composites are
    // only present if there are at least two Links welded together. The first
    // Link in the composite is the active Link.
    std::vector<std::vector<BodyIndex>> link_composites;

    bool forest_is_valid{false};  // set false whenever changes are made

    // These parallel {link,joint}_name_to_index except they hold mappings
    // for ephemeral links and joints added during forest-building. We keep
    // them separately so they are easy to get rid of.
    string_unordered_multimap<BodyIndex> ephemeral_link_name_to_index;
    string_unordered_multimap<JointIndex> ephemeral_joint_name_to_index;

    ForestBuildingOptions global_forest_building_options{
        ForestBuildingOptions::kDefault};

    // Indexed by ModelInstanceIndex.
    std::vector<std::optional<ForestBuildingOptions>>
        model_instance_forest_building_options;

    // Contains a back pointer to the graph so needs fixup post copy or move.
    copyable_unique_ptr<SpanningForest> forest;
  } data_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
