#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_GRAPH_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/graph.h".
#endif

#include <algorithm>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/string_unordered_map.h"
#include "drake/multibody/topology/link_joint_graph_defs.h"

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

/* Represents a graph consisting of Links (user-defined rigid bodies)
interconnected by Joints.

Terminology notes:
 - For clarity we use "Link" here to mean what MultibodyPlant calls a
   "RigidBody", that is, what a user inputs as an SDFormat or URDF "link", as a
   MuJoCo "body", or using the AddRigidBody() call in MultibodyPlant's API. (It
   would have been preferable to use Link exclusively in MultibodyPlant as well,
   but that ship has sailed and there is less chance of confusion there.)
 - The LinkIndex we use here is just an alias for MultibodyPlant's BodyIndex;
   they may be used interchangeably.
 - The spanning forest we generate uses "mobilized bodies" (Mobods). A single
   Mobod may model a single Link or a composite formed of multiple Links that
   are welded together. Conversely, a single Link may be split to create
   multiple Mobods (to break cycles in the graph). As there is not necessarily
   a one-to-one mapping, we're careful in this topology code not to use the term
   "body" when we mean "link". If we say "body" at all it is in the context of
   "mobilized body" or Mobod.
 - Because Links and Joints may be removed from an existing graph, there may be
   gaps in the sequence of LinkIndex and JointIndex values. However, any
   remaining Links and Joints are stored consecutively, indexed by LinkOrdinal
   and JointOrdinal. The sequence of ordinals does not have gaps. Indices are
   persistent once assigned; ordinals may change as elements are added or
   removed.

%LinkJointGraph is a directed, possibly cyclic, graph whose nodes are Links and
whose edges are Joints, defined by a sequence of calls to AddLink() and
AddJoint(). At any point, a user can ask graph specific questions such as how
Links are connected, by which Joints, or perform more complex queries such as
identifying "assemblies" (see below).

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

While building a SpanningForest for a LinkJointGraph, we learn useful things
about the graph and update the LinkJointGraph accordingly to include:
- a limited set of _ephemeral_ "as built" Links, Joints, and Constraints,
- the correspondence between each graph element and its representation in the
  forest, and
- identification of the graph's _assemblies_: maximal connected subgraphs
  composed only of Links and Weld Joints.

Ephemeral elements are kept distinct from user Links and Joints so we can
easily restore the graph to its original user-specified condition. These
additions are limited to those needed for breaking loops (by splitting a Link)
and ensuring that every Link follows a Mobod whose mobility path leads to World
(by adding floating or weld joints).

In general during SpanningForest building there will not be a one-to-one
correspondence between elements of the graph and the forest:
  - A user-specified Link may get split into a "primary" Link and one or more
    "shadow" Links in order to break loops. Each of those follows its own Mobod,
    so a user Link can generate multiple Mobods. (Geometry should remain
    attached to the Mobod followed by the primary Link.)
  - A primary Link and its shadows must be welded together by weld constraints
    which will be included in the forest and added as an ephemeral element to
    the graph.
  - An assembly (links welded together) may be represented using fewer Mobods
    than links so many Links may follow one Mobod.
  - Building the forest may require additional Joints to mobilize free bodies or
    immobilize static bodies. Each Joint maps to at most one Mobod; a weld joint
    whose parent and child Links follow the same composite Mobod will not be
    modeled.
  - We never delete any of the user's Links or Joints; we may add new ones
    during forest building (the ephemeral elements mentioned above) but those
    are kept distinct from the user's elements. */
class LinkJointGraph {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(LinkJointGraph);

  class Link;  // Defined in separate headers.
  class Joint;
  class LoopConstraint;

  /* This is all we need to know about joint types for topological purposes. */
  struct JointTraits {
    std::string name;
    int nq{-1};
    int nv{-1};
    bool has_quaternion{false};  // If so, the first 4 qs are wxyz.
  };

  /* A WeldedLinksAssembly is a set of Links and Weld Joints where the links
  are interconnected by paths comprised of one or more of those welds. Thus
  the whole assembly will move as a single rigid object. An assembly is
  massless only if _all_ its constituent links are massless. */
  class WeldedLinksAssembly {
   public:
    const std::vector<LinkIndex>& links() const { return links_; }
    const std::vector<JointIndex>& joints() const { return joints_; }
    bool is_massless() const { return is_massless_; }
    bool HasLink(LinkIndex index) const {
      return std::find(links_.begin(), links_.end(), index) != links_.end();
    }
    bool HasJoint(JointIndex index) const {
      return std::find(joints_.begin(), joints_.end(), index) != joints_.end();
    }

   private:
    // LinkJointGraph has direct access to these data members and is responsible
    // for coherence; SanityCheckForest() is used to verify.
    friend class LinkJointGraph;
    std::vector<LinkIndex> links_;
    std::vector<JointIndex> joints_;
    bool is_massless_{false};
  };

  /* Default construction defines well-known joint types and World. */
  LinkJointGraph();

  /* Restores this graph to its default-constructed state. The contained
  SpanningForest object is preserved but now invalid. */
  void Clear();

  /* Sets the forest building options to be used for elements of any model
  instance that does not have its own forest building options set. Invalidates
  the current forest.
  @see SetForestBuildingOptions() */
  void SetGlobalForestBuildingOptions(ForestBuildingOptions global_options);

  /* Gets the forest building options to be used for elements of any model
  instance that does not have its own forest building options set. If no call
  has yet been made to SetGlobalForestBuildingOptions(), returns
  ForestBuildingOptions::kDefault. */
  [[nodiscard]] ForestBuildingOptions get_global_forest_building_options()
      const {
    return data_.global_forest_building_options;
  }

  /* Sets the forest building options to be used for elements of the given
  `model_instance`, completely overriding (not blending with) global options.
  Invalidates the current forest. Model instances do not have to
  be pre-declared; we will simply record the given `options` with the given
  index and apply them if any elements are tagged with that index.
  @note MultibodyPlant predefines model instances 0 and 1 as the
  world_model_instance() and default_model_instance() respectively.
  @see SetGlobalForestBuildingOptions() */
  void SetForestBuildingOptions(ModelInstanceIndex model_instance,
                                ForestBuildingOptions options);

  /* Gets the forest building options to be used for elements of the given
  model instance. Returns the options set explicitly for this model instance,
  if any. Otherwise, returns what get_global_forest_building_options() returns.
  @pre `instance_index` is any valid index */
  [[nodiscard]] ForestBuildingOptions get_forest_building_options_in_use(
      ModelInstanceIndex instance_index) const {
    return get_model_instance_options(instance_index)
        .value_or(get_global_forest_building_options());
  }

  /* Resets the global forest building options to kDefault and removes any
  per-model instance options that were set. */
  void ResetForestBuildingOptions();

  /* Models this LinkJointGraph as a spanning forest, records the result
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

  /* Invalidates this LinkJointGraph's SpanningForest and removes ephemeral
  "as-built" Link, Joint, and Constraint elements that were added to the graph
  during forest building. After this call the graph will contain only
  user-provided elements and forest_is_valid() will return `false`. A
  subsequent call to BuildForest() is required to rebuild the forest.
  @note Any change to graph contents or forest building options will also
    invalidate the forest as a side effect. */
  void InvalidateForest();

  /* Has the SpanningForest been rebuilt with BuildForest() since the most
  recent change to this LinkJointGraph or to the forest building options? If
  this returns `false` then a call to BuildForest() is required before you can
  make use of the SpanningForest. */
  [[nodiscard]] bool forest_is_valid() const { return data_.forest_is_valid; }

  /* Returns a reference to this LinkJointGraph's SpanningForest
  object (without rebuilding). The forest is a data member of LinkJointGraph and
  the _reference_ remains unchanged across repeated graph modifications and
  calls to BuildForest(), though the _contents_ will change. You can call the
  graph's forest_is_valid() or the forest's `is_valid()` function to check
  whether the forest is currently up to date with respect to the graph's
  contents. Use BuildForest() to bring the forest up to date if necessary.
  @warning If you retain this reference be aware that the contents are only
    meaningful when `forest_is_valid()` returns true. We do not promise
    anything about the contents otherwise. */
  [[nodiscard]] const SpanningForest& forest() const { return *data_.forest; }

  /* Adds a new Link to the graph. Invalidates the SpanningForest; call
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
  @returns The unique LinkIndex for the added Link in the graph.
  @throws std::exception if `name` is duplicated within `model_instance`.
  @throws std::exception if an attempt is made to add a link into the World's
    model instance.
  @throws std::exception if the `flags` parameter has any internal-only flags
    set (currently just `Shadow`). */
  LinkIndex AddLink(const std::string& name, ModelInstanceIndex model_instance,
                    LinkFlags flags = LinkFlags::kDefault);

  // TODO(sherm1) Add this method;
  //  void RemoveLink(LinkIndex doomed_link_index);

  /* Returns true if the given `link_index` is in range and the corresponding
  Link hasn't been removed. Can be a user or ephemeral Link. */
  [[nodiscard]] bool has_link(LinkIndex link_index) const {
    return link_index < num_link_indexes() &&
           data_.link_index_to_ordinal[link_index].has_value();
  }

  /* Returns true if the given Link exists and is ephemeral. Ephemeral links
  are present only if the SpanningForest is valid. */
  [[nodiscard]] bool link_is_ephemeral(LinkIndex link_index) const {
    if (has_link(link_index) && link_index > data_.max_user_link_index) {
      DRAKE_ASSERT(index_to_ordinal(link_index) >= num_user_links());
      return true;
    }
    return false;
  }

  /* Adds a new Joint to the graph.
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
    LinkIndex(0) for the World Link.
  @param[in] child_link_index
    The index of a Link previously obtained with a call to AddLink(), or
    LinkIndex(0) for the World Link.
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
                      const std::string& type, LinkIndex parent_link_index,
                      LinkIndex child_link_index,
                      JointFlags flags = JointFlags::kDefault);

  /* Removes a previously-added Joint and any references to it in the graph.
  Invalidates the SpanningForest. Will repack and change ordinals for the
  remaining Joints in the joints() vector, though all JointIndexes are
  preserved. The `doomed_joint_index` given here will never be re-used.
  @param[in] doomed_joint_index The JointIndex of an existing Joint.
  @throws std::exception if the given index is out of range, refers to
    an already-removed Joint, or refers to an ephemeral Joint. */
  void RemoveJoint(JointIndex doomed_joint_index);

  /* Returns true if the given `joint_index` is in range and the corresponding
  Joint hasn't been removed. Can be a user or ephemeral Joint. */
  [[nodiscard]] bool has_joint(JointIndex joint_index) const {
    return joint_index < num_joint_indexes() &&
           data_.joint_index_to_ordinal[joint_index].has_value();
  }

  /* Returns true if the given Joint exists and is ephemeral. Ephemeral joints
  are present only if the SpanningForest is valid. */
  [[nodiscard]] bool joint_is_ephemeral(JointIndex joint_index) const {
    if (has_joint(joint_index) && joint_index > data_.max_user_joint_index) {
      DRAKE_ASSERT(index_to_ordinal(joint_index) >= num_user_joints());
      return true;
    }
    return false;
  }

  /* Returns the Link that corresponds to World (always predefined). */
  [[nodiscard]] const Link& world_link() const;

  /* Registers a joint type by name and provides the joint traits for it.
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

  /* Returns `true` if the given `joint_type_name` was previously registered
  via a call to RegisterJointType(), or is one of the pre-registered names. */
  [[nodiscard]] bool IsJointTypeRegistered(
      const std::string& joint_type_name) const;

  /* Returns a reference to the vector of traits for the known and registered
  Joint types. */
  [[nodiscard]] const std::vector<JointTraits>& joint_traits() const {
    return data_.joint_traits;
  }

  /* Returns a reference to a particular JointTraits object using one of the
  predefined indices or an index returned by RegisterJointType(). Requires a
  JointTraitsIndex, not a plain integer.*/
  [[nodiscard]] const JointTraits& joint_traits(JointTraitsIndex index) const {
    return joint_traits().at(index);
  }

  /* Returns a reference to the vector of Link objects, contiguous and ordered
  by ordinal (not by LinkIndex). World is always the first entry and is always
  present. `ssize(links())` is the number of Links currently in this graph. */
  [[nodiscard]] const std::vector<Link>& links() const { return data_.links; }

  /* Returns the number of Links currently in this graph, including World and
  ephemeral Links. This is the same as `ssize(links())`. Links that have been
  removed are not counted. */
  [[nodiscard]] inline int num_links() const;

  // TODO(sherm1) Re-think this naming strategy to use either link() or
  //  link_by_ordinal(), and similar for the other APIs.

  /* Returns a reference to a particular Link using its current ordinal
  within the links() vector. Requires a LinkOrdinal, not a plain integer. */
  [[nodiscard]] inline const Link& links(LinkOrdinal link_ordinal) const;

  /* Returns a reference to a particular Link using the index returned by
  AddLink(). The World Link is LinkIndex(0). Ephemeral links added by
  BuildForest() are indexed last. Requires a LinkIndex, not a plain integer.
  @throws std::exception if the index is out of range or if the selected Link
    was removed. */
  [[nodiscard]] inline const Link& link_by_index(LinkIndex link_index) const;

  /* Returns a reference to the vector of Joint objects, contiguous and ordered
  by ordinal (not by JointIndex). `ssize(joints())` is the number of Joints
  currently in this graph. */
  [[nodiscard]] const std::vector<Joint>& joints() const {
    return data_.joints;
  }

  /* Returns the number of Joints currently in this graph, including ephemeral
  joints. This is the same as `ssize(joints())`. Joints that have been removed
  are not counted. */
  [[nodiscard]] inline int num_joints() const;

  /* Returns a reference to a particular Joint using its current ordinal
  within the joints() vector. Requires a JointOrdinal, not a plain integer. */
  [[nodiscard]] inline const Joint& joints(JointOrdinal joint_ordinal) const;

  /* Returns a reference to a particular Joint using the index returned by
  AddJoint(). Ephemeral joints added by BuildForest() are indexed last. Requires
  a JointIndex, not a plain integer.
  @throws std::exception if the index is out of range or if the selected Joint
    was removed. */
  [[nodiscard]] inline const Joint& joint_by_index(
      JointIndex joint_index) const;

  /* Returns a reference to the vector of LoopConstraint objects. These are
  always "ephemeral" (added during forest-building) so this will return empty
  if there is no valid forest. See the class comment for more information. */
  [[nodiscard]] const std::vector<LoopConstraint>& loop_constraints() const {
    return data_.loop_constraints;
  }

  /* Returns a reference to a particular LoopConstraint. Requires a
  LoopConstraintIndex, not a plain integer.*/
  [[nodiscard]] inline const LoopConstraint& loop_constraints(
      LoopConstraintIndex constraint_index) const;

  /* Links with this ordinal or higher are "ephemeral" (added during
  forest-building). See the class comment for more information. */
  [[nodiscard]] int num_user_links() const { return data_.num_user_links; }

  /* Joints with this ordinal or higher are "ephemeral" (added during
  forest-building). See the class comment for more information. */
  [[nodiscard]] int num_user_joints() const { return data_.num_user_joints; }

  /* After the SpanningForest has been built, returns the index of the mobilized
  body (Mobod) followed by this Link. If the Link is part of an optimized
  WeldedLinksAssembly, the Mobod may be a composite body modeling other links
  in addition to the given one. If the given Link was split into a primary and
  shadows, this is the mobilized body followed by the primary. If there is no
  valid forest, the returned index will be invalid. */
  [[nodiscard]] MobodIndex link_to_mobod(LinkIndex index) const;

  /* After the SpanningForest has been built, returns the graph's maximal
  connected subgraphs of Links and Weld Joints. We call these subgraphs
  _assemblies_, by analogy with CAD assemblies. They are returned as
  WeldedLinksAssembly objects. An assembly has no internal degrees of freedom
  and will move as a single rigid body.

  The list of links in a WeldedLinksAssembly is ordered to facilitate
  post-processing computations for links that follow composite Mobods. The first
  Link entry is always the _active link_, whose non-weld inboard Joint moves the
  whole assembly. (By convention, we consider World to be the active link for
  the welded-to-World WeldedLinksAssembly.) The remaining links are ordered such
  that kinematics of all links within a composite can be determined by
  processing the links in the listed order, starting with the known kinematics
  of the active link (in general, the most-inboard link of any composite Mobod).
  See the note below for more detail.

  Modeling options affect how an assembly is modeled in the SpanningForest:
  (1) If assembly optimization is disabled, then each link is modeled by
      its own single-link Mobod with a Weld mobilizer, in the same manner as
      is done for non-assembly links. Loops are cut by splitting links as
      necessary. Reaction forces will be calculated for all the joints.
  (2) If assembly optimization is enabled and no weld joint is marked for
      special treatment, then the entire assembly is modeled with one
      composite Mobod and its weld joints are not modeled at all. No loop
      splitting occurs; weld joints that close a loop are simply added to
      the assembly and left unmodeled. No reaction forces are calculated.
  (3) If any link's weld joint has been marked "must be modeled" then that link
      begins a new Mobod with a Weld mobilizer modeling the joint. In this case
      the assembly is modeled by several Mobods, which can be a mix of
      single-link and composite Mobods. Loop splitting occurs when a _modeled_
      weld closes a loop, but not when an _unmodeled_ one does. Reaction
      forces are calculated for the modeled joints only.

  Most multibody algorithms have complexity proportional to the number of
  Mobods, so optimized assemblies can result in considerable speedups. The
  tradeoff is that joints that are not modeled will not have reaction forces
  calculated. By marking some welds "must be modeled" you can get those
  reactions and still obtain most of the advantage of optimized assemblies.

  The 0th WeldedLinksAssembly is always present (if there is a valid
  SpanningForest) and its first entry is World (Link 0), even if nothing else is
  welded to World. Otherwise, assemblies are present here only if they contain
  two or more welded links; links that aren't welded to any other links are not
  included here at all. WeldedLinksAssemblies are discovered as a side effect of
  forest-building; there is no cost to accessing them here.

  A WeldedLinksAssembly consisting only of massless Links is itself massless. If
  an assembly contains World or any massful body then it is massful. You can
  check with WeldedLinksAssembly::is_massless().

  If there is no valid forest, the returned vector is empty.

  @note (Advanced) To be precise about the order in which the links appear,
  consider a typical weld joint jᵢ interior to a WeldedLinksAssembly with active
  link A that has been modeled with a single composite Mobod. Joint jᵢ connects
  parent link pᵢ to child link cᵢ. Whichever of those links is topologically
  closer to A (typically, but not necessarily, pᵢ) will appear in the list
  before, but not necessarily adjacent to, the more-distal link. In case of a
  tie (only possible if the welds form a topological loop interior to the
  assembly) the ordering of the two links is arbitrary. That ordering
  permits intra-assembly kinematics (i.e., X_ALᵢ for the iᵗʰ link following a
  composite Mobod) to be efficiently calculated by traversing the links in
  order. (Precalculating those transforms allows for efficient post-processing
  to get link kinematics from the calculated composite Mobod kinematics.)
  Although we don't expect to report reaction forces for unmodeled joints for
  a composite Mobod, this ordering can also facilitate computing those forces
  for testing or other internal purposes: When there are no loops, it is
  possible to post-process in reverse order to calculate reaction forces for
  unmodeled joints interior to a composite body. Processed that way, each link
  will have only one unknown reaction force at the time it is encountered. */
  [[nodiscard]] const std::vector<WeldedLinksAssembly>&
  welded_links_assemblies() const {
    return data_.welded_links_assemblies;
  }

  /* Returns a reference to a particular WeldedLinksAssembly. Requires a
  WeldedLinksAssemblyIndex, not a plain integer.
  @pre `welded_links_assembly_index` is valid and in range. */
  [[nodiscard]] const WeldedLinksAssembly& welded_links_assemblies(
      WeldedLinksAssemblyIndex welded_links_assembly_index) const {
    return welded_links_assemblies().at(welded_links_assembly_index);
  }

  /* @returns `true` if a Link named `name` was added to `model_instance`.
  @see AddLink().
  @throws std::exception if `model_instance_index` is not a valid index. */
  [[nodiscard]] bool HasLinkNamed(
      std::string_view name, ModelInstanceIndex model_instance_index) const;

  /* @returns `true` if a Joint named `name` was added to `model_instance`.
  @see AddJoint().
  @throws std::exception if `model_instance_index` is not a valid index. */
  [[nodiscard]] bool HasJointNamed(
      std::string_view name, ModelInstanceIndex model_instance_index) const;

  /* If there is a Joint connecting the given Links, returns its index. You can
  call this any time and it will work with whatever Joints have been defined.
  But note that Links may be split and additional Joints added during forest
  building, so you may get a different answer before and after modeling. Cost is
  O(j) where j=min(j₁,j₂) with jᵢ the number of Joints attached to linkᵢ. */
  [[nodiscard]] std::optional<JointIndex> MaybeGetJointBetween(
      LinkIndex link1_index, LinkIndex link2_index) const;

  /* Returns true if the given Link should be treated as massless. That requires
  that the Link was flagged kMassless and is not connected by a Weld Joint to a
  massful WeldedLinksAssembly. If this Link is not part of a WeldedLinksAssembly
  then the result is the same as Link::is_massless(). */
  [[nodiscard]] bool link_and_its_assembly_are_massless(
      LinkOrdinal link_ordinal) const;

  /* (Internal use only) For testing -- invalidates the forest. */
  void ChangeLinkFlags(LinkIndex link_index, LinkFlags flags);

  /* (Internal use only) For testing -- invalidates the forest. */
  void ChangeJointFlags(JointIndex joint_index, JointFlags flags);

  /* (Internal use only) Changes the type of an existing user-defined Joint
  without making any other changes. Invalidates the forest, even if the new
  type is the same as the current type.
  @throws std::exception if called on an ephemeral (added) joint.
  @throws std::exception if an attempt is made to change a static link's joint
   to anything other than a weld.
  @pre the joint index is in range and the type name is of a registered or
    predefined joint type. */
  void ChangeJointType(JointIndex existing_joint_index,
                       const std::string& name_of_new_type);

  /* After the SpanningForest is built, returns the sequence of links from World
  to the given Link L in the forest. This is done by finding the path of Mobods
  leading to L's Mobod and reporting the sequence of links following those
  Mobods. However, if any of those are composite Mobods (because of optimized
  welded link assemblies) we report only the most-inboard link of each Mobod so
  that there is only one link returned for each level in Link L's tree. World is
  always the most-inboard link for its assembly so will always be the first
  entry in the result. However, if L follows a composite Mobod, the final entry
  will be L's Mobod's most-inboard link, which might not be L. Cost is O(ℓ)
  where ℓ is Link L's level in the SpanningForest.
  @throws std::exception if the SpanningForest hasn't been built yet.
  @see welded_link_assemblies(), SpanningForest::FindPathFromWorld() */
  std::vector<LinkIndex> FindPathFromWorld(LinkIndex link_index) const;

  /* After the SpanningForest is built, finds the first Link common to the paths
  towards World from each of two links in the forest. In case the ancestor is
  on a composite Mobod (due to the ancestor being part of a WeldedLinksAssembly
  that was optimized), the returned link will be that Mobod's most-inboard link,
  not necessarily the link that would be the common ancestor if every link had
  its own Mobod.

  Returns World immediately if the links are on different trees of the forest.
  Otherwise the cost is O(ℓ) where ℓ is the length of the longer path from one
  of the links to the ancestor.
  @throws std::exception if the SpanningForest hasn't been built yet.
  @see SpanningForest::FindFirstCommonAncestor()
  @see SpanningForest::FindPathsToFirstCommonAncestor() */
  LinkIndex FindFirstCommonAncestor(LinkIndex link1_index,
                                    LinkIndex link2_index) const;

  /* After the SpanningForest is built, finds all the Links following the forest
  subtree whose root mobilized body is the one followed by the given Link. That
  is, we look up the given Link's Mobod B and return all the links that follow
  B or any other Mobod in the subtree rooted at B. The links following B
  come first, and the rest follow the depth-first ordering of the Mobods.
  In particular, the result is _not_ sorted by LinkIndex. Computational cost
  is O(ℓ) where ℓ is the number of links following the subtree.
  @throws std::exception if the SpanningForest hasn't been built yet.
  @see SpanningForest::FindSubtreeLinks() */
  std::vector<LinkIndex> FindSubtreeLinks(LinkIndex link_index) const;

  /* After the SpanningForest is built, this method can be called to return a
  partitioning of the LinkJointGraph into subgraphs such that (a) every link is
  in one and only one subgraph, and (b) two links are in the same subgraph iff
  there is a path between them which consists only of Weld joints. Each subgraph
  of welded links is represented as a set of link indexes (using LinkIndex). By
  definition, these subgraphs will be disconnected by any non-Weld joint between
  two Links. A few notes:
    - The maximum number of returned subgraphs equals the number of links in
      the graph. This corresponds to a graph with no Weld joints.
    - The World link is included in a set of welded links, and this set is
      element zero in the returned vector. The other subgraphs are in no
      particular order.
    - The minimum number of subgraphs is one. This corresponds to a graph with
      just World or all links welded to World.

  @throws std::exception if the SpanningForest hasn't been built yet.
  @see CalcSubgraphsOfWeldedLinks() if you haven't built a forest yet
  @see welded_links_assemblies() for fast access to the welded subgraphs */
  std::vector<std::set<LinkIndex>> GetSubgraphsOfWeldedLinks() const;

  /* This much-slower method does not depend on the SpanningForest having
  already been built. It is a fallback for when there is no forest.
  @see GetSubgraphsOfWeldedLinks() if you already have a forest built */
  std::vector<std::set<LinkIndex>> CalcSubgraphsOfWeldedLinks() const;

  /* After the forest is built, returns all Links that are transitively welded,
  or rigidly affixed, to `link_index`, per these two definitions:
    1. A link is always considered welded to itself.
    2. Two unique links are considered welded together exclusively by the
       presence of Weld joints, not by other constructs such as constraints.

  Therefore, if `link_index` is a valid index to a link in this graph, then the
  returned set will always contain at least one entry storing `link_index`.
  This is fast because we just need to sort the already-calculated
  WeldedLinksAssembly the given `link_index` is part of (if any).

  @throws std::exception if the SpanningForest hasn't been built yet or
                         `link_index` is out of range
  @see CalcLinksWeldedTo() if you haven't built a forest yet */
  std::set<LinkIndex> GetLinksWeldedTo(LinkIndex link_index) const;

  /* This slower method does not depend on the SpanningForest having
  already been built. It is a fallback for when there is no forest.
  @see GetLinksWeldedTo() if you already have a forest built */
  std::set<LinkIndex> CalcLinksWeldedTo(LinkIndex link_index) const;

  // FYI Debugging APIs (including Graphviz-related) are defined in
  // link_joint_graph_debug.cc.

  /* Generate a graphviz representation of this %LinkJointGraph, with the
  given label at the top. Will include ephemeral elements if they are
  available (that is, if the forest is valid) unless suppressed explicitly.
  The result is in the "dot" language, see https://graphviz.org. If you
  write it to some file foo.dot, you can generate a viewable png (for
  example) using the command `dot -Tpng foo.dot >foo.png`.
  @see SpanningForest::GenerateGraphvizString()
  @see MakeGraphvizFiles() for an easier way to get pngs. */
  std::string GenerateGraphvizString(
      std::string_view label, bool include_ephemeral_elements = true) const;

  // TODO(sherm1) This function should be removed or reworked before upgrading
  //  to Drake public API.

  /* This is a useful debugging and presentation utility for getting
  viewable "dot diagrams" of the graph and forest. You provide a directory
  and a base name for the results. This function will generate
  `basename_graph.png` showing the graph as the user defined it. If the
  forest has been built, it will also produce `basename_graph+.png` showing
  the augmented graph with its ephemeral elements, and `basename_forest.png`
  showing the spanning forest.
  @param where The directory in which to put the files. If empty, the
    current directory is used.
  @param basename The base of the file names to be produced, see above.
  @returns the absolute path of the directory into which the files were
    created.
  @throws std::exception if files can't be created, or the `dot` command
    fails or isn't in /usr/bin, /usr/local/bin, /opt/homebrew/bin, or /bin. */
  std::filesystem::path MakeGraphvizFiles(std::filesystem::path where,
                                          std::string_view basename) const;

  // Forest building requires these joint types so they are predefined.

  /* The predefined index for the "weld" joint type's traits. */
  [[nodiscard]] static JointTraitsIndex weld_joint_traits_index() {
    return JointTraitsIndex(0);
  }

  /* The predefined index for the "quaternion_floating" joint type's traits. */
  [[nodiscard]] static JointTraitsIndex
  quaternion_floating_joint_traits_index() {
    return JointTraitsIndex(1);
  }

  /* The predefined index for the "rpy_floating" joint type's traits. */
  [[nodiscard]] static JointTraitsIndex rpy_floating_joint_traits_index() {
    return JointTraitsIndex(2);
  }

  /* Given a link index returns that link's ordinal.
  @pre the index refers to a link that exists and hasn't been removed. */
  [[nodiscard]] LinkOrdinal index_to_ordinal(LinkIndex link_index) const {
    DRAKE_ASSERT(link_index.is_valid() &&
                 link_index < ssize(data_.link_index_to_ordinal));
    const std::optional<LinkOrdinal>& ordinal =
        data_.link_index_to_ordinal[link_index];
    DRAKE_ASSERT(ordinal.has_value());
    return *ordinal;
  }

  /* Given a joint index returns that joint's ordinal.
  @pre the index refers to a joint that exists and hasn't been removed. */
  [[nodiscard]] JointOrdinal index_to_ordinal(JointIndex joint_index) const {
    DRAKE_ASSERT(joint_index.is_valid() &&
                 joint_index < ssize(data_.joint_index_to_ordinal));
    const std::optional<JointOrdinal>& ordinal =
        data_.joint_index_to_ordinal[joint_index];
    DRAKE_ASSERT(ordinal.has_value());
    return *ordinal;
  }

 private:
  friend class SpanningForest;
  friend class LinkJointGraphTester;

  [[nodiscard]] inline Link& mutable_link(LinkOrdinal link_ordinal);

  [[nodiscard]] inline Joint& mutable_joint(JointOrdinal joint_ordinal);

  // Returns the maximum number of link indices we may have (some of those
  // may have been removed).
  [[nodiscard]] int num_link_indexes() const {
    return ssize(data_.link_index_to_ordinal);
  }

  // Returns the maximum number of joint indices we may have (some of those
  // may have been removed).
  [[nodiscard]] int num_joint_indexes() const {
    return ssize(data_.joint_index_to_ordinal);
  }

  // Tells this Link which Mobod it follows and which Joint corresponds to
  // that Mobod's inboard mobilizer.
  void set_primary_mobod_for_link(LinkOrdinal link_ordinal,
                                  MobodIndex primary_mobod_index,
                                  JointIndex primary_joint_index);

  // Tells this currently-unmodeled Joint that the given Mobod models it.
  void set_mobod_for_joint(JointOrdinal joint_ordinal, MobodIndex mobod_index);

  // The World Link must already be in the graph but there are no
  // WeldedLinksAssemblies yet. This creates the 0th WeldedLinksAssembly and
  // puts World in it.
  void CreateWorldWeldedLinksAssembly();

  // Registers a loop-closing weld constraint between these Links and updates
  // the Links to know about it.
  LoopConstraintIndex AddLoopClosingWeldConstraint(
      LinkOrdinal primary_link_ordinal, LinkOrdinal shadow_link_ordinal);

  // Delegates to each MobodIndex-keeping element to renumber those indices
  // using the given map.
  // @pre old_to_new is a permutation of [0, ..., num_mobods-1].
  void RenumberMobodIndexes(const std::vector<MobodIndex>& old_to_new);

  // The forest already has the given (inboard) Mobod, and we want to add a new
  // Mobod outboard of that one to model the given Joint. At least one of the
  // Joint's two Links must already be following the inboard Mobod. That one
  // will be the inboard Link (that is, the Link following the more-inboard
  // Mobod). If that's the Joint's parent Link then parent->child and
  // inboard->outboard will match, otherwise the mobilizer must be reversed.
  // Returned tuple is: inboard link, outboard link, is_reversed.
  std::tuple<LinkOrdinal, LinkOrdinal, bool> FindInboardOutboardLinks(
      MobodIndex inboard_mobod_index, JointOrdinal joint_ordinal) const;

  // Adds the ephemeral Joint for a floating or fixed base Link to mirror a
  // mobilizer added during BuildForest(). World is the parent and the given
  // base Link is the child for the new Joint.
  JointIndex AddEphemeralJointToWorld(JointTraitsIndex type_index,
                                      LinkOrdinal child_link_ordinal);

  // Adds the new Link and Joint to the WeldedLinksAssembly of which
  // maybe_assembly_link is a member. If maybe_assembly_link is not a member of
  // any WeldedLinksAssembly, then we create a new WeldedLinksAssembly with
  // maybe_assembly_link as the first (and hence "active") Link.
  WeldedLinksAssemblyIndex AddToWeldedLinksAssembly(
      LinkOrdinal maybe_assembly_link_ordinal, LinkOrdinal new_link_ordinal,
      JointOrdinal weld_joint_ordinal);

  // While building the forest, adds a new Shadow link to the given Primary
  // link, with the Shadow mobilized by the given joint. We'll derive a name for
  // the Shadow from the Primary, create the link with appropriate bookkeeping,
  // and add a LoopConstraint (a weld) between the Primary and Shadow. The
  // Shadow link's Mobod is always terminal in the forest but the Shadow link
  // may serve as the parent link for the joint if the sense of the joint is
  // reversed from the implementing mobilizer. Note: A Drake weld constraint
  // connects a "parent" link to a "child" link; that ordering determines the
  // sign convention for its multipliers (forces). We always make the Primary
  // the weld's parent link, and the Shadow its child.
  LinkOrdinal AddShadowLink(LinkOrdinal primary_link_ordinal,
                            JointOrdinal shadow_joint_ordinal,
                            bool shadow_is_parent);

  // Finds the assigned index for a joint's traits from the joint's type name.
  // Returns nullopt if `joint_type_name` was not previously registered
  // with a call to RegisterJointType().
  std::optional<JointTraitsIndex> GetJointTraitsIndex(
      const std::string& joint_type_name) const;

  // Links that were explicitly flagged LinkFlags::kStatic in AddLink().
  const std::vector<LinkIndex>& static_link_indexes() const {
    return data_.static_link_indexes;
  }

  // Links that were flagged LinkFlags::kMustBeBaseBody in AddLink() but were
  // not also flagged kStatic (those are inherently base bodies since by
  // definition they are welded to World).
  const std::vector<LinkIndex>& non_static_must_be_base_body_link_indexes()
      const {
    return data_.non_static_must_be_base_body_link_indexes;
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

  // Notes that we didn't model this Joint in the forest because it is just a
  // weld within an existing composite Mobod. We expect that the joint has
  // already been added to the assembly; we just need to note that we decided
  // not to model it.
  void NoteUnmodeledJointInWeldedLinksAssembly(
      JointOrdinal unmodeled_joint_ordinal, WeldedLinksAssemblyIndex which);

  // Adds the joint to the assembly and notes that it is unmodeled as above.
  void AddUnmodeledJointToWeldedLinksAssembly(
      JointOrdinal unmodeled_joint_ordinal, WeldedLinksAssemblyIndex which);

  // This is the implementation for CalcLinksWeldedTo().
  void AppendLinksWeldedTo(LinkIndex link_index,
                           std::set<LinkIndex>* result) const;

  void ThrowIfForestNotBuiltYet(const char* func) const;

  [[noreturn]] void ThrowLinkWasRemoved(const char* func,
                                        LinkIndex link_index) const;

  [[noreturn]] void ThrowJointWasRemoved(const char* func,
                                         JointIndex joint_index) const;

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

    // The first entry in links is the World Link with LinkIndex(0) and name
    // world_link().name(). Ephemeral "as-built" links and joints are placed
    // at the end of these lists, following the user-supplied ones.
    // Access these lists by _ordinal_ rather than _index_.
    std::vector<Link> links;
    std::vector<Joint> joints;

    // Map link and joint indices to the corresponding ordinals for
    // elements that have not been removed. Note that an element's index is
    // persistent but its ordinal can change.
    std::vector<std::optional<LinkOrdinal>> link_index_to_ordinal;
    std::vector<std::optional<JointOrdinal>> joint_index_to_ordinal;

    // The first num_user_links etc. elements are user-supplied; the rest were
    // added during modeling.
    int num_user_links{0};
    int num_user_joints{0};

    // Records the highest index assigned to a user element. Anything higher
    // is an index temporarily assigned to ephemeral elements. Note that the
    // highest-ordinal user link or joint does not necessarily have the
    // highest-ever user index since higher ones might have been removed.
    LinkIndex max_user_link_index;
    JointIndex max_user_joint_index;

    // Loop Weld Constraints are only added during forest building; we don't
    // record any user constraints in the LinkJointGraph because they don't
    // affect how we build the forest.
    std::vector<LoopConstraint> loop_constraints;

    std::vector<LinkIndex> static_link_indexes;
    std::vector<LinkIndex> non_static_must_be_base_body_link_indexes;

    // Every user Link, organized by Model Instance.
    std::map<ModelInstanceIndex, std::vector<LinkIndex>>
        model_instance_to_link_indexes;

    // This must always have the same number of entries as joint_traits_.
    string_unordered_map<JointTraitsIndex> joint_type_name_to_index;

    // The xxx_name_to_index_ structures are multimaps because
    // links/joints/actuators/etc may appear with the same name in different
    // model instances. The index values are still unique across the graph.
    // These include only user-supplied links and joints.
    string_unordered_multimap<LinkIndex> link_name_to_index;
    string_unordered_multimap<JointIndex> joint_name_to_index;

    // The 0th assembly always exists and contains World (listed first) and any
    // links welded (recursively) to World. Other assemblies are only
    // present if there are at least two Links welded together. The first Link
    // in an assembly is the active Link.
    std::vector<WeldedLinksAssembly> welded_links_assemblies;

    bool forest_is_valid{false};  // set false whenever changes are made

    // These parallel {link,joint}_name_to_index except they hold mappings
    // for ephemeral links and joints added during forest-building. We keep
    // them separately so they are easy to get rid of.
    string_unordered_multimap<LinkIndex> ephemeral_link_name_to_index;
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
