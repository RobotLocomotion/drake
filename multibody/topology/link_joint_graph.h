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

/** Represents a graph consisting of Links (user-defined rigid bodies)
interconnected by Joints.

Terminology note: for clarity we use "Link" here to mean what MultibodyPlant
calls a "RigidBody" (or just "Body"), that is, what a user inputs as an sdf or
urdf "link", as a MuJoCo "body", or using the AddRigidBody() call in
MultibodyPlant's API. (It would have been preferable to use Link in
MultibodyPlant as well, but that ship has sailed and there is less chance of
confusion there.) The spanning forest we generate uses "mobilized bodies"
(Mobods) which can represent multiple Links, and Links may be split to create
multiple Mobods, so we're careful not to mix those up here.

%LinkJointGraph is a directed, possibly cyclic, graph defined by a sequence of
calls to AddLink() and AddJoint(). At any point, a user can ask graph specific
questions such as how Links are connected, by which Joints, or perform more
complex queries such as what sets of Links are interconnected by Weld Joints.

LinkJointGraph is intended to directly represent the user-specified elements and
connectivity of a multibody system, for example as supplied in an sdf file.
For efficient computation in Drake (using joint-space coordinates) we need to
create a model of this graph as a forest of spanning trees plus loop-closing
constraints. We call that a SpanningForest, one of which is owned by the graph.
The forest consists of "Mobods" (mobilized bodies), each representing a node of
the forest and that node's unique root-directed edge. A Mobod represents (1) a
rigid body (modeling one or more Links) and (2) that rigid body's unique inboard
mobilizer (modeling a Joint). We say a Link "follows" a Mobod if it is one of
the Links represented by that Mobod.

When we build a SpanningForest for a LinkJointGraph, the process may add
_modeling_ Links, Joints, and Constraints to the graph. Those are ephemeral and
kept distinct from _user_ Links and Joints so we can easily restore the graph to
its original condition.

In general during SpanningForest building:
  - A user-specified Link may get split into a "primary" Link and one or more
    "shadow" Links in order to break loops. Each of those has its own Mobod, so
    a user Link can generate multiple Mobods. (Geometry should remain attached
    to the primary Link.)
  - A primary Link and its shadows must be welded together by Weld Constraints
    which will be added to this Graph during modeling.
  - Building the Forest may require additional Joints to mobilize free bodies or
    immobilize static bodies. Each Joint maps to at most one Mobod; some Joints
    may instead be represented as Constraints or (for welds) implicitly as
    part of a composite rigid body.
  - Composite bodies (Links welded together) can be represented by a single
    Mobod, so many Links may follow one Mobod.
  - We never delete any of the user's Links or Joints; we may add new ones
    during Forest building but those are distinct from the user's.

@note Links are indexed using MultibodyPlant's BodyIndex type; there is no
separate LinkIndex type since these are necessarily the same. */
class LinkJointGraph {
 public:
  class Link;  // Defined in separate headers.
  class Joint;
  class Constraint;

  struct JointType;  // Defined below.

  /** Default construction defines well-known joint types and World. */
  LinkJointGraph();

  /** Copies both the graph and the forest if there is one. */
  LinkJointGraph(const LinkJointGraph& source);

  /** Assigns both the graph and the forest if there is one. */
  LinkJointGraph& operator=(const LinkJointGraph& source);

  /** Move construction leaves the source as though it had just been
  default constructed. */
  LinkJointGraph(LinkJointGraph&& source);

  /** Move assignment leaves the source as though it had just been
  default constructed. */
  LinkJointGraph& operator=(LinkJointGraph&& source);

  /** Restores this graph to its default-constructed state. */
  void Clear();

  /** Models this LinkJointGraph as a SpanningForest and returns a reference to
  the Forest. The old Forest is cleared and rebuilt. The returned reference is
  always to the same SpanningForest object; we do not create a new one when
  rebuilding.
  @see ClearForest() */
  const SpanningForest& BuildForest(
      ModelingOptions global_options = ModelingOptions::Default,
      std::map<ModelInstanceIndex, ModelingOptions> instance_options = {});

  /** Unmodels this graph and removes ephemeral modeling elements that were
  added to the graph during Forest building.
  @param[in] keep_modeling_additions
    Optionally leave modeling Links, Joints, and Constraints in place as
    though the user had added them. A subsequent BuildForest() using the same
    options should leave the graph unchanged. */
  void ClearForest(bool keep_modeling_additions = false);

  /** Has BuildForest() been called since the most recent change to this
  LinkJointGraph? */
  bool forest_is_valid() const { return data_.forest_is_valid; }

  /** Returns a reference to the internal SpanningForest whether it is
  valid or not.
  @see forest_is_valid() */
  const SpanningForest& forest() const { return *data_.forest; }

  // TODO(sherm1) Add editing functions like Remove{Link,Joint} and maybe
  //  ReplaceJoint().

  /** Adds a new Link to the graph.
  @param[in] name
    A unique name for the new Link (rigid body) in the given `model_instance`.
    Several Links can have the same name within a %LinkJointGraph as long as
    they are unique within their model instances.
  @param[in] model_instance
    The model instance to which this Link belongs, see @ref model_instance.
  @param[in] flags
    Any special handling required for this Link.
  @note The World link is always predefined with name "world", model instance
    world_model_instance(), and index 0.
  @returns The unique BodyIndex for the added Link in the graph.
  @throws std::exception if `name` is duplicated within `model_instance`. */
  BodyIndex AddLink(const std::string& name, ModelInstanceIndex model_instance,
                    LinkFlags flags = LinkFlags::Default);

  /** Adds a new Joint to the graph.
  @param[in] name
    A unique name for the new Joint in the given `model_instance`. Several
    Joints can have the same name within a %LinkJointGraph as long as they are
    unique within their model instances.
  @param[in] model_instance
    The model instance to which this joint belongs, see @ref model_instance.
  @param[in] type
    A string designating the type of this joint, such as "revolute" or
    "ball". This must be chosen from the set of joint types previously
    registered with calls to RegisterJointType().
  @param[in] parent_link_index
    This must be the index of a Link previously obtained with a call to
    AddLink(), or it must be BodyIndex(0) for the World Link.
  @param[in] child_link_index
    This must be the index of a Link previously obtained with a call to
    AddLink(), or it must be BodyIndex(0) for the World Link.
  @returns The unique JointIndex for the added joint in the graph.
  @throws std::exception if `name` is duplicated within `model_instance`.
  @throws std::exception if `type` has not been registered with
      RegisterJointType().
  @throws std::exception if `parent_link_index` or `child_link_index` are
      not valid Link indexes for this graph, if they are the same index,
      or if there is already a Joint between them.
  @throws std::exception if a static Link is connected to World by anything
      other than a Weld Joint. */
  JointIndex AddJoint(const std::string& name,
                      ModelInstanceIndex model_instance,
                      const std::string& type, BodyIndex parent_link_index,
                      BodyIndex child_link_index,
                      JointFlags flags = JointFlags::Default);

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
  @pre 0≤nq≤7, 0≤nv≤6, nv≤nq, !has_quaternion or nq≥4.

  @note LinkJointGraph is preloaded with Drake-specific Joint types it needs to
  know about including "weld", "quaternion_floating", and "rpy_floating". */
  JointTypeIndex RegisterJointType(const std::string& joint_type_name, int nq,
                                   int nv, bool has_quaternion = false);

  /** Returns `true` if the given `joint_type_name` was previously registered
  via a call to RegisterJointType(), or is one of the pre-registered names. */
  bool IsJointTypeRegistered(const std::string& joint_type_name) const;

  /** Returns a reference to the vector of known and registered joint types. */
  const std::vector<JointType>& joint_types() const {
    return data_.joint_types;
  }
  /** Returns a reference to a particular JointType using one of the
  predefined indices or an index returned by RegisterJointType. */
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

  const std::vector<Constraint>& constraints() const {
    return data_.constraints;
  }
  inline const Constraint& constraints(
      ConstraintIndex constraint_index) const;  // Defined below.

  /** Links with this index or higher were added during modeling. */
  int num_user_links() const { return data_.num_user_links; }

  /** Joints with this index or higher were added during modeling. */
  int num_user_joints() const { return data_.num_user_joints; }

  /** Constraints with this index or higher were added during modeling. */
  int num_user_constraints() const { return data_.num_user_constraints; }

  /** After the Forest has been built, returns the mobilized body (Mobod)
  followed by this Link. If the Link is part of a composite, this will be the
  mobilized body for the whole composite. If the Link was split into a primary
  and shadows, this is the mobilized body followed by the primary. */
  MobodIndex link_to_mobod(BodyIndex index) const;  // See below

  /** Links that were explicitly designated "Static" in AddLink. Links that
  are part of a Static ModelInstance are not included here unless they were
  also explicitly designated. */
  const std::vector<BodyIndex>& static_links() const {
    return data_.static_links;
  }

  /** Non-static Links that were designated "MustBeBaseBody" in AddLink. */
  const std::vector<BodyIndex>& must_be_base_body_links() const {
    return data_.must_be_base_body_links;
  }

  /** After the SpanningForest has been built, returns groups of Links that are
  welded together, which we call "Link Composites". Each group may be modeled
  in the Forest with a single mobilized body or multiple mobilized bodies
  depending on modeling options, but that doesn't change anything here. The
  first entry in each Link Composite is the "active link", the one whose
  (non-weld) Joint moves the whole Link Composite due to its modeling in the
  Spanning Forest. The rest of the Links in the composite are listed in no
  particular order.

  The 0th Link Composite is always present and its first entry is World
  (Link 0), even if nothing else is welded to World. Otherwise, composites
  are present here if they contain two or more welded Links; Links that aren't
  welded to any other Links are not included here at all. Link Composites
  are discovered as a side effect of Forest-building; there is no cost to
  accessing them here. */
  const std::vector<std::vector<BodyIndex>>& link_composites() const {
    return data_.link_composites;
  }
  const std::vector<BodyIndex>& link_composites(
      LinkCompositeIndex composite_link_index) const {
    return link_composites().at(composite_link_index);
  }

  /** @returns `true` if a Link named `name` was added to `model_instance`.
  @see AddLink().
  @throws std::exception if `model_instance` is not a valid index. */
  bool HasLinkNamed(const std::string& name,
                    ModelInstanceIndex model_instance) const;

  /** @returns `true` if a Joint named `name` was added to `model_instance`.
  @see AddJoint().
  @throws std::exception if `model_instance` is not a valid index. */
  bool HasJointNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const;

  /** If there is a Joint connecting the given Links, returns its index.
  Otherwise the returned index will be invalid. You can call this any time and
  it will work with whatever Joints have been defined. But note that Links may
  be split and additional Joints added during Forest building, so you may get a
  different answer before and after modeling. Cost is O(j) where j=min(j₁,j₂)
  with jᵢ the number of Joints attached to linkᵢ. */
  JointIndex MaybeGetJointBetween(BodyIndex link1_index,
                                  BodyIndex link2_index) const;

  /** After the Forest is built, returns the sequence of Links from World to the
  given Link L in the Forest. In case the Forest was built with a single Mobod
  for each Link Composite (Links connected by Weld joints), we only report the
  "active Link" for each Link Composite so that there is only one Link returned
  for each level in Link L's tree. World is always the active Link for its
  composite so will always be the first entry in the result. However, if L is
  part of a Link Composite the final entry will be L's composite's active link,
  which might not be L. Cost is O(ℓ) where ℓ is Link L's level in the
  SpanningForest.
  @throws std::exception if called prior to modeling
  @see link_composites(), SpanningForest::FindPathFromWorld() */
  std::vector<BodyIndex> FindPathFromWorld(BodyIndex link_index) const;

  /** After the Forest is built, finds the first Link common to the paths
  towards World from each of two Links in the SpanningForest. Returns World
  immediately if the Links are on different trees of the forest. Otherwise the
  cost is O(ℓ) where ℓ is the length of the longer path from one of the Links
  to the ancestor. */
  BodyIndex FindFirstCommonAncestor(BodyIndex link1_index,
                                    BodyIndex link2_index) const;

  /** After the Forest is built, finds all the Links following the Forest
  subtree whose root mobilized body is the one followed by the given Link. That
  is, we look up the given Link's Mobod B and return all the Links that follow
  B or any other Mobod in the subtree rooted at B. The Links following B
  come first, and the rest follow the depth-first ordering of the Mobods.
  In particular, the result is _not_ sorted by BodyIndex. Computational cost
  is O(ℓ) where ℓ is the number of Links following the subtree. */
  std::vector<BodyIndex> FindSubtreeLinks(BodyIndex link_index) const;

  /** After the Forest is built, this method can be called to return a
  partitioning of the LinkJointGraph into subgraphs such that (a) every Link is
  in one and only one subgraph, and (b) two Links are in the same subgraph iff
  there is a path between them which consists only of Weld joints.
  Each subgraph of welded Links is represented as a set of
  Link indexes (using BodyIndex). By definition, these subgraphs will be
  disconnected by any
  non-Weld joint between two Links. A few notes:
    - The maximum number of returned subgraphs equals the number of Links in
      the graph. This corresponds to a graph with no Weld joints.
    - The World Link is included in a set of welded Links, and this set is
      element zero in the returned vector. The other subgraphs are in no
      particular order.
    - The minimum number of subgraphs is one. This corresponds to a graph with
      all Links welded to World.

  @throws std::exception if the SpanningForest hasn't been built yet.
  @see CalcSubgraphsOfWeldedLinks() if you haven't built a Forest yet */
  std::vector<std::set<BodyIndex>> GetSubgraphsOfWeldedLinks() const;

  /** This much-slower method does not depend on the SpanningForest having
  already been built. It is a fallback for when there is no Forest.
  @see GetSubgraphsOfWeldedLinks() if you already have a Forest built */
  std::vector<std::set<BodyIndex>> CalcSubgraphsOfWeldedLinks() const;

  /** After the Forest is built, returns all Links that are transitively welded,
  or rigidly affixed, to `link_index`, per these two definitions:
    1. A Link is always considered welded to itself.
    2. Two unique Links are considered welded together exclusively by the
       presence of weld joints, not by other constructs such as constraints.
  Therefore, if `link_index` is a valid index to a Link in this graph, then the
  return vector will always contain at least one entry storing `link_index`.
  This is fast because we just need to sort the already-calculated
  LinkComposite the given `link_index` is part of (if any).

  @throws std::exception if the SpanningForest hasn't been built yet or
                         `link_index` is out of range */
  std::set<BodyIndex> GetLinksWeldedTo(BodyIndex link_index) const;

  /** This slower method does not depend on the SpanningForest having
  already been built. It is a fallback for when there is no Forest. */
  std::set<BodyIndex> CalcLinksWeldedTo(BodyIndex link_index) const;

  /** Returns true if the given Link should be treated as massless. That
  requires that the Link was marked TreatAsMassless and is not connected by
  a Weld Joint to a massful Link or Composite. */
  bool must_treat_as_massless(BodyIndex link_index) const;

  /** (Internal use only) For testing -- clears the Forest. */
  void change_link_flags(BodyIndex link_index, LinkFlags flags);

  /** (Internal use only) For testing -- clears the Forest. */
  void change_joint_flags(JointIndex joint_index, JointFlags flags);

  /** (Internal use only) For testing. */
  void DumpGraph(std::string title) const;

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

  inline Link& mutable_link(BodyIndex link_index);  // Defined below.

  inline Joint& mutable_joint(JointIndex joint_index);  // Defined below.

  // For use by SpanningForest.
  void set_primary_mobod_for_link(BodyIndex link_index,
                                  MobodIndex primary_mobod_index,
                                  JointIndex primary_joint_index);
  void set_mobod_for_joint(JointIndex joint_index, MobodIndex mobod_index);
  void ignore_loop_joint(JointIndex joint_index);
  void RenumberMobodIndexes(const std::vector<MobodIndex>& old_to_new);

  // While building the Forest, we're trying to add the given Joint outboard of
  // the given Mobod. At least one of the Joint's two Links must already be
  // modeled with that Mobod. That one will be the inboard Link. If that's the
  // parent Link then parent->child and inboard->outboard will match, otherwise
  // the mobilizer must be reversed. The bool return is true if we're reversing.
  // tuple is: inboard, outboard, is_reversed
  std::tuple<BodyIndex, BodyIndex, bool> FindInboardOutboardLinks(
      MobodIndex mobod_index, JointIndex joint_index) const;

  // While building the Forest, add a new Shadow Link to the given Primary Link,
  // with the Shadow mobilized by the given Joint. We'll derive a name for the
  // shadow from the primary, create the Link with appropriate bookkeeping, and
  // add a Weld Constraint between the primary and shadow (primary will be
  // Weld's "parent").
  BodyIndex AddShadowLink(BodyIndex primary_link_index,
                          JointIndex shadow_joint_index);

  ConstraintIndex AddLoopClosingWeldConstraint(BodyIndex primary_link_index,
                                               BodyIndex shadow_link_index);

  // Adds the implicit Joint for a floating or fixed base Link, with World
  // as the parent and the base Link as the child.
  JointIndex AddModelingJointToWorld(JointTypeIndex type_index,
                                     BodyIndex child_link_index);

  // Adds the new link to the composite of which the existing_link is a
  // member. If the existing_link is not a member of any composite, then we
  // create a new composite with the existing_link as the first (and hence
  // "active") link.
  LinkCompositeIndex AddToLinkComposite(BodyIndex existing_link_index,
                                        BodyIndex new_link_index);

  // The World Link must already be in the graph but there are no Composite
  // Links yet. This creates the 0th Link Composite and puts World in it.
  void CreateWorldLinkComposite();

  // Notes that we didn't model this Joint in the Forest because it is just a
  // weld to an existing Composite.
  void AddUnmodeledJointToComposite(JointIndex unmodeled_joint_index,
                                    LinkCompositeIndex which);

  // Finds the assigned index for a joint type from the type name. Returns an
  // invalid index if `joint_type_name` was not previously registered with a
  // call to RegisterJointType().
  JointTypeIndex GetJointTypeIndex(const std::string& joint_type_name) const;

  // This is the implementation for CalcLinksWeldedTo().
  void AppendLinksWeldedTo(BodyIndex link_index,
                           std::set<BodyIndex>* result) const;

  void ThrowIfForestNotBuiltYet(const char* func) const;

  // Ensure that all data members are set or restored to their
  // default-constructed condition.
  void DefaultConstruct();

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
    std::vector<Constraint> constraints;

    // The first num_user_links etc. elements are user-supplied; the rest were
    // added during modeling.
    int num_user_links{0};
    int num_user_joints{0};
    int num_user_constraints{0};

    std::vector<BodyIndex> static_links;
    std::vector<BodyIndex> must_be_base_body_links;  // but not static

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
    std::unordered_multimap<std::string, BodyIndex> model_link_name_to_index;
    std::unordered_multimap<std::string, JointIndex> model_joint_name_to_index;

    // Contains a back pointer to the graph so needs fixup post copy or move.
    copyable_unique_ptr<SpanningForest> forest;
  } data_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
