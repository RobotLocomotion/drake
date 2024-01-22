#pragma once

#ifndef DRAKE_MULTIBODY_TOPOLOGY_FOREST_INCLUDED
#error Do not include this file. Use "drake/multibody/topology/forest.h".
#endif

#include <functional>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/ssize.h"
#include "drake/multibody/topology/graph.h"

namespace drake {
namespace multibody {
// TODO(sherm1) Promote from internal once API has stabilized: issue #11307.
namespace internal {

using WeldedMobodsIndex = TypeSafeIndex<class CompositeMobodTag>;
using LoopConstraintIndex = TypeSafeIndex<class LoopConstraintTag>;

/** SpanningForest models a LinkJointGraph via a set of spanning trees and
loop-closing constraints. This is a directed forest, with edges ordered from
inboard (closer to World) to outboard. The nodes are mobilized bodies (Mobods)
containing a body and its inboard mobilizer. While building the Forest we also
study the original LinkJointGraph and update it to reflect how each of its
elements was modeled, any new elements that needed to be added for the model,
and which Links are welded together into composites with no relative
motion possible. Those should be excluded from mutual collision computations
and can (optionally) be modeled with a single mobilized body.

Problem statement

We're given a collection of directed, possibly cyclic, graphs. Some have edges
to World, others are free-floating. Nodes (links) and edges (joints) are
annotated with properties that may affect modeling. For example, a joint may be
fixed (a weld) indicating that its connected links can be combined. Links may
be massless in which case they can't be terminal nodes of a tree unless they
are also welded to something massful. Users may also have modeling preferences,
such as which nodes should be used to connect free-floating input graphs to
World, and how they should be connected (e.g. fixed or floating base).

The task here is to efficiently convert the input graph collection into a single
directed _acyclic_ graph considered as a forest of directed trees each rooted
at World, plus additional edges to restore cycles. The resulting forest is to
be optimized for computation speed and numerical accuracy, consistent with user
preferences.

These are the properties we want in the resulting forest:
  - All nodes have a path to World
  - The maximum branch length is minimized when breaking cycles
  - Welded-together nodes are combined (optionally)
  - Input parent/child edge directions are preserved when possible
  - Specific user modeling instructions are obeyed
  - Massless bodies never appear as terminal nodes in the forest
  - Each tree comprises contiguous nodes ordered depth first

Discussion

A LinkJointGraph consists of Links (user-defined rigid bodies) and Joints in a
directed, possibly cyclic graph. Link 0 is designated as the World and is
modeled by Mobod 0 in the Forest. Each Joint connects a "parent" Link to a
"child" Link. The parent/child ordering is arbitrary but sets the user's
expected sign conventions for the Joint coordinates. Those must be preserved,
even though inboard/outboard ordering may differ from parent/child. (That can
happen even in the absence of loops.) We distinguish "moving" (or
"articulated") Joints from "weld" (0 dof) Joints; every moving Joint is
modeled by a mobilizer or constraint but welds may be eliminated by creating
Link Composites that require only a single mobilizer for a group of Links.

The SpanningForest contains a Mobod node corresponding to each Link or Link
Composite (group of welded-together Links). A mobilizer connects each Mobod to
an inboard Mobod (except for World). Additional "shadow" Links and their Mobods
are created for a Link when loops are broken by cutting that Link. Every moving
Joint will map to a Mobod and there will likely be additional Mobods connecting
tree root nodes (a.k.a. "base bodies") to World via free or weld mobilizers.

When extra bodies, mobilizers, and constraints are needed to construct the
Forest, corresponding Links and Joints are added to the LinkJointGraph so a
user can view the Forest as an extended LinkJointGraph (the original graph is
not modified). For example, added 6dof mobilizers can be viewed as though they
had been floating Joints in the LinkJointGraph.

We sort the mobilized bodies into depth-first order, beginning with the 0th
Mobod corresponding to World. The data structures are designed to support fast
computation directly so that the information here does not need to be duplicated
elsewhere.

Things we get for free (O(1)) here as a side effect of building the Forest:
  - access the mobilized bodies (Mobods) in depth-first order
  - access Trees also in depth-first order and find out which Mobods belong
      to a given Tree and what range of coordinates belongs to each Tree or
      subtree
  - ask a Mobod which Tree it is in, and which coordinates it uses
  - find out which groups of Mobods are welded together so have no relative
      motion
  - ask a Mobod which WeldedMobods grouping it belongs to, if any
  - ask a coordinate (q or v) which Mobod or Tree it belongs to
  - ask for the max height of a Tree or the level of a particular Mobod
  - determine which Mobods are anchored to World, directly or indirectly
  - find out which Mobod represents a given Link or Joint
  - find all the Links that follow a particular Mobod
  - find out what Links, Joints, and Constraints appear in the Forest but not
      the source Graph.

Supported operations at minimal cost:
  - find the path from World to a given Link or Mobod
  - find the closest common ancestor of a pair of Links or Mobods
*/
class SpanningForest {
 public:
  // Constructors and assignment are private; only LinkJointGraph may access.
  // These can't be default because there are back pointers to clean up.

  class Mobod;  // Defined in separate headers.
  class Tree;
  class LoopConstraint;

  using Link = LinkJointGraph::Link;
  using Joint = LinkJointGraph::Joint;

  /** Clears the existing model and builds a new one. Any as-built information
  in the owning LinkJointGraph is removed and replaced with new as-built
  information. */
  void BuildForest(
      ModelingOptions global_options = ModelingOptions::Default,
      std::map<ModelInstanceIndex, ModelingOptions> instance_options = {});

  /** Returns the sequence of mobilized bodies from World to the given mobod B,
  inclusive of both. The 0th element is always present in the result and is
  the World (at level 0) with each entry being the Mobod at the next-higher
  level along the path to B. Cost is O(ℓ) where ℓ is B's level in its tree. */
  std::vector<MobodIndex> FindPathFromWorld(MobodIndex index) const;

  /** Finds the highest-numbered mobilized body that is common to the paths
  from each of the given ones to World. Returns World immediately if the bodies
  are on different trees; otherwise the cost is O(ℓ) where ℓ is the length
  of the longer path from one of the bodies to the ancestor. */
  MobodIndex FindFirstCommonAncestor(MobodIndex mobod1_index,
                                     MobodIndex mobod2_index) const;

  /** Finds all the Links following the Forest subtree whose root mobilized body
  B is given. That is, we return all the Links that follow B or any other Mobod
  in the subtree rooted at B. The Links following B come first, and the rest
  follow the depth-first ordering of the Mobods. In particular, the result is
  _not_ sorted by BodyIndex. Computational cost is O(ℓ) where ℓ is the number of
  Links following the subtree. */
  std::vector<BodyIndex> FindSubtreeLinks(MobodIndex root_mobod_index) const;

  /** Returns a reference to the graph that owns this Forest (as set during
  construction). */
  const LinkJointGraph& graph() const {
    DRAKE_ASSERT(data_.graph != nullptr);
    return *data_.graph;
  }

  /** Provides convenient access to the owning graph's links. */
  const std::vector<Link>& links() const { return graph().links(); }
  /** Provides convenient access to one of the owning graph's links.
  @pre link_index is in range */
  const Link& links(BodyIndex link_index) const { return links()[link_index]; }

  /** Provides convenient access to the owning graph's joints. */
  const std::vector<Joint>& joints() const { return graph().joints(); }
  /** Provides convenient access to one of the owning graph's joints.
  @pre joint_index is in range */
  const Joint& joints(JointIndex joint_index) const {
    return joints()[joint_index];
  }

  /** All the mobilized bodies, in depth-first order. World comes first,
  then every Mobod in tree 0, then every Mobod in tree 1, etc. Free bodies
  that weren't explicitly connected to World by a Joint come last. */
  const std::vector<Mobod>& mobods() const { return data_.mobods; }
  /** Provides convenient access to a particular Mobod.
  @pre mobod_index is in range */
  inline const Mobod& mobods(MobodIndex mobod_index) const;  // Defined below.

  /** The mobilized body (Mobod) corresponding to the World Link. */
  const Mobod& world_mobod() const { return mobods(MobodIndex(0)); }

  /** Constraints we added to close loops we had to cut. */
  const std::vector<LoopConstraint>& loop_constraints() const {
    return data_.loop_constraints;
  }
  inline const LoopConstraint& loop_constraints(
      LoopConstraintIndex index) const;  // Defined below.

  /** The partitioning of the forest of mobilized bodies into trees. Each Tree
  has a base (root) mobilized body that is connected directly to the World
  Mobod (which may represent a Link Composite). World is not considered to
  be part of any Tree; it is the root of the Forest. */
  const std::vector<Tree>& trees() const { return data_.trees; }
  /** Provides convenient access to a particular Tree.
  @pre tree_index is in range */
  inline const Tree& trees(TreeIndex tree_index) const;  // Defined below.

  /** Returns the height of the Forest, defined as the height of the tallest
  Tree in the Forest, plus 1 for World. This is always at least 1 since World
  is always present. */
  int height() const { return data_.forest_height; }

  /** Returns precalculated groups of mobilized bodies that are mutually
  interconnected by Weld mobilizers so have no relative degrees of freedom.
  Note that if you have chosen the modeling option to combine welded-together
  Links into single bodies, then each Composite Link gets only a single Mobod
  and hence there won't be any WeldedMobods groups here (except for World which
  is always considered to be in a WeldedMobods group even if nothing is welded
  to it). Use mobod_to_links() to find all the Links following a single Mobod.

  The World WeldedMobods group comes first and contains World, mobilized bodies
  representing Links marked "Static", and bodies (if any) welded to Static
  bodies or World (recursively). Those are the "anchored" mobilized bodies. The
  other groups represent sets of welded-together mobilized bodies, with the
  first one in the group the only Mobod with a non-weld inboard mobilizer.
  That moving Mobod is necessarily the lowest numbered (most inboard) Mobod
  of the WeldedMobods group. The remaining Mobods are in no particular order.

  Except for World, Mobods not welded to any other Mobods do not appear here.
  @see mobod_to_links() */
  const std::vector<std::vector<MobodIndex>>& welded_mobods() const {
    return data_.welded_mobods;
  }
  const std::vector<MobodIndex>& welded_mobods(WeldedMobodsIndex index) const {
    return welded_mobods()[index];
  }

  /** Returns the global ModelingOptions that were used when this model was last
  built with BuildForest(). You can also provide a ModelInstanceIndex to see
  ModelingOptions specific to that Model Instance. */
  ModelingOptions options() const { return data_.global_modeling_options; }

  /** Returns the ModelingOptions that were last used for elements of the given
  ModelInstance. If we don't have specific options for this instance, we
  return the global ModelingOptions as returned by options(). */
  ModelingOptions options(ModelInstanceIndex index) const {
    const auto instance_options = data_.instance_modeling_options.find(index);
    return instance_options == data_.instance_modeling_options.end()
               ? data_.global_modeling_options
               : instance_options->second;
  }

  /** Returns the Link that is represented by the given Mobod. This could be
  one of the Links from the original graph or an added shadow Link. If this
  Mobod represents a Link Composite, the Link returned here is the
  "active" Link, that is, the one whose mobilizer is used to move the whole
  Composite. Cost is O(1) and very fast. */
  inline BodyIndex mobod_to_link(
      MobodIndex mobod_index) const;  // Defined below.

  /** Returns all the Links mobilized by this Mobod. The "active" Link returned
  by mobod_to_link() comes first, then any other Links in the same Composite. */
  inline const std::vector<BodyIndex>& mobod_to_links(
      MobodIndex mobod_index) const;  // Defined below.

  /** Returns the total number of generalized position coordinates q used by
  this model. O(1), very fast. */
  int num_positions() const { return ssize(data_.q_to_mobod); }

  /** Returns the total number of generalized velocity coordinates v used by
  this model. O(1), very fast. */
  int num_velocities() const { return ssize(data_.v_to_mobod); }

  /** Returns the Mobod to which a given position coordinate q belongs.
  O(1), very fast. */
  MobodIndex q_to_mobod(int q_index) const {
    DRAKE_ASSERT(0 <= q_index && q_index < num_positions());
    return data_.q_to_mobod[q_index];
  }

  /** Returns the Mobod to which a given velocity coordinate v belongs.
  O(1), very fast. */
  MobodIndex v_to_mobod(int v_index) const {
    DRAKE_ASSERT(0 <= v_index && v_index < num_velocities());
    return data_.v_to_mobod[v_index];
  }

  /** Returns the Tree to which a given position coordinate q belongs.
  O(1), very fast. */
  inline TreeIndex q_to_tree(int q_index) const;  // Defined below.

  /** Returns the Tree to which a given velocity coordinate v belongs.
  O(1), very fast. */
  inline TreeIndex v_to_tree(int v_index) const;  // Defined below.

  /** (Debugging, Testing) Runs a series of expensive tests to see that the
  Forest is internally consistent and aborts if not. */
  void SanityCheckForest() const;

  /** (Debugging) Produces a human-readable summary of this Forest. */
  void DumpForest(std::string title) const;

 private:
  friend class LinkJointGraph;
  friend class copyable_unique_ptr<SpanningForest>;

  /* Only the owning LinkJointGraph may call this constructor. The owner must
  outlive this model, which may be built and rebuilt repeatedly for the same
  owner. For copy and move, the owner is responsible for repairing the back
  pointer via SetNewOwner().
  @pre `graph` is non-null */
  explicit SpanningForest(LinkJointGraph* graph) {
    DRAKE_DEMAND(graph != nullptr);
    data_.graph = graph;
  }

  // The caller (only LinkJointGraph) must provide a new back pointer after copy
  // or move so these can't be default.
  SpanningForest(const SpanningForest& source);
  SpanningForest(SpanningForest&& source);
  SpanningForest& operator=(const SpanningForest& source);
  SpanningForest& operator=(SpanningForest&& source);

  // LinkJointGraph uses this to fix the graph back pointer after copy or move.
  void SetNewOwner(LinkJointGraph* graph) {
    DRAKE_DEMAND(graph != nullptr);
    data_.graph = graph;
  }

  // After copy/move/assign: any element that has a pointer back to its owning
  // SpanningForest should replace the pointer with `this`.
  void FixInternalPointers();

  // Restores this SpanningTree to the state it has immediately after
  // construction. The owning LinkJointGraph remains unchanged.
  void Clear();

  // Produce the optimal forest, but with suboptimal node ordering.
  void ChooseForestTopology();

  // Determine proposed depth-first reordering.
  // @retval old_to_new
  std::vector<MobodIndex> CreateDepthFirstReordering() const;

  // Update the forest to reorder the Mobods into depth-first order, and
  // update the as-modeled information in the graph to match.
  void FixupForestToUseNewNumbering(const std::vector<MobodIndex>& old_to_new);

  // Once we have a depth-first ordering we can assign q's and v's.
  void AssignCoordinates();

  // We're given a set of Joints for which one if its Links has already been
  // modeled by some Mobod. Grow the trees by modeling these Joints and the
  // other Link they connect. Then continue growing in a breadth-first manner,
  // one level at a time.
  void ExtendTrees(const std::vector<JointIndex>& joints_to_model,
                   int* num_unprocessed_links);

  // Grow the trees containing each of the given Joints by one level.
  // Returns the set of Joints that should be modeled next.
  void ExtendTreesOneLevel(const std::vector<JointIndex>& joints_to_model,
                           int* num_unprocessed_links,
                           std::vector<JointIndex>* joints_to_model_next);

  void ChooseBaseBodiesAndAddTrees(int* num_unprocessed_links);

  const Mobod& AddNewMobod(BodyIndex outboard_link_index,
                           JointIndex joint_index,
                           MobodIndex inboard_mobod_index, bool is_reversed);

  const Mobod& AddShadowMobod(BodyIndex outboard_link_index,
                              JointIndex joint_index);

  // Adds the follower Link to the LinkComposite that inboard_mobod is
  // mobilizing and notes that the Joint is internal to that LinkComposite
  // so is not modeled. Will create the LinkComposite if there was only one
  // Link mobilized before.
  const Mobod& JoinExistingMobod(Mobod* inboard_mobod,
                                 BodyIndex follower_link_index,
                                 JointIndex weld_joint_index);

  // Greedily extend this Mobod by recursively following the given weld Joint
  // to include all the Links welded to this Mobod's Link. As we encounter
  // non-weld Joints attached to this composite we record them in
  // `outboard_joints` for processing next.
  void GrowCompositeMobod(Mobod* inboard_mobod, BodyIndex follower_link_index,
                          JointIndex weld_joint_index,
                          std::vector<JointIndex>* outboard_joints,
                          int* num_unprocessed_links);

  // This not-yet-modeled Joint connects two Links both of which are already
  // modeled in the Forest. We will model the Joint either directly as a
  // Constraint or by splitting off a shadow of one of the Links and
  // mobilizing the shadow with a forward or reversed Mobilizer of the Joint's
  // type. Then we add a Weld Constraint to attach the shadow to its primary.
  // Some details:
  //  - we can only use a direct constraint if both Links are massful
  //  - if one is massless, split the other one
  //  - if both are massless we have an invalid forest
  //  - either or both Links may be composites; it is the mass properties
  //    of the whole composite that determines masslessness.
  void HandleLoopClosure(JointIndex loop_joint_index);

  void ModelLoopJointAsConstraint(JointIndex joint_index) {
    // TODO(sherm1) Write this; ignoring for now.
    mutable_graph().ignore_loop_joint(joint_index);
  }

  // Given a list of Static or MustBeBaseBody Links, adds a weld or floating
  // Joint to World for each Link that doesn't already have one.
  void ConnectLinksToWorld(const std::vector<BodyIndex>& links, bool use_weld);

  void SetBaseBodyChoicePolicy();

  void DumpForestImpl(MobodIndex mobod = MobodIndex(0), int level = 0) const;

  bool model_instance_is_static(ModelInstanceIndex index) const {
    return static_cast<bool>(options(index) & ModelingOptions::Static);
  }

  bool use_fixed_base(ModelInstanceIndex index) const {
    return static_cast<bool>(options(index) & ModelingOptions::UseFixedBase);
  }

  bool use_rpy_floating_joint(ModelInstanceIndex index) const {
    return static_cast<bool>(options(index) &
                             ModelingOptions::UseRpyFloatingJoints);
  }

  bool combine_composite_links(ModelInstanceIndex index) const {
    return static_cast<bool>(options(index) &
                             ModelingOptions::CombineLinkComposites);
  }

  // Is this Joint a Weld, and if so should we bury it in a Composite?
  // Note that the parent link, child link, and joint could all be in different
  // model instances. If _any_ of those asks for combining into Composites we'll
  // say yes, unless the Joint itself overrides that.
  bool should_be_unmodeled_weld(const Joint& joint) {
    if (!joint.is_weld() || joint.must_be_modeled()) return false;
    return combine_composite_links(joint.model_instance()) ||
           combine_composite_links(
               links(joint.parent_link()).model_instance()) ||
           combine_composite_links(links(joint.child_link()).model_instance());
  }

  // Returns the appropriate joint type to use for this model instance when
  // attaching a base body to World. Can be fixed or floating, and floating
  // can be rpy or quaternion, depending on modeling options.
  JointTypeIndex base_joint_type_index(
      ModelInstanceIndex model_instance_index) const {
    if (use_fixed_base(model_instance_index))
      return LinkJointGraph::weld_joint_type_index();
    return use_rpy_floating_joint(model_instance_index)
               ? LinkJointGraph::rpy_floating_joint_type_index()
               : LinkJointGraph::quaternion_floating_joint_type_index();
  }

  bool link_is_already_in_forest(BodyIndex link_index) const {
    return graph().link_to_mobod(link_index).is_valid();
  }

  LinkJointGraph& mutable_graph() {
    DRAKE_ASSERT(data_.graph != nullptr);
    return *data_.graph;
  }

  struct Data {
    // These are all default but definitions deferred to .cc file so
    // that all the local classes are defined (Mac requirement).
    Data();
    Data(const Data&);
    Data(Data&&);
    ~Data();
    Data& operator=(const Data&);
    Data& operator=(Data&&);

    LinkJointGraph* graph{};  // The graph we're modeling.
    ModelingOptions global_modeling_options{ModelingOptions::Default};
    std::map<ModelInstanceIndex, ModelingOptions> instance_modeling_options;

    // A SpanningForest always has at least one mobilized body: World.
    std::vector<Mobod> mobods;

    // If we had to break any loops we'll add constraints to re-close them.
    std::vector<LoopConstraint> loop_constraints;

    // How the forest is partitioned into trees.
    std::vector<Tree> trees;

    // The height of the tallest tree in the forest. Always at least 1 when
    // valid since World is always present in a forest.
    int forest_height{0};

    // Composite Mobods. These are groups of Mobods that have no relative motion
    // due to _modeled_ weld Joints. (If we are building Composite Links, there
    // won't be any Composite Mobods because we'll make all welded-together
    // Links follow a single Mobod.) The World composite, if there is one,
    // appears first here. Must be renumbered by FixupForestToUseNewNumbering().
    std::vector<std::vector<MobodIndex>> welded_mobods;

    // Map from coordinates to their associated mobilized bodies. These are
    // filled in late with the post-renumbered MobodIndex values so should
    // NOT be renumbered by FixupForestToUseNewNumbering().
    std::vector<MobodIndex> q_to_mobod;  // size is nq
    std::vector<MobodIndex> v_to_mobod;  // size is nv

    // This policy is expressed as a "less than" comparator of the type used by
    // std::priority_queue. It should return true if the left argument is a
    // worse choice than the right argument, according to the policy.
    std::function<bool(const BodyIndex&, const BodyIndex&)> base_body_policy;
  } data_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
