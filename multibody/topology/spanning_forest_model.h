#pragma once

#include <functional>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/ssize.h"
#include "drake/multibody/topology/link_joint_graph.h"

namespace drake {
namespace multibody {
namespace internal {

using TreeIndex = TypeSafeIndex<class TreeTag>;
using CompositeMobodIndex = TypeSafeIndex<class CompositeMobodTag>;
using LoopConstraintIndex = TypeSafeIndex<class LoopConstraintTag>;

/** SpanningForestModel models a LinkJointGraph via a set of spanning trees and
loop-closing constraints. This is a directed forest, with edges ordered from
inboard (closer to World) to outboard. The nodes are mobilized bodies (Mobods)
containing a body and its inboard mobilizer. While building the Forest we also
study the original LinkJointGraph and update it to reflect how each of its
elements was modeled, any new elements that needed to be added for the model,
and which Links are welded together into composites with no relative
motion possible. Those should be excluded from mutual collision computations
and can be modeled with a single mobilized body.

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
  - Welded-together nodes are combined into composites
  - Input parent/child edge directions are preserved when possible
  - Specific user modeling instructions are obeyed
  - Massless bodies never appear as terminal nodes in the forest
  - Each tree comprises contiguous nodes ordered depth first

Discussion

A LinkJointGraph consists of Links (bodies) and Joints in a directed, possibly
cyclic graph. Link 0 is designated as the World and is modeled by Mobod 0 in
the Forest. Each Joint connects a "parent" Link to a "child" Link. The
parent/child ordering is arbitrary but sets the user's expected sign conventions
for the Joint coordinates. Those must be preserved, even though inboard/outboard
ordering may differ from parent/child. (That can happen even in the absence of
loops.) We distinguish "moving" (or "articulated") Joints from "weld" (0 dof)
Joints; every moving Joint is modeled by a mobilizer or constraint but welds
may be eliminated by creating Composites that require only a single mobilizer
for a group of Links.

The SpanningForestModel contains a Mobod node corresponding to each Link or
Composite (group of welded-together Links). A mobilizer connects each Mobod to
an inboard Mobod (except for World). Additional "shadow" Links and their Mobods
are created for a Link when loops are broken by cutting that Link. Every moving
Joint will map to a Mobod and there will likely be additional Mobods connecting
tree root nodes (a.k.a. "base bodies") to World via free or weld mobilizers.

When extra bodies, mobilizers, and constraints are needed to construct the
Forest, corresponding Links and Joints are added to the LinkJointGraph so a
user can view the Forest as an extended LinkJointGraph (the original graph is
not modified).

We sort the mobilized bodies into depth-first order, beginning with the 0th
Mobod corresponding to World. The data structures are designed to support fast
computation directly so that the information here does not need to be duplicated
elsewhere.

Things we get for free (O(1)) here as a side effect of building the Forest:
  - access the mobilized bodies (Mobods) in depth-first order
  - access Trees also in depth-first order and find out which Mobods belong
      to a given Tree and what range of coordinates belongs to each Tree
  - ask a Mobod which Tree it is in, and which coordinates it uses
  - ask a Mobod which Composite it belongs to, if any
  - ask a coordinate (q or v) which Mobod or Tree it belongs to
  - ask for the max height of a Tree or the level of a particular Mobod
  - determine which Mobods are anchored to World, directly or indirectly
  - find out which Mobods are welded together so could be treated as composites
  - find out which Mobod represents a given Link or Joint
  - find out what Links, Joints, and Constraints appear in the Forest but not
      the source Graph.

Supported operations at minimal cost:
  - find the path from a Link or Mobod down to World
  - find the closest common ancestor of a pair of Links or Mobods
  - is there a Joint between two Links?
  - what Mobods or coordinates are outboard (affected by) a given Mobod
*/
class SpanningForestModel {
 public:
  // Constructors and assignment are private; only LinkJointGraph may access.
  // These can't be default because there are back pointers to clean up.

  class Tree;
  class Mobod;
  class LoopConstraint;

  using Link = LinkJointGraph::Link;
  using Joint = LinkJointGraph::Joint;

  /** Clears the existing model and builds a new one. Any as-built information
  in the owning LinkJointGraph is removed and replaced with new as-built
  information. */
  void BuildForest(
      ModelingOptions global_options = ModelingOptions::Default,
      std::map<ModelInstanceIndex, ModelingOptions> instance_options = {});

  /** Returns the sequence of mobilized bodies from the given one to World,
  inclusive of both. Cost is O(ℓ) where ℓ is the given body's level in
  its tree. */
  std::vector<MobodIndex> FindPathToWorld(MobodIndex index) const;

  /** Finds the highest-numbered mobilized body that is common to the paths
  from each of the given ones to World. Returns World immediately if the bodies
  are on different trees; otherwise the cost is O(ℓ) where ℓ is the length
  of the longer path from one of the bodies to the ancestor. */
  MobodIndex FindFirstCommonAncestor(MobodIndex mobod1_index,
                                     MobodIndex mobod2_index) const;


  /** Given a Mobod B, returns the velocity coordinates for the subtree rooted
  at B, _including_ B's velocities. Because velocities are assigned depth-first,
  all the velocities in a subtree are consecutive so we need only return the
  index of the first one and the number of velocities. We return the pair [i, n]
  where i is the index of B's first velocity coordinate vᵢ and n is the number
  of subtree velocity coordinates. So the subtree coordinates are [vᵢ..vᵢ₊ₙ).
  Performance is O(1) if B is World or a base body, otherwise we have to follow
  the highest-numbered branch in expected O(log(n)) time, worst case O(n). */
  std::pair<int, int> FindSubtreeVelocities(MobodIndex mobod_index) const;

  /** Returns the velocity coordinates v that are outboard
  of the given Mobod, _not including_ its own velocities. See
  FindSubtreeVelocities() for performance and return semantics. */
  std::pair<int, int> FindOutboardVelocities(MobodIndex mobod_index) const;

  /** Returns a reference to the graph that owns this model (as set during
  construction). */
  const LinkJointGraph& graph() const {
    DRAKE_ASSERT(data_.graph != nullptr);
    return *data_.graph;
  }

  /** Provides convenient access to the owning graph's links. */
  const std::vector<Link>& links() const { return graph().links(); }
  /** Provides convenient access to one of the owning graph's links.
  @pre link_index is in range */
  const Link& links(LinkIndex link_index) const { return links()[link_index]; }

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
  const Mobod& mobods(MobodIndex mobod_index) const {
    return mobods()[mobod_index];
  }
  /** The mobilized body (Mobod) corresponding to the World Link. */
  const Mobod& world_mobod() const { return mobods(MobodIndex(0)); }

  /** Constraints we added to close loops we had to cut. */
  const std::vector<LoopConstraint>& loop_constraints() const {
    return data_.loop_constraints;
  }
  const LoopConstraint& loop_constraints(LoopConstraintIndex index) const {
    return loop_constraints()[index];
  }

  /** The partitioning of the forest of mobilized bodies into trees. Each Tree
  has a base (root) mobilized body that is connected directly to the World
  Mobod (which may represent a Composite of Links). World is not considered to
  be part of any Tree; it is the root of the Forest. */
  const std::vector<Tree>& trees() const { return data_.trees; }
  /** Provides convenient access to a particular Tree.
  @pre tree_index is in range */
  const Tree& trees(TreeIndex tree_index) const { return trees()[tree_index]; }

  /** Groups of mobilized bodies that are connected by Weld mobilizers so have
  no relative degrees of freedom.  Note that if we are modeling welded-together
  Links as single Composite Links then each composite gets only a single Mobod
  and hence there won't be any composite Mobods here. Use mobod_to_links() to
  find all the Links following a single Mobod.

  If anything is welded to World, that Composite Mobod comes first and contains
  World, mobilized bodies representing Links marked "Static", and bodies (if
  any) welded to Static bodies or World (recursively). Those are the "anchored"
  mobilized bodies. The other groups represent sets of welded-together mobilized
  bodies. Mobods that are not welded to any other Mobods do not appear here.

  @see mobod_to_links() */
  const std::vector<std::vector<MobodIndex>>& composite_mobods() const {
    return data_.composite_mobods;
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
  Mobod represents a Composite of many Links, the Link returned here is the
  "representative" Link, that is, the one whose mobilizer is used for the whole
  Composite. Cost is O(1) and very fast. */
  inline LinkIndex mobod_to_link(MobodIndex mobod_index) const;  // see below

  /** Returns all the Links mobilized by this Mobod. The "representative"
  Link returned by mobod_to_link() comes first, then any other Links in the
  same Composite. */
  inline const std::vector<LinkIndex>& mobod_to_links(
      MobodIndex mobod_index) const;  // see below

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
  inline TreeIndex q_to_tree(int q_index) const;  // See below

  /** Returns the Tree to which a given velocity coordinate v belongs.
  O(1), very fast. */
  inline TreeIndex v_to_tree(int v_index) const;  // See below

  /** (Debugging, Testing) Runs a series of expensive tests to see that the
  Forest is internally consistent and aborts if not. */
  void SanityCheckForest() const;

  /** (Debugging) Produces a human-readable summary of this Forest. */
  void DumpModel(std::string title) const;

 private:
  friend class LinkJointGraph;
  friend class copyable_unique_ptr<SpanningForestModel>;

  /* Only the owning LinkJointGraph may call this constructor. The owner must
  outlive this model, which may be built and rebuilt repeatedly for the same
  owner. For copy and move, the owner is responsible for repairing the back
  pointer via SetNewOwner().
  @pre `graph` is non-null */
  explicit SpanningForestModel(LinkJointGraph* graph) {
    DRAKE_DEMAND(graph != nullptr);
    data_.graph = graph;
  }

  // The caller (only LinkJointGraph) must provide a new back pointer after copy
  // or move so these can't be default.
  SpanningForestModel(const SpanningForestModel& source);
  SpanningForestModel(SpanningForestModel&& source);
  SpanningForestModel& operator=(const SpanningForestModel& source);
  SpanningForestModel& operator=(SpanningForestModel&& source);

  // LinkJointGraph uses this to fix the graph back pointer after copy or move.
  void SetNewOwner(LinkJointGraph* graph) {
    DRAKE_DEMAND(graph != nullptr);
    data_.graph = graph;
  }

  // After copy/move/assign: any element that has a pointer back to its owning
  // SpanningForestModel should replace the pointer with `this`.
  void FixInternalPointers();

  // Restores this SpanningTree the state it has immediately after
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

  const Mobod& AddNewMobod(LinkIndex outboard_link_index,
                           JointIndex joint_index,
                           MobodIndex inboard_mobod_index, bool is_reversed);

  const Mobod& AddShadowMobod(LinkIndex outboard_link_index,
                              JointIndex joint_index);

  // Adds the follower Link to the CompositeLink that inboard_mobod is
  // mobilizing and notes that the Joint is included in that CompositeLink
  // so is not modeled. Will create the CompositeLink if there was only one
  // Link mobilized before.
  const Mobod& JoinExistingMobod(Mobod* inboard_mobod,
                                 LinkIndex follower_link_index,
                                 JointIndex weld_joint_index);

  // Greedily extend this Mobod by recursively following the given weld Joint
  // to include all the Links welded to this Mobod's Link. As we encounter
  // non-weld Joints attached to this composite we record them in
  // `outboard_joints` for processing next.
  void GrowCompositeMobod(Mobod* inboard_mobod, LinkIndex follower_link_index,
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
  void ConnectLinksToWorld(const std::vector<LinkIndex>& links, bool use_weld);

  void SetBaseBodyChoicePolicy();

  void DumpModelImpl(MobodIndex mobod = MobodIndex(0), int level = 0) const;

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
                             ModelingOptions::CombineCompositeLinks);
  }

  // Is this Joint a Weld?
  bool is_weld(const Joint& joint) {
    return joint.type_index() == graph().weld_type_index();
  }

  // Is this Joint a Weld, and if so should we bury it in a Composite?
  // Note that the parent link, child link, and joint could all be in different
  // model instances. If _any_ of those asks for combining into Composites we'll
  // say yes, unless the Joint itself overrides that.
  bool should_be_unmodeled_weld(const Joint& joint) {
    if (!is_weld(joint) || joint.must_be_modeled()) return false;
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
    if (use_fixed_base(model_instance_index)) return graph().weld_type_index();
    return use_rpy_floating_joint(model_instance_index)
               ? graph().rpy_floating_type_index()
               : graph().quaternion_floating_type_index();
  }

  bool link_is_already_in_forest(LinkIndex link_index) const {
    return graph().link_to_mobod(link_index).is_valid();
  }

  LinkJointGraph& mutable_graph() {
    DRAKE_ASSERT(data_.graph != nullptr);
    return *data_.graph;
  }

  struct Data {
    LinkJointGraph* graph{};  // The graph we're modeling.
    ModelingOptions global_modeling_options{ModelingOptions::Default};
    std::map<ModelInstanceIndex, ModelingOptions> instance_modeling_options;

    // A SpanningForestModel always has at least one mobilized body: World.
    std::vector<Mobod> mobods;

    // If we had to break any loops we'll add constraints to re-close them.
    std::vector<LoopConstraint> loop_constraints;

    // How the forest is partitioned into trees.
    std::vector<Tree> trees;

    // Composite Mobods. These are groups of Mobods that have no relative motion
    // due to _modeled_ weld Joints. (If we are building Composite Links, there
    // won't be any Composite Mobods because we'll make all welded-together
    // Links follow a single Mobod.) The World composite, if there is one,
    // appears first here. Must be renumbered by FixupForestToUseNewNumbering().
    std::vector<std::vector<MobodIndex>> composite_mobods;

    // Map from coordinates to their associated mobilized bodies. These are
    // filled in late with the post-renumbered MobodIndex values so should
    // NOT be renumbered by FixupForestToUseNewNumbering().
    std::vector<MobodIndex> q_to_mobod;  // size is nq
    std::vector<MobodIndex> v_to_mobod;  // size is nv

    // This policy is expressed as a "less than" comparator of the type used by
    // std::priority_queue. It should return true if the left argument is a
    // worse choice than the right argument, according to the policy.
    std::function<bool(const LinkIndex&, const LinkIndex&)> base_body_policy;
  } data_;
};

//================================= Mobod ======================================
/** Everything you might want to know about a mobilized body.
 */
class SpanningForestModel::Mobod {
 public:
  bool is_world() const { return level_ == 0; }
  bool is_base_body() const { return level_ == 1; }

  /** A mobod is "anchored" if it is the World mobod or if it is a mobod
  directly or indirectly connected to the World mobod by Weld mobilizers. */
  bool is_anchored() const {
    return is_world() || (composite_mobod_index_.is_valid() &&
                          composite_mobod_index_ == CompositeMobodIndex(0));
  }

  bool is_terminal_body() const { return outboard_mobods_.empty(); }
  bool is_reversed() const { return use_reverse_mobilizer_; }

  MobodIndex index() const { return index_; }
  MobodIndex inboard() const { return inboard_mobod_; }
  const std::vector<MobodIndex>& outboards() const { return outboard_mobods_; }

  /** Returns the index of the Link mobilized by this Mobod. For a Composite
  Link, this is the "representative" Link, the one whose Joint connects the
  whole Composite Link to its inboard Mobod. */
  LinkIndex link() const { return follower_links()[0]; }

  /** Returns all the Links of a Composite Link that are mobilized by this
  Mobod. The first one is the "representative" Link as returned by link(). There
  is always at least one Link. */
  const std::vector<LinkIndex>& follower_links() const {
    DRAKE_ASSERT(!follower_links_.empty());
    return follower_links_;
  }

  JointIndex joint() const { return joint_index_; }
  TreeIndex tree() const { return tree_index_; }
  CompositeMobodIndex composite() const { return composite_mobod_index_; }
  int level() const { return level_; }

  /** Starting offset within the contiguous q vector. */
  int q_start() const { return q_start_; }
  int nq() const { return nq_; }

  /** Start offset within the contiguous v vector. */
  int v_start() const { return v_start_; }
  int nv() const { return nv_; }

  /** (Internal use only) This constructor is just for World, the only Mobod
  with no mobilizer. (Or you can consider it welded to the universe at the]
  World origin.) */
  Mobod(MobodIndex mobod_index, LinkIndex world_link);

  /** (Internal use only) The general constructor for anything but
  shadow bodies. */
  Mobod(MobodIndex mobod_index, LinkIndex link_index, JointIndex joint_index,
        int level, bool is_reversed);

  /** (Internal use only) Copy/Move constructor & assignment. */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Mobod)

 private:
  friend class SpanningForestModel;

  // Update all MobodIndex entries to use the new numbering.
  void FixupAfterReordering(const std::vector<MobodIndex>& old_to_new);

  // Switch the contents of `this` and `other` mobilized bodies.
  void Swap(Mobod& other);

  // Given a mapping from old MobodIndex to new MobodIndex, repair an
  // existing vector of old MobodIndexes to use the new numbering. Any invalid
  // indexes are left untouched.
  static void RenumberMobodIndexVector(
      const std::vector<MobodIndex>& old_to_new,
      std::vector<MobodIndex>* to_be_renumbered);

  // CAREFUL: if you add any members here, update Swap!

  // Links represented by this Mobod. The first one is always present and is
  // the representative Link if we're mobilizing a Composite.
  std::vector<LinkIndex> follower_links_;

  // Corresponding Joint (user or modeling joint). In case of Composites, this
  // is the Joint that mobilizes the whole Composite.
  JointIndex joint_index_;

  // For an already-existing Joint, must we use a reverse mobilizer? If true,
  // we know tree inboard/outboard order is opposite graph parent/child order.
  bool use_reverse_mobilizer_{false};

  int level_{-1};  // Path length from World

  // These references must be renumbered when we reorder the mobods.
  MobodIndex index_;          // Index of this mobilized body.
  MobodIndex inboard_mobod_;  // Tree parent at level-1 (invalid for World)
  std::vector<MobodIndex> outboard_mobods_;  // Tree children at level+1

  TreeIndex tree_index_;  // Which Tree is this Mobod part of?
  CompositeMobodIndex composite_mobod_index_;  // Part of a CompositeMobod?

  // Coordinate assignments (done last). For welds, q/v_start is still set to
  // where coordinates would have started if there were any, with nq/v==0.
  int q_start_{-1};  // within the full q vector
  int nq_{-1};
  int v_start_{-1};
  int nv_{-1};  // within the full v vector
};

//============================= Loop Constraint ================================
/** Weld constraints added during modeling to close loops. */
class SpanningForestModel::LoopConstraint {
 public:
  LoopConstraint(MobodIndex parent_mobod_index, MobodIndex child_mobod_index,
                 ConstraintIndex graph_constraint_index)
      : parent_mobod_index_(parent_mobod_index),
        child_mobod_index_(child_mobod_index),
        graph_constraint_index_(graph_constraint_index) {}

  /** (Internal use only) Copy/Move constructor & assignment. */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LoopConstraint)

 private:
  friend class SpanningForestModel;

  void FixupAfterReordering(const std::vector<MobodIndex>& old_to_new) {
    parent_mobod_index_ = old_to_new[parent_mobod_index_];
    child_mobod_index_ = old_to_new[child_mobod_index_];
  }

  MobodIndex parent_mobod_index_;
  MobodIndex child_mobod_index_;

  ConstraintIndex graph_constraint_index_;  // As added to the graph.
};

//================================== Tree ======================================
/** Everything you might want to know about an individual tree in the forest.
A Tree consists of consecutively numbered Mobod nodes in depth first order.
The first node is its base body (root node) and the last node is its
"rightmost" terminal body (assuming you draw the tree with the root at the
bottom, and children consistently left-to-right). Duplication is avoided by
keeping a back pointer to the owning Forest. */
class SpanningForestModel::Tree {
 public:
  Tree(const SpanningForestModel* forest, TreeIndex index,
       MobodIndex base_mobod)
      : index_(index), base_mobod_(base_mobod), height_(1), forest_(forest) {}

  TreeIndex index() const { return index_; }
  int height() const { return height_; }
  MobodIndex base_mobod() const { return base_mobod_; }
  MobodIndex last_mobod() const { return last_mobod_; }

  const SpanningForestModel::Mobod* begin() const {
    return &forest_->mobods()[base_mobod_];
  }
  const SpanningForestModel::Mobod* end() const {
    return &forest_->mobods()[last_mobod_] + 1;
  }
  const SpanningForestModel::Mobod& front() const {
    return forest_->mobods()[base_mobod_];
  }
  const SpanningForestModel::Mobod& back() const {
    return forest_->mobods()[last_mobod_];
  }

  int num_mobods() const { return last_mobod_ - base_mobod_ + 1; }

  int q_start() const { return front().q_start(); }
  int v_start() const { return front().v_start(); }
  int nq() const {
    // We depend on Weld Mobods to set their q_start to where it
    // would be if they had qs.
    const int last_q_plus_1 = back().q_start() + back().nq();
    return last_q_plus_1 - q_start();
  }
  int nv() const {
    // We depend on Weld Mobods to set their v_start to where it
    // would be if they had vs.
    const int last_v_plus_1 = back().v_start() + back().nv();
    return last_v_plus_1 - v_start();
  }

  /** (Internal use only) Copy/Move constructor & assignment. Back pointer
  requires fixup afterwards. */
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Tree)

 private:
  friend class SpanningForestModel;

  // Update all MobodIndex entries to use the new numbering.
  void FixupAfterReordering(const std::vector<MobodIndex>& old_to_new) {
    base_mobod_ = old_to_new[base_mobod_];
    // last_mobod_ is always set using the new ordering
  }

  TreeIndex index_;
  MobodIndex base_mobod_;  // The tree's root node that connects it to World.
  MobodIndex last_mobod_;  // The tree's highest numbered node.
  int height_{-1};         // 1 if just base body.
  const SpanningForestModel* forest_{nullptr};  // The containing forest.
};

//================================ inlines =====================================

inline LinkIndex SpanningForestModel::mobod_to_link(
    MobodIndex mobod_index) const {
  return mobods(mobod_index).link();
}

inline const std::vector<LinkIndex>& SpanningForestModel::mobod_to_links(
    MobodIndex mobod_index) const {
  return mobods(mobod_index).follower_links();
}

inline TreeIndex SpanningForestModel::q_to_tree(int q_index) const {
  return mobods(q_to_mobod(q_index)).tree();
}

inline TreeIndex SpanningForestModel::v_to_tree(int v_index) const {
  return mobods(v_to_mobod(v_index)).tree();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
