// NOLINTNEXTLINE(build/include): prevent complaint re spanning_forest.h
#include <algorithm>
#include <queue>
#include <stack>
#include <tuple>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/topology/forest.h"

namespace drake {
namespace multibody {
namespace internal {

SpanningForest::SpanningForest(const SpanningForest& source)
    : data_(source.data_) {
  FixInternalPointers();
}

SpanningForest::SpanningForest(SpanningForest&& source)
    : data_(std::move(source.data_)) {
  FixInternalPointers();
}

SpanningForest& SpanningForest::operator=(const SpanningForest& source) {
  if (&source != this) {
    data_ = source.data_;
    FixInternalPointers();
  }
  return *this;
}

SpanningForest& SpanningForest::operator=(SpanningForest&& source) {
  if (&source != this) {
    data_ = std::move(source.data_);
    FixInternalPointers();
  }
  return *this;
}

void SpanningForest::FixInternalPointers() {
  /* Only Tree currently has a back pointer. */
  for (auto& tree : data_.trees) tree.forest_ = this;
}

/* Clear() leaves this with nothing, not even a Mobod for World. */
void SpanningForest::Clear() {
  /* We want the graph back pointer to remain unchanged. */
  LinkJointGraph* const saved_graph = data_.graph;
  data_ = Data{};
  data_.graph = saved_graph;
}

/* This is the algorithm that takes an arbitrary link-joint graph and models
it as a spanning forest of mobilized bodies plus loop-closing constraints.

@note An assumption that affects some implementation choices: in a large system,
most links will be either (1) welded to World or to each other to create complex
objects, or (2) free bodies representing manipulands. The number of articulated
links representing a robot or robots will be modest in comparison. Hence we
want to handle welded and free links efficiently.

The algorithm has three phases:
  1 Produce the best tree structures we can in one pass. For example, when
    breaking loops we want to minimize the maximum branch length for better
    numerics, but we can't allow massless terminal bodies unless they are welded
    to a massful body. Welded-together Links form a WeldedLinksAssembly object
    which will be modeled by one or more Mobods which may be a mix of
    single-link and composite Mobods.
  2 Reorder the forest nodes (mobilized bodies) depth first for optimal
    computation. Each tree will consist only of consecutively-numbered nodes.
  3 Assign joint coordinates q and velocities v sequentially according
    to the depth-first ordering.

  During execution of this algorithm, we opportunistically collect information
  that can be used for fast queries at run time, both for the graph and the
  forest. For example: Which Links are welded together (WeldedLinksAssembly)?
  Which Mobods are welded together (WeldedMobods groups)? Which of
  those are anchored to World? Which Tree does a given Mobod belong to? How
  many coordinates are inboard or outboard of a Mobod?

ForestBuildingOptions (global or per-model instance) determine
  - whether we produce a Mobod for _each_ Link in a WeldedLinksAssembly or
    should produce a minimal number of Mobods by creating composite Mobods (as
    permitted by options on individual weld joints), and
  - what kind of Joint we use to mobilize unconnected root Links: 0 dof fixed,
    6 dof roll-pitch-yaw floating, or 6 dof quaternion floating.

Here is an expansion of the above three phases. The code refers back to the
numbering here for clarification.

1. Construct a forest with good topology
   1.1 Add the World mobilized body to the forest at level 0.
   1.2 Add missing Joints to World if needed for
     a) Links in a Static ModelInstance,
     b) Links explicitly flagged "Static", and
     c) Links marked "MustBeBaseBody".
     (Links can be static due to individual LinkFlags or membership in a static
     model instance.)
   1.3 ExtendTrees(all existing base bodies)   (see algorithm E below)
   1.4 While there are unprocessed _jointed_ (articulated) Links
     a) Choose an unprocessed Link L and model it with a new Mobod B that is
        to be the root node (base body) of a new tree. L is chosen by picking
        the "best" base Link according to some policy.
        (Heuristic: pick one that never appears as a child of a Joint.)
     b) ExtendTrees(B)  (just this one tree; see algorithm E below)
   1.5 While there are unprocessed _unjointed_ Links (single bodies)
     a) Each of these can be the base body of a new one-Mobod tree
     b) If a Link is in a "use fixed base" model instance, weld it to World.
        If we're optimizing welded links it just follows the World Mobod
        does not form a new Tree; otherwise it gets its own Mobod and Tree.
     c) Otherwise add a floating Joint to World (rpy or quaternion depending
        on options) and start a new Tree.
     d) Note that any unjointed _static_ Link, or a Link in a static model
        instance, had a weld joint added in step 1.2 above so was processed in
        the first call to ExtendTrees().

   E. Algorithm ExtendTrees(tree_base_bodies):
      E.1 Extend each tree one level at a time.
      E.2 Process all level 1 (base) Links directly jointed to World.
      E.3 Then process all level 2 Links jointed to level 1 Links, etc.
      E.4 WeldedLinksAssemblies are (optionally) modeled using composite Mobods
          where permitted (one level per Mobod).
      E.5 If we encounter a Link that has already been processed there is a
          loop. Split the Link into primary and shadow Links, and allocate
          two Mobods, one for each of those to follow. Add a loop constraint to
          weld them back together.
      E.6 Don't end a level with a massless Link; if we encounter one
          keep going until we can end on a massful Link.

2. Reorder depth first
   2.1 Determine the depth first re-ordering of the mobilized bodies.
   2.2 Use that reordering to update the SpanningForest in place.

3. Assign coordinates q and velocities v
   3.1 Each tree gets a consecutive set of q's and a consecutive set of v's,
       doled out following the depth-first ordering of the trees.
   3.2 Build maps from q and v to corresponding Mobod (includes fast q,v->Tree
       also).
   3.3 Precalculate for each Mobod the number of coordinates along its inboard
       path to World and in the outboard subtree for which it is the root.
*/
bool SpanningForest::BuildForest() {
  Clear();  // In case we're reusing this forest.

  SetBaseBodyChoicePolicy();

  /* Model the World (Link 0) with mobilized body 0. This also starts the 0th
  WeldedLinksAssembly in the graph and the 0th Welded Mobods group in the
  Forest. (1.1) */
  Mobod& world = data_.mobods.emplace_back(MobodIndex(0), LinkOrdinal(0));
  world.has_massful_follower_link_ = true;  // World is heavy!
  data_.welded_mobods.emplace_back(std::vector{MobodIndex(0)});
  world.welded_mobods_index_ = WeldedMobodsIndex(0);
  mutable_graph().set_primary_mobod_for_link(LinkOrdinal(0), MobodIndex(0),
                                             JointIndex{});
  mutable_graph().CreateWorldWeldedLinksAssembly();

  data_.forest_height = 1;  // Just World so far.

  /* Decide on forward/reverse mobilizers; optionally optimize
  WeldedLinksAssemblies so that all the Links in a WeldedLinksAssembly follow a
  single Mobod; choose links to serve as base (root) bodies and add 6dof
  mobilizers for them; split loops; add shadow bodies and weld constraints.
  (1.2-1.5) */
  ChooseForestTopology();

  /* Determine the desired depth-first ordering and apply it. (Phase 2) */
  const std::vector<MobodIndex> old_to_new = CreateDepthFirstReordering();
  FixupForestToUseNewNumbering(old_to_new);

  /* Dole out the q's and v's, depth-first. (Phase 3) */
  AssignCoordinates();

  DRAKE_ASSERT_VOID(SanityCheckForest());

  return dynamics_ok();
}

void SpanningForest::ChooseForestTopology() {
  /* When this goes to zero, we're done (World is already done). */
  int num_unprocessed_links = ssize(links()) - 1;

  /* Every Static Link should be welded to World. If there is not an explicit
  Weld Joint in the LinkJointGraph we'll add one now. Typically these Joints
  won't be modeled since they will be be interior to the World
  WeldedLinksAssembly, but they are necessary edges for graph analysis. */

  /* Look through the model instances to see if any of them are static and if so
  weld all their bodies to World. (1.2a) */
  for (const auto& [instance, link_indexes] :
       graph().data_.model_instance_to_link_indexes) {
    if (!model_instance_is_static(instance)) continue;
    ConnectLinksToWorld(link_indexes, true /* use weld joint */);
  }

  /* Add welds for any links that were explicitly marked "static". (1.2b) */
  ConnectLinksToWorld(graph().static_link_indexes(), true /* use weld joint */);

  /* Every MustBeBaseBody Link must have a Joint directly connecting it to
  World. If there isn't one already, we'll add a "floating" joint of the
  appropriate type to give it 6 dofs. It is important to add these Joints prior
  to building the Forest because they are necessary edges for graph analysis.
  For example, they can create cycles in the graph that need to be removed in
  the Forest. Note that Static Links were processed above and are not repeated
  on the must_be_base_body list even if they were so designated. (1.2c) */
  ConnectLinksToWorld(graph().non_static_must_be_base_body_link_indexes(),
                      false /* use floating joint */);

  /* Now grow all the trees whose base bodies have joints to World. This
  includes: (a) Links which were explicitly jointed to World by the user,
  (b) Static Links for which we just added weld joints, and (c) MustBeBaseBody
  Links for which we just added floating joints. (1.3) */
  ExtendTrees(graph().world_link().joints(), &num_unprocessed_links);

  /* What's left unprocessed now (if anything) is a collection of disjoint
  subgraphs that have no connection to World, including trivial subgraphs of
  lone free bodies. For each of those subgraphs we have to pick a root (base
  body) to connect to World by a suitable mobilizer (weld or floating). For each
  of the non-trivial (articulated) subgraphs, we need to select one of its Links
  as the "best" base according to some policy, and then extend from there to
  build a spanning tree for that subgraph. Finally, we make each of the lone
  free bodies its own "tree" (or add it to the World WeldedLinksAssembly if
  appropriate). (1.4-1.5) */
  ChooseBaseBodiesAndAddTrees(&num_unprocessed_links);

  DRAKE_DEMAND(num_unprocessed_links == 0);
}

/* Given the forest of mobilized bodies numbered in some arbitrary order,
reorder them depth-first. (The arbitrary numbering was produced by the
construction algorithm which may, for example, prefer to balance chain lengths
when breaking loops.) Considering the result as a forest of trees rooted at
their base bodies, the new numbering has the property that all mobods in tree i
have lower indexes than any mobod in tree i+1. And within a tree, all mobods in
the leftmost branch have lower numbers than any mobod in the next branch.
(2.1) */
std::vector<MobodIndex> SpanningForest::CreateDepthFirstReordering() const {
  const int num_mobods = ssize(mobods());

  std::vector<MobodIndex> old_to_new(num_mobods);

  std::stack<MobodIndex> to_process;
  to_process.push(MobodIndex(0));  // Start with World

  MobodIndex next(0);
  while (!to_process.empty()) {
    const MobodIndex top_index = to_process.top();
    old_to_new[top_index] = next++;
    to_process.pop();  // Done with this one.

    /* Push outboard bodies in reverse order so we'll process them in the
    order they were defined. */
    const std::vector<MobodIndex>& outboard_mobods =
        mobods(top_index).outboard_mobods_;
    for (auto outboard = outboard_mobods.rbegin();
         outboard != outboard_mobods.rend(); ++outboard) {
      to_process.push(*outboard);
    }
  }

  /* Every mobilized body should have been processed. */
  DRAKE_DEMAND(next == num_mobods);
  return old_to_new;
}

/* Applies the given mapping to renumber all Mobods and references to them
within the forest and its owning graph. (2.2) */
void SpanningForest::FixupForestToUseNewNumbering(
    const std::vector<MobodIndex>& old_to_new) {
  /* First update each Mobod to use only the new MobodIndex values, and
  record the highest-numbered Mobod in each tree. */
  for (auto& mobod : data_.mobods) {
    mobod.FixupAfterReordering(old_to_new);
    if (!mobod.is_world()) {
      Tree& tree = data_.trees[mobod.tree_index_];
      if (!tree.last_mobod_.is_valid() || mobod.index() > tree.last_mobod_)
        tree.last_mobod_ = mobod.index();  // Using new ordering.
    }
  }

  const int num_mobods = ssize(mobods());  // Reminder: includes World.

  /* Next, sort the Mobods into the proper order. Despite appearances, this
  is an O(n) algorithm, and usually num_swaps << n. (Hand-wavy proof: each
  iteration of the inner loop moves one Mobod to its final location from which
  it never leaves.) */
  for (MobodIndex new_index(1);  // World is always right already.
       new_index < num_mobods; ++new_index) {
    /* Work on the current Mobod until it holds the right one. Each iteration
    puts one Mobod in its right location so this must terminate. */
    Mobod& current_mobod = data_.mobods[new_index];
    while (current_mobod.index() != new_index) {
      /* Move current_mobod to its final location; replace with whatever was
      there before. */
      std::swap(current_mobod, data_.mobods[current_mobod.index()]);
    }
  }

  for (auto& constraint : data_.loop_constraints)
    constraint.FixupAfterReordering(old_to_new);
  for (auto& tree : data_.trees) tree.FixupAfterReordering(old_to_new);
  for (auto& welded_mobods : data_.welded_mobods)
    Mobod::RenumberMobodIndexVector(old_to_new, &welded_mobods);

  /* Fix up the Mobod references in the LinkJointGraph. */
  mutable_graph().RenumberMobodIndexes(old_to_new);
}

/* Dole out the q's and v's to each of the Mobods, following the depth-first
ordering. (Phase 3) */
void SpanningForest::AssignCoordinates() {
  int next_q = 0, next_v = 0;

  /* O(n) outward pass assigns coordinates, counts inboard ones, and builds map
  from coordinates to Mobods. (3.1, 3.2, and part of 3.3) */
  for (auto& mobod : data_.mobods) {
    mobod.q_start_ = next_q;
    mobod.v_start_ = next_v;
    if (!mobod.joint_ordinal().is_valid()) {
      mobod.nq_ = mobod.nv_ = 0;  // Treat World as though welded.
      mobod.nq_inboard_ = mobod.nv_inboard_ = 0;
      continue;
    }
    const JointTraitsIndex joint_traits_index =
        joints(mobod.joint_ordinal()).traits_index();
    const LinkJointGraph::JointTraits& joint_traits =
        graph().joint_traits()[joint_traits_index];

    mobod.nq_ = joint_traits.nq;
    mobod.nv_ = joint_traits.nv;

    if (joint_traits.has_quaternion) {
      mobod.has_quaternion_ = true;
      data_.quaternion_starts.push_back(mobod.q_start_);
    }

    /* Keep a running count of inboard coordinates. */
    DRAKE_DEMAND(mobod.inboard().is_valid());  // Non-World must have inboard.
    const Mobod& parent = mobods(mobod.inboard());
    /* Parent should have been processed before child. */
    DRAKE_DEMAND(parent.nq_inboard_ >= 0 && parent.nv_inboard_ >= 0);
    mobod.nq_inboard_ = parent.nq_inboard_ + mobod.nq_;
    mobod.nv_inboard_ = parent.nv_inboard_ + mobod.nv_;

    /* Map coordinates to their Mobod. */
    DRAKE_DEMAND(ssize(data_.q_to_mobod) == next_q &&
                 ssize(data_.v_to_mobod) == next_v);
    for (int i = 0; i < mobod.nq_; ++i)
      data_.q_to_mobod.push_back(mobod.index());
    for (int i = 0; i < mobod.nv_; ++i)
      data_.v_to_mobod.push_back(mobod.index());

    next_q += mobod.nq_;
    next_v += mobod.nv_;
  }

  /* O(n) inward pass counts outboard bodies and coordinates for each Mobod.
  (the rest of 3.3) */
  for (auto m = data_.mobods.rbegin(); m != data_.mobods.rend(); ++m) {
    m->nq_outboard_ = m->nv_outboard_ = 0;
    m->num_subtree_mobods_ = 1;  // Count this one.
    for (MobodIndex child_index : m->outboard_mobods_) {
      const Mobod& child = mobods(child_index);
      // Children should have been processed before parent.
      DRAKE_DEMAND(child.nq_outboard_ >= 0 && child.nv_outboard_ >= 0 &&
                   child.num_subtree_mobods_ >= 1);
      m->nq_outboard_ += child.nq_outboard_ + child.nq_;
      m->nv_outboard_ += child.nv_outboard_ + child.nv_;
      m->num_subtree_mobods_ += child.num_subtree_mobods_;
    }
  }
}

/* Phase 1 steps 1.4 and 1.5. */
void SpanningForest::ChooseBaseBodiesAndAddTrees(int* num_unprocessed_links) {
  // TODO(sherm1) Consider whether an optimized WeldedLinksAssembly (i.e.,
  //  follows just one Mobod) should be allowed to be a base body or
  //  should wait along with the other unjointed Links.

  /* Partition the unmodeled links into jointed and unjointed. O(n) */
  std::vector<LinkOrdinal> jointed_links, unjointed_links;
  for (const auto& link : links()) {
    const LinkOrdinal link_ordinal = link.ordinal();
    if (link_is_already_in_forest(link_ordinal)) continue;
    if (ssize(link.joints()) == 0) {
      unjointed_links.push_back(link_ordinal);
    } else {
      jointed_links.push_back(link_ordinal);
    }
  }

  /* Use the active base body choice policy to create a priority queue of
  links that could be base bodies. Complexity is O(j) to create a
  priority_queue of j jointed links. */
  std::priority_queue<LinkOrdinal, std::vector<LinkOrdinal>,
                      decltype(data_.base_body_policy)>
      eligible_bases(data_.base_body_policy, std::move(jointed_links));

  /* Process all the non-trivial (more than one link) subgraphs. Pop the top
  off the priority queue to get the best base link, then grow that tree.
  (1.4) Complexity is O(j log j). */
  while (*num_unprocessed_links > ssize(unjointed_links)) {
    DRAKE_DEMAND(!eligible_bases.empty());  // Must be something here!
    const LinkOrdinal next_base_link = eligible_bases.top();
    eligible_bases.pop();                                     // O(log j)
    if (link_is_already_in_forest(next_base_link)) continue;  // try another one
    const ModelInstanceIndex model_instance_index =
        links(next_base_link).model_instance();
    const JointIndex next_joint_index =
        mutable_graph().AddEphemeralJointToWorld(
            base_joint_traits_index(model_instance_index), next_base_link);
    ExtendTrees({next_joint_index}, &*num_unprocessed_links);
  }

  /* Should be nothing left now except unjointed (single) Links. We'll attach
  with either a floating joint or a weld joint depending on modeling options. If
  a weld and we're optimizing WeldedLinksAssemblies, the Link will just follow
  the World Mobod. Otherwise it gets a new Mobod that serves as the base body of
  a new Tree. Although static links and links belonging to a static model
  instance get welded to World at the start of forest building, it is still
  possible to need a weld here for a lone link that is in a "use fixed base"
  model instance (that's not technically a static link). (1.5) */
  DRAKE_DEMAND(*num_unprocessed_links == ssize(unjointed_links));

  for (LinkOrdinal unjointed_link : unjointed_links) {
    const ModelInstanceIndex model_instance_index =
        links(unjointed_link).model_instance();
    const JointTraitsIndex joint_type_to_use =
        base_joint_traits_index(model_instance_index);
    const JointIndex next_joint_index =
        mutable_graph().AddEphemeralJointToWorld(joint_type_to_use,
                                                 unjointed_link);
    const JointOrdinal next_joint_ordinal =
        graph().index_to_ordinal(next_joint_index);
    if (should_merge_parent_and_child(joints(next_joint_ordinal))) {
      JoinExistingMobod(&data_.mobods[0], unjointed_link, next_joint_ordinal);
    } else {
      AddNewMobod(unjointed_link, next_joint_ordinal, world_mobod().index(),
                  false);  // Not reversed; World is parent
    }
    /* No tree to extend here. */
  }
  *num_unprocessed_links -= ssize(unjointed_links);
}

/* This is algorithm E in the documentation above. */
void SpanningForest::ExtendTrees(const std::vector<JointIndex>& joints_to_model,
                                 int* num_unprocessed_links) {
  std::vector<JointIndex> to_model(joints_to_model), to_model_next;

  /* E.1 - E.3 */
  while (!to_model.empty()) {
    ExtendTreesOneLevel(to_model, &to_model_next, &*num_unprocessed_links);
    std::swap(to_model, to_model_next);
  }
}

/* Grows each Tree by one level with two exceptions:
   1 If we're optimizing WeldedLinksAssemblies (by merging welded-together
     Links onto a single Mobod), keep growing a WeldedLinkAssembly as far
     as possible to build the complete assembly and model it with a single
     Mobod.
   2 If we encounter a massless Link (or massless optimized WeldedLinkAssembly),
     we don't want its Mobod to be a terminal body of a branch. In that case we
     keep growing that branch until we can end with something massful. If we
     fail, we have to mark this forest as unsuited for dynamics since its mass
     matrix will be singular.

Exception 1 preserves the goal of minimizing maximum branch length since we
still only advance by one Mobod, regardless of how many merged Links that
entails.

Exception 2 sacrifices the even growth of branches in order to achieve the more
critical goal of ending every branch with a massful body to avoid making the
system mass matrix singular.

Definitions used below
----------------------
- Given a link L, we denote the _optimized_ WeldedLinksAssembly it belongs to
    as L+. If we're not optimizing WeldedLinksAssemblies, then L+ == L.
- A joint connects two links, parent and child. Every joint of interest here
    has at least one of its links already in the forest; that is its "inboard"
    link I. The other link is the "outboard" link O and is usually not yet in
    the forest.
- A "merged joint" is a weld joint that connects two links that should be part
    of the same optimized WeldedLinksAssembly. Merged joints do not have a
    corresponding mobilizer in the forest; they are unmodeled.
- Define Jₒₚₑₙ(L+) as the set of all not-yet-processed joints connected to any
    link in L+. These are "open" in the sense that only inboard link I of the
    joint is in L+; the other link O still needs to be dealt with. Open joints
    define the next level to be added to the forest outboard of L+.

Algorithm A(Jᵢₙ, &Jₒᵤₜ)
-----------------------
Inputs:  graph, partially built forest F, a set Jᵢₙ of unmodeled joints that
         will extend F by one level (i.e. one joint per branch).
Outputs: updated forest F, the set Jₒᵤₜ of as-yet-unmodeled joints to add at
         the next level.

Note: Every j ∈ Jᵢₙ has at least one of its two links (parent and child) already
  modeled in the current forest F. On return, every j ∈ Jₒᵤₜ has at least one of
  its links already modeled in the updated forest. In either case, if both
  links are already in the forest then j is a loop joint.

Jₒᵤₜ = {}
For each jᵢₙ ∈ Jᵢₙ:
  A.1 If jᵢₙ should be a merged joint (see above), then merge O+ with the
      WeldedLinksAssembly I+. Mark weld joint jᵢₙ as "unmodeled". We haven't yet
      extended this branch by a level though, so we need to keep going.
      Set Jₗₑᵥₑₗ = Jₒₚₑₙ(O+). Otherwise (jᵢₙ not a merged joint), set
      Jₗₑᵥₑₗ = {jᵢₙ}.

  For each jₗₑᵥₑₗ ∈ Jₗₑᵥₑₗ:
    A.2 If both parent and child links of jₗₑᵥₑₗ are already in the forest,
        jₗₑᵥₑₗ is a loop closing joint. Handle the loop. -> NEXT jₗₑᵥₑₗ
    A.3 Otherwise, add a new Mobod M to the forest whose body models link O and
        whose mobilizer models joint jₗₑᵥₑₗ.
    A.4 Determine O+ (O or its WeldedLinksAssembly) and make all links in O+
        follow M.
    A.5 If M is massful, set Jₒᵤₜ += Jₒₚₑₙ(O+). -> NEXT jₗₑᵥₑₗ
    A.6 (M is massless) Examine the joints in Jₒₚₑₙ(O+). If there aren't any
        open joints then we have to terminate this branch with a massless body
        and note that the forest can't be used for dynamics. Make a note and a
        suitable message. -> NEXT jₗₑᵥₑₗ
    A.7 (M is massless, but Jₒₚₑₙ(O+) is not empty) If any one of those open
        joints leads to an outboard link which is massful, we're fine.
        Set Jₒᵤₜ += Jₒₚₑₙ(O+). -> NEXT jₗₑᵥₑₗ
    A.8 (M is massless, and all outboard links in Jₒₚₑₙ(O+) are massless).
        Recursively run this algorithm A(J'ᵢₙ, &J'ₒᵤₜ) with J'ᵢₙ=Jₒₚₑₙ(O+) to
        extend this branch another level in the hope of encountering something
        massful. Set Jₒᵤₜ += J'ₒᵤₜ. -> NEXT jₗₑᵥₑₗ
*/
void SpanningForest::ExtendTreesOneLevel(const std::vector<JointIndex>& J_in,
                                         std::vector<JointIndex>* J_out,
                                         int* num_unprocessed_links) {
  DRAKE_DEMAND(!J_in.empty());
  DRAKE_DEMAND(num_unprocessed_links != nullptr);
  DRAKE_DEMAND(J_out != nullptr);
  J_out->clear();

  std::vector<JointIndex> J_level, J_open;  // Temps for use below.
  for (JointIndex j_in_index : J_in) {
    const JointOrdinal joint_in_ordinal = graph().index_to_ordinal(j_in_index);
    const Joint& j_in = joints(joint_in_ordinal);
    if (j_in.has_been_processed())
      continue;  // Already did this one from the other side.

    /* Find the inboard Mobod that we're extending via j_in, which link
    follows that Mobod (link I), which is the outboard link O, and are those
    reversed from parent P and child C. */
    const auto [inboard_mobod_index, inboard_link_ordinal,
                outboard_link_ordinal, is_joint_in_reversed] =
        FindInboardMobod(j_in);

    /* Step A.1 */
    FindNextLevelJoints(inboard_mobod_index, {j_in_index}, &J_level,
                        &*num_unprocessed_links);

    for (JointIndex j_level_index : J_level) {
      const JointOrdinal j_level_ordinal =
          graph().index_to_ordinal(j_level_index);
      const Joint& j_level = joints(j_level_ordinal);
      DRAKE_DEMAND(!should_merge_parent_and_child(j_level));

      /* The inboard link returned here is guaranteed to be in the forest
      already; in particular it follows the indicated inboard_mobod. The
      outboard link usually isn't in the forest yet but can be. */
      const auto [j_level_inboard_link_ordinal, j_level_outboard_link_ordinal,
                  is_reversed] =
          graph().FindInboardOutboardLinks(inboard_mobod_index,
                                           j_level_ordinal);

      /* If the outboard link is already modeled in the forest, this is a
      loop-closing joint. (A.2). */
      if (link_is_already_in_forest(j_level_outboard_link_ordinal)) {
        /* Invalidates references to Links, Joints, Mobods, LoopConstraints. */
        HandleLoopClosure(j_level_ordinal);
        continue;
      }

      /* There isn't a loop and the outboard link isn't part of the same
      optimized WeldedLinksAssembly as the inboard link. That means we are going
      to need a new Mobod. Note: a reference to this Mobod could be invalidated
      below; refer to it by its stable index instead. (A.3)*/
      const MobodIndex new_mobod_index =
          AddNewMobod(j_level_outboard_link_ordinal, j_level_ordinal,
                      inboard_mobod_index, is_reversed)
              .index();
      --(*num_unprocessed_links);

      /* We just put link O on the new Mobod. O might just be the active link of
      an optimized WeldedLinksAssembly O+. If so we need to build O+ now and
      find all its open joints. (A.4) */
      J_open.clear();
      const Link& j_level_outboard_link = links(j_level_outboard_link_ordinal);
      FindNextLevelJoints(new_mobod_index, j_level_outboard_link.joints(),
                          &J_open, &*num_unprocessed_links);

      /* Now determine if we can stop here or if we have to keep extending
      the branch we're on. If the Mobod we just added was a massless body
      on an articulated (non-weld) mobilizer, we may need to extend "one more
      level" from here. This can continue recursively if we run into
      more massless Links. */
      if (j_level.is_weld() ||
          mobods(new_mobod_index).has_massful_follower_link()) {
        /* We can stop here. Collect up the Joints for the next level and
        go on to the next Joint to model at this level. (A.5) */
        J_out->insert(J_out->end(), J_open.begin(), J_open.end());
        continue;  // -> NEXT jₗₑᵥₑₗ
      }

      /* The Link or WeldedLinksAssembly O+ we just added to the branch is
      massless and articulated; we don't want to terminate the branch with it.
      We have Jₒₚₑₙ(O+) in J_open. Three possibilities:
      1 If Jₒₚₑₙ is empty we're stuck and will just have to declare this as a
        "no dynamics" model.
      2 If there is a joint in Jₒₚₑₙ that leads to a massful link, we'll
        assume we're fine and can end here knowing that the massless Mobod will
        get covered at the next level. (There could still be problems that
        we can't detect here.)
      3 If all the outboard joints in Jₒₚₑₙ lead to massless links, we must not
        stop here but instead need to move on to the next level in the hope that
        we'll eventually run into something massful. */

      /* Case 1: we're not going to be able to do dynamics. (A.6)*/
      if (J_open.empty()) {
        /* If this is the first problem we've seen, record it. */
        if (data_.dynamics_ok) {
          const LinkJointGraph::JointTraits& j_level_traits =
              graph().joint_traits(j_level.traits_index());
          data_.dynamics_ok = false;
          data_.why_no_dynamics = fmt::format(
              "Link {} on {} joint {} is a terminal, articulated, massless "
              "link. The resulting multibody system will have a singular "
              "mass matrix so cannot be used for dynamics.",
              j_level_outboard_link.name(), j_level_traits.name,
              j_level.name());
        }
        continue;  // -> NEXT jₗₑᵥₑₗ
      }

      /* Case 2: all good. (A.7) */
      if (HasMassfulOutboardLink(new_mobod_index, J_open)) {
        /* The outboard joints we just collected are the ones we should
        process at the next level. */
        J_out->insert(J_out->end(), J_open.begin(), J_open.end());
        continue;  // -> NEXT jₗₑᵥₑₗ
      }

      /* Case 3: there is still hope. The massless link does have some
      not-yet-modeled joints, but they all lead to more massless links. Extend
      further and collect up the outboard joints at the next level. (A.8) */
      std::vector<JointIndex> next_level_J_open;
      ExtendTreesOneLevel(J_open, &next_level_J_open, &*num_unprocessed_links);
      J_out->insert(J_out->end(), next_level_J_open.begin(),
                    next_level_J_open.end());
      // -> NEXT jₗₑᵥₑₗ
    }
  }
}

std::tuple<MobodIndex, LinkOrdinal, LinkOrdinal, bool>
SpanningForest::FindInboardMobod(const Joint& open_joint) const {
  const LinkOrdinal parent_link_ordinal =
      graph().index_to_ordinal(open_joint.parent_link_index());
  const LinkOrdinal child_link_ordinal =
      graph().index_to_ordinal(open_joint.child_link_index());
  const Link& parent_link = links(parent_link_ordinal);
  if (parent_link.mobod_index().is_valid()) {
    return std::make_tuple(parent_link.mobod_index(), parent_link_ordinal,
                           child_link_ordinal, false);
  }
  const Link& child_link = links(child_link_ordinal);
  DRAKE_DEMAND(child_link.mobod_index().is_valid());
  return std::make_tuple(child_link.mobod_index(), child_link_ordinal,
                         parent_link_ordinal, true);
}

void SpanningForest::FindNextLevelJoints(MobodIndex inboard_mobod_index,
                                         const std::vector<JointIndex>& J_in,
                                         std::vector<JointIndex>* J_level,
                                         int* num_unprocessed_links) {
  DRAKE_DEMAND(J_level != nullptr && num_unprocessed_links != nullptr);
  J_level->clear();

  for (auto j_in_index : J_in) {
    const Joint& j_in = joint_by_index(j_in_index);
    if (j_in.has_been_processed()) continue;

    if (!should_merge_parent_and_child(j_in)) {
      J_level->push_back(j_in.index());
      continue;
    }

    /* This has to be a merged (unmodeled) joint with parent & child links part
    of the same optimized WeldedLinksAssembly. We must grow the assembly in the
    outboard direction of the joint, which is the end that is _not_ already
    following the inboard Mobod. */
    const LinkIndex outboard_link_index =
        FindOutboardLink(inboard_mobod_index, j_in);

    /* Adds the outboard link O to the inboard assembly and keeps collecting
    the welded links of O+ to the assembly recursively, while collecting up
    all the further-outboard non-weld joints Jₒₚₑₙ(O+) into J_level. */
    GrowAssemblyMobod(&data_.mobods[inboard_mobod_index], outboard_link_index,
                      j_in.ordinal(), &*J_level, &*num_unprocessed_links);
  }
}

LinkIndex SpanningForest::FindOutboardLink(MobodIndex inboard_mobod_index,
                                           const Joint& joint) const {
  const Link& parent_link = link_by_index(joint.parent_link_index());
  const Link& child_link = link_by_index(joint.child_link_index());
  if (parent_link.mobod_index().is_valid() &&
      parent_link.mobod_index() == inboard_mobod_index) {
    return child_link.index();
  }
  DRAKE_DEMAND(child_link.mobod_index().is_valid() &&
               child_link.mobod_index() == inboard_mobod_index);
  return parent_link.index();
}

bool SpanningForest::HasMassfulOutboardLink(
    MobodIndex inboard_mobod_index,
    const std::vector<JointIndex>& joints) const {
  for (JointIndex joint_index : joints) {
    const Joint& joint = joint_by_index(joint_index);
    const LinkIndex outboard_link_index =
        FindOutboardLink(inboard_mobod_index, joint);
    if (!link_by_index(outboard_link_index).is_massless()) return true;
  }
  return false;
}

/* See documentation in header before trying to decipher this. */
const SpanningForest::Mobod& SpanningForest::AddNewMobod(
    LinkOrdinal outboard_link_ordinal, JointOrdinal joint_ordinal,
    MobodIndex inboard_mobod_index, bool is_reversed) {
  const Joint& joint = joints(joint_ordinal);

  /* Careful -- don't hold Mobod references until _after_ we grow the vector. */

  /* Create the new Mobod one level higher than the inboard Mobod. */
  const int inboard_level = mobods(inboard_mobod_index).level();
  const MobodIndex new_mobod_index(ssize(mobods()));
  Mobod& new_mobod =
      data_.mobods.emplace_back(new_mobod_index, outboard_link_ordinal,
                                joint_ordinal, inboard_level + 1, is_reversed);
  const Link& outboard_link = links(outboard_link_ordinal);
  if (!outboard_link.is_massless()) new_mobod.has_massful_follower_link_ = true;

  /* If the inboard Mobod is World, start a new Tree. */
  Mobod& inboard_mobod = data_.mobods[inboard_mobod_index];
  TreeIndex tree_index = inboard_mobod.tree();
  if (!tree_index.is_valid()) {
    DRAKE_DEMAND(inboard_mobod.is_world());  // Otherwise should have a tree.
    tree_index = TreeIndex(ssize(trees()));
    data_.trees.emplace_back(this, tree_index, new_mobod.index());
  }

  /* At this point tree_index selects the right Tree. Put new_mobod in and
  update tree and forest stats. */
  Tree& tree = data_.trees[tree_index];
  new_mobod.tree_index_ = tree_index;
  tree.height_ = std::max(tree.height_, new_mobod.level_);
  data_.forest_height = std::max(data_.forest_height, tree.height_ + 1);

  /* Record connections in forest and graph. */
  new_mobod.inboard_mobod_ = inboard_mobod_index;
  inboard_mobod.outboard_mobods_.push_back(new_mobod_index);
  mutable_graph().set_primary_mobod_for_link(outboard_link_ordinal,
                                             new_mobod_index, joint.index());
  mutable_graph().set_mobod_for_joint(joint_ordinal, new_mobod_index);

  /* Build up both WeldedMobods group (in forest) and WeldedLinksAssembly (in
  graph) if we have a Weld joint, starting a new group or assembly as needed.
  Note that if we get here we are _not_ optimizing WeldedLinkAssemblies or we
  wouldn't have asked for a new Mobod! */
  if (joint.is_weld()) {
    if (!inboard_mobod.welded_mobods_index_.has_value()) {
      inboard_mobod.welded_mobods_index_ =
          WeldedMobodsIndex(ssize(welded_mobods()));
      data_.welded_mobods.emplace_back(std::vector{inboard_mobod_index});
    }
    new_mobod.welded_mobods_index_ = inboard_mobod.welded_mobods_index_;
    data_.welded_mobods[*inboard_mobod.welded_mobods_index_].push_back(
        new_mobod_index);

    mutable_graph().AddToWeldedLinksAssembly(
        inboard_mobod.link_ordinal(), outboard_link_ordinal, joint_ordinal);
  }

  return new_mobod;
}

void SpanningForest::ConnectLinksToWorld(
    const std::vector<LinkIndex>& links_to_connect, bool use_weld) {
  for (const auto& link_index : links_to_connect) {
    const LinkOrdinal link_ordinal = graph().index_to_ordinal(link_index);
    DRAKE_DEMAND(!link_is_already_in_forest(link_ordinal));
    const Link& link = links(link_ordinal);
    bool found_joint_to_world = false;
    for (const auto& joint_index : link.joints()) {
      const JointOrdinal joint_ordinal = graph().index_to_ordinal(joint_index);
      const Joint& joint = joints(joint_ordinal);
      if (joint.connects(graph().world_link().index())) {
        found_joint_to_world = true;
        break;
      }
    }
    if (!found_joint_to_world) {
      const JointTraitsIndex joint_traits_index =
          use_weld ? LinkJointGraph::weld_joint_traits_index()
                   : base_joint_traits_index(link.model_instance());
      mutable_graph().AddEphemeralJointToWorld(joint_traits_index,
                                               link_ordinal);
    }
  }
}

/* Defines the function that expresses our base body choice policy.
This could depend on modeling options but for now we only have a single
policy:
  - Choose the Link that appears as a child least often. Typically the user's
    intended base body will serve only as a parent. (Think of the "torso" of
    a humanoid.)
  - In case of a tie, we pick the one that also appears most often as a parent.
  - If that is still a tie, we just pick the one that came first. */
void SpanningForest::SetBaseBodyChoicePolicy() {
  /* std::priority_queue puts the "largest" element on top, and this is its
  "less than" test. The worse choice must test "less than" the better one. */
  data_.base_body_policy = [this](const LinkOrdinal& left,
                                  const LinkOrdinal& right) -> bool {
    const Link& left_link = links(left);
    const Link& right_link = links(right);
    const int lnc = ssize(left_link.joints_as_child());
    const int rnc = ssize(right_link.joints_as_child());
    if (lnc > rnc) return true;   // left is a child more often (bad)
    if (lnc < rnc) return false;  // right is a child more often (good)

    /* Both appear as child Links equally often. */
    const int lnp = ssize(left_link.joints_as_parent());
    const int rnp = ssize(right_link.joints_as_parent());
    if (lnp < rnp) return true;   // left is a parent less often (bad)
    if (lnp > rnp) return false;  // right is a parent less often (good)

    /* Both appear as child & parent Links equally often; take lowest index. */
    return left > right;
  };
}

void SpanningForest::HandleLoopClosure(JointOrdinal loop_joint_ordinal) {
  Joint& loop_joint = mutable_graph().mutable_joint(loop_joint_ordinal);
  const LinkOrdinal parent_ordinal =
      graph().index_to_ordinal(loop_joint.parent_link_index());
  const LinkOrdinal child_ordinal =
      graph().index_to_ordinal(loop_joint.child_link_index());

  DRAKE_DEMAND(link_is_already_in_forest(parent_ordinal) &&
               link_is_already_in_forest(child_ordinal));

  /* If one of the two bodies is massless, that's no problem - we just have
  to be sure to cut the massful one. The two branches will then each be
  terminated with massful bodies (at 1/2 the mass each). However, if both
  bodies are massless this forest can only be used for kinematics. */
  const bool parent_is_massless =
      graph().link_and_its_assembly_are_massless(parent_ordinal);
  const bool child_is_massless =
      graph().link_and_its_assembly_are_massless(child_ordinal);

  /* Save an explanation the first time we are forced to end a branch with
  a massless body. */
  if (parent_is_massless && child_is_massless && data_.dynamics_ok) {
    data_.dynamics_ok = false;
    const Link& parent_link = links(parent_ordinal);
    const Link& child_link = links(child_ordinal);
    data_.why_no_dynamics = fmt::format(
        "Loop breaks at joint {} between two massless links {} and {}. "
        "That means these links are terminal bodies in the tree which "
        "will produce a singular mass matrix. Hence this model cannot "
        "be used for dynamics.\n",
        loop_joint.name(), parent_link.name(), child_link.name());
  }

  /* If the branches leading to each link are of unequal length, we prefer to
  split the one on the longer branch to keep the branches more even. Otherwise
  we prefer to split the child since that will preserve the joint's
  parent->child order in the inboard->outboard order for the Mobod. */

  bool split_parent = false;  // Prefer child.
  if (!(child_is_massless || parent_is_massless)) {
    const int child_level =
        mobods(graph().link_to_mobod(loop_joint.child_link_index())).level();
    const int parent_level =
        mobods(graph().link_to_mobod(loop_joint.parent_link_index())).level();
    if (parent_level > child_level) split_parent = true;
  } else if (child_is_massless) {
    split_parent = true;
  }

  AddShadowMobod(split_parent ? parent_ordinal : child_ordinal,
                 loop_joint_ordinal);
}

/* We're going to add a shadow Link to the given primary Link and create a Mobod
for the shadow appropriate for the given Joint. Then we'll add a weld constraint
between the shadow and its primary. */
const SpanningForest::Mobod& SpanningForest::AddShadowMobod(
    LinkOrdinal primary_link_ordinal, JointOrdinal shadow_joint_ordinal) {
  const Link& primary_link = links(primary_link_ordinal);
  Joint& shadow_joint = mutable_graph().mutable_joint(shadow_joint_ordinal);
  DRAKE_DEMAND(shadow_joint.connects(primary_link.index()));
  const LinkIndex inboard_link_index =
      shadow_joint.other_link_index(primary_link.index());

  /* The Joint was written to connect inboard_link to primary_link but is
  actually going to connect inboard_link to shadow_link. */
  const bool is_reversed =
      shadow_joint.parent_link_index() != inboard_link_index;

  const LinkOrdinal shadow_link_ordinal = mutable_graph().AddShadowLink(
      primary_link_ordinal, shadow_joint_ordinal, is_reversed);

  const LoopConstraintIndex loop_constraint_index =
      mutable_graph().AddLoopClosingWeldConstraint(primary_link_ordinal,
                                                   shadow_link_ordinal);

  const LinkOrdinal inboard_link_ordinal =
      graph().index_to_ordinal(inboard_link_index);
  const MobodIndex inboard_mobod_index =
      links(inboard_link_ordinal).mobod_index();
  const Mobod& shadow_mobod =
      AddNewMobod(shadow_link_ordinal, shadow_joint_ordinal,
                  inboard_mobod_index, is_reversed);

  const MobodIndex primary_mobod_index =
      links(primary_link_ordinal).mobod_index();
  data_.loop_constraints.emplace_back(
      loop_constraint_index, primary_mobod_index, shadow_mobod.index());

  return shadow_mobod;
}

const SpanningForest::Mobod& SpanningForest::JoinExistingMobod(
    Mobod* inboard_mobod, LinkOrdinal follower_link_ordinal,
    JointOrdinal weld_joint_ordinal) {
  const Joint& weld_joint = joints(weld_joint_ordinal);
  DRAKE_DEMAND(weld_joint.traits_index() ==
               LinkJointGraph::weld_joint_traits_index());
  const WeldedLinksAssemblyIndex assembly_index =
      mutable_graph().AddToWeldedLinksAssembly(inboard_mobod->link_ordinal(),
                                               follower_link_ordinal,
                                               weld_joint_ordinal);
  mutable_graph().set_primary_mobod_for_link(
      follower_link_ordinal, inboard_mobod->index(), weld_joint.index());
  inboard_mobod->follower_link_ordinals_.push_back(follower_link_ordinal);
  const Link& follower_link = links(follower_link_ordinal);
  if (!follower_link.is_massless())
    inboard_mobod->has_massful_follower_link_ = true;

  /* We're not going to model this weld Joint since it is interior to an
  optimized WeldedLinksAssembly. We need to note the assembly it is part of. */
  mutable_graph().NoteUnmodeledJointInWeldedLinksAssembly(weld_joint_ordinal,
                                                          assembly_index);
  return *inboard_mobod;
}

void SpanningForest::GrowAssemblyMobod(
    Mobod* mobod, LinkIndex outboard_link_index,
    JointOrdinal weld_joint_ordinal,
    std::vector<JointIndex>* open_joint_indexes, int* num_unprocessed_links) {
  /* If the outboard_link has already been processed we're looking at a loop
  of welds within this WeldedLinksAssembly. That's harmless: we just add the
  Joint to the assembly and move on. */
  const LinkOrdinal outboard_link_ordinal =
      graph().index_to_ordinal(outboard_link_index);
  const Link& outboard_link = links(outboard_link_ordinal);
  if (link_is_already_in_forest(outboard_link_ordinal)) {
    const Link& inboard_link = links(mobod->link_ordinal());
    DRAKE_DEMAND(outboard_link.welded_links_assembly() ==
                 inboard_link.welded_links_assembly());
    mutable_graph().AddUnmodeledJointToWeldedLinksAssembly(
        weld_joint_ordinal, *outboard_link.welded_links_assembly());
    return;
  }

  /* At this point we know the outboard_link has not been processed yet. Add
  it to the assembly's Mobod. */

  JoinExistingMobod(&*mobod, outboard_link_ordinal, weld_joint_ordinal);
  --(*num_unprocessed_links);

  for (JointIndex joint_index : outboard_link.joints()) {
    const JointOrdinal joint_ordinal = graph().index_to_ordinal(joint_index);
    if (joint_ordinal == weld_joint_ordinal)
      continue;  // That's the one we just did.
    const Joint& joint = joints(joint_ordinal);

    /* If any of outboard_link's other joints had been processed, outboard_link
    would have already been in the forest. Hence all its non-merge joints
    are open (unprocessed). */
    DRAKE_DEMAND(!joint.has_been_processed());
    if (!should_merge_parent_and_child(joint)) {
      open_joint_indexes->push_back(joint_index);
      continue;  // On to the next Joint.
    }

    /* We've found another unprocessed joint that needs merging onto this
    assembly. One of its links is the outboard_link (already in the forest
    at this point). Find the other link. */
    const LinkIndex other_link_index =
        joint.other_link_index(outboard_link_index);

    /* Recursively extend the Assembly along the new merge-weld joint. */
    GrowAssemblyMobod(&*mobod, other_link_index, joint_ordinal,
                      &*open_joint_indexes, &*num_unprocessed_links);
  }
}

std::vector<MobodIndex> SpanningForest::FindPathFromWorld(
    MobodIndex index) const {
  const Mobod* mobod = &mobods(index);
  std::vector<MobodIndex> path(mobod->level() + 1);
  while (mobod->inboard().is_valid()) {
    path[mobod->level()] = mobod->index();
    mobod = &mobods(mobod->inboard());
  }
  DRAKE_DEMAND(mobod->is_world());
  path[0] = MobodIndex(0);
  return path;
}

MobodIndex SpanningForest::FindFirstCommonAncestor(
    MobodIndex mobod1_index, MobodIndex mobod2_index) const {
  // A body is its own first common ancestor.
  if (mobod1_index == mobod2_index) return mobod1_index;

  // If either body is World, that's the first common ancestor.
  if (mobod1_index == world_mobod_index() ||
      mobod2_index == world_mobod_index()) {
    return world_mobod_index();
  }

  const Mobod* branch1 = &mobods(mobod1_index);
  const Mobod* branch2 = &mobods(mobod2_index);

  // If they are in different trees, World is the ancestor.
  if (branch1->tree() != branch2->tree()) return world_mobod().index();

  // Get down to a common level, then go down both branches.
  while (branch1->level() > branch2->level())
    branch1 = &mobods(branch1->inboard());
  while (branch2->level() > branch1->level())
    branch2 = &mobods(branch2->inboard());

  // Both branches are at the same level now.
  while (branch1->index() != branch2->index()) {
    branch1 = &mobods(branch1->inboard());
    branch2 = &mobods(branch2->inboard());
  }

  return branch1->index();  // Same as branch2->index().
}

MobodIndex SpanningForest::FindPathsToFirstCommonAncestor(
    MobodIndex mobod1_index, MobodIndex mobod2_index,
    std::vector<MobodIndex>* path1, std::vector<MobodIndex>* path2) const {
  DRAKE_DEMAND(path1 != nullptr && path2 != nullptr);
  path1->clear();
  path2->clear();
  // A body is its own first common ancestor.
  if (mobod1_index == mobod2_index) return mobod1_index;

  const Mobod* branch1 = &mobods(mobod1_index);
  const Mobod* branch2 = &mobods(mobod2_index);

  // Get down to a common level, then go down both branches.
  while (branch1->level() > branch2->level()) {
    path1->push_back(branch1->index());
    branch1 = &mobods(branch1->inboard());
  }
  while (branch2->level() > branch1->level()) {
    path2->push_back(branch2->index());
    branch2 = &mobods(branch2->inboard());
  }

  // Both branches are at the same level now.
  while (branch1->index() != branch2->index()) {
    path1->push_back(branch1->index());
    path2->push_back(branch2->index());
    branch1 = &mobods(branch1->inboard());
    branch2 = &mobods(branch2->inboard());
  }

  return branch1->index();  // Same as branch2->index().
}

std::vector<LinkIndex> SpanningForest::FindSubtreeLinks(
    MobodIndex root_mobod_index) const {
  const int num_subtree_mobods = mobods(root_mobod_index).num_subtree_mobods();
  std::vector<LinkIndex> result;
  result.reserve(num_subtree_mobods);  // Will be at least this big.
  for (int i = 0; i < num_subtree_mobods; ++i) {
    const Mobod& subtree_mobod = mobods(MobodIndex(root_mobod_index + i));
    for (LinkOrdinal ordinal : subtree_mobod.follower_link_ordinals()) {
      const Link& link = links(ordinal);
      result.push_back(link.index());
    }
  }
  return result;
}

SpanningForest::Data::Data() = default;
SpanningForest::Data::Data(const Data&) = default;
SpanningForest::Data::Data(Data&&) = default;
SpanningForest::Data::~Data() = default;
auto SpanningForest::Data::operator=(const Data&) -> Data& = default;
auto SpanningForest::Data::operator=(Data&&) -> Data& = default;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
