// NOLINTNEXTLINE(build/include): prevent complaint re spanning_forest.h
#include <queue>
#include <stack>
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

// TODO(sherm1) For review purposes, the code here implements only a
//  subset of the algorithm described.
//  What's here: the ability to model tree-structured graphs, add joints to
//  world as needed, grow breadth-first and then renumber depth-first,
//  assign coordinates.
//  What's not here (see #20225): breaking loops, treat massless bodies
//  specially, use a single Mobod for composites. The
//  related options are allowed but ignored.

/* This is the algorithm that takes an arbitrary link-joint graph and models
it as a spanning forest of mobilized bodies plus loop-closing constraints.

@note An assumption that affects some implementation choices: in a large system,
most links will be either (1) welded to World or to each other to create complex
objects, or (2) free bodies representing manipulands. The number of articulated
links representing a robot or robots will be modest in comparison. Hence we
want to handle welded and free links efficiently.

The algorithm has three phases:
  1 Produce the best tree structures we can. For example, when breaking
    loops we want to minimize the maximum branch length for better numerics,
    but we can't allow massless terminal bodies unless they are welded to
    a massful body. Welded-together Links form a LinkComposite which can
    (optionally) be modeled using a single mobilized body.
  2 Reorder the forest nodes (mobilized bodies) depth first for optimal
    computation. Each tree will consist only of consecutively-numbered nodes.
  3 Assign joint coordinates q and velocities v sequentially according
    to the depth-first ordering.

  During execution of this algorithm, we opportunistically collect information
  that can be used for fast queries at run time, both for the graph and the
  forest. For example: Which Links are welded together (LinkComposites)?
  Which Mobods are welded together (WeldedMobod groups)? Which of
  those are anchored to World? Which Tree does a given Mobod belong to? How
  many coordinates are inboard or outboard of a Mobod?

ForestBuildingOptions (global or per-model instance) determine
  - whether we produce a mobilized body for _each_ Link or just for each
    _group_ of welded-together Links (a LinkComposite), and
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
        If we're combining welded links it just joins the World LinkComposite
        and does not form a new Tree.
     c) Otherwise add a floating Joint to World (rpy or quaternion depending
        on options) and start a new Tree.
     d) Note that any unjointed _static_ Link, or a Link in a static model
        instance, had a weld joint added in step 1.2 above so was processed in
        the first call to ExtendTrees().

   E. Algorithm ExtendTrees(tree_base_bodies):
      E.1 Extend each tree one level at a time.
      E.2 Process all level 1 (base) Links directly jointed to World.
      E.3 Then process all level 2 Links jointed to level 1 Links, etc.
      E.4 Composites are (optionally) modeled with a single Mobod (one level).
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
void SpanningForest::BuildForest() {
  Clear();  // In case we're reusing this forest.

  SetBaseBodyChoicePolicy();

  /* Model the World (Link 0) with mobilized body 0. This also starts the 0th
  LinkComposite in the graph and the 0th Welded Mobods group in the Forest.
  (1.1) */
  data_.mobods.emplace_back(MobodIndex(0), BodyIndex(0));
  data_.welded_mobods.emplace_back(std::vector{MobodIndex(0)});
  data_.mobods[MobodIndex(0)].welded_mobods_index_ = WeldedMobodsIndex(0);
  mutable_graph().set_primary_mobod_for_link(BodyIndex(0), MobodIndex(0),
                                             JointIndex{});
  mutable_graph().CreateWorldLinkComposite();

  data_.forest_height = 1;  // Just World so far.

  /* Decide on forward/reverse mobilizers; combine LinkComposites so that all
  the Links in a LinkComposite follow a single Mobod; choose links to serve
  as base (root) bodies and add 6dof mobilizers for them; split loops; add
  shadow bodies and weld constraints. (1.2-1.5) */
  ChooseForestTopology();

  /* Determine the desired depth-first ordering and apply it. (Phase 2) */
  const std::vector<MobodIndex> old_to_new = CreateDepthFirstReordering();
  FixupForestToUseNewNumbering(old_to_new);

  /* Dole out the q's and v's, depth-first. (Phase 3) */
  AssignCoordinates();
}

void SpanningForest::ChooseForestTopology() {
  /* When this goes to zero, we're done (World is already done). */
  int num_unprocessed_links = ssize(links()) - 1;

  /* Every Static Link should be welded to World. If there is not an explicit
  Weld Joint in the LinkJointGraph we'll add one now. Typically these Joints
  won't be modeled since they will be be interior to the World LinkComposite,
  but they are necessary edges for graph analysis. */

  /* Look through the model instances to see if any of them are static and if so
  weld all their bodies to World. (1.2a) */
  for (const auto& [instance, links] : graph().data_.model_instance_to_links) {
    if (!model_instance_is_static(instance)) continue;
    ConnectLinksToWorld(links, true /* use weld joint */);
  }

  /* Add welds for any links that were explicitly marked "static". (1.2b) */
  ConnectLinksToWorld(graph().static_links(), true /* use weld joint */);

  /* Every MustBeBaseBody Link must have a Joint directly connecting it to
  World. If there isn't one already, we'll add a "floating" joint of the
  appropriate type to give it 6 dofs. It is important to add these Joints prior
  to building the Forest because they are necessary edges for graph analysis.
  For example, they can create cycles in the graph that need to be removed in
  the Forest. Note that Static Links were processed above and are not repeated
  on the must_be_base_body list even if they were so designated. (1.2c) */
  ConnectLinksToWorld(graph().non_static_must_be_base_body_links(),
                      false /* use floating joint */);

  /* Now grow all the trees whose base bodies have joints to World (or to the
  World LinkComposite). This includes: (a) Links which were explicitly jointed
  to World by the user, (b) Static Links for which we just added weld joints,
  and (c) MustBeBaseBody Links for which we just added floating joints. (1.3) */
  ExtendTrees(graph().world_link().joints(), &num_unprocessed_links);

  /* What's left unprocessed now (if anything) is a collection of disjoint
  subgraphs that have no connection to World, including trivial subgraphs of
  lone free bodies. For each of those subgraphs we have to pick a root (base
  body) to connect to World by a suitable mobilizer (weld or floating). For each
  of the non-trivial (articulated) subgraphs, we need to select one of its Links
  as the "best" base according to some policy, and then extend from there to
  build a spanning tree for that subgraph. Finally, we make each of the lone
  free bodies its own "tree" (or add it to the World composite if appropriate).
  (1.4-1.5) */
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
      current_mobod.Swap(data_.mobods[current_mobod.index()]);
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
    if (!mobod.joint().is_valid()) {
      mobod.nq_ = mobod.nv_ = 0;  // Treat World as though welded.
      mobod.nq_inboard_ = mobod.nv_inboard_ = 0;
      continue;
    }
    const JointTypeIndex joint_type_index = joints(mobod.joint()).type_index();
    const LinkJointGraph::JointType& joint_type =
        graph().joint_types()[joint_type_index];

    mobod.nq_ = joint_type.nq;
    mobod.nv_ = joint_type.nv;

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
  /* Partition the unmodeled links into jointed and unjointed. O(n) */
  std::vector<BodyIndex> jointed_links, unjointed_links;
  for (const auto& link : links()) {
    const BodyIndex link_index = link.index();
    if (link_is_already_in_forest(link_index)) continue;
    if (ssize(link.joints()) == 0) {
      unjointed_links.push_back(link_index);
    } else {
      jointed_links.push_back(link_index);
    }
  }

  /* Use the active base body choice policy to create a priority queue of
  links that could be base bodies. Complexity is O(j) to create a
  priority_queue of j jointed links. */
  std::priority_queue<BodyIndex, std::vector<BodyIndex>,
                      decltype(data_.base_body_policy)>
      eligible_bases(data_.base_body_policy, std::move(jointed_links));

  /* Process all the non-trivial (more than one link) subgraphs. Pop the top
  off the priority queue to get the best base link, then grow that tree.
  (1.4) Complexity is O(j log j). */
  while (*num_unprocessed_links > ssize(unjointed_links)) {
    DRAKE_DEMAND(!eligible_bases.empty());  // Must be something here!
    const BodyIndex next_base_link = eligible_bases.top();
    eligible_bases.pop();                                     // O(log j)
    if (link_is_already_in_forest(next_base_link)) continue;  // try another one
    const ModelInstanceIndex model_instance_index =
        links(next_base_link).model_instance();
    const JointIndex next_joint_index =
        mutable_graph().AddEphemeralJointToWorld(
            base_joint_type_index(model_instance_index), next_base_link);
    ExtendTrees({next_joint_index}, &*num_unprocessed_links);
  }

  /* Should be nothing left now except unjointed (single) Links. We'll attach
  with either a floating joint or a weld joint depending on modeling options.
  If a weld and we're optimizing composites the Link will just join the World
  Mobod. Otherwise it gets a new Mobod that serves as the base body of a new
  Tree. Although static links and links belonging to a static model instance
  get welded to World at the start of forest building, it is still possible
  to need a weld here for a lone link that is in a "use fixed base" model
  instance (that's not technically a static link). (1.5) */
  DRAKE_DEMAND(*num_unprocessed_links == ssize(unjointed_links));

  for (BodyIndex unjointed_link : unjointed_links) {
    const ModelInstanceIndex model_instance_index =
        links(unjointed_link).model_instance();
    const JointTypeIndex joint_type_to_use =
        base_joint_type_index(model_instance_index);
    const JointIndex next_joint_index =
        mutable_graph().AddEphemeralJointToWorld(joint_type_to_use,
                                                 unjointed_link);
    AddNewMobod(unjointed_link, next_joint_index, world_mobod().index(),
                false);  // Not reversed; World is parent
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
    ExtendTreesOneLevel(to_model, &*num_unprocessed_links, &to_model_next);
    std::swap(to_model, to_model_next);
  }
}

/* Grows each Tree by one level. */
// TODO(sherm1) Exceptions stubbed out: (1) keep going for massless links so we
//  don't ever leave one as a leaf when breaking loops, and (2) for composites
//  implemented with a single Mobod keep going until all those links are done.
//  See PR #20225 for the whole story.
void SpanningForest::ExtendTreesOneLevel(
    const std::vector<JointIndex>& joints_to_model, int* num_unprocessed_links,
    std::vector<JointIndex>* joints_to_model_next) {
  DRAKE_DEMAND(!joints_to_model.empty());
  DRAKE_DEMAND(num_unprocessed_links != nullptr);
  DRAKE_DEMAND(joints_to_model_next != nullptr);
  joints_to_model_next->clear();

  for (JointIndex joint_index : joints_to_model) {
    const Joint& joint = joints(joint_index);
    if (joint.has_been_processed())
      continue;  // Already did this one from the other side.

    /* At least one of the Joint's Links must have already been modeled.
    (Could be both in the case of a loop.) */
    const bool parent_is_modeled =
        link_is_already_in_forest(joint.parent_link());
    const bool child_is_modeled = link_is_already_in_forest(joint.child_link());
    DRAKE_DEMAND(parent_is_modeled || child_is_modeled);

    const BodyIndex inboard_link_index =
        parent_is_modeled ? joint.parent_link() : joint.child_link();

    /* Don't keep references to Mobods since we're growing the vector below. */
    const MobodIndex inboard_mobod_index =
        graph().link_to_mobod(inboard_link_index);

    const JointIndex modeled_joint_index = joint_index;  // For easier stubbing.

    // TODO(sherm1) Combining composites stubbed out here (E.4)

    /* The modeled_joint might connect to any Link of an inboard LinkComposite
    that uses inboard_mobod. That's the Link we want as the inboard Link. */
    const auto [modeled_inboard_link_index, modeled_outboard_link_index,
                is_reversed] =
        graph().FindInboardOutboardLinks(inboard_mobod_index,
                                         modeled_joint_index);

    /* If the outboard link is already modeled, this is a loop joint. */
    // TODO(sherm1) Loops are stubbed out here. (E.5)
    DRAKE_THROW_UNLESS(!link_is_already_in_forest(modeled_outboard_link_index));

    /* Note: invalidates references to Mobods. */
    AddNewMobod(modeled_outboard_link_index, modeled_joint_index,
                inboard_mobod_index,
                is_reversed);  // Is mobilizer reversed from Joint?
    --(*num_unprocessed_links);

    const Link& modeled_outboard_link = links(modeled_outboard_link_index);

    // TODO(sherm1) Massless body check goes here. For now we assume we
    //  can always stop after doing one level. (E.6)

    /* We can stop here. Collect up the Joints for the next level and go on to
    the next Joint to model at this level. */
    for (JointIndex next_joint_index : modeled_outboard_link.joints()) {
      if (!joints(next_joint_index).has_been_processed())
        joints_to_model_next->push_back(next_joint_index);
    }
  }
}

/* See documentation in header before trying to decipher this. */
const SpanningForest::Mobod& SpanningForest::AddNewMobod(
    BodyIndex outboard_link_index, JointIndex joint_index,
    MobodIndex inboard_mobod_index, bool is_reversed) {
  const Joint& joint = joints(joint_index);

  /* Careful -- don't hold Mobod references until _after_ we grow the vector. */

  /* Create the new Mobod one level higher than the inboard Mobod. */
  const int inboard_level = mobods(inboard_mobod_index).level();
  const MobodIndex new_mobod_index(ssize(mobods()));
  Mobod& new_mobod =
      data_.mobods.emplace_back(new_mobod_index, outboard_link_index,
                                joint_index, inboard_level + 1, is_reversed);
  Mobod& inboard_mobod = data_.mobods[inboard_mobod_index];

  /* If the inboard Mobod is World, start a new Tree. */
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

  // Record connections in forest and graph.
  new_mobod.inboard_mobod_ = inboard_mobod_index;
  inboard_mobod.outboard_mobods_.push_back(new_mobod_index);
  mutable_graph().set_primary_mobod_for_link(outboard_link_index,
                                             new_mobod_index, joint_index);
  mutable_graph().set_mobod_for_joint(joint_index, new_mobod_index);

  /* Build up both WeldedMobods group (in forest) and LinkComposite (in graph)
  if we have a Weld joint, starting a new group or composite as needed. */
  if (joint.type_index() == LinkJointGraph::weld_joint_type_index()) {
    if (!inboard_mobod.welded_mobods_index_.is_valid()) {
      inboard_mobod.welded_mobods_index_ =
          WeldedMobodsIndex(ssize(welded_mobods()));
      data_.welded_mobods.emplace_back(std::vector{inboard_mobod_index});
    }
    new_mobod.welded_mobods_index_ = inboard_mobod.welded_mobods_index_;
    data_.welded_mobods[inboard_mobod.welded_mobods_index_].push_back(
        new_mobod_index);

    mutable_graph().AddToLinkComposite(inboard_mobod.link(),
                                       outboard_link_index);
  }

  return new_mobod;
}

void SpanningForest::ConnectLinksToWorld(
    const std::vector<BodyIndex>& links_to_connect, bool use_weld) {
  for (const auto& link_index : links_to_connect) {
    DRAKE_DEMAND(!link_is_already_in_forest(link_index));
    const Link& link = links(link_index);
    bool found_joint_to_world = false;
    for (const auto& joint_index : link.joints()) {
      const Joint& joint = joints(joint_index);
      if (joint.connects(graph().world_link().index())) {
        found_joint_to_world = true;
        break;
      }
    }
    if (!found_joint_to_world) {
      const JointTypeIndex joint_type_index =
          use_weld ? LinkJointGraph::weld_joint_type_index()
                   : base_joint_type_index(link.model_instance());
      mutable_graph().AddEphemeralJointToWorld(joint_type_index, link_index);
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
  data_.base_body_policy = [this](const BodyIndex& left,
                                  const BodyIndex& right) -> bool {
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

// TODO(sherm1) Remove this.
/* To permit testing the APIs of Tree and LoopConstraint before the implementing
code is merged, we'll stub a forest that looks like this:

           -> mobod1 => mobod2
    World                 ^
           -> mobod3 .....|  loop constraint

There are two trees and a loop constraint where mobod3 is primary and
mobod2 is the shadow. The two joints to World have 1 dof, the "=>" joint
is a weld with 0 dofs.

Note that there are no graph elements corresponding to any of this stuff
in the stub; we're just testing SpanningForest APIs here which don't care.
This will not be a well-formed forest for other purposes! */
void SpanningForest::AddStubTreeAndLoopConstraint() {
  /* Add three dummy Mobods. */
  data_.mobods.reserve(4);  // Prevent invalidation of the references.
  auto& mobod1 =
      data_.mobods.emplace_back(MobodIndex(1), BodyIndex(1), JointIndex(1),
                                1 /* level */, false /* is_reversed */);
  auto& mobod2 =
      data_.mobods.emplace_back(MobodIndex(2), BodyIndex(2), JointIndex(2),
                                2 /* level */, false /* is_reversed */);
  auto& mobod3 =
      data_.mobods.emplace_back(MobodIndex(3), BodyIndex(3), JointIndex(3),
                                1 /* level */, false /* is_reversed */);

  /* Assign depth-first coordinates. */
  mobod1.q_start_ = 0;
  mobod1.nq_ = 1;
  mobod1.v_start_ = 0;
  mobod1.nv_ = 1;
  mobod2.q_start_ = 1;
  mobod2.nq_ = 0;
  mobod2.v_start_ = 1;
  mobod2.nv_ = 0;
  mobod3.q_start_ = 1;
  mobod3.nq_ = 1;
  mobod3.v_start_ = 1;
  mobod3.nv_ = 1;

  // Make the trees.
  data_.trees.reserve(2);
  auto& tree0 = data_.trees.emplace_back(this, TreeIndex(0), MobodIndex(1));
  tree0.last_mobod_ = MobodIndex(2);
  tree0.height_ = 2;
  auto& tree1 = data_.trees.emplace_back(this, TreeIndex(1), MobodIndex(3));
  tree1.last_mobod_ = MobodIndex(3);
  tree1.height_ = 1;

  mobod1.tree_index_ = tree0.index();
  mobod2.tree_index_ = tree0.index();
  mobod3.tree_index_ = tree1.index();

  // Add the loop constraint.
  data_.loop_constraints.emplace_back(LoopConstraintIndex(0), MobodIndex(3),
                                      MobodIndex(2));
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
