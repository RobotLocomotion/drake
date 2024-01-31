// NOLINTNEXTLINE(build/include): prevent complaint re spanning_forest.h
#include <iostream>
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
  // Only Tree currently has a back pointer.
  for (auto& tree : data_.trees) tree.forest_ = this;
}

void SpanningForest::Clear() {
  // We want the graph back pointer to remain unchanged.
  LinkJointGraph* const saved_graph = data_.graph;
  data_ = Data{};
  data_.graph = saved_graph;
}

/* This is the algorithm that takes an arbitrary link-joint graph and models
it as a spanning forest of mobilized bodies plus loop-closing constraints.

The algorithm has three phases:
  1 Produce the best tree structures we can. For example, when breaking
    loops we want to minimize the maximum branch length for better numerics,
    but we can't allow massless terminal bodies unless they are welded to
    a massful body. Welded-together Links can be combined into single
    composite bodies.
  2 Reorder the forest nodes (mobilized bodies) depth first for optimal
    computation. Each tree will consist only of consecutively-numbered nodes.
  3 Assign joint coordinates q and velocities v sequentially according
    to the depth-first ordering.

ModelingOptions (global or per-model instance) determine
  - whether we produce a mobilized body for _each_ Link or just for each
    _group_ of Links that are welded together into a composite body, and
  - what kind of Joint we use to mobilize unconnected root Links: 0 dof fixed,
    6 dof roll-pitch-yaw, or 6 dof quaternion.

1. Construct a forest with good topology
   - Add the World mobilized body to the forest at level 0.
   - Add missing Joints to World if needed for (a) Static Links,
     and (b) Links marked "MustBeBaseBody".
   - Extend each tree by levels
       - Process all level 1 (base) Links directly jointed to World
       - Then process all level 2 Links jointed to level 1 Links, etc.
       - Composites are treated as single bodies (one level)
   - While there are unprocessed _jointed_ (articulated) Links
       - Choose the "best" base body B according to some policy.
         (Heuristic: pick one that never appears as a child.)
       - Process the tree rooted at B by levels as above.
   - While there are unprocessed _unjointed_ Links (single bodies)
       - Add a 6 dof or fixed Joint to World for each Link as appropriate
       - Each 6 dof (free) body is a new tree of one mobilized body
       - Each welded (fixed) body is anchored and part of the World
         Mobod (if we're combining composites)

2. Reorder depth first
   - Determine the depth first re-ordering of the mobilized bodies.
   - Use that reordering to update the SpanningForest in place.

3. Assign coordinates q and velocities v
   - Each tree gets a consecutive set of q's and a consecutive set of v's,
     doled out in tree order.
   - Build maps from q and v to corresponding Mobod (includes fast q,v->Tree
     also).
*/
void SpanningForest::BuildForest() {
  Clear();  // In case we're reusing this forest.

  SetBaseBodyChoicePolicy();

  // Model the World (Link 0) with mobilized body 0. This also starts the
  // 0th Link Composite in the graph and the 0th Composite Mobod in the Forest.
  data_.mobods.emplace_back(MobodIndex(0), BodyIndex(0));
  data_.welded_mobods.emplace_back(std::vector{MobodIndex(0)});
  data_.mobods[MobodIndex(0)].welded_mobods_index_ = WeldedMobodsIndex(0);
  mutable_graph().set_primary_mobod_for_link(BodyIndex(0), MobodIndex(0),
                                             JointIndex{});
  mutable_graph().CreateWorldLinkComposite();

  data_.forest_height = 1;  // Just World so far.

  /* Decides on forward/reverse mobilizers; combines composite bodies; chooses
  base bodies and adds 6dof mobilizers for them; splits loops; adds shadow
  bodies and weld constraints. */
  ChooseForestTopology();

  // Determine the desired depth-first ordering and apply it.
  const std::vector<MobodIndex> old_to_new = CreateDepthFirstReordering();
  FixupForestToUseNewNumbering(old_to_new);

  // Dole out the q's and v's, depth-first.
  AssignCoordinates();

  DRAKE_ASSERT_VOID(SanityCheckForest());
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

std::vector<BodyIndex> SpanningForest::FindSubtreeLinks(
    MobodIndex root_mobod_index) const {
  const int num_subtree_mobods = mobods(root_mobod_index).num_subtree_mobods();
  std::vector<BodyIndex> result;
  result.reserve(num_subtree_mobods);  // Will be at least this big.
  for (int i = 0; i < num_subtree_mobods; ++i) {
    const Mobod& subtree_mobod = mobods(MobodIndex(root_mobod_index + i));
    result.insert(result.end(), subtree_mobod.follower_links().cbegin(),
                  subtree_mobod.follower_links().cend());
  }
  return result;
}

void SpanningForest::ChooseForestTopology() {
  // When this goes to zero, we're done (World is already done).
  int num_unprocessed_links = ssize(links()) - 1;

  /* Every Static Link should be welded to World. If there is not an explicit
  Weld Joint in the LinkJointGraph we'll add one now. Typically these Joints
  won't be modeled since they will be be interior to the World Link Composite,
  but they are necessary edges for graph analysis. */

  /* First, look through the model instances to see if any of them are static
  and if so weld all their bodies to World. */
  for (const auto& [instance, links] : graph().data_.model_instance_to_links) {
    if (!model_instance_is_static(instance)) continue;
    ConnectLinksToWorld(links, true /* use weld joint */);
  }

  // Second, add welds for any links that were explicitly marked "static".
  ConnectLinksToWorld(graph().static_links(), true /* use weld joint */);

  /* Every MustBeBaseBody Link must have a Joint directly connecting it to
  World. If there isn't one already, we'll add a "floating" joint of the
  appropriate type to give it 6 dofs. It is important to add these Joints prior
  to building the Forest because they are necessary edges for graph analysis.
  For example, they can create cycles in the graph that need to be removed in
  the Forest. Note that Static Links were processed above and are not repeated
  on the must_be_base_body list even if they were so designated. */
  ConnectLinksToWorld(graph().non_static_must_be_base_body_links(),
                      false /* use floating joint */);

  /* Now grow all the trees whose base bodies have joints to World (or to the
  World Link Composite). This includes: (a) Links which were explicitly jointed
  to World by the user, (b) Static Links for which we just added weld joints,
  and (c) MustBeBaseBody Links for which we just added floating joints. */
  ExtendTrees(graph().world_link().joints(), &num_unprocessed_links);

  /* What's left unprocessed now (if anything) is a collection of trees that
  have no connection to World, including trivial trees of lone free bodies.
  For each of those trees we have to pick a root (base body) to connect to
  World by a 6dof (floating) mobilizer. For each of the non-trivial
  (articulated) trees, we need to select one of its Links as the "best" base
  according to some policy, and then extend from there to build the whole tree.
  Then we add each of the lone free body "trees" last. */
  ChooseBaseBodiesAndAddTrees(&num_unprocessed_links);

  DRAKE_DEMAND(num_unprocessed_links == 0);
}

void SpanningForest::ChooseBaseBodiesAndAddTrees(int* num_unprocessed_links) {
  // TODO(sherm1) Consider whether a LinkComposite that is treated as a single
  //  body (i.e., follows just one Mobod) should be allowed to be a base body or
  //  should wait along with the other unjointed Links.

  /* Use the active base body choice policy to create a priority queue of
  Links that could be base bodies. */
  std::priority_queue<BodyIndex, std::vector<BodyIndex>,
                      decltype(data_.base_body_policy)>
      eligible_bases(data_.base_body_policy);

  // This just collects up all the lone free bodies.
  std::vector<BodyIndex> unjointed_links;

  for (const auto& link : links()) {
    const BodyIndex link_index = link.index();
    if (link_is_already_in_forest(link_index)) continue;
    if (ssize(link.joints()) == 0) {
      unjointed_links.push_back(link_index);  // O(1)
    } else {
      eligible_bases.push(link_index);  // O(log n)
    }
  }

  /* Process all the non-trivial (more than one body) trees. Pop the top
  off the priority queue to get the best base body, then grow that tree. */
  while (*num_unprocessed_links > ssize(unjointed_links)) {
    DRAKE_DEMAND(!eligible_bases.empty());  // Must be something here!
    const BodyIndex next_base_link = eligible_bases.top();
    eligible_bases.pop();
    if (link_is_already_in_forest(next_base_link)) continue;  // try another one
    const ModelInstanceIndex model_instance_index =
        links(next_base_link).model_instance();
    const JointIndex next_joint_index = mutable_graph().AddModelingJointToWorld(
        base_joint_type_index(model_instance_index), next_base_link);
    ExtendTrees({next_joint_index}, &*num_unprocessed_links);
  }

  /* Should be nothing left now except unjointed (free) Links. We'll attach with
  either a floating joint or a weld joint depending on modeling options. If a
  weld and we're optimizing composites the Link will just join the World Mobod.
  Otherwise it gets a new Mobod that serves as the base body of a new Tree. */
  DRAKE_DEMAND(*num_unprocessed_links == ssize(unjointed_links));

  for (BodyIndex unjointed_link : unjointed_links) {
    const ModelInstanceIndex model_instance_index =
        links(unjointed_link).model_instance();
    const JointIndex next_joint_index = mutable_graph().AddModelingJointToWorld(
        base_joint_type_index(model_instance_index), unjointed_link);
    if (should_be_unmodeled_weld(joints(next_joint_index))) {
      JoinExistingMobod(&data_.mobods[0], unjointed_link, next_joint_index);
    } else {
      AddNewMobod(unjointed_link, next_joint_index, world_mobod().index(),
                  false);  // Not reversed; World is parent
    }
    // No tree to extend here.
  }
  *num_unprocessed_links -= ssize(unjointed_links);
}

void SpanningForest::ExtendTrees(const std::vector<JointIndex>& joints_to_model,
                                 int* num_unprocessed_links) {
  std::vector<JointIndex> to_model(joints_to_model), to_model_next;

  while (!to_model.empty()) {
    ExtendTreesOneLevel(to_model, &*num_unprocessed_links, &to_model_next);
    std::swap(to_model, to_model_next);
  }
}

/* Grows each Tree by one level with two exceptions:
   - if we're optimizing composites, keep growing LinkComposites as far as
     possible since they use only a single Mobod, and
   - if we encounter a "treat as massless" Link it can't be a terminal body
     of a branch. In that case we keep growing that branch until we can end
     with something massful. Note that a LinkComposite can be terminal if _any_
     of its Links are massful. */
// TODO(sherm1) Handle massless composites.
void SpanningForest::ExtendTreesOneLevel(
    const std::vector<JointIndex>& joints_to_model, int* num_unprocessed_links,
    std::vector<JointIndex>* joints_to_model_next) {
  DRAKE_DEMAND(!joints_to_model.empty());
  DRAKE_DEMAND(num_unprocessed_links != nullptr);
  joints_to_model_next->clear();

  /* If we're optimizing composites and a joint_to_model turns out to be a weld,
  that doesn't actually extend the Forest. Instead we'll grow that composite
  until we find moving (non-weld) joints connected to it and model those. */
  std::vector<JointIndex> extended_joints_to_model;  // Temp for use below.
  for (JointIndex joint_index : joints_to_model) {
    const Joint& joint = joints(joint_index);
    if (joint.has_been_processed())
      continue;  // Already did this one from the other side.

    const bool parent_is_modeled =
        link_is_already_in_forest(joint.parent_link());
    const BodyIndex inboard_link_index =
        parent_is_modeled ? joint.parent_link() : joint.child_link();

    // Don't keep references to Mobods since we're growing the vector below.
    const MobodIndex inboard_mobod_index =
        graph().link_to_mobod(inboard_link_index);

    extended_joints_to_model.clear();
    if (should_be_unmodeled_weld(joint)) {
      GrowCompositeMobod(&data_.mobods[inboard_mobod_index],
                         joint.other_link_index(inboard_link_index),
                         joint_index, &extended_joints_to_model,
                         &*num_unprocessed_links);
    } else {
      extended_joints_to_model.push_back(joint_index);
    }

    for (JointIndex modeled_joint_index : extended_joints_to_model) {
      const Joint& modeled_joint = joints(modeled_joint_index);
      DRAKE_DEMAND(!should_be_unmodeled_weld(modeled_joint));

      /* The modeled_joint might connect to any Link of an inboard LinkComposite
      that uses inboard_mobod. That's the Link we want as the inboard Link. */
      const auto [modeled_inboard_link_index, modeled_outboard_link_index,
                  is_reversed] =
          graph().FindInboardOutboardLinks(inboard_mobod_index,
                                           modeled_joint_index);

      // If the outboard link is already modeled, this is a loop joint.
      if (link_is_already_in_forest(modeled_outboard_link_index)) {
        // Invalidates references to Links, Joints, Mobods, and LoopConstraints.
        HandleLoopClosure(modeled_joint_index);
        continue;
      }

      // Invalidates references to Mobods.
      AddNewMobod(modeled_outboard_link_index, modeled_joint_index,
                  inboard_mobod_index,
                  is_reversed);  // Is mobilizer reversed from Joint?
      --(*num_unprocessed_links);

      /* Now determine if we can stop here or if we have to keep extending
      the tree we're on. If the Mobod we just added was a massless body
      on an articulated (non-weld) mobilizer, we need to extend "one more
      level" from here. This can continue recursively if we run into
      more massless Links. */
      const Link& modeled_outboard_link = links(modeled_outboard_link_index);
      if (modeled_joint.is_weld() ||
          !modeled_outboard_link.treat_as_massless()) {
        /* We can stop here. Collect up the Joints for the next level and
        go on to the next Joint to model at this level. */
        for (JointIndex next_joint_index : modeled_outboard_link.joints()) {
          if (!joints(next_joint_index).has_been_processed())
            joints_to_model_next->push_back(next_joint_index);
        }
        continue;
      }

      /* The Link we just added to the tree is massless; can't stop here.
      If there aren't any more Joints attached to this massless link we
      have nowhere to go. */
      std::vector<JointIndex> massless_link_unmodeled_joints;
      for (JointIndex massless_link_joint_index :
           modeled_outboard_link.joints()) {
        if (!joints(massless_link_joint_index).has_been_processed()) {
          massless_link_unmodeled_joints.push_back(massless_link_joint_index);
          break;
        }
      }
      if (massless_link_unmodeled_joints.empty()) {
        const std::string message = fmt::format(
            "Warning: Articulated massless Link {} was modeled as a "
            "terminal body in the SpanningForest, meaning the model "
            "is suited only for kinematics. Most likely the input "
            "Link-Joint graph is actually singular; however, it is "
            "possible that this is a failure of the modeling heuristics "
            "for choosing a cut point when a loop in the graph involves "
            "massless Links. In that case you can fix the problem by "
            "specifying that a particular Joint is to be the cut point.",
            modeled_outboard_link.name());
        // TODO(sherm1) Log or return the message instead.
        std::cerr << message << "\n";
        continue;
      }

      /* The massless Link does have some not-yet-modeled Joints connected to
      it so there is still hope. Extend further and collect up the outboard
      Joints at the next level. */
      std::vector<JointIndex> next_level_outboard_joints;
      ExtendTreesOneLevel(massless_link_unmodeled_joints,
                          &*num_unprocessed_links, &next_level_outboard_joints);
      joints_to_model_next->insert(joints_to_model_next->end(),
                                   next_level_outboard_joints.begin(),
                                   next_level_outboard_joints.end());
    }
  }
}

/* Adds a new mobilized body outboard of the given inboard body:
   - set the level to one higher than the inboard level
   - if inboard is World, start a new tree otherwise new Mobod is in
     the same tree as inboard
   - add the new Mobod to the list of outboard Mobods in the inboard body
   - update maps of link-to-mobod and joint-to-mobod
   - if joint type is Weld, we are creating or joining a WeldedMobods group;
     if that is the World group then the Mobod is "anchored". */
const SpanningForest::Mobod& SpanningForest::AddNewMobod(
    BodyIndex outboard_link_index, JointIndex joint_index,
    MobodIndex inboard_mobod_index, bool is_reversed) {
  const Joint& joint = joints(joint_index);
  // Careful -- don't hold Mobod references until _after_ we grow the vector.
  const int inboard_level = mobods(inboard_mobod_index).level();
  const MobodIndex new_mobod_index(ssize(mobods()));
  Mobod& new_mobod =
      data_.mobods.emplace_back(new_mobod_index, outboard_link_index,
                                joint_index, inboard_level + 1, is_reversed);
  Mobod& inboard_mobod = data_.mobods[inboard_mobod_index];
  // If the inboard body is World, start a new Tree.
  TreeIndex tree_index = inboard_mobod.tree();
  if (!tree_index.is_valid()) {
    DRAKE_DEMAND(inboard_mobod.is_world());  // Otherwise should have a tree.
    tree_index = TreeIndex(ssize(trees()));
    data_.trees.emplace_back(this, tree_index, new_mobod.index());
    // A new tree has level 1, so forest has at least a height of 2 now.
    if (data_.forest_height < 2) data_.forest_height = 2;
  }
  Tree& tree = data_.trees[tree_index];
  new_mobod.tree_index_ = tree_index;
  if (new_mobod.level_ > tree.height_) {
    tree.height_ = new_mobod.level_;
    if (tree.height_ + 1 > data_.forest_height)
      data_.forest_height = tree.height_ + 1;
  }
  new_mobod.inboard_mobod_ = inboard_mobod_index;
  inboard_mobod.outboard_mobods_.push_back(new_mobod_index);
  mutable_graph().set_primary_mobod_for_link(outboard_link_index,
                                             new_mobod_index, joint_index);
  mutable_graph().set_mobod_for_joint(joint_index, new_mobod_index);

  // Build up both WeldedMobods group and LinkComposite if we have a Weld joint.
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

/* We're going to add a shadow Link to the given primary Link and create a Mobod
for the shadow appropriate for the given Joint. Then we'll add a weld constraint
between the shadow and its primary. */
const SpanningForest::Mobod& SpanningForest::AddShadowMobod(
    BodyIndex primary_link_index, JointIndex shadow_joint_index) {
  Joint& shadow_joint = mutable_graph().mutable_joint(shadow_joint_index);
  DRAKE_DEMAND(shadow_joint.connects(primary_link_index));
  const BodyIndex inboard_link_index =
      shadow_joint.other_link_index(primary_link_index);

  /* The Joint was written to connect inboard_link to primary_link but is
  actually going to connect inboard_link to shadow_link. */
  const bool is_reversed = shadow_joint.parent_link() != inboard_link_index;
  const Link& inboard_link = links(inboard_link_index);
  const MobodIndex inboard_mobod_index = inboard_link.mobod_index();

  const BodyIndex shadow_link_index =
      mutable_graph().AddShadowLink(primary_link_index, shadow_joint_index);

  const LoopConstraintIndex loop_constraint_index =
      mutable_graph().AddLoopClosingWeldConstraint(primary_link_index,
                                                   shadow_link_index);

  const Mobod& shadow_mobod = AddNewMobod(shadow_link_index, shadow_joint_index,
                                          inboard_mobod_index, is_reversed);

  const MobodIndex primary_mobod_index =
      links(primary_link_index).mobod_index();
  data_.loop_constraints.emplace_back(
      loop_constraint_index, primary_mobod_index, shadow_mobod.index());

  return shadow_mobod;
}

const SpanningForest::Mobod& SpanningForest::JoinExistingMobod(
    Mobod* inboard_mobod, BodyIndex follower_link_index,
    JointIndex weld_joint_index) {
  DRAKE_DEMAND(joints(weld_joint_index).type_index() ==
               LinkJointGraph::weld_joint_type_index());
  const LinkCompositeIndex link_composite_index =
      mutable_graph().AddToLinkComposite(inboard_mobod->link(),
                                         follower_link_index);
  mutable_graph().set_primary_mobod_for_link(
      follower_link_index, inboard_mobod->index(), weld_joint_index);
  inboard_mobod->follower_links_.push_back(follower_link_index);

  /* We're not going to model this weld Joint since it is interior to
  a LinkComposite. We need to note the composite it is part of. */
  mutable_graph().AddUnmodeledJointToComposite(weld_joint_index,
                                               link_composite_index);
  return *inboard_mobod;
}

void SpanningForest::GrowCompositeMobod(
    Mobod* mobod, BodyIndex follower_link_index, JointIndex weld_joint_index,
    std::vector<JointIndex>* outboard_joints, int* num_unprocessed_links) {
  /* If the follower_link has already been processed we're looking at a loop
  of welds within this LinkComposite. That's harmless: we just add the Joint
  to the composite and move on. */
  if (link_is_already_in_forest(follower_link_index)) {
    const Link& inboard_link = links(mobod->link());
    const Link& follower_link = links(follower_link_index);
    DRAKE_DEMAND(follower_link.composite() == inboard_link.composite());
    mutable_graph().AddUnmodeledJointToComposite(weld_joint_index,
                                                 follower_link.composite());
    return;
  }

  JoinExistingMobod(&*mobod, follower_link_index, weld_joint_index);
  --(*num_unprocessed_links);

  const Link& follower_link = links(follower_link_index);
  for (JointIndex joint_index : follower_link.joints()) {
    const Joint& joint = joints(joint_index);
    if (!should_be_unmodeled_weld(joint)) {
      outboard_joints->push_back(joint_index);
      continue;
    }
    const BodyIndex other_link_index =
        joint.other_link_index(follower_link_index);

    // If the other link has already been processed this is a loop as above.
    if (link_is_already_in_forest(other_link_index)) {
      const Link& other_link = links(other_link_index);
      DRAKE_DEMAND(other_link.composite() == follower_link.composite());
      mutable_graph().AddUnmodeledJointToComposite(joint_index,
                                                   follower_link.composite());
      continue;  // On to the next Joint.
    }

    // Recursively extend the Composite along the new weld.
    GrowCompositeMobod(&*mobod, other_link_index, joint_index,
                       &*outboard_joints, &*num_unprocessed_links);
  }
}

void SpanningForest::HandleLoopClosure(JointIndex loop_joint_index) {
  Joint& joint = mutable_graph().mutable_joint(loop_joint_index);
  DRAKE_DEMAND(link_is_already_in_forest(joint.parent_link()) &&
               link_is_already_in_forest(joint.child_link()));

  const bool parent_is_massless =
      graph().must_treat_as_massless(joint.parent_link());
  const bool child_is_massless =
      graph().must_treat_as_massless(joint.child_link());

  if (parent_is_massless && child_is_massless) {
    const Link& parent_link = links(joint.parent_link());
    const Link& child_link = links(joint.child_link());
    fmt::print(
        "Loop breaks at Joint {} between two massless Links {} and {}. "
        "That means these Links are terminal bodies in the tree which "
        "will produce a singular mass matrix. Hence this model may be "
        "used only for kinematics.\n",
        joint.name(), parent_link.name(), child_link.name());
    ModelLoopJointAsConstraint(loop_joint_index);
    return;
  }

  // We're going to split a Link, preferably the child.
  AddShadowMobod(child_is_massless ? joint.parent_link() : joint.child_link(),
                 joint.index());
  // Doesn't change the remaining number of links to process.
}

/* Given the forest of mobilized bodies numbered in some arbitrary
order, reorder them depth-first. (The arbitrary numbering was produced by the
construction algorithm which may, for example, prefer to balance chain
lengths when breaking loops.) Considering the result as a forest of trees
rooted at their base bodies, the new numbering has the property that all mobods
in tree i have lower indexes than any mobod in tree i+1. And within a tree,
all mobods in the leftmost branch have lower numbers than any mobod in the
next branch. */
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

  // Every mobilized body should have been processed.
  DRAKE_DEMAND(next == num_mobods);
  return old_to_new;
}

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

  const int num_mobods = ssize(mobods());

  /* Next, sort the Mobods into the proper order. Despite appearances, this
  is an O(n) algorithm, and usually num_swaps << n. */
  int num_swaps = 0;
  for (MobodIndex new_index(1);  // World is always right already.
       new_index < num_mobods && num_swaps < num_mobods; ++new_index) {
    while (mobods(new_index).index_ != new_index) {
      data_.mobods[new_index].Swap(data_.mobods[mobods(new_index).index_]);
      ++num_swaps;
    }
  }

  for (auto& constraint : data_.loop_constraints)
    constraint.FixupAfterReordering(old_to_new);
  for (auto& tree : data_.trees) tree.FixupAfterReordering(old_to_new);
  for (auto& welded_mobods : data_.welded_mobods)
    Mobod::RenumberMobodIndexVector(old_to_new, &welded_mobods);

  // Fix up the Mobod references in the LinkJointGraph.
  mutable_graph().RenumberMobodIndexes(old_to_new);
}

// N.B. this code depends on the fact that a Mobod's inboard and ancestor
// Mobods always come earlier in data_.mobods than it does. Depth-first
// ordering meets that condition.
void SpanningForest::AssignCoordinates() {
  int next_q = 0, next_v = 0;

  // O(n) outward pass assigns coordinates, counts inboard ones, and builds map
  // from coordinates to Mobods.
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

    // Keep a running count of inboard coordinates.
    DRAKE_DEMAND(mobod.inboard().is_valid());  // Non-World must have inboard.
    const Mobod& parent = mobods(mobod.inboard());
    // Parent should have been processed before child.
    DRAKE_DEMAND(parent.nq_inboard_ >= 0 && parent.nv_inboard_ >= 0);
    mobod.nq_inboard_ = parent.nq_inboard_ + mobod.nq_;
    mobod.nv_inboard_ = parent.nv_inboard_ + mobod.nv_;

    // Map coordinates to their Mobod.
    DRAKE_DEMAND(ssize(data_.q_to_mobod) == next_q &&
                 ssize(data_.v_to_mobod) == next_v);
    for (int i = 0; i < mobod.nq_; ++i)
      data_.q_to_mobod.push_back(mobod.index());
    for (int i = 0; i < mobod.nv_; ++i)
      data_.v_to_mobod.push_back(mobod.index());

    next_q += mobod.nq_;
    next_v += mobod.nv_;
  }

  // O(n) inward pass counts outboard bodies and coordinates for each Mobod.
  // TODO(sherm1) Clean this up with a reverse range view in C++20.
  for (auto m = data_.mobods.rbegin(); m != data_.mobods.rend(); ++m) {
    m->nq_outboard_ = m->nv_outboard_ = 0;
    m->num_subtree_mobods_ = 1;  // Count this one.
    for (MobodIndex child_index : m->outboard_mobods_) {
      const Mobod& child = mobods(child_index);
      // Children should have been processed before parent.
      DRAKE_DEMAND(child.nq_outboard_ >= 0 && child.nv_outboard_ >= 0 &&
                   child.num_subtree_mobods_ >= 0);
      m->nq_outboard_ += child.nq_outboard_ + child.nq_;
      m->nv_outboard_ += child.nv_outboard_ + child.nv_;
      m->num_subtree_mobods_ += child.num_subtree_mobods_;
    }
  }
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
      mutable_graph().AddModelingJointToWorld(joint_type_index, link_index);
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

    // Both appear as child Links equally often.

    const int lnp = ssize(left_link.joints_as_parent());
    const int rnp = ssize(right_link.joints_as_parent());
    if (lnp < rnp) return true;   // left is a parent less often (bad)
    if (lnp > rnp) return false;  // right is a parent less often (good)

    // Both appear as child & parent Links equally often; take lowest index.
    return left > right;
  };
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
