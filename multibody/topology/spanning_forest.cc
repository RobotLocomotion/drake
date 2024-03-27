// NOLINTNEXTLINE(build/include): prevent complaint re spanning_forest.h
#include <queue>
#include <stack>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"
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

// Clear() leaves this with nothing, not even a Mobod for World.
void SpanningForest::Clear() {
  // We want the graph back pointer to remain unchanged.
  LinkJointGraph* const saved_graph = data_.graph;
  data_ = Data{};
  data_.graph = saved_graph;
}

// TODO(sherm1) The following describes the full algorithm but the code
//  currently just has stubs. See PR #20225 for the implementation.

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
    a massful body. Welded-together Links form a Link Composite which can
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

Here is an expansion of the above three phases:

1. Construct a forest with good topology
   - Add the World mobilized body to the forest at level 0.
   - Add missing Joints to World if needed for (a) Static Links,
     and (b) Links marked "MustBeBaseBody". (Links can be static due
     to individual LinkFlags or membership in a static model instance.)
   - ExtendTrees(all existing base bodies)   (see algorithm below)
   - While there are unprocessed _jointed_ (articulated) Links
       - Choose an unprocessed Link L and model it with a new Mobod B that is
         to be the root node (base body) of a new tree. L is chosen by picking
         the "best" base Link according to some policy.
         (Heuristic: pick one that never appears as a child of a Joint.)
       - ExtendTrees(B)  (just this one tree; see algorithm below)
   - While there are unprocessed _unjointed_ Links (single bodies)
       - Each of these can be the base body of a new one-Mobod tree
       - If a Link is in a "use fixed base" model instance, weld it to World.
         If we're combining welded links it just joins the World LinkComposite
         and does not form a new Tree.
       - Otherwise add a floating Joint to World (rpy or quaternion depending
         on options) and start a new Tree.
       - Note that any unjointed _static_ Link, or a Link in a static model
         instance, had a weld joint added in the second step above so was
         processed in the first call to ExtendTrees().

   Algorithm ExtendTrees(tree_base_bodies):
       - Extend each tree one level at a time.
       - Process all level 1 (base) Links directly jointed to World.
       - Then process all level 2 Links jointed to level 1 Links, etc.
       - Composites are (optionally) modeled with a single Mobod (one level).
       - If we encounter a Link that has already been processed there is a
         loop. Split the Link onto primary and shadow Mobods and add a loop
         constraint to weld them back together.

2. Reorder depth first
   - Determine the depth first re-ordering of the mobilized bodies.
   - Use that reordering to update the SpanningForest in place.

3. Assign coordinates q and velocities v
   - Each tree gets a consecutive set of q's and a consecutive set of v's,
     doled out following the depth-first ordering of the trees.
   - Build maps from q and v to corresponding Mobod (includes fast q,v->Tree
     also).
*/
void SpanningForest::BuildForest() {
  Clear();  // In case we're reusing this forest.

  // Model the World (Link 0) with mobilized body 0. This also starts the 0th
  // Link Composite in the graph and the 0th Welded Mobods group in the Forest.
  data_.mobods.emplace_back(MobodIndex(0), BodyIndex(0));
  data_.welded_mobods.emplace_back(std::vector{MobodIndex(0)});
  data_.mobods[MobodIndex(0)].welded_mobods_index_ = WeldedMobodsIndex(0);
  data_.graph->set_primary_mobod_for_link(BodyIndex(0), MobodIndex(0),
                                          JointIndex{});
  data_.graph->CreateWorldLinkComposite();

  data_.forest_height = 1;  // Just World so far.

  // Decide on forward/reverse mobilizers; combine composite links onto a single
  // Mobod; choose links to serve as base (root) bodies and add 6dof mobilizers
  // for them; split loops; add shadow bodies and weld constraints.
  ChooseForestTopology();

  // Determine the desired depth-first ordering and apply it.
  const std::vector<MobodIndex> old_to_new = CreateDepthFirstReordering();
  FixupForestToUseNewNumbering(old_to_new);

  // Dole out the q's and v's, depth-first.
  AssignCoordinates();
}

void SpanningForest::ChooseForestTopology() {
  // Stub; see #20225.

  // Fill in World info that will be calculated in the implementation.
  Mobod& world = data_.mobods[MobodIndex(0)];
  world.nq_outboard_ = 0;  // Nothing other than World.
  world.nv_outboard_ = 0;
  // World isn't part of a tree but gets counted here.
  world.num_subtree_mobods_ = 1;
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
  DRAKE_DEMAND(ssize(mobods()) == 1);  // Just World in this stub.
  const std::vector<MobodIndex> old_to_new{MobodIndex(0)};  // 0 maps to 0.
  // Stub; see #20225.
  return old_to_new;
}

/* Applies the given mapping to renumber all Mobods and references to them
within the forest and its owning graph. */
void SpanningForest::FixupForestToUseNewNumbering(
    const std::vector<MobodIndex>& old_to_new) {
  // Stub; see #20225.
  unused(old_to_new);
}

/* Dole out the q's and v's to each of the Mobods, following the depth-first
ordering. */
void SpanningForest::AssignCoordinates() {
  // Stub; see #20225.
}

// TODO(sherm1) Remove this.
// To permit testing the APIs of Tree and LoopConstraint before the implementing
// code is merged, we'll stub a forest that looks like this:
//
//            -> mobod1 => mobod2
//     World                 ^
//            -> mobod3 .....|  loop constraint
//
// There are two trees and a loop constraint where mobod3 is primary and
// mobod2 is the shadow. The two joints to World have 1 dof, the "=>" joint
// is a weld with 0 dofs.
//
// Note that there are no graph elements corresponding to any of this stuff
// in the stub; we're just testing SpanningForest APIs here which don't care.
// This will not be a well-formed forest for other purposes!
void SpanningForest::AddStubTreeAndLoopConstraint() {
  // Add three dummy Mobods.
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

  // Assign depth-first coordinates.
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
