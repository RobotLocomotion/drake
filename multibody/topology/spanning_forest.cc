// NOLINTNEXTLINE(build/include): prevent complaint re spanning_forest.h
#include <queue>
#include <stack>
#include <utility>
#include <vector>

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

ForestBuildingOptions (global or per-model instance) determine
  - whether we produce a mobilized body for _each_ Link or just for each
    _group_ of Links that are welded together into a composite body, and
  - what kind of Joint we use to mobilize unconnected root Links: 0 dof fixed,
    6 dof roll-pitch-yaw floating, or 6 dof quaternion floating.

1. Construct a forest with good topology
   - Add the World mobilized body to the forest at level 0.
   - Add missing Joints to World if needed for (a) Static Links,
     and (b) Links marked "MustBeBaseBody".
   - Extend each tree a level at a time
       - Process all level 1 (base) Links directly jointed to World
       - Then process all level 2 Links jointed to level 1 Links, etc.
       - Composites are treated as single bodies (one level)
   - While there are unprocessed _jointed_ (articulated) Links
       - Choose the "best" base body B according to some policy.
         (Heuristic: pick one that never appears as a child.)
       - Process the tree rooted at B by levels as above.
   - While there are unprocessed _unjointed_ Links (single bodies)
       - Add a 6 dof or fixed Joint to World for each Link as instructed
       - Each 6 dof (free) body is a new tree of one mobilized body
       - Each welded (fixed) body is anchored and part of the World
         Welded Mobods group (if we're combining composites)

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

  // Model the World (Link 0) with mobilized body 0. This also starts the 0th
  // Link Composite in the graph and the 0th Welded Mobods group in the Forest.
  data_.mobods.emplace_back(MobodIndex(0), BodyIndex(0));
  data_.welded_mobods.emplace_back(std::vector{MobodIndex(0)});
  data_.mobods[MobodIndex(0)].welded_mobods_index_ = WeldedMobodsIndex(0);
  data_.graph->set_primary_mobod_for_link(BodyIndex(0), MobodIndex(0),
                                          JointIndex{});
  data_.graph->CreateWorldLinkComposite();

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
}

void SpanningForest::ChooseForestTopology() {
  // Stub; see #20225.
  // Fill in World info that will be calculated in the implementation.
  Mobod& world = data_.mobods[MobodIndex(0)];
  world.nq_outboard_ = 0;  // Nothing other than World.
  world.nv_outboard_ = 0;
  world.num_subtree_mobods_ = 0;  // World is not part of any Tree.
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

SpanningForest::Data::Data() = default;
SpanningForest::Data::Data(const Data&) = default;
SpanningForest::Data::Data(Data&&) = default;
SpanningForest::Data::~Data() = default;
auto SpanningForest::Data::operator=(const Data&) -> Data& = default;
auto SpanningForest::Data::operator=(Data&&) -> Data& = default;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
