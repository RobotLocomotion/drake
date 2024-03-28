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

using WeldedMobodsIndex = TypeSafeIndex<class WeldedMobodsTag>;

/** SpanningForest models a LinkJointGraph via a set of spanning trees and
loop-closing constraints. This is a directed forest, with edges ordered from
inboard (closer to World) to outboard. The nodes are mobilized bodies (Mobods)
representing a body and its inboard mobilizer. While building the Forest we also
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
tree root nodes (a.k.a. "base bodies") to World via floating or weld mobilizers.

When extra bodies, mobilizers, and constraints are needed to construct the
Forest, corresponding Links, Joints, and LoopConstraints are added to the
LinkJointGraph so a user can view the Forest as an extended LinkJointGraph (the
original graph is not modified). For example, added 6dof mobilizers can be
viewed as though they had been floating Joints in the LinkJointGraph.

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
  - find out what Links, Joints, and LoopConstraints appear in the Forest but
      not the source Graph.

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

  /** Returns a reference to the graph that owns this forest (as set during
  construction). */
  const LinkJointGraph& graph() const {
    DRAKE_ASSERT(data_.graph != nullptr);
    return *data_.graph;
  }

  /** Returns `true` if this forest is up to date with respect to its owning
  graph. */
  bool is_valid() const { return graph().forest_is_valid(); }

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

 private:
  friend class LinkJointGraph;
  friend class copyable_unique_ptr<SpanningForest>;

  // Only the owning LinkJointGraph may call this constructor. The owner must
  // outlive this model, which may be built and rebuilt repeatedly for the same
  // owner. For copy and move, the owner is responsible for repairing the back
  // pointer via SetNewOwner().
  // @pre `graph` is non-null
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

  // Clears the existing forest and builds a new one using the current forest
  // building options in the owning LinkJointGraph. This is for LinkJointGraph
  // to call _after_ it has cleaned out its own ephemeral elements and
  // out-of-date modeling information. New ephemeral elements and modeling info
  // will be written to the graph.
  void BuildForest();

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

    // A valid SpanningForest always has at least one mobilized body: World.
    std::vector<Mobod> mobods;

    // If we had to break any loops we'll add constraints to re-close them.
    std::vector<LoopConstraint> loop_constraints;

    // How the forest is partitioned into trees.
    std::vector<Tree> trees;

    // The height of the tallest tree in the forest. Always at least 1 when
    // valid since World is always present in a forest.
    int forest_height{0};

    // Welded Mobod groups. These are disjoint sets of Mobods that have no
    // relative motion due to _modeled_ weld Joints. (If we are building
    // Composite Links, there won't be any Mobods welded together because we'll
    // make all welded-together Links follow a single Mobod.) The World Mobod
    // group is always present and comes first even if nothing is welded to it.
    // The indexes here must be renumbered by FixupForestToUseNewNumbering().
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
