#pragma once

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/sorted_vectors_have_intersection.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {

// TODO(SeanCurtis-TRI): Replace this "legacy" mechanism with the
// new-and-improved alternative when it is ready.
/** A simple class for providing collision filtering functionality similar to
 that found in RigidBodyTree but made compatible with fcl. The majority of
 this code is lifted verbatim from drake/multibody/collision/element.{h, cc}.

 The basic principle is that we create "cliques". If geometries A and B belong
 to the same clique, then the pair (A, B) is filtered. It is possible for A and
 B to redundantly belong to multiple cliques.

 Geometries are identified by their encoded ids (see EncodedData). Before a
 geometry can be added to a clique (AddToCollisionClique()) it must be added
 to this filter system (AddGeometry()).  */
class CollisionFilterLegacy {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterLegacy)

  CollisionFilterLegacy() = default;

  /** Adds a geometry to the filter system as represented by its encoded `id`
   (see EncodedData). When added, it will not be part of any filtered pairs.  */
  void AddGeometry(uintptr_t id) {
    collision_cliques_.insert({id, std::vector<int>()});
  }

  /** Removes the geometry represented by its encoded `id` from the data. If
   `id` has no collision filters; no action is taken.  */
  void RemoveGeometry(uintptr_t id) {
    collision_cliques_.erase(id);
  }

  /** Reports true if the geometry pair (`id_A`, `id_B`) has been explicitly
   added to the set of filtered pairs.  */
  bool CanCollideWith(uintptr_t id_A, uintptr_t id_B) const {
    // These ids should all be registered with the filter machinery.
    DRAKE_ASSERT(collision_cliques_.count(id_A) == 1);
    DRAKE_ASSERT(collision_cliques_.count(id_B) == 1);

    bool excluded = id_A == id_B ||
                    SortedVectorsHaveIntersection(collision_cliques_.at(id_A),
                                                  collision_cliques_.at(id_B));

    return !excluded;
  }

  /** Adds the previously registered geometry indicated by its encoded id
   `geometry_id` to the clique with the given `clique_id`.  */
  void AddToCollisionClique(uintptr_t geometry_id, int clique_id) {
    DRAKE_ASSERT(collision_cliques_.count(geometry_id) == 1);

    std::vector<int>& cliques = collision_cliques_[geometry_id];
    // Order(N) insertion.
    // `cliques` is a sorted vector so that checking if two collision elements
    // belong to a common group can be performed efficiently in order N.
    // See Element::CanCollideWith() and Element::collision_cliques_ for
    // explanation.
    auto it = std::lower_bound(cliques.begin(), cliques.end(), clique_id);

    // This test prevents duplicate clique ids from being added.
    if (it == cliques.end() || clique_id < *it) cliques.insert(it, clique_id);
  }

  int num_cliques(uintptr_t geometry_id) const {
    DRAKE_ASSERT(collision_cliques_.count(geometry_id) == 1);
    return static_cast<int>(collision_cliques_.at(geometry_id).size());
  }

  /** Allocates a new clique and returns its id (to use with
   AddToCollisionClique()). Allocation is _not_ threadsafe.
   @throws std::logic_error if all cliques have been allocated.  */
  int next_clique_id() {
    int clique = next_available_clique_++;
    if (clique < 0) {
      throw std::logic_error(
          "SceneGraph has run out of cliques (more than two billion served)");
    }
    return clique;
  }

  // TODO(SeanCurtis-TRI): Eliminate this method; I'm sure I can reframe the
  // tests without this.
  /** (Test support) reports what the next value that would be returned by
   next_clique_id().  */
  int peek_next_clique() const { return next_available_clique_; }

 private:
  // A map between the EncodedData::encoding() value for a geometry and
  // its set of cliques.
  std::unordered_map<uintptr_t, std::vector<int>> collision_cliques_;

  int next_available_clique_{0};
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
