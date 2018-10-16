#pragma once

#include <memory>
#include <unordered_set>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/geometry/dev/geometry_index.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/query_results/signed_distance_pair.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace dev {

template <typename T> class GeometryState;

namespace internal {

#ifndef DRAKE_DOXYGEN_CXX
// This provides GeometryState limited "friend" access to ProximityEngine for
// the purpose of collision filters.
class GeometryStateCollisionFilterAttorney;
#endif

// TODO(SeanCurtis-TRI): Swap Isometry3 for the new Transform class.

/** The underlying engine for performing geometric _proximity_ queries.
 It owns the geometry instances and, once it has been provided with the poses
 of the geometry, it provides geometric queries on that geometry.

 Proximity queries span a range of types, including:

   - penetration
   - distance
   - ray-intersection

 @tparam T The scalar type. Must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:

 - double
 - AutoDiffXd

 They are already available to link against in the containing library.
 No other values for T are currently supported.

 @internal Historically, this replaces the DrakeCollision::Model class.  */
template <typename T>
class ProximityEngine {
 public:
  ProximityEngine();
  ~ProximityEngine();

  /** Construct a deep copy of the provided `other` engine. */
  ProximityEngine(const ProximityEngine& other);

  /** Set `this` engine to be a deep copy of the `other` engine. */
  ProximityEngine& operator=(const ProximityEngine& other);

  /** Construct an engine by moving the data of a source engine. The source
   engine will be returned to its default-initialized state. */
  ProximityEngine(ProximityEngine&& other) noexcept;

  /** Move assign a source engine to this engine. The source
   engine will be returned to its default-initialized state. */
  ProximityEngine& operator=(ProximityEngine&& other) noexcept;

  /** Returns an independent copy of this engine templated on the AutoDiffXd
   scalar type. If the engine is already an AutoDiffXd engine, it is equivalent
   to using the copy constructor to create a duplicate on the heap. */
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ToAutoDiffXd() const;

  /** @name Topology management */
  //@{

  /** Adds the given `shape` to the engine's dynamic geometry. Also provides the
   dynamic geometry's internal index for the shape so it can be mapped back into
   the world. */
  ProximityIndex AddDynamicGeometry(const Shape& shape, InternalIndex index);

  /** Adds the given `shape` to the engine's anchored geometry at the fixed
   pose given by `X_WG` (in the world frame W). Also provides the
   dynamic geometry's internal index for the shape so it can be mapped back into
   the world. */
  ProximityIndex AddAnchoredGeometry(const Shape& shape,
                                     const Isometry3<double>& X_WG,
                                     InternalIndex index);

  /** Reports the _total_ number of geometries in the engine -- dynamic and
   anchored (spanning all sources).  */
  int num_geometries() const;

  /** Reports the number of _dynamic_ geometries (spanning all sources). */
  int num_dynamic() const;

  /** Reports the number of _anchored_ geometries (spanning all sources). */
  int num_anchored() const;

  /** The distance (signed/unsigned/penetration distance) is generally computed
   * from an iterative process. The distance_tolerance determines when the
   * iterative process will terminate.
   * As a rule of rule of thumb, one can generally assume that the answer will
   * be within 10 * tol to the true answer.
   */
  void set_distance_tolerance(double tol);

  double distance_tolerance() const;

  //@}

  /** Updates the poses for all of the dynamic geometries in the engine. It
   is an invariant that _every_ registered dynamic geometry, across _all_
   geometry sources, has a _unique_ index that lies in the range
   [0, num_dynamic() - 1]. `pose_indices` should have size equal to
   num_dynamics() and any other length will cause program failure. The iᵗʰ entry
   of `pose_indices` is an index into `X_WG` which contains contains the pose
   for the geometry whose GeometryIndex value is `i`.
   @param X_WG      The poses of each geometry `G` measured and expressed in the
                    world frame `W` (including geometries which may *not* be
                    registered with the proximity engine.
   @param indices   Indices into `X_WG` mapping engine index into pose index. */
  // TODO(SeanCurtis-TRI): I could do things here differently a number of ways:
  //  1. I could make this move semantics (or swap semantics).
  //  2. I could simply have a method that returns a mutable reference to such
  //    a vector and the caller sets values there directly.
  void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WG,
                        const std::vector<InternalIndex>& indices);

  // ----------------------------------------------------------------------
  /**@name              Signed Distance Queries
  See @ref signed_distance_query "Signed Distance Query" for more details. */

  //@{
  // NOTE: This maps to Model::ClosestPointsAllToAll().
  /** Determines all the closest points between any pair of bodies/elements.
   This function returns the _signed_ distance function between each pair of
   elements in @p dynamic_map (object whose pose will change), and between
   each pair between an element in @p dynamic_map and another element in
   @p anchored_map. The order and size of the closest points are invariant
   when the poses of the objects are changed.

   @param[in]   geometry_map  A map from geometry _index_ to the corresponding
                              global geometry identifier.
   @returns signed_distance_pair A vector populated with all pairs of witness
                                 points. For a pair consisting of geometries A
                                 and B, distances will always be reported as the
                                 pair (A, B). In other words, it won't sometimes
                                 be (A, B) and other times be (B, A). The pair
                                 is defined with respect to a fixed, arbitrary
                                 ordering.
   */
  std::vector<SignedDistancePair<double>>
  ComputeSignedDistancePairwiseClosestPoints(
      const std::vector<GeometryId>& geometry_map) const;
  //@}


  //----------------------------------------------------------------------------
  /** @name                Collision Queries

   These queries detect _collisions_ between geometry. Two geometries collide
   if they overlap each other and are not explicitly excluded through
   @ref collision_filter_concepts "collision filtering". These algorithms find
   those colliding cases, characterize them, and report the essential
   characteristics of that collision.  */

  //@{

  // NOTE: This maps to Model::ComputeMaximumDepthCollisionPoints().
  /** Computes the penetrations across all pairs of geometries in the world.
   Only reports results for _penetrating_ geometries; if two geometries are
   separated, there will be no result for that pair.

   The penetrations are characterized by pairs of points (providing some measure
   of the penetration "depth" of the two objects -- but _not_ the overlapping
   volume.

   This method is affected by collision filtering; geometry pairs that
   have been filtered will not produce contacts, even if their collision
   geometry is penetrating.

   For two penetrating geometries g₁ and g₂, it is guaranteed that they will
   map to `id_A` and `id_B` in a fixed, repeatable manner.

   @param[in]   geometry_map  A map from geometry _index_ to the corresponding
                              global geometry identifier.
   @returns A vector populated with all detected penetrations characterized as
            point pairs. */
  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration(
      const std::vector<GeometryId>& geometry_map) const;

  //@}

  /** @name               Collision filters

   This interface provides the mechanism through which pairs of geometries are
   removed from the "candidate pair set" for collision detection.

   See @ref scene_graph_collision_filtering "Scene Graph Collision Filtering"
   for more details.
   */
  //@{

  /** Excludes geometry pairs from collision evaluation by updating the
   candidate pair set `C = C - P`, where `P = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ G` and
   `G = dynamic ⋃ anchored = {g₀, g₁, ..., gₙ}`.
   @param[in]   dynamic     The set of _dynamic_ geometry indices for which no
                            collisions can be reported.
   @param[in]   anchored    The set of _anchored_ geometry indices for which no
                            collisions can be reported.  */
  void ExcludeCollisionsWithin(
      const std::unordered_set<InternalIndex>& dynamic,
      const std::unordered_set<InternalIndex>& anchored);

  /** Excludes geometry pairs from collision evaluation by updating the
   candidate pair set `C = C - P`, where `P = {(a, b)}, ∀ a ∈ A, b ∈ B` and
   `A = dynamic1 ⋃ anchored1 = {a₀, a₁, ..., aₘ}` and
   `B = dynamic2 ⋃ anchored2 = {b₀, b₁, ..., bₙ}`. This does _not_
   preclude collisions between members of the _same_ set.   */
  void ExcludeCollisionsBetween(
      const std::unordered_set<InternalIndex>& dynamic1,
      const std::unordered_set<InternalIndex>& anchored1,
      const std::unordered_set<InternalIndex>& dynamic2,
      const std::unordered_set<InternalIndex>& anchored2);

  //@}

 private:
  // Class to give GeometryState access to clique management.
  friend class GeometryStateCollisionFilterAttorney;

  // Retrieves the next available clique.
  int get_next_clique();

  // Assigns the given clique to the dynamic geometry indicated by `index`.
  // This is exposed via the GeometryStateCollisionFilterAttorney to allow
  // GeometryState to set up cliques between sibling geometries.
  void set_clique(InternalIndex index, int clique);

  ////////////////////////////////////////////////////////////////////////////

  // Testing utilities:
  // These functions facilitate *limited* introspection into the engine state.
  // This enables unit tests to make assertions about pre- and post-operation
  // state.

  // Reports true if other is detectably a deep copy of this engine.
  bool IsDeepCopy(const ProximityEngine<T>& other) const;

  // Reveals what the next generated clique will be (without changing it).
  int peek_next_clique() const;

  ////////////////////////////////////////////////////////////////////////////

  // TODO(SeanCurtis-TRI): Pimpl + template implementation has proven
  // problematic. This gets around it but it isn't a reliable long-term
  // solution. Figure out how to make this work with unique_ptr or
  // copyable unique_ptr
  //
  // The implementation details.
  class Impl;
  Impl* impl_{};

  // Private constructor to use for scalar conversion.
  explicit ProximityEngine(Impl* impl);

  // Engine on one scalar can see the members of other engines.
  template <typename> friend class ProximityEngine;

  // Facilitate testing.
  friend class ProximityEngineTester;
};

#ifndef DRAKE_DOXYGEN_CXX
// This is an attorney-client pattern providing GeometryState limited access to
// the collision filtering mechanism of the ProximityEngine in order to be able
// to filter collisions between geometries affixed to the same frame.
//
// This class (and the supporting functions in ProximityEngine) are short-term
// hacks. SceneGraph needs to be able to exclude collisions between geometries
// affixed to the same frame. Using the public API would lead to a proliferation
// of cliques. This exploits knowledge of the underlying representation
// (cliques) to avoid egregious redundancy; the SceneGraph explicitly
// manipulates cliques. When the legacy collision filter mechanism is removed
// (and the cliques with it), this class and its supporting functions will
// likewise go.
// TODO(SeanCurtis-TRI): Delete this with the change in collision filtering
// mechanism.
class GeometryStateCollisionFilterAttorney {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometryStateCollisionFilterAttorney);
  GeometryStateCollisionFilterAttorney() = delete;

 private:
  template <typename T>
  friend class drake::geometry::dev::GeometryState;

  // Allocates a unique, unused clique from the underlying engine's set of
  // cliques.
  template <typename T>
  static int get_next_clique(ProximityEngine<T>* engine) {
    return engine->get_next_clique();
  }

  // Assigns the given clique to the *dynamic* geometry indicated by the given
  // index. This function exists for one reason, and one reason only. To allow
  // GeometryState to automatically exclude pair (gᵢ, gⱼ) from collision if gᵢ
  // and gⱼ are affixed to the same frame.
  template <typename T>
  static void set_dynamic_geometry_clique(ProximityEngine<T>* engine,
                                          InternalIndex geometry_index,
                                          int clique) {
    engine->set_clique(geometry_index, clique);
  }

  // Utility for GeometryState tests.
  template <typename T>
  static int peek_next_clique(const ProximityEngine<T>& engine) {
    return engine.peek_next_clique();
  }
};
#endif

}  // namespace internal
}  // namespace dev
}  // namespace geometry
}  // namespace drake
