#pragma once

#include <memory>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/autodiff.h"
#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

template <typename T> class GeometryState;

namespace internal {

/** Specification of a pair of geometry indices. Serves as part of the query
 interface to allow queries on explicitly itemized pairs of geometries. */
struct GeometryIndexPair {
  GeometryIndexPair(GeometryIndex i1, GeometryIndex i2)
      : index1(i1), index2(i2) {}
  GeometryIndex index1;
  GeometryIndex index2;
};

/** The underlying engine for performing geometric queries.
 It owns the geometry instances and, once it has been provided with the poses
 of the geometry, it provides geometric queries on that geometry.

 @internal Historically, this replaces the DrakeCollision::Model class.

 @tparam T The underlying scalar type. Must be a valid Eigen scalar. */
template <typename T>
class FclEngine : public ShapeReifier {
 public:
  FclEngine() = default;

  /** Construct a deep copy of the provided `other` engine. */
  FclEngine(const FclEngine& other);
  FclEngine& operator=(const FclEngine& other);
  FclEngine(FclEngine&& other) noexcept;
  FclEngine& operator=(FclEngine&& other) noexcept;

  /** Returns an independent copy of this engine templated on the AutoDiffXd
   scalar type. If the engine is already an AutoDiffXd engine, it is equivalent
   to using the copy constructor to create a duplicate on the heap. */
  std::unique_ptr<FclEngine<AutoDiffXd>> ToAutoDiff() const;

  /** @name Topology management */
  //@{

  /** Adds the given `shape` to the engine's dynamic geometry.  */
  GeometryIndex AddDynamicGeometry(const Shape& shape);

  /** Adds the given `shape` with the given pose in the world frame, `X_WG`, to
   the world.  */
  AnchoredGeometryIndex AddAnchoredGeometry(const Shape& shape,
                                            const Isometry3<double>& X_WG);

  /** Reports the _total_ number of geometries in the engine -- dynamic and
   anchored.  */
  int num_geometries() const {
    return static_cast<int>(dynamic_objects_.size() + anchored_objects_.size());
  }

  /** Reports the number of _dynamic_ geometries (spanning all sources). */
  int num_dynamic() const {
    return static_cast<int>(dynamic_objects_.size());
  }

  /** Reports the number of _anchored_ geometries (spanning all sources). */
  int num_anchored() const {
    return static_cast<int>(anchored_objects_.size());
  }

  //@}

  /** Updates the poses for all of the dynamic geometries in the engine. It
   is an invariant that _every_ registered dynamic geometry, across _all_
   geometry sources, has a _unique_ index that lies in the range
   [0, num_dynamic() - 1]. Therefore the vector should have size equal to
   num_dynamics() and any other length will cause program failure. The iᵗʰ entry
   contains the pose for the geometry whose GeometryIndex value is `i`.
   @param X_WG     The poses of each geometry `G` measured and expressed in the
                   world frame `W`. */
  // TODO(SeanCurtis-TRI): I could do things here differently a number of ways:
  //  1. I could make this move semantics (or swap semantics).
  //  2. I could simply have a method that returns a mutable reference to such
  //    a vector and the caller sets values there directly.
  void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WG);

  /** @name Implementation of ShapeReifier interface  */
  //@{

  void ImplementGeometry(const Sphere& sphere) override;
  void ImplementGeometry(const Cylinder& cylinder) override;
  void ImplementGeometry(const HalfSpace& half_space) override;

  //@}

 private:
  // Engine on one scalar can see the members of other engines.
  template <typename> friend class FclEngine;

  // Facilitate testing.
  friend class FclEngineTester;

  // TODO(SeanCurtis-TRI): Convert these to scalar type T when I know how to
  // transmogrify them. Otherwise, while the engine can be transmogrified, the
  // results on an <AutoDiffXd> type will still be double.

  // Helper method called by the various ImplementGeometry overrides to
  // facilitate the logistics of creating shapes from specifications.
  void TakeShapeOwnership(const std::shared_ptr<fcl::ShapeBased>& shape);

  // This is the last geometry that was implemented. It is set by the Implement
  // methods and cleared by the Add*Geometry() methods.
  std::unique_ptr<fcl::CollisionObject<double>> last_implemented_;

  // The BVH of all dynamic geometries; this depends on *all* inputs.
  // TODO(SeanCurtis-TRI): Ultimately, this should probably be a cache entry.
  fcl::DynamicAABBTreeCollisionManager<double> dynamic_tree_;

  // All of the *dynamic* collision elements (spanning all sources). Their
  // GeometryIndex maps to their position in *this* vector.
  // TODO(SeanCurtis-TRI): Cluster the geometries on source such that each
  // source owns a _contiguous_ block of engine indices.
  std::vector<std::unique_ptr<fcl::CollisionObject<double>>> dynamic_objects_;

  // The tree containing all of the anchored geometry.
  fcl::DynamicAABBTreeCollisionManager<double> anchored_tree_;

  // All of the *anchored* collision elements (spanning *all* sources). Their
  // AnchoredGeometryIndex maps to their position in *this* vector.
  std::vector<std::unique_ptr<fcl::CollisionObject<double>>> anchored_objects_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
