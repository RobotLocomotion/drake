#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_optional.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_index.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

template <typename T> class GeometryState;

namespace internal {

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
  std::unique_ptr<ProximityEngine<AutoDiffXd>> ToAutoDiff() const;

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
  int num_geometries() const;

  /** Reports the number of _dynamic_ geometries (spanning all sources). */
  int num_dynamic() const;

  /** Reports the number of _anchored_ geometries (spanning all sources). */
  int num_anchored() const;

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

 private:
  // Facilitate testing; these functions exist so that the impl can be tested
  // through the public API. They are private because they are not for general
  // consumption.

  // Reports true if other is detectably a deep copy of this engine.
  bool IsDeepCopy(const ProximityEngine<T>& other) const;

  // Reports the translation of the dynamic geometry at the given index.
  Vector3<double> GetDynamicTranslation(int index) const;

  // Reports the translation of the anchored geometry at the given index.
  Vector3<double> GetAnchoredTranslation(int index) const;

  // Reports the index stored in the reified collision object at the given
  // index. It should always return the input value if the data structure is
  // maintained through geometry removal.
  int GetDynamicGeometryIndex(int index) const;

  // Reports the index stored in the reified collision object at the given
  // index. It should always return the input value if the data structure is
  // maintained through geometry removal.
  int GetAnchoredGeometryIndex(int index) const;

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

}  // namespace internal
}  // namespace geometry
}  // namespace drake
