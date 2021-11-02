#pragma once

#include <unordered_map>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/query_results/signed_distance_pair.h"
#include "drake/math/rigid_transform.h"

/** @file Provides the structures and logic to support signed distance queries
 between shapes.  */

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

/* Supporting data for the shape-to-shape signed distance callback (see
 Callback below). It includes:

    - A collision filter instance.
    - The T-valued poses of _all_ geometries in the corresponding SceneGraph
      keyed on their GeometryId.
    - A vector of distance results -- one instance of SignedDistancePair for
      every supported geometry which lies within the threshold.

 @tparam T The computation scalar.  */
template <typename T>
struct CallbackData {
  /* Constructs the mostly-specified callback data. The fcl distance request is
   left in its default constructed state for subsequent configuration. The
   values are as described in the class documentation. The parameters are almost
   all aliased in the data and require the aliased parameters to remain valid at
   least as long as the CallbackData instance.

   @param collision_filter_in     The collision filter system. Aliased.
   @param X_WGs_in                The T-valued poses. Aliased.
   @param max_distance_in         The maximum distance at which a pair is
                                  reported.
   @param nearest_pairs_in[out]   The output results. Aliased.  */
  CallbackData(
      const CollisionFilter* collision_filter_in,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_in,
      double max_distance_in,
      std::vector<SignedDistancePair<T>>* nearest_pairs_in)
      : collision_filter(*collision_filter_in),
        X_WGs(*X_WGs_in),
        max_distance(max_distance_in),
        nearest_pairs(*nearest_pairs_in) {
    DRAKE_DEMAND(collision_filter_in != nullptr);
    DRAKE_DEMAND(X_WGs_in != nullptr);
    DRAKE_DEMAND(nearest_pairs_in != nullptr);
  }

  /* The collision filter system.  */
  const CollisionFilter& collision_filter;

  /* The T-valued poses of all geometries.  */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /* The maximum distance at which a pair's distance will be reported.  */
  const double max_distance;

  /* The distance query parameters.  */
  fcl::DistanceRequestd request;

  /* The results of the distance query.  */
  std::vector<SignedDistancePair<T>>& nearest_pairs{};
};

/* A functor to support ComputeNarrowPhaseDistance(). It computes the signed
 distance between a supported pair of geometries. Each overload to the call
 operator reports the signed distance (encoded in SignedDistancePair<T>)
 between the two given geometry arguments using the functor's stored poses.

 @tparam T  Computation scalar type.  */
template <typename T>
class DistancePairGeometry {
 public:
  /* Constructs the functor caching all parameters that do _not_ depend on
   the shape type.

   @param id_A         Identifier of the first geometry passed to operator().
   @param id_B         Identifier of the second geometry passed to operator().
   @param X_WA         Pose of geometry A in the world frame.
   @param X_WB         Pose of geometry B in the world frame
   @param[out] result  The signed distance values are stored here.  */
  DistancePairGeometry(const GeometryId& id_A, const GeometryId& id_B,
                       const math::RigidTransform<T>& X_WA,
                       const math::RigidTransform<T>& X_WB,
                       SignedDistancePair<T>* result)
      : id_A_(id_A), id_B_(id_B), X_WA_(X_WA), X_WB_(X_WB), result_(result) {
    DRAKE_ASSERT(result != nullptr);
  }

  /* @name  Overloads in support of sphere-shape computation

   Given a sphere A centered at Ao with radius r and another geometry B,
   we want to compute
   1. φ_A,B = the signed distance between the two objects, which is positive
      for non-overlapping objects and equals the negative penetration depth
      for overlapping objects.
   2. Na, Nb = a pair of witness points, Na ∈ ∂A, Nb ∈ ∂B (not necessarily
      unique), |Na-Nb| = |φ_A,B|.

   Define these functions: (available from SignedDistanceToPoint)
     φ_B:ℝ³→ℝ, φ_B(p)  = signed distance to point p from B.
     η_B:ℝ³→ℝ³, η_B(p) = a nearest point to p on the boundary ∂B (not
                         necessarily unique).
     ∇φ_B:ℝ³→ℝ³, ∇φ_B(p) = gradient vector of φ_B with respect to p.
                           It has unit length by construction.
   Algorithm:
   1. φ_A,B = φ_B(Ao) - r
   2. Nb = η_B(Ao)
   3. Na = Ao - r * ∇φ_B(Ao)  */
  //@{

  void operator()(const fcl::Sphered& sphere_A, const fcl::Sphered& sphere_B);

  void operator()(const fcl::Sphered& sphere_A, const fcl::Boxd& box_B);

  void operator()(const fcl::Sphered& sphere_A,
                  const fcl::Cylinderd& cylinder_B);

  void operator()(const fcl::Sphered& sphere_A,
                  const fcl::Halfspaced& halfspace_B);

  void operator()(const fcl::Sphered& sphere_A, const fcl::Capsuled& capsule_B);

  //@}

 private:
  // Distance computation between a sphere A and a generic shape B. We use
  // the overloaded call operators above to limit the kinds of queries, and
  // they all call this private template function to minimize code duplication.
  template <typename FclShape>
  void SphereShapeDistance(const fcl::Sphered& sphere_A,
                           const FclShape& shape_B);

  GeometryId id_A_;
  GeometryId id_B_;
  math::RigidTransform<T> X_WA_;
  math::RigidTransform<T> X_WB_;
  SignedDistancePair<T>* result_;
};

/* @name  Definition of fallback for missing primitive-primitive tests

 Generally, we favor hand-crafted functions for determining the signed distance
 between two shapes. However, just because we have no such function for a
 particular geometry pair does not mean we have no recourse for computing
 signed distance. Instead, we have the "fallback" function.

 The fallback function is a simple interface that takes two fcl collision
 objects (called `a` and `b`), fcl distance request parameters, and a pointer
 to the SignedDistancePair which will be populated with the results of the
 query.

 Currently, our ability to fallback depends on the scalar type. This family
 of functions defines the fallback logic for the various scalar types. It
 applies a _whitelist_-based approach in that we assume there is no fallback
 for a scalar type unless one is explicitly provided.  */
//@{

/* For all non-whitelisted scalar types T, throws an exception declaring the
 unsupported combination of geometry types and scalar type.  */
template <typename T>
void CalcDistanceFallback(const fcl::CollisionObjectd& a,
                          const fcl::CollisionObjectd& b,
                          const fcl::DistanceRequestd&,
                          SignedDistancePair<T>* /* pair_data */) {
  // By default, there is no fallback. For every scalar type for which one
  // actually exists, it should be specialized below.
  throw std::logic_error(fmt::format(
      "Signed distance queries between shapes '{}' and '{}' "
      "are not supported for scalar type {}",
      GetGeometryName(a), GetGeometryName(b), NiceTypeName::Get<T>()));
}

/* For the double scalar, computes the signed distance between the two objects.
 */
template <>
void CalcDistanceFallback<double>(const fcl::CollisionObjectd& a,
                                  const fcl::CollisionObjectd& b,
                                  const fcl::DistanceRequestd& request,
                                  SignedDistancePair<double>* pair_data);

//@}

/* Reports if the given geometries require using the fallback. */
bool RequiresFallback(const fcl::CollisionObjectd& a,
                      const fcl::CollisionObjectd& b);

/* Dispatches the narrowphase shape-shape query for the object pair (`a`, `b`)
 to the appropriate primitive-primitive function (optionally defaulting to the
 type- and shape-dependent fallback function).

 @param a               The first object in the pair.
 @param X_WA            The pose of object `a` expressed in the world frame.
 @param b               The second object in the pair.
 @param X_WB            The pose of object `b` expressed in the world frame.
 @param request         The distance request parameters.
 @param result          The structure to capture the computation results in.
 @tparam T Computation scalar type.
 @pre The pair should *not* be (Halfspace, X), unless X is Sphere.  */
template <typename T>
void ComputeNarrowPhaseDistance(const fcl::CollisionObjectd& a,
                                const math::RigidTransform<T>& X_WA,
                                const fcl::CollisionObjectd& b,
                                const math::RigidTransform<T>& X_WB,
                                const fcl::DistanceRequestd& request,
                                SignedDistancePair<T>* result);

// TODO(SeanCurtis-TRI): Replace this clunky mechanism with a new mechanism
// which does this implicitly via ADL and templates.
/* @name   Mechanism for reporting on which scalars and for which shape-pairs
            shape-to-shape queries can be made.

 By default, nothing is supported. For each supported scalar type, a class
 specialization is provided which whitelists the supported shape types.

 @tparam T      The computational scalar type.  */
//@{

template <typename T>
struct ScalarSupport {
  static bool is_supported(fcl::NODE_TYPE node1, fcl::NODE_TYPE node2) {
    return false;
  }
};

/* Primitive support for double-valued query.  */
template <>
struct ScalarSupport<double> {
  static bool is_supported(fcl::NODE_TYPE node1, fcl::NODE_TYPE node2);
};

/* Primitive support for AutoDiff-valued query.  */
template <typename DerType>
struct ScalarSupport<Eigen::AutoDiffScalar<DerType>> {
  static bool is_supported(fcl::NODE_TYPE node1, fcl::NODE_TYPE node2) {
    // TODO(SeanCurtis-TRI): Confirm derivatives for sphere-capsule.
    // Explicitly permit the following pair types (with ordering
    // permutations):
    //  (sphere, sphere)
    //  (sphere, box)
    //  (sphere, halfspace)
    return (node1 == fcl::GEOM_SPHERE &&
            (node2 == fcl::GEOM_SPHERE || node2 == fcl::GEOM_BOX ||
             node2 == fcl::GEOM_HALFSPACE)) ||
           (node2 == fcl::GEOM_SPHERE &&
            (node1 == fcl::GEOM_BOX || node1 == fcl::GEOM_HALFSPACE));
  }
};

//@}

/* The callback function for computing signed distance between two arbitrary
 shapes.

 This callback is used in the context where we're going to report the distance
 between all O(N²) objects, regardless of the results. The final parameter
 in the callback serves as a threshold to the FCL broadphase -- any objects
 that are known to be farther than the `dist` value will not be dispatched to
 this narrow phase callback. This query does not *reduce* the distance so
 it does not set it.

 @tparam T  The scalar type for the query.
 @param object_A_ptr    Pointer to the first object in the pair (the order has
                        no significance).
 @param object_B_ptr    Pointer to the second object in the pair (the order has
                        no significance).
 @param callback_data   Supporting data to compute distance. This includes the
                        results of this narrowphase computation.
 @param max_distance    The maximum distance the broadphase should consider
                        for candidate pairs.
 @returns False; the broadphase should *not* terminate its process.  */
template <typename T>
bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr,
              // NOLINTNEXTLINE
              void* callback_data, double& max_distance);

}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
