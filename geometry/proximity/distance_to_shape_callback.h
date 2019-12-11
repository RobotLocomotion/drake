#pragma once

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/query_results/signed_distance_pair.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

/** @file Provides the structures and logic to support signed distance queries
 between shapes.  */

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

/** Supporting data for the shape-to-shape signed distance callback (see
 Callback below). It includes:

    - A collision filter instance.
    - The T-valued poses of _all_ geometries in the corresponding SceneGraph
      keyed on their GeometryId.
    - A vector of distance results -- one instance of SignedDistancePair for
      every supported geometry which lies within the threshold.

 @tparam T The computation scalar.  */
template <typename T>
struct CallbackData {
  /** Constructs the mostly-specified callback data. The fcl distance request is
   left in its default constructed state for subsequent configuration. The
   values are as described in the class documentation. The parameters are almost
   all aliased in the data and require the aliased parameters to remain valid at
   least as long as the CallbackData instance.

   @param collision_filter_in     The collision filter system. Aliased.
   @param X_WGs_in                The T-valued poses. Aliased.
   @param max_distance_in         The maximum distance at which a pair is
                                  reported.
   @param nearest_pairs_in[out]   The output results. Aliased.  */
  CallbackData(const CollisionFilterLegacy* collision_filter_in,
               const std::unordered_map<GeometryId,
               math::RigidTransform<T>>* X_WGs_in,
               const double max_distance_in,
               std::vector<SignedDistancePair<T>>* nearest_pairs_in)
      : collision_filter(*collision_filter_in),
        X_WGs(*X_WGs_in),
        max_distance(max_distance_in),
        nearest_pairs(*nearest_pairs_in) {
    DRAKE_DEMAND(collision_filter_in);
    DRAKE_DEMAND(X_WGs_in);
    DRAKE_DEMAND(nearest_pairs_in);
  }

  /** The collision filter system.  */
  const CollisionFilterLegacy& collision_filter;

  /** The T-valued poses of all geometries.  */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /** The maximum distance at which a pair's distance will be reported.  */
  const double max_distance;

  /** The distance query parameters.  */
  fcl::DistanceRequestd request;

  /** The results of the distance query.  */
  std::vector<SignedDistancePair<T>>& nearest_pairs{};
};

/** A functor to support ComputeNarrowPhaseDistance(). It computes the signed
 distance between a supported pair of geometries. Each overload to the call
 operator reports the signed distance (encoded in SignedDistancePair<T>)
 between the two given geometry arguments using the functor's stored poses.

 @tparam T  Computation scalar type.  */
template <typename T>
class DistancePairGeometry {
 public:
  /** Constructs the functor caching all parameters that do _not_ depend on
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
    DRAKE_ASSERT(result);
  }

  /** @name  Overloads in support of sphere-shape computation

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

  void operator()(const fcl::Sphered& sphere_A, const fcl::Sphered& sphere_B) {
    SphereShapeDistance(sphere_A, sphere_B);
  }

  void operator()(const fcl::Sphered& sphere_A, const fcl::Boxd& box_B) {
    SphereShapeDistance(sphere_A, box_B);
  }

  void operator()(const fcl::Sphered& sphere_A,
                  const fcl::Cylinderd& cylinder_B) {
    SphereShapeDistance(sphere_A, cylinder_B);
  }

  void operator()(const fcl::Sphered& sphere_A,
                  const fcl::Halfspaced& halfspace_B) {
    SphereShapeDistance(sphere_A, halfspace_B);
  }

  void operator()(const fcl::Sphered& sphere_A,
                  const fcl::Capsuled& capsule_B) {
    SphereShapeDistance(sphere_A, capsule_B);
  }

  //@}

 private:
  // Distance computation between a sphere A and a generic shape B. We use
  // the overloaded call operators above to limit the kinds of queries, and
  // they all call this private template function to minimize code duplication.
  template <typename FclShape>
  void SphereShapeDistance(const fcl::Sphered& sphere_A,
                           const FclShape& shape_B) {
    const SignedDistanceToPoint<T> shape_B_to_point_Ao =
        point_distance::DistanceToPoint<T>(id_B_, X_WB_,
                                           X_WA_.translation())(shape_B);
    result_->id_A = id_A_;
    result_->id_B = id_B_;
    result_->distance = shape_B_to_point_Ao.distance - sphere_A.radius;
    // p_BCb is the witness point on ∂B measured and expressed in B.
    result_->p_BCb = shape_B_to_point_Ao.p_GN;
    result_->nhat_BA_W = shape_B_to_point_Ao.grad_W;
    result_->is_nhat_BA_W_unique =
        shape_B_to_point_Ao.is_grad_W_unique;
    // p_ACa is the witness point on ∂A measured and expressed in A.
    const math::RotationMatrix<T> R_AW = X_WA_.rotation().transpose();
    result_->p_ACa = -sphere_A.radius * (R_AW * shape_B_to_point_Ao.grad_W);
  }

  GeometryId id_A_;
  GeometryId id_B_;
  math::RigidTransform<T> X_WA_;
  math::RigidTransform<T> X_WB_;
  SignedDistancePair<T>* result_;
};

/** @name  Definition of fallback for missing primitive-primitive tests

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

/** For all non-whitelisted scalar types T, throws an exception declaring the
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

/** For the double scalar, computes the signed distance between the two objects.
 */
template <>
void CalcDistanceFallback<double>(const fcl::CollisionObjectd& a,
                                  const fcl::CollisionObjectd& b,
                                  const fcl::DistanceRequestd& request,
                                  SignedDistancePair<double>* pair_data) {
  fcl::DistanceResultd result;
  fcl::distance(&a, &b, request, result);

  pair_data->id_A = EncodedData(a).id();
  pair_data->id_B = EncodedData(b).id();

  pair_data->distance = result.min_distance;

  // Setting the witness points.
  const Eigen::Vector3d& p_WCa = result.nearest_points[0];
  pair_data->p_ACa = a.getTransform().inverse() * p_WCa;
  const Eigen::Vector3d& p_WCb = result.nearest_points[1];
  pair_data->p_BCb = b.getTransform().inverse() * p_WCb;

  // Setting the normal.
  // TODO(DamrongGuoy): We should set the tolerance through SceneGraph for
  //  determining whether the two geometries are touching or not. For now, we
  //  use this number.
  const double kEps = 1e-14;
  const double kNan = std::numeric_limits<double>::quiet_NaN();

  // Returns NaN in nhat when min_distance is 0 or almost 0.
  // TODO(DamrongGuoy): In the future, we should return nhat_BA_W as the
  //  outward face normal when the two objects are touching and set
  //  is_nhat_BA_W_unique to true.
  if (std::abs(result.min_distance) < kEps) {
    pair_data->nhat_BA_W = Eigen::Vector3d(kNan, kNan, kNan);
    pair_data->is_nhat_BA_W_unique = false;
  } else {
    pair_data->nhat_BA_W = (p_WCa - p_WCb) / result.min_distance;
    pair_data->is_nhat_BA_W_unique = true;
  }
}

//@}

/** Dispatches the narrowphase shape-shape query for the object pair (`a`, `b`)
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
                                SignedDistancePair<T>* result) {
  DRAKE_DEMAND(result != nullptr);
  const fcl::CollisionGeometryd* a_geometry = a.collisionGeometry().get();
  const fcl::CollisionGeometryd* b_geometry = b.collisionGeometry().get();

  const bool a_is_sphere = a_geometry->getNodeType() == fcl::GEOM_SPHERE;
  const bool b_is_sphere = b_geometry->getNodeType() == fcl::GEOM_SPHERE;
  const bool no_sphere = !(a_is_sphere || b_is_sphere);
  if (no_sphere) {
    CalcDistanceFallback<T>(a, b, request, result);
    return;
  }
  DRAKE_ASSERT(a_is_sphere || b_is_sphere);
  // We write `s` for the sphere object and `o` for the other object. We
  // assign either (a,b) or (b,a) to (s,o) depending on whether `a` is a
  // sphere or not. Therefore, we only need the helper DistancePairGeometry
  // that takes (sphere, other) but not (other, sphere).  This scheme helps us
  // keep the code compact; however, we might have to re-order the result
  // afterwards.
  const fcl::CollisionObjectd& s = a_is_sphere ? a : b;
  const fcl::CollisionObjectd& o = a_is_sphere ? b : a;
  const fcl::CollisionGeometryd* s_geometry = s.collisionGeometry().get();
  const fcl::CollisionGeometryd* o_geometry = o.collisionGeometry().get();
  const math::RigidTransform<T> X_WS(a_is_sphere ? X_WA : X_WB);
  const math::RigidTransform<T> X_WO(a_is_sphere ? X_WB : X_WA);
  const auto id_S = EncodedData(s).id();
  const auto id_O = EncodedData(o).id();
  DistancePairGeometry<T> distance_pair(id_S, id_O, X_WS, X_WO, result);
  const auto& sphere_S = *static_cast<const fcl::Sphered*>(s_geometry);
  switch (o_geometry->getNodeType()) {
    case fcl::GEOM_SPHERE: {
      const auto& sphere_O = *static_cast<const fcl::Sphered*>(o_geometry);
      distance_pair(sphere_S, sphere_O);
      break;
    }
    case fcl::GEOM_BOX: {
      const auto& box_O = *static_cast<const fcl::Boxd*>(o_geometry);
      distance_pair(sphere_S, box_O);
      break;
    }
    case fcl::GEOM_CYLINDER: {
      const auto& cylinder_O = *static_cast<const fcl::Cylinderd*>(o_geometry);
      distance_pair(sphere_S, cylinder_O);
      break;
    }
    case fcl::GEOM_HALFSPACE: {
      const auto& halfspace_O =
          *static_cast<const fcl::Halfspaced*>(o_geometry);
      distance_pair(sphere_S, halfspace_O);
      break;
    }
    case fcl::GEOM_CAPSULE: {
      const auto& capsule_O =
          *static_cast<const fcl::Capsuled*>(o_geometry);
      distance_pair(sphere_S, capsule_O);
      break;
    }
    default: {
      // We don't have a closed form solution for the other geometry, so we
      // call FCL GJK/EPA.
      CalcDistanceFallback<T>(a, b, request, result);
      break;
    }
  }
  // If needed, re-order the result for (s,o) back to the result for (a,b).
  if (!a_is_sphere) {
    result->SwapAAndB();
  }
}

// TODO(SeanCurtis-TRI): Replace this clunky mechanism with a new mechanism
// which does this implicitly via ADL and templates.
/** @name   Mechanism for reporting on which scalars and for which shape-pairs
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

/** Primitive support for double-valued query.  */
template <>
struct ScalarSupport<double> {
  static bool is_supported(fcl::NODE_TYPE node1, fcl::NODE_TYPE node2) {
    // Doubles (via its fallback) can support anything *except*
    // halfspace-X (where X is not sphere).
    // We use FCL's GJK/EPA fallback in those geometries we haven't explicitly
    // supported. However, FCL doesn't support: half spaces, planes, triangles,
    // or octtrees in that workflow. We need to give intelligent feedback rather
    // than the segfault otherwise produced.
    // NOTE: Currently this only tests for halfspace (because it is an otherwise
    // supported geometry type in SceneGraph. When meshes, planes, and/or
    // octrees are supported, this error would have to be modified.
    // TODO(SeanCurtis-TRI): Remove this test when FCL/Drake supports signed
    // distance queries for halfspaces (see issue #10905). Also see FCL issue
    // https://github.com/flexible-collision-library/fcl/issues/383.
    return (node1 != fcl::GEOM_HALFSPACE || node2 == fcl::GEOM_SPHERE) &&
        (node2 != fcl::GEOM_HALFSPACE || node1 == fcl::GEOM_SPHERE);
  }
};

/** Primitive support for AutoDiff-valued query.  */
template <typename DerType>
struct ScalarSupport<Eigen::AutoDiffScalar<DerType>> {
  static bool is_supported(fcl::NODE_TYPE node1, fcl::NODE_TYPE node2) {
    // Explicitly whitelist the  following pair types (with ordering
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

/** The callback function for computing signed distance between two arbitrary
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
              void* callback_data, double& max_distance) {
  auto& data = *static_cast<CallbackData<T>*>(callback_data);

  // Three things:
  //   1. We repeatedly set max_distance in each call to the callback because we
  //   can't initialize it. The cost is negligible but maximizes any culling
  //   benefit.
  //   2. Due to how FCL is implemented, passing a value <= 0 will cause results
  //   to be omitted because the bounding box test only considers *separating*
  //   distance and doesn't do any work if the distance between bounding boxes
  //   is zero.
  //   3. We pass in a number smaller than the typical epsilon because typically
  //   computation tolerances are greater than or equal to epsilon() and we
  //   don't want this value to trip those tolerances. This is safe because the
  //   bounding box test in which this is used doesn't produce a code via
  //   calculation; it is a perfect, hard-coded zero.
  const double kEps = std::numeric_limits<double>::epsilon() / 10;
  max_distance = std::max(data.max_distance, kEps);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide = data.collision_filter.CanCollideWith(
      encoding_a.encoding(), encoding_b.encoding());

  if (can_collide) {
    // Throw if the geometry-pair isn't supported.
    if (ScalarSupport<T>::is_supported(
            object_A_ptr->collisionGeometry()->getNodeType(),
            object_B_ptr->collisionGeometry()->getNodeType())) {
      // We want to pass object_A and object_B to the narrowphase distance in a
      // specific order. This way the broadphase distance is free to give us
      // either (A,B) or (B,A), but the narrowphase distance will always receive
      // the result in a consistent order.
      const GeometryId orig_id_A = encoding_a.id();
      const GeometryId orig_id_B = encoding_b.id();
      const bool swap_AB = (orig_id_B < orig_id_A);

      // NOTE: Although this function *takes* pointers to non-const objects to
      // satisfy the fcl api, it should not exploit the non-constness to modify
      // the collision objects. We ensure this by a reference to a const version
      // and not directly use the provided pointers afterwards.
      const fcl::CollisionObjectd& fcl_object_A =
          *(swap_AB ? object_B_ptr : object_A_ptr);
      const fcl::CollisionObjectd& fcl_object_B =
          *(swap_AB ? object_A_ptr : object_B_ptr);

      const GeometryId id_A =
          swap_AB ? encoding_b.id() : encoding_a.id();
      const GeometryId id_B =
          swap_AB ? encoding_a.id() : encoding_b.id();

      SignedDistancePair<T> signed_pair;
      ComputeNarrowPhaseDistance(fcl_object_A, data.X_WGs.at(id_A),
                                 fcl_object_B, data.X_WGs.at(id_B),
                                 data.request, &signed_pair);
      if (ExtractDoubleOrThrow(signed_pair.distance) <= data.max_distance) {
        data.nearest_pairs.emplace_back(std::move(signed_pair));
      }
    } else {
      throw std::logic_error(
          fmt::format("Signed distance queries between shapes '{}' and '{}' "
                      "are not supported for scalar type {}",
                      GetGeometryName(*object_A_ptr),
                      GetGeometryName(*object_B_ptr), NiceTypeName::Get<T>()));
    }
  }
  // Returning true would tell the broadphase manager to terminate early. Since
  // we want to find all the signed distance present in the model's current
  // configuration, we return false.
  return false;
}

}  // namespace shape_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
