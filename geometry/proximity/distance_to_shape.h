#pragma once

#include <limits>
#include <utility>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/collision_filter_legacy.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {
namespace shape_distance {

// Struct for use in DistanceCallback(). Contains the distance request
// and accumulates result in a drake::geometry::SignedDistancePair vector.
struct CallbackData {
  CallbackData(const std::vector<GeometryId>* geometry_map_in,
               const CollisionFilterLegacy* collision_filter_in)
      : geometry_map(*geometry_map_in),
        collision_filter(*collision_filter_in) {}
  // Maps so the distance call back can map from engine index to geometry id.
  const std::vector<GeometryId>& geometry_map;
  const CollisionFilterLegacy& collision_filter;

  // Distance request
  fcl::DistanceRequestd request;

  // Vectors of distance results
  std::vector<SignedDistancePair<double>>* nearest_pairs{};
};

// An internal functor to support ComputeNarrowPhaseDistance(). It computes
// the signed distance between a supported pair of geometries.  Each overload
// to the call operator reports the signed distance (encoded in
// fcl::DistanceResultd) between the two given geometry arguments using the
// functor's stored poses.
class DistancePairGeometry {
 public:
  // @param result   We report the signed distance and the witness points in
  //                 the struct fcl::DistanceResultd pointed by `result`.
  // @note Those aspects of the query geometry that do not depend on the
  // shape type are provided to the constructor. The overloaded call operator
  // takes the actual shape types.
  DistancePairGeometry(const GeometryId& id_A, const GeometryId& id_B,
                       const math::RigidTransformd& X_WA,
                       const math::RigidTransformd& X_WB,
                       fcl::DistanceResultd* result)
      : id_A_(id_A), id_B_(id_B), X_WA_(X_WA), X_WB_(X_WB), result_(result) {}

  // Given a sphere A centered at Ao with radius r and another geometry B,
  // we want to compute
  // 1. φ_A,B = the signed distance between the two objects, which is positive
  //    for non-overlapping objects and equals the negative penetration depth
  //    for overlapping objects.
  // 2. Na, Nb = a pair of witness points, Na ∈ ∂A, Nb ∈ ∂B (not necessarily
  //    unique), |Na-Nb| = |φ_A,B|.
  //
  // Define these functions: (available from SignedDistanceToPoint)
  //   φ_B:ℝ³→ℝ, φ_B(p)  = signed distance to point p from B.
  //   η_B:ℝ³→ℝ³, η_B(p) = a nearest point to p on the boundary ∂B (not
  //                       necessarily unique).
  //   ∇φ_B:ℝ³→ℝ³, ∇φ_B(p) = gradient vector of φ_B with respect to p.
  //                         It has unit length by construction.
  // Algorithm:
  // 1. φ_A,B = φ_B(Ao) - r
  // 2. Nb = η_B(Ao)
  // 3. Na = Ao - r * ∇φ_B(Ao)
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

 private:
  // Distance computation between a sphere A and a generic shape B. We use
  // the overloaded call operators above to limit the kinds of queries, and
  // they all call this private template function to minimize code duplication.
  template <typename FclShape>
  void SphereShapeDistance(const fcl::Sphered& sphere_A,
                           const FclShape& shape_B) {
    const SignedDistanceToPoint<double> shape_B_to_point_Ao =
        point_distance::DistanceToPoint<double>(id_B_, X_WB_,
                                                X_WA_.translation())(shape_B);
    const double distance = shape_B_to_point_Ao.distance - sphere_A.radius;
    // Nb is the witness point on ∂B.
    const Eigen::Vector3d& p_BNb = shape_B_to_point_Ao.p_GN;
    const Eigen::Vector3d p_WNb = X_WB_ * p_BNb;
    const Eigen::Vector3d& gradB_W = shape_B_to_point_Ao.grad_W;
    // Na is the witness point on ∂A.
    const Eigen::Vector3d p_WNa =
        WitnessPointOnSphere(sphere_A.radius, X_WA_, gradB_W);
    // fcl::DistanceResult expects -1 for primitive shapes.
    result_->update(distance, &sphere_A, &shape_B, -1, -1, p_WNa, p_WNb);
  }
  // Performs step 3. Na = Ao - r * ∇φ_B(Ao).
  // @param radius_A the radius of the sphere A.
  // @param X_WA the pose of the sphere A.
  // @param gradB_W the gradient vector ∇φ_B(Ao).
  // @retval p_WNa the witness point Na ∈ ∂A expressed in World frame.
  Eigen::Vector3d WitnessPointOnSphere(
      const double radius_A, const math::RigidTransformd& X_WA,
      const Eigen::Vector3d& gradB_W) const {
    // Notation:
    // gradB_W = ∇φ_B(Ao) expressed in World frame.
    // gradB_A = ∇φ_B(Ao) expressed in A's frame.
    const math::RotationMatrixd& R_WA = X_WA.rotation();
    const math::RotationMatrixd R_AW = R_WA.transpose();
    const Eigen::Vector3d gradB_A = R_AW * gradB_W;
    // By construction gradB_A has unit length.
    const Eigen::Vector3d p_ANa = -radius_A * gradB_A;
    const Eigen::Vector3d p_WNa = X_WA * p_ANa;
    return p_WNa;
  }

  GeometryId id_A_;
  GeometryId id_B_;
  math::RigidTransformd X_WA_;
  math::RigidTransformd X_WB_;
  fcl::DistanceResultd* result_;
};

// Helps DistanceCallback(). Do it in closed forms for sphere-sphere,
// sphere-box, or sphere-cylinder. Otherwise, use FCL GJK/EPA.
void ComputeNarrowPhaseDistance(const fcl::CollisionObjectd* a,
                                const fcl::CollisionObjectd* b,
                                const std::vector<GeometryId>& geometry_map,
                                const fcl::DistanceRequestd& request,
                                fcl::DistanceResultd* result) {
  const fcl::CollisionGeometryd* a_geometry = a->collisionGeometry().get();
  const fcl::CollisionGeometryd* b_geometry = b->collisionGeometry().get();

  // We use FCL's GJK/EPA fallback in those geometries we haven't explicitly
  // supported. However, FCL doesn't support: half spaces, planes, triangles, or
  // octtrees in that workflow. We need to give intelligent feedback rather than
  // the segfault otherwise produced.
  // NOTE: Currently this only tests for halfspace (because it is an otherwise
  // supported geometry type in SceneGraph. When meshes, planes, and/or octrees
  // are supported, this error would have to be modified.
  // TODO(SeanCurtis-TRI): Remove this test when FCL supports signed distance
  // queries for halfspaces (see issue #10905). Also see FCL issue
  // https://github.com/flexible-collision-library/fcl/issues/383.
  if (a_geometry->getNodeType() == fcl::GEOM_HALFSPACE ||
      b_geometry->getNodeType() == fcl::GEOM_HALFSPACE) {
    throw std::logic_error(
        "Signed distance queries on halfspaces are not currently supported. "
        "Try using a large box instead.");
  }

  const bool a_is_sphere =
      a->collisionGeometry().get()->getNodeType() == fcl::GEOM_SPHERE;
  const bool b_is_sphere =
      b->collisionGeometry().get()->getNodeType() == fcl::GEOM_SPHERE;
  const bool no_sphere = ((!a_is_sphere) && (!b_is_sphere));
  if (no_sphere) {
    fcl::distance(a, b, request, *result);
    return;
  }
  DRAKE_ASSERT(a_is_sphere || b_is_sphere);
  // We write `s` for the sphere object and `o` for the other object. We
  // assign either (a,b) or (b,a) to (s,o) depending on whether `a` is a
  // sphere or not. Therefore, we only need the helper DistancePairGeometry
  // that takes (sphere, other) but not (other, sphere).  This scheme helps us
  // keep the code compact; however, we might have to re-order the result
  // afterwards.
  const fcl::CollisionObjectd* s = a_is_sphere ? a : b;
  const fcl::CollisionObjectd* o = a_is_sphere ? b : a;
  const fcl::CollisionGeometryd* s_geometry = s->collisionGeometry().get();
  const fcl::CollisionGeometryd* o_geometry = o->collisionGeometry().get();
  const math::RigidTransformd X_WS(s->getTransform());
  const math::RigidTransformd X_WO(o->getTransform());
  const auto id_S = EncodedData(*s).id(geometry_map);
  const auto id_O = EncodedData(*o).id(geometry_map);
  DistancePairGeometry distance_pair(id_S, id_O, X_WS, X_WO, result);
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
    default: {
      // We don't have a closed form solution for the other geometry, so we
      // call FCL GJK/EPA.
      fcl::distance(s, o, request, *result);
      break;
    }
  }
  // If needed, re-order the result for (s,o) back to the result for (a,b).
  if (!a_is_sphere) {
    std::swap(result->o1, result->o2);
    std::swap(result->nearest_points[0], result->nearest_points[1]);
  }
}

// The callback function in fcl::distance request. The final unnamed parameter
// is `dist`, which is used in fcl::distance, that if the distance between two
// geometries is proved to be greater than `dist` (for example, the smallest
// distance between the bounding boxes containing object A and object B is
// greater than `dist`), then fcl::distance will skip this callback. In our
// case, as we want to compute the distance between any pair of geometries, we
// leave `dist` unchanged as its default value (max_double). So the last
// parameter is merely a placeholder, and not being used or updated in the
// callback.
bool Callback(fcl::CollisionObjectd* fcl_object_A_ptr,
              fcl::CollisionObjectd* fcl_object_B_ptr,
              // NOLINTNEXTLINE
              void* callback_data, double&) {
  auto& distance_data = *static_cast<CallbackData*>(callback_data);
  const std::vector<GeometryId>& geometry_map = distance_data.geometry_map;
  // We want to pass object_A and object_B to the narrowphase distance in a
  // specific order. This way the broadphase distance is free to give us
  // either (A,B) or (B,A), and the narrowphase distance will always give the
  // same result.
  const GeometryId orig_id_A = EncodedData(*fcl_object_A_ptr).id(geometry_map);
  const GeometryId orig_id_B = EncodedData(*fcl_object_B_ptr).id(geometry_map);
  const bool swap_AB = (orig_id_B < orig_id_A);
  const GeometryId id_A = swap_AB ? orig_id_B : orig_id_A;
  const GeometryId id_B = swap_AB ? orig_id_A : orig_id_B;
  // NOTE: Although this function *takes* pointers to non-const objects to
  // satisfy the fcl api, it should not exploit the non-constness to modify
  // the collision objects. We ensure this by a reference to a const version
  // and not directly use the provided pointers afterwards.
  const fcl::CollisionObjectd& fcl_object_A =
      *(swap_AB ? fcl_object_B_ptr : fcl_object_A_ptr);
  const fcl::CollisionObjectd& fcl_object_B =
      *(swap_AB ? fcl_object_A_ptr : fcl_object_B_ptr);

  // Extract the collision filter keys from the fcl collision objects. These
  // keys will also be used to map the fcl collision object back to the Drake
  // GeometryId for colliding geometries.
  const EncodedData encoding_A(fcl_object_A);
  const EncodedData encoding_B(fcl_object_B);

  const bool can_collide = distance_data.collision_filter.CanCollideWith(
      encoding_A.encoded_data(), encoding_B.encoded_data());

  if (can_collide) {
    fcl::DistanceResultd result;
    ComputeNarrowPhaseDistance(&fcl_object_A, &fcl_object_B, geometry_map,
                               distance_data.request, &result);
    const Eigen::Vector3d& p_WCa = result.nearest_points[0];
    const Eigen::Vector3d& p_WCb = result.nearest_points[1];
    const Eigen::Vector3d p_ACa = fcl_object_A.getTransform().inverse() * p_WCa;
    const Eigen::Vector3d p_BCb = fcl_object_B.getTransform().inverse() * p_WCb;
    // TODO(DamrongGuoy): For sphere-{sphere,box,cylinder} we will start
    //  working on the right nhat when min_distance is 0 or almost 0 after
    //  PR #10813 lands to avoid conflicts with this PR #10823. For now,
    //  we simply return NaN in nhat when min_distance is 0 or almost 0.
    const Eigen::Vector3d nhat_BA_W =
        (std::abs(result.min_distance) < std::numeric_limits<double>::epsilon())
        ? Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN(),
                   std::numeric_limits<double>::quiet_NaN())
        : (p_WCa - p_WCb) / result.min_distance;
    distance_data.nearest_pairs->emplace_back(id_A, id_B, p_ACa, p_BCb,
                                              result.min_distance, nhat_BA_W);
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
