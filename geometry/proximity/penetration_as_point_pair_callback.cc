#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"

#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/proximity/distance_to_point_callback.h"
#include "drake/geometry/query_results/signed_distance_to_point.h"

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair {
using Eigen::Vector3d;

/* A functor to support ComputeNarrowPhasePenetration(). It computes the
 penetration distance between a supported pair of geometries. Each overload to
 the call operator reports the penetration distance (encoded in
 PenetrationAsPointPair<T>) between the two given geometry arguments using the
 functor's stored poses.

 @tparam T  Computation scalar type.  */
template <typename T>
class PenetrationPairFunctor {
 public:
  /* Constructs the functor caching all parameters that do _not_ depend on
   the shape type.

   @param id_A         Identifier of the first geometry passed to operator().
   @param id_B         Identifier of the second geometry passed to operator().
   @param X_WA         Pose of geometry A in the world frame.
   @param X_WB         Pose of geometry B in the world frame */
  PenetrationPairFunctor(const GeometryId& id_A, const GeometryId& id_B,
                         const math::RigidTransform<T>& X_WA,
                         const math::RigidTransform<T>& X_WB)
      : id_A_(id_A), id_B_(id_B), X_WA_(X_WA), X_WB_(X_WB) {}

  /* @name  Overloads in support of sphere-shape computation
   Computes penetration between two shapes. If there is no penetration
   `result` will not be modified.

   Given a sphere A centered at Ao with radius r and another geometry B,
   we want to compute
   1. φ_A,B = the signed penetration between the two objects, which is positive
      for overlapping objects and negative for non-overlapping objects.
   2. Na, Nb = a pair of witness points, Na ∈ ∂A, Nb ∈ ∂B (not necessarily
      unique), |Na-Nb| = |φ_A,B|.

   Define these functions: (available from SignedDistanceToPoint)
     φ_B:ℝ³→ℝ, φ_B(p)  = signed distance to point p from B.
     η_B:ℝ³→ℝ³, η_B(p) = a nearest point to p on the boundary ∂B (not
                         necessarily unique).
     ∇φ_B:ℝ³→ℝ³, ∇φ_B(p) = gradient vector of φ_B with respect to p.
                           It has unit length by construction.
   Algorithm:
   1. φ_A,B = r - φ_B(Ao)
   2. Nb = η_B(Ao)
   3. Na = Ao - r * ∇φ_B(Ao)  */
  //@{

  void operator()(const fcl::Sphered& sphere_A, const fcl::Sphered& sphere_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, sphere_B, result);
  }

  void operator()(const fcl::Sphered& sphere_A, const fcl::Boxd& box_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, box_B, result);
  }

  void operator()(const fcl::Sphered& sphere_A,
                  const fcl::Cylinderd& cylinder_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, cylinder_B, result);
  }

  void operator()(const fcl::Sphered& sphere_A,
                  const fcl::Halfspaced& halfspace_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, halfspace_B, result);
  }

  void operator()(const fcl::Sphered& sphere_A, const fcl::Capsuled& capsule_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, capsule_B, result);
  }

  //@}

 private:
  // Penetration computation between a sphere A and a generic shape B. We use
  // the overloaded call operators above to limit the kinds of queries, and
  // they all call this private template function to minimize code duplication.
  template <typename FclShape>
  void SphereShapePenetration(const fcl::Sphered& sphere_A,
                              const FclShape& shape_B,
                              PenetrationAsPointPair<T>* result) {
    DRAKE_ASSERT(result != nullptr);
    const SignedDistanceToPoint<T> shape_B_to_point_Ao =
        point_distance::DistanceToPoint<T>(id_B_, X_WB_,
                                           X_WA_.translation())(shape_B);
    const T depth = sphere_A.radius - shape_B_to_point_Ao.distance;
    if (ExtractDoubleOrThrow(depth) >= 0) {
      result->id_A = id_A_;
      result->id_B = id_B_;
      // Note that we use a signed penetration here.
      result->depth = depth;
      // p_WCb is the witness point on ∂B measured and expressed in W.
      result->p_WCb = X_WB_ * (shape_B_to_point_Ao.p_GN);
      result->nhat_BA_W = shape_B_to_point_Ao.grad_W;
      // p_WCa is the witness point on ∂A measured and expressed in W.
      result->p_WCa =
          X_WA_.translation() - sphere_A.radius * shape_B_to_point_Ao.grad_W;
    }
  }

  GeometryId id_A_;
  GeometryId id_B_;
  math::RigidTransform<T> X_WA_;
  math::RigidTransform<T> X_WB_;
  PenetrationAsPointPair<T>* result_;
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
 of functions defines the fallback logic for the various scalar types. We assume
 there is no fallback for a scalar type unless one is explicitly provided.  */
//@{

/* For all scalar types T that are not explicitly enumerated, throws an
 exception declaring the unsupported combination of geometry types and scalar
 type.  */
template <typename T>
void CalcDistanceFallback(const fcl::CollisionObjectd& a,
                          const fcl::CollisionObjectd& b,
                          const fcl::CollisionRequestd&,
                          PenetrationAsPointPair<T>* /* pair_data */) {
  // By default, there is no fallback. For every scalar type for which one
  // actually exists, it should be specialized below.
  throw std::logic_error(fmt::format(
      "PenetrationAsPointPair() cannot be evaluated for shapes '{}' and '{}' "
      "for scalar type {}",
      GetGeometryName(a), GetGeometryName(b), NiceTypeName::Get<T>()));
}

/* For the double scalar, computes the signed distance between the two objects.
 */
template <>
void CalcDistanceFallback<double>(const fcl::CollisionObjectd& a,
                                  const fcl::CollisionObjectd& b,
                                  const fcl::CollisionRequestd& request,
                                  PenetrationAsPointPair<double>* pair_data) {
  DRAKE_DEMAND(pair_data != nullptr);
  fcl::CollisionResult<double> result;

  // Perform nearphase collision detection
  collide(&a, &b, request, result);

  if (!result.isCollision()) return;

  // Process the contact points
  // NOTE: This assumes that the request is configured to use a single contact.
  const fcl::Contact<double>& contact = result.getContact(0);

  // Signed distance is negative when penetration depth is positive.
  const double depth = contact.penetration_depth;

  // TODO(SeanCurtis-TRI): Remove this test when FCL issue 375 is fixed.
  // FCL returns osculation as contact but doesn't guarantee a non-zero
  // normal. Drake isn't really in a position to define that normal from the
  // geometry or contact results so, if the geometry is sufficiently close
  // to osculation, we consider the geometries to be non-penetrating.
  if (depth <= std::numeric_limits<double>::epsilon()) return;
  pair_data->depth = depth;

  // By convention, Drake requires the contact normal to point out of B
  // and into A. FCL uses the opposite convention.
  pair_data->nhat_BA_W = -contact.normal;

  // FCL returns a single contact point centered between the two
  // penetrating surfaces. PenetrationAsPointPair expects
  // two, one on the surface of body A (Ac) and one on the surface of body
  // B (Bc). Choose points along the line defined by the contact point and
  // normal, equidistant to the contact point. Recall that depth
  // is non-negative, so depth * nhat_BA_W points out of A and into B.
  pair_data->p_WCa =
      contact.pos - 0.5 * pair_data->depth * pair_data->nhat_BA_W;
  pair_data->p_WCb =
      contact.pos + 0.5 * pair_data->depth * pair_data->nhat_BA_W;

  pair_data->id_A = EncodedData(a).id();
  pair_data->id_B = EncodedData(b).id();
}

//@}

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
void ComputeNarrowPhasePenetration(const fcl::CollisionObjectd& a,
                                   const math::RigidTransform<T>& X_WA,
                                   const fcl::CollisionObjectd& b,
                                   const math::RigidTransform<T>& X_WB,
                                   const fcl::CollisionRequestd& request,
                                   PenetrationAsPointPair<T>* result) {
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
  const math::RigidTransform<T>& X_WS(a_is_sphere ? X_WA : X_WB);
  const math::RigidTransform<T>& X_WO(a_is_sphere ? X_WB : X_WA);
  const auto id_S = EncodedData(s).id();
  const auto id_O = EncodedData(o).id();
  PenetrationPairFunctor<T> calc_penetration_pair(id_S, id_O, X_WS, X_WO);
  const auto& sphere_S = *static_cast<const fcl::Sphered*>(s_geometry);
  switch (o_geometry->getNodeType()) {
    case fcl::GEOM_SPHERE: {
      // TODO(sean.curtis@tri.global) Here we use signed distance computation
      // but we actually only need to perform penetration query. This could
      // incur unnecessary computation. Depending on the data types, we can
      // potentially use collision query for double type, and signed distance
      // query for autodiff scalar.
      const auto& sphere_O = *static_cast<const fcl::Sphered*>(o_geometry);
      calc_penetration_pair(sphere_S, sphere_O, result);
      break;
    }
    case fcl::GEOM_BOX: {
      const auto& box_O = *static_cast<const fcl::Boxd*>(o_geometry);
      calc_penetration_pair(sphere_S, box_O, result);
      break;
    }
    case fcl::GEOM_CYLINDER: {
      const auto& cylinder_O = *static_cast<const fcl::Cylinderd*>(o_geometry);
      calc_penetration_pair(sphere_S, cylinder_O, result);
      break;
    }
    case fcl::GEOM_HALFSPACE: {
      const auto& halfspace_O =
          *static_cast<const fcl::Halfspaced*>(o_geometry);
      calc_penetration_pair(sphere_S, halfspace_O, result);
      break;
    }
    case fcl::GEOM_CAPSULE: {
      const auto& capsule_O = *static_cast<const fcl::Capsuled*>(o_geometry);
      calc_penetration_pair(sphere_S, capsule_O, result);
      break;
    }
    case fcl::GEOM_ELLIPSOID:
    case fcl::GEOM_CONVEX:
      // We don't have a closed form solution for these geometries, so we
      // call FCL.
      CalcDistanceFallback<T>(a, b, request, result);
      break;
    case fcl::GEOM_PLANE:
    case fcl::BV_AABB:
    case fcl::BV_OBB:
    case fcl::BV_RSS:
    case fcl::BV_kIOS:
    case fcl::BV_OBBRSS:
    case fcl::BV_KDOP16:
    case fcl::BV_KDOP18:
    case fcl::BV_KDOP24:
    case fcl::GEOM_CONE:
    case fcl::GEOM_TRIANGLE:
    case fcl::GEOM_OCTREE:
    case fcl::BV_UNKNOWN:
    case fcl::NODE_COUNT:
      // Fcl NodeTypes that are *not* currently supported by Drake.
      DRAKE_UNREACHABLE();
  }
  // If needed, re-order the result for (s,o) back to the result for (a,b).
  if (!a_is_sphere) {
    result->SwapAAndB();
  }
}

// TODO(SeanCurtis-TRI): Replace this clunky mechanism with a new mechanism
// which does this implicitly via ADL and templates.
/* @name   Mechanism for reporting on which scalars and for which shape-pairs
            shape-to-shape queries can be made.

 By default, nothing is supported. For each supported scalar type, a class
 specialization is provided which explicitly enumerates the supported shape
 types.

 @tparam T      The computational scalar type.  */
//@{
template <typename T>
struct ScalarSupport {
  static bool is_supported(fcl::NODE_TYPE, fcl::NODE_TYPE) { return false; }
};

/* Primitive support for double-valued query.  */
template <>
struct ScalarSupport<double> {
  static bool is_supported(fcl::NODE_TYPE node1, fcl::NODE_TYPE node2) {
    // Doubles (via its fallback) can support *almost* anything. Here, we
    // enumerate the *very* limited unsupported cases. We can't meaningfully
    // collide two half spaces, but fcl doesn't have an intelligent response.
    // So, we simply short circuit at this point.
    return !(node1 == fcl::GEOM_HALFSPACE && node2 == fcl::GEOM_HALFSPACE);
  }
};

/* Primitive support for AutoDiff-valued query.  */
template <typename DerType>
struct ScalarSupport<Eigen::AutoDiffScalar<DerType>> {
  static bool is_supported(fcl::NODE_TYPE node1, fcl::NODE_TYPE node2) {
    // Explicitly permit the following pair types (with ordering permutations):
    //  (sphere, sphere)
    //  (sphere, box)
    //  (sphere, cylinder)
    //  (sphere, halfspace)
    //  (sphere, capsule)
    return (node1 == fcl::GEOM_SPHERE &&
            (node2 == fcl::GEOM_SPHERE || node2 == fcl::GEOM_BOX ||
             node2 == fcl::GEOM_HALFSPACE || node2 == fcl::GEOM_CYLINDER ||
             node2 == fcl::GEOM_CAPSULE)) ||
           (node2 == fcl::GEOM_SPHERE &&
            (node1 == fcl::GEOM_BOX || node1 == fcl::GEOM_HALFSPACE ||
             node1 == fcl::GEOM_CYLINDER || node1 == fcl::GEOM_CAPSULE));
  }
};
//@}

template <typename T>
bool Callback(fcl::CollisionObjectd* fcl_object_A_ptr,
              fcl::CollisionObjectd* fcl_object_B_ptr, void* callback_data) {
  auto& data = *static_cast<CallbackData<T>*>(callback_data);

  // Extract the collision filter keys from the fcl collision objects. These
  // keys will also be used to map the fcl collision object back to the Drake
  // GeometryId for colliding geometries.
  EncodedData encoding_A(*fcl_object_A_ptr);
  EncodedData encoding_B(*fcl_object_B_ptr);

  // Guarantee for geometries A and B, we always evaluate the collision between
  // them in a fixed order (e.g., the first geometry gets transformed into the
  // second geometry's frame for evaluation).
  if (encoding_B.id() < encoding_A.id()) {
    std::swap(encoding_A, encoding_B);
    std::swap(fcl_object_A_ptr, fcl_object_B_ptr);
  }

  // NOTE: Although this function *takes* non-const pointers to satisfy the
  // fcl api, it should not exploit the non-constness to modify the collision
  // objects. We ensure this by immediately assigning to a const version and
  // not directly using the provided parameters.
  const GeometryId id_A = encoding_A.id();
  const GeometryId id_B = encoding_B.id();

  const bool can_collide = data.collision_filter.CanCollideWith(
      encoding_A.id(), encoding_B.id());

  // NOTE: Here and below, false is returned regardless of whether collision
  // is detected or not because true tells the broadphase manager to terminate.
  // Since we want *all* collisions, we return false.
  if (!can_collide) return false;

  if (ScalarSupport<T>::is_supported(
          fcl_object_A_ptr->collisionGeometry()->getNodeType(),
          fcl_object_B_ptr->collisionGeometry()->getNodeType())) {
    // Unpack the callback data
    const fcl::CollisionRequestd& request = data.request;

    // This callback only works for a single contact, this confirms a request
    // hasn't been made for more contacts.
    DRAKE_ASSERT(request.num_max_contacts == 1);
    PenetrationAsPointPair<T> penetration;
    ComputeNarrowPhasePenetration(*fcl_object_A_ptr, data.X_WGs.at(id_A),
                                  *fcl_object_B_ptr, data.X_WGs.at(id_B),
                                  data.request, &penetration);
    if (ExtractDoubleOrThrow(penetration.depth) >= 0) {
      data.point_pairs.push_back(std::move(penetration));
    }
  } else {
    throw std::logic_error(fmt::format(
        "Penetration queries between shapes '{}' and '{}' "
        "are not supported for scalar type {}",
        GetGeometryName(*fcl_object_A_ptr), GetGeometryName(*fcl_object_B_ptr),
        NiceTypeName::Get<T>()));
  }

  return false;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &Callback<T>
))

}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
