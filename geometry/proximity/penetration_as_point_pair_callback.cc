#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"

#include <limits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/proximity/collision_backend.h"
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
template <typename T, typename Backend>
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

  void operator()(const typename Backend::Sphere& sphere_A,
                  const typename Backend::Sphere& sphere_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, sphere_B, result);
  }

  void operator()(const typename Backend::Sphere& sphere_A,
                  const typename Backend::Box& box_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, box_B, result);
  }

  void operator()(const typename Backend::Sphere& sphere_A,
                  const typename Backend::Cylinder& cylinder_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, cylinder_B, result);
  }

  void operator()(const typename Backend::Sphere& sphere_A,
                  const typename Backend::Halfspace& halfspace_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, halfspace_B, result);
  }

  void operator()(const typename Backend::Sphere& sphere_A,
                  const typename Backend::Capsule& capsule_B,
                  PenetrationAsPointPair<T>* result) {
    SphereShapePenetration(sphere_A, capsule_B, result);
  }

  //@}

 private:
  // Penetration computation between a sphere A and a generic shape B. We use
  // the overloaded call operators above to limit the kinds of queries, and
  // they all call this private template function to minimize code duplication.
  template <typename BackendShape>
  void SphereShapePenetration(const typename Backend::Sphere& sphere_A,
                              const BackendShape& shape_B,
                              PenetrationAsPointPair<T>* result) {
    DRAKE_ASSERT(result != nullptr);
    const SignedDistanceToPoint<T> shape_B_to_point_Ao =
        point_distance::DistanceToPoint<T, Backend>(
            id_B_, X_WB_, X_WA_.translation())(shape_B);
    const T depth = Backend::radius(sphere_A) - shape_B_to_point_Ao.distance;
    if (ExtractDoubleOrThrow(depth) >= 0) {
      result->id_A = id_A_;
      result->id_B = id_B_;
      // Note that we use a signed penetration here.
      result->depth = depth;
      // p_WCb is the witness point on ∂B measured and expressed in W.
      result->p_WCb = X_WB_ * (shape_B_to_point_Ao.p_GN);
      result->nhat_BA_W = shape_B_to_point_Ao.grad_W;
      // p_WCa is the witness point on ∂A measured and expressed in W.
      result->p_WCa = X_WA_.translation() -
                      Backend::radius(sphere_A) * shape_B_to_point_Ao.grad_W;
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

 The fallback function is a simple interface that takes two collision
 objects (called `a` and `b`), the backend's request parameters, and a
 pointer
 to the SignedDistancePair which will be populated with the results of the
 query.

 Currently, our ability to fallback depends on the scalar type. This family
 of functions defines the fallback logic for the various scalar types. We assume
 there is no fallback for a scalar type unless one is explicitly provided.  */
//@{

/* Computes the signed distance between the two objects for the double scalar.
 For every other scalar type there is no fallback, and the call throws,
 declaring the unsupported combination of geometry types and scalar type.

 This was a function template plus an explicit specialization on double.
 Adding the backend parameter makes that a *partial* specialization, which
 C++ does not allow for function templates, so the dispatch is now an
 `if constexpr`.  */
template <typename T, typename Backend>
void CalcDistanceFallback(const typename Backend::Object& a,
                          const typename Backend::Object& b,
                          const typename Backend::CollisionRequest& request,
                          PenetrationAsPointPair<T>* pair_data) {
  if constexpr (!std::is_same_v<T, double>) {
    unused(request, pair_data);
    throw std::logic_error(fmt::format(
        "PenetrationAsPointPair() cannot be evaluated for shapes '{}' and "
        "'{}' for scalar type {}. See the documentation for "
        "QueryObject::ComputePointPairPenetration() for the full status of "
        "supported geometries.",
        GetGeometryName<Backend>(a), GetGeometryName<Backend>(b),
        NiceTypeName::Get<T>()));
  } else {
    DRAKE_DEMAND(pair_data != nullptr);
    typename Backend::CollisionResult result;

    // Perform nearphase collision detection
    Backend::Collide(&a, &b, request, &result);

    if (!result.isCollision()) return;

    // Process the contact points
    // NOTE: This assumes that the request is configured to use a single
    // contact.
    const typename Backend::Contact& contact = result.getContact(0);

    // Positive when the geometries overlap; the backends disagree on the sign
    // convention, so the policy normalizes it (see fcl_backend.h, D3).
    const double depth = Backend::ContactDepth(contact);

    // TODO(SeanCurtis-TRI): Remove this test when FCL issue 375 is fixed.
    // The solver returns osculation as contact but doesn't guarantee a non-zero
    // normal. Drake isn't really in a position to define that normal from the
    // geometry or contact results so, if the geometry is sufficiently close
    // to osculation, we consider the geometries to be non-penetrating.
    if (depth <= std::numeric_limits<double>::epsilon()) return;
    pair_data->depth = depth;

    // By convention, Drake requires the contact normal to point out of B
    // and into A. The backends use the opposite convention.
    pair_data->nhat_BA_W = -contact.normal;

    // The backend returns a single contact point centered between the two
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
template <typename T, typename Backend>
void ComputeNarrowPhasePenetration(
    const typename Backend::Object& a, const math::RigidTransform<T>& X_WA,
    const typename Backend::Object& b, const math::RigidTransform<T>& X_WB,
    const typename Backend::CollisionRequest& request,
    PenetrationAsPointPair<T>* result) {
  DRAKE_DEMAND(result != nullptr);
  const typename Backend::Geometry* a_geometry = a.collisionGeometry().get();
  const typename Backend::Geometry* b_geometry = b.collisionGeometry().get();
  const ShapeKind a_kind = Backend::Classify(a_geometry->getNodeType());
  const ShapeKind b_kind = Backend::Classify(b_geometry->getNodeType());
  const bool a_is_sphere = a_kind == ShapeKind::kSphere;
  const bool b_is_sphere = b_kind == ShapeKind::kSphere;
  const bool no_sphere = !(a_is_sphere || b_is_sphere);
  if (no_sphere) {
    CalcDistanceFallback<T, Backend>(a, b, request, result);
    return;
  }
  DRAKE_ASSERT(a_is_sphere || b_is_sphere);
  // We write `s` for the sphere object and `o` for the other object. We
  // assign either (a,b) or (b,a) to (s,o) depending on whether `a` is a
  // sphere or not. Therefore, we only need the helper DistancePairGeometry
  // that takes (sphere, other) but not (other, sphere).  This scheme helps us
  // keep the code compact; however, we might have to re-order the result
  // afterwards.
  const typename Backend::Object& s = a_is_sphere ? a : b;
  const typename Backend::Object& o = a_is_sphere ? b : a;
  const typename Backend::Geometry* s_geometry = s.collisionGeometry().get();
  const typename Backend::Geometry* o_geometry = o.collisionGeometry().get();
  const math::RigidTransform<T>& X_WS(a_is_sphere ? X_WA : X_WB);
  const math::RigidTransform<T>& X_WO(a_is_sphere ? X_WB : X_WA);
  const auto id_S = EncodedData(s).id();
  const auto id_O = EncodedData(o).id();
  PenetrationPairFunctor<T, Backend> calc_penetration_pair(id_S, id_O, X_WS,
                                                           X_WO);
  const auto& sphere_S =
      *static_cast<const typename Backend::Sphere*>(s_geometry);
  switch (a_is_sphere ? b_kind : a_kind) {
    case ShapeKind::kSphere: {
      // TODO(sean.curtis@tri.global) Here we use signed distance computation
      // but we actually only need to perform penetration query. This could
      // incur unnecessary computation. Depending on the data types, we can
      // potentially use collision query for double type, and signed distance
      // query for autodiff scalar.
      const auto& sphere_O =
          *static_cast<const typename Backend::Sphere*>(o_geometry);
      calc_penetration_pair(sphere_S, sphere_O, result);
      break;
    }
    case ShapeKind::kBox: {
      const auto& box_O =
          *static_cast<const typename Backend::Box*>(o_geometry);
      calc_penetration_pair(sphere_S, box_O, result);
      break;
    }
    case ShapeKind::kCylinder: {
      const auto& cylinder_O =
          *static_cast<const typename Backend::Cylinder*>(o_geometry);
      calc_penetration_pair(sphere_S, cylinder_O, result);
      break;
    }
    case ShapeKind::kHalfspace: {
      const auto& halfspace_O =
          *static_cast<const typename Backend::Halfspace*>(o_geometry);
      calc_penetration_pair(sphere_S, halfspace_O, result);
      break;
    }
    case ShapeKind::kCapsule: {
      const auto& capsule_O =
          *static_cast<const typename Backend::Capsule*>(o_geometry);
      calc_penetration_pair(sphere_S, capsule_O, result);
      break;
    }
    case ShapeKind::kEllipsoid:
    case ShapeKind::kConvex:
      // We don't have a closed form solution for these geometries, so we
      // defer to the backend's solver.
      CalcDistanceFallback<T, Backend>(a, b, request, result);
      break;
    case ShapeKind::kUnsupported:
      // Shape kinds that Drake never registers.
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
  static bool is_supported(ShapeKind, ShapeKind) { return false; }
};

/* Primitive support for double-valued query.  */
template <>
struct ScalarSupport<double> {
  static bool is_supported(ShapeKind kind1, ShapeKind kind2) {
    // Doubles (via its fallback) can support *almost* anything. Here, we
    // enumerate the *very* limited unsupported cases. We can't meaningfully
    // collide two half spaces, but the backends don't have an intelligent
    // response. So, we simply short circuit at this point.
    return !(kind1 == ShapeKind::kHalfspace && kind2 == ShapeKind::kHalfspace);
  }
};

/* Primitive support for AutoDiff-valued query.  */
template <>
struct ScalarSupport<AutoDiffXd> {
  static bool is_supported(ShapeKind kind1, ShapeKind kind2) {
    // Explicitly permit the following pair types (with ordering permutations):
    //  (sphere, sphere)
    //  (sphere, box)
    //  (sphere, cylinder)
    //  (sphere, halfspace)
    //  (sphere, capsule)
    return (kind1 == ShapeKind::kSphere &&
            (kind2 == ShapeKind::kSphere || kind2 == ShapeKind::kBox ||
             kind2 == ShapeKind::kHalfspace || kind2 == ShapeKind::kCylinder ||
             kind2 == ShapeKind::kCapsule)) ||
           (kind2 == ShapeKind::kSphere &&
            (kind1 == ShapeKind::kBox || kind1 == ShapeKind::kHalfspace ||
             kind1 == ShapeKind::kCylinder || kind1 == ShapeKind::kCapsule));
  }
};
//@}

template <typename T, typename Backend>
bool Callback(typename Backend::Object* object_A_ptr,
              typename Backend::Object* object_B_ptr, void* callback_data) {
  auto& data = *static_cast<CallbackData<T, Backend>*>(callback_data);

  // Extract the collision filter keys from the collision objects. These
  // keys will also be used to map the collision object back to the Drake
  // GeometryId for colliding geometries.
  EncodedData encoding_A(*object_A_ptr);
  EncodedData encoding_B(*object_B_ptr);

  // Guarantee for geometries A and B, we always evaluate the collision between
  // them in a fixed order (e.g., the first geometry gets transformed into the
  // second geometry's frame for evaluation).
  if (encoding_B.id() < encoding_A.id()) {
    std::swap(encoding_A, encoding_B);
    std::swap(object_A_ptr, object_B_ptr);
  }

  const bool can_collide =
      data.collision_filter.CanCollideWith(encoding_A.id(), encoding_B.id());

  // NOTE: Here and below, false is returned regardless of whether collision
  // is detected or not because true tells the broadphase manager to terminate.
  // Since we want *all* collisions, we return false.
  if (!can_collide) return false;

  auto result =
      MaybeMakePointPair<T, Backend>(object_A_ptr, object_B_ptr, data);
  if (result.has_value()) {
    data.point_pairs.push_back(std::move(*result));
  }

  return false;
}

template <typename T, typename Backend>
std::optional<PenetrationAsPointPair<T>> MaybeMakePointPair(
    typename Backend::Object* object_A_ptr,
    typename Backend::Object* object_B_ptr,
    const CallbackData<T, Backend>& data) {
  // Extract the collision filter keys from the collision objects. These
  // keys will also be used to map the collision object back to the Drake
  // GeometryId for colliding geometries.
  EncodedData encoding_A(*object_A_ptr);
  EncodedData encoding_B(*object_B_ptr);

  // Guarantee for geometries A and B, we always evaluate the collision between
  // them in a fixed order (e.g., the first geometry gets transformed into the
  // second geometry's frame for evaluation).
  if (encoding_B.id() < encoding_A.id()) {
    std::swap(encoding_A, encoding_B);
    std::swap(object_A_ptr, object_B_ptr);
  }

  // NOTE: Although this function *takes* non-const pointers to satisfy the
  // backend api, it should not exploit the non-constness to modify the
  // collision objects. We ensure this by immediately assigning to a const
  // version and not directly using the provided parameters.
  const GeometryId id_A = encoding_A.id();
  const GeometryId id_B = encoding_B.id();

  if (ScalarSupport<T>::is_supported(
          Backend::Classify(object_A_ptr->collisionGeometry()->getNodeType()),
          Backend::Classify(
              object_B_ptr->collisionGeometry()->getNodeType()))) {
    // Unpack the callback data
    const typename Backend::CollisionRequest& request = data.request;

    // This callback only works for a single contact, this confirms a request
    // hasn't been made for more contacts.
    DRAKE_ASSERT(request.num_max_contacts == 1);
    PenetrationAsPointPair<T> penetration;
    ComputeNarrowPhasePenetration<T, Backend>(
        *object_A_ptr, data.X_WGs.at(id_A), *object_B_ptr, data.X_WGs.at(id_B),
        data.request, &penetration);
    if (ExtractDoubleOrThrow(penetration.depth) >= 0) {
      return penetration;
    }
  } else {
    throw std::logic_error(fmt::format(
        "Penetration queries between shapes '{}' and '{}' "
        "are not supported for scalar type {}. See the documentation for "
        "QueryObject::ComputePointPairPenetration() for the full status of "
        "supported geometries.",
        GetGeometryName<Backend>(*object_A_ptr),
        GetGeometryName<Backend>(*object_B_ptr), NiceTypeName::Get<T>()));
  }
  return {};
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_SCALARS_AND_BACKENDS(
    (&Callback<T, Backend>, &MaybeMakePointPair<T, Backend>));

}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
