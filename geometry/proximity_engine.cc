#include "drake/geometry/proximity_engine.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

#include <fcl/common/types.h>
#include <fcl/fcl.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/convex.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/distance_request.h>
#include <spruce.hh>
#include <tiny_obj_loader.h>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_variant.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_vectors_have_intersection.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace internal {

template <int Rows> using Vectord = Vector<double, Rows>;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Isometry3d;
using std::make_shared;
using std::make_unique;
using std::move;
using std::shared_ptr;
using std::unique_ptr;
using math::RigidTransformd;
using math::RotationMatrixd;

namespace {

// TODO(SeanCurtis-TRI): Swap all Isometry3 for RigidTransforms.

// Utilities/functions for working with the encoding of collision object index
// and mobility type in the fcl::CollisionObject user data field. The encoded
// data produces a value that is guaranteed to be unique across all geometries.
// The guarantee arises from the fact that it's an engine index combined with
// a bit reporting anchored vs dynamic geometry; every geometry is uniquely
// identified by its engine index and its dynamic/anchored state.
//
// The encoding packs a geometry index value with a bit indicating if the index
// refers to dynamic or anchored geometry (they are stored in separate vectors
// which means indices can be repeated). The highest-order bit indicates
// dynamic (1) or anchored (0). The remaining lower bits store the index. The
// data is stored in a pointer-sized integer. This integer is, in turn, stored
// directly into fcl::CollisionObject's void* user data member.
class EncodedData {
 public:
  // Constructor which defines its encoded data by combining a geometry's index
  // and its dynamic/anchored characterization.
  EncodedData(int index, bool is_dynamic)
      : data_(static_cast<uintptr_t>(index)) {
    if (is_dynamic) set_dynamic();
    // NOTE: data is encoded as anchored by default. So, an action only needs to
    // be taken in the dynamic case.
  }

  // Constructor which defines its encoded data value by extracting it from the
  // given Fcl collision object.
  explicit EncodedData(const fcl::CollisionObject<double>& fcl_object)
      : data_(reinterpret_cast<uintptr_t>(fcl_object.getUserData())) {}

  // Factory method for creating EncodedData from the given `index` for a
  // dynamic geometry.
  static EncodedData encode_dynamic(GeometryIndex index) {
    return EncodedData(index, true);
  }

  // Factory method for creating EncodedData from the given `index` for an
  // anchored geometry.
  static EncodedData encode_anchored(GeometryIndex index) {
    return EncodedData(index, false);
  }

  // Sets the encoded data to be dynamic.
  void set_dynamic() { data_ |= kIsDynamicMask; }

  // Sets the encoded data to be anchored.
  void set_anchored() { data_ &= ~kIsDynamicMask; }

  // Stores the encoded data in the collision object's user data.
  void store_in(fcl::CollisionObject<double>* object) {
    object->setUserData(reinterpret_cast<void*>(data_));
  }

  // Reports true if the given fcl_object's user data is encoded as dynamic.
  // False if anchored.
  bool is_dynamic() const { return (data_ & kIsDynamicMask) != 0; }

  // Reports the stored index.
  GeometryIndex index() const {
    return static_cast<GeometryIndex>(data_ & ~kIsDynamicMask);
  }

  // Given an fcl object and maps from index to id of both dynamic and anchored
  // geometry, returns the geometry id for the given fcl object.
  GeometryId id(const std::vector<GeometryId>& geometry_map) const {
    return geometry_map[index()];
  }

  // Reports the encoded data.
  uintptr_t encoded_data() const { return data_; }

  // Fcl Collision objects allow for user data. We're storing *two* pieces of
  // information: the engine index of the corresponding geometry and the
  // _mobility_ type (i.e., dynamic vs anchored). We do this by mangling bits.
  // Because user data is a void*, we can read it like a uintptr_t. The highest
  // bit will represent dynamic (1) or anchored (0).The remaining lower order
  // bits store the index.

  // Note: This sets a mask to be 1000...0 based on the size of a pointer.
  // C++ guarantees that uintptr_t can hold a pointer and cast to a pointer,
  // but it doesn't guarantee it's the *same size* as a pointer. So, we set
  // the mask value based on pointer size.
  static const uintptr_t kIsDynamicMask = uintptr_t{1}
                                          << (sizeof(void*) * 8 - 1);

 private:
  // The encoded data - index and mobility type.
  // We're using an unsigned value here because:
  //   - Bitmasking games are typically more intuitive with unsigned values
  //     (think of bit shifting as an example here).
  //   - This unsigned integer doesn't bleed out into the API at all.
  uintptr_t data_{};
};

// A simple class for providing collision filtering functionality similar to
// that found in RigidBodyTree but made compatible with fcl. The majority of
// this code is lifted verbatim from drake/multibody/collision/element.{h, cc}.
//
// Note: I'm using uintptr_t instead of EncodedData directly to avoid having
// to hash EncodedData.
// TODO(SeanCurtis-TRI): Replace this "legacy" mechanism with the
// new-and-improved alternative when it is ready.
class CollisionFilterLegacy {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterLegacy)

  CollisionFilterLegacy() = default;

  void AddGeometry(uintptr_t id) {
    collision_cliques_.insert({id, std::vector<int>()});
  }

  // NOTE: This assumes that `id_A` and `id_B` will *never* both be anchored
  // geometries. The structure of the collision query logic precludes that
  // possibility; dynamic is collided against dynamic and dynamic is collided
  // against anchored, but anchored is never collided against anchored.
  bool CanCollideWith(uintptr_t id_A, uintptr_t id_B) const {
    // These ids should all be registered with the filter machinery.
    DRAKE_ASSERT(collision_cliques_.count(id_A) == 1);
    DRAKE_ASSERT(collision_cliques_.count(id_B) == 1);

    bool excluded = id_A == id_B ||
                    SortedVectorsHaveIntersection(collision_cliques_.at(id_A),
                                                  collision_cliques_.at(id_B));

    return !excluded;
  }

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

  // This method is not thread safe.
  int next_clique_id() {
    int clique = next_available_clique_++;
    if (clique < 0) {
      throw std::logic_error(
          "SceneGraph has run out of cliques (more than two billion served)");
    }
    return clique;
  }

  // Test support; to detect when cliques are generated.
  int peek_next_clique() const { return next_available_clique_; }

 private:
  // A map between the EncodedData::encoded_data() value for a geometry and
  // its set of cliques.
  std::unordered_map<uintptr_t, std::vector<int>> collision_cliques_;
  int next_available_clique_{0};
};

// Struct for use in DistanceCallback(). Contains the distance request
// and accumulates result in a drake::geometry::SignedDistancePair vector.
struct DistanceData {
  DistanceData(const std::vector<GeometryId>* geometry_map_in,
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

// Struct for use in DistanceFromPointCallback(). Contains the distance request
// and accumulates result in a vector of drake::geometry::SignedDistanceToPoint.
struct DistanceFromPointCallbackData {
  DistanceFromPointCallbackData(
      fcl::CollisionObjectd* query_in,
      const std::vector<GeometryId>* geometry_map_in,
      const double threshold_in,
      std::vector<SignedDistanceToPoint<double>>* distances_in)
      : query_point(query_in),
        geometry_map(*geometry_map_in),
        threshold(threshold_in),
        distances(distances_in) {}

  // The query point is represented as a zero-radius sphere.
  fcl::CollisionObjectd* query_point;

  // Maps so the distance call back can map from geometry index to geometry id.
  const std::vector<GeometryId>& geometry_map;

  // We ignore any object beyond this distance.
  const double threshold;

  std::vector<SignedDistanceToPoint<double>>* distances;
};

// Struct for use in SingleCollisionCallback(). Contains the collision request
// and accumulates results in a drake::multibody::collision::PointPair vector.
struct CollisionData {
  CollisionData(const std::vector<GeometryId>* geometry_map_in,
                const CollisionFilterLegacy* collision_filter_in)
      : geometry_map(*geometry_map_in),
        collision_filter(*collision_filter_in) {}
  // Maps so the penetration call back can map from engine index to geometry id.
  const std::vector<GeometryId>& geometry_map;
  const CollisionFilterLegacy& collision_filter;

  // Collision request
  fcl::CollisionRequestd request;

  // Vector of distance results
  std::vector<PenetrationAsPointPair<double>>* contacts{};
};

// An internal functor to support DistanceFromPointCallback() and
// ComputeNarrowPhaseDistance(). It computes the signed distance to a query
// point from a supported geometry. Each overload to the call operator
// reports the signed distance (encoded as SignedDistanceToPoint) between the
// functor's stored query point and the given geometry argument.
class DistanceToPoint {
 public:
  // Constructs the functor DistanceToPoint.
  // @param id    the id of the geometry G,
  // @param X_WG  pose of the geometry G in World frame,
  // @param p_WQ  position of the query point Q in World frame.
  // @note Some parts of the geometry (id, pose) are initialized in this
  // constructor, and the remaining part of the geometry (shape) is a parameter
  // to the call operator() below.
  DistanceToPoint(const GeometryId id,
                  const RigidTransformd& X_WG,
                  const Vector3d& p_WQ) :
      geometry_id_(id), X_WG_(X_WG), p_WQ_(p_WQ) {}

  // Overload for Sphere.
  SignedDistanceToPoint<double> operator()(const fcl::Sphered& sphere);

  // Overload for Box.
  SignedDistanceToPoint<double> operator()(const fcl::Boxd& box);

  // Overload for Cylinder.
  SignedDistanceToPoint<double> operator()(const fcl::Cylinderd& cylinder);

 private:
  // Calculates a tolerance relative to a given `size` parameter with a lower
  // bound of 1e-14 meter. If the `size` parameter is larger than 1 meter, we
  // use the relative tolerance of 1e-14 times the `size`.  If the `size` is
  // smaller than 1 meter, we use the absolute tolerance 1e-14 meter. The
  // 1e-14-meter lower bound helps us handle possible round off errors arising
  // from applying a pose X_WG to a geometry G. Given a query point Q exactly
  // on the boundary ∂G, if we apply X_WG to both Q and G, the point Q is likely
  // to deviate from ∂G more than the machine epsilon, which is around 2e-16.
  static double RelativeTolerance(double size) {
    return 1e-14 * std::max(1., size);
  }
  // This version of Sign(x) returns +1.0 for zero.
  static double Sign(double x) { return (x < 0.0) ? -1. : 1.; }
  // Picks the axis i whose coordinate p(i) is closest to the boundary value
  // ±bounds(i). If there are ties, we prioritize according to an arbitrary
  // ordering: +x,-x,+y,-y,+z,-z.
  // @tparam V The vector type: Vector2d or Vector3d.
  template <typename V>
  static int ExtremalAxis(const V& p, const V& bounds) {
    DRAKE_DEMAND(V::SizeAtCompileTime == 2 ||
                 V::SizeAtCompileTime == 3);
    double min_dist = std::numeric_limits<double>::infinity();
    int axis = -1;
    for (int i = 0; i < V::SizeAtCompileTime; ++i) {
      for (const auto bound : {bounds(i), -bounds(i)}) {
        const double dist = std::abs(bound - p(i));
        if (dist < min_dist) {
          min_dist = dist;
          axis = i;
        }
      }
    }
    return axis;
  }
  // Calculates the signed distance from a query point Q to a box, G. The box
  // can be either two-dimensional or three-dimensional.  We use the 3D
  // version directly for the signed distance from Box and use the 2D version
  // indirectly for the signed distance from Cylinder.
  // @tparam dim     The dimension, usually 2 or 3.
  // @param h        The vector from the center of the box to the positive
  //                 corner of the box.  In 2D, it consists of half the width
  //                 and half the length of G. In 3D, it consists of half the
  //                 width, half the depth, and half the height of G.
  //                 Mathematically the box is the Cartesian product
  //                 [-h(0),h(0)]x[-h(1),h(1)] in 2D and
  //                 [-h(0),h(0)]x[-h(1),h(1)]x[-h(2),h(2)] in 3D.
  // @param p_GQ_G   The position of the query point Q in G's frame.
  // @retval {p_GN_G, distance, grad_G}   The tuple of the position of
  //                 the nearest point N in G's frame, the signed distance,
  //                 and the gradient vector in G's frame.
  template <int dim>
  std::tuple<Vectord<dim>, double, Vectord<dim>>
  BoxDistanceXd(const Vectord<dim>& h, const Vectord<dim>& p_GQ_G);

  // The id of the geometry G.
  const GeometryId geometry_id_;
  // The pose of the geometry G in World frame.
  const RigidTransformd X_WG_;
  // The position of the query point Q in World frame.
  const Vector3d p_WQ_;
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
                       const RigidTransformd& X_WA, const RigidTransformd& X_WB,
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
  void operator()(const fcl::Sphered* sphere_A, const fcl::Sphered* sphere_B);
  void operator()(const fcl::Sphered* sphere_A, const fcl::Boxd* box_B);
  void operator()(const fcl::Sphered* sphere_A,
                  const fcl::Cylinderd* cylinder_B);

 private:
  // Distance computation between a sphere A and a generic shape B. We use
  // the overloaded call operators above to limit the kinds of queries, and
  // they all call this private template function to minimize code duplication.
  template <typename FclShape>
  void SphereShapeDistance(const fcl::Sphered* sphere_A,
                           const FclShape* shape_B);
  // Performs step 3. Na = Ao - r * ∇φ_B(Ao).
  // @param radius_A the radius of the sphere A.
  // @param X_WA the pose of the sphere A.
  // @param gradB_W the gradient vector ∇φ_B(Ao).
  // @retval p_WNa the witness point Na ∈ ∂A expressed in World frame.
  Vector3d WitnessPointOnSphere(
      const double radius_A, const RigidTransformd& X_WA,
      const Vector3d& gradB_W) const;

  GeometryId id_A_;
  GeometryId id_B_;
  RigidTransformd X_WA_;
  RigidTransformd X_WB_;
  fcl::DistanceResultd* result_;
};

Vector3d DistancePairGeometry::WitnessPointOnSphere(
    const double radius_A, const RigidTransformd& X_WA,
    const Vector3d& gradB_W) const {
  // Notation:
  // gradB_W = ∇φ_B(Ao) expressed in World frame.
  // gradB_A = ∇φ_B(Ao) expressed in A's frame.
  const RotationMatrixd& R_WA = X_WA.rotation();
  const RotationMatrixd R_AW = R_WA.transpose();
  const Vector3d gradB_A = R_AW * gradB_W;
  // By construction gradB_A has unit length.
  const Vector3d p_ANa = -radius_A * gradB_A;
  const Vector3d p_WNa = X_WA * p_ANa;
  return p_WNa;
}

// Signed distance between a sphere and another shape.
template <typename FclShape>
void DistancePairGeometry::SphereShapeDistance(const fcl::Sphered* sphere_A,
                                               const FclShape* shape_B) {
  const SignedDistanceToPoint<double> shape_B_to_point_Ao =
      DistanceToPoint(id_B_, X_WB_, X_WA_.translation())(*shape_B);
  const double distance = shape_B_to_point_Ao.distance - sphere_A->radius;
  // Nb is the witness point on ∂B.
  const Vector3d& p_BNb = shape_B_to_point_Ao.p_GN;
  const Vector3d p_WNb = X_WB_ * p_BNb;
  const Vector3d& gradB_W = shape_B_to_point_Ao.grad_W;
  // Na is the witness point on ∂A.
  const Vector3d p_WNa = WitnessPointOnSphere(sphere_A->radius, X_WA_, gradB_W);
  // fcl::DistanceResult expects -1 for primitive shapes.
  result_->update(distance, sphere_A, shape_B, -1, -1, p_WNa, p_WNb);
}

// Signed distance between two spheres.
void DistancePairGeometry::operator()(const fcl::Sphered* sphere_A,
                                      const fcl::Sphered* sphere_B) {
  SphereShapeDistance(sphere_A, sphere_B);
}

// Signed distance between a sphere and a box.
void DistancePairGeometry::operator()(const fcl::Sphered* sphere_A,
                                      const fcl::Boxd* box_B) {
  SphereShapeDistance(sphere_A, box_B);
}

// Signed distance between a sphere and a cylinder.
void DistancePairGeometry::operator()(const fcl::Sphered* sphere_A,
                                      const fcl::Cylinderd* cylinder_B) {
  SphereShapeDistance(sphere_A, cylinder_B);
}

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
  const RigidTransformd X_WS(s->getTransform());
  const RigidTransformd X_WO(o->getTransform());
  const auto id_S = EncodedData(*s).id(geometry_map);
  const auto id_O = EncodedData(*o).id(geometry_map);
  DistancePairGeometry distance_pair(id_S, id_O, X_WS, X_WO, result);
  auto sphere_S = static_cast<const fcl::Sphered*>(s_geometry);
  switch (o_geometry->getNodeType()) {
    case fcl::GEOM_SPHERE: {
      auto sphere_O = static_cast<const fcl::Sphered*>(o_geometry);
      distance_pair(sphere_S, sphere_O);
      break;
    }
    case fcl::GEOM_BOX: {
      auto box_O = static_cast<const fcl::Boxd*>(o_geometry);
      distance_pair(sphere_S, box_O);
      break;
    }
    case fcl::GEOM_CYLINDER: {
      auto cylinder_O = static_cast<const fcl::Cylinderd*>(o_geometry);
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
bool DistanceCallback(fcl::CollisionObjectd* fcl_object_A_ptr,
                      fcl::CollisionObjectd* fcl_object_B_ptr,
                      // NOLINTNEXTLINE
                      void* callback_data, double&) {
  auto& distance_data = *static_cast<DistanceData*>(callback_data);
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
    const Vector3d& p_WCa = result.nearest_points[0];
    const Vector3d& p_WCb = result.nearest_points[1];
    const Vector3d p_ACa = fcl_object_A.getTransform().inverse() * p_WCa;
    const Vector3d p_BCb = fcl_object_B.getTransform().inverse() * p_WCb;
    // TODO(DamrongGuoy): For sphere-{sphere,box,cylinder} we will start
    //  working on the right nhat when min_distance is 0 or almost 0 after
    //  PR #10813 lands to avoid conflicts with this PR #10823. For now,
    //  we simply return NaN in nhat when min_distance is 0 or almost 0.
    const Vector3d nhat_BA_W =
        (std::abs(result.min_distance) < std::numeric_limits<double>::epsilon())
            ? Vector3d(std::numeric_limits<double>::quiet_NaN(),
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

// Overload for Sphere.
SignedDistanceToPoint<double> DistanceToPoint::operator()(
    const fcl::Sphered& sphere) {
  // TODO(DamrongGuoy): Move most code of this function into FCL.
  const double radius = sphere.radius;
  const Vector3d p_GQ_G = X_WG_.inverse() * p_WQ_;
  const double dist_GQ = p_GQ_G.norm();
  const double distance = dist_GQ - radius;

  // The gradient is always in the direction from the center of the sphere to
  // the query point Q, regardless of whether the point Q is outside or inside
  // the sphere G.  The gradient is undefined if the query point Q is at the
  // center of the sphere G.
  //
  // If the query point Q is near the center of the sphere G within a
  // tolerance, we arbitrarily set the gradient vector as documented in
  // query_object.h (QueryObject::ComputeSignedDistanceToPoint).
  const double tolerance = RelativeTolerance(radius);
  // Unit vector in x-direction of G's frame.
  const Vector3d Gx(1., 0., 0.);
  // Gradient vector expressed in G's frame.
  Vector3d grad_G = (dist_GQ > tolerance) ? p_GQ_G / dist_GQ : Gx;

  // Position vector of the nearest point N on G's surface from the query
  // point Q, expressed in G's frame.
  const Vector3d p_GN_G = radius * grad_G;
  // Gradient vector expressed in World frame.
  const Vector3d grad_W = X_WG_.rotation() * grad_G;

  return SignedDistanceToPoint<double>{geometry_id_, p_GN_G, distance,
                                       grad_W};
}

template <int dim>
std::tuple<Vectord<dim>, double, Vectord<dim>>
DistanceToPoint::BoxDistanceXd(
    const Vectord<dim>& h, const Vectord<dim>& p_GQ_G) {
  // TODO(DamrongGuoy): Move into FCL.
  // We need to classify Q as inside, outside, or on the boundary of B,
  // where 'on the boundary' means within a tolerance of the boundary.
  // This helper function takes the i-th coordinate `coord` of p_GQ_G.
  // It returns the clamped value of `coord` within ±h(i). It also returns
  // an enum to indicate whether the i-th coordinate is inside the interval
  // (-h(i),+h(i)), or within a tolerance of the bounded value ±h(i), or
  // outside the interval.
  enum class Location {kInside, kBoundary, kOutside};
  auto clamp = [&h](const int i, const double coord,
                    Location* location) -> double {
    const double tolerance = RelativeTolerance(h(i));
    if (std::abs(coord) > h(i) + tolerance) {
      *location = Location::kOutside;
      return Sign(coord) * h(i);
    } else if (std::abs(coord) >= h(i) - tolerance) {
      *location = Location::kBoundary;
      return Sign(coord) * h(i);
    } else {
      *location = Location::kInside;
      return coord;
    }
  };

  // Declare a type of vector of Location parallel to the vector of
  // coordinates of the position p_GQ_G of Q.
  typedef Vector<Location, dim> VectorLoc;

  // The clamp point C has coordinates of Q clamped onto the box.
  // Note that:
  // 1. C is the nearest point to Q on ∂B if Q is classified as outside B.
  // 2. C is at the same position as Q if Q is classified as inside B.
  // 3. C is exactly on ∂B if Q is within a tolerance from ∂B.
  Vectord<dim> p_GC_G;
  VectorLoc location;
  for (int i = 0; i < p_GC_G.size(); ++i)
    p_GC_G(i) = clamp(i, p_GQ_G(i), &location(i));

  // Initialize the position of the nearest point N on ∂B as that of C.
  Vectord<dim> p_GN_G = p_GC_G;
  double distance;
  Vectord<dim> grad_G = Vectord<dim>::Zero();

  if ((location.array() == Location::kOutside).any()) {
    // Q is outside the box.
    Vectord<dim> p_NQ_G = p_GQ_G - p_GN_G;
    distance = p_NQ_G.norm();
    DRAKE_DEMAND(distance != 0.);
    grad_G = p_NQ_G / distance;
  } else if ((location.array() == Location::kBoundary).any()) {
    // Q is on the boundary of the box.
    distance = 0.0;
    // In any number of dimension, a point on the boundary of a box must have
    // location(i) == kBoundary for one or more values of i. The gradient is
    // defined as the average outward normal direction for each axis where
    // location(axis) == kBoundary. This provides a unique value even when
    // no such value exists (e.g., at a box corner).
    for (int i = 0; i < dim; ++i) {
      if (location(i) == Location::kBoundary)
        grad_G(i) = Sign(p_GC_G(i));
    }
    grad_G.normalize();
  } else {
    // Q is inside the box.
    // In 2D (3D), the nearest point N is the axis-aligned projection of Q
    // onto one of the edge (faces) of the box.  The gradient vector is along
    // that direction.
    int axis = ExtremalAxis(p_GQ_G, h);
    double sign = Sign(p_GQ_G(axis));
    p_GN_G(axis) = sign * h(axis);
    grad_G(axis) = sign;
    distance = std::abs(p_GQ_G(axis)) - h(axis);
  }
  return std::make_tuple(p_GN_G, distance, grad_G);
}

// Overload for Box.
SignedDistanceToPoint<double> DistanceToPoint::operator()(
    const fcl::Boxd& box) {
  // TODO(DamrongGuoy): Move most code of this function into FCL.
  // Express the given query point Q in the frame of the box geometry G.
  const Vector3d p_GQ_G = X_WG_.inverse() * p_WQ_;
  // The box G is an axis-aligned box [-h(0),h(0)]x[-h(1),h(1)]x[-h(2),h(2)]
  // centered at the origin, where h(i) is half the size of the box in the
  // i-th coordinate.
  const Vector3d h = box.side / 2.0;
  Vector3d p_GN_G;
  double distance;
  Vector3d grad_G;
  std::tie(p_GN_G, distance, grad_G) = BoxDistanceXd(h, p_GQ_G);
  // Use R_WG for vectors. Use X_WG for points.
  const auto& R_WG = X_WG_.rotation();
  Vector3d grad_W = R_WG * grad_G;
  return SignedDistanceToPoint<double>{geometry_id_, p_GN_G, distance, grad_W};
}

// Overload for Cylinder.
SignedDistanceToPoint<double> DistanceToPoint::operator()(
    const fcl::Cylinderd& cylinder) {
  // Overview. First, we map the problem from the 3D cylinder G (in frame G) to
  // the 2D box B (in frame B) of G's cross section through the plane
  // containing the query point Q and the center line of G. Then, we call the
  // function BoxDistanceXd() to get the signed distance, the nearest point N,
  // and the gradient vector expressed in B's frame. Finally, we map the
  // answers back into G's frame.
  // TODO(DamrongGuoy): Move into FCL.
  // Express the query point Q in the cylinder geometry G's frame.
  const Vector3d p_GQ = X_WG_.inverse() * p_WQ_;
  // The 2D cross section B is the box [-h(0),h(0)]x[-h(1),h(1)], where
  // h(0) and h(1) are the radius and the half length of the cylinder.
  const Vector2d h(cylinder.radius, cylinder.lz / 2.0);
  // Transform coordinates between (x,y,z) in G's frame and (r,z) in B's frame.
  // The basis vector `Bz` is aligned with `Gz`, and `r` is the distance to
  // the z-axis (i.e., √(x² + y²)), and z transfers unchanged.
  // The coordinate r is in the radial direction of G in the plane through Q
  // and z-axis of G.
  //
  //  3D cylinder G (x,y,z)     2D box B (r,z)          .
  //       z                         z                  .
  //       |      y                  |                  .
  //       |     /                   |                  .
  //     o | o  /                    |                  .
  //   o   |   o                     |                  .
  //   o   |  /o                  +--|--+               .
  //   | o o o |          <==>    |  |  |               .
  //   |   |/  |                  |  |  |               .
  //   |   +-----------x          |  +------Q-------r   .
  //   |    \  |                  |     |               .
  //   o     \ o                  +-----+               .
  //     o o o\                                         .
  //           \                                        .
  //            Q                                       .
  //             \                                      .
  //              \                                     .
  //               r                                    .
  //
  // Mathematically the (r,z)-to-(x,y,z) transformation is not defined if Q
  // is on the z-axis of G because the cross section is not unique.  In that
  // case, we will treat Q as if it lies on x-axis of G.
  //
  auto xyz_to_rz = [](Vector3d xyz) -> Vector2d {
    const double r = sqrt(xyz(0) * xyz(0) + xyz(1) * xyz(1));
    return Vector2d(r, xyz(2));
  };
  // We compute the the basis vector Br in G's frame. Although a 3D vector, it
  // is stored implicitly in a 2D vector, because v_GBr(2) must be zero. If
  // Q is within a tolerance from the center line, v_GBr = <1, 0, 0> by
  // convention.
  const double r_Q = std::sqrt(p_GQ(0) * p_GQ(0) + p_GQ(1) * p_GQ(1));
  const bool near_center_line = (r_Q < RelativeTolerance(cylinder.radius));
  const Vector2d v_GBr = near_center_line ? Vector2d(1., 0.)
                                          : Vector2d(p_GQ(0), p_GQ(1)) / r_Q;
  auto rz_to_xyz = [&v_GBr](Vector2d rz) -> Vector3d {
    const double r = rz(0);
    const double z = rz(1);
    return Vector3d (r * v_GBr(0), r * v_GBr(1), z);
  };

  // Transform Q from 3D cylinder G to 2D box B.
  const Vector2d p_BQ = xyz_to_rz(p_GQ);

  // The position of the nearest point N expressed in 2D box B's frame in
  // coordinates (r,z).
  Vector2d p_BN;
  double distance;
  // The gradient vector expressed in B's frame in coordinates (r,z).
  Vector2d grad_B;
  std::tie(p_BN, distance, grad_B) = BoxDistanceXd(h, p_BQ);

  // Transform coordinates from (r,z) in B's frame to (x,y,z) in G's frame.
  const Vector3d p_GN = rz_to_xyz(p_BN);
  const Vector3d grad_G = rz_to_xyz(grad_B);

  // Use R_WG for vectors. Use X_WG for points.
  const auto& R_WG = X_WG_.rotation();
  const Vector3d grad_W = R_WG * grad_G;
  return SignedDistanceToPoint<double>{geometry_id_, p_GN, distance, grad_W};
}

// Callback function from fcl::distance to help ComputeSignedDistanceToPoint.
bool DistanceFromPointCallback(fcl::CollisionObjectd* fcl_object_A_ptr,
                               fcl::CollisionObjectd* fcl_object_B_ptr,
                               // NOLINTNEXTLINE
                               void* callback_data, double& threshold) {
  auto& data = *static_cast<DistanceFromPointCallbackData*>(callback_data);

  // We intentionally pass the same number back to FCL in every callback.
  // It instructs FCL to skip any objects proven to be beyond this threshold
  // distance (for example, by checking bounding boxes).
  threshold = data.threshold;

  // We use `const` to prevent modification of the collision objects.
  const fcl::CollisionObjectd* point_object = data.query_point;
  const fcl::CollisionObjectd* geometry_object =
      (data.query_point == fcl_object_A_ptr) ?
      fcl_object_B_ptr : fcl_object_A_ptr;

  const std::vector<GeometryId>& geometry_map = data.geometry_map;
  GeometryId geometry_id = EncodedData(*geometry_object).id(geometry_map);

  const fcl::CollisionGeometryd* collision_geometry =
      geometry_object->collisionGeometry().get();

  // TODO(DamrongGuoy): Replace this custom code when FCL does this for us with
  // the required accuracy and performance.
  //
  DistanceToPoint distance_to_point(geometry_id,
      RigidTransformd(geometry_object->getTransform()),
      point_object->getTranslation());

  SignedDistanceToPoint<double> distance;
  switch (collision_geometry->getNodeType()) {
    case fcl::GEOM_SPHERE:
      distance = distance_to_point(
          *static_cast<const fcl::Sphered*>(collision_geometry));
      break;
    case fcl::GEOM_BOX:
      distance = distance_to_point(
          *static_cast<const fcl::Boxd*>(collision_geometry));
      break;
    case fcl::GEOM_CYLINDER:
      distance = distance_to_point(
          *static_cast<const fcl::Cylinderd*>(collision_geometry));
      break;
    default:
      return false;  // Returning false tells fcl to continue to other objects.
  }

  if (distance.distance <= data.threshold)
    data.distances->emplace_back(distance);

  return false;  // Returning false tells fcl to continue to other objects.
}

// Callback function for FCL's collide() function for retrieving a *single*
// contact.
bool SingleCollisionCallback(fcl::CollisionObjectd* fcl_object_A_ptr,
                             fcl::CollisionObjectd* fcl_object_B_ptr,
                             void* callback_data) {
  // NOTE: Although this function *takes* non-const pointers to satisfy the
  // fcl api, it should not exploit the non-constness to modify the collision
  // objects. We insure this by immediately assigning to a const version and
  // not directly using the provided parameters.
  const fcl::CollisionObjectd& fcl_object_A = *fcl_object_A_ptr;
  const fcl::CollisionObjectd& fcl_object_B = *fcl_object_B_ptr;

  auto& collision_data = *static_cast<CollisionData*>(callback_data);

  // Extract the collision filter keys from the fcl collision objects. These
  // keys will also be used to map the fcl collision object back to the Drake
  // GeometryId for colliding geometries.
  EncodedData encoding_A(fcl_object_A);
  EncodedData encoding_B(fcl_object_B);

  const bool can_collide = collision_data.collision_filter.CanCollideWith(
      encoding_A.encoded_data(), encoding_B.encoded_data());

  // NOTE: Here and below, false is returned regardless of whether collision
  // is detected or not because true tells the broadphase manager to terminate.
  // Since we want *all* collisions, we return false.
  if (!can_collide) return false;

  // Unpack the callback data
  const fcl::CollisionRequestd& request = collision_data.request;

  // This callback only works for a single contact, this confirms a request
  // hasn't been made for more contacts.
  DRAKE_ASSERT(request.num_max_contacts == 1);
  fcl::CollisionResultd result;

  // Perform nearphase collision detection
  fcl::collide(&fcl_object_A, &fcl_object_B, request, result);

  if (!result.isCollision()) return false;

  // Process the contact points
  // NOTE: This assumes that the request is configured to use a single
  // contact.
  const fcl::Contactd& contact = result.getContact(0);

  // Signed distance is negative when penetration depth is positive.
  const double depth = contact.penetration_depth;

  // TODO(SeanCurtis-TRI): Remove this test when FCL issue 375 is fixed.
  // FCL returns osculation as contact but doesn't guarantee a non-zero
  // normal. Drake isn't really in a position to define that normal from the
  // geometry or contact results so, if the geometry is sufficiently close
  // to osculation, we consider the geometries to be non-penetrating.
  if (depth <= std::numeric_limits<double>::epsilon()) return false;

  // By convention, Drake requires the contact normal to point out of B
  // and into A. FCL uses the opposite convention.
  Vector3d drake_normal = -contact.normal;

  // FCL returns a single contact point centered between the two
  // penetrating surfaces. PenetrationAsPointPair expects
  // two, one on the surface of body A (Ac) and one on the surface of body
  // B (Bc). Choose points along the line defined by the contact point and
  // normal, equidistant to the contact point. Recall that signed_distance
  // is strictly non-positive, so signed_distance * drake_normal points
  // out of A and into B.
  Vector3d p_WAc = contact.pos - 0.5 * depth * drake_normal;
  Vector3d p_WBc = contact.pos + 0.5 * depth * drake_normal;

  PenetrationAsPointPair<double> penetration;
  penetration.depth = depth;
  // The engine doesn't know geometry ids; it returns engine indices. The
  // caller must map engine indices to geometry ids.
  const std::vector<GeometryId>& geometry_map = collision_data.geometry_map;
  penetration.id_A = encoding_A.id(geometry_map);
  penetration.id_B = encoding_B.id(geometry_map);
  penetration.p_WCa = p_WAc;
  penetration.p_WCb = p_WBc;
  penetration.nhat_BA_W = drake_normal;
  // Guarantee fixed ordering of pair (A, B). Swap the ids and points on
  // surfaces and then flip the normal.
  if (penetration.id_B < penetration.id_A) {
    std::swap(penetration.id_A, penetration.id_B);
    std::swap(penetration.p_WCa, penetration.p_WCb);
    penetration.nhat_BA_W = -penetration.nhat_BA_W;
  }
  collision_data.contacts->emplace_back(std::move(penetration));

  return false;
}

// Returns a copy of the given fcl collision geometry; throws an exception for
// unsupported collision geometry types. This supplements the *missing* cloning
// functionality in FCL. Issue has been submitted to FCL:
// https://github.com/flexible-collision-library/fcl/issues/246
shared_ptr<fcl::ShapeBased> CopyShapeOrThrow(
    const fcl::CollisionGeometryd& geometry) {
  // NOTE: Returns a shared pointer because of the FCL API in assigning
  // collision geometry to collision objects.
  switch (geometry.getNodeType()) {
    case fcl::GEOM_SPHERE: {
      const auto& sphere = dynamic_cast<const fcl::Sphered&>(geometry);
      return make_shared<fcl::Sphered>(sphere.radius);
    }
    case fcl::GEOM_CYLINDER: {
      const auto& cylinder = dynamic_cast<const fcl::Cylinderd&>(geometry);
      return make_shared<fcl::Cylinderd>(cylinder.radius, cylinder.lz);
    }
    case fcl::GEOM_HALFSPACE:
      // All half spaces are defined exactly the same.
      return make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    case fcl::GEOM_BOX: {
      const auto& box = dynamic_cast<const fcl::Boxd&>(geometry);
      return make_shared<fcl::Boxd>(box.side);
    }
    case fcl::GEOM_CONVEX: {
      const auto& convex = dynamic_cast<const fcl::Convexd&>(geometry);
      // TODO(DamrongGuoy): Change to the copy constructor Convex(other) when
      //  we figure out why "Convex(const Convex& other) = default" created
      //  link errors for Xenial Debug build.  For now we do deep copy of the
      //  vertices and faces instead of simply copying the shared pointer.
      return make_shared<fcl::Convexd>(
          make_shared<const std::vector<Vector3d>>(convex.getVertices()),
          convex.getFaceCount(),
          make_shared<const std::vector<int>>(convex.getFaces()));
    }
    case fcl::GEOM_ELLIPSOID:
    case fcl::GEOM_CAPSULE:
    case fcl::GEOM_CONE:
    case fcl::GEOM_PLANE:
    case fcl::GEOM_TRIANGLE:
      throw std::logic_error(
          "Trying to copy fcl::CollisionGeometry of unsupported GEOM_* type");
    default:
      throw std::logic_error(
          "Trying to copy fcl::CollisionGeometry of non GEOM_* type");
  }
}

// Helper function that creates a *deep* copy of the given collision object.
unique_ptr<fcl::CollisionObjectd> CopyFclObjectOrThrow(
    const fcl::CollisionObjectd& object) {
  shared_ptr<fcl::ShapeBased> geometry_copy =
      CopyShapeOrThrow(*object.collisionGeometry());
  auto copy = make_unique<fcl::CollisionObjectd>(geometry_copy);
  copy->setUserData(object.getUserData());
  copy->setTransform(object.getTransform());
  copy->computeAABB();
  return copy;
}

// Helper function that creates a deep copy of a vector of collision objects.
// Assumes the input vector has already been cleared. The `copy_map` parameter
// serves as a mapping from each source object to its corresponding copy. Used
// to facilitate copying broadphase culling data structures (see
// ProximityEngine::operator=()).
void CopyFclObjectsOrThrow(
    const std::vector<unique_ptr<fcl::CollisionObjectd>>& source_objects,
    std::vector<unique_ptr<fcl::CollisionObjectd>>* target_objects,
    std::unordered_map<const fcl::CollisionObjectd*, fcl::CollisionObjectd*>*
        copy_map) {
  DRAKE_ASSERT(target_objects->size() == 0);
  target_objects->reserve(source_objects.size());
  for (const unique_ptr<fcl::CollisionObjectd>& source_object :
       source_objects) {
    target_objects->emplace_back(CopyFclObjectOrThrow(*source_object));
    copy_map->insert({source_object.get(), target_objects->back().get()});
  }
}

// Builds into the target AABB tree manager based on the reference "other"
// manager and the lookup table from other's collision objects to the target's
// collision objects (the map populated by CopyFclObjectsOrThrow()).
void BuildTreeFromReference(
    const fcl::DynamicAABBTreeCollisionManager<double>& other,
    const std::unordered_map<const fcl::CollisionObjectd*,
                             fcl::CollisionObjectd*>& copy_map,
    fcl::DynamicAABBTreeCollisionManager<double>* target) {
  std::vector<fcl::CollisionObjectd*> other_objects;
  other.getObjects(other_objects);
  for (auto* other_object : other_objects) {
    target->registerObject(copy_map.at(other_object));
  }
  target->update();
}

}  // namespace

// The implementation class for the fcl engine. Each of these functions
// mirrors a method on the ProximityEngine (unless otherwise indicated.
// See ProximityEngine for documentation.
template <typename T>
class ProximityEngine<T>::Impl : public ShapeReifier {
 public:
  Impl() = default;

  Impl(const Impl& other) : ShapeReifier(other) {
    dynamic_tree_.clear();
    dynamic_objects_.clear();
    anchored_tree_.clear();
    anchored_objects_.clear();

    // Copy all of the geometry to ensure that the engine index values stay
    // aligned.
    std::unordered_map<const fcl::CollisionObjectd*, fcl::CollisionObjectd*>
        object_map;
    CopyFclObjectsOrThrow(other.anchored_objects_, &anchored_objects_,
                          &object_map);
    CopyFclObjectsOrThrow(other.dynamic_objects_, &dynamic_objects_,
                          &object_map);

    // Build new AABB trees from the input AABB trees.
    BuildTreeFromReference(other.dynamic_tree_, object_map, &dynamic_tree_);
    BuildTreeFromReference(other.anchored_tree_, object_map, &anchored_tree_);
    collision_filter_ = other.collision_filter_;
  }

  // Only the copy constructor is used to facilitate copying of the parent
  // ProximityEngine class.
  Impl& operator=(const Impl&) = delete;
  Impl(Impl&& other) = delete;
  Impl& operator=(Impl&&) = delete;

  std::unique_ptr<ProximityEngine<AutoDiffXd>::Impl> ToAutoDiff() const {
    auto engine = make_unique<ProximityEngine<AutoDiffXd>::Impl>();

    // TODO(SeanCurtis-TRI): When AutoDiff is fully supported in the internal
    // types, modify this map to the appropriate scalar and modify consuming
    // functions accordingly.
    // Copy all of the geometry to ensure that the engine index values stay
    // aligned.
    std::unordered_map<const fcl::CollisionObjectd*, fcl::CollisionObjectd*>
        object_map;
    CopyFclObjectsOrThrow(anchored_objects_, &engine->anchored_objects_,
                          &object_map);
    CopyFclObjectsOrThrow(dynamic_objects_, &engine->dynamic_objects_,
                          &object_map);
    engine->collision_filter_ = this->collision_filter_;

    // Build new AABB trees from the input AABB trees.
    BuildTreeFromReference(dynamic_tree_, object_map, &engine->dynamic_tree_);
    BuildTreeFromReference(anchored_tree_, object_map, &engine->anchored_tree_);

    return engine;
  }

  ProximityIndex AddDynamicGeometry(const Shape& shape,
                                    GeometryIndex index) {
    // The collision object gets instantiated in the reification process and
    // placed in this unique pointer.
    std::unique_ptr<fcl::CollisionObject<double>> fcl_object;
    shape.Reify(this, &fcl_object);
    dynamic_tree_.registerObject(fcl_object.get());
    ProximityIndex proximity_index(static_cast<int>(dynamic_objects_.size()));
    // Encode the *global* pose index so that the geometry id can be looked up
    // in the event of a collision.
    EncodedData encoding(index, true /* is dynamic */);
    encoding.store_in(fcl_object.get());
    dynamic_objects_.emplace_back(std::move(fcl_object));
    collision_filter_.AddGeometry(encoding.encoded_data());
    return proximity_index;
  }

  ProximityIndex AddAnchoredGeometry(const Shape& shape,
                                     const Isometry3<double>& X_WG,
                                     GeometryIndex index) {
    // The collision object gets instantiated in the reification process and
    // placed in this unique pointer.
    std::unique_ptr<fcl::CollisionObject<double>> fcl_object;
    shape.Reify(this, &fcl_object);
    fcl_object->setTransform(X_WG);
    fcl_object->computeAABB();
    anchored_tree_.registerObject(fcl_object.get());
    anchored_tree_.update();
    ProximityIndex proximity_index(static_cast<int>(anchored_objects_.size()));
    EncodedData encoding(index, false /* is dynamic */);
    encoding.store_in(fcl_object.get());
    anchored_objects_.emplace_back(std::move(fcl_object));
    collision_filter_.AddGeometry(encoding.encoded_data());

    return proximity_index;
  }

  void UpdateGeometryIndex(ProximityIndex proximity_index, bool is_dynamic,
                           GeometryIndex geometry_index) {
    if (is_dynamic) {
      EncodedData::encode_dynamic(geometry_index)
          .store_in(dynamic_objects_[proximity_index].get());
    } else {
      EncodedData::encode_anchored(geometry_index)
          .store_in(anchored_objects_[proximity_index].get());
    }
  }

  optional<GeometryIndex> RemoveGeometry(ProximityIndex index,
                                         bool is_dynamic) {
    if (is_dynamic) {
      return RemoveGeometry(index, &dynamic_tree_, &dynamic_objects_);
    } else {
      return RemoveGeometry(index, &anchored_tree_, &anchored_objects_);
    }
  }

  int num_geometries() const {
    return static_cast<int>(dynamic_objects_.size() + anchored_objects_.size());
  }

  int num_dynamic() const { return static_cast<int>(dynamic_objects_.size()); }

  int num_anchored() const {
    return static_cast<int>(anchored_objects_.size());
  }

  void set_distance_tolerance(double tol) { distance_tolerance_ = tol; }

  double distance_tolerance() const { return distance_tolerance_; }

  // TODO(SeanCurtis-TRI): I could do things here differently a number of ways:
  //  1. I could make this move semantics (or swap semantics).
  //  2. I could simply have a method that returns a mutable reference to such
  //    a vector and the caller sets values there directly.
  void UpdateWorldPoses(const std::vector<Isometry3<T>>& X_WG,
                        const std::vector<GeometryIndex>& indices) {
    DRAKE_DEMAND(indices.size() == dynamic_objects_.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      dynamic_objects_[i]->setTransform(convert(X_WG[indices[i]]));
      dynamic_objects_[i]->computeAABB();
    }
    dynamic_tree_.update();
  }

  // Implementation of ShapeReifier interface

  void ImplementGeometry(const Sphere& sphere, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_sphere = make_shared<fcl::Sphered>(sphere.get_radius());
    TakeShapeOwnership(fcl_sphere, user_data);
  }

  void ImplementGeometry(const Cylinder& cylinder, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_cylinder = make_shared<fcl::Cylinderd>(cylinder.get_radius(),
                                                    cylinder.get_length());
    TakeShapeOwnership(fcl_cylinder, user_data);
  }

  void ImplementGeometry(const HalfSpace&, void* user_data) override {
    // Note: Using `shared_ptr` because of FCL API requirements.
    auto fcl_half_space = make_shared<fcl::Halfspaced>(0, 0, 1, 0);
    TakeShapeOwnership(fcl_half_space, user_data);
  }

  void ImplementGeometry(const Box& box, void* user_data) override {
    auto fcl_box = make_shared<fcl::Boxd>(box.size());
    TakeShapeOwnership(fcl_box, user_data);
  }

  void ImplementGeometry(const Mesh&, void*) override {
    throw std::domain_error("The proximity engine does not support meshes yet");
  }

  //
  // Convert vertices from tinyobj format to FCL format.
  //
  // Vertices from tinyobj are in a vector of floating-points like this:
  //     attrib.vertices = {c0,c1,c2, c3,c4,c5, c6,c7,c8,...}
  //                     = {x, y, z,  x, y, z,  x, y, z,...}
  // We will convert to a vector of Vector3d for FCL like this:
  //     vertices = {{c0,c1,c2}, {c3,c4,c5}, {c6,c7,c8},...}
  //              = {    v0,         v1,         v2,    ...}
  //
  // The size of `attrib.vertices` is three times the number of vertices.
  //
  std::vector<Vector3d> TinyObjToFclVertices(const tinyobj::attrib_t& attrib,
                                             const double scale) const {
    int num_coords = attrib.vertices.size();
    DRAKE_DEMAND(num_coords % 3 == 0);
    std::vector<Vector3d> vertices;
    vertices.reserve(num_coords / 3);

    auto iter = attrib.vertices.begin();
    while (iter != attrib.vertices.end()) {
      // We increment `iter` three times for x, y, and z coordinates.
      double x = *(iter++) * scale;
      double y = *(iter++) * scale;
      double z = *(iter++) * scale;
      vertices.emplace_back(x, y, z);
    }

    return vertices;
  }

  //
  // Convert faces from tinyobj to FCL.
  //
  //
  // A tinyobj mesh has an integer array storing the number of vertices of
  // each polygonal face.
  //     mesh.num_face_vertices = {n0,n1,n2,...}
  //         face0 has n0 vertices.
  //         face1 has n1 vertices.
  //         face2 has n2 vertices.
  //         ...
  // A tinyobj mesh has a vector of vertices that belong to the faces.
  //     mesh.indices = {v0_0, v0_1,..., v0_n0-1,
  //                     v1_0, v1_1,..., v1_n1-1,
  //                     v2_0, v2_1,..., v2_n2-1,
  //                     ...}
  //         face0 has vertices v0_0, v0_1,...,v0_n0-1.
  //         face1 has vertices v1_0, v1_1,...,v1_n1-1.
  //         face2 has vertices v2_0, v2_1,...,v2_n2-1.
  //         ...
  // For fcl::Convex, the `faces` is as an array of integers in this format.
  //     faces = { n0, v0_0,v0_1,...,v0_n0-1,
  //               n1, v1_0,v1_1,...,v1_n1-1,
  //               n2, v2_0,v2_1,...,v2_n2-1,
  //               ...}
  // where n_i is the number of vertices of face_i.
  //
  int TinyObjToFclFaces(const tinyobj::mesh_t& mesh,
                        std::vector<int>* faces) const {
    auto iter = mesh.indices.begin();
    for (const int& num : mesh.num_face_vertices) {
      faces->push_back(num);
      std::for_each(iter, iter + num, [faces](const tinyobj::index_t& index) {
        faces->push_back(index.vertex_index);
      });
      iter += num;
    }

    return mesh.num_face_vertices.size();
  }

  void ImplementGeometry(const Convex& convex, void* user_data) override {
    // We use tiny_obj_loader to read the .obj file of the convex shape.
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    // We keep polygonal faces without triangulating them. Some algorithms for
    // convex geometry perform better with fewer faces.
    bool do_tinyobj_triangulation = false;
    // We use default value (NULL) for the base directory of .mtl file (material
    // description), so it will be searched from the working directory.
    const char* mtl_basedir = nullptr;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
        convex.filename().c_str(), mtl_basedir, do_tinyobj_triangulation);
    if (!ret || !err.empty()) {
      throw std::runtime_error("Error parsing file '" + convex.filename() +
          "' : " + err);
    }

    // TODO(DamrongGuoy) Check that the input is a valid convex polyhedron.
    // 1. Each face is a planar polygon.
    // 2. Each face is a convex polygon.
    // 3. The polyhedron is convex.

    //
    // Now we convert tinyobj data for fcl::Convex.
    //

    if (shapes.size() != 1) {
      throw std::runtime_error("For Convex geometry, the .obj file must have "
                               "one and only one object defined in it.");
    }

    auto vertices = std::make_shared<std::vector<Vector3d>>(
        TinyObjToFclVertices(attrib, convex.scale()));

    const tinyobj::mesh_t& mesh = shapes[0].mesh;

    // We will have `faces.size()` larger than the number of faces. For each
    // face_i, the vector `faces` contains both the number and indices of its
    // vertices:
    //     faces = { n0, v0_0,v0_1,...,v0_n0-1,
    //               n1, v1_0,v1_1,...,v1_n1-1,
    //               n2, v2_0,v2_1,...,v2_n2-1,
    //               ...}
    // where n_i is the number of vertices of face_i.
    //
    auto faces = std::make_shared<std::vector<int>>();
    int num_faces = TinyObjToFclFaces(mesh, faces.get());

    // Create fcl::Convex.
    auto fcl_convex = make_shared<fcl::Convexd>(
        vertices, num_faces, faces);
    TakeShapeOwnership(fcl_convex, user_data);

    // TODO(DamrongGuoy): Per f2f with SeanCurtis-TRI, we want ProximityEngine
    // to own vertices and face by a map from filename.  This way we won't have
    // to read the same file again and again when we create multiple Convex
    // objects from the same file.
  }

  std::vector<SignedDistancePair<double>>
  ComputeSignedDistancePairwiseClosestPoints(
      const std::vector<GeometryId>& geometry_map) const {
    std::vector<SignedDistancePair<double>> witness_pairs;
    DistanceData distance_data{&geometry_map, &collision_filter_};
    distance_data.nearest_pairs = &witness_pairs;
    distance_data.request.enable_nearest_points = true;
    distance_data.request.enable_signed_distance = true;
    distance_data.request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    distance_data.request.distance_tolerance = distance_tolerance_;

    dynamic_tree_.distance(&distance_data, DistanceCallback);
    dynamic_tree_.distance(
        const_cast<fcl::DynamicAABBTreeCollisionManager<double>*>(
            &anchored_tree_),
        &distance_data, DistanceCallback);
    return witness_pairs;
  }

  std::vector<SignedDistanceToPoint<double>> ComputeSignedDistanceToPoint(
      const Vector3d& p_WQ,
      const std::vector<GeometryId>& geometry_map,
      const double threshold) const {
    // We create a sphere of zero radius centered at the query point and put
    // it into a fcl::CollisionObject.
    auto fcl_sphere = make_shared<fcl::Sphered>(0.0);  // sphere of zero radius
    fcl::CollisionObjectd query_point(fcl_sphere);
    query_point.setTranslation(p_WQ);
    query_point.computeAABB();

    std::vector<SignedDistanceToPoint<double>> distances;

    DistanceFromPointCallbackData data{&query_point, &geometry_map,
                                       threshold, &distances};

    anchored_tree_.distance(&query_point, &data, DistanceFromPointCallback);
    dynamic_tree_.distance(&query_point, &data, DistanceFromPointCallback);

    return distances;
  }


  std::vector<PenetrationAsPointPair<double>> ComputePointPairPenetration(
      const std::vector<GeometryId>& geometry_map) const {
    std::vector<PenetrationAsPointPair<double>> contacts;
    // CollisionData stores references to the provided data structures.
    CollisionData collision_data{&geometry_map, &collision_filter_};
    collision_data.contacts = &contacts;
    collision_data.request.num_max_contacts = 1;
    collision_data.request.enable_contact = true;
    // NOTE: As of 5/1/2018 the GJK implementation of Libccd appears to be
    // superior to FCL's "independent" implementation. Furthermore, libccd
    // appears to behave badly if its gjk tolerance is much tighter than
    // 2e-12. Until this changes, we explicitly specify these parameters rather
    // than relying on FCL's defaults.
    collision_data.request.gjk_tolerance = 2e-12;
    collision_data.request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

    dynamic_tree_.collide(&collision_data, SingleCollisionCallback);

    // NOTE: The interface to DynamicAABBTreeCollisionManager::collide
    // requires the input collision manager pointer to be *non* const.
    // As of 02/06/2018, it appears the only opportunity for modification
    // of the AABB tree (and its contents) occurs in the callback provided.
    // See the definition of SingleCollisionCallback above to see that no
    // modification takes place.
    dynamic_tree_.collide(
        const_cast<fcl::DynamicAABBTreeCollisionManager<double>*>(
            &anchored_tree_),
        &collision_data, SingleCollisionCallback);
    return contacts;
  }

  // TODO(SeanCurtis-TRI): Update this with the new collision filter method.
  void ExcludeCollisionsWithin(
      const std::unordered_set<GeometryIndex>& dynamic,
      const std::unordered_set<GeometryIndex>& anchored) {
    // Preventing collision between members in a single set is simple: assign
    // every geometry to the same clique.

    // There are no collisions between anchored geometries. So, to meaningfully
    // add collisions, there must be dynamic geometry. Only perform work if:
    //   1. There are multiple dynamic geometries, or
    //   2. There are non-zero numbers of dynamic *and* anchored geometries.
    //
    // NOTE: Given the set of geometries G, if the pair (gᵢ, gⱼ), gᵢ, gⱼ ∈ G
    // is already filtered, this *will* add a redundant clique.
    // TODO(SeanCurtis-TRI): If redundant cliques proves to have a performance
    // impact before the alternate filtering mechanism is in place, revisit this
    // algorithm to prevent redundancy.

    if (dynamic.size() > 1 || (dynamic.size() > 0 && anchored.size() > 0)) {
      int clique = collision_filter_.next_clique_id();
      for (auto index : dynamic) {
        EncodedData encoding(index, true /* is dynamic */);
        collision_filter_.AddToCollisionClique(encoding.encoded_data(), clique);
      }
      for (auto index : anchored) {
        EncodedData encoding(index, false /* is dynamic */);
        collision_filter_.AddToCollisionClique(encoding.encoded_data(), clique);
      }
    }
  }

  void ExcludeCollisionsBetween(
      const std::unordered_set<GeometryIndex>& dynamic1,
      const std::unordered_set<GeometryIndex>& anchored1,
      const std::unordered_set<GeometryIndex>& dynamic2,
      const std::unordered_set<GeometryIndex>& anchored2) {
    // TODO(SeanCurtis-TRI): Update this with the new collision filter method.

    // NOTE: This is a brute-force implementation. It does not claim to be
    // optimal in any way. It merely guarantees the semantics given. If the
    // two sets of geometries are in fact the *same* set (i.e., this is used
    // to create the same effect as ExcludeCollisionsWithin), it will work, but
    // be horribly inefficient (with each geometry picking up N cliques).
    using std::transform;
    using std::back_inserter;
    using std::vector;

    // There are no collisions between anchored geometries. So, there must be
    // dynamic geometry for work to be necessary.
    if (dynamic1.size() > 0 || dynamic2.size() > 0) {
      vector<EncodedData> group1;
      transform(dynamic1.begin(), dynamic1.end(), back_inserter(group1),
                EncodedData::encode_dynamic);
      transform(anchored1.begin(), anchored1.end(), back_inserter(group1),
                EncodedData::encode_anchored);
      vector<EncodedData> group2;
      transform(dynamic2.begin(), dynamic2.end(), back_inserter(group2),
                EncodedData::encode_dynamic);
      transform(anchored2.begin(), anchored2.end(), back_inserter(group2),
                EncodedData::encode_anchored);

      // O(N²) process which generates O(N²) cliques. For the two collision
      // groups G and H, each pair (g, h), g ∈ G, h ∈ H, requires a unique
      // clique. If the cliques were not unique, then there would be a pair
      // (a, b) where a, b ∈ G or a, b ∈ H where a and b have the same clique.
      // And that would lead to removal of self-collision within one of the
      // collision groups.
      //
      // NOTE: Using cliques for this purpose is *horribly* inefficient. This is
      // exactly what collision filter groups are best at.
      for (auto encoding1 : group1) {
        for (auto encoding2 : group2) {
          if ((encoding1.is_dynamic() || encoding2.is_dynamic()) &&
              collision_filter_.CanCollideWith(encoding1.encoded_data(),
                                               encoding2.encoded_data())) {
            int clique = collision_filter_.next_clique_id();
            collision_filter_.AddToCollisionClique(encoding2.encoded_data(),
                                                   clique);
            collision_filter_.AddToCollisionClique(encoding1.encoded_data(),
                                                   clique);
          }
        }
      }
    }
  }

  bool CollisionFiltered(GeometryIndex index1, bool is_dynamic_1,
                         GeometryIndex index2, bool is_dynamic_2) const {
    // Collisions between anchored geometries are implicitly filtered.
    if (!is_dynamic_1 && !is_dynamic_2) return true;
    EncodedData encoding1(index1, is_dynamic_1);
    EncodedData encoding2(index2, is_dynamic_2);
    return !collision_filter_.CanCollideWith(encoding1.encoded_data(),
                                             encoding2.encoded_data());
  }

  int get_next_clique() { return collision_filter_.next_clique_id(); }

  void set_clique(GeometryIndex index, int clique) {
    EncodedData encoding(index, true /* is dynamic */);
    collision_filter_.AddToCollisionClique(encoding.encoded_data(), clique);
  }

  // Testing utilities

  bool IsDeepCopy(const Impl& other) const {
    if (this != &other) {
      // Function for validating that two objects are different objects with
      // "identical" data. The test isn't exhaustive (for example, the
      // parameters of the particular geometric shape are not compared--instead,
      // we compare the AABBs).
      auto ValidateObject = [](const fcl::CollisionObject<double>& test,
                               const fcl::CollisionObject<double>& ref) {
        return test.getUserData() == ref.getUserData() && &test != &ref &&
               test.getNodeType() == ref.getNodeType() &&
               test.getObjectType() == ref.getObjectType() &&
               test.getAABB().center() == ref.getAABB().center() &&
               test.getAABB().width() == ref.getAABB().width() &&
               test.getAABB().height() == ref.getAABB().height() &&
               test.getAABB().depth() == ref.getAABB().depth();
      };
      bool is_copy = true;
      is_copy = is_copy &&
                this->dynamic_objects_.size() == other.dynamic_objects_.size();
      is_copy =
          is_copy &&
          this->anchored_objects_.size() == other.anchored_objects_.size();
      if (is_copy) {
        for (size_t i = 0; i < this->dynamic_objects_.size(); ++i) {
          const fcl::CollisionObject<double>& test_object =
              *this->dynamic_objects_.at(i);
          const fcl::CollisionObject<double>& ref_object =
              *other.dynamic_objects_.at(i);
          is_copy = is_copy && ValidateObject(test_object, ref_object);
        }
        for (size_t i = 0; i < this->anchored_objects_.size(); ++i) {
          const fcl::CollisionObject<double>& test_object =
              *this->anchored_objects_.at(i);
          const fcl::CollisionObject<double>& ref_object =
              *other.anchored_objects_.at(i);
          is_copy = is_copy && ValidateObject(test_object, ref_object);
        }
      }
      return is_copy;
    }

    return false;
  }

  int peek_next_clique() const { return collision_filter_.peek_next_clique(); }

  const Isometry3<double>& GetX_WG(ProximityIndex index,
                                   bool is_dynamic) const {
    if (is_dynamic) {
      return dynamic_objects_[index]->getTransform();
    } else {
      return anchored_objects_[index]->getTransform();
    }
  }

  GeometryIndex GetGeometryIndex(ProximityIndex index, bool is_dynamic) const {
    if (is_dynamic) {
      return EncodedData(*dynamic_objects_[index]).index();
    } else {
      return EncodedData(*anchored_objects_[index]).index();
    }
  }

 private:
  // Engine on one scalar can see the members of other engines.
  friend class ProximityEngineTester;
  template <typename>
  friend class ProximityEngine;

  // Removes the geometry with the given proximity index from the given tree. It
  // potentially moves another object to take its slot in the vector of objects
  // to maintain a contiguous memory block.
  optional<GeometryIndex> RemoveGeometry(
      ProximityIndex index, fcl::DynamicAABBTreeCollisionManager<double>* tree,
      std::vector<std::unique_ptr<fcl::CollisionObject<double>>>* geometries) {
    std::vector<unique_ptr<fcl::CollisionObject<double>>>& typed_geometries =
        *geometries;
    fcl::CollisionObjectd* fcl_object = typed_geometries[index].get();
    const size_t old_size = tree->size();
    tree->unregisterObject(fcl_object);
    // NOTE: The FCL API provides no other mechanism for confirming the
    // unregistration was successful.
    DRAKE_DEMAND(old_size == tree->size() + 1);
    optional<GeometryIndex> moved{};
    DRAKE_DEMAND(typed_geometries.size() > 0);
    const size_t last = typed_geometries.size() - 1;
    if (index < last) {
      // Removed geometry that *isn't* the last in the geometries. Swap last
      // into empty slot.
      const fcl::CollisionObjectd* move_object = typed_geometries[last].get();
      EncodedData encoding(*move_object);
      moved = encoding.index();
      typed_geometries[index].swap(typed_geometries.back());
    }
    typed_geometries.pop_back();
    return moved;
  }

  // TODO(SeanCurtis-TRI): Convert these to scalar type T when I know how to
  // transmogrify them. Otherwise, while the engine can't be transmogrified, the
  // results on an <AutoDiffXd> type will still be double.

  // Helper method called by the various ImplementGeometry overrides to
  // facilitate the logistics of creating shapes from specifications. `data`
  // is a unique_ptr of an fcl CollisionObject that should be instantiated
  // with the given shape.
  void TakeShapeOwnership(const std::shared_ptr<fcl::ShapeBased>& shape,
                          void* data) {
    DRAKE_ASSERT(data != nullptr);
    std::unique_ptr<fcl::CollisionObject<double>>& fcl_object_ptr =
        *reinterpret_cast<std::unique_ptr<fcl::CollisionObject<double>>*>(data);
    fcl_object_ptr = make_unique<fcl::CollisionObject<double>>(shape);
  }

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

  // The mechanism for dictating collision filtering.
  CollisionFilterLegacy collision_filter_;

  // The tolerance that determines when the iterative process would terminate.
  // @see ProximityEngine::set_distance_tolerance() for more details.
  double distance_tolerance_{1E-6};
};

template <typename T>
ProximityEngine<T>::ProximityEngine() : impl_(new Impl()) {}

template <typename T>
ProximityEngine<T>::~ProximityEngine() {
  delete impl_;
}

template <typename T>
ProximityEngine<T>::ProximityEngine(ProximityEngine<T>::Impl* impl)
    : impl_(impl) {}

template <typename T>
ProximityEngine<T>::ProximityEngine(const ProximityEngine<T>& other)
    : impl_(new ProximityEngine<T>::Impl(*other.impl_)) {}

template <typename T>
ProximityEngine<T>& ProximityEngine<T>::operator=(
    const ProximityEngine<T>& other) {
  if (this == &other) return *this;
  if (impl_) delete impl_;
  impl_ = new ProximityEngine<T>::Impl(*other.impl_);
  return *this;
}

template <typename T>
ProximityEngine<T>::ProximityEngine(ProximityEngine<T>&& other) noexcept
    : impl_(std::move(other.impl_)) {
  other.impl_ = new ProximityEngine<T>::Impl();
}

template <typename T>
ProximityEngine<T>& ProximityEngine<T>::operator=(
    ProximityEngine<T>&& other) noexcept {
  if (this == &other) return *this;
  if (impl_) delete impl_;
  impl_ = std::move(other.impl_);
  other.impl_ = new ProximityEngine<T>::Impl();
  return *this;
}

template <typename T>
ProximityIndex ProximityEngine<T>::AddDynamicGeometry(
    const Shape& shape, GeometryIndex index) {
  return impl_->AddDynamicGeometry(shape, index);
}

template <typename T>
ProximityIndex ProximityEngine<T>::AddAnchoredGeometry(
    const Shape& shape, const Isometry3<double>& X_WG, GeometryIndex index) {
  return impl_->AddAnchoredGeometry(shape, X_WG, index);
}

template <typename T>
void ProximityEngine<T>::UpdateGeometryIndex(ProximityIndex proximity_index,
                                             bool is_dynamic,
                                             GeometryIndex geometry_index) {
  impl_->UpdateGeometryIndex(proximity_index, is_dynamic, geometry_index);
}

template <typename T>
optional<GeometryIndex> ProximityEngine<T>::RemoveGeometry(
    ProximityIndex index, bool is_dynamic) {
  return impl_->RemoveGeometry(index, is_dynamic);
}

template <typename T>
int ProximityEngine<T>::num_geometries() const {
  return impl_->num_geometries();
}

template <typename T>
int ProximityEngine<T>::num_dynamic() const {
  return impl_->num_dynamic();
}

template <typename T>
int ProximityEngine<T>::num_anchored() const {
  return impl_->num_anchored();
}

template <typename T>
void ProximityEngine<T>::set_distance_tolerance(double tol) {
  impl_->set_distance_tolerance(tol);
}

template <typename T>
double ProximityEngine<T>::distance_tolerance() const {
  return impl_->distance_tolerance();
}

template <typename T>
std::unique_ptr<ProximityEngine<AutoDiffXd>> ProximityEngine<T>::ToAutoDiffXd()
    const {
  return unique_ptr<ProximityEngine<AutoDiffXd>>(
      new ProximityEngine<AutoDiffXd>(impl_->ToAutoDiff().release()));
}

template <typename T>
void ProximityEngine<T>::UpdateWorldPoses(
    const std::vector<Isometry3<T>>& X_WG,
    const std::vector<GeometryIndex>& indices) {
  impl_->UpdateWorldPoses(X_WG, indices);
}

template <typename T>
std::vector<SignedDistancePair<double>>
ProximityEngine<T>::ComputeSignedDistancePairwiseClosestPoints(
    const std::vector<GeometryId>& geometry_map) const {
  return impl_->ComputeSignedDistancePairwiseClosestPoints(geometry_map);
}

template <typename T>
std::vector<SignedDistanceToPoint<double>>
ProximityEngine<T>::ComputeSignedDistanceToPoint(
    const Vector3<double>& query,
    const std::vector<GeometryId>& geometry_map,
    const double threshold) const {
  return impl_->ComputeSignedDistanceToPoint(query, geometry_map, threshold);
}


template <typename T>
std::vector<PenetrationAsPointPair<double>>
ProximityEngine<T>::ComputePointPairPenetration(
    const std::vector<GeometryId>& geometry_map) const {
  return impl_->ComputePointPairPenetration(geometry_map);
}

template <typename T>
void ProximityEngine<T>::ExcludeCollisionsWithin(
    const std::unordered_set<GeometryIndex>& dynamic,
    const std::unordered_set<GeometryIndex>& anchored) {
  impl_->ExcludeCollisionsWithin(dynamic, anchored);
}

template <typename T>
void ProximityEngine<T>::ExcludeCollisionsBetween(
    const std::unordered_set<GeometryIndex>& dynamic1,
    const std::unordered_set<GeometryIndex>& anchored1,
    const std::unordered_set<GeometryIndex>& dynamic2,
    const std::unordered_set<GeometryIndex>& anchored2) {
  impl_->ExcludeCollisionsBetween(dynamic1, anchored1, dynamic2, anchored2);
}

template <typename T>
bool ProximityEngine<T>::CollisionFiltered(
    GeometryIndex index1, bool is_dynamic_1,
    GeometryIndex index2, bool is_dynamic_2) const {
  return impl_->CollisionFiltered(index1, is_dynamic_1, index2, is_dynamic_2);
}

// Client-attorney interface for GeometryState to manipulate collision filters.

template <typename T>
int ProximityEngine<T>::get_next_clique() {
  return impl_->get_next_clique();
}

template <typename T>
void ProximityEngine<T>::set_clique(GeometryIndex index, int clique) {
  impl_->set_clique(index, clique);
}

// Testing utilities

template <typename T>
bool ProximityEngine<T>::IsDeepCopy(const ProximityEngine<T>& other) const {
  return impl_->IsDeepCopy(*other.impl_);
}

template <typename T>
int ProximityEngine<T>::peek_next_clique() const {
  return impl_->peek_next_clique();
}

template <typename T>
const Isometry3<double>& ProximityEngine<T>::GetX_WG(ProximityIndex index,
                                                     bool is_dynamic) const {
  return impl_->GetX_WG(index, is_dynamic);
}

template <typename T>
GeometryIndex ProximityEngine<T>::GetGeometryIndex(ProximityIndex index,
                                                   bool is_dynamic) const {
  return impl_->GetGeometryIndex(index, is_dynamic);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::internal::ProximityEngine)
