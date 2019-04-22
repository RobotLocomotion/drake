#pragma once

// Exclude internal classes from doxygen.
#if !defined(DRAKE_DOXYGEN_CXX)

#include <limits>
#include <tuple>
#include <vector>

#include <fcl/fcl.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/query_results/signed_distance_to_point.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace point_distance {

/** Supporting data for the distance-to-point callback (see Callback below).
 It includes:
    - The query point Q, measured and expressed in the world frame, `p_WQ_W`.
    - The fcl collision object representing the query point, Q.
    - A map from GeometryIndex to GeometryId (to facilitate reporting GeometryId
      values in the results).
    - A distance threshold beyond which distances will not be reported.
    - The pose of the geometry being queried against.
    - The T-valued poses of _all_ geometries in the corresponding SceneGraph
      indexable by their GeometryIndex.
    - A vector of distance results -- one instance of SignedDistanceToPoint for
      every supported geometry which lies with the threshold.
 */
template <typename T>
struct CallbackData {
  /** Constructs the fully-specified callback data. The values are as described
   above. _Some_ of the parameters are aliased in the data and require the
   aliased parameters to remain valid at least as long as the CallbackData
   instance.

   @param query_in            The object representing the query point. Aliased.
   @param geometry_map_in     The index -> id map. Aliased.
   @param threshold_in        The query threshold.
   @param p_WQ_W_in           The T-valued position of the query point.
   @param X_WGs_in             The T-valued poses. Aliased.
   @param distances_in[out]   The output results. Aliased.
   */
  CallbackData(
      fcl::CollisionObjectd* query_in,
      const std::vector<GeometryId>* geometry_map_in,
      const double threshold_in,
      const Vector3<T>& p_WQ_W_in,
      const std::vector<Isometry3<T>>* X_WGs_in,
      std::vector<SignedDistanceToPoint<T>>* distances_in)
      : query_point(*query_in),
        geometry_map(*geometry_map_in),
        threshold(threshold_in),
        p_WQ_W(p_WQ_W_in),
        X_WGs(*X_WGs_in),
        distances(*distances_in) {
    DRAKE_DEMAND(query_in);
    DRAKE_DEMAND(geometry_map_in);
    DRAKE_DEMAND(X_WGs_in);
    DRAKE_DEMAND(distances_in);
  }

  /** The query fcl object.  */
  const fcl::CollisionObjectd& query_point;

  /** The index --> id map.  */
  const std::vector<GeometryId>& geometry_map;

  /** The query threshold.  */
  const double threshold;

  /** The T-valued query point Q.  */
  const Vector3<T> p_WQ_W;

  /** The T-valued pose of every geometry.  */
  const std::vector<Isometry3<T>>& X_WGs;

  /** The accumulator for results.  */
  std::vector<SignedDistanceToPoint<T>>& distances;
};


/** @name Functions for computing distance from point to primitives
 This family of functions compute the distance from a point Q to a primitive
 shape.  Refer to QueryObject::ComputeSignedDistanceToPoint() for more details.

 @param X_WG            The pose of the primitive geometry G in the world frame.
 @param p_WQ            The position of the point Q in the world frame.
 @param p_GN[out]       The position of the witness point N on the primitive,
                        expressed in the primitive geometry frame G.
 @param distance[out]   The signed distance from the point Q to the primitive.
 @param grad_W[out]     The gradient of the distance function w.r.t p_GQ (the
                        position of the point Q in the primitive's frame). This
                        gradient vector has unit-length and is expressed in the
                        world frame. Where grad_W is not well-defined, then
                        an arbitrary value is assigned (as documented in
                        QueryObject::ComputeSignedDistanceToPoint().  */
//@{

/** Overload of ComputeDistanceToPrimitive() for sphere primitive. */
template <typename T>
void ComputeDistanceToPrimitive(const fcl::Sphered& sphere,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W,
                                bool* is_grad_W_well_defined) {
  const double radius = sphere.radius;
  const Vector3<T> p_GQ_G = X_WG.inverse() * p_WQ;
  const T dist_GQ = p_GQ_G.norm();

  // The gradient is always in the direction from the center of the sphere to
  // the query point Q, regardless of whether the point Q is outside or inside
  // the sphere G.  The gradient is undefined if the query point Q is at the
  // center of the sphere G.
  //
  // If the query point Q is near the center of the sphere G within a
  // tolerance, we arbitrarily set the gradient vector as documented in
  // query_object.h (QueryObject::ComputeSignedDistanceToPoint).
  const double tolerance = DistanceToPointRelativeTolerance(radius);
  // Unit vector in x-direction of G's frame.
  const Vector3<T> Gx = Vector3<T>::UnitX();
  *is_grad_W_well_defined = (dist_GQ > tolerance);
  // Gradient vector expressed in G's frame.
  const Vector3<T> grad_G = *is_grad_W_well_defined ? p_GQ_G / dist_GQ : Gx;

  // p_GN is the position of a witness point N in the geometry frame G.
  *p_GN = T(radius) * grad_G;

  // Do not compute distance as ∥p_GQ∥₂, because the gradient of ∥p_GQ∥₂ w.r.t.
  // p_GQ is p_GQᵀ/∥p_GQ∥₂ which is not well defined at p_GQ = 0. Instead,
  // compute the distance as p_NQ_G.dot(grad_G).
  *distance = (p_GQ_G - *p_GN).dot(grad_G);

  // Gradient vector expressed in World frame.
  *grad_W = X_WG.rotation() * grad_G;
}

/** Overload of ComputeDistanceToPrimitive() for halfspace primitive. */
template <typename T>
void ComputeDistanceToPrimitive(const fcl::Halfspaced& halfspace,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W,
                                bool* is_grad_W_well_defined) {
  // FCL stores the halfspace as {x | nᵀ * x <= d}, with n being a unit length
  // normal vector. Both n and x are expressed in the halfspace frame.
  const Vector3<T> n_G = halfspace.n.cast<T>();
  const Vector3<T> p_GQ = X_WG.inverse() * p_WQ;
  *distance = n_G.dot(p_GQ) - T(halfspace.d);
  *p_GN = p_GQ - *distance * n_G;
  *grad_W = X_WG.rotation() * n_G;
  *is_grad_W_well_defined = true;
}

// TODO(DamrongGuoy): Add overloads for all supported geometries.

//@}

/** A functor to compute signed distance between a point and a geometry. By
 design, one instance should be created for each unique pairing of query point Q
 with geometry G. The functor is constructed with all the parameters _except_
 the actual shape. It relies on overloading the () operator and ADL to handle
 specific shape types.  */
template <typename T>
class DistanceToPoint {
 public:
  /** Constructs the functor DistanceToPoint.
   @param id    The id of the geometry G,
   @param X_WG  The pose of the G in world frame,
   @param p_WQ  The position of the query point Q in world frame.  */
  DistanceToPoint(const GeometryId id,
                  const math::RigidTransform<T>& X_WG,
                  const Vector3<T>& p_WQ) :
      geometry_id_(id), X_WG_(X_WG), p_WQ_(p_WQ) {}

  // TODO(DamrongGuoy): Revisit computation over operator() overloads as per
  //  issue: https://github.com/RobotLocomotion/drake/issues/11227

  /** Overload to compute distance to a sphere.  */
  SignedDistanceToPoint<T> operator()(const fcl::Sphered& sphere) {
    T distance{};
    Vector3<T> p_GN_G, grad_W;
    bool is_grad_W_well_defined{};
    ComputeDistanceToPrimitive(sphere, X_WG_, p_WQ_, &p_GN_G, &distance,
                               &grad_W, &is_grad_W_well_defined);

    return SignedDistanceToPoint<T>{geometry_id_, p_GN_G, distance, grad_W,
                                    is_grad_W_well_defined};
  }

  /** Overload to compute distance to a box.  */
  SignedDistanceToPoint<T> operator()(const fcl::Boxd& box) {
    // Express the given query point Q in the frame of the box geometry G.
    const Vector3<T> p_GQ_G = X_WG_.inverse() * p_WQ_;
    // The box G is an axis-aligned box [-h(0),h(0)]x[-h(1),h(1)]x[-h(2),h(2)]
    // centered at the origin, where h(i) is half the size of the box in the
    // i-th coordinate.
    const Eigen::Vector3d h = box.side / 2.0;
    Vector3<T> p_GN_G, grad_G;
    bool is_Q_on_edge_or_vertex{};
    std::tie(p_GN_G, grad_G, is_Q_on_edge_or_vertex) =
        ComputeDistanceToBox(h, p_GQ_G);
    const bool is_grad_W_well_defined = !is_Q_on_edge_or_vertex;
    const Vector3<T> grad_W = X_WG_.rotation() * grad_G;
    const Vector3<T> p_WN = X_WG_ * p_GN_G;
    T distance = grad_W.dot(p_WQ_ - p_WN);
    return SignedDistanceToPoint<T>{geometry_id_, p_GN_G, distance, grad_W,
                                    is_grad_W_well_defined};
  }

  /** Overload to compute distance to a cylinder.  */
  SignedDistanceToPoint<T> operator()(const fcl::Cylinderd& cylinder) {
    using std::sqrt;
    // TODO(SeanCurtis-TRI): This is not a good algorithm for differentiation.
    //  Replace it with one that is.

    // Overview. First, we map the problem from the 3D cylinder G (in frame G)
    // to the 2D box B (in frame B) of G's cross section through the plane
    // containing the query point Q and the center line of G. Then, we call the
    // function ComputeDistanceToBox() to get the signed distance, the nearest
    // point N, and the gradient vector expressed in B's frame. Finally, we map
    // the answers back into G's frame.
    // Express the query point Q in the cylinder geometry G's frame.
    const Vector3<T> p_GQ = X_WG_.inverse() * p_WQ_;
    // The 2D cross section B is the box [-h(0),h(0)]x[-h(1),h(1)], where
    // h(0) and h(1) are the radius and the half length of the cylinder.
    const Eigen::Vector2d h(cylinder.radius, cylinder.lz / 2.0);
    // Transform coordinates between (x,y,z) in G's frame and (r,z) in B's
    // frame. The basis vector `Bz` is aligned with `Gz`, and `r` is the
    // distance to the z-axis (i.e., √(x² + y²)), and z transfers unchanged. The
    // coordinate r is in the radial direction of G in the plane through Q and
    // z-axis of G.
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
    auto xyz_to_rz = [](Vector3<T> xyz) -> Vector2<T> {
      // TODO(hongkai.dai): Why doesn't this cause bad derivatives? See test
      // distance_to_point_test.cc, (DistanceToPoint, Cylinder), Case 1.
      const T r = sqrt(xyz(0) * xyz(0) + xyz(1) * xyz(1));
      return Vector2<T>(r, xyz(2));
    };
    // We compute the the basis vector Br in G's frame. Although a 3D vector, it
    // is stored implicitly in a 2D vector, because v_GBr(2) must be zero. If
    // Q is within a tolerance from the center line, v_GBr = <1, 0, 0> by
    // convention.
    const T r_Q = sqrt(p_GQ(0) * p_GQ(0) + p_GQ(1) * p_GQ(1));
    const bool near_center_line =
        (r_Q < DistanceToPointRelativeTolerance(cylinder.radius));
    const Vector2<T> v_GBr = near_center_line
                                 ? Vector2<T>(1., 0.)
                                 : Vector2<T>(p_GQ(0), p_GQ(1)) / r_Q;
    auto rz_to_xyz = [&v_GBr](Vector2<T> rz) -> Vector3<T> {
      const T r = rz(0);
      const T z = rz(1);
      return Vector3<T>(r * v_GBr(0), r * v_GBr(1), z);
    };

    // Transform Q from 3D cylinder G to 2D box B.
    const Vector2<T> p_BQ = xyz_to_rz(p_GQ);

    // The position of the nearest point N expressed in 2D box B's frame in
    // coordinates (r,z).
    Vector2<T> p_BN;
    // The gradient vector expressed in B's frame in coordinates (r,z).
    Vector2<T> grad_B;
    bool is_Q_on_edge_or_vertex{};
    std::tie(p_BN, grad_B, is_Q_on_edge_or_vertex) =
        ComputeDistanceToBox(h, p_BQ);

    // Transform coordinates from (r,z) in B's frame to (x,y,z) in G's frame.
    const Vector3<T> p_GN = rz_to_xyz(p_BN);
    const Vector3<T> grad_G = rz_to_xyz(grad_B);

    // Use R_WG for vectors. Use X_WG for points.
    const auto& R_WG = X_WG_.rotation();
    const Vector3<T> grad_W = R_WG * grad_G;
    const Vector3<T> p_WN = X_WG_ * p_GN;
    T distance = grad_W.dot(p_WQ_ - p_WN);
    // TODO(hongkai.dai): grad_W is not well defined when Q is on the top or
    // bottom rims of the cylinder, or when it is inside the box and on the
    // central axis, with the nearest feature being the barrel of the cylinder.
    return SignedDistanceToPoint<T>{geometry_id_, p_GN, distance, grad_W,
                                    true /* is_grad_W_well_defined */};
  }

  /** Reports the "sign" of x with a small modification; Sign(0) --> 1.
   @tparam U  Templated to allow DistanceToPoint<AutoDiffXd> to still compute
              Sign<double> or Sign<AutoDiffXd> as needed.  */
  template <typename U = T>
  static U Sign(const U& x) { return (x < U(0.0)) ? U(-1.0) : U(1.0); }

  /** Picks the axis i whose coordinate p(i) is closest to the boundary value
   ±bounds(i). If there are ties, we prioritize according to an arbitrary
   ordering: +x,-x,+y,-y,+z,-z.

   Note: the two vector types can be different scalars, they must be extractable
   to double.

   @tparam dim The number of rows in the column vector.
   @tparam U   The scalar type of the point `p`; defaults to T.  */
  template <int dim, typename U = T>
  static int ExtremalAxis(const Vector<U, dim>& p,
                          const Vector<double, dim>& bounds) {
    double min_dist = std::numeric_limits<double>::infinity();
    int axis = -1;
    for (int i = 0; i < dim; ++i) {
      for (const auto bound : {bounds(i), -bounds(i)}) {
        const double dist = std::abs(bound - ExtractDoubleOrThrow(p(i)));
        if (dist < min_dist) {
          min_dist = dist;
          axis = i;
        }
      }
    }
    return axis;
  }

  /** Calculates the signed distance from a query point Q to a box, G. The box
   can be either two-dimensional or three-dimensional. It is defined centered
   on its frame G and aligned with G's axes. We use the 3D version directly
   for the signed distance from Box and use the 2D version indirectly for the
   signed distance from Cylinder.
   @param h        The vector from the center of the box to the positive
                   corner of the box measured in frame G (the "half" size of
                   the box).
                   Mathematically the box is the Cartesian product
                   [-h(0),h(0)]x[-h(1),h(1)] in 2D or
                   [-h(0),h(0)]x[-h(1),h(1)]x[-h(2),h(2)] in 3D.
   @param p_GQ_G   The position of the query point Q in G's frame.
   @retval {p_GN_G, grad_G, is_Q_on_edge_or_vertex}   The tuple of the position
                   of the nearest point N in G's frame and the gradient vector
                   in the same frame G, together with the type of the nearest
                   feature.
   @tparam dim     The dimension, must be 2 or 3.  */
  template <int dim>
  std::tuple<Vector<T, dim>, Vector<T, dim>, bool> ComputeDistanceToBox(
      const Vector<double, dim>& h, const Vector<T, dim>& p_GQ_G) {
    using std::abs;

    // TODO(DamrongGuoy): Revisit this implementation based on a recommendation
    // made in PR 11208:
    // https://reviewable.io/reviews/robotlocomotion/drake/11208#-Lc6yion1LIgZrj5yohz:-Lc6yion1LIgZrj5yoi-:b-1qyfoi

    // We need to classify Q as inside, outside, or on the boundary of G,
    // where 'on the boundary' means within a tolerance of the boundary.
    // This helper function takes the i-th coordinate `coord` of p_GQ_G.
    // It returns the clamped value of `coord` within ±h(i). It also returns
    // an enum to indicate whether the i-th coordinate is inside the interval
    // (-h(i),+h(i)), or within a tolerance of the bounded value ±h(i), or
    // outside the interval.
    enum class Location {kInside, kBoundary, kOutside};
    auto clamp = [&h](const int i, const T& coord,
                      Location* location) -> T {
      const double tolerance = DistanceToPointRelativeTolerance(h(i));
      if (abs(coord) > h(i) + tolerance) {
        *location = Location::kOutside;
        return Sign(coord) * h(i);
      } else if (abs(coord) >= h(i) - tolerance) {
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

    Vector<T, dim> p_GC_G;
    VectorLoc location;
    for (int i = 0; i < p_GC_G.size(); ++i) {
      p_GC_G(i) = clamp(i, p_GQ_G(i), &location(i));
    }
    int num_dim_on_boundary = 0;
    for (int i = 0; i < dim; ++i) {
      num_dim_on_boundary += location(i) == Location::kBoundary ? 1 : 0;
    }
    const bool is_Q_on_edge_or_vertex = num_dim_on_boundary >= 2 ? true : false;

    // Initialize the position of the nearest point N on ∂B as that of C.
    // Note: if Q is outside or on the boundary of B, then C is N. In the
    // case where Q is inside, this value will be changed.
    Vector<T, dim> p_GN_G = p_GC_G;
    T distance;
    Vector<T, dim> grad_G = Vector<T, dim>::Zero();

    if ((location.array() == Location::kOutside).any()) {
      // Q is outside the box.
      Vector<T, dim> p_NQ_G = p_GQ_G - p_GN_G;
      distance = p_NQ_G.norm();
      DRAKE_DEMAND(distance != 0.);
      grad_G = p_NQ_G / distance;
    } else if ((location.array() == Location::kBoundary).any()) {
      for (int i = 0; i < dim; ++i) {
        if (location(i) == Location::kBoundary) {
          grad_G(i) = Sign(ExtractDoubleOrThrow(p_GC_G(i)));
        }
      }
      grad_G.normalize();
    } else {
      // Q is inside the box.
      // In 2D (3D), the nearest point N is the axis-aligned projection of Q
      // onto one of the edge (faces) of the box.  The gradient vector is along
      // that direction.
      int axis = this->ExtremalAxis(p_GQ_G, h);
      // NOTE: This will do funny things to the derivatives; this functionally
      // treats it as constant w.r.t. all derivatives. However, as the point
      // rolls from one Voronoi region to another, it goes funny.
      double sign = Sign(ExtractDoubleOrThrow(p_GQ_G(axis)));
      p_GN_G(axis) = sign * h(axis);
      grad_G(axis) = sign;
    }

    return std::make_tuple(p_GN_G, grad_G, is_Q_on_edge_or_vertex);
  }

 private:
  // The id of the geometry G.
  const GeometryId geometry_id_;
  // The pose of the geometry G in World frame.
  const math::RigidTransform<T> X_WG_;
  // The position of the query point Q in World frame.
  const Vector3<T> p_WQ_;
};

// TODO(SeanCurtis-TRI): Replace this clunky mechanism with a new mechanism
// which does this implicitly via ADL and templates.
/** @name   Mechanism for reporting on which scalars and for which shapes
            point-to-shape queries can be made.

 By default, nothing is supported. For each supported scalar type, a class
 specialization is provided which whitelists the supported shape types.

 @tparam T      The computational scalar type.  */
//@{

template <typename T>
struct ScalarSupport {
  static bool is_supported(fcl::NODE_TYPE node_type) { return false; }
};

/** Primitive support for double-valued query.  */
template <>
struct ScalarSupport<double> {
  static bool is_supported(fcl::NODE_TYPE node_type) {
    switch (node_type) {
      case fcl::GEOM_SPHERE:
      case fcl::GEOM_BOX:
      case fcl::GEOM_CYLINDER:
        return true;
      default:
        return false;
    }
  }
};

/** Primitive support for AutoDiff-valued query.  */
template <typename DerType>
struct ScalarSupport<Eigen::AutoDiffScalar<DerType>> {
  static bool is_supported(fcl::NODE_TYPE node_type) {
    switch (node_type) {
      case fcl::GEOM_SPHERE:
      case fcl::GEOM_BOX:
        return true;
      default:
        return false;
    }
  }
};

//@}

/** The callback function for computing the signed distance between a point and
 a supported shape. Intended to be invoked as a result of broadphase culling
 of candidate geometry pairs for the pair (`object_A_ptr`, `object_B_ptr`).

 @pre The `callback_data` is an instance of point_distance::CallbackData.
 @pre One of the two fcl objects matches the CallbackData.query_point object.
 */
template <typename T>
bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr,
              // NOLINTNEXTLINE
              void* callback_data, double& threshold) {
  auto& data = *static_cast<CallbackData<T>*>(callback_data);

  // We intentionally pass the same number back to FCL in every callback.
  // It instructs FCL to skip any objects proven to be beyond this threshold
  // distance (for example, by checking bounding boxes).
  threshold = data.threshold;

  // We use `const` to prevent modification of the collision objects.
  const fcl::CollisionObjectd* geometry_object =
      (&data.query_point == object_A_ptr) ? object_B_ptr : object_A_ptr;

  const std::vector<GeometryId>& geometry_map = data.geometry_map;
  const EncodedData encoding(*geometry_object);
  GeometryId geometry_id = encoding.id(geometry_map);

  const fcl::CollisionGeometryd* collision_geometry =
      geometry_object->collisionGeometry().get();
  if (ScalarSupport<T>::is_supported(collision_geometry->getNodeType())) {
    const math::RigidTransform<T> typed_X_WG(data.X_WGs[encoding.index()]);
    DistanceToPoint<T> distance_to_point(geometry_id, typed_X_WG, data.p_WQ_W);

    SignedDistanceToPoint<T> distance;
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
        // Returning false tells fcl to continue to other objects.
        return false;
    }

    if (distance.distance <= data.threshold) {
      data.distances.emplace_back(distance);
    }
  }

  return false;  // Returning false tells fcl to continue to other objects.
}

}  // namespace point_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake

#endif  // !defined(DRAKE_DOXYGEN_CXX)
