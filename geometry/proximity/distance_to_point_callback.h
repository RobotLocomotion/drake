#pragma once

#include <algorithm>
#include <limits>
#include <tuple>
#include <unordered_map>
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

/* Supporting data for the distance-to-point callback (see Callback below).
 It includes:
    - The query point Q, measured and expressed in the world frame, `p_WQ_W`.
    - The fcl collision object representing the query point, Q.
    - A distance threshold beyond which distances will not be reported.
    - The pose of the geometry being queried against.
    - The T-valued poses of _all_ geometries in the corresponding SceneGraph
      keyed on their GeometryId.
    - A vector of distance results -- one instance of SignedDistanceToPoint for
      every supported geometry which lies with the threshold.
 */
template <typename T>
struct CallbackData {
  /* Constructs the fully-specified callback data. The values are as described
   above. _Some_ of the parameters are aliased in the data and require the
   aliased parameters to remain valid at least as long as the CallbackData
   instance.

   @param query_in            The object representing the query point. Aliased.
   @param threshold_in        The query threshold.
   @param p_WQ_W_in           The T-valued position of the query point.
   @param X_WGs_in            The T-valued poses. Aliased.
   @param distances_in[out]   The output results. Aliased.
   */
  CallbackData(
      fcl::CollisionObjectd* query_in,
      const double threshold_in,
      const Vector3<T>& p_WQ_W_in,
      const std::unordered_map<GeometryId, math::RigidTransform<T>>* X_WGs_in,
      std::vector<SignedDistanceToPoint<T>>* distances_in)
      : query_point(*query_in),
        threshold(threshold_in),
        p_WQ_W(p_WQ_W_in),
        X_WGs(*X_WGs_in),
        distances(*distances_in) {
    DRAKE_DEMAND(query_in != nullptr);
    DRAKE_DEMAND(X_WGs_in != nullptr);
    DRAKE_DEMAND(distances_in != nullptr);
  }

  /* The query fcl object.  */
  const fcl::CollisionObjectd& query_point;

  /* The query threshold.  */
  const double threshold;

  /* The T-valued query point Q.  */
  const Vector3<T> p_WQ_W;

  /* The T-valued pose of every geometry.  */
  const std::unordered_map<GeometryId, math::RigidTransform<T>>& X_WGs;

  /* The accumulator for results.  */
  std::vector<SignedDistanceToPoint<T>>& distances;
};

/* @name Functions for computing distance from point to primitives
 This family of functions compute the distance from a point Q to a primitive
 shape.  Refer to QueryObject::ComputeSignedDistanceToPoint() for more details.

 @param X_WG                  The pose of the primitive geometry G in the world
                              frame.
 @param p_WQ                  The position of the point Q in the world frame.
 @param p_GN[out]             The position of the witness point N on the
                              primitive, expressed in the primitive geometry
                              frame G.
 @param distance[out]         The signed distance from the point Q to the
                              primitive.
 @param grad_W[out]           The gradient of the distance function w.r.t p_GQ
                              (the position of the point Q in the primitive's
                              frame). This gradient vector has unit-length and
                              is expressed in the world frame. Where grad_W is
                              not unique, then an arbitrary value is
                              assigned (as documented in
                              QueryObject::ComputeSignedDistanceToPoint().  */
//@{

/* Computes distance from point to sphere with the understanding that all
 quantities are measured and expressed in the sphere's frame, S. Otherwise, the
 semantics of the parameters are as documented as above.  */
template <typename T>
void SphereDistanceInSphereFrame(const fcl::Sphered& sphere,
                                 const Vector3<T>& p_SQ, Vector3<T>* p_SN,
                                 T* distance, Vector3<T>* grad_S);

/* Overload of ComputeDistanceToPrimitive() for sphere primitive. */
template <typename T>
void ComputeDistanceToPrimitive(const fcl::Sphered& sphere,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W);

/* Overload of ComputeDistanceToPrimitive() for halfspace primitive. */
template <typename T>
void ComputeDistanceToPrimitive(const fcl::Halfspaced& halfspace,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W);

/* Overload of ComputeDistanceToPrimitive() for capsule primitive. */
template <typename T>
void ComputeDistanceToPrimitive(const fcl::Capsuled& capsule,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W);

// TODO(DamrongGuoy): Add overloads for all supported geometries.

//@}

/* A functor to compute signed distance between a point and a geometry. By
 design, one instance should be created for each unique pairing of query point Q
 with geometry G. The functor is constructed with all the parameters _except_
 the actual shape. It relies on overloading the () operator and ADL to handle
 specific shape types.  */
template <typename T>
class DistanceToPoint {
 public:
  /* Constructs the functor DistanceToPoint.
   @param id    The id of the geometry G,
   @param X_WG  The pose of the G in world frame,
   @param p_WQ  The position of the query point Q in world frame.  */
  DistanceToPoint(const GeometryId id,
                  const math::RigidTransform<T>& X_WG,
                  const Vector3<T>& p_WQ) :
      geometry_id_(id), X_WG_(X_WG), p_WQ_(p_WQ) {}

  // TODO(DamrongGuoy): Revisit computation over operator() overloads as per
  //  issue: https://github.com/RobotLocomotion/drake/issues/11227

  /* Overload to compute distance to a box.  */
  SignedDistanceToPoint<T> operator()(const fcl::Boxd& box);

  /* Overload to compute distance to a capsule.  */
  SignedDistanceToPoint<T> operator()(const fcl::Capsuled& capsule);

  /* Overload to compute distance to a cylinder.  */
  SignedDistanceToPoint<T> operator()(const fcl::Cylinderd& cylinder);

  /* Overload to compute distance to an ellipsoid.  */
  SignedDistanceToPoint<T> operator()(const fcl::Ellipsoidd& ellipsoid);

  /* Overload to compute distance to a halfspace.  */
  SignedDistanceToPoint<T> operator()(const fcl::Halfspaced& halfspace);

  /* Overload to compute distance to a sphere.  */
  SignedDistanceToPoint<T> operator()(const fcl::Sphered& sphere);

  /* Reports the "sign" of x with a small modification; Sign(0) --> 1.
   @tparam U  Templated to allow DistanceToPoint<AutoDiffXd> to still compute
              Sign<double> or Sign<AutoDiffXd> as needed.  */
  template <typename U = T>
  static U Sign(const U& x) { return (x < U(0.0)) ? U(-1.0) : U(1.0); }

  /* Picks the axis i whose coordinate p(i) is closest to the boundary value
   ±bounds(i). If there are ties, we prioritize according to an arbitrary
   ordering: +x,-x,+y,-y,+z,-z.

   Note: the two vector types can be different scalars, they must be extractable
   to double.

   @tparam dim The number of rows in the column vector.
   @tparam U   The scalar type of the point `p`; defaults to T.  */
  template <int dim, typename U = T>
  static int ExtremalAxis(const Vector<U, dim>& p,
                          const Vector<double, dim>& bounds);

  /* Calculates the signed distance from a query point Q to a box, G. The box
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
  static std::tuple<Vector<T, dim>, Vector<T, dim>, bool> ComputeDistanceToBox(
      const Vector<double, dim>& h, const Vector<T, dim>& p_GQ_G);

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
/* @name   Mechanism for reporting on which scalars and for which shapes
            point-to-shape queries can be made.

 By default, nothing is supported. For each supported scalar type, a class
 specialization is provided which whitelists the supported shape types.

 @tparam T      The computational scalar type.  */
//@{

template <typename T>
struct ScalarSupport {
  static bool is_supported(fcl::NODE_TYPE node_type) { return false; }
};

/* Primitive support for double-valued query.  */
template <>
struct ScalarSupport<double> {
  static bool is_supported(fcl::NODE_TYPE node_type);
};

/* Primitive support for AutoDiff-valued query.  */
template <typename DerType>
struct ScalarSupport<Eigen::AutoDiffScalar<DerType>> {
  static bool is_supported(fcl::NODE_TYPE node_type) {
    switch (node_type) {
      case fcl::GEOM_SPHERE:
      case fcl::GEOM_BOX:
      case fcl::GEOM_HALFSPACE:
      case fcl::GEOM_CAPSULE:
        return true;
      default:
        return false;
    }
  }
};

//@}

/* The callback function for computing the signed distance between a point and
 a supported shape. Intended to be invoked as a result of broadphase culling
 of candidate geometry pairs for the pair (`object_A_ptr`, `object_B_ptr`).

 @pre The `callback_data` is an instance of point_distance::CallbackData.
 @pre One of the two fcl objects matches the CallbackData.query_point object.
 */
template <typename T>
bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr,
              // NOLINTNEXTLINE
              void* callback_data, double& threshold);

}  // namespace point_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake
