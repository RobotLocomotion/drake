#include "drake/geometry/proximity/distance_to_point_callback.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {
namespace internal {
namespace point_distance {

template <typename T>
void SphereDistanceInSphereFrame(const fcl::Sphered& sphere,
                                 const Vector3<T>& p_SQ, Vector3<T>* p_SN,
                                 T* distance, Vector3<T>* grad_S) {
  const double radius = sphere.radius;
  const T dist_SQ = p_SQ.norm();
  // The gradient is always in the direction from the center of the sphere to
  // the query point Q, regardless of whether the point Q is outside or inside
  // the sphere S.  The gradient is undefined if the query point Q is at the
  // center of the sphere S.
  //
  // If the query point Q is near the center of the sphere S within a
  // tolerance, we arbitrarily set the gradient vector as documented in
  // query_object.h (QueryObject::ComputeSignedDistanceToPoint).
  const double tolerance = DistanceToPointRelativeTolerance(radius);
  // Unit vector in x-direction of S's frame.
  const Vector3<T> Sx = Vector3<T>::UnitX();
  const bool non_zero_displacement = (dist_SQ > tolerance);
  // Gradient vector expressed in S's frame.
  *grad_S = non_zero_displacement ? p_SQ / dist_SQ : Sx;

  // p_SN is the position of a witness point N in the geometry frame S.
  *p_SN = T(radius) * (*grad_S);

  // Do not compute distance as ∥p_SQ∥₂, because the gradient of ∥p_SQ∥₂ w.r.t.
  // p_SQ is p_SQᵀ/∥p_SQ∥₂ which is not well defined at p_SQ = 0. Instead,
  // compute the distance as p_NQ_S.dot(grad_S).
  *distance = (p_SQ - *p_SN).dot(*grad_S);
}

template <typename T>
void ComputeDistanceToPrimitive(const fcl::Sphered& sphere,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W) {
  const Vector3<T> p_GQ_G = X_WG.inverse() * p_WQ;
  Vector3<T> grad_G;
  SphereDistanceInSphereFrame(sphere, p_GQ_G, p_GN, distance, &grad_G);

  // Gradient vector expressed in World frame.
  *grad_W = X_WG.rotation() * grad_G;
}

template <typename T>
void ComputeDistanceToPrimitive(const fcl::Halfspaced& halfspace,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W) {
  // FCL stores the halfspace as {x | nᵀ * x > d}, with n being a unit length
  // normal vector. Both n and x are expressed in the halfspace frame.
  // In Drake, the halfspace is *always* defined as n_G = (0, 0, 1), d = 0.
  // That means the distance to the plane is merely the z-component of p_GQ and
  // the nearest point on the surface is (p_GQ(0), pGQ(1), 0).
  const Vector3<T> n_G = halfspace.n.cast<T>();
  const Vector3<T> p_GQ = X_WG.inverse() * p_WQ;
  DRAKE_ASSERT(n_G(0) == 0 && n_G(1) == 0 && n_G(2) == 1);
  DRAKE_DEMAND(halfspace.d == 0);
  *distance = p_GQ(2);
  *p_GN << p_GQ(0), p_GQ(1), 0;
  *grad_W = X_WG.rotation() * n_G;
}

template <typename T>
void ComputeDistanceToPrimitive(const fcl::Capsuled& capsule,
                                const math::RigidTransform<T>& X_WG,
                                const Vector3<T>& p_WQ, Vector3<T>* p_GN,
                                T* distance, Vector3<T>* grad_W) {
  const double radius = capsule.radius;
  const double half_length = capsule.lz / 2;

  // If the query point Q is closest to the end caps of the capsule, then we can
  // re-use the distance to sphere calculations since they are effectively the
  // same. Since our capsule is aligned with the local z-axis, we can simply
  // compare the z co-ordinates in the capsule's frame G to determine which
  // section the query point Q falls in.
  // z
  // ^           ●●
  // |         ●    ●    Top end cap
  // |--------●------●-------------------
  // |        ●      ●
  // |        ●      ●   Spine
  // |        ●      ●
  // |--------●------●-------------------
  // |         ●    ●    Bottom end cap
  // |           ●●

  const Vector3<T> p_GQ = X_WG.inverse() * p_WQ;
  if (p_GQ.z() >= half_length || p_GQ.z() <= -half_length) {
    // Represent the end cap of the capsule using a sphere S of the same radius.
    const fcl::Sphered sphere_S(radius);
    // The sphere is defined centered on the origin of frame S. Frame S and G
    // are related by a simple translation (their bases are perfectly aligned).
    // So, a vector quantity expressed in frame G is the same as when expressed
    // in S.
    const Vector3<T> p_GS{
        0, 0, (p_GQ.z() >= half_length) ? half_length : -half_length};
    // The query point measured w.r.t. the sphere origin (equivalently expressed
    // in G or S).
    const Vector3<T> p_SQ = p_GQ - p_GS;
    // Position vector of the nearest point N expressed in S's frame.
    Vector3<T> grad_S;
    Vector3<T> p_SN;
    SphereDistanceInSphereFrame(sphere_S, p_SQ, &p_SN, distance, &grad_S);
    *grad_W = X_WG.rotation() * grad_S;  // grad_S = grad_G because R_GS = I.
    *p_GN = p_GS + p_SN;                 // p_SN = p_SN_G because R_GS = I.
  } else {
    // The query point Q projects onto (and is nearest to) the spine. The
    // gradient is perpendicular to the spine and points from N' to Q (where
    // N' is the point on the spine nearest Q). Equivalently, the gradient is
    // in the same direction as the vector from G's origin to R, where point R
    // is the projection of Q onto G's x-y plane. And M is the nearest point
    // on the capsule's surface to R. The distance between Q and N is the same
    // as between R and M.
    //
    // This allows us to solve for the gradient, distance, and nearest point
    // by computing the distance from R to a sphere with the same radius and
    // centered on G's origin. We can then shift M to N by adding the
    // z-component back into N.
    //
    // z
    // ^           ● ●
    // |         ●     ●
    // |        ●       ●
    // |        ●   |---N---> Q
    // |        ●   |   ●
    // |        ●   G---M---> R
    // |        ●   |   ●
    // |        ●   |   ●
    // |        ●       ●
    // |         ●     ●
    // |           ● ●

    // TODO(SeanCurtis-TRI): For further efficiency, consider doing these
    //  calculations in 2D and then promoting them back into 3D.
    const Vector3<T> p_GR{p_GQ.x(), p_GQ.y(), 0};
    const fcl::Sphered sphere_S(radius);
    Vector3<T> p_GM;
    Vector3<T> grad_G;
    SphereDistanceInSphereFrame(sphere_S, p_GR, &p_GM, distance, &grad_G);
    *p_GN << p_GM.x(), p_GM.y(), p_GQ.z();
    *grad_W = X_WG.rotation() * grad_G;
  }
}

#define INSTANTIATE_DISTANCE_TO_PRIMITIVE(Shape, S)                         \
template void ComputeDistanceToPrimitive<S>(                                \
    const fcl::Shape&, const math::RigidTransform<S>&, const Vector3<S>&,   \
    Vector3<S>*, S*, Vector3<S>*)

// INSTANTIATE_DISTANCE_TO_PRIMITIVE(Sphered, double);
// INSTANTIATE_DISTANCE_TO_PRIMITIVE(Sphered, AutoDiffXd);
// INSTANTIATE_DISTANCE_TO_PRIMITIVE(Halfspaced, double);
// INSTANTIATE_DISTANCE_TO_PRIMITIVE(Halfspaced, AutoDiffXd);
// INSTANTIATE_DISTANCE_TO_PRIMITIVE(Capsuled, double);
// INSTANTIATE_DISTANCE_TO_PRIMITIVE(Capsuled, AutoDiffXd);

#undef INSTANTIATE_DISTANCE_TO_PRIMITIVE

template <typename T>
SignedDistanceToPoint<T> DistanceToPoint<T>::operator()(const fcl::Boxd& box) {
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
  const Vector3<T> grad_W = X_WG_.rotation() * grad_G;
  const Vector3<T> p_WN = X_WG_ * p_GN_G;
  T distance = grad_W.dot(p_WQ_ - p_WN);
  return SignedDistanceToPoint<T>{geometry_id_, p_GN_G, distance, grad_W};
}

template <typename T>
SignedDistanceToPoint<T> DistanceToPoint<T>::operator()(
    const fcl::Capsuled& capsule) {
  // TODO(SeanCurtis-TRI): This would be better if `SignedDistanceToPoint`
  //  could be default constructed in an uninitialized state and then
  //  pointers to its contents could be passed directly to ComputeDistance...
  //  This would eliminate the inevitable copy in the constructor.
  T distance{};
  Vector3<T> p_GN_G, grad_W;
  ComputeDistanceToPrimitive(capsule, X_WG_, p_WQ_, &p_GN_G, &distance,
                             &grad_W);

  return SignedDistanceToPoint<T>{geometry_id_, p_GN_G, distance, grad_W};
}

template <typename T>
SignedDistanceToPoint<T> DistanceToPoint<T>::operator()(
    const fcl::Cylinderd& cylinder) {
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
  // We compute the basis vector Br in G's frame. Although a 3D vector, it is
  // stored implicitly in a 2D vector, because v_GBr(2) must be zero. If Q is
  // within a tolerance from the center line, v_GBr = <1, 0, 0> by convention.
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
  return SignedDistanceToPoint<T>{geometry_id_, p_GN, distance, grad_W};
}

template <typename T>
SignedDistanceToPoint<T> DistanceToPoint<T>::operator()(
    const fcl::Ellipsoidd& ellipsoid) {
  if constexpr (std::is_same_v<T, double>) {
    // TODO(SeanCurtis-TRI): Replace this short-term hack with something that
    //  provides higher precision. Can an iterative method provide derivatives?
    //  See: https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf

    // For now, we'll simply use FCL's sphere-ellipsoid algorithm to compute
    // the signed distance for a zero-radius sphere. (Note: this uses the
    // generic GJK-EPA algorithm pair).
    const fcl::Sphered sphere_Q(0.0);

    fcl::DistanceRequestd request;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    request.distance_tolerance = 1e-6;
    request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

    // By passing in poses X_WG and X_WQ, the result will likewise be in the
    // world frame.
    fcl::DistanceResultd result_W;

    fcl::distance(&ellipsoid, X_WG_.GetAsIsometry3(),
                  &sphere_Q, math::RigidTransformd(p_WQ_).GetAsIsometry3(),
                  request, result_W);

    const Vector3d& p_WN = result_W.nearest_points[0];
    const Vector3d p_GN = X_WG_.inverse() * p_WN;
    const Vector3d& radii = ellipsoid.radii;
    // The gradient is always perpendicular to the ellipsoid at the witness
    // point on the ellipsoid surface. Therefore, for the implicit equation of
    // the ellipsoid:
    //    f(x, y, z) = x² / a² + y² / b² + z² / c² - 1 = 0
    // Its gradient is normal to the surface.
    //    ∇f = <2x / a², 2y / b², 2z / c²>
    //    n = ∇f / |∇f|
    // We are assured that all radii are strictly greater than zero (see the
    // constructor for geometry::Ellipsoid), so we don't risk a divide by zero
    // and it likewise guarantees that this gradient vector will always be
    // normalizable and unique; the point on the surface of the ellipsoid can
    // *never* be the zero vector. Because we're normalizing, we omit the scale
    // factor of 2 -- we'll just be dividing it right back out.
    // This gives us a good normal, regardless of how close to the surface the
    // query point Q is.
    const Vector3d grad_G = Vector3d(p_GN.x() / (radii.x() * radii.x()),
                                     p_GN.y() / (radii.y() * radii.y()),
                                     p_GN.z() / (radii.z() * radii.z()))
                                .normalized();

    const Vector3d grad_W = X_WG_.rotation() * grad_G;
    return SignedDistanceToPoint<T>(geometry_id_, p_GN, result_W.min_distance,
                                    grad_W);
  } else {
    // ScalarSupport<AutoDiffXd> should preclude ever calling this with
    // T = AutoDiffXd.
    DRAKE_UNREACHABLE();
  }
}

template <typename T>
SignedDistanceToPoint<T> DistanceToPoint<T>::operator()(
    const fcl::Halfspaced& halfspace) {
  T distance{};
  Vector3<T> p_GN_G, grad_W;
  ComputeDistanceToPrimitive(halfspace, X_WG_, p_WQ_, &p_GN_G, &distance,
                             &grad_W);

  return SignedDistanceToPoint<T>{geometry_id_, p_GN_G, distance, grad_W};
}

template <typename T>
SignedDistanceToPoint<T> DistanceToPoint<T>::operator()(
    const fcl::Sphered& sphere) {
  T distance{};
  Vector3<T> p_GN_G, grad_W;
  ComputeDistanceToPrimitive(sphere, X_WG_, p_WQ_, &p_GN_G, &distance, &grad_W);

  return SignedDistanceToPoint<T>{geometry_id_, p_GN_G, distance, grad_W};
}


template <typename T>
template <int dim, typename U>
int DistanceToPoint<T>::ExtremalAxis(const Vector<U, dim>& p,
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

template <typename T>
template <int dim>
std::tuple<Vector<T, dim>, Vector<T, dim>, bool>
DistanceToPoint<T>::ComputeDistanceToBox(const Vector<double, dim>& h,
                                         const Vector<T, dim>& p_GQ_G) {
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
  enum class Location { kInside, kBoundary, kOutside };
  auto clamp = [&h](const int i, const T& coord, Location* location) -> T {
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
    int axis = ExtremalAxis(p_GQ_G, h);
    // NOTE: This will do funny things to the derivatives; this functionally
    // treats it as constant w.r.t. all derivatives. However, as the point
    // rolls from one Voronoi region to another, it goes funny.
    double sign = Sign(ExtractDoubleOrThrow(p_GQ_G(axis)));
    p_GN_G(axis) = sign * h(axis);
    grad_G(axis) = sign;
  }

  return std::make_tuple(p_GN_G, grad_G, is_Q_on_edge_or_vertex);
}

bool ScalarSupport<double>::is_supported(fcl::NODE_TYPE node_type) {
  switch (node_type) {
    case fcl::GEOM_BOX:
    case fcl::GEOM_CAPSULE:
    case fcl::GEOM_CYLINDER:
    case fcl::GEOM_ELLIPSOID:
    case fcl::GEOM_HALFSPACE:
    case fcl::GEOM_SPHERE:
      return true;
    default:
      return false;
  }
}

template <typename T>
bool Callback(fcl::CollisionObjectd* object_A_ptr,
              fcl::CollisionObjectd* object_B_ptr,
              // NOLINTNEXTLINE
              void* callback_data, double& threshold) {
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
  threshold = std::max(data.threshold, kEps);

  // We use `const` to prevent modification of the collision objects.
  const fcl::CollisionObjectd* geometry_object =
      (&data.query_point == object_A_ptr) ? object_B_ptr : object_A_ptr;

  const EncodedData encoding(*geometry_object);
  GeometryId geometry_id = encoding.id();

  const fcl::CollisionGeometryd* collision_geometry =
      geometry_object->collisionGeometry().get();
  if (ScalarSupport<T>::is_supported(collision_geometry->getNodeType())) {
    const math::RigidTransform<T> typed_X_WG(data.X_WGs.at(geometry_id));
    DistanceToPoint<T> distance_to_point(geometry_id, typed_X_WG, data.p_WQ_W);

    SignedDistanceToPoint<T> distance;
    switch (collision_geometry->getNodeType()) {
      case fcl::GEOM_BOX:
        distance = distance_to_point(
            *static_cast<const fcl::Boxd*>(collision_geometry));
        break;
      case fcl::GEOM_CAPSULE:
        distance = distance_to_point(
            *static_cast<const fcl::Capsuled*>(collision_geometry));
        break;
      case fcl::GEOM_CYLINDER:
        distance = distance_to_point(
            *static_cast<const fcl::Cylinderd*>(collision_geometry));
        break;
      case fcl::GEOM_ELLIPSOID:
        distance = distance_to_point(
            *static_cast<const fcl::Ellipsoidd*>(collision_geometry));
        break;
      case fcl::GEOM_HALFSPACE:
        distance = distance_to_point(
            *static_cast<const fcl::Halfspaced*>(collision_geometry));
        break;
      case fcl::GEOM_SPHERE:
        distance = distance_to_point(
            *static_cast<const fcl::Sphered*>(collision_geometry));
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

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS((
    &Callback<T>
))

}  // namespace point_distance
}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::internal::point_distance::DistanceToPoint)
