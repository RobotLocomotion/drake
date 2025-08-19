#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

namespace util {
// Generic scalar abs that uses ADL to find the right overload.
template <typename T>
inline T Abs(const T& x) {
  using std::abs;
  return abs(x);
}

template <typename T>
inline T Max(const T& a, const T& b) {
  using std::max;
  return max(a, b);
}

}  // namespace util

template <typename T>
std::optional<Eigen::Vector3<T>> GetNormalAtPointForBox(
    const Box& box, const Eigen::Vector3<T>& p) {
  static const T tol = 1e-5;

  // Extract box half dimensions
  const double h_w = box.width() / 2;   // half_width
  const double h_d = box.depth() / 2;   // half_depth
  const double h_h = box.height() / 2;  // half_height

  // A container for all planes defined by the normals of a box
  // and their distance to the origin. Each plane is defined by
  // a 4-vector (n, d)^T where n is the normal vector and d is
  // the distance to the plane along the normal direction. If a
  // point p lies on a plane (n, d) then n*p - d = 0. This is
  // used to verify if a point p lines on a plane; if so, the normal
  // at point p is n.
  Eigen::Matrix<T, 4, 6> box_planes = Eigen::Matrix<T, 4, 6>::Zero();
  // clang-format off
  box_planes << 1.0, -1.0, 0.0,  0.0, 0.0, 0.0,
                0.0,  0.0, 1.0, -1.0, 0.0, 0.0,
                0.0,  0.0, 0.0,  0.0, 1.0, -1.0,
                -h_w, -h_w, -h_d, -h_d, -h_h, -h_h;
  // clang-format on

  // Iterate through the planes. Return the normal vector of first
  // plane that contains the point.
  Eigen::Vector4<T> p_h(p.x(), p.y(), p.z(), 1.0);
  for (Eigen::Index c = 0; c < box_planes.cols(); ++c) {
    if ((box_planes.col(c).dot(p_h) * box_planes.col(c).dot(p_h)) < tol) {
      return box_planes.block(0, c, 3, 1);
    }
  }

  return std::nullopt;
}

template <typename T>
std::optional<Eigen::Vector3<T>> GetNormalAtPointForSphere(
    const Sphere& sphere, const Eigen::Vector3<T>& p) {
  static const T tol = 1e-5;
  // Check that the point is close enough to the surface of the sphere.
  // If yes, the normal direction is in the same as the point.
  const T diff = p.norm() - sphere.radius();
  if ((diff * diff) < tol) {
    return p.normalized();
  }
  return std::nullopt;
}

template <typename T>
std::optional<Eigen::Vector3<T>> GetNormalAtPointForCapsule(
    const Capsule& capsule, const Eigen::Vector3<T>& p) {
  static const T tol = 1e-5;

  const T h_l = capsule.length() / 2.0;  // half length
  const T r = capsule.radius();

  // If a point (x,y,z) is on the surface of the capsule, there are two
  // possibilities:
  //   A. it is on the surface of the cylindrical section
  //   B. it is on either of the semispherical sections
  //
  // In case A, we only need to check the length of subvector (x,y) is equal to
  // the radius r, and ignore coordinate z (make it 0). In case B, we subtract
  // half length from the z coordinate, and check the norm of the resulting
  // vector (x,y,z-h_l) is  equal to radius r. We can combine both conditions
  // into a "test point" as follows
  //
  //       |(x,y, max(z-h_l,0))| = r
  //
  // to verify p is in the surface. The normal vector at point p is in the same
  // direction as the test point.
  T zero = 0.0;
  T z_delta = util::Abs<T>(p.z()) - h_l;
  T d_z = util::Max<T>(z_delta, zero);
  Eigen::Vector3<T> test_p(p.x(), p.y(), d_z);
  const T diff = test_p.norm() - r;
  if ((diff * diff) < tol) {
    test_p.z() *= p.z() > 0 ? 1 : -1;
    return test_p.normalized();
  }

  return std::nullopt;
}

template <typename T>
std::optional<Eigen::Vector3<T>> GetNormalAtPointForCylinder(
    const Cylinder& cylinder, const Eigen::Vector3<T>& p) {
  static const T tol = 1e-5;

  const T h_l = cylinder.length() / 2.0;
  const T r = cylinder.radius();

  // Check point is within bounds of cylinder
  const T p_z_abs = util::Abs(p.z());
  if (p_z_abs > (h_l + tol)) {
    return std::nullopt;
  }
  const T p_r = p.head(2).norm();
  if (p_r > (r + tol)) {
    return std::nullopt;
  }

  // If point is on the curved surface of the cylinder, normal is (x, y, 0)
  if ((r - p_r) <= tol) {
    Eigen::Vector3<T> normal(p.x(), p.y(), 0);
    return normal.normalized();
  } else {
    // Check if point is either on top or bottom
    if (util::Abs(h_l - p_z_abs) <= tol) {
      Eigen::Vector3<T> normal(0.0, 0.0, p.z() > 0 ? 1.0 : -1.0);
      return normal;
    }
  }

  return std::nullopt;
}

template <typename T>
std::optional<Eigen::Vector3<T>> GetNormalAtPointForEllipsoid(
    const Ellipsoid& ellipsoid, const Eigen::Vector3<T>& p) {
  const T tol = 1e-5;
  const double a = ellipsoid.a();
  const double b = ellipsoid.b();
  const double c = ellipsoid.c();
  // If a point p = (x, y, z) is on the surface of the ellipsoid, it
  // satisfies the function f(x, y, z):
  //       f(x, y, z) = x²/a² + y²/b² + z²/c² - 1 = 0,
  // And the normal vector is given by the gradient of
  const T eq = (p.x() * p.x() / (a * a)) + (p.y() * p.y() / (b * b)) +
               (p.z() * p.z() / (c * c)) - 1.0;
  if (util::Abs(eq) < tol) {
    Eigen::Vector3<T> n(2 * p.x() / (a * a), 2 * p.y() / (b * b),
                        2 * p.z() / (c * c));
    return n.normalized();
  }
  return std::nullopt;
}

template <typename T>
std::optional<Eigen::Vector3<T>> GetNormalAtPointForMesh(
    const Mesh& mesh, const Eigen::Vector3<T>& p) {
  const double tol = 1e-5;
  const Eigen::Vector3d pd(ExtractDoubleOrThrow(p(0)),
                           ExtractDoubleOrThrow(p(1)),
                           ExtractDoubleOrThrow(p(2)));
  const auto& tri_mesh = mesh.GetSurfaceMesh();
  const auto& bvh = mesh.GetBVH();
  const auto& fns = mesh.GetFeatureNormalSet();

  internal::SquaredDistanceToTriangle closest =
      internal::CalcSquaredDistance(pd, tri_mesh, bvh, fns);
  const double unsigned_distance = std::sqrt(closest.squared_distance);

  if (std::abs(unsigned_distance) < tol) {
    Eigen::Vector3<T> normal(closest.feature_normal.x(),
                             closest.feature_normal.y(),
                             closest.feature_normal.z());
    return normal;
  }

  return std::nullopt;
}

template <typename T>
std::optional<Eigen::Vector3<T>> GetNormalAtPoint(const Shape& shape,
                                                  const Eigen::Vector3<T>& p) {
  return shape.Visit<std::optional<Eigen::Vector3<T>>>(
      overloaded{[&](const Box& box) {
                   return GetNormalAtPointForBox<T>(box, p);
                 },
                 [&](const Capsule& capsule) {
                   return GetNormalAtPointForCapsule<T>(capsule, p);
                 },
                 [&](const Convex& convex) {
                   (void)convex;
                   return Eigen::Vector3<T>(p);
                 },
                 [&](const Cylinder& cylinder) {
                   return GetNormalAtPointForCylinder(cylinder, p);
                 },
                 [&](const Ellipsoid& ellipsoid) {
                   return GetNormalAtPointForEllipsoid(ellipsoid, p);
                 },
                 [&](const HalfSpace&) {
                   return Eigen::Vector3<T>(p);
                 },
                 [&](const Mesh& mesh) {
                   return GetNormalAtPointForMesh(mesh, p);
                 },
                 [&](const MeshcatCone& cone) {
                   (void)cone;
                   return Eigen::Vector3<T>(p);
                 },
                 [&](const Sphere& sphere) {
                   return GetNormalAtPointForSphere(sphere, p);
                 }});
}

#define DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(func, shape) \
  template std::optional<Eigen::Vector3<double>> func(                        \
      const shape&, const Eigen::Vector3<double>&);                           \
  template std::optional<Eigen::Vector3<float>> func(                         \
      const shape&, const Eigen::Vector3<float>&);                            \
  template std::optional<Eigen::Vector3<::drake::AutoDiffXd>> func(           \
      const shape&, const Eigen::Vector3<::drake::AutoDiffXd>&);              \
  template std::optional<Eigen::Vector3<::drake::symbolic::Expression>> func( \
      const shape&, const Eigen::Vector3<::drake::symbolic::Expression>&);

DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(GetNormalAtPointForBox,
                                                         Box);
DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(
    GetNormalAtPointForSphere, Sphere);
DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(
    GetNormalAtPointForCapsule, Capsule);
DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(
    GetNormalAtPointForCylinder, Cylinder);
DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(
    GetNormalAtPointForEllipsoid, Ellipsoid);
DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(
    GetNormalAtPointForMesh, Mesh);
DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(GetNormalAtPoint,
                                                         Shape);

}  // namespace geometry
}  // namespace drake