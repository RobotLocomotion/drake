#include "drake/common/overloaded.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

template <typename T>
std::optional<Eigen::Vector3<T>> GetNormalAtPointForBox(
    const Box& box, const Eigen::Vector3<T>& p) {
  const T tol = 1e-5;

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
  const T tol = 1e-5;
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
  const T tol = 1e-5;

  const T h_l = capsule.length() / 2.0;  // half length
  const T r = capsule.radius();

  // Point is not on surface if is z coordinate is larger than
  // the total height of the cylinder (half length + radius)
  const T z_diff = p.z() - (h_l + r);
  if ((z_diff * z_diff) > tol) {
    return std::nullopt;
  }

  // Also is not on surface if the norm of the (x,y) vector is larger
  // than the radius
  const T r_diff = p.head(2).norm() - r;
  if ((r * r) > tol) {
    return std::nullopt;
  }

  // If the point is on the cylindrical section of the capsule, its z
  // coordinate must be such that -half_length < z < half_length.
  // Then the normal vector is defined by the x and y coordinates of the
  // point and constrained to the x-y plane, i.e : (x, y, 0)
  if (p.z().abs() < h_l) {
    const T diff = p.head(2).norm() - r;
    if ((diff * diff) < tol) {
      Eigen::Vector3<T> n(p.x(), p.y(), 0);
      return n.normalized();
    }
  }

  // Check if the point is in any of the semispherical sections
  Eigen::Vector3<T> https
      :  // chatgpt.com/share/689df8f1-eca0-8006-be9d-445702bea040

         return std::nullopt;
}

template <typename T>
std::optional<Eigen::Vector3<T>> GetNormalAtPoint(const Shape& shape,
                                                  const Eigen::Vector3<T>& p) {
  return shape.Visit<std::optional<Eigen::Vector3<T>>>(
      overloaded{[&](const Box& box) {
                   (void)box;
                   return GetNormalAtPointForBox<T>(box, p);
                 },
                 [&](const Capsule& capsule) {
                   (void)capsule;
                   return Eigen::Vector3<T>(p);
                 },
                 [&](const Convex& convex) {
                   (void)convex;
                   return Eigen::Vector3<T>(p);
                 },
                 [&](const Cylinder& cylinder) {
                   (void)cylinder;
                   return Eigen::Vector3<T>(p);
                 },
                 [&](const Ellipsoid& ellipsoid) {
                   (void)ellipsoid;
                   return Eigen::Vector3<T>(p);
                 },
                 [&](const HalfSpace&) {
                   return Eigen::Vector3<T>(p);
                 },
                 [&](const Mesh& mesh) {
                   (void)mesh;
                   return Eigen::Vector3<T>(p);
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
DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(GetNormalAtPoint,
                                                         Shape);

}  // namespace geometry
}  // namespace drake