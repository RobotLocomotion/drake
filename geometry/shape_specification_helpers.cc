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

#define DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(func)        \
  template std::optional<Eigen::Vector3<double>> func(                        \
      const Box&, const Eigen::Vector3<double>&);                             \
  template std::optional<Eigen::Vector3<float>> func(                         \
      const Box&, const Eigen::Vector3<float>&);                              \
  template std::optional<Eigen::Vector3<::drake::AutoDiffXd>> func(           \
      const Box&, const Eigen::Vector3<::drake::AutoDiffXd>&);                \
  template std::optional<Eigen::Vector3<::drake::symbolic::Expression>> func( \
      const Box&, const Eigen::Vector3<::drake::symbolic::Expression>&);

DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(
    GetNormalAtPointForBox);

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
                   (void)sphere;
                   return Eigen::Vector3<T>(p);
                 }});
}

template std::optional<Eigen::Vector3<double>> GetNormalAtPoint(
    const Shape& shape, const Eigen::Vector3<double>& p);
template std::optional<Eigen::Vector3<float>> GetNormalAtPoint(
    const Shape& shape, const Eigen::Vector3<float>& p);
template std::optional<Eigen::Vector3<::drake::AutoDiffXd>> GetNormalAtPoint(
    const Shape& shape, const Eigen::Vector3<::drake::AutoDiffXd>& p);
template std::optional<Eigen::Vector3<::drake::symbolic::Expression>>
GetNormalAtPoint(const Shape& shape,
                 const Eigen::Vector3<::drake::symbolic::Expression>& p);

}  // namespace geometry
}  // namespace drake