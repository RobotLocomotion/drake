#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

using math::RigidTransform;

Shape::~Shape() {}

void Shape::Reify(ShapeReifier* reifier, void* user_data) const {
  reifier_(*this, reifier, user_data); }

std::unique_ptr<Shape> Shape::Clone() const { return cloner_(*this); }

Sphere::Sphere(double radius)
    : Shape(ShapeTag<Sphere>()), radius_(radius) {}

Cylinder::Cylinder(double radius, double length)
    : Shape(ShapeTag<Cylinder>()),
      radius_(radius),
      length_(length) {}

HalfSpace::HalfSpace() : Shape(ShapeTag<HalfSpace>()) {}

RigidTransform<double> HalfSpace::MakePose(const Vector3<double>& Hz_dir_F,
                                           const Vector3<double>& p_FB) {
  const double norm = Hz_dir_F.norm();
  // Note: this value of epsilon is somewhat arbitrary. It's merely a minor
  // fence over which ridiculous vectors will trip.
  if (norm < 1e-10) {
    throw std::logic_error("Can't make pose from a zero vector.");
  }

  // First create basis.
  // Projects the normal into the first quadrant in order to identify the
  // *smallest* component of the normal.
  const Vector3<double> u(Hz_dir_F.cwiseAbs());
  int min_axis;
  u.minCoeff(&min_axis);
  // The axis corresponding to the smallest component of the normal will be
  // *most* perpendicular.
  Vector3<double> perp_axis{0, 0, 0};
  perp_axis(min_axis) = 1;
  // Now define x-, y-, and z-axes. The z-axis lies in the given direction.
  Vector3<double> Hz_F = Hz_dir_F / norm;
  Vector3<double> Hx_F = Hz_F.cross(perp_axis).normalized();
  Vector3<double> Hy_F = Hz_F.cross(Hx_F);

  // Transformation from canonical frame C to target frame F.
  const auto R_FH =
      math::RotationMatrixd::MakeFromOrthonormalColumns(Hx_F, Hy_F, Hz_F);
  const Vector3<double> p_FH = Hz_F.dot(p_FB) * Hz_F;
  return RigidTransform<double>(R_FH, p_FH);
}

Box::Box(double width, double depth, double height)
    : Shape(ShapeTag<Box>()),
      size_(width, depth, height) {}

Box Box::MakeCube(double edge_size) {
  return Box(edge_size, edge_size, edge_size);
}

Mesh::Mesh(const std::string& absolute_filename, double scale)
    : Shape(ShapeTag<Mesh>()), filename_(absolute_filename), scale_(scale) {}

Convex::Convex(const std::string& absolute_filename, double scale)
    : Shape(ShapeTag<Convex>()), filename_(absolute_filename), scale_(scale) {}

}  // namespace geometry
}  // namespace drake
