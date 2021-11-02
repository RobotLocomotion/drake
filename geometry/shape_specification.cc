#include "drake/geometry/shape_specification.h"

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace geometry {

using math::RigidTransform;

Shape::~Shape() {}

void Shape::Reify(ShapeReifier* reifier, void* user_data) const {
  reifier_(*this, reifier, user_data); }

std::unique_ptr<Shape> Shape::Clone() const { return cloner_(*this); }

Sphere::Sphere(double radius)
    : Shape(ShapeTag<Sphere>()), radius_(radius) {
  if (radius < 0) {
    throw std::logic_error(
        fmt::format("Sphere radius should be >= 0 (was {}).", radius));
  }
}

Cylinder::Cylinder(double radius, double length)
    : Shape(ShapeTag<Cylinder>()),
      radius_(radius),
      length_(length) {
  if (radius <= 0 || length <= 0) {
    throw std::logic_error(
        fmt::format("Cylinder radius and length should both be > 0 (were {} "
                    "and {}, respectively).",
                    radius, length));
  }
}

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
      size_(width, depth, height) {
  if (width <= 0 || depth <= 0 || height <= 0) {
    throw std::logic_error(
        fmt::format("Box width, depth, and height should all be > 0 (were {}, "
                    "{}, and {}, respectively).",
                    width, depth, height));
  }
}

Box Box::MakeCube(double edge_size) {
  return Box(edge_size, edge_size, edge_size);
}

Capsule::Capsule(double radius, double length)
    : Shape(ShapeTag<Capsule>()), radius_(radius), length_(length) {
  if (radius <= 0 || length <= 0) {
    throw std::logic_error(
        fmt::format("Capsule radius and length should both be > 0 (were {} "
                    "and {}, respectively).",
                    radius, length));
  }
}

Ellipsoid::Ellipsoid(double a, double b, double c)
    : Shape(ShapeTag<Ellipsoid>()), radii_(a, b, c) {
  if (a <= 0 || b <= 0 || c <= 0) {
    throw std::logic_error(
        fmt::format("Ellipsoid lengths of principal semi-axes a, b, and c "
                    "should all be > 0 (were {}, {}, and {}, respectively).",
                    a, b, c));
  }
}

Mesh::Mesh(const std::string& absolute_filename, double scale)
    : Shape(ShapeTag<Mesh>()), filename_(absolute_filename), scale_(scale) {
  if (std::abs(scale) < 1e-8) {
    throw std::logic_error("Mesh |scale| cannot be < 1e-8.");
  }
}

Convex::Convex(const std::string& absolute_filename, double scale)
    : Shape(ShapeTag<Convex>()), filename_(absolute_filename), scale_(scale) {
  if (std::abs(scale) < 1e-8) {
    throw std::logic_error("Convex |scale| cannot be < 1e-8.");
  }
}

MeshcatCone::MeshcatCone(double height, double a, double b)
    : Shape(ShapeTag<MeshcatCone>()), height_(height), a_(a), b_(b) {
  if (height <= 0 || a <= 0 || b <= 0) {
    throw std::logic_error(fmt::format(
        "MeshcatCone parameters height, a, and b should all be > 0 (they were "
        "{}, {}, and {}, respectively).",
        height, a, b));
  }
}

void ShapeReifier::ImplementGeometry(const Sphere&, void*) {
  ThrowUnsupportedGeometry("Sphere");
}

void ShapeReifier::ImplementGeometry(const Cylinder&, void*) {
  ThrowUnsupportedGeometry("Cylinder");
}

void ShapeReifier::ImplementGeometry(const HalfSpace&, void*) {
  ThrowUnsupportedGeometry("HalfSpace");
}

void ShapeReifier::ImplementGeometry(const Box&, void*) {
  ThrowUnsupportedGeometry("Box");
}

void ShapeReifier::ImplementGeometry(const Capsule&, void*) {
  ThrowUnsupportedGeometry("Capsule");
}

void ShapeReifier::ImplementGeometry(const Ellipsoid&, void*) {
  ThrowUnsupportedGeometry("Ellipsoid");
}
void ShapeReifier::ImplementGeometry(const Mesh&, void*) {
  ThrowUnsupportedGeometry("Mesh");
}

void ShapeReifier::ImplementGeometry(const Convex&, void*) {
  ThrowUnsupportedGeometry("Convex");
}

void ShapeReifier::ImplementGeometry(const MeshcatCone&, void*) {
  ThrowUnsupportedGeometry("MeshcatCone");
}

void ShapeReifier::ThrowUnsupportedGeometry(const std::string& shape_name) {
  throw std::runtime_error(fmt::format("This class ({}) does not support {}.",
                                       NiceTypeName::Get(*this), shape_name));
}

std::ostream& operator<<(std::ostream& out, const ShapeName& name) {
  out << name.name();
  return out;
}

}  // namespace geometry
}  // namespace drake
