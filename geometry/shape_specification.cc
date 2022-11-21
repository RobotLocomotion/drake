#include "drake/geometry/shape_specification.h"

#include <algorithm>
#include <filesystem>
#include <limits>

#include <fmt/format.h>

#include "drake/common/nice_type_name.h"
#include "drake/geometry/proximity/meshing_utilities.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {

using math::RigidTransform;

Shape::~Shape() {}

void Shape::Reify(ShapeReifier* reifier, void* user_data) const {
  reifier_(*this, reifier, user_data);
}

std::unique_ptr<Shape> Shape::Clone() const { return cloner_(*this); }

template <typename S>
Shape::Shape(ShapeTag<S>) {
  static_assert(std::is_base_of_v<Shape, S>,
                "Concrete shapes *must* be derived from the Shape class");
  cloner_ = [](const Shape& shape_arg) {
    DRAKE_DEMAND(typeid(shape_arg) == typeid(S));
    const S& derived_shape = static_cast<const S&>(shape_arg);
    return std::unique_ptr<Shape>(new S(derived_shape));
  };
  reifier_ = [](const Shape& shape_arg, ShapeReifier* reifier,
                void* user_data) {
    DRAKE_DEMAND(typeid(shape_arg) == typeid(S));
    const S& derived_shape = static_cast<const S&>(shape_arg);
    reifier->ImplementGeometry(derived_shape, user_data);
  };
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

Box::Box(const Vector3<double>& measures)
    : Box(measures(0), measures(1), measures(2)) {}

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

Capsule::Capsule(const Vector2<double>& measures)
    : Capsule(measures(0), measures(1)) {}

Convex::Convex(const std::string& absolute_filename, double scale)
    : Shape(ShapeTag<Convex>()), filename_(absolute_filename), scale_(scale) {
  if (std::abs(scale) < 1e-8) {
    throw std::logic_error("Convex |scale| cannot be < 1e-8.");
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

Cylinder::Cylinder(const Vector2<double>& measures)
    : Cylinder(measures(0), measures(1)) {}

Ellipsoid::Ellipsoid(double a, double b, double c)
    : Shape(ShapeTag<Ellipsoid>()), radii_(a, b, c) {
  if (a <= 0 || b <= 0 || c <= 0) {
    throw std::logic_error(
        fmt::format("Ellipsoid lengths of principal semi-axes a, b, and c "
                    "should all be > 0 (were {}, {}, and {}, respectively).",
                    a, b, c));
  }
}

Ellipsoid::Ellipsoid(const Vector3<double>& measures)
    : Ellipsoid(measures(0), measures(1), measures(2)) {}

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

Mesh::Mesh(const std::string& absolute_filename, double scale)
    : Shape(ShapeTag<Mesh>()), filename_(absolute_filename), scale_(scale) {
  if (std::abs(scale) < 1e-8) {
    throw std::logic_error("Mesh |scale| cannot be < 1e-8.");
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

MeshcatCone::MeshcatCone(const Vector3<double>& measures)
    : MeshcatCone(measures(0), measures(1), measures(2)) {}

Sphere::Sphere(double radius)
    : Shape(ShapeTag<Sphere>()), radius_(radius) {
  if (radius < 0) {
    throw std::logic_error(
        fmt::format("Sphere radius should be >= 0 (was {}).", radius));
  }
}

ShapeReifier::~ShapeReifier() = default;

void ShapeReifier::ImplementGeometry(const Box&, void*) {
  ThrowUnsupportedGeometry("Box");
}

void ShapeReifier::ImplementGeometry(const Capsule&, void*) {
  ThrowUnsupportedGeometry("Capsule");
}

void ShapeReifier::ImplementGeometry(const Convex&, void*) {
  ThrowUnsupportedGeometry("Convex");
}

void ShapeReifier::ImplementGeometry(const Cylinder&, void*) {
  ThrowUnsupportedGeometry("Cylinder");
}

void ShapeReifier::ImplementGeometry(const Ellipsoid&, void*) {
  ThrowUnsupportedGeometry("Ellipsoid");
}

void ShapeReifier::ImplementGeometry(const HalfSpace&, void*) {
  ThrowUnsupportedGeometry("HalfSpace");
}

void ShapeReifier::ImplementGeometry(const Mesh&, void*) {
  ThrowUnsupportedGeometry("Mesh");
}

void ShapeReifier::ImplementGeometry(const MeshcatCone&, void*) {
  ThrowUnsupportedGeometry("MeshcatCone");
}

void ShapeReifier::ImplementGeometry(const Sphere&, void*) {
  ThrowUnsupportedGeometry("Sphere");
}

void ShapeReifier::ThrowUnsupportedGeometry(const std::string& shape_name) {
  throw std::runtime_error(fmt::format("This class ({}) does not support {}.",
                                       NiceTypeName::Get(*this), shape_name));
}

ShapeName::ShapeName(const Shape& shape) {
  shape.Reify(this);
}

ShapeName::~ShapeName() = default;

void ShapeName::ImplementGeometry(const Box&, void*) {
  string_ = "Box";
}

void ShapeName::ImplementGeometry(const Capsule&, void*) {
  string_ = "Capsule";
}

void ShapeName::ImplementGeometry(const Convex&, void*) {
  string_ = "Convex";
}

void ShapeName::ImplementGeometry(const Cylinder&, void*) {
  string_ = "Cylinder";
}

void ShapeName::ImplementGeometry(const Ellipsoid&, void*) {
  string_ = "Ellipsoid";
}

void ShapeName::ImplementGeometry(const HalfSpace&, void*) {
  string_ = "HalfSpace";
}

void ShapeName::ImplementGeometry(const Mesh&, void*) {
  string_ = "Mesh";
}

void ShapeName::ImplementGeometry(const MeshcatCone&, void*) {
  string_ = "MeshcatCone";
}

void ShapeName::ImplementGeometry(const Sphere&, void*) {
  string_ = "Sphere";
}

std::ostream& operator<<(std::ostream& out, const ShapeName& name) {
  out << name.name();
  return out;
}

namespace {

template <class MeshType>
double CalcMeshVolumeFromFile(const MeshType& mesh) {
  std::string extension = std::filesystem::path(mesh.filename()).extension();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  // TODO(russt): Support .vtk files.
  if (extension != ".obj") {
    throw std::runtime_error(fmt::format(
        "CalcVolume currently only supports .obj files for mesh geometries; "
        "but the volume of {} was requested.",
        mesh.filename()));
  }
  TriangleSurfaceMesh<double> surface_mesh =
      ReadObjToTriangleSurfaceMesh(mesh.filename(), mesh.scale());
  return internal::CalcEnclosedVolume(surface_mesh);
}

class CalcVolumeReifier final : public ShapeReifier {
 public:
  CalcVolumeReifier() = default;

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box& box, void*) final {
    volume_ = box.width() * box.depth() * box.height();
  }
  void ImplementGeometry(const Capsule& capsule, void*) final {
    volume_ = M_PI * std::pow(capsule.radius(), 2) * capsule.length() +
         4.0 / 3.0 * M_PI * std::pow(capsule.radius(), 3);
  }
  void ImplementGeometry(const Convex& mesh, void*) {
    volume_ = CalcMeshVolumeFromFile(mesh);
  }
  void ImplementGeometry(const Cylinder& cylinder, void*) final {
    volume_ = M_PI * std::pow(cylinder.radius(), 2) * cylinder.length();
  }
  void ImplementGeometry(const Ellipsoid& ellipsoid, void*) final {
    volume_ = 4.0 / 3.0 * M_PI * ellipsoid.a() * ellipsoid.b() * ellipsoid.c();
  }
  void ImplementGeometry(const HalfSpace&, void*) final {
    volume_ = std::numeric_limits<double>::infinity();
  }
  void ImplementGeometry(const Mesh& mesh, void*) {
    volume_ = CalcMeshVolumeFromFile(mesh);
  }
  void ImplementGeometry(const MeshcatCone& cone, void*) final {
    volume_ = 1.0 / 3.0 * M_PI * cone.a() * cone.b() * cone.height();
  }
  void ImplementGeometry(const Sphere& sphere, void*) final {
    volume_ = 4.0 / 3.0 * M_PI * std::pow(sphere.radius(), 3);
  }

  double volume() const { return volume_; }

 private:
  double volume_{0.0};
};

}  // namespace

double CalcVolume(const Shape& shape) {
  CalcVolumeReifier reifier;
  shape.Reify(&reifier);
  return reifier.volume();
}

}  // namespace geometry
}  // namespace drake
