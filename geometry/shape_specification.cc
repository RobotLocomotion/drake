#include "drake/geometry/shape_specification.h"

#include <algorithm>
#include <filesystem>
#include <limits>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/proximity/meshing_utilities.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace {

std::string GetExtensionLower(const std::string& filename) {
  std::filesystem::path file_path(filename);
  std::string ext = file_path.extension();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  return ext;
}

}  // namespace

using math::RigidTransform;

Shape::Shape() = default;

Shape::~Shape() = default;

void Shape::Reify(ShapeReifier* reifier, void* user_data) const {
  DRAKE_THROW_UNLESS(reifier != nullptr);
  DoReify(reifier, user_data);
}

std::unique_ptr<Shape> Shape::Clone() const {
  return DoClone();
}

Box::Box(double width, double depth, double height)
    : size_(width, depth, height) {
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

std::string Box::do_to_string() const {
  return fmt::format("Box(width={}, depth={}, height={})", width(), depth(),
                     height());
}

Capsule::Capsule(double radius, double length)
    : radius_(radius), length_(length) {
  if (radius <= 0 || length <= 0) {
    throw std::logic_error(
        fmt::format("Capsule radius and length should both be > 0 (were {} "
                    "and {}, respectively).",
                    radius, length));
  }
}

Capsule::Capsule(const Vector2<double>& measures)
    : Capsule(measures(0), measures(1)) {}

std::string Capsule::do_to_string() const {
  return fmt::format("Capsule(radius={}, length={})", radius(), length());
}

Convex::Convex(const std::string& filename, double scale)
    : filename_(std::filesystem::absolute(filename)),
      extension_(GetExtensionLower(filename_)),
      scale_(scale) {
  if (std::abs(scale) < 1e-8) {
    throw std::logic_error("Convex |scale| cannot be < 1e-8.");
  }
}

std::string Convex::do_to_string() const {
  return fmt::format("Convex(filename='{}', scale={})", filename(), scale());
}

Cylinder::Cylinder(double radius, double length)
    : radius_(radius), length_(length) {
  if (radius <= 0 || length <= 0) {
    throw std::logic_error(
        fmt::format("Cylinder radius and length should both be > 0 (were {} "
                    "and {}, respectively).",
                    radius, length));
  }
}

Cylinder::Cylinder(const Vector2<double>& measures)
    : Cylinder(measures(0), measures(1)) {}

std::string Cylinder::do_to_string() const {
  return fmt::format("Cylinder(radius={}, length={})", radius(), length());
}

Ellipsoid::Ellipsoid(double a, double b, double c) : radii_(a, b, c) {
  if (a <= 0 || b <= 0 || c <= 0) {
    throw std::logic_error(
        fmt::format("Ellipsoid lengths of principal semi-axes a, b, and c "
                    "should all be > 0 (were {}, {}, and {}, respectively).",
                    a, b, c));
  }
}

Ellipsoid::Ellipsoid(const Vector3<double>& measures)
    : Ellipsoid(measures(0), measures(1), measures(2)) {}

std::string Ellipsoid::do_to_string() const {
  return fmt::format("Ellipsoid(a={}, b={}, c={})", a(), b(), c());
}

HalfSpace::HalfSpace() = default;

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

std::string HalfSpace::do_to_string() const {
  return "HalfSpace()";
}

Mesh::Mesh(const std::string& filename, double scale)
    : filename_(std::filesystem::absolute(filename)),
      extension_(GetExtensionLower(filename_)),
      scale_(scale) {
  if (std::abs(scale) < 1e-8) {
    throw std::logic_error("Mesh |scale| cannot be < 1e-8.");
  }
}

std::string Mesh::do_to_string() const {
  return fmt::format("Mesh(filename='{}', scale={})", filename(), scale());
}

MeshcatCone::MeshcatCone(double height, double a, double b)
    : height_(height), a_(a), b_(b) {
  if (height <= 0 || a <= 0 || b <= 0) {
    throw std::logic_error(fmt::format(
        "MeshcatCone parameters height, a, and b should all be > 0 (they were "
        "{}, {}, and {}, respectively).",
        height, a, b));
  }
}

MeshcatCone::MeshcatCone(const Vector3<double>& measures)
    : MeshcatCone(measures(0), measures(1), measures(2)) {}

std::string MeshcatCone::do_to_string() const {
  return fmt::format("MeshcatCone(height={}, a={}, b={})", height(), a(), b());
}

Sphere::Sphere(double radius) : radius_(radius) {
  if (radius < 0) {
    throw std::logic_error(
        fmt::format("Sphere radius should be >= 0 (was {}).", radius));
  }
}

std::string Sphere::do_to_string() const {
  return fmt::format("Sphere(radius={})", radius());
}

ShapeReifier::~ShapeReifier() = default;

void ShapeReifier::ImplementGeometry(const Box& box, void*) {
  DefaultImplementGeometry(box);
}

void ShapeReifier::ImplementGeometry(const Capsule& capsule, void*) {
  DefaultImplementGeometry(capsule);
}

void ShapeReifier::ImplementGeometry(const Convex& convex, void*) {
  DefaultImplementGeometry(convex);
}

void ShapeReifier::ImplementGeometry(const Cylinder& cylinder, void*) {
  DefaultImplementGeometry(cylinder);
}

void ShapeReifier::ImplementGeometry(const Ellipsoid& ellipsoid, void*) {
  DefaultImplementGeometry(ellipsoid);
}

void ShapeReifier::ImplementGeometry(const HalfSpace& hs, void*) {
  DefaultImplementGeometry(hs);
}

void ShapeReifier::ImplementGeometry(const Mesh& mesh, void*) {
  DefaultImplementGeometry(mesh);
}

void ShapeReifier::ImplementGeometry(const MeshcatCone& cone, void*) {
  DefaultImplementGeometry(cone);
}

void ShapeReifier::ImplementGeometry(const Sphere& sphere, void*) {
  DefaultImplementGeometry(sphere);
}

void ShapeReifier::DefaultImplementGeometry(const Shape& shape) {
  ThrowUnsupportedGeometry(std::string{shape.type_name()});
}

void ShapeReifier::ThrowUnsupportedGeometry(const std::string& shape_name) {
  throw std::runtime_error(fmt::format("This class ({}) does not support {}.",
                                       NiceTypeName::Get(*this), shape_name));
}

ShapeName::ShapeName(const Shape& shape) {
  shape.Reify(this);
}

ShapeName::~ShapeName() = default;

void ShapeName::DefaultImplementGeometry(const Shape& shape) {
  string_ = shape.type_name();
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
std::ostream& operator<<(std::ostream& out, const ShapeName& name) {
  out << name.name();
  return out;
}
#pragma GCC diagnostic pop

namespace {

template <class MeshType>
double CalcMeshVolumeFromFile(const MeshType& mesh) {
  std::string extension = std::filesystem::path(mesh.filename()).extension();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) {
                   return std::tolower(c);
                 });
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

// The NVI function definitions are enough boilerplate to merit a macro to
// implement them, and we might as well toss in the dtor for good measure.

#define DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(ShapeType)              \
  ShapeType::~ShapeType() = default;                                    \
  void ShapeType::DoReify(ShapeReifier* shape_reifier, void* user_data) \
      const {                                                           \
    shape_reifier->ImplementGeometry(*this, user_data);                 \
  }                                                                     \
  std::unique_ptr<Shape> ShapeType::DoClone() const {                   \
    return std::unique_ptr<ShapeType>(new ShapeType(*this));            \
  }                                                                     \
  std::string_view ShapeType::do_type_name() const { return #ShapeType; }

DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(Box)
DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(Capsule)
DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(Convex)
DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(Cylinder)
DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(Ellipsoid)
DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(HalfSpace)
DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(Mesh)
DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(MeshcatCone)
DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(Sphere)

#undef DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE

}  // namespace geometry
}  // namespace drake
