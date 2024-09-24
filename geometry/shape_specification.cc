#include "drake/geometry/shape_specification.h"

#include <algorithm>
#include <filesystem>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/overloaded.h"
#include "drake/geometry/proximity/make_convex_hull_mesh_impl.h"
#include "drake/geometry/proximity/meshing_utilities.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"

namespace drake {
namespace geometry {
namespace {

// Computes a convex hull and assigns it to the given shared pointer in a
// thread-safe manner. Only does work if the shared_ptr is null (i.e., there is
// no convex hull yet). Used by Mesh::GetConvexHull() and
// Convex::GetConvexHull(). Note: the correctness of this function is tested in
// shape_specification_thread_test.cc.
void ComputeConvexHullAsNecessary(
    std::shared_ptr<PolygonSurfaceMesh<double>>* hull_ptr,
    std::string_view filename, double scale) {
  std::shared_ptr<PolygonSurfaceMesh<double>> check =
      std::atomic_load(hull_ptr);
  if (check == nullptr) {
    // Note: This approach means that multiple threads *may* redundantly compute
    // the convex hull; but only the first one will set the hull.
    auto new_hull = std::make_shared<PolygonSurfaceMesh<double>>(
        internal::MakeConvexHull(filename, scale));
    std::atomic_compare_exchange_strong(hull_ptr, &check, new_hull);
  }
}

// Support for Mesh and Convex do_to_string(). It does the hard work of
// converting the MeshSource into the appropriate parameter name and value
// representation and then packages it into the full Mesh string representation.
std::string MeshToString(std::string_view class_name, const MeshSource& source,
                         double scale) {
  const std::string mesh_parameter = [&source]() {
    if (source.is_path()) {
      return fmt::format("filename='{}'", source.path().string());
    } else {
      return fmt::format("mesh_data={}", source.in_memory());
    }
  }();
  return fmt::format("{}({}, scale={})", class_name, mesh_parameter, scale);
}

void ThrowForBadScale(double scale, std::string_view source) {
  if (std::abs(scale) < 1e-8) {
    throw std::logic_error(
        fmt::format("{} |scale| cannot be < 1e-8, given {}.", source, scale));
  }
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

Convex::Convex(const std::filesystem::path& filename, double scale)
    : Convex(MeshSource(std::filesystem::absolute(filename)), scale) {}

Convex::Convex(InMemoryMesh mesh_data, double scale)
    : Convex(MeshSource(std::move(mesh_data)), scale) {}

Convex::Convex(MeshSource source, double scale)
    : source_(std::move(source)), scale_(scale) {
  // Note: We don't validate extensions because there's a possibility that a
  // mesh of unsupported type is used, but only processed by client code.
  ThrowForBadScale(scale, "Convex");
}

std::string Convex::filename() const {
  if (source_.is_path()) {
    return source_.path().string();
  }
  throw std::runtime_error(
      fmt::format("Convex::filename() cannot be called when constructed on "
                  "in-memory mesh data: '{}'. Call Convex::source().path() "
                  "instead.",
                  source_.in_memory().mesh_file.filename_hint()));
}

const PolygonSurfaceMesh<double>& Convex::GetConvexHull() const {
  if (source_.is_in_memory()) {
    throw std::runtime_error(
        "In-memory meshes are still being implemented. Convex::GetConvexHull() "
        "still requires a filesystem path specification.");
  }
  ComputeConvexHullAsNecessary(&hull_, source_.path().string(), scale_);
  return *hull_;
}

std::string Convex::do_to_string() const {
  return MeshToString(type_name(), source(), scale());
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

Mesh::Mesh(const std::filesystem::path& filename, double scale)
    : Mesh(MeshSource(std::filesystem::absolute(filename)), scale) {}

Mesh::Mesh(InMemoryMesh mesh_data, double scale)
    : Mesh(MeshSource(std::move(mesh_data)), scale) {}

Mesh::Mesh(MeshSource source, double scale)
    : source_(std::move(source)), scale_(scale) {
  // Note: We don't validate extensions because there's a possibility that a
  // mesh of unsupported type is used, but only processed by client code.
  ThrowForBadScale(scale, "Mesh");
}

std::string Mesh::filename() const {
  if (source_.is_path()) {
    return source_.path().string();
  }
  throw std::runtime_error(
      fmt::format("Mesh::filename() cannot be called when constructed on "
                  "in-memory mesh data: '{}'. Call Mesh::source().path() "
                  "instead.",
                  source_.in_memory().mesh_file.filename_hint()));
}

const PolygonSurfaceMesh<double>& Mesh::GetConvexHull() const {
  if (source_.is_in_memory()) {
    throw std::runtime_error(
        "In-memory meshes are still being implemented. Mesh::GetConvexHull() "
        "still requires a filesystem path specification.");
  }
  ComputeConvexHullAsNecessary(&hull_, source_.path().string(), scale_);
  return *hull_;
}

std::string Mesh::do_to_string() const {
  return MeshToString(type_name(), source(), scale());
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

namespace {

double CalcMeshVolume(const Mesh& mesh) {
  // TODO(russt): Support .vtk files.
  if (mesh.extension() != ".obj") {
    throw std::runtime_error(fmt::format(
        "CalcVolume currently only supports .obj files for mesh geometries; "
        "but the volume of '{}' was requested.",
        mesh.filename()));
  }
  TriangleSurfaceMesh<double> surface_mesh =
      ReadObjToTriangleSurfaceMesh(mesh.filename(), mesh.scale());
  return internal::CalcEnclosedVolume(surface_mesh);
}

}  // namespace

double CalcVolume(const Shape& shape) {
  return shape.Visit<double>(overloaded{
      [](const Box& box) {
        return box.width() * box.depth() * box.height();
      },
      [](const Capsule& capsule) {
        return M_PI * std::pow(capsule.radius(), 2) * capsule.length() +
               4.0 / 3.0 * M_PI * std::pow(capsule.radius(), 3);
      },
      [](const Convex& convex) {
        return internal::CalcEnclosedVolume(
            internal::MakeTriangleFromPolygonMesh(convex.GetConvexHull()));
      },
      [](const Cylinder& cylinder) {
        return M_PI * std::pow(cylinder.radius(), 2) * cylinder.length();
      },
      [](const Ellipsoid& ellipsoid) {
        return 4.0 / 3.0 * M_PI * ellipsoid.a() * ellipsoid.b() * ellipsoid.c();
      },
      [](const HalfSpace&) {
        return std::numeric_limits<double>::infinity();
      },
      [](const Mesh& mesh) {
        return CalcMeshVolume(mesh);
      },
      [](const MeshcatCone& cone) {
        return 1.0 / 3.0 * M_PI * cone.a() * cone.b() * cone.height();
      },
      [](const Sphere& sphere) {
        return 4.0 / 3.0 * M_PI * std::pow(sphere.radius(), 3);
      }});
}

// The NVI function definitions are enough boilerplate to merit a macro to
// implement them, and we might as well toss in the dtor for good measure.

#define DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(ShapeType)                \
  ShapeType::~ShapeType() = default;                                      \
  void ShapeType::DoReify(ShapeReifier* shape_reifier, void* user_data)   \
      const {                                                             \
    shape_reifier->ImplementGeometry(*this, user_data);                   \
  }                                                                       \
  std::unique_ptr<Shape> ShapeType::DoClone() const {                     \
    return std::unique_ptr<ShapeType>(new ShapeType(*this));              \
  }                                                                       \
  std::string_view ShapeType::do_type_name() const { return #ShapeType; } \
  Shape::VariantShapeConstPtr ShapeType::get_variant_this() const {       \
    return this;                                                          \
  }

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
