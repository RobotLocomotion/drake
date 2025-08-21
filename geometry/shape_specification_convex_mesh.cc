#include "geometry/shape_specification_convex_mesh.h"


#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/make_convex_hull_mesh_impl.h"
#include "drake/geometry/proximity/meshing_utilities.h"
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
    const MeshSource& mesh_source, const Vector3<double>& scale) {
  // TODO(jwnimmer-tri) Once we drop support for Jammy (i.e., once we can use
  // GCC >= 12 as our minimum), then we should respell these atomics to use the
  // C++20 syntax and remove the warning suppressions here and below. (We need
  // the warning supression because newer Clang complains.)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::shared_ptr<PolygonSurfaceMesh<double>> check =
      std::atomic_load(hull_ptr);
#pragma GCC diagnostic pop
  if (check == nullptr) {
    // Note: This approach means that multiple threads *may* redundantly compute
    // the convex hull; but only the first one will set the hull.
    auto new_hull = std::make_shared<PolygonSurfaceMesh<double>>(
        internal::MakeConvexHull(mesh_source, scale));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    std::atomic_compare_exchange_strong(hull_ptr, &check, new_hull);
#pragma GCC diagnostic pop
  }
}

// Support for Mesh and Convex do_to_string(). It does the hard work of
// converting the MeshSource into the appropriate parameter name and value
// representation and then packages it into the full Mesh string representation.
std::string MeshToString(std::string_view class_name, const MeshSource& source,
                         const Vector3<double>& scale) {
  const std::string mesh_parameter = [&source]() {
    if (source.is_path()) {
      return fmt::format("filename='{}'", source.path().string());
    } else {
      return fmt::format("mesh_data={}", source.in_memory());
    }
  }();
  return fmt::format("{}({}, scale=[{}])", class_name, mesh_parameter,
                     fmt_eigen(scale.transpose()));
}

void ThrowForBadScale(const Vector3<double>& scale, std::string_view source) {
  if ((scale.array().abs() < 1e-8).any()) {
    throw std::logic_error(
        fmt::format("{} |scale| cannot be < 1e-8 on any axis, given [{}].",
                    source, fmt_eigen(scale.transpose())));
  }
}

}  // namespace

double CalcMeshVolume(const Mesh& mesh) {
  // TODO(russt): Support .vtk files.
  const MeshSource& mesh_source = mesh.source();
  if (mesh_source.extension() != ".obj") {
    throw std::runtime_error(fmt::format(
        "CalcVolume currently only supports .obj files for mesh geometries; "
        "but the volume of '{}' was requested.",
        mesh_source.description()));
  }
  TriangleSurfaceMesh<double> surface_mesh =
      ReadObjToTriangleSurfaceMesh(mesh_source, mesh.scale3());
  return internal::CalcEnclosedVolume(surface_mesh);
}

double CalcConvexVolume(const Convex& convex) {
  return internal::CalcEnclosedVolume(
      internal::MakeTriangleFromPolygonMesh(convex.GetConvexHull()));
}

namespace {
std::string PointsToObjString(const Eigen::Matrix3X<double>& points) {
  std::string result = "";

  for (int i = 0; i < points.cols(); ++i) {
    result +=
        fmt::format("v {} {} {}\n", points(0, i), points(1, i), points(2, i));
  }

  return result;
}

}  // namespace

void ShapeReifier::ImplementGeometry(const Convex& convex, void*) {
  DefaultImplementGeometry(convex);
}

void ShapeReifier::ImplementGeometry(const Mesh& mesh, void*) {
  DefaultImplementGeometry(mesh);
}

Convex::Convex(const std::filesystem::path& filename, double scale)
    : Convex(filename, Vector3<double>::Constant(scale)) {}

Convex::Convex(const std::filesystem::path& filename,
               const Vector3<double>& scale3)
    : Convex(MeshSource(std::filesystem::absolute(filename)), scale3) {}

Convex::Convex(InMemoryMesh mesh_data, double scale)
    : Convex(std::move(mesh_data), Vector3<double>::Constant(scale)) {}

Convex::Convex(InMemoryMesh mesh_data, const Vector3<double>& scale3)
    : Convex(MeshSource(std::move(mesh_data)), scale3) {}

Convex::Convex(MeshSource source, double scale)
    : Convex(std::move(source), Vector3<double>::Constant(scale)) {}

Convex::Convex(MeshSource source, const Vector3<double>& scale3)
    : source_(std::move(source)), scale_(scale3) {
  // Note: We don't validate extensions because there's a possibility that a
  // mesh of unsupported type is used, but only processed by client code.
  ThrowForBadScale(scale_, "Convex");
}

Convex::Convex(const Eigen::Matrix3X<double>& points, const std::string& label,
               double scale)
    : Convex(points, label, Vector3<double>::Constant(scale)) {}

Convex::Convex(const Eigen::Matrix3X<double>& points, const std::string& label,
               const Vector3<double>& scale3)
    : Convex(InMemoryMesh{.mesh_file = MemoryFile(PointsToObjString(points),
                                                  ".obj", label)},
             scale3) {}

double Convex::scale() const {
  if ((scale_.array() != scale_[0]).any()) {
    throw std::runtime_error(
        fmt::format("Convex::scale() can only be called for uniform scaling. "
                    "This mesh has scale [{}]. Use Convex::scale3() instead.",
                    fmt_eigen(scale_.transpose())));
  }
  return scale_[0];
}

const PolygonSurfaceMesh<double>& Convex::GetConvexHull() const {
  ComputeConvexHullAsNecessary(&hull_, source_, scale_);
  return *hull_;
}

std::string Convex::do_to_string() const {
  return MeshToString(type_name(), source(), scale_);
}

const geometry::TriangleSurfaceMesh<double>& Convex::GetSurfaceMesh() const {
  if (tri_mesh_ == nullptr) {
    const geometry::TriangleSurfaceMesh<double> surface =
        ReadObjToTriangleSurfaceMesh(source(), scale3());
    tri_mesh_ =
        std::make_shared<geometry::TriangleSurfaceMesh<double>>(surface);
  }
  return *tri_mesh_;
}

const geometry::internal::FeatureNormalSet& Convex::GetFeatureNormalSet()
    const {
  if (feature_normal_set_ == nullptr) {
    feature_normal_set_ =
        std::make_shared<geometry::internal::FeatureNormalSet>(
            std::get<geometry::internal::FeatureNormalSet>(
                geometry::internal::FeatureNormalSet::MaybeCreate(
                    GetSurfaceMesh())));
  }
  return *feature_normal_set_;
}

const geometry::internal::Bvh<geometry::Obb,
                              geometry::TriangleSurfaceMesh<double>>&
Convex::GetBVH() const {
  if (tri_bvh_ == nullptr) {
    tri_bvh_ = std::make_shared<geometry::internal::Bvh<
        geometry::Obb, geometry::TriangleSurfaceMesh<double>>>(
        GetSurfaceMesh());
  }
  return *tri_bvh_;
}

Mesh::Mesh(const std::filesystem::path& filename, double scale)
    : Mesh(filename, Vector3<double>::Constant(scale)) {}

Mesh::Mesh(const std::filesystem::path& filename, const Vector3<double>& scale3)
    : Mesh(MeshSource(std::filesystem::absolute(filename)), scale3) {}

Mesh::Mesh(InMemoryMesh mesh_data, double scale)
    : Mesh(std::move(mesh_data), Vector3<double>::Constant(scale)) {}

Mesh::Mesh(InMemoryMesh mesh_data, const Vector3<double>& scale3)
    : Mesh(MeshSource(std::move(mesh_data)), scale3) {}

Mesh::Mesh(MeshSource source, double scale)
    : Mesh(std::move(source), Vector3<double>::Constant(scale)) {}

Mesh::Mesh(MeshSource source, const Vector3<double>& scale3)
    : source_(std::move(source)), scale_(scale3) {
  // Note: We don't validate extensions because there's a possibility that a
  // mesh of unsupported type is used, but only processed by client code.
  ThrowForBadScale(scale_, "Mesh");
}

double Mesh::scale() const {
  if ((scale_.array() != scale_[0]).any()) {
    throw std::runtime_error(
        fmt::format("Mesh::scale() can only be called for uniform scaling. "
                    "This mesh has scale [{}]. Use Mesh::scale3() instead.",
                    fmt_eigen(scale_.transpose())));
  }
  return scale_[0];
}

const PolygonSurfaceMesh<double>& Mesh::GetConvexHull() const {
  ComputeConvexHullAsNecessary(&hull_, source_, scale_);
  return *hull_;
}

const geometry::TriangleSurfaceMesh<double>& Mesh::GetSurfaceMesh() const {
  if (tri_mesh_ == nullptr) {
    const geometry::TriangleSurfaceMesh<double> surface =
        ReadObjToTriangleSurfaceMesh(source(), scale3());
    tri_mesh_ =
        std::make_shared<geometry::TriangleSurfaceMesh<double>>(surface);
  }
  return *tri_mesh_;
}

const geometry::internal::FeatureNormalSet& Mesh::GetFeatureNormalSet() const {
  if (feature_normal_set_ == nullptr) {
    feature_normal_set_ =
        std::make_shared<geometry::internal::FeatureNormalSet>(
            std::get<geometry::internal::FeatureNormalSet>(
                geometry::internal::FeatureNormalSet::MaybeCreate(
                    GetSurfaceMesh())));
  }
  return *feature_normal_set_;
}

const geometry::internal::Bvh<geometry::Obb,
                              geometry::TriangleSurfaceMesh<double>>&
Mesh::GetBVH() const {
  if (tri_bvh_ == nullptr) {
    tri_bvh_ = std::make_shared<geometry::internal::Bvh<
        geometry::Obb, geometry::TriangleSurfaceMesh<double>>>(
        GetSurfaceMesh());
  }
  return *tri_bvh_;
}

std::string Mesh::do_to_string() const {
  return MeshToString(type_name(), source(), scale_);
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
std::optional<Eigen::Vector3<T>> GetNormalAtPointForConvex(
    const Convex& convex, const Eigen::Vector3<T>& p) {
  const double tol = 1e-5;
  const Eigen::Vector3d pd(ExtractDoubleOrThrow(p(0)),
                           ExtractDoubleOrThrow(p(1)),
                           ExtractDoubleOrThrow(p(2)));
  const auto& tri_mesh = convex.GetSurfaceMesh();
  const auto& bvh = convex.GetBVH();
  const auto& fns = convex.GetFeatureNormalSet();

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

#define DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(func, shape) \
  template std::optional<Eigen::Vector3<double>> func(                        \
      const shape&, const Eigen::Vector3<double>&);                           \
  template std::optional<Eigen::Vector3<float>> func(                         \
      const shape&, const Eigen::Vector3<float>&);                            \
  template std::optional<Eigen::Vector3<::drake::AutoDiffXd>> func(           \
      const shape&, const Eigen::Vector3<::drake::AutoDiffXd>&);              \
  template std::optional<Eigen::Vector3<::drake::symbolic::Expression>> func( \
      const shape&, const Eigen::Vector3<::drake::symbolic::Expression>&);



DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(
    GetNormalAtPointForConvex, Convex);
DRAKE_DEFINE_FUNCTION_INSTANTIATIONS_FOR_DEFAULT_SCALARS(
    GetNormalAtPointForMesh, Mesh);




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
  std::string_view ShapeType::do_type_name() const {                    \
    return #ShapeType;                                                  \
  }                                                                     \
  Shape::VariantShapeConstPtr ShapeType::get_variant_this() const {     \
    return this;                                                        \
  }

DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(Convex)
DRAKE_DEFINE_SHAPE_SUBCLASS_BOILERPLATE(Mesh)

}  // namespace geometry
}  // namespace drake
