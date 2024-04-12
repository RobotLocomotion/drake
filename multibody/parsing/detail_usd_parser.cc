#include "drake/multibody/parsing/detail_usd_parser.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pxr/base/gf/transform.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"
#include "pxr/usd/usdGeom/capsule.h"
#include "pxr/usd/usdGeom/cone.h"
#include "pxr/usd/usdGeom/cube.h"
#include "pxr/usd/usdGeom/cylinder.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/sphere.h"
#include "pxr/usd/usdGeom/xformable.h"
#include <fmt/format.h>

#include "drake/common/find_runfiles.h"
#include "drake/common/unused.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/detail_common.h"

namespace drake {
namespace multibody {
namespace internal {

namespace fs = std::filesystem;
#if 0
namespace pxr = drake_vendor_pxr;
#endif

struct UsdStageMetadata {
    double meters_per_unit = 1.0;
    pxr::TfToken up_axis = pxr::TfToken("Z");
};

class UsdParser {
 public:
  explicit UsdParser(const ParsingWorkspace& ws);
  ~UsdParser() = default;
  std::vector<ModelInstanceIndex> AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace);

 private:
  UsdStageMetadata GetStageMetadata(const pxr::UsdStageRefPtr stage);
  void ProcessPrim(const pxr::UsdPrim& prim);
  void ProcessStaticCollider(const pxr::UsdPrim& prim);
  std::unique_ptr<geometry::Shape> CreateCollisionGeometry(
    const pxr::UsdPrim& prim);
  std::unique_ptr<geometry::Shape> CreateVisualGeometry(
    const pxr::UsdPrim& prim);
  std::unique_ptr<geometry::Shape> CreateGeometryMesh(
    const pxr::UsdPrim& prim);
  std::unique_ptr<geometry::Shape> CreateGeometryCylinder(
    const pxr::UsdPrim& prim);
  std::unique_ptr<geometry::Shape> CreateGeometryCapsule(
    const pxr::UsdPrim& prim);
  std::unique_ptr<geometry::Shape> CreateGeometrySphere(
    const pxr::UsdPrim& prim);
  std::unique_ptr<geometry::Shape> CreateGeometryCube(
    const pxr::UsdPrim& prim);
  std::string WriteMeshToObjFile(
    const pxr::VtArray<pxr::GfVec3f>& vertices,
    const pxr::VtArray<int>& indices);
  void ValidatePrimExtent(const pxr::UsdPrim& prim,
    bool check_if_isotropic = false);
  void RaiseFailedToReadAttributeError(const std::string& attr_name,
    const pxr::UsdPrim& prim);
  Vector4<double> GetGeomPrimColor(const pxr::UsdPrim& prim);
  CoulombFriction<double> GetPrimFriction(const pxr::UsdPrim& prim);
  Eigen::Vector3d GetPrimScale(const pxr::UsdPrim& prim);
  math::RigidTransform<double> GetPrimRigidTransform(const pxr::UsdPrim& prim);

  inline static std::vector<std::string> mesh_filenames_;
  const ParsingWorkspace& w_;
  UsdStageMetadata metadata_;
};

namespace {
void InitializeOpenUsdLibrary() {
#if 0
  // Register all relevant plugins.
  auto& registry = pxr::PlugRegistry::PlugRegistry::GetInstance();
  std::vector<std::string> json_paths{{
      "openusd_internal/pxr/usd/ar/plugInfo.json",
      "openusd_internal/pxr/usd/ndr/plugInfo.json",
      "openusd_internal/pxr/usd/sdf/plugInfo.json",
      "openusd_internal/pxr/usd/usdGeom/plugInfo.json",
      "openusd_internal/pxr/usd/usd/plugInfo.json",
      "openusd_internal/pxr/usd/usdShade/plugInfo.json",
  }};
  for (const auto& json_path : json_paths) {
    const RlocationOrError location = FindRunfile(json_path);
    if (!location.error.empty()) {
      throw std::runtime_error(location.error);
    }
    const fs::path info_dir = fs::path(location.abspath).parent_path();
    registry.RegisterPlugins(info_dir.string());
  }
#endif
}
}  // namespace

UsdParserWrapper::UsdParserWrapper() {
  static const int ignored = []() {
    InitializeOpenUsdLibrary();
    return 0;
  }();
  unused(ignored);
}

UsdParserWrapper::~UsdParserWrapper() {
  // TODO(hong-nvidia):
  // Delete all the temporary files (UsdParserWrapper::mesh_filenames_)
  // generated during the parsing process
}

std::optional<ModelInstanceIndex> UsdParserWrapper::AddModel(
    const DataSource& data_source, const std::string& model_name,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  unused(data_source, model_name, parent_model_name, workspace);
  throw std::runtime_error("UsdParser::AddModel is not implemented");
}

std::vector<ModelInstanceIndex> UsdParserWrapper::AddAllModels(
    const DataSource& data_source,
    const std::optional<std::string>& parent_model_name,
    const ParsingWorkspace& workspace) {
  UsdParser parser(workspace);
  return parser.AddAllModels(data_source, parent_model_name, workspace);
}

Eigen::Matrix3d UsdMat3dToEigen(pxr::GfMatrix3d m) {
  Eigen::Matrix3d ret;
  ret << m[0][0], m[0][1], m[0][2],
         m[1][0], m[1][1], m[1][2],
         m[2][0], m[2][1], m[2][2];
  return ret;
}

Eigen::Vector3d UsdVec3dToEigen(pxr::GfVec3d v) {
  return Eigen::Vector3d{ v[0], v[1], v[2] };
}

Eigen::Quaterniond UsdQuatdToEigen(pxr::GfQuatd q) {
  return Eigen::Quaterniond(
    q.GetReal(),
    q.GetImaginary()[0],
    q.GetImaginary()[1],
    q.GetImaginary()[2]);
}

UsdParser::UsdParser(const ParsingWorkspace& ws) : w_{ws} { }

math::RigidTransform<double> UsdParser::GetPrimRigidTransform(
  const pxr::UsdPrim& prim) {
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);

  pxr::GfMatrix4d transform_matrix = xformable.ComputeLocalToWorldTransform(
    pxr::UsdTimeCode::Default());

  pxr::GfTransform transform(transform_matrix);
  pxr::GfVec3d translation = transform.GetTranslation();
  pxr::GfRotation rotation = transform.GetRotation();
  translation *= metadata_.meters_per_unit;

  math::RotationMatrix<double> rotation_matrix(
    UsdQuatdToEigen(rotation.GetQuat()));

  return math::RigidTransform<double>(
    rotation_matrix,
    UsdVec3dToEigen(translation));
}

Eigen::Vector3d UsdParser::GetPrimScale(const pxr::UsdPrim& prim) {
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);

  pxr::GfMatrix4d transform_matrix = xformable.ComputeLocalToWorldTransform(
    pxr::UsdTimeCode::Default());

  pxr::GfTransform transform(transform_matrix);
  pxr::GfVec3d scale = transform.GetScale();
  return UsdVec3dToEigen(scale);
}

CoulombFriction<double> UsdParser::GetPrimFriction(const pxr::UsdPrim& prim) {
  // TODO(hong-nvidia): Use the prim's friction attributes if has those
  // For now, we just use default friction
  return default_friction();
}

Vector4<double> UsdParser::GetGeomPrimColor(const pxr::UsdPrim& prim) {
  pxr::UsdGeomGprim gprim = pxr::UsdGeomGprim(prim);
  pxr::VtArray<pxr::GfVec3f> colors;
  if (gprim.GetDisplayColorAttr().Get(&colors)) {
    pxr::GfVec3f color = colors[0];
    return Vector4<double>(color[0], color[1], color[2], 1.0);
  } else {
    // Prim does not contain color attribute, use default color instead
    return Vector4<double>(0.5, 0.5, 0.5, 1.0);
  }
}

void UsdParser::RaiseFailedToReadAttributeError(const std::string& attr_name,
  const pxr::UsdPrim& prim) {
    w_.diagnostic.Error(fmt::format(
      "Failed to read the \"{}\" attribute of the prim at {}", attr_name,
      prim.GetPath().GetString()));
}

void UsdParser::ValidatePrimExtent(const pxr::UsdPrim& prim,
  bool check_if_isotropic) {
  pxr::VtVec3fArray extent;
  if (!prim.GetAttribute(pxr::TfToken("extent")).Get(&extent)) {
    RaiseFailedToReadAttributeError("extent", prim);
  }
  const pxr::GfVec3f& lower_bound = extent[0];
  const pxr::GfVec3f& upper_bound = extent[1];
  if (-lower_bound[0] != upper_bound[0] ||
      -lower_bound[1] != upper_bound[1] ||
      -lower_bound[2] != upper_bound[2]) {
    w_.diagnostic.Error(fmt::format(
      "The extent of the prim at {} is not symmetric",
      prim.GetPath().GetString()));
  }
  if (check_if_isotropic) {
    if (lower_bound[0] != lower_bound[1] ||
        lower_bound[1] != lower_bound[2] ||
        upper_bound[0] != upper_bound[1] ||
        upper_bound[1] != upper_bound[2]) {
      w_.diagnostic.Error(fmt::format(
        "The extent of the prim at {} should be of the same magnitude across"
        "all three dimensions", prim.GetPath().GetString()));
    }
  }
}

std::unique_ptr<geometry::Shape> UsdParser::CreateGeometryCube(
  const pxr::UsdPrim& prim) {
  ValidatePrimExtent(prim, true);
  pxr::UsdGeomCube cube = pxr::UsdGeomCube(prim);

  double cube_size = 0;
  if (!cube.GetSizeAttr().Get(&cube_size)) {
    RaiseFailedToReadAttributeError("size", prim);
  }

  Eigen::Vector3d cube_dimension = GetPrimScale(prim) * cube_size;
  cube_dimension *= metadata_.meters_per_unit;
  return std::make_unique<geometry::Box>(cube_dimension);
}

std::unique_ptr<geometry::Shape> UsdParser::CreateGeometrySphere(
  const pxr::UsdPrim& prim) {
  ValidatePrimExtent(prim, true);
  pxr::UsdGeomSphere sphere = pxr::UsdGeomSphere(prim);

  double sphere_radius = 0;
  if (!sphere.GetRadiusAttr().Get(&sphere_radius)) {
    RaiseFailedToReadAttributeError("radius", prim);
  }
  sphere_radius *= metadata_.meters_per_unit;

  Eigen::Vector3d prim_scale = GetPrimScale(prim);
  if (prim_scale[0] == prim_scale[1] && prim_scale[1] == prim_scale[2]) {
    return std::make_unique<geometry::Sphere>(sphere_radius * prim_scale[0]);
  } else {
    return std::make_unique<geometry::Ellipsoid>(
      sphere_radius * prim_scale[0],
      sphere_radius * prim_scale[1],
      sphere_radius * prim_scale[2]);
  }
}

std::unique_ptr<geometry::Shape> UsdParser::CreateGeometryCapsule(
  const pxr::UsdPrim& prim) {
  pxr::UsdGeomCapsule capsule = pxr::UsdGeomCapsule(prim);
  Eigen::Vector3d prim_scale = GetPrimScale(prim);

  ValidatePrimExtent(prim);

  pxr::TfToken capsule_axis;
  if (!capsule.GetAxisAttr().Get(&capsule_axis)) {
    RaiseFailedToReadAttributeError("axis", prim);
  }
  if (capsule_axis != metadata_.up_axis) {
    w_.diagnostic.Error(fmt::format(
      "Only upright capsules are supported at the moment. The capsule at {} "
      "is not upright because its axis ({}) differs from the axis of the "
      "stage ({})", prim.GetPath().GetString(), capsule_axis.GetString(),
      metadata_.up_axis.GetString()));
  }

  // Makes the assumption that axis X/Y scales the radius of the capsule
  // and axis Z scales the height of the capsule
  if (prim_scale[0] != prim_scale[1]) {
    w_.diagnostic.Error(fmt::format(
      "The capsule at {} has different scaling in X and Y axis, and that is "
      "not supported", prim.GetPath().GetString()));
  }

  double capsule_height, capsule_radius;
  if (!capsule.GetHeightAttr().Get(&capsule_height)) {
    RaiseFailedToReadAttributeError("height", prim);
  }
  if (!capsule.GetRadiusAttr().Get(&capsule_radius)) {
    RaiseFailedToReadAttributeError("radius", prim);
  }

  return std::make_unique<geometry::Capsule>(
    capsule_radius * prim_scale[0] * metadata_.meters_per_unit,
    capsule_height * prim_scale[2] * metadata_.meters_per_unit);
}

std::unique_ptr<geometry::Shape> UsdParser::CreateGeometryCylinder(
  const pxr::UsdPrim& prim) {
  pxr::UsdGeomCylinder cylinder = pxr::UsdGeomCylinder(prim);
  Eigen::Vector3d prim_scale = GetPrimScale(prim);

  ValidatePrimExtent(prim);

  pxr::TfToken cylinder_axis;
  if (!cylinder.GetAxisAttr().Get(&cylinder_axis)) {
    RaiseFailedToReadAttributeError("axis", prim);
  }
  if (cylinder_axis != metadata_.up_axis) {
    w_.diagnostic.Error(fmt::format(
      "Only upright cylinders are supported at the moment. The cylinder at {} "
      "is not upright because its axis ({}) differs from the axis of the "
      "stage ({})", prim.GetPath().GetString(), cylinder_axis.GetString(),
      metadata_.up_axis.GetString()));
  }

  // Makes the assumption that axis X/Y scales the radius of the cylinder
  // and axis Z scales the height of the cylinder
  if (prim_scale[0] != prim_scale[1]) {
    w_.diagnostic.Error(fmt::format(
      "The cylinder at {} has different scaling in X and Y axis, and that is "
      "not supported", prim.GetPath().GetString()));
  }

  double cylinder_height, cylinder_radius;
  if (!cylinder.GetHeightAttr().Get(&cylinder_height)) {
    RaiseFailedToReadAttributeError("height", prim);
  }
  if (!cylinder.GetRadiusAttr().Get(&cylinder_radius)) {
    RaiseFailedToReadAttributeError("radius", prim);
  }

  return std::make_unique<geometry::Cylinder>(
    cylinder_radius * prim_scale[0] * metadata_.meters_per_unit,
    cylinder_height * prim_scale[2] * metadata_.meters_per_unit);
}

std::string UsdParser::WriteMeshToObjFile(
  const pxr::VtArray<pxr::GfVec3f>& vertices,
  const pxr::VtArray<int>& indices) {
  std::string obj_file_contents;
  int num_triangles = indices.size() / 3;
  for (auto& vertex : vertices) {
    obj_file_contents.append(
      fmt::format("v {} {} {}\n", vertex[0], vertex[1], vertex[2]));
  }
  for (int i = 0; i < num_triangles; ++i) {
    // Adding one to all three indices because obj index starts at one
    int index0 = indices[i * 3] + 1;
    int index1 = indices[i * 3 + 1] + 1;
    int index2 = indices[i * 3 + 2] + 1;
    obj_file_contents.append(
      fmt::format("f {} {} {}\n", index0, index1, index2));
  }

  std::string filename = fmt::format("{}.obj", mesh_filenames_.size());
  std::ofstream out_file(filename);
  if (!out_file.is_open()) {
    w_.diagnostic.Error(
      fmt::format("Failed to create file {} for obj mesh", filename));
  }
  out_file << obj_file_contents;
  out_file.close();
  return filename;
}

std::unique_ptr<geometry::Shape> UsdParser::CreateGeometryMesh(
  const pxr::UsdPrim& prim) {
  pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh(prim);

  Eigen::Vector3d prim_scale = GetPrimScale(prim);
  if (prim_scale[0] != prim_scale[1] || prim_scale[1] != prim_scale[2]) {
    w_.diagnostic.Error(fmt::format(
      "The scaling of the mesh at {} is not isotropic. Non-isotropic scaling "
      "of a mesh is not supported", prim.GetPath().GetString()));
  }

  pxr::VtArray<int> face_vertex_counts;
  if (!mesh.GetFaceVertexCountsAttr().Get(&face_vertex_counts)) {
     RaiseFailedToReadAttributeError("faceVertexCounts", prim);
  }
  for (int count : face_vertex_counts) {
    if (count != 3) {
      w_.diagnostic.Error(fmt::format(
        "The mesh at {} is not a triangle mesh. Only triangle mesh are "
        "supported at the moment", prim.GetPath().GetString()));
    }
  }

  pxr::VtArray<pxr::GfVec3f> vertices;
  if (!mesh.GetPointsAttr().Get(&vertices)) {
     RaiseFailedToReadAttributeError("points", prim);
  }

  pxr::VtArray<int> indices;
  if (!mesh.GetFaceVertexIndicesAttr().Get(&indices)) {
    RaiseFailedToReadAttributeError("faceVertexIndices", prim);
  }

  std::string obj_filename = WriteMeshToObjFile(vertices, indices);

  return std::make_unique<geometry::Mesh>(
    obj_filename, prim_scale[0] * metadata_.meters_per_unit);
}

std::unique_ptr<geometry::Shape> UsdParser::CreateVisualGeometry(
  const pxr::UsdPrim& prim) {
  if (prim.IsA<pxr::UsdGeomCube>()) {
    return CreateGeometryCube(prim);
  } else if (prim.IsA<pxr::UsdGeomSphere>()) {
    return CreateGeometrySphere(prim);
  } else if (prim.IsA<pxr::UsdGeomCapsule>()) {
    return CreateGeometryCapsule(prim);
  } else if (prim.IsA<pxr::UsdGeomCylinder>()) {
    return CreateGeometryCylinder(prim);
  } else if (prim.IsA<pxr::UsdGeomMesh>()) {
    return CreateGeometryMesh(prim);
  } else {
    pxr::TfToken prim_type = prim.GetTypeName();
    w_.diagnostic.Error(
      fmt::format("Unsupported Prim type: {}", prim_type));
    return nullptr;
  }
}

std::unique_ptr<geometry::Shape> UsdParser::CreateCollisionGeometry(
  const pxr::UsdPrim& prim) {
  // For now, we use the raw visual geometry for collision detection
  // for all geometry types
  return CreateVisualGeometry(prim);
}

void UsdParser::ProcessStaticCollider(const pxr::UsdPrim& prim) {
  math::RigidTransform<double> transform = GetPrimRigidTransform(prim);

  auto collision_geometry = CreateCollisionGeometry(prim);
  auto visual_geometry = CreateVisualGeometry(prim);

  if (collision_geometry && visual_geometry) {
    w_.plant->RegisterCollisionGeometry(
      w_.plant->world_body(),
      transform,
      *collision_geometry,
      fmt::format("{}-CollisionGeometry", prim.GetPath().GetString()),
      GetPrimFriction(prim));

    w_.plant->RegisterVisualGeometry(
      w_.plant->world_body(),
      transform,
      *visual_geometry,
      fmt::format("{}-VisualGeometry", prim.GetPath().GetString()),
      GetGeomPrimColor(prim));
  }
}

void UsdParser::ProcessPrim(const pxr::UsdPrim& prim) {
  drake::log()->info(fmt::format("Processing {}", prim.GetPath().GetString()));
  if (prim.HasAPI(pxr::TfToken("PhysicsCollisionAPI"))) {
    if (prim.HasAPI(pxr::TfToken("PhysicsRigidBodyAPI"))) {
      // ProcessRigidBody(prim, plant);
    } else {
      ProcessStaticCollider(prim);
    }
  }
}

UsdStageMetadata UsdParser::GetStageMetadata(const pxr::UsdStageRefPtr stage) {
  UsdStageMetadata metadata;
  if (!stage->GetMetadata(pxr::TfToken("metersPerUnit"),
    &metadata.meters_per_unit)) {
    w_.diagnostic.Warning(fmt::format(
      "Failed to read metersPerUnit in stage metadata. "
      "Using the default value ({}) instead.", metadata.meters_per_unit));
  }
  if (!stage->GetMetadata(pxr::TfToken("upAxis"), &metadata.up_axis)) {
    w_.diagnostic.Warning(fmt::format(
      "Failed to read upAxis in stage metadata. "
      "Using the default value ({}) instead.", metadata.up_axis));
  }
  if (metadata.up_axis != "Z") {
    throw std::runtime_error("Only Z-up stages are supported at the moment");
  }
  return metadata;
}

std::vector<ModelInstanceIndex> UsdParser::AddAllModels(
  const DataSource& data_source,
  const std::optional<std::string>& parent_model_name,
  const ParsingWorkspace& workspace) {
  unused(parent_model_name);

  if (data_source.IsContents()) {
    throw std::runtime_error(
      "Ingesting raw USD content from DataSource is not implemented");
  }

  pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(
    data_source.GetAbsolutePath());
  if (!stage) {
    workspace.diagnostic.Error(fmt::format("Failed to open USD stage",
      data_source.filename()));
  }

  metadata_ = GetStageMetadata(stage);

  for (pxr::UsdPrim prim : stage->Traverse()) {
    ProcessPrim(prim);
  }

  // Returning an empty vector, since right now we only support static
  // colliders attached to the world body (i.e., no actual models are
  // involved).
  return std::vector<ModelInstanceIndex>();
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
