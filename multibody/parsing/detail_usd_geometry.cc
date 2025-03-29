#include "drake/multibody/parsing/detail_usd_geometry.h"

#include <fstream>
#include <string>

#include "pxr/base/gf/transform.h"
#include "pxr/usd/usdGeom/capsule.h"
#include "pxr/usd/usdGeom/cube.h"
#include "pxr/usd/usdGeom/cylinder.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/sphere.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdPhysics/massAPI.h"

namespace drake {
namespace multibody {
namespace internal {

Eigen::Matrix3d UsdMat3dToEigen(const pxr::GfMatrix3d& m) {
  Eigen::Matrix3d matrix;
  matrix << m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0],
      m[2][1], m[2][2];
  return matrix;
}

Eigen::Vector3d UsdVec3dToEigen(const pxr::GfVec3d& v) {
  return Eigen::Vector3d{v[0], v[1], v[2]};
}

Eigen::Quaterniond UsdQuatdToEigen(const pxr::GfQuatd& q) {
  return Eigen::Quaterniond(q.GetReal(), q.GetImaginary()[0],
                            q.GetImaginary()[1], q.GetImaginary()[2]);
}

std::optional<Eigen::Vector3d> GetPrimScale(
    const pxr::UsdPrim& prim, const DiagnosticPolicy& diagnostic) {
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);
  if (!xformable) {
    diagnostic.Error(
        fmt::format("Failed to cast the Prim at {} into an UsdGeomXformable.",
                    prim.GetPath().GetString()));
    return std::nullopt;
  }

  pxr::GfMatrix4d transform_matrix =
      xformable.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());

  pxr::GfTransform transform(transform_matrix);
  pxr::GfVec3d scale = transform.GetScale();
  return UsdVec3dToEigen(scale);
}

CoulombFriction<double> GetPrimFriction(const pxr::UsdPrim& prim) {
  // TODO(hong-nvidia): Use the prim's friction attributes if has those.
  // For now, we just use default friction.
  return default_friction();
}

double GetPrimMass(const pxr::UsdPrim& prim,
                   const DiagnosticPolicy& diagnostic) {
  if (prim.HasAPI(pxr::TfToken("PhysicsMassAPI"))) {
    auto mass_attribute = pxr::UsdPhysicsMassAPI(prim).GetMassAttr();
    float mass = 0.f;
    if (mass_attribute.Get(&mass)) {
      return static_cast<double>(mass);
    } else {
      // Failed to read the value of the mass attribute.
      // One potential cause is that the author specified its type as double
      // rather than float.
      if ("double" == mass_attribute.GetTypeName()) {
        diagnostic.Error(
            "Double precision float is not supported by "
            "UsdPhysicsMassAPI at the moment. Please use single precision "
            "float "
            "instead.");
      }
    }
  }
  // When mass is not provided explicitly, we assume the value to be 1.0, per
  // https://openusd.org/release/wp_rigid_body_physics.html#body-mass-properties
  const double default_mass = 1.0;
  diagnostic.Warning(
      fmt::format("Failed to read the mass of the Prim at {}. Using the "
                  "default value '{}' instead.",
                  prim.GetPath().GetString(), default_mass));
  return default_mass;
}

std::optional<Eigen::Vector4d> GetGeomPrimColor(
    const pxr::UsdPrim& prim, const DiagnosticPolicy& diagnostic) {
  pxr::UsdGeomGprim gprim = pxr::UsdGeomGprim(prim);
  if (!gprim) {
    return std::nullopt;
  }
  pxr::VtArray<pxr::GfVec3f> colors;
  if (gprim.GetDisplayColorAttr().Get(&colors)) {
    pxr::GfVec3f color = colors[0];
    return Eigen::Vector4d(color[0], color[1], color[2], 1.0);
  } else {
    return std::nullopt;
  }
}

Eigen::Vector4d default_geom_prim_color() {
  return Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
}

void RaiseFailedToReadAttributeError(const std::string& attr_name,
                                     const pxr::UsdPrim& prim,
                                     const DiagnosticPolicy& diagnostic) {
  diagnostic.Error(
      fmt::format("Failed to read the '{}' attribute of the Prim at {}.",
                  attr_name, prim.GetPath().GetString()));
}

std::optional<math::RigidTransform<double>> GetPrimRigidTransform(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const DiagnosticPolicy& diagnostic) {
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);
  if (!xformable) {
    diagnostic.Error(
        fmt::format("Failed to cast the Prim at {} into an UsdGeomXformable.",
                    prim.GetPath().GetString()));
    return std::nullopt;
  }

  pxr::GfMatrix4d transform_matrix =
      xformable.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());

  pxr::GfTransform transform(transform_matrix);
  pxr::GfVec3d translation = transform.GetTranslation() * meters_per_unit;
  pxr::GfRotation rotation = transform.GetRotation();

  math::RotationMatrix<double> rotation_matrix(
      UsdQuatdToEigen(rotation.GetQuat()));

  return math::RigidTransform<double>(rotation_matrix,
                                      UsdVec3dToEigen(translation));
}

bool WriteMeshToObjFile(const std::string& file_path,
                        const pxr::VtArray<pxr::GfVec3f>& vertices,
                        const pxr::VtArray<int>& indices,
                        const DiagnosticPolicy& diagnostic) {
  std::string obj_file_contents;
  int num_triangles = indices.size() / 3;
  for (const auto& vertex : vertices) {
    obj_file_contents.append(
        fmt::format("v {} {} {}\n", vertex[0], vertex[1], vertex[2]));
  }
  for (int i = 0; i < num_triangles; ++i) {
    // Adding one to all three indices because obj index starts at one.
    int index0 = indices[i * 3] + 1;
    int index1 = indices[i * 3 + 1] + 1;
    int index2 = indices[i * 3 + 2] + 1;
    obj_file_contents.append(
        fmt::format("f {} {} {}\n", index0, index1, index2));
  }

  std::ofstream out_file(file_path);
  if (!out_file.is_open()) {
    diagnostic.Error(
        fmt::format("Failed to create file {} for obj mesh.", file_path));
    return false;
  }
  out_file << obj_file_contents;
  out_file.close();
  return true;
}

std::optional<Eigen::Vector3d> GetBoxDimension(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const DiagnosticPolicy& diagnostic) {
  pxr::UsdGeomCube cube = pxr::UsdGeomCube(prim);
  if (!cube) {
    diagnostic.Error(
        fmt::format("Failed to cast the Prim at {} into an UsdGeomCube.",
                    prim.GetPath().GetString()));
    return std::nullopt;
  }

  double cube_size = 0;
  if (!cube.GetSizeAttr().Get(&cube_size)) {
    RaiseFailedToReadAttributeError("size", prim, diagnostic);
    return std::nullopt;
  }

  std::optional<Eigen::Vector3d> prim_scale = GetPrimScale(prim, diagnostic);
  if (!prim_scale.has_value()) {
    return std::nullopt;
  }

  return prim_scale.value() * cube_size * meters_per_unit;
}

std::optional<Eigen::Vector3d> GetEllipsoidDimension(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const DiagnosticPolicy& diagnostic) {
  pxr::UsdGeomSphere sphere = pxr::UsdGeomSphere(prim);
  if (!sphere) {
    diagnostic.Error(
        fmt::format("Failed to cast the Prim at {} into an UsdGeomSphere.",
                    prim.GetPath().GetString()));
    return std::nullopt;
  }

  double sphere_radius = 0;
  if (!sphere.GetRadiusAttr().Get(&sphere_radius)) {
    RaiseFailedToReadAttributeError("radius", prim, diagnostic);
    return std::nullopt;
  }

  std::optional<Eigen::Vector3d> prim_scale = GetPrimScale(prim, diagnostic);
  if (!prim_scale.has_value()) {
    return std::nullopt;
  }
  return prim_scale.value() * sphere_radius * meters_per_unit;
}

std::optional<Eigen::Vector2d> GetCylinderDimension(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic) {
  const pxr::UsdGeomCylinder cylinder = pxr::UsdGeomCylinder(prim);
  if (!cylinder) {
    diagnostic.Error(
        fmt::format("Failed to cast the Prim at {} into an UsdGeomCylinder.",
                    prim.GetPath().GetString()));
    return std::nullopt;
  }

  std::optional<pxr::TfToken> cylinder_axis = GetUsdGeomAxis(prim, diagnostic);
  if (!cylinder_axis.has_value()) {
    return std::nullopt;
  }
  if (cylinder_axis.value() != stage_up_axis) {
    diagnostic.Error(fmt::format(
        "Only upright cylinders are supported at the moment. The cylinder at "
        "{} is not upright because its axis '{}' differs from the axis of the "
        "stage '{}'.",
        prim.GetPath().GetString(), cylinder_axis.value().GetString(),
        stage_up_axis.GetString()));
    return std::nullopt;
  }

  std::optional<Eigen::Vector3d> prim_scale_opt =
      GetPrimScale(prim, diagnostic);
  if (!prim_scale_opt.has_value()) {
    return std::nullopt;
  }
  Eigen::Vector3d prim_scale = prim_scale_opt.value();

  // Makes the assumption that axis X/Y scales the radius of the cylinder
  // and axis Z scales the height of the cylinder.
  if (prim_scale[0] != prim_scale[1]) {
    diagnostic.Error(fmt::format(
        "The cylinder at {} has different scaling in X and Y axis, and that is "
        "not supported.",
        prim.GetPath().GetString()));
    return std::nullopt;
  }

  double cylinder_raw_height, cylinder_raw_radius;
  if (!cylinder.GetRadiusAttr().Get(&cylinder_raw_radius)) {
    RaiseFailedToReadAttributeError("radius", prim, diagnostic);
    return std::nullopt;
  }
  if (!cylinder.GetHeightAttr().Get(&cylinder_raw_height)) {
    RaiseFailedToReadAttributeError("height", prim, diagnostic);
    return std::nullopt;
  }
  double cylinder_radius =
      cylinder_raw_radius * prim_scale[0] * meters_per_unit;
  double cylinder_height =
      cylinder_raw_height * prim_scale[2] * meters_per_unit;
  return Eigen::Vector2d(cylinder_radius, cylinder_height);
}

std::optional<Eigen::Vector2d> GetCapsuleDimension(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic) {
  pxr::UsdGeomCapsule capsule = pxr::UsdGeomCapsule(prim);
  if (!capsule) {
    diagnostic.Error(
        fmt::format("Failed to cast the Prim at {} into an UsdGeomCapsule.",
                    prim.GetPath().GetString()));
    return std::nullopt;
  }

  std::optional<pxr::TfToken> capsule_axis = GetUsdGeomAxis(prim, diagnostic);
  if (!capsule_axis.has_value()) {
    return std::nullopt;
  }
  if (capsule_axis.value() != stage_up_axis) {
    diagnostic.Error(fmt::format(
        "Only upright capsules are supported at the moment. The capsule at {} "
        "is not upright because its axis '{}' differs from the axis of the "
        "stage '{}'.",
        prim.GetPath().GetString(), capsule_axis.value().GetString(),
        stage_up_axis.GetString()));
    return std::nullopt;
  }

  std::optional<Eigen::Vector3d> prim_scale_opt =
      GetPrimScale(prim, diagnostic);
  if (!prim_scale_opt.has_value()) {
    return std::nullopt;
  }
  Eigen::Vector3d prim_scale = prim_scale_opt.value();

  // Makes the assumption that axis X/Y scales the radius of the capsule
  // and axis Z scales the height of the capsule.
  if (prim_scale[0] != prim_scale[1]) {
    diagnostic.Error(fmt::format(
        "The capsule at {} has different scaling in X and Y axis, and that is "
        "not supported.",
        prim.GetPath().GetString()));
    return std::nullopt;
  }

  double capsule_raw_radius, capsule_raw_height;
  if (!capsule.GetRadiusAttr().Get(&capsule_raw_radius)) {
    RaiseFailedToReadAttributeError("radius", prim, diagnostic);
    return std::nullopt;
  }
  if (!capsule.GetHeightAttr().Get(&capsule_raw_height)) {
    RaiseFailedToReadAttributeError("height", prim, diagnostic);
    return std::nullopt;
  }

  double capsule_radius = capsule_raw_radius * prim_scale[0] * meters_per_unit;
  double capsule_height = capsule_raw_height * prim_scale[2] * meters_per_unit;
  return Eigen::Vector2d(capsule_radius, capsule_height);
}

std::unique_ptr<geometry::Shape> CreateGeometryBox(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const DiagnosticPolicy& diagnostic) {
  const std::optional<Eigen::Vector3d> dimension =
      GetBoxDimension(prim, meters_per_unit, diagnostic);
  if (dimension.has_value()) {
    return std::make_unique<geometry::Box>(dimension.value());
  } else {
    return nullptr;
  }
}

std::unique_ptr<geometry::Shape> CreateGeometryEllipsoid(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const DiagnosticPolicy& diagnostic) {
  const std::optional<Eigen::Vector3d> dimension =
      GetEllipsoidDimension(prim, meters_per_unit, diagnostic);
  if (dimension.has_value()) {
    if (dimension.value()[0] == dimension.value()[1] &&
        dimension.value()[1] == dimension.value()[2]) {
      return std::make_unique<geometry::Sphere>(dimension.value()[0]);
    } else {
      return std::make_unique<geometry::Ellipsoid>(
          dimension.value()[0], dimension.value()[1], dimension.value()[2]);
    }
  } else {
    return nullptr;
  }
}

std::unique_ptr<geometry::Shape> CreateGeometryCapsule(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic) {
  const std::optional<Eigen::Vector2d> dimension =
      GetCapsuleDimension(prim, meters_per_unit, stage_up_axis, diagnostic);
  if (dimension.has_value()) {
    return std::make_unique<geometry::Capsule>(dimension.value());
  } else {
    return nullptr;
  }
}

std::unique_ptr<geometry::Shape> CreateGeometryCylinder(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic) {
  const std::optional<Eigen::Vector2d> dimension =
      GetCylinderDimension(prim, meters_per_unit, stage_up_axis, diagnostic);
  if (dimension.has_value()) {
    return std::make_unique<geometry::Cylinder>(dimension.value());
  } else {
    return nullptr;
  }
}

std::unique_ptr<geometry::Shape> CreateGeometryMesh(
    const std::string& obj_file_path, const pxr::UsdPrim& prim,
    double meters_per_unit, const DiagnosticPolicy& diagnostic) {
  pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh(prim);
  if (!mesh) {
    diagnostic.Error(
        fmt::format("Failed to cast the Prim at {} into an UsdGeomMesh.",
                    prim.GetPath().GetString()));
    return nullptr;
  }

  std::optional<Eigen::Vector3d> prim_scale = GetPrimScale(prim, diagnostic);
  if (!prim_scale.has_value()) {
    return nullptr;
  }

  pxr::VtArray<int> face_vertex_counts;
  if (!mesh.GetFaceVertexCountsAttr().Get(&face_vertex_counts)) {
    RaiseFailedToReadAttributeError("faceVertexCounts", prim, diagnostic);
    return nullptr;
  }
  for (int count : face_vertex_counts) {
    if (count != 3) {
      diagnostic.Error(fmt::format(
          "The mesh at {} is not a triangle mesh. Only triangle mesh are "
          "supported at the moment.",
          prim.GetPath().GetString()));
      return nullptr;
    }
  }

  pxr::VtArray<pxr::GfVec3f> vertices;
  if (!mesh.GetPointsAttr().Get(&vertices)) {
    RaiseFailedToReadAttributeError("points", prim, diagnostic);
    return nullptr;
  }
  for (auto& vertex : vertices) {
    vertex *= meters_per_unit;
  }

  pxr::VtArray<int> indices;
  if (!mesh.GetFaceVertexIndicesAttr().Get(&indices)) {
    RaiseFailedToReadAttributeError("faceVertexIndices", prim, diagnostic);
    return nullptr;
  }

  if (!WriteMeshToObjFile(obj_file_path, vertices, indices, diagnostic)) {
    return nullptr;
  }

  return std::make_unique<geometry::Mesh>(obj_file_path, prim_scale.value());
}

std::optional<SpatialInertia<double>> CreateSpatialInertiaForBox(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const DiagnosticPolicy& diagnostic) {
  const std::optional<Eigen::Vector3d> dimension =
      GetBoxDimension(prim, meters_per_unit, diagnostic);
  if (dimension.has_value()) {
    return SpatialInertia<double>::SolidBoxWithMass(
        GetPrimMass(prim, diagnostic), dimension.value()[0],
        dimension.value()[1], dimension.value()[2]);
  } else {
    return std::nullopt;
  }
}

std::optional<SpatialInertia<double>> CreateSpatialInertiaForEllipsoid(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const DiagnosticPolicy& diagnostic) {
  const std::optional<Eigen::Vector3d> dimension =
      GetEllipsoidDimension(prim, meters_per_unit, diagnostic);
  if (dimension.has_value()) {
    return SpatialInertia<double>::SolidEllipsoidWithMass(
        GetPrimMass(prim, diagnostic), dimension.value()[0],
        dimension.value()[1], dimension.value()[2]);
  } else {
    return std::nullopt;
  }
}

std::optional<SpatialInertia<double>> CreateSpatialInertiaForCylinder(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic) {
  const std::optional<Eigen::Vector2d> dimension =
      GetCylinderDimension(prim, meters_per_unit, stage_up_axis, diagnostic);
  auto cylinder_axis = GetUsdGeomAxisUnitVector(prim, diagnostic);
  if (dimension.has_value() && cylinder_axis.has_value()) {
    return SpatialInertia<double>::SolidCylinderWithMass(
        GetPrimMass(prim, diagnostic), dimension.value()[0],
        dimension.value()[1], cylinder_axis.value());
  } else {
    return std::nullopt;
  }
}

std::optional<SpatialInertia<double>> CreateSpatialInertiaForCapsule(
    const pxr::UsdPrim& prim, double meters_per_unit,
    const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic) {
  const std::optional<Eigen::Vector2d> dimension =
      GetCapsuleDimension(prim, meters_per_unit, stage_up_axis, diagnostic);
  auto capsule_axis = GetUsdGeomAxisUnitVector(prim, diagnostic);
  if (dimension.has_value() && capsule_axis.has_value()) {
    return SpatialInertia<double>::SolidCapsuleWithMass(
        GetPrimMass(prim, diagnostic), dimension.value()[0],
        dimension.value()[1], capsule_axis.value());
  } else {
    return std::nullopt;
  }
}

std::optional<pxr::TfToken> GetUsdGeomAxis(const pxr::UsdPrim& prim,
                                           const DiagnosticPolicy& diagnostic) {
  pxr::TfToken axis;
  bool success = false;
  if (prim.IsA<pxr::UsdGeomCylinder>()) {
    pxr::UsdGeomCylinder cylinder = pxr::UsdGeomCylinder(prim);
    success = cylinder.GetAxisAttr().Get(&axis);
  } else if (prim.IsA<pxr::UsdGeomCapsule>()) {
    pxr::UsdGeomCapsule capsule = pxr::UsdGeomCapsule(prim);
    success = capsule.GetAxisAttr().Get(&axis);
  }

  if (!success) {
    RaiseFailedToReadAttributeError("axis", prim, diagnostic);
    return std::nullopt;
  }
  return axis;
}

std::optional<Eigen::Vector3d> GetUsdGeomAxisUnitVector(
    const pxr::UsdPrim& prim, const DiagnosticPolicy& diagnostic) {
  std::optional<pxr::TfToken> axis = GetUsdGeomAxis(prim, diagnostic);
  if (!axis.has_value()) {
    return std::nullopt;
  }
  if (axis.value() == "X") {
    return Eigen::Vector3d(1.0, 0.0, 0.0);
  } else if (axis.value() == "Y") {
    return Eigen::Vector3d(0.0, 1.0, 0.0);
  } else if (axis.value() == "Z") {
    return Eigen::Vector3d(0.0, 0.0, 1.0);
  } else {
    diagnostic.Error(fmt::format("The axis of the geometry at {} is invalid.",
                                 prim.GetPath().GetString()));
    return std::nullopt;
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
