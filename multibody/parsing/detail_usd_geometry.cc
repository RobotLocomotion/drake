#include "drake/multibody/parsing/detail_usd_geometry.h"

#include <string>
#include <fstream>

#include "pxr/base/gf/transform.h"
#include "pxr/usd/usdGeom/capsule.h"
#include "pxr/usd/usdGeom/cube.h"
#include "pxr/usd/usdGeom/cylinder.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/sphere.h"
#include "pxr/usd/usdGeom/xformable.h"

#include "drake/multibody/parsing/detail_parsing_workspace.h"

namespace drake {
namespace multibody {
namespace internal {

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

Eigen::Vector3d GetPrimScale(const pxr::UsdPrim& prim) {
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);

  pxr::GfMatrix4d transform_matrix = xformable.ComputeLocalToWorldTransform(
    pxr::UsdTimeCode::Default());

  pxr::GfTransform transform(transform_matrix);
  pxr::GfVec3d scale = transform.GetScale();
  return UsdVec3dToEigen(scale);
}

CoulombFriction<double> GetPrimFriction(const pxr::UsdPrim& prim) {
  // TODO(hong-nvidia): Use the prim's friction attributes if has those
  // For now, we just use default friction
  return default_friction();
}

Vector4<double> GetGeomPrimColor(const pxr::UsdPrim& prim) {
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

void RaiseFailedToReadAttributeError(const std::string& attr_name,
  const pxr::UsdPrim& prim, const ParsingWorkspace& w) {
    w.diagnostic.Error(fmt::format(
      "Failed to read the \"{}\" attribute of the prim at {}", attr_name,
      prim.GetPath().GetString()));
}

math::RigidTransform<double> GetPrimRigidTransform(
  const pxr::UsdPrim& prim, double meters_per_unit) {
  pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable(prim);

  pxr::GfMatrix4d transform_matrix = xformable.ComputeLocalToWorldTransform(
    pxr::UsdTimeCode::Default());

  pxr::GfTransform transform(transform_matrix);
  pxr::GfVec3d translation = transform.GetTranslation();
  pxr::GfRotation rotation = transform.GetRotation();
  translation *= meters_per_unit;

  math::RotationMatrix<double> rotation_matrix(
    UsdQuatdToEigen(rotation.GetQuat()));

  return math::RigidTransform<double>(
    rotation_matrix,
    UsdVec3dToEigen(translation));
}

void ValidatePrimExtent(const pxr::UsdPrim& prim,
  const ParsingWorkspace& w, bool check_if_isotropic) {
  pxr::VtVec3fArray extent;
  if (!prim.GetAttribute(pxr::TfToken("extent")).Get(&extent)) {
    RaiseFailedToReadAttributeError("extent", prim, w);
  }
  const pxr::GfVec3f& lower_bound = extent[0];
  const pxr::GfVec3f& upper_bound = extent[1];
  if (-lower_bound[0] != upper_bound[0] ||
      -lower_bound[1] != upper_bound[1] ||
      -lower_bound[2] != upper_bound[2]) {
    w.diagnostic.Error(fmt::format(
      "The extent of the prim at {} is not symmetric",
      prim.GetPath().GetString()));
  }
  if (check_if_isotropic) {
    if (lower_bound[0] != lower_bound[1] ||
        lower_bound[1] != lower_bound[2] ||
        upper_bound[0] != upper_bound[1] ||
        upper_bound[1] != upper_bound[2]) {
      w.diagnostic.Error(fmt::format(
        "The extent of the prim at {} should be of the same magnitude across"
        "all three dimensions", prim.GetPath().GetString()));
    }
  }
}

void WriteMeshToObjFile(
  const std::string filename,
  const pxr::VtArray<pxr::GfVec3f>& vertices,
  const pxr::VtArray<int>& indices,
  const ParsingWorkspace& w) {
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
  
  std::ofstream out_file(filename);
  if (!out_file.is_open()) {
    w.diagnostic.Error(
      fmt::format("Failed to create file {} for obj mesh", filename));
  }
  out_file << obj_file_contents;
  out_file.close();
}

std::unique_ptr<geometry::Shape> CreateGeometryCube(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const ParsingWorkspace& w) {
  ValidatePrimExtent(prim, w, true);
  pxr::UsdGeomCube cube = pxr::UsdGeomCube(prim);

  double cube_size = 0;
  if (!cube.GetSizeAttr().Get(&cube_size)) {
    RaiseFailedToReadAttributeError("size", prim, w);
  }

  Eigen::Vector3d cube_dimension = GetPrimScale(prim) * cube_size;
  cube_dimension *= meters_per_unit;
  return std::make_unique<geometry::Box>(cube_dimension);
}

std::unique_ptr<geometry::Shape> CreateGeometrySphere(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const ParsingWorkspace& w) {
  ValidatePrimExtent(prim, w, true);
  pxr::UsdGeomSphere sphere = pxr::UsdGeomSphere(prim);

  double sphere_radius = 0;
  if (!sphere.GetRadiusAttr().Get(&sphere_radius)) {
    RaiseFailedToReadAttributeError("radius", prim, w);
  }
  sphere_radius *= meters_per_unit;

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

std::unique_ptr<geometry::Shape> CreateGeometryCapsule(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const ParsingWorkspace& w) {
  pxr::UsdGeomCapsule capsule = pxr::UsdGeomCapsule(prim);
  Eigen::Vector3d prim_scale = GetPrimScale(prim);

  ValidatePrimExtent(prim, w);

  pxr::TfToken capsule_axis;
  if (!capsule.GetAxisAttr().Get(&capsule_axis)) {
    RaiseFailedToReadAttributeError("axis", prim, w);
  }
  if (capsule_axis != stage_up_axis) {
    w.diagnostic.Error(fmt::format(
      "Only upright capsules are supported at the moment. The capsule at {} "
      "is not upright because its axis ({}) differs from the axis of the "
      "stage ({})", prim.GetPath().GetString(), capsule_axis.GetString(),
      stage_up_axis.GetString()));
  }

  // Makes the assumption that axis X/Y scales the radius of the capsule
  // and axis Z scales the height of the capsule
  if (prim_scale[0] != prim_scale[1]) {
    w.diagnostic.Error(fmt::format(
      "The capsule at {} has different scaling in X and Y axis, and that is "
      "not supported", prim.GetPath().GetString()));
  }

  double capsule_height, capsule_radius;
  if (!capsule.GetHeightAttr().Get(&capsule_height)) {
    RaiseFailedToReadAttributeError("height", prim, w);
  }
  if (!capsule.GetRadiusAttr().Get(&capsule_radius)) {
    RaiseFailedToReadAttributeError("radius", prim, w);
  }

  return std::make_unique<geometry::Capsule>(
    capsule_radius * prim_scale[0] * meters_per_unit,
    capsule_height * prim_scale[2] * meters_per_unit);
}

std::unique_ptr<geometry::Shape> CreateGeometryCylinder(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const ParsingWorkspace& w) {
  pxr::UsdGeomCylinder cylinder = pxr::UsdGeomCylinder(prim);
  Eigen::Vector3d prim_scale = GetPrimScale(prim);

  ValidatePrimExtent(prim, w);

  pxr::TfToken cylinder_axis;
  if (!cylinder.GetAxisAttr().Get(&cylinder_axis)) {
    RaiseFailedToReadAttributeError("axis", prim, w);
  }
  if (cylinder_axis != stage_up_axis) {
    w.diagnostic.Error(fmt::format(
      "Only upright cylinders are supported at the moment. The cylinder at {} "
      "is not upright because its axis ({}) differs from the axis of the "
      "stage ({})", prim.GetPath().GetString(), cylinder_axis.GetString(),
      stage_up_axis.GetString()));
  }

  // Makes the assumption that axis X/Y scales the radius of the cylinder
  // and axis Z scales the height of the cylinder
  if (prim_scale[0] != prim_scale[1]) {
    w.diagnostic.Error(fmt::format(
      "The cylinder at {} has different scaling in X and Y axis, and that is "
      "not supported", prim.GetPath().GetString()));
  }

  double cylinder_height, cylinder_radius;
  if (!cylinder.GetHeightAttr().Get(&cylinder_height)) {
    RaiseFailedToReadAttributeError("height", prim, w);
  }
  if (!cylinder.GetRadiusAttr().Get(&cylinder_radius)) {
    RaiseFailedToReadAttributeError("radius", prim, w);
  }

  return std::make_unique<geometry::Cylinder>(
    cylinder_radius * prim_scale[0] * meters_per_unit,
    cylinder_height * prim_scale[2] * meters_per_unit);
}

std::unique_ptr<geometry::Shape> CreateGeometryMesh(
  const std::string obj_filename, const pxr::UsdPrim& prim,
  double meters_per_unit, const ParsingWorkspace& w) {
  pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh(prim);

  Eigen::Vector3d prim_scale = GetPrimScale(prim);
  if (prim_scale[0] != prim_scale[1] || prim_scale[1] != prim_scale[2]) {
    w.diagnostic.Error(fmt::format(
      "The scaling of the mesh at {} is not isotropic. Non-isotropic scaling "
      "of a mesh is not supported", prim.GetPath().GetString()));
  }

  pxr::VtArray<int> face_vertex_counts;
  if (!mesh.GetFaceVertexCountsAttr().Get(&face_vertex_counts)) {
     RaiseFailedToReadAttributeError("faceVertexCounts", prim, w);
  }
  for (int count : face_vertex_counts) {
    if (count != 3) {
      w.diagnostic.Error(fmt::format(
        "The mesh at {} is not a triangle mesh. Only triangle mesh are "
        "supported at the moment", prim.GetPath().GetString()));
    }
  }

  pxr::VtArray<pxr::GfVec3f> vertices;
  if (!mesh.GetPointsAttr().Get(&vertices)) {
     RaiseFailedToReadAttributeError("points", prim, w);
  }

  pxr::VtArray<int> indices;
  if (!mesh.GetFaceVertexIndicesAttr().Get(&indices)) {
    RaiseFailedToReadAttributeError("faceVertexIndices", prim, w);
  }
  
  WriteMeshToObjFile(obj_filename, vertices, indices, w);

  return std::make_unique<geometry::Mesh>(
    obj_filename, prim_scale[0] * meters_per_unit);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake