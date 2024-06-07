#pragma once

#include "drake/common/diagnostic_policy.h"
#include "drake/multibody/parsing/detail_common.h"

#include "pxr/usd/usd/prim.h"

namespace drake {
namespace multibody {
namespace internal {

using drake::internal::DiagnosticPolicy;

// Returns the dimension of an UsdGeomCube as `Vector3d(length_x, length_y,
// length_z)`, or nullopt if an error occurs.
std::optional<Eigen::Vector3d> GetBoxDimension(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const DiagnosticPolicy& diagnostic);

// Returns the dimension of an UsdGeomSphere as `Vector3d(length_x, length_y,
// length_z)`, or nullopt if an error occurs.
std::optional<Eigen::Vector3d> GetEllipsoidDimension(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const DiagnosticPolicy& diagnostic);

// Returns the dimension of an UsdGeomCylinder as `Vector2d(radius, length)`,
// or nullopt if an error occurs
std::optional<Eigen::Vector2d> GetCylinderDimension(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic);

// Returns the dimension of an UsdGeomCapsule as `Vector2d(radius, length)`,
// or nullopt if an error occurs
std::optional<Eigen::Vector2d> GetCapsuleDimension(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic);

// Returns the scale factor of an UsdGeomMesh, or nullopt if an error occurs.
std::optional<double> GetMeshScale(
  const pxr::UsdPrim& prim, const DiagnosticPolicy& diagnostic);

// Creates a geometry::Box with a dimension specified by the UsdGeomCube prim.
// Returns nullptr if an error occurs.
std::unique_ptr<geometry::Shape> CreateGeometryBox(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const DiagnosticPolicy& diagnostic);

// Creates a geometry::Sphere or a drake::geometry::Ellipsoid with a dimension
// specified by the UsdGeomSphere prim. Returns nullptr if an error occurs.
std::unique_ptr<geometry::Shape> CreateGeometryEllipsoid(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const DiagnosticPolicy& diagnostic);

// Creates a geometry::Cylinder with a dimension specified by the
// UsdGeomCylinder prim. Returns nullptr if an error occurs.
std::unique_ptr<geometry::Shape> CreateGeometryCylinder(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic);

// Creates a geometry::Capsule with a dimension specified by the
// UsdGeomCapsule prim. Returns nullptr if an error occurs.
std::unique_ptr<geometry::Shape> CreateGeometryCapsule(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic);

// Creates a geometry::Mesh with a dimension specified by the UsdGeomMesh prim.
// Returns nullptr if an error occurs.
std::unique_ptr<geometry::Shape> CreateGeometryMesh(
  const std::string obj_filename, const pxr::UsdPrim& prim,
  double meters_per_unit, const DiagnosticPolicy& diagnostic);

// Returns the SpatialInertia corresponding to the geometry represented
// by the UsdGeomCube prim, or nullopt if an error occurs.
std::optional<SpatialInertia<double>> CreateSpatialInertiaForBox(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const DiagnosticPolicy& diagnostic);

// Returns the SpatialInertia corresponding to the geometry represented
// by the UsdGeomSphere prim, or nullopt if an error occurs.
std::optional<SpatialInertia<double>> CreateSpatialInertiaForEllipsoid(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const DiagnosticPolicy& diagnostic);

// Returns the SpatialInertia corresponding to the geometry represented
// by the UsdGeomCylinder prim, or nullopt if an error occurs.
std::optional<SpatialInertia<double>> CreateSpatialInertiaForCylinder(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic);

// Returns the SpatialInertia corresponding to the geometry represented
// by the UsdGeomCapsule prim, or nullopt if an error occurs.
std::optional<SpatialInertia<double>> CreateSpatialInertiaForCapsule(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const DiagnosticPolicy& diagnostic);

// Returns the axis (the orientation of the geometry) of a UsdGeomCylinder or
// a UsdGeomCapsule as a pxr::TfToken, which could be one of `"X"`, `"Y"`, or
// `"Z"`. Returns nullopt if an error occurs.
std::optional<pxr::TfToken> GetUsdGeomAxis(
  const pxr::UsdPrim& prim, const DiagnosticPolicy& diagnostic);

// Returns the axis (the orientation of the geometry) of a UsdGeomCylinder or
// a UsdGeomCapsule as a Vector3d, which could be one of `Vector3d(1,0,0)`,
// `Vector3d(0,1,0)`, or `Vector3d(0,0,1)`. Rerturns nullopt if an error
// occurs.
std::optional<Eigen::Vector3d> GetUsdGeomAxisUnitVector(
  const pxr::UsdPrim& prim, const DiagnosticPolicy& diagnostic);

// Returns the CoulombFriction of a prim if the prim specifies its
// `physics:dynamicFriction` and `physics:staticFriction attributes`. If not,
// it returns the default friction values.
CoulombFriction<double> GetPrimFriction(const pxr::UsdPrim& prim);

// Returns the mass of a prim if the prim specifies its `physics:mass`
// attribute. If not, it returns the default mass (1.0 kg) and emits a warning.
double GetPrimMass(const pxr::UsdPrim& prim,
  const DiagnosticPolicy& diagnostic);

// Returns the color of a UsdGeom prim as `Vector4d(r,g,b,a)` if it specifies
// its `primvars:displayColor` attribute. If not, it returns the default color.
Eigen::Vector4d GetGeomPrimColor(const pxr::UsdPrim& prim,
  const DiagnosticPolicy& diagnostic);

// Returns the RigidTransform of a prim relative to the world frame, or nullopt
// if an error occurs.
std::optional<math::RigidTransform<double>> GetPrimRigidTransform(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const DiagnosticPolicy& diagnostic);

// Throws an error and returns false if the extent of a UsdGeom is invalid
// (e.g., the upper bound and lower bound have different magnitudes).
// Otherwise, it returns true.
bool ValidatePrimExtent(const pxr::UsdPrim& prim,
  const DiagnosticPolicy& diagnostic, bool check_if_isotropic = false);

// Formats the vertices and indices of a mesh and write to a new obj file.
// Returns true if the file is sucessfully written, false otherwise.
bool WriteMeshToObjFile(const std::string filename,
  const pxr::VtArray<pxr::GfVec3f>& vertices,
  const pxr::VtArray<int>& indices,
  const DiagnosticPolicy& diagnostic);

// Returns the scale of a prim as `Vector3d(scale_x, scale_y, scale_z)`,
// or nullopt if an error occurs.
std::optional<Eigen::Vector3d> GetPrimScale(const pxr::UsdPrim& prim,
  const DiagnosticPolicy& diagnostic);

void RaiseFailedToReadAttributeError(const std::string& attr_name,
  const pxr::UsdPrim& prim, const DiagnosticPolicy& diagnostic);

Eigen::Matrix3d UsdMat3dToEigen(pxr::GfMatrix3d m);

Eigen::Vector3d UsdVec3dToEigen(pxr::GfVec3d v);

Eigen::Quaterniond UsdQuatdToEigen(pxr::GfQuatd q);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
