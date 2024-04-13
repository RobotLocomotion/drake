#include "drake/multibody/parsing/detail_common.h"


#include "pxr/usd/usd/prim.h"

namespace drake {
namespace multibody {
namespace internal {

struct ParsingWorkspace;

std::unique_ptr<geometry::Shape> CreateGeometryCube(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const ParsingWorkspace& w);

std::unique_ptr<geometry::Shape> CreateGeometrySphere(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const ParsingWorkspace& w);

std::unique_ptr<geometry::Shape> CreateGeometryCylinder(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const ParsingWorkspace& w);

std::unique_ptr<geometry::Shape> CreateGeometryCapsule(
  const pxr::UsdPrim& prim, double meters_per_unit,
  const pxr::TfToken& stage_up_axis, const ParsingWorkspace& w);

std::unique_ptr<geometry::Shape> CreateGeometryMesh(
  const std::string obj_filename, const pxr::UsdPrim& prim,
  double meters_per_unit, const ParsingWorkspace& w);

CoulombFriction<double> GetPrimFriction(const pxr::UsdPrim& prim);

Vector4<double> GetGeomPrimColor(const pxr::UsdPrim& prim);

math::RigidTransform<double> GetPrimRigidTransform(const pxr::UsdPrim& prim,
  double meters_per_unit);

void RaiseFailedToReadAttributeError(const std::string& attr_name,
  const pxr::UsdPrim& prim, const ParsingWorkspace& w);

void ValidatePrimExtent(const pxr::UsdPrim& prim,
  const ParsingWorkspace& w, bool check_if_isotropic = false);

void WriteMeshToObjFile(const std::string filename,
  const pxr::VtArray<pxr::GfVec3f>& vertices,
  const pxr::VtArray<int>& indices,
  const ParsingWorkspace& w);

Eigen::Matrix3d UsdMat3dToEigen(pxr::GfMatrix3d m);

Eigen::Vector3d UsdVec3dToEigen(pxr::GfVec3d v);

Eigen::Quaterniond UsdQuatdToEigen(pxr::GfQuatd q);

Eigen::Vector3d GetPrimScale(const pxr::UsdPrim& prim);

}  // namespace internal
}  // namespace multibody
}  // namespace drake