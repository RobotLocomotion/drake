#pragma once

#include <memory>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh_field.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
Plane<T> CalcEquilibriumPlane(
    VolumeElementIndex element0,
    const VolumeMeshFieldLinear<double, double>& field0_M,
    VolumeElementIndex element1,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<T>& X_MN);

template <typename T>
void IntersectPlaneTetrahedron(
    VolumeElementIndex tetrahedron,
    const VolumeMeshFieldLinear<double, double>& field_M,
    const Plane<T>& plane_M, std::vector<Vector3<T>>* polygon_M);

/*
 @retval polygon_M
 */
template <typename T>
std::vector<Vector3<T>> IntersectTetrahedra(
    VolumeElementIndex element0,
    const VolumeMeshFieldLinear<double, double>& field0_M,
    VolumeElementIndex element1,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<T>& X_MN,
    Vector3<T>* output_normal);

void FieldIntersection(
    const VolumeMeshFieldLinear<double, double>& field0_M,
    const VolumeMeshFieldLinear<double, double>& field1_N,
    const math::RigidTransform<double>& X_MN,
    std::unique_ptr<SurfaceMesh<double>>* surface_MN_M,
    std::unique_ptr<SurfaceMeshFieldLinear<double, double>>* e_MN_M,
    std::vector<Vector3<double>>* grad_eM_Ms,
    std::vector<Vector3<double>>* grad_eN_Ms);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
