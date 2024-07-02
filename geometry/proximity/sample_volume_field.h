#pragma once

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

// Samples the volume field at a point p inside the domain the field.
// @pre Point p is expressed in the same frame in which the mesh of the field is
// expressed in.
// @throws std::exception if point p lies outside the domain of the field.
double SampleVolumeFieldOrThrow(const VolumeMeshFieldLinear<double, double>& f,
                                const Bvh<Obb, VolumeMesh<double>> bvh,
                                const Vector3<double>& p);

double SampleVolumeFieldOrThrow(const VolumeMeshFieldLinear<double, double>& f,
                                const Vector3<double>& p);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
