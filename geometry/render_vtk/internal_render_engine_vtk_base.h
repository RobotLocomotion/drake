#pragma once

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkCylinderSource.h>           // vtkFiltersSources
#include <vtkSmartPointer.h>             // vtkCommonCore
#include <vtkTexturedSphereSource.h>     // vtkFiltersSources
#include <vtkTransform.h>                // vtkCommonTransforms
#include <vtkTransformPolyDataFilter.h>  // vtkFiltersGeneral

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/render/render_mesh.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

// Creates a z-axis aligned VTK capsule.
vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkCapsule(const Capsule& capsule);

// Creates a box with texture coordinates such that the image is stretched
// over each face. The texture can be optionally scaled/tiled via texture
// scale properties in the given properties.
vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkBox(
    const Box& box, const PerceptionProperties& properties);

// Creates a VTK ellipsoid scaled from a sphere.
vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkEllipsoid(
    const Ellipsoid& ellipsoid);

// Creates a VTK mesh from the given mesh data.
vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkMesh(
    geometry::internal::RenderMesh mesh_data);

// Sets common sphere options such as its dimensions and resolution.
void SetSphereOptions(vtkTexturedSphereSource* vtk_sphere, double radius);

// Sets common cylinder options such as its dimensions and resolution.
void SetCylinderOptions(vtkCylinderSource* vtk_cylinder, double height,
                        double radius);

// Transforms a vtk cylinder that is y-axis aligned to be z-axis aligned, which
// is what Drake uses. It does a further orientation so that the longitudinal
// texture seam aligns with the seam on the sphere.
vtkSmartPointer<vtkPolyDataAlgorithm> TransformToDrakeCylinder(
    vtkCylinderSource* vtk_cylinder);

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
