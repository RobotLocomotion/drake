#pragma once

#include <vtkCylinderSource.h>
#include <vtkSmartPointer.h>
#include <vtkTexturedSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace render {

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

// Sets common sphere options such as its dimensions and resolution.
void SetSphereOptions(vtkTexturedSphereSource* vtk_sphere, double radius);

// Sets common cylinder options such as its dimensions and resolution.
void SetCylinderOptions(vtkCylinderSource* vtk_cylinder, double height,
                        double radius);

// Transforms a vtk cylinder that is y-axis aligned to be z-axis aligned, which
// is what Drake uses.
void TransformToDrakeCylinder(vtkTransform* transform,
                              vtkTransformPolyDataFilter* transform_filter,
                              vtkCylinderSource* vtk_cylinder);

}  // namespace render
}  // namespace geometry
}  // namespace drake
