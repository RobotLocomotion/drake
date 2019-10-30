#pragma once

#include <vtkCylinderSource.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

namespace drake {
namespace geometry {
namespace render {

void SetSphereOptions(vtkSphereSource* vtk_sphere, double radius);

void SetCylinderOptions(vtkCylinderSource* vtk_cylinder, double height,
                        double radius);

void TransformToDrakeCylinder(vtkTransform* transform,
                              vtkTransformPolyDataFilter* transform_filter,
                              vtkCylinderSource* vtk_cylinder);

}  // namespace render
}  // namespace geometry
}  // namespace drake
