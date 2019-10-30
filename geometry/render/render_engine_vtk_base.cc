#include "drake/geometry/render/render_engine_vtk_base.h"

namespace drake {
namespace geometry {
namespace render {

void CreateVtkCapsule(vtkSmartPointer<vtkAppendPolyData> append_filter,
                      double radius, double length) {
  // Since there's no native capsule support, we represent it by appending a
  // cylinder and two spheres.
  // TODO(tehbelinda): Replace with the Apache licensed vtkCapsuleSource from
  // https://github.com/finetjul/bender/tree/master/Libs/VTK/Filters/Sources.

  const double half_length = length / 2;
  // Place the spheres at the two opposite ends along the z-axis.
  const double sphere_offsets[2] = {half_length, -half_length};
  for (const double& offset : sphere_offsets) {
    vtkNew<vtkSphereSource> vtk_sphere;
    SetSphereOptions(vtk_sphere.GetPointer(), radius);
    vtkNew<vtkTransform> transform;
    vtkNew<vtkTransformPolyDataFilter> transform_filter;
    transform->Translate(0, 0, offset);
    transform_filter->SetInputConnection(vtk_sphere->GetOutputPort());
    transform_filter->SetTransform(transform);
    transform_filter->Update();
    append_filter->AddInputData(transform_filter->GetOutput());
  }

  vtkNew<vtkCylinderSource> vtk_cylinder;
  SetCylinderOptions(vtk_cylinder, length, radius);
  // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
  // to rotate it to be z-axis aligned because that is what Drake uses.
  vtkNew<vtkTransform> transform;
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  TransformToDrakeCylinder(transform, transform_filter, vtk_cylinder);
  append_filter->AddInputData(transform_filter->GetOutput());
}

void SetSphereOptions(vtkSphereSource* vtk_sphere, double radius) {
  vtk_sphere->SetRadius(radius);
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_sphere->SetThetaResolution(50);
  vtk_sphere->SetPhiResolution(50);
}

void SetCylinderOptions(vtkCylinderSource* vtk_cylinder, double height,
                        double radius) {
  vtk_cylinder->SetHeight(height);
  vtk_cylinder->SetRadius(radius);
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_cylinder->SetResolution(50);
}

void TransformToDrakeCylinder(vtkTransform* transform,
                              vtkTransformPolyDataFilter* transform_filter,
                              vtkCylinderSource* vtk_cylinder) {
  transform->RotateX(90);
  transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
  transform_filter->SetTransform(transform);
  transform_filter->Update();
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
