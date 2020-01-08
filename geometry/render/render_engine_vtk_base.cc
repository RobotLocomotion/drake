#include "drake/geometry/render/render_engine_vtk_base.h"

#include "third_party/com_github_finetjul_bender/vtkCapsuleSource.h"

namespace drake {
namespace geometry {
namespace render {

using com_github_finetjul_bender::vtkCapsuleSource;

vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkCapsule(const Capsule& capsule) {
  vtkNew<vtkCapsuleSource> vtk_capsule;
  vtk_capsule->SetCylinderLength(capsule.length());
  vtk_capsule->SetRadius(capsule.radius());
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_capsule->SetThetaResolution(50);
  vtk_capsule->SetPhiResolution(50);
  // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
  // to rotate it to be z-axis aligned because that is what Drake uses.
  vtkNew<vtkTransform> transform;
  transform->RotateX(90);
  vtkSmartPointer<vtkTransformPolyDataFilter> transform_filter =
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transform_filter->SetInputConnection(vtk_capsule->GetOutputPort());
  transform_filter->SetTransform(transform);
  transform_filter->Update();
  return transform_filter;
}

vtkSmartPointer<vtkPolyDataAlgorithm> CreateVtkEllipsoid(
    const Ellipsoid& ellipsoid) {
  vtkNew<vtkTexturedSphereSource> vtk_ellipsoid;
  vtk_ellipsoid->SetRadius(1.0);
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_ellipsoid->SetThetaResolution(50);
  vtk_ellipsoid->SetPhiResolution(50);

  // Scale sphere by each axis extent to generate the ellipsoid.
  vtkNew<vtkTransform> transform;
  transform->Scale(ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
  vtkSmartPointer<vtkTransformPolyDataFilter> transform_filter =
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transform_filter->SetInputConnection(vtk_ellipsoid->GetOutputPort());
  transform_filter->SetTransform(transform);
  transform_filter->Update();
  return transform_filter;
}

void SetSphereOptions(vtkTexturedSphereSource* vtk_sphere, double radius) {
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
