#include "drake/geometry/render_vtk/internal_vtk_util.h"

#include <Eigen/Dense>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkNew.h>           // vtkCommonCore
#include <vtkPlaneSource.h>   // vtkFiltersSources
#include <vtkSmartPointer.h>  // vtkCommonCore
#include <vtkTransform.h>     // vtkCommonTransforms

#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace render_vtk {
namespace internal {

vtkSmartPointer<vtkPlaneSource> CreateSquarePlane(double size) {
  vtkSmartPointer<vtkPlaneSource> plane =
      vtkSmartPointer<vtkPlaneSource>::New();
  const double half_size = size * 0.5;
  plane->SetOrigin(-half_size, -half_size, 0.);
  plane->SetPoint1(-half_size, half_size, 0.);
  plane->SetPoint2(half_size, -half_size, 0.);
  plane->SetNormal(0., 0., 1.);
  plane->Update();

  return plane;
}

vtkSmartPointer<vtkTransform> ConvertToVtkTransform(
    const math::RigidTransformd& transform, const Vector3<double>& scale) {
  vtkNew<vtkMatrix4x4> vtk_mat;
  for (int i = 0; i < 3; ++i) {
    const auto& row = transform.rotation().row(i);
    for (int j = 0; j < 3; ++j) {
      vtk_mat->SetElement(i, j, row(j) * scale(i));
    }
    vtk_mat->SetElement(i, 3, transform.translation()(i));
  }
  for (int j = 0; j < 3; ++j) {
    vtk_mat->SetElement(3, j, 0.0);
  }
  vtk_mat->SetElement(3, 3, 1.0);

  vtkSmartPointer<vtkTransform> vtk_transform =
      vtkSmartPointer<vtkTransform>::New();
  vtk_transform->SetMatrix(vtk_mat.GetPointer());

  return vtk_transform;
}

}  // namespace internal
}  // namespace render_vtk
}  // namespace geometry
}  // namespace drake
