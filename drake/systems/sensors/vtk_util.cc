#include "drake/systems/sensors/vtk_util.h"

#include <Eigen/Dense>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

namespace drake {
namespace systems {
namespace sensors {

vtkSmartPointer<vtkPlaneSource> VtkUtil::CreateSquarePlane(double size) {
  vtkSmartPointer<vtkPlaneSource> plane =
      vtkSmartPointer<vtkPlaneSource>::New();
  const double half_size = size * 0.5;
  plane->SetOrigin(-half_size, -half_size, 0.);
  plane->SetPoint1(-half_size, half_size, 0.);
  plane->SetPoint2(half_size, -half_size, 0.);
  plane->Update();

  return plane;
}

vtkSmartPointer<vtkTransform> VtkUtil::ConvertToVtkTransform(
    const Eigen::Isometry3d& transform) {
  vtkNew<vtkMatrix4x4> vtk_mat;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      vtk_mat->SetElement(i, j, transform.matrix()(i, j));
    }
  }

  vtkSmartPointer<vtkTransform> vtk_transform =
      vtkSmartPointer<vtkTransform>::New();
  vtk_transform->SetMatrix(vtk_mat.GetPointer());

  return vtk_transform;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
