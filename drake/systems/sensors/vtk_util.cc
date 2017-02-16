#include "drake/systems/sensors/vtk_util.h"

#include <Eigen/Dense>

#include <vtkCellData.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkUnsignedCharArray.h>

#include "drake/math/rotation_matrix.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {
const double kRadToDeg = 57.29577951308232;
}  // namespace

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
  auto position = transform.translation();
  auto axis_angle = drake::math::rotmat2axis(transform.linear());

  vtkSmartPointer<vtkTransform> vtk_transform =
      vtkSmartPointer<vtkTransform>::New();
  // The order must be Translate first, and then RotateWXYZ.
  vtk_transform->Translate(position[0], position[1], position[2]);
  vtk_transform->RotateWXYZ(axis_angle[3] * kRadToDeg,
                            axis_angle[0], axis_angle[1], axis_angle[2]);

  return vtk_transform;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
