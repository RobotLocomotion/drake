#include "drake/perception/transform_point_cloud.h"

namespace drake {
namespace perception {

TransformPointCloud::TransformPointCloud() {
  // Create input port for point cloud.
  point_cloud_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  // Create input port for rigid transform.
  rigid_transform_input_port_index_ =
      this->DeclareAbstractInputPort().get_index();

  // Create output port for transformed point cloud.
  this->DeclareAbstractOutputPort(
      &TransformPointCloud::MakeOutputPointCloud,
      &TransformPointCloud::ApplyTransformToPointCloud);
}

PointCloud TransformPointCloud::MakeOutputPointCloud() const {
  PointCloud cloud(0);
  return cloud;
}

void TransformPointCloud::ApplyTransformToPointCloud(
    const systems::Context<double>& context, PointCloud* output) const {
  const PointCloud* input_point_cloud =
      this->EvalInputValue<PointCloud>(context, point_cloud_input_port_index_);
  DRAKE_ASSERT(input_point_cloud != nullptr);

  const math::RigidTransform<float>* input_rigid_transform =
      this->EvalInputValue<math::RigidTransform<float>>(
          context, rigid_transform_input_port_index_);
  DRAKE_ASSERT(input_rigid_transform != nullptr);

  output->resize(input_point_cloud->size());
  for (int i = 0; i < output->size(); i++) {
    output->mutable_xyz(i) = *input_rigid_transform * input_point_cloud->xyz(i);
  }
}

}  // namespace perception
}  // namespace drake
