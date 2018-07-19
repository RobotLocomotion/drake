#include "drake/perception/transform_point_cloud.h"

namespace drake {
namespace perception {

TransformPointCloud::TransformPointCloud() {
  // input port for point cloud
  point_cloud_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  // input port for rigid transform
  rigid_transform_input_port_index_ =
      this->DeclareAbstractInputPort().get_index();

  /// output port for transformed point cloud
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
  const systems::AbstractValue* input_point_cloud_ptr =
        this->EvalAbstractInput(context, point_cloud_input_port_index_);
  DRAKE_ASSERT(input_point_cloud_ptr != nullptr);
  const auto& input_point_cloud =
        input_point_cloud_ptr->GetValue<PointCloud>();

  const systems::AbstractValue* input_rigid_transform_ptr =
        this->EvalAbstractInput(context, rigid_transform_input_port_index_);
  DRAKE_ASSERT(input_rigid_transform_ptr != nullptr);
  const auto& input_rigid_transform =
        input_rigid_transform_ptr->GetValue<math::RigidTransform<float>>();

  output->resize(input_point_cloud.size());
  for(int i = 0; i < output->size(); i++) {
    output->mutable_xyz(i) = input_rigid_transform * input_point_cloud.xyz(i);
  }
}

}  // namespace perception
}  // namespace drake
