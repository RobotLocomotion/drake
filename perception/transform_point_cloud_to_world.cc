#include "drake/perception/transform_point_cloud_to_world.h"

namespace drake {
namespace perception {

TransformPointCloudToWorld::TransformPointCloudToWorld(
    const RigidBodyTree<double>& tree, int parent_frame_index,
    int child_frame_index)
    : tree_(tree), parent_frame_index_(parent_frame_index),
    child_frame_index_(child_frame_index) {
  this->CreatePorts();
}

TransformPointCloudToWorld::TransformPointCloudToWorld(
    const RigidBodyTree<double>& tree,
    int child_frame_index)
    : TransformPointCloudToWorld(tree,
        tree.findFrame("world")->get_frame_index(), child_frame_index) { }

PointCloud TransformPointCloudToWorld::MakeOutputPointCloud() const {
  PointCloud cloud(0);
  return cloud;
}

void TransformPointCloudToWorld::ApplyTransformToPointCloud(
    const systems::Context<double>& context, PointCloud* output) const {
  const PointCloud* input_point_cloud =
      this->EvalInputValue<PointCloud>(context, point_cloud_input_port_index_);
  DRAKE_ASSERT(input_point_cloud != nullptr);

  const Eigen::VectorXd q =
      this->EvalEigenVectorInput(context, state_input_port_index_)
          .head(tree_.get_num_positions());

  const KinematicsCache<double> cache = tree_.doKinematics(q);

  const Isometry3<double> isom =
    tree_.relativeTransform(cache, parent_frame_index_, child_frame_index_);

  const math::RigidTransform<float> rigid_transform(isom.cast<float>());

  output->resize(input_point_cloud->size());
  for (int i = 0; i < output->size(); i++) {
    output->mutable_xyz(i) = rigid_transform * input_point_cloud->xyz(i);
  }
}

void TransformPointCloudToWorld::CreatePorts() {
  // Create input port for point cloud.
  point_cloud_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  // Create input port for state of a RigidBodyTree.
  const int q_dim = tree_.get_num_positions();
  const int v_dim = tree_.get_num_velocities();
  state_input_port_index_ =
      this->DeclareInputPort(systems::kVectorValued, q_dim + v_dim).get_index();

  // Create output port for transformed point cloud.
  this->DeclareAbstractOutputPort(
      &TransformPointCloudToWorld::MakeOutputPointCloud,
      &TransformPointCloudToWorld::ApplyTransformToPointCloud);
}

}  // namespace perception
}  // namespace drake
