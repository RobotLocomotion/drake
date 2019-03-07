#include "drake/perception/transform_point_cloud.h"

namespace drake {
namespace perception {

TransformPointCloud::TransformPointCloud(const RigidBodyTree<double>& tree,
                                         int src_frame_index,
                                         int dest_frame_index)
    : tree_(tree),
      src_frame_index_(src_frame_index),
      dest_frame_index_(dest_frame_index) {
  CreatePorts();
}

TransformPointCloud::TransformPointCloud(const RigidBodyTree<double>& tree,
                                         int src_frame_index)
    : TransformPointCloud(tree, src_frame_index,
                          tree.findFrame("world")->get_frame_index()) {}

PointCloud TransformPointCloud::MakeOutputPointCloud() const {
  PointCloud cloud(0);
  return cloud;
}

void TransformPointCloud::ApplyTransformToPointCloud(
    const systems::Context<double>& context, PointCloud* output) const {
  const PointCloud* input_point_cloud =
      EvalInputValue<PointCloud>(context, point_cloud_input_port_index_);
  DRAKE_ASSERT(input_point_cloud != nullptr);

  const Eigen::VectorXd q =
      EvalEigenVectorInput(context, state_input_port_index_)
          .head(tree_.get_num_positions());

  const KinematicsCache<double> cache = tree_.doKinematics(q);

  const Isometry3<double> isom =
      tree_.relativeTransform(cache, dest_frame_index_, src_frame_index_);

  const Eigen::Isometry3f isomf(isom);
  const auto out = isomf * input_point_cloud->xyzs();
  output->resize(input_point_cloud->size());
  output->mutable_xyzs() = out;
}

void TransformPointCloud::CreatePorts() {
  // Create input port for point cloud.
  point_cloud_input_port_index_ = DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<PointCloud>{}).get_index();

  // Create input port for state of a RigidBodyTree.
  const int q_dim = tree_.get_num_positions();
  const int v_dim = tree_.get_num_velocities();
  state_input_port_index_ =
      DeclareInputPort(systems::kVectorValued, q_dim + v_dim).get_index();

  // Create output port for transformed point cloud.
  DeclareAbstractOutputPort(&TransformPointCloud::MakeOutputPointCloud,
                            &TransformPointCloud::ApplyTransformToPointCloud);
}

}  // namespace perception
}  // namespace drake
