#include "drake/perception/rigid_body_tree_removal.h"

namespace drake {
namespace perception {

RigidBodyTreeRemoval::RigidBodyTreeRemoval(const RigidBodyTree<double>& tree)
    : tree_(tree) {
  /// input port for point cloud
  point_cloud_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  /// input port for tree positions and velocities
  state_input_port_index_ =
      this->DeclareInputPort(
              systems::kVectorValued,
              tree_.get_num_positions() + tree_.get_num_velocities())
          .get_index();

  /// output port for filtered point cloud
  this->DeclareAbstractOutputPort(&RigidBodyTreeRemoval::MakeOutputPointCloud,
                                  &RigidBodyTreeRemoval::FilterPointCloud);
}

PointCloud RigidBodyTreeRemoval::MakeOutputPointCloud() const {
  PointCloud cloud(0);
  return cloud;
}

void RigidBodyTreeRemoval::FilterPointCloud(
    const systems::Context<double>& context, PointCloud* output) const {
  // 1. Create the list of points to be considered.
  const systems::AbstractValue* input =
      this->EvalAbstractInput(context, point_cloud_input_port_index_);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_cloud = input->GetValue<PointCloud>();

  std::vector<Eigen::Vector3d> points;
  points.resize(input_cloud.size());
  for (int i = 0; i < input_cloud.size(); i++) {
    points[i] = input_cloud.xyz(i).cast<double>();
  }

  // 2. Extract the indices of the points in collision.
  Eigen::VectorXd q =
      this->EvalEigenVectorInput(context, state_input_port_index_)
          .head(tree_.get_num_positions());
  KinematicsCache<double> kinematics_cache = tree_.doKinematics(q);
  std::vector<size_t> filtered_point_indices =
      const_cast<RigidBodyTree<double>&>(tree_).collidingPoints(
          kinematics_cache, points, collision_threshold_);

  // 3. Create a new point cloud without the colliding points.
  output->resize(filtered_point_indices.size());
  for (size_t i = 0; i < filtered_point_indices.size(); i++) {
    output->mutable_xyz(i) = input_cloud.xyz(filtered_point_indices[i]);
  }
}

}  // namespace perception
}  // namespace drake
