#include "drake/perception/rigid_body_point_cloud_filter.h"

#include <algorithm>
#include <unordered_set>
#include <vector>

namespace drake {
namespace perception {

RigidBodyPointCloudFilter::RigidBodyPointCloudFilter(
    RigidBodyTree<double>* tree, double collision_threshold)
    : tree_(tree), collision_threshold_(collision_threshold) {
  // Create input port for point cloud.
  point_cloud_input_port_index_ = DeclareAbstractInputPort(
      systems::kUseDefaultName,
      Value<PointCloud>{}).get_index();

  // Create input port for tree positions and velocities.
  state_input_port_index_ =
      DeclareInputPort(systems::kVectorValued,
                       tree_->get_num_positions() + tree_->get_num_velocities())
          .get_index();

  // Create output port for filtered point cloud.
  DeclareAbstractOutputPort(&RigidBodyPointCloudFilter::MakeOutputPointCloud,
                            &RigidBodyPointCloudFilter::FilterPointCloud);
}

PointCloud RigidBodyPointCloudFilter::MakeOutputPointCloud() const {
  PointCloud cloud(0);
  return cloud;
}

void RigidBodyPointCloudFilter::FilterPointCloud(
    const systems::Context<double>& context, PointCloud* output) const {
  // 1. Create the list of points to be considered.
  const PointCloud* input_cloud =
      EvalInputValue<PointCloud>(context, point_cloud_input_port_index_);
  DRAKE_ASSERT(input_cloud != nullptr);

  std::vector<Eigen::Vector3d> points;
  points.resize(input_cloud->size());
  for (int i = 0; i < input_cloud->size(); i++) {
    points[i] = input_cloud->xyz(i).cast<double>();
  }

  // 2. Extract the indices of the points in collision.
  const Eigen::VectorXd q =
      EvalEigenVectorInput(context, state_input_port_index_)
          .head(tree_->get_num_positions());
  const KinematicsCache<double> kinematics_cache = tree_->doKinematics(q);
  std::vector<size_t> filtered_point_indices =
      tree_->collidingPoints(kinematics_cache, points, collision_threshold_);

  // 3. Create a new point cloud without the colliding points.
  if (!filtered_point_indices.empty()) {
    std::unordered_set<size_t> unique_indices(filtered_point_indices.begin(),
        filtered_point_indices.end());
    DRAKE_DEMAND(unique_indices.size() <= points.size());
    output->resize(points.size() - unique_indices.size());
    int k = 0;
    for (size_t i = 0; i < points.size(); ++i) {
      if (unique_indices.find(i) == unique_indices.end()) {
        output->mutable_xyz(k) = points[i].cast<float>();
        k++;
      }
    }
  } else {
    *output = *input_cloud;
  }
}

}  // namespace perception
}  // namespace drake
