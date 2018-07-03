#include "drake/perception/rigid_body_tree_removal.h"

namespace drake {
namespace perception {

RigidBodyTreeRemoval::RigidBodyTreeRemoval(
    std::unique_ptr<const RigidBodyTree<double>> tree)
    : tree_(std::move(tree)) {
  /// input port for point cloud
  input_port_index_point_cloud_ = this->DeclareAbstractInputPort().get_index();

  drake::log()->info(" constructor {}", tree_->get_num_positions() + tree_->get_num_velocities());

  /// input port for tree positions
  input_port_index_tree_positions_ =
      this->DeclareInputPort(systems::kVectorValued, tree_->get_num_positions() + tree_->get_num_velocities())
          .get_index();
//  input_port_index_tree_positions_ =
//      this->DeclareInputPort(systems::kVectorValued, tree_->get_num_positions())
//          .get_index();

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
  //  this->EvalAbstractInput(context, color_image_input_port_index_);
  //  const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
  //    DRAKE_ASSERT(input != nullptr);
  //    const auto& command = input->GetValue<lcmt_iiwa_command>();
  // 1. Create the list of points to be considered.
  const systems::AbstractValue* input =
      this->EvalAbstractInput(context, input_port_index_point_cloud_);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_cloud = input->GetValue<PointCloud>();

  std::vector<Eigen::Vector3d> points;
  points.resize(input_cloud.size());
  for (int i = 0; i < input_cloud.size(); i++) {
    points[i] = input_cloud.xyz(i).cast<double>();
  }

  // 2. Extract the indices of the points in collision.
  Eigen::VectorXd q =
      this->EvalEigenVectorInput(context, input_port_index_tree_positions_);
  KinematicsCache<double> kinematics_cache = tree_->doKinematics(q);
  std::vector<size_t> filtered_point_indices =
      const_cast<RigidBodyTree<double>*>(tree_.get())->collidingPoints(kinematics_cache, points, collision_threshold_);

//  RigidBodyTree<double> tree_non_const = *tree_.get();
//  std::vector<size_t> filtered_point_indices =
//        tree_non_const.collidingPoints(kinematics_cache, points, collision_threshold_);
//  std::vector<size_t> filtered_point_indices =
//      const_cast<RigidBodyTree<double>>(tree_.get()).collidingPoints(kinematics_cache, points, collision_threshold_);

  // 3. Create a new point cloud without the colliding points.
  output->resize(filtered_point_indices.size());
  for (size_t i = 0; i < filtered_point_indices.size(); i++) {
    output->mutable_xyz(i) = input_cloud.xyz(filtered_point_indices[i]);
  }

  //  const systems::BasicVector<double>* input_tree_positions =
  //  this->EvalVectorInput(context, input_port_index_tree_positions_);
  //
  //  Eigen::VectorXd u = this->EvalEigenVectorInput(context,
  //  input_port_index_tree_positions_);
  //  auto q = u.head(tree_.get_num_positions());
  //  KinematicsCache<double> kinematics_cache = tree_.doKinematics(q);

  //  VectorXd u = this->EvalEigenVectorInput(context, 0);
  //  auto q = u.head(tree_.get_num_positions());
  //  KinematicsCache<double> kinematics_cache = tree_.doKinematics(q);

  //  KinematicsCache<double> cache = tree_.doKinematics
  //  std::vector<size_t> filtered_point_indices = tree_.collidingPoints(cache);
  //
  //  virtual std::vector< size_t >     collidingPoints (const KinematicsCache<
  //  double > &cache,
  //  const std::vector< Eigen::Vector3d > &points, double collision_threshold)

  //  delete input_tree_positions;
}

// example for abstract output port declaration
// lcmt_iiwa_status IiwaStatusSender::MakeOutputStatus() const {
//  lcmt_iiwa_status msg{};
//  msg.num_joints = num_joints_;
//  msg.joint_position_measured.resize(msg.num_joints, 0);
//  msg.joint_velocity_estimated.resize(msg.num_joints, 0);
//  msg.joint_position_commanded.resize(msg.num_joints, 0);
//  msg.joint_position_ipo.resize(msg.num_joints, 0);
//  msg.joint_torque_measured.resize(msg.num_joints, 0);
//  msg.joint_torque_commanded.resize(msg.num_joints, 0);
//  msg.joint_torque_external.resize(msg.num_joints, 0);
//  return msg;
//}
//
// void IiwaStatusSender::OutputStatus(const Context<double>& context,
//                                    lcmt_iiwa_status* output) const {
//  lcmt_iiwa_status& status = *output;
//
//  status.utime = context.get_time() * 1e6;
//  const systems::BasicVector<double>* command =
//      this->EvalVectorInput(context, 0);
//  const systems::BasicVector<double>* state = this->EvalVectorInput(context,
//  1);
//  const systems::BasicVector<double>* commanded_torque =
//      this->EvalVectorInput(context, 2);
//  const systems::BasicVector<double>* measured_torque =
//      this->EvalVectorInput(context, 3);
//  const systems::BasicVector<double>* external_torque =
//      this->EvalVectorInput(context, 4);
//
//  for (int i = 0; i < num_joints_; ++i) {
//    status.joint_position_measured[i] = state->GetAtIndex(i);
//    status.joint_velocity_estimated[i] = state->GetAtIndex(i + num_joints_);
//    status.joint_position_commanded[i] = command->GetAtIndex(i);
//    status.joint_torque_commanded[i] = commanded_torque->GetAtIndex(i);
//
//    if (external_torque) {
//      status.joint_torque_external[i] = external_torque->GetAtIndex(i);
//    }
//    if (measured_torque) {
//      status.joint_torque_measured[i] = measured_torque->GetAtIndex(i);
//    } else {
//      // TODO(rcory) Update joint_torque_measured to report actual measured
//      // torque once RigidBodyPlant supports it. For now, assume
//      // joint_torque_measured == joint_torque_commanded.
//      status.joint_torque_measured[i] = commanded_torque->GetAtIndex(i);
//    }
//  }
//}

}  // namespace perception
}  // namespace drake
