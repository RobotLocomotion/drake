#include "drake/multibody/global_inverse_kinematics.h"

using Eigen::Isometry3d;

namespace drake {
namespace multibody {
GlobalInverseKinematics::GlobalInverseKinematics(const RigidBodyTree<double> &robot) : robot_(&robot) {

  int num_bodies = robot_->get_num_bodies();
  body_rotmat_.resize(num_bodies);
  body_pos_.resize(num_bodies);
  // Loop through each body in the robot, to add the constraint that the bodies
  // are welded by joints.
  for (int model_id = 0; model_id < robot_->get_num_model_instances(); ++model_id) {
    // In each model, start from the base links to parse the tree.
    for (int body_idx : robot_->FindBaseBodies(model_id)) {
      const RigidBody<double>& body = robot_->get_body(body_idx);
      body_rotmat_[body_idx] = NewContinuousVariables<3, 3>(body.get_name() + "_R");
      body_pos_[body_idx] = NewContinuousVariables<3>(body.get_name() + "_pos");
      // If the body is fixed to the world, the fix the decision variables on
      // the body position and orientation.
      if (body.IsRigidlyFixedToWorld()) {
        Isometry3d body_pose = body.ComputeWorldFixedPose();
        AddBoundingBoxConstraint()

      }
    }
  }
}
}  // namespace multibody
}  // namespace drake
