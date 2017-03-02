#include <drake/multibody/joints/drake_joints.h>
#include "drake/multibody/global_inverse_kinematics.h"

#include "drake/solvers/rotation_constraint.h"
using Eigen::Isometry3d;

using std::string;

namespace drake {
namespace multibody {
GlobalInverseKinematics::GlobalInverseKinematics(const RigidBodyTree<double> &robot) : robot_(&robot) {

  const int num_bodies = robot_->get_num_bodies();
  body_rotmat_.resize(num_bodies);
  body_pos_.resize(num_bodies);
  // Loop through each body in the robot, to add the constraint that the bodies
  // are welded by joints.
  for (int model_id = 0; model_id < robot_->get_num_model_instances(); ++model_id) {
    // In each model, start from the base links to parse the tree.
    const std::vector<const RigidBody<double>*>& model_bodies = robot_->FindModelInstanceBodies(model_id);
    for (const auto& body : model_bodies) {
      const int body_idx = body->get_body_index();
      const string body_R_name = body->get_name() + "_R";
      const string body_pos_name = body->get_name() + "_pos";
      body_rotmat_[body_idx] = NewContinuousVariables<3, 3>(body_R_name);
      body_pos_[body_idx] = NewContinuousVariables<3>(body_pos_name);
      // If the body is fixed to the world, the fix the decision variables on
      // the body position and orientation.
      if (body->IsRigidlyFixedToWorld()) {
        Isometry3d body_pose = body->ComputeWorldFixedPose();
        AddBoundingBoxConstraint(body_rotmat_[body_idx].resize(9, 1),
                                 body_pose.linear().resize(9, 1),
                                 body_pose.linear().resize(9, 1));
        AddBoundingBoxConstraint(body_pos_[body_idx],
                                 body_pose.translation(),
                                 body_pose.translation());
      } else {
        body_rotmat_[body_idx] =
            solvers::NewRotationMatrixVars(this, body_R_name);
        body_pos_[body_idx] =
            solvers::NewRotationMatrixVars(this, body_pos_name);
        solvers::AddRotationMatrixOrthonormalSocpConstraint(this,
                                                            body_rotmat_[body_idx]);
        solvers::AddRotationMatrixMcCormickEnvelopeMilpConstraints(this,
                                                                   body_rotmat_[body_idx],
                                                                   2);

        // If the body has a parent, then add the constraint to connect the
        // parent body with this body through a joint.
        if (body->has_parent_body()) {
          const RigidBody<double> *parent_body = body->get_parent();
          const int parent_idx = parent_body->get_body_index();
          DrakeJoint joint = body->getJoint();
          const auto &joint_to_parent_transform =
              joint.get_transform_to_parent_body();
          if (joint.get_num_velocities() == 1) {
            // Should NOT do this evil dynamic cast here, but currently we do
            // not have a method to tell if a joint is revolute or not.
            if (dynamic_cast<RevoluteJoint *>(&joint)
                == static_cast<RevoluteJoint *>(&joint)) {
              const RevoluteJoint
                  *revolute_joint = static_cast<RevoluteJoint *>(&joint);
              const auto &joint_axis = revolute_joint->joint_axis();
              // The rotation joint is the same in both child body and the parent body.
              AddLinearEqualityConstraint(body_rotmat_[body_idx] * joint_axis - body_rotmat_[parent_idx] * joint_to_parent_transform.linear() * joint_axis, Eigen::Vector3d::Zero());
            }
          }
        }
      }
    }
  }
}
}  // namespace multibody
}  // namespace drake
