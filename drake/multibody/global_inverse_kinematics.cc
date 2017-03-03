#include <drake/multibody/joints/drake_joints.h>
#include "drake/multibody/global_inverse_kinematics.h"

#include "drake/common/eigen_types.h"
#include "drake/solvers/rotation_constraint.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;

using std::string;

using drake::symbolic::Expression;

namespace drake {
namespace multibody {
GlobalInverseKinematics::GlobalInverseKinematics(const RigidBodyTreed& robot, int num_binary_vars_per_half_axis) : robot_(&robot) {

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
      // If the body is fixed to the world, then fix the decision variables on
      // the body position and orientation.
      if (body->IsRigidlyFixedToWorld()) {
        Isometry3d body_pose = body->ComputeWorldFixedPose();
        for (int i = 0; i < 3; ++i) {
          AddBoundingBoxConstraint(body_pose.linear().col(i), body_pose.linear().col(i), body_rotmat_[body_idx].col(i));
        }
        AddBoundingBoxConstraint(body_pose.translation(),
                                 body_pose.translation(),
                                 body_pos_[body_idx]);
      } else {
        body_rotmat_[body_idx] =
            solvers::NewRotationMatrixVars(this, body_R_name);

        solvers::AddRotationMatrixOrthonormalSocpConstraint(
            this, body_rotmat_[body_idx]);

        body_pos_[body_idx] = NewContinuousVariables<3>(body_pos_name);

        // If the body has a parent, then add the constraint to connect the
        // parent body with this body through a joint.
        if (body->has_parent_body()) {
          const RigidBody<double> *parent_body = body->get_parent();
          const int parent_idx = parent_body->get_body_index();
          const DrakeJoint* joint = &(body->getJoint());
          const auto &joint_to_parent_transform =
              joint->get_transform_to_parent_body();
          switch (joint->get_num_velocities()) {
            case 1 : {
              // Should NOT do this evil dynamic cast here, but currently we do
              // not have a method to tell if a joint is revolute or not.
              if (dynamic_cast<const RevoluteJoint*>(joint)
                  == static_cast<const RevoluteJoint*>(joint)) {
                // Adding McCormick Envelope will add binary variables into
                // the program.
                solvers::AddRotationMatrixMcCormickEnvelopeMilpConstraints(
                    this, body_rotmat_[body_idx], num_binary_vars_per_half_axis);

                const RevoluteJoint
                    *revolute_joint = static_cast<const RevoluteJoint*>(joint);
                const Vector3d rotate_axis = revolute_joint->joint_axis().head<3>();
                // The rotation joint is the same in both child body and the parent body.
                AddLinearEqualityConstraint(body_rotmat_[body_idx] * rotate_axis - body_rotmat_[parent_idx] * joint_to_parent_transform.linear() * rotate_axis, Vector3d::Zero());

                // The position of the rotation axis is the same on both child and parent bodies.
                AddLinearEqualityConstraint(body_pos_[parent_idx] + body_rotmat_[parent_idx] * joint_to_parent_transform.translation() - body_pos_[body_idx], Vector3d::Zero());

                // Now add the joint limits constraint -α ≤ θ ≤ α, where α is the
                // maximal angle the joint can rotate. Notice we require that
                // the joint lower and upper limits are symmetric.
                double joint_lb = joint->getJointLimitMin()(0);
                double joint_ub = joint->getJointLimitMax()(0);

                // TODO(hongkai.dai): add the function to change joint_to_parent_transform if the joint lower bound and the joint upper bound do not adds up to 0.
                DRAKE_DEMAND(joint_lb + joint_ub == 0);

                if (joint_ub < M_PI) {
                  // To constrain a joint angle θ within revolute range [-α, α],
                  // we can consider a unit length vector v perpendicular to the
                  // rotation axis; after rotating v by angle θ we get another
                  // vector u, and
                  //    |u - v| <= 2*sin(α/2)
                  // which is a second order cone constraint.

                  // First generate a vector that is perpendicular to rotation axis.
                  Vector3d revolute_vector = rotate_axis.cross(Vector3d(1, 0, 0));
                  double revolute_vector_norm = revolute_vector.norm();
                  if (revolute_vector_norm < 1E-2) {
                    // rotate_axis is almost parallel to [1; 0; 0].
                    revolute_vector = rotate_axis.cross(Vector3d(0, 1, 0));
                    revolute_vector_norm = revolute_vector.norm();
                  }
                  DRAKE_DEMAND(revolute_vector_norm >= 1E-2 - 1E-10);
                  // Normalizes the revolute vector.
                  revolute_vector /= revolute_vector_norm;
                  Eigen::Matrix<Expression, 4, 1> joint_limit_expr;

                  // clang-format off
                  joint_limit_expr << 2 * sin(joint_ub),
                      body_rotmat_[body_idx] * revolute_vector -
                          body_rotmat_[parent_idx] *
                              joint_to_parent_transform.linear() *
                              revolute_vector;
                  // clang-format on;
                  AddLorentzConeConstraint(joint_limit_expr);
                }
              } else {
                throw std::runtime_error("Unsupported joint type.");
              }
              break;
            }
            case 0 : {
              // Fixed to the parent body.

              // The position can be computed from the parent body pose.
              AddLinearEqualityConstraint(body_pos_[parent_idx] + body_rotmat_[parent_idx] * joint_to_parent_transform.translation() - body_pos_[body_idx], Vector3d::Zero());

              // The orientation can be computed from the parent body orientation.
              Matrix3<Expression> orient_invariance = body_rotmat_[parent_idx] * joint_to_parent_transform.linear() - body_rotmat_[body_idx];
              for (int i = 0; i < 3; ++i) {
                AddLinearEqualityConstraint(orient_invariance.col(i), Vector3d::Zero());
              }
              break;
            }
            case 6 : {
              break;
            }
            default : throw std::runtime_error("Unsupporte joint type.");
          }

        }
      }
    }
  }
}

void GlobalInverseKinematics::AddWorldPositionConstraint(int body_idx, const Eigen::Vector3d& body_pt, const Eigen::Vector3d& box_lb, const Eigen::Vector3d& box_ub, const Isometry3d& measured_transform) {
  Vector3<Expression> body_pt_pos = body_pos_[body_idx] + body_rotmat_[body_idx] * body_pt;
  Vector3<Expression> body_pt_in_measured_frame = measured_transform.linear().transpose() * (body_pt_pos - measured_transform.translation());
  AddLinearConstraint(body_pt_in_measured_frame, box_lb, box_ub);
}

void GlobalInverseKinematics::AddWorldOrientationConstraint(
    int body_idx,
    const Eigen::Quaterniond &desired_orientation,
    double angle_tol) {
  // The rotation matrix err R_e satisfies
  // trace(R_e) = 4 * cos(θ / 2)^2 - 1, where θ is the rotation angle between
  // desired orientation and the current orientation. Thus the constraint is
  // 4 * cos(angle_tol / 2) ^ 2 - 1 <= trace(R_e) <= 3
  Matrix3<Expression> rotation_matrix_err =
  desired_orientation.toRotationMatrix() * body_rotmat_[body_idx].transpose();
  double lb = angle_tol < M_PI ? 4 * pow(cos(angle_tol / 2), 2) - 1 : -1;
  AddLinearConstraint(rotation_matrix_err.trace(), lb, 3);
}
}  // namespace multibody
}  // namespace drake
