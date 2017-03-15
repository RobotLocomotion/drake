#include "drake/multibody/global_inverse_kinematics.h"

#include <string>

#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/drake_joints.h"
#include "drake/solvers/rotation_constraint.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Matrix3d;

using std::string;

using drake::symbolic::Expression;
using drake::solvers::VectorDecisionVariable;

namespace drake {
namespace multibody {
GlobalInverseKinematics::GlobalInverseKinematics(
    const RigidBodyTreed& robot, int num_binary_vars_per_half_axis)
    : robot_(&robot) {
  const int num_bodies = robot_->get_num_bodies();
  R_WB_.resize(num_bodies);
  p_WBo_.resize(num_bodies);
  // Loop through each body in the robot, to add the constraint that the bodies
  // are welded by joints.
  for (int body_idx = 1; body_idx < num_bodies; ++body_idx) {
    const RigidBody<double>& body = robot_->get_body(body_idx);
    const string body_R_name = body.get_name() + "_R";
    const string body_pos_name = body.get_name() + "_pos";
    p_WBo_[body_idx] = NewContinuousVariables<3>(body_pos_name);
    // If the body is fixed to the world, then fix the decision variables on
    // the body position and orientation.
    if (body.IsRigidlyFixedToWorld()) {
      R_WB_[body_idx] = NewContinuousVariables<3, 3>(body_R_name);
      Isometry3d X_WB = body.ComputeWorldFixedPose();
      // TODO(hongkai.dai): clean up this for loop using elementwise matrix
      // constraint when it is ready.
      for (int i = 0; i < 3; ++i) {
        AddBoundingBoxConstraint(X_WB.linear().col(i),
                                 X_WB.linear().col(i),
                                 R_WB_[body_idx].col(i));
      }
      AddBoundingBoxConstraint(X_WB.translation(),
                               X_WB.translation(),
                               p_WBo_[body_idx]);
    } else {
      R_WB_[body_idx] =
          solvers::NewRotationMatrixVars(this, body_R_name);

      solvers::AddRotationMatrixOrthonormalSocpConstraint(
          this, R_WB_[body_idx]);

      // If the body has a parent, then add the constraint to connect the
      // parent body with this body through a joint.
      if (body.has_parent_body()) {
        const RigidBody<double> *parent_body = body.get_parent();
        const int parent_idx = parent_body->get_body_index();
        const DrakeJoint* joint = &(body.getJoint());
        // Frame `F` is the inboard frame of the joint, rigidly attached to the
        // parent link.
        const auto &X_PF = joint->get_transform_to_parent_body();
        switch (joint->get_num_velocities()) {
          case 0 : {
            // Fixed to the parent body.

            // The position can be computed from the parent body pose.
            // p_WBc = p_WBp + R_WBp * p_BpBc
            // where Bc is the child body frame.
            //       Bp is the parent body frame.
            //       W is the world frame.
            AddLinearEqualityConstraint(
                p_WBo_[parent_idx] +
                    R_WB_[parent_idx] *
                        X_PF.translation() -
                    p_WBo_[body_idx],
                Vector3d::Zero());

            // The orientation can be computed from the parent body orientation.
            // R_WBp * R_BpBc = R_WBc
            Matrix3<Expression> orient_invariance =
                R_WB_[parent_idx] * X_PF.linear() -
                    R_WB_[body_idx];
            for (int i = 0; i < 3; ++i) {
              AddLinearEqualityConstraint(orient_invariance.col(i),
                                          Vector3d::Zero());
            }
            break;
          }
          case 1 : {
            // Should NOT do this evil dynamic cast here, but currently we do
            // not have a method to tell if a joint is revolute or not.
            if (dynamic_cast<const RevoluteJoint*>(joint) != nullptr) {
              // Adding McCormick Envelope will add binary variables into
              // the program.
              solvers::AddRotationMatrixMcCormickEnvelopeMilpConstraints(
                  this, R_WB_[body_idx], num_binary_vars_per_half_axis);

              const RevoluteJoint
                  *revolute_joint = dynamic_cast<const RevoluteJoint*>(joint);
              // axis_F is the vector of the rotation axis in the joint
              // inboard/outboard frame.
              const Vector3d axis_F =
                  revolute_joint->joint_axis().head<3>();

              // Add the constraint R_WB * axis_B = R_WP * R_PF * axis_F, where
              // axis_B = axis_F since the rotation axis is invaraiant in the
              // inboard frame F and the outboard frame B.
              AddLinearEqualityConstraint(
                  R_WB_[body_idx] * axis_F -
                      R_WB_[parent_idx] *
                          X_PF.linear() * axis_F,
                  Vector3d::Zero());

              // The position of the rotation axis is the same on both child and
              // parent bodies.
              AddLinearEqualityConstraint(
                  p_WBo_[parent_idx] +
                      R_WB_[parent_idx] *
                          X_PF.translation() -
                      p_WBo_[body_idx],
                  Vector3d::Zero());

              // Now we process the joint limits constraint.
              double joint_lb = joint->getJointLimitMin()(0);
              double joint_ub = joint->getJointLimitMax()(0);
              double joint_bound = (joint_ub - joint_lb) / 2;

              if (joint_bound < M_PI) {
                // We use the fact that if the angle between two unit length
                // vectors u and v is smaller than α, it is equivalent to
                // |u - v| <= 2*sin(α/2)
                // which is a second order cone constraint.

                // If the rotation angle θ satisfies
                // a <= θ <= b
                // This is equivalent to
                // -(b-a)/2 <= θ - (a+b)/2 <= (b-a)/2
                // where (a+b)/2 is the joint offset, such that the bounds on
                // θ - (a+b)/2 are symmetric.
                // We use the following notation:
                // R_WP     The rotation matrix of parent frame `P` to world
                //          frame `W`.
                // R_WC     The rotation matrix of child frame `C` to world
                //          frame `W`.
                // R_PF     The rotation matrix of joint frame `F` to parent
                //          frame `P`.
                // R(k, θ)  The rotation matrix along joint axis k by angle θ.
                // The kinematics constraint is
                // R_WP * R_PJ * R(k, θ) = R_WC.
                // This is equivalent to
                // R_WP * R_PF * R(k, (a+b)/2) * R(k, θ-(a+b)/2)) = R_WC.
                // So to constrain that -(b-a)/2 <= θ - (a+b)/2 <= (b-a)/2,
                // we can constrain the angle between the two vectors
                // R_WC * v and R_WP * R_PF * R(k,(a+b)/2) * v is no larger than
                // (b-a)/2, where v is a unit length vector perpendicular to
                // the rotation axis k, in the joint frame.
                // Thus we can constrain that
                // |R_WC*v - R_WP * R_PF * R(k,(a+b)/2)*v | <= 2*sin ((b-a) / 4)
                // as we explained above.

                // First generate a vector v_C that is perpendicular to rotation
                // axis, in child frame.
                Vector3d v_C = axis_F.cross(Vector3d(1, 0, 0));
                double v_C_norm = v_C.norm();
                if (v_C_norm < sqrt(2) / 2) {
                  // axis_F is almost parallel to [1; 0; 0]. Try another axis
                  // [0, 1, 0]
                  v_C = axis_F.cross(Vector3d(0, 1, 0));
                  v_C_norm = v_C.norm();
                }
                // Normalizes the revolute vector.
                v_C /= v_C_norm;

                // joint_limit_expr is going to be within the Lorentz cone.
                Eigen::Matrix<Expression, 4, 1> joint_limit_expr;
                joint_limit_expr(0) = 2 * sin(joint_bound / 2);
                // rotmat_joint_offset is R(k, (a+b)/2) explained above.
                Matrix3d rotmat_joint_offset =
                    Eigen::AngleAxisd((joint_lb + joint_ub) / 2, axis_F)
                        .toRotationMatrix();

                // joint_limit_expr.tail<3> is
                // R_WC * v - R_WP * R_PF * R(k,(a+b)/2) * v mentioned above.
                joint_limit_expr.tail<3>() = R_WB_[body_idx] * v_C -
                                             R_WB_[parent_idx] * X_PF.linear() *
                                                 rotmat_joint_offset * v_C;
                AddLorentzConeConstraint(joint_limit_expr);
              }
            } else {
              // TODO(hongkai.dai): Add prismatic and helical joint.
              throw std::runtime_error("Unsupported joint type.");
            }
            break;
          }
          case 6 : {
            // This is the floating base case, just add the rotation matrix
            // constraint.
            solvers::AddRotationMatrixMcCormickEnvelopeMilpConstraints(
                this, R_WB_[body_idx], num_binary_vars_per_half_axis);
            break;
          }
          default : throw std::runtime_error("Unsupported joint type.");
        }
      }
    }
  }
}

const solvers::MatrixDecisionVariable<3, 3>&
GlobalInverseKinematics::body_rotation_matrix(int body_index) const {
  if (body_index >= robot_->get_num_bodies() || body_index <= 0) {
    throw std::runtime_error("body index out of range.");
  }
  return R_WB_[body_index];
}

const solvers::VectorDecisionVariable<3>&
GlobalInverseKinematics::body_position(int body_index) const {
  if (body_index >= robot_->get_num_bodies() || body_index <= 0) {
    throw std::runtime_error("body index out of range.");
  }
  return p_WBo_[body_index];
}

Eigen::VectorXd
GlobalInverseKinematics::ReconstructGeneralizedPositionSolution() const {
  Eigen::VectorXd q(robot_->get_num_positions());
  for (int body_idx = 1; body_idx < robot_->get_num_bodies(); ++body_idx) {
    const RigidBody<double>& body = robot_->get_body(body_idx);
    const Matrix3d R_WC = GetSolution(R_WB_[body_idx]);
    if (!body.IsRigidlyFixedToWorld() && body.has_parent_body()) {
      const RigidBody<double>* parent = body.get_parent();
      // R_WP is the rotation matrix of parent frame to the world frame.
      const Matrix3d R_WP = GetSolution(R_WB_[parent->get_body_index()]);
      const DrakeJoint* joint = &(body.getJoint());
      const auto& X_PF = joint->get_transform_to_parent_body();

      int num_positions = joint->get_num_positions();
      // For each different type of joints, use a separate branch to compute
      // the posture for that joint.
      if (joint->is_floating()) {
        // p_WBi is the position of the body frame in the world frame.
        Vector3d p_WBi = GetSolution(p_WBo_[body_idx]);
        Matrix3d normalized_rotmat = math::ProjectMatToRotMat(R_WC);

        q.segment<3>(body.get_position_start_index()) = p_WBi;
        if (num_positions == 6) {
          // The position order is x-y-z-roll-pitch-yaw.
          q.segment<3>(body.get_position_start_index() + 3) =
              math::rotmat2rpy(normalized_rotmat);
        } else {
          // The position order is x-y-z-qw-qx-qy-qz, namely translation first,
          // and quaternion second.
          q.segment<4>(body.get_position_start_index() + 3) =
              math::rotmat2quat(normalized_rotmat);
        }
      } else if (num_positions == 1) {
        // Should NOT do this evil dynamic cast here, but currently we do
        // not have a method to tell if a joint is revolute or not.
        if (dynamic_cast<const RevoluteJoint *>(joint) != nullptr) {
          const RevoluteJoint* revolute_joint =
              dynamic_cast<const RevoluteJoint*>(joint);
          Matrix3d joint_rotmat =
              X_PF.linear().transpose() *
              R_WP.transpose() * R_WC;
          // The joint_angle_axis computed from the body orientation is very
          // likely not being aligned with the real joint axis. The reason is
          // that we use a relaxation of the rotation matrix, and thus
          // R_WC might not lie on SO(3) exactly.
          Matrix3d normalized_rotmat = math::ProjectMatToRotMat(joint_rotmat);
          Eigen::AngleAxisd joint_angle_axis(normalized_rotmat);

          const Vector3d rotate_axis = revolute_joint->joint_axis().head<3>();
          // There can be two possible angle-axis combination, negating the
          // both angle and axis will result in the same rotation.
          q(body.get_position_start_index()) =
              joint_angle_axis.axis().dot(rotate_axis) > 0
              ? joint_angle_axis.angle() : -joint_angle_axis.angle();
          // TODO(hongkai.dai): check if the joint is within the limit, shift
          // it by 2 * PI if it is out of range.
        } else {
          // TODO(hongkai.dai): add prismatic and helical joints.
          throw std::runtime_error("Unsupported joint type.");
        }
      } else if (num_positions == 0) {
        // Deliberately left empty because the joint is removed by welding the
        // parent body to the child body.
      }
    }
  }

  return q;
}

void GlobalInverseKinematics::AddWorldPositionConstraint(
    int body_idx, const Eigen::Vector3d& p_BQ, const Eigen::Vector3d& box_lb_F,
    const Eigen::Vector3d& box_ub_F, const Isometry3d& X_WF) {
  Vector3<Expression> body_pt_pos =
      p_WBo_[body_idx] + R_WB_[body_idx] * p_BQ;
  Vector3<Expression> body_pt_in_measured_frame =
      X_WF.linear().transpose() *
      (body_pt_pos - X_WF.translation());
  AddLinearConstraint(body_pt_in_measured_frame, box_lb_F, box_ub_F);
}

void GlobalInverseKinematics::AddWorldOrientationConstraint(
    int body_idx,
    const Eigen::Quaterniond& desired_orientation,
    double angle_tol) {
  // The rotation matrix error R_e satisfies
  // trace(R_e) = 2 * cos(θ) + 1, where θ is the rotation angle between
  // desired orientation and the current orientation. Thus the constraint is
  // 2 * cos(angle_tol) + 1 <= trace(R_e) <= 3
  Matrix3<Expression> rotation_matrix_err =
  desired_orientation.toRotationMatrix() * R_WB_[body_idx].transpose();
  double lb = angle_tol < M_PI ? 2 * cos(angle_tol) + 1 : -1;
  AddLinearConstraint(rotation_matrix_err.trace(), lb, 3);
}

void GlobalInverseKinematics::AddPostureCost(
    const Eigen::Ref<const Eigen::VectorXd>& q_desired,
    const Eigen::Ref<const Eigen::VectorXd>& body_position_cost,
    const Eigen::Ref<const Eigen::VectorXd>& body_orientation_cost) {
  const int num_bodies = robot_->get_num_bodies();
  if (body_position_cost.rows() != num_bodies) {
    std::ostringstream oss;
    oss << "body_position_cost should have " << num_bodies << " rows.";
    throw std::runtime_error(oss.str());
  }
  if (body_orientation_cost.rows() != num_bodies) {
    std::ostringstream oss;
    oss << "body_orientation_cost should have " << num_bodies << " rows.";
    throw std::runtime_error(oss.str());
  }
  for (int i = 1; i < num_bodies; ++i) {
    if (body_position_cost(i) < 0) {
      std::ostringstream oss;
      oss << "body_position_cost(" << i << ") is negative.";
      throw std::runtime_error(oss.str());
    }
    if (body_orientation_cost(i) < 0) {
      std::ostringstream oss;
      oss << "body_orientation_cost(" << i << ") is negative.";
      throw std::runtime_error(oss.str());
    }
  }
  auto cache = robot_->CreateKinematicsCache();
  cache.initialize(q_desired);
  robot_->doKinematics(cache);

  // Sum up the orientation error for each body to orient_err_sum.
  Expression orient_err_sum(0);
  // p_WBo_err(i) is the slack variable, representing the position error for
  // the (i+1)'th body, which is the Euclidean distance from the body origin
  // position, to the desired position.
  solvers::VectorXDecisionVariable p_WBo_err =
      NewContinuousVariables(num_bodies - 1, "p_WBo_error");
  for (int i = 1; i < num_bodies; ++i) {
    // body 0 is the world. There is no position or orientation error on the
    // world, so we just skip i = 0 and start from i = 1.
    const auto& X_WB_desired = robot_->CalcFramePoseInWorldFrame(
        cache, robot_->get_body(i), Isometry3d::Identity());
    // Add the constraint p_WBo_err(i-1) >= body_position_cost(i) *
    // |p_WBo(i) - p_WBo_desired(i) |
    Vector4<symbolic::Expression> pos_error_expr;
    pos_error_expr << p_WBo_err(i - 1),
        body_position_cost(i) * (p_WBo_[i] - X_WB_desired.translation());
    AddLorentzConeConstraint(pos_error_expr);

    // The orientation error is on the angle θ between the body orientation and
    // the desired orientation, namely 1 - cos(θ).
    // cos(θ) can be computed as (trace( R_WB_desired * R_WB_[i]ᵀ) - 1) / 2
    // To see how the angle is computed from a rotation matrix, please refer to
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
    orient_err_sum +=
        body_orientation_cost(i) *
        (1 - ((X_WB_desired.linear() * R_WB_[i].transpose()).trace() - 1) / 2);
  }

  // The total cost is the summation of the position error and the orientation
  // error.
  AddCost(p_WBo_err.cast<Expression>().sum() + orient_err_sum);
}
}  // namespace multibody
}  // namespace drake
