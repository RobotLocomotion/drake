#include "drake/multibody/inverse_kinematics/global_inverse_kinematics.h"

#include <array>
#include <limits>
#include <stack>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/rotation_constraint.h"

using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

using std::string;

using drake::solvers::VectorDecisionVariable;
using drake::symbolic::Expression;

namespace drake {
namespace multibody {
namespace {
std::map<BodyIndex, JointIndex> GetBodyToJointMap(
    const MultibodyPlant<double>& plant) {
  // First loop through each joint, stores the map from the child body to the
  // joint.
  std::map<BodyIndex, JointIndex> body_to_joint_map;
  for (JointIndex joint_index{0}; joint_index < plant.num_joints();
       ++joint_index) {
    body_to_joint_map.emplace(plant.get_joint(joint_index).child_body().index(),
                              joint_index);
  }
  return body_to_joint_map;
}

std::unordered_set<BodyIndex> GetWeldToWorldBodyIndexSet(
    const MultibodyPlant<double>& plant) {
  const std::vector<const Body<double>*> weld_to_world_bodies =
      plant.GetBodiesWeldedTo(plant.world_body());
  std::unordered_set<BodyIndex> weld_to_world_body_index_set;
  for (const auto weld_body : weld_to_world_bodies) {
    weld_to_world_body_index_set.insert(weld_body->index());
  }
  return weld_to_world_body_index_set;
}
}  // namespace

GlobalInverseKinematics::GlobalInverseKinematics(
    const MultibodyPlant<double>& plant,
    const GlobalInverseKinematics::Options& options)
    : prog_{},
      plant_(plant),
      joint_lower_bounds_{Eigen::VectorXd::Constant(
          plant_.num_positions(), -std::numeric_limits<double>::infinity())},
      joint_upper_bounds_{Eigen::VectorXd::Constant(
          plant_.num_positions(), std::numeric_limits<double>::infinity())} {
  const int num_bodies = plant_.num_bodies();
  R_WB_.resize(num_bodies);
  p_WBo_.resize(num_bodies);
  const Eigen::VectorXd q_lower = plant_.GetPositionLowerLimits();
  const Eigen::VectorXd q_upper = plant_.GetPositionUpperLimits();

  solvers::MixedIntegerRotationConstraintGenerator rotation_generator(
      options.approach, options.num_intervals_per_half_axis,
      options.interval_binning);

  const std::map<BodyIndex, JointIndex> body_to_joint_map =
      GetBodyToJointMap(plant_);
  // Loop through each body in the robot, to add the constraint that the bodies
  // are welded by joints.
  const std::unordered_set<BodyIndex> weld_to_world_body_index_set =
      GetWeldToWorldBodyIndexSet(plant_);
  // This dummy_plant_context is used to compute the pose of a body welded to
  // the world.
  auto dummy_plant_context = plant_.CreateDefaultContext();
  for (BodyIndex body_idx{1}; body_idx < num_bodies; ++body_idx) {
    const Body<double>& body = plant_.get_body(body_idx);
    const string body_R_name = body.name() + "_R";
    const string body_pos_name = body.name() + "_pos";
    p_WBo_[body_idx] = prog_.NewContinuousVariables<3>(body_pos_name);
    // If the body is fixed to the world, then fix the decision variables on
    // the body position and orientation.
    if (weld_to_world_body_index_set.count(body_idx) > 0) {
      // This body is welded to the world.
      R_WB_[body_idx] = prog_.NewContinuousVariables<3, 3>(body_R_name);
      const math::RigidTransform<double> X_WB = plant_.CalcRelativeTransform(
          *dummy_plant_context, plant_.world_frame(), body.body_frame());
      // TODO(hongkai.dai): clean up this for loop using
      // elementwise matrix constraint when it is ready.
      for (int i = 0; i < 3; ++i) {
        prog_.AddBoundingBoxConstraint(X_WB.rotation().matrix().col(i),
                                       X_WB.rotation().matrix().col(i),
                                       R_WB_[body_idx].col(i));
      }
      prog_.AddBoundingBoxConstraint(X_WB.translation(), X_WB.translation(),
                                     p_WBo_[body_idx]);
    } else {
      // This body is not rigidly fixed to the world.
      R_WB_[body_idx] = solvers::NewRotationMatrixVars(&prog_, body_R_name);

      if (!options.linear_constraint_only) {
        solvers::AddRotationMatrixOrthonormalSocpConstraint(&prog_,
                                                            R_WB_[body_idx]);
      }
      if (body.is_floating()) {
        // This is the floating base case, just add the rotation matrix
        // constraint.
        rotation_generator.AddToProgram(R_WB_[body_idx], &prog_);
        // No need to add the kinematic constraint between the parent and the
        // child link. Skip the code below.
        continue;
      }

      // If the body has a parent, then add the constraint to connect the
      // parent body with this body through a joint.
      const auto joint = &(plant_.get_joint(body_to_joint_map.at(body_idx)));
      if (joint->parent_body().index().is_valid()) {
        // Frame Jp is the inboard frame of the joint, rigidly attached to the
        // parent link. Frame Jc is the outboard frame of the joint, rigidly
        // attached to the child link.
        const int parent_idx = joint->parent_body().index();
        const math::RigidTransformd X_PJp =
            joint->frame_on_parent().GetFixedPoseInBodyFrame();
        const math::RigidTransformd X_CJc =
            joint->frame_on_child().GetFixedPoseInBodyFrame();
        if (dynamic_cast<const WeldJoint<double>*>(joint) != nullptr) {
          const WeldJoint<double>* weld_joint =
              dynamic_cast<const WeldJoint<double>*>(joint);

          const math::RigidTransformd X_PC = weld_joint->X_PC();
          // Fixed to the parent body.

          // The position can be computed from the parent body pose.
          // p_WC = p_WP + R_WP * p_PC
          // where C is the child body frame.
          //       P is the parent body frame.
          //       W is the world frame.
          prog_.AddLinearEqualityConstraint(
              p_WBo_[parent_idx] + R_WB_[parent_idx] * X_PC.translation() -
                  p_WBo_[body_idx],
              Vector3d::Zero());

          // The orientation can be computed from the parent body orientation.
          // R_WP * R_PC = R_WC
          Matrix3<Expression> orient_invariance =
              R_WB_[parent_idx] * X_PC.rotation().matrix() - R_WB_[body_idx];
          for (int i = 0; i < 3; ++i) {
            prog_.AddLinearEqualityConstraint(orient_invariance.col(i),
                                              Vector3d::Zero());
          }
        } else if (dynamic_cast<const RevoluteJoint<double>*>(joint) !=
                   nullptr) {
          const RevoluteJoint<double>* revolute_joint =
              dynamic_cast<const RevoluteJoint<double>*>(joint);
          // Adding mixed-integer constraint will add binary variables into
          // the program.
          rotation_generator.AddToProgram(R_WB_[body_idx], &prog_);

          // axis is the vector of the rotation axis in the joint
          // inboard/outboard frame.
          const Vector3d axis = revolute_joint->revolute_axis();

          // Add the constraint R_WC * R_CJc * axis_Jc = R_WP * R_PJp * axis_Jp,
          // where axis_Jc = axis_Jp since the rotation axis is invaraiant in
          // the inboard frame Jp and the outboard frame Jc.
          prog_.AddLinearEqualityConstraint(
              R_WB_[body_idx] * X_CJc.rotation().matrix().transpose() * axis -
                  R_WB_[parent_idx] * X_PJp.rotation().matrix() * axis,
              Vector3d::Zero());

          // The position of the rotation axis is the same on both child and
          // parent bodies.
          prog_.AddLinearEqualityConstraint(
              p_WBo_[parent_idx] + R_WB_[parent_idx] * X_PJp.translation() -
                  p_WBo_[body_idx] + R_WB_[body_idx] * X_CJc.translation(),
              Vector3d::Zero());

          // Now we process the joint limits constraint.
          const double joint_lb = q_lower(revolute_joint->position_start());
          const double joint_ub = q_upper(revolute_joint->position_start());
          AddJointLimitConstraint(body_idx, joint_lb, joint_ub,
                                  options.linear_constraint_only);
        } else {
          throw std::runtime_error("Unsupported joint type.");
        }
      }
    }
  }
}

const solvers::MatrixDecisionVariable<3, 3>&
GlobalInverseKinematics::body_rotation_matrix(BodyIndex body_index) const {
  if (body_index >= plant_.num_bodies() || body_index <= 0) {
    throw std::runtime_error("body index out of range.");
  }
  return R_WB_[body_index];
}

const solvers::VectorDecisionVariable<3>&
GlobalInverseKinematics::body_position(BodyIndex body_index) const {
  if (body_index >= plant_.num_bodies() || body_index <= 0) {
    throw std::runtime_error("body index out of range.");
  }
  return p_WBo_[body_index];
}

void GlobalInverseKinematics::ReconstructGeneralizedPositionSolutionForBody(
    const solvers::MathematicalProgramResult& result, BodyIndex body_idx,
    const std::map<BodyIndex, JointIndex>& body_to_joint_map,
    const std::unordered_set<BodyIndex>& weld_to_world_body_index_set,
    Eigen::Ref<Eigen::VectorXd> q,
    std::vector<Eigen::Matrix3d>* reconstruct_R_WB) const {
  const Body<double>& body = plant_.get_body(body_idx);
  const Matrix3d R_WC = result.GetSolution(R_WB_[body_idx]);
  if (body.is_floating()) {
    // p_WBi is the position of the body frame in the world frame.
    const Vector3d p_WBi = result.GetSolution(p_WBo_[body_idx]);
    const math::RotationMatrix<double> normalized_rotmat =
        math::RotationMatrix<double>::ProjectToRotationMatrix(R_WC);

    q.segment<3>(body.floating_positions_start()) = p_WBi;
    if (body.has_quaternion_dofs()) {
      // The position order is x-y-z-qw-qx-qy-qz, namely translation
      // first, and quaternion second.
      q.segment<4>(body.floating_positions_start() + 3) =
          normalized_rotmat.ToQuaternionAsVector4();
    } else {
      // The position order is x-y-z-roll-pitch-yaw.
      q.segment<3>(body.floating_positions_start() + 3) =
          math::RollPitchYaw<double>(normalized_rotmat).vector();
    }
    reconstruct_R_WB->at(body_idx) = normalized_rotmat.matrix();
    return;
  }
  // This dummy_plant_context is used to compute the pose of a body welded to
  // the world.
  auto dummy_plant_context = plant_.CreateDefaultContext();
  const Joint<double>& joint = plant_.get_joint(body_to_joint_map.at(body_idx));
  const Body<double>& parent = joint.parent_body();
  if (weld_to_world_body_index_set.count(body_idx) == 0) {
    // R_WP is the rotation matrix of parent frame to the world frame.
    const Matrix3d& R_WP = reconstruct_R_WB->at(parent.index());
    const math::RigidTransformd X_PJp =
        joint.frame_on_parent().GetFixedPoseInBodyFrame();
    const math::RigidTransformd X_CJc =
        joint.frame_on_child().GetFixedPoseInBodyFrame();

    // For each different type of joints, use a separate branch to compute
    // the posture for that joint.
    if (joint.num_positions() == 1) {
      const int position_idx = joint.position_start();
      const double joint_lb = joint_lower_bounds_(position_idx);
      const double joint_ub = joint_upper_bounds_(position_idx);
      // Should NOT do this evil dynamic cast here, but currently we do
      // not have a method to tell if a joint is revolute or not.
      if (dynamic_cast<const RevoluteJoint<double>*>(&joint) != nullptr) {
        const RevoluteJoint<double>* revolute_joint =
            dynamic_cast<const RevoluteJoint<double>*>(&joint);
        const Matrix3d R_JpJc = X_PJp.rotation().matrix().transpose() *
                                R_WP.transpose() * R_WC *
                                X_CJc.rotation().matrix();
        // The matrix R_JpJc is very likely not on SO(3). The reason is
        // that we use a relaxation of the rotation matrix, and thus
        // R_WC and R_WP might not lie on SO(3) exactly. Here we need to project
        // R_JpJc to SO(3), with joint axis as the rotation axis, and
        // joint limits as the lower and upper bound on the rotation angle.
        const Vector3d axis_F = revolute_joint->revolute_axis();
        const double revolute_joint_angle = math::ProjectMatToRotMatWithAxis(
            R_JpJc, axis_F, joint_lb, joint_ub);
        q(position_idx) = revolute_joint_angle;
        reconstruct_R_WB->at(body_idx) =
            R_WP * X_PJp.rotation().matrix() *
            Eigen::AngleAxisd(revolute_joint_angle, axis_F).toRotationMatrix() *
            X_CJc.rotation().matrix().transpose();
      } else {
        // TODO(hongkai.dai): add prismatic and helical joints.
        throw std::runtime_error("Unsupported joint type.");
      }
    } else if (joint.num_positions() == 0) {
      // Deliberately left empty because the joint is removed by welding the
      // parent body to the child body.
    }
  } else {
    // The reconstructed body orientation is just the world fixed
    // orientation.
    const math::RigidTransformd X_WB = plant_.CalcRelativeTransform(
        *dummy_plant_context, plant_.world_frame(), body.body_frame());
    reconstruct_R_WB->at(body_idx) = X_WB.rotation().matrix();
  }
}

Eigen::VectorXd GlobalInverseKinematics::ReconstructGeneralizedPositionSolution(
    const solvers::MathematicalProgramResult& result) const {
  Eigen::VectorXd q(plant_.num_positions());
  // First loop through each joint, stores the map from the child body to the
  // joint.
  const std::map<BodyIndex, JointIndex> body_to_joint_map =
      GetBodyToJointMap(plant_);
  const std::unordered_set<BodyIndex> weld_to_world_body_index_set =
      GetWeldToWorldBodyIndexSet(plant_);
  // reconstruct_R_WB[i] is the orientation of body i'th body frame expressed in
  // the world frame, computed from the reconstructed posture.
  std::vector<Eigen::Matrix3d> reconstruct_R_WB(plant_.num_bodies());
  // is_link_visited[i] is set to true, if the angle of the joint on link i has
  // been reconstructed.
  std::vector<bool> is_link_visited(plant_.num_bodies(), false);
  // The first one is the world frame, thus the orientation is identity.
  reconstruct_R_WB[0].setIdentity();
  is_link_visited[0] = true;
  int num_link_visited = 1;
  BodyIndex body_idx{1};
  while (num_link_visited < plant_.num_bodies()) {
    if (!is_link_visited[body_idx]) {
      // unvisited_links records all the unvisited links, along the kinematic
      // path from the root to the body with index body_idx (including
      // body_idx).
      std::stack<BodyIndex> unvisited_links;
      unvisited_links.push(body_idx);
      BodyIndex parent_idx{};
      if (plant_.get_body(body_idx).is_floating()) {
        parent_idx = plant_.world_body().index();
      } else {
        parent_idx = plant_.get_joint(body_to_joint_map.at(body_idx))
                         .parent_body()
                         .index();
      }
      while (!is_link_visited[parent_idx]) {
        unvisited_links.push(parent_idx);
        // Now update parent_idx
        if (plant_.get_body(BodyIndex{parent_idx}).is_floating()) {
          parent_idx = plant_.world_body().index();
        } else {
          parent_idx =
              plant_.get_joint(body_to_joint_map.at(BodyIndex{parent_idx}))
                  .parent_body()
                  .index();
        }
      }
      // Now the link parent_idx has been visited.
      while (!unvisited_links.empty()) {
        const BodyIndex unvisited_link_idx = unvisited_links.top();
        unvisited_links.pop();
        ReconstructGeneralizedPositionSolutionForBody(
            result, unvisited_link_idx, body_to_joint_map,
            weld_to_world_body_index_set, q, &reconstruct_R_WB);
        is_link_visited[unvisited_link_idx] = true;
        ++num_link_visited;
      }
    }
    ++body_idx;
  }
  return q;
}

solvers::Binding<solvers::LinearConstraint>
GlobalInverseKinematics::AddWorldPositionConstraint(
    BodyIndex body_idx, const Eigen::Vector3d& p_BQ,
    const Eigen::Vector3d& box_lb_F, const Eigen::Vector3d& box_ub_F,
    const Isometry3d& X_WF) {
  Vector3<Expression> body_pt_pos = p_WBo_[body_idx] + R_WB_[body_idx] * p_BQ;
  Vector3<Expression> body_pt_in_measured_frame =
      X_WF.linear().transpose() * (body_pt_pos - X_WF.translation());
  return prog_.AddLinearConstraint(body_pt_in_measured_frame, box_lb_F,
                                   box_ub_F);
}

solvers::Binding<solvers::LinearConstraint>
GlobalInverseKinematics::AddWorldOrientationConstraint(
    BodyIndex body_idx, const Eigen::Quaterniond& desired_orientation,
    double angle_tol) {
  // The rotation matrix error R_e satisfies
  // trace(R_e) = 2 * cos(θ) + 1, where θ is the rotation angle between
  // desired orientation and the current orientation. Thus the constraint is
  // 2 * cos(angle_tol) + 1 <= trace(R_e) <= 3
  Matrix3<Expression> rotation_matrix_err =
      desired_orientation.toRotationMatrix() * R_WB_[body_idx].transpose();
  double lb = angle_tol < M_PI ? 2 * cos(angle_tol) + 1 : -1;
  return prog_.AddLinearConstraint(rotation_matrix_err.trace(), lb, 3);
}

void GlobalInverseKinematics::AddPostureCost(
    const Eigen::Ref<const Eigen::VectorXd>& q_desired,
    const Eigen::Ref<const Eigen::VectorXd>& body_position_cost,
    const Eigen::Ref<const Eigen::VectorXd>& body_orientation_cost) {
  const int num_bodies = plant_.num_bodies();
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
  auto context = plant_.CreateDefaultContext();
  plant_.SetPositions(context.get(), q_desired);

  // Sum up the orientation error for each body to orient_err_sum.
  Expression orient_err_sum(0);
  // p_WBo_err(i) is the slack variable, representing the position error for
  // the (i+1)'th body, which is the Euclidean distance from the body origin
  // position, to the desired position.
  solvers::VectorXDecisionVariable p_WBo_err =
      prog_.NewContinuousVariables(num_bodies - 1, "p_WBo_error");
  for (int i = 1; i < num_bodies; ++i) {
    // body 0 is the world. There is no position or orientation error on the
    // world, so we just skip i = 0 and start from i = 1.
    const auto& X_WB_desired = plant_.CalcRelativeTransform(
        *context, plant_.world_frame(),
        plant_.get_body(BodyIndex{i}).body_frame());
    // Add the constraint p_WBo_err(i-1) >= body_position_cost(i) *
    // |p_WBo(i) - p_WBo_desired(i) |
    Vector4<symbolic::Expression> pos_error_expr;
    pos_error_expr << p_WBo_err(i - 1),
        body_position_cost(i) * (p_WBo_[i] - X_WB_desired.translation());
    prog_.AddLorentzConeConstraint(pos_error_expr);

    // The orientation error is on the angle θ between the body orientation and
    // the desired orientation, namely 1 - cos(θ).
    // cos(θ) can be computed as (trace( R_WB_desired * R_WB_[i]ᵀ) - 1) / 2
    // To see how the angle is computed from a rotation matrix, please refer to
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
    orient_err_sum +=
        body_orientation_cost(i) *
        (1 -
         ((X_WB_desired.rotation().matrix() * R_WB_[i].transpose()).trace() -
          1) /
             2);
  }

  // The total cost is the summation of the position error and the orientation
  // error.
  prog_.AddCost(p_WBo_err.cast<Expression>().sum() + orient_err_sum);
}

solvers::VectorXDecisionVariable
GlobalInverseKinematics::BodyPointInOneOfRegions(
    BodyIndex body_index, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    const std::vector<Eigen::Matrix3Xd>& region_vertices) {
  const auto& R_WB = body_rotation_matrix(body_index);
  const auto& p_WBo = body_position(body_index);
  const int num_regions = region_vertices.size();
  const string& body_name = plant_.get_body(body_index).name();
  solvers::VectorXDecisionVariable z =
      prog_.NewBinaryVariables(num_regions, "z_" + body_name);
  std::vector<solvers::VectorXDecisionVariable> w(num_regions);

  // We will write p_WQ in two ways, we first write p_WQ as
  // sum_i (w_i1 * v_i1 + w_i2 * v_i2 + ... + w_in * v_in). As the convex
  // combination of vertices in one of the regions.
  Vector3<symbolic::Expression> p_WQ;
  p_WQ << 0, 0, 0;
  for (int i = 0; i < num_regions; ++i) {
    const int num_vertices_i = region_vertices[i].cols();
    if (num_vertices_i < 3) {
      throw std::runtime_error("Each region should have at least 3 vertices.");
    }
    w[i] = prog_.NewContinuousVariables(
        num_vertices_i, "w_" + body_name + "_region_" + std::to_string(i));
    prog_.AddLinearConstraint(w[i].cast<symbolic::Expression>().sum() - z(i) ==
                              0);
    prog_.AddBoundingBoxConstraint(Eigen::VectorXd::Zero(num_vertices_i),
                                   Eigen::VectorXd::Ones(num_vertices_i), w[i]);
    p_WQ += region_vertices[i] * w[i];
  }

  prog_.AddLinearConstraint(z.cast<symbolic::Expression>().sum() == 1);

  // p_WQ must match the body pose, as p_WQ = p_WBo + R_WB * p_BQ
  prog_.AddLinearEqualityConstraint(p_WBo + R_WB * p_BQ - p_WQ,
                                    Eigen::Vector3d::Zero());

  return z;
}

solvers::VectorXDecisionVariable
GlobalInverseKinematics::BodySphereInOneOfPolytopes(
    BodyIndex body_index, const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
    double radius,
    const std::vector<GlobalInverseKinematics::Polytope3D>& polytopes) {
  DRAKE_DEMAND(radius >= 0);
  const int num_polytopes = static_cast<int>(polytopes.size());
  const auto z = prog_.NewBinaryVariables(num_polytopes, "z");
  // z1 + ... + zn = 1
  prog_.AddLinearEqualityConstraint(Eigen::RowVectorXd::Ones(num_polytopes), 1,
                                    z);

  const auto y =
      prog_.NewContinuousVariables<3, Eigen::Dynamic>(3, num_polytopes, "y");
  const Vector3<symbolic::Expression> p_WQ =
      p_WBo_[body_index] + R_WB_[body_index] * p_BQ;
  // p_WQ = y.col(0) + ... + y.col(n)
  prog_.AddLinearEqualityConstraint(
      p_WQ - y.cast<symbolic::Expression>().rowwise().sum(),
      Eigen::Vector3d::Zero());

  for (int i = 0; i < num_polytopes; ++i) {
    DRAKE_DEMAND(polytopes[i].A.rows() == polytopes[i].b.rows());
    prog_.AddLinearConstraint(
        polytopes[i].A * y.col(i) <=
        (polytopes[i].b - polytopes[i].A.rowwise().norm() * radius) * z(i));
  }

  return z;
}

// Approximate a quadratic constraint (which could be formulated as a Lorentz
// cone constraint) xᵀx ≤ c² by
// -c ≤ xᵢ ≤ c
// ± xᵢ ± xⱼ ≤ √2 * c
// ± x₀ ± x₁ ± x₂ ≤ √3 * c
// These linear approximation are obtained as the tangential planes at some
// points on the surface of the sphere xᵀx ≤ c².
void ApproximateBoundedNormByLinearConstraints(
    const Eigen::Ref<const Vector3<symbolic::Expression>>& x, double c,
    solvers::MathematicalProgram* prog) {
  DRAKE_DEMAND(c >= 0);
  // -c ≤ xᵢ ≤ c
  prog->AddLinearConstraint(x, Eigen::Vector3d::Constant(-c),
                            Eigen::Vector3d::Constant(c));
  const double sqrt2_c = std::sqrt(2) * c;
  const double sqrt3_c = std::sqrt(3) * c;
  // ± xᵢ ± xⱼ ≤ √2 * c
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      prog->AddLinearConstraint(x(i) + x(j), -sqrt2_c, sqrt2_c);
      prog->AddLinearConstraint(x(i) - x(j), -sqrt2_c, sqrt2_c);
    }
  }
  // ± x₀ ± x₁ ± x₂ ≤ √3 * c
  prog->AddLinearConstraint(x(0) + x(1) + x(2), -sqrt3_c, sqrt3_c);
  prog->AddLinearConstraint(x(0) + x(1) - x(2), -sqrt3_c, sqrt3_c);
  prog->AddLinearConstraint(x(0) - x(1) + x(2), -sqrt3_c, sqrt3_c);
  prog->AddLinearConstraint(x(0) - x(1) - x(2), -sqrt3_c, sqrt3_c);
}

void GlobalInverseKinematics::AddJointLimitConstraint(
    BodyIndex body_index, double joint_lower_bound, double joint_upper_bound,
    bool linear_constraint_approximation) {
  if (joint_lower_bound > joint_upper_bound) {
    throw std::runtime_error(
        "The joint lower bound should be no larger than the upper bound.");
  }
  if (plant_.get_body(body_index).is_floating()) {
    throw std::runtime_error(
        "The body is floating, do not use AddJointLimitConstraint(), impose "
        "the bounds on R_WB and p_WB directly.");
  }
  const Joint<double>* joint{nullptr};
  for (JointIndex joint_index{0}; joint_index < plant_.num_joints();
       ++joint_index) {
    if (plant_.get_joint(joint_index).child_body().index() == body_index) {
      joint = &(plant_.get_joint(joint_index));
      break;
    }
  }
  if (joint == nullptr) {
    throw std::runtime_error(
        fmt::format("The body {} is not the child of any joint in the plant.",
                    plant_.get_body(body_index).name()));
  }
  const Body<double>& parent = joint->parent_body();
  const int parent_idx = parent.index();
  const math::RigidTransformd X_PJp =
      joint->frame_on_parent().GetFixedPoseInBodyFrame();
  const math::RigidTransformd X_CJc =
      joint->frame_on_child().GetFixedPoseInBodyFrame();
  switch (joint->num_velocities()) {
    case 0: {
      // Fixed to the parent body.
      throw std::runtime_error("Cannot impose joint limits for a fixed joint.");
    }
    case 1: {
      // If the new bound [joint_lower_bound joint_upper_bound] is not tighter
      // than the existing bound, then we ignore it, without adding new
      // constraints.
      bool is_limits_tightened = false;
      const int position_idx = joint->position_start();
      if (joint_lower_bound > joint_lower_bounds_(position_idx)) {
        joint_lower_bounds_(position_idx) = joint_lower_bound;
        is_limits_tightened = true;
      }
      if (joint_upper_bound < joint_upper_bounds_(position_idx)) {
        joint_upper_bounds_(position_idx) = joint_upper_bound;
        is_limits_tightened = true;
      }
      if (is_limits_tightened) {
        // Should NOT do this evil dynamic cast here, but currently we do
        // not have a method to tell if a joint is revolute or not.
        if (dynamic_cast<const RevoluteJoint<double>*>(joint) != nullptr) {
          const auto* revolute_joint =
              dynamic_cast<const RevoluteJoint<double>*>(joint);
          // axis_F is the vector of the rotation axis in the joint
          // inboard/outboard frame.
          const Vector3d axis_F = revolute_joint->revolute_axis();

          // Now we process the joint limits constraint.
          const double joint_bound = (joint_upper_bounds_[position_idx] -
                                      joint_lower_bounds_[position_idx]) /
                                     2;

          if (joint_bound < M_PI) {
            // We use the fact that if the angle between two unit length
            // vectors u and v is smaller than α, it is equivalent to
            // |u - v| <= 2*sin(α/2)
            // which is a second order cone constraint.

            // If the rotation angle θ satisfies
            // a <= θ <= b
            // This is equivalent to
            // -α <= θ - (a+b)/2 <= α
            // where α = (b-a) / 2, (a+b) / 2 is the joint offset, such that
            // the bounds on β = θ - (a+b)/2 are symmetric.
            // We use the following notation:
            // R_WP     The rotation matrix of parent frame `P` to world
            //          frame `W`.
            // R_WC     The rotation matrix of child frame `C` to world
            //          frame `W`.
            // R_PJp    The rotation matrix of joint frame `Jp` to parent
            //          frame `P`.
            // R(k, β)  The rotation matrix along joint axis k by angle β.
            // R_JcC    The rotation matrix of child frame `C` to the
            //          outboard frame `Jc`.
            // The kinematics constraint is
            // R_WP * R_PJp * R(k, θ) * R_JcC = R_WC.
            // This is equivalent to
            // R_WP * R_PJp * R(k, (a+b)/2) * R(k, β)) = R_WC * R_CJc.
            // So to constrain that -α <= β <= α,
            // we can constrain the angle between the two vectors
            // R_WC * R_CJc * v and R_WP * R_PJp * R(k,(a+b)/2) * v is no larger
            // than α, where v is a unit length vector perpendicular to the
            // rotation axis k, in the joint frame. Thus we can constrain that
            // |R_WC*R_CJc*v - R_WP * R_PJp * R(k,(a+b)/2)*v | <= 2*sin (α / 2)
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

            // The constraint would be tighter, if we choose many unit
            // length vector `v`, perpendicular to the joint axis, in the
            // joint frame. Here to balance between the size of the
            // optimization problem, and the tightness of the convex
            // relaxation, we just use four vectors in `v`. Notice that
            // v_basis contains the orthonormal basis of the null space
            // null(axis_F).
            std::array<Eigen::Vector3d, 2> v_basis = {{v_C, axis_F.cross(v_C)}};
            v_basis[1] /= v_basis[1].norm();

            std::array<Eigen::Vector3d, 4> v_samples;
            v_samples[0] = v_basis[0];
            v_samples[1] = v_basis[1];
            v_samples[2] = v_basis[0] + v_basis[1];
            v_samples[2] /= v_samples[2].norm();
            v_samples[3] = v_basis[0] - v_basis[1];
            v_samples[3] /= v_samples[3].norm();

            // rotmat_joint_offset is R(k, (a+b)/2) explained above.
            const Matrix3d rotmat_joint_offset =
                Eigen::AngleAxisd((joint_lower_bounds_[position_idx] +
                                   joint_upper_bounds_[position_idx]) /
                                      2,
                                  axis_F)
                    .toRotationMatrix();

            // joint_limit_expr is going to be within the Lorentz cone.
            Eigen::Matrix<Expression, 4, 1> joint_limit_expr;
            const double joint_limit_lorentz_rhs = 2 * sin(joint_bound / 2);
            joint_limit_expr(0) = joint_limit_lorentz_rhs;
            for (const auto& v : v_samples) {
              // joint_limit_expr.tail<3> is
              // R_WC * R_CJc * v - R_WP * R_PJp * R(k,(a+b)/2) * v mentioned
              // above.
              joint_limit_expr.tail<3>() =
                  R_WB_[body_index] * X_CJc.rotation().matrix() * v -
                  R_WB_[parent_idx] * X_PJp.rotation().matrix() *
                      rotmat_joint_offset * v;
              if (linear_constraint_approximation) {
                ApproximateBoundedNormByLinearConstraints(
                    joint_limit_expr.tail<3>(), joint_limit_lorentz_rhs,
                    &prog_);

              } else {
                prog_.AddLorentzConeConstraint(joint_limit_expr);
              }
            }

            const std::unordered_set<BodyIndex> weld_to_world_body_index_set =
                GetWeldToWorldBodyIndexSet(plant_);
            // This dummy_plant_context is used to compute the pose of a body
            // welded to the world.
            auto dummy_plant_context = plant_.CreateDefaultContext();
            if (weld_to_world_body_index_set.count(BodyIndex{parent_idx}) > 0) {
              const math::RigidTransformd X_WP = plant_.CalcRelativeTransform(
                  *dummy_plant_context, plant_.world_frame(),
                  plant_.get_body(BodyIndex{parent_idx}).body_frame());
              // If the parent body is rigidly fixed to the world. Then we
              // can impose a tighter constraint. Based on the derivation
              // above, we have
              // R(k, β) = [R_WP * R_PJp * R(k, (a+b)/2)]ᵀ * R_WC * R_CJc
              // as a linear expression of the decision variable R_WC
              // (notice that R_WP is constant, since the parent body is
              // rigidly fixed to the world.
              // Any unit length vector `v` that is perpendicular to
              // joint axis `axis_F` in the joint Frame, can be written as
              //   v = V * u, uᵀ * u = 1
              // where V = [v_basis[0] v_basis[1]] containing the basis
              // vectors for the linear space Null(axis_F).
              // On the other hand, we know
              //   vᵀ * R(k, β) * v = cos(β) >= cos(α)
              // due to the joint limits constraint
              //   -α <= β <= α.
              // So we have the condition that
              // uᵀ * u = 1
              //    => uᵀ * Vᵀ * R(k, β) * V * u >= cos(α)
              // Using S-lemma, we know this implication is equivalent to
              // Vᵀ * [R(k, β) + R(k, β)ᵀ]/2 * V - cos(α) * I is p.s.d
              // We let a 2 x 2 matrix
              //   M = Vᵀ * [R(k, β) + R(k, β)ᵀ]/2 * V - cos(α) * I
              // A 2 x 2 matrix M being positive semidefinite (p.s.d) is
              // equivalent to the condition that
              // [M(0, 0), M(1, 1), M(1, 0)] is in the rotated Lorentz cone.
              // R_joint_beta is R(k, β) in the documentation.
              Eigen::Matrix<symbolic::Expression, 3, 3> R_joint_beta =
                  (X_WP.rotation().matrix() * X_PJp.rotation().matrix() *
                   rotmat_joint_offset)
                      .transpose() *
                  R_WB_[body_index] * X_CJc.rotation().matrix();
              const double joint_bound_cos{std::cos(joint_bound)};
              if (!linear_constraint_approximation) {
                Eigen::Matrix<double, 3, 2> V;
                V << v_basis[0], v_basis[1];
                const Eigen::Matrix<symbolic::Expression, 2, 2> M =
                    V.transpose() * (R_joint_beta + R_joint_beta.transpose()) /
                        2 * V -
                    joint_bound_cos * Eigen::Matrix2d::Identity();
                prog_.AddRotatedLorentzConeConstraint(
                    Vector3<symbolic::Expression>(M(0, 0), M(1, 1), M(1, 0)));
              }

              // From Rodriguez formula, we know that -α <= β <= α implies
              // trace(R(k, β)) = 1 + 2 * cos(β) >= 1 + 2*cos(α)
              // So we can impose the constraint
              // 1+2*cos(α) ≤ trace(R(k, β))
              const symbolic::Expression R_joint_beta_trace{
                  R_joint_beta.trace()};
              prog_.AddLinearConstraint(R_joint_beta_trace >=
                                        1 + 2 * joint_bound_cos);
            }
          }
        } else {
          // TODO(hongkai.dai): add prismatic and helical joint.
          throw std::runtime_error("Unsupported joint type.");
        }
      }
      break;
    }
    case 6: {
      break;
    }
    default:
      throw std::runtime_error("Unsupported joint type.");
  }
}
}  // namespace multibody
}  // namespace drake
