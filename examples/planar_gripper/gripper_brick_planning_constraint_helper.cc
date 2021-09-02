#include "drake/examples/planar_gripper/gripper_brick_planning_constraint_helper.h"

#include <limits>
#include <memory>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_constraint_utilities.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"

namespace drake {
namespace examples {
namespace planar_gripper {
template <typename T>
void AddFrictionConeConstraint(
    const GripperBrickHelper<T>& gripper_brick_system, const Finger finger,
    const BrickFace brick_face,
    const Eigen::Ref<const Vector2<symbolic::Variable>>& f_Cb_B,
    double friction_cone_shrink_factor, solvers::MathematicalProgram* prog) {
  const multibody::CoulombFriction<double> combined_friction =
      gripper_brick_system.GetFingerTipBrickCoulombFriction(finger);
  const double mu =
      combined_friction.static_friction() * friction_cone_shrink_factor;
  switch (brick_face) {
    case BrickFace::kNegY: {
      prog->AddLinearConstraint(f_Cb_B(0) >= 0);
      prog->AddLinearConstraint(f_Cb_B(1) <= mu * f_Cb_B(0));
      prog->AddLinearConstraint(f_Cb_B(1) >= -mu * f_Cb_B(0));
      break;
    }
    case BrickFace::kNegZ: {
      prog->AddLinearConstraint(f_Cb_B(1) >= 0);
      prog->AddLinearConstraint(f_Cb_B(0) <= mu * f_Cb_B(1));
      prog->AddLinearConstraint(f_Cb_B(0) >= -mu * f_Cb_B(1));
      break;
    }
    case BrickFace::kPosY: {
      prog->AddLinearConstraint(f_Cb_B(0) <= 0);
      prog->AddLinearConstraint(f_Cb_B(1) <= -mu * f_Cb_B(0));
      prog->AddLinearConstraint(f_Cb_B(1) >= mu * f_Cb_B(0));
      break;
    }
    case BrickFace::kPosZ: {
      prog->AddLinearConstraint(f_Cb_B(1) <= 0);
      prog->AddLinearConstraint(f_Cb_B(0) <= -mu * f_Cb_B(1));
      prog->AddLinearConstraint(f_Cb_B(0) >= mu * f_Cb_B(1));
      break;
    }
  }
}

void AddFingerTipInContactWithBrickFaceConstraint(
    const GripperBrickHelper<double>& gripper_brick_system, Finger finger,
    BrickFace brick_face, solvers::MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    systems::Context<double>* plant_context, double face_shrink_factor,
    double depth) {
  const multibody::Frame<double>& finger_link2 =
      gripper_brick_system.finger_link2_frame(finger);
  // position of finger tip in the finger link 2 farme (F2).
  const Eigen::Vector3d p_L2Fingertip = gripper_brick_system.p_L2Fingertip();
  const multibody::Frame<double>& brick = gripper_brick_system.brick_frame();
  const Eigen::Vector3d brick_size = gripper_brick_system.brick_size();
  Eigen::Vector3d p_BFingertip_lower = -brick_size * face_shrink_factor / 2;
  Eigen::Vector3d p_BFingertip_upper = brick_size * face_shrink_factor / 2;
  const double finger_tip_radius = gripper_brick_system.finger_tip_radius();
  switch (brick_face) {
    case BrickFace::kPosZ: {
      p_BFingertip_lower(2) = brick_size(2) / 2 + finger_tip_radius - depth;
      p_BFingertip_upper(2) = brick_size(2) / 2 + finger_tip_radius - depth;
      break;
    }
    case BrickFace::kNegZ: {
      p_BFingertip_lower(2) = -brick_size(2) / 2 - finger_tip_radius + depth;
      p_BFingertip_upper(2) = -brick_size(2) / 2 - finger_tip_radius + depth;
      break;
    }
    case BrickFace::kPosY: {
      p_BFingertip_lower(1) = brick_size(1) / 2 + finger_tip_radius - depth;
      p_BFingertip_upper(1) = brick_size(1) / 2 + finger_tip_radius - depth;
      break;
    }
    case BrickFace::kNegY: {
      p_BFingertip_lower(1) = -brick_size(1) / 2 - finger_tip_radius + depth;
      p_BFingertip_upper(1) = -brick_size(1) / 2 - finger_tip_radius + depth;
      break;
    }
  }
  auto constraint = prog->AddConstraint(
      std::make_shared<multibody::PositionConstraint>(
          &(gripper_brick_system.plant()), brick, p_BFingertip_lower,
          p_BFingertip_upper, finger_link2, p_L2Fingertip, plant_context),
      q_vars);
  constraint.evaluator()->set_description(to_string(finger) +
                                          "_tip_in_contact");
}

Eigen::Vector3d ComputeFingerTipInBrickFrame(
    const GripperBrickHelper<double>& gripper_brick, const Finger finger,
    const systems::Context<double>& plant_context,
    const Eigen::Ref<const Eigen::VectorXd>&) {
  Eigen::Vector3d p_BFingertip;
  gripper_brick.plant().CalcPointsPositions(
      plant_context, gripper_brick.finger_link2_frame(finger),
      gripper_brick.p_L2Fingertip(), gripper_brick.brick_frame(),
      &p_BFingertip);
  return p_BFingertip;
}

Vector3<AutoDiffXd> ComputeFingerTipInBrickFrame(
    const GripperBrickHelper<double>& gripper_brick, const Finger finger,
    const systems::Context<double>& plant_context,
    const Eigen::Ref<const AutoDiffVecXd>& q) {
  Eigen::Vector3d p_BFingertip;
  gripper_brick.plant().CalcPointsPositions(
      plant_context, gripper_brick.finger_link2_frame(finger),
      gripper_brick.p_L2Fingertip(), gripper_brick.brick_frame(),
      &p_BFingertip);
  Eigen::Matrix3Xd Jv_BF2_B(3, gripper_brick.plant().num_positions());
  gripper_brick.plant().CalcJacobianTranslationalVelocity(
      plant_context, multibody::JacobianWrtVariable::kQDot,
      gripper_brick.finger_link2_frame(finger), gripper_brick.p_L2Fingertip(),
      gripper_brick.brick_frame(), gripper_brick.brick_frame(), &Jv_BF2_B);
  return math::InitializeAutoDiff(
      p_BFingertip, Jv_BF2_B * math::ExtractGradient(q));
}

namespace internal {
FingerNoSlidingConstraint::FingerNoSlidingConstraint(
    const GripperBrickHelper<double>* gripper_brick, Finger finger,
    BrickFace face, systems::Context<double>* from_context,
    systems::Context<double>* to_context)
    : solvers::Constraint(1 /* number of constraint */,
                          2 * gripper_brick->plant()
                                  .num_positions(),  // Number of variables,
                                                     // the variables are q_to
                                                     // and q_from.
                          Vector1d(0), Vector1d(0),
                          "finger_no_sliding_constraint"),
      gripper_brick_(gripper_brick),
      finger_(finger),
      face_(face),
      from_context_(from_context),
      to_context_(to_context) {}

template <typename T>
void FingerNoSlidingConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  y->resize(1);
  const int nq = gripper_brick_->plant().num_positions();
  const auto& q_from = x.head(nq);
  const auto& q_to = x.tail(nq);
  multibody::internal::UpdateContextConfiguration(
      from_context_, gripper_brick_->plant(), q_from);
  multibody::internal::UpdateContextConfiguration(
      to_context_, gripper_brick_->plant(), q_to);

  const Vector3<T> p_BTip_from = ComputeFingerTipInBrickFrame(
      *gripper_brick_, finger_, *from_context_, x.head(nq));
  const Vector3<T> p_BTip_to = ComputeFingerTipInBrickFrame(
      *gripper_brick_, finger_, *to_context_, x.tail(nq));
  const T theta_from = gripper_brick_->CalcFingerLink2Orientation(
      finger_, T(q_from(gripper_brick_->finger_base_position_index(finger_))),
      T(q_from(gripper_brick_->finger_mid_position_index(finger_))));
  const T theta_to = gripper_brick_->CalcFingerLink2Orientation(
      finger_, T(q_to(gripper_brick_->finger_base_position_index(finger_))),
      T(q_to(gripper_brick_->finger_mid_position_index(finger_))));
  switch (face_) {
    case BrickFace::kPosY:
    case BrickFace::kNegY: {
      // rolling with positive delta_theta about x axis causes positive
      // translation along the z axis.
      (*y)(0) = p_BTip_to(2) - p_BTip_from(2) -
                gripper_brick_->finger_tip_radius() * (theta_to - theta_from);
      break;
    }
    case BrickFace::kPosZ:
    case BrickFace::kNegZ: {
      // rolling with positive delta_theta about x axis causes negative
      // translation along the y axis.
      (*y)(0) = -(p_BTip_to(1) - p_BTip_from(1)) -
                gripper_brick_->finger_tip_radius() * (theta_to - theta_from);
      break;
    }
  }
}

void FingerNoSlidingConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric<double>(x, y);
}

void FingerNoSlidingConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                       AutoDiffVecXd* y) const {
  DoEvalGeneric<AutoDiffXd>(x, y);
}

}  // namespace internal

void AddFingerNoSlidingConstraint(
    const GripperBrickHelper<double>& gripper_brick, Finger finger,
    BrickFace face, double rolling_angle_bound,
    solvers::MathematicalProgram* prog, systems::Context<double>* from_context,
    systems::Context<double>* to_context,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_from,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_to,
    double face_shrink_factor, double depth) {
  AddFingerTipInContactWithBrickFaceConstraint(gripper_brick, finger, face,
                                               prog, q_to, to_context,
                                               face_shrink_factor, depth);

  auto constraint = prog->AddConstraint(
      std::make_shared<internal::FingerNoSlidingConstraint>(
          &gripper_brick, finger, face, from_context, to_context),
      {q_from, q_to});
  constraint.evaluator()->set_description(to_string(finger) + "no_sliding");
  const symbolic::Expression theta_from =
      gripper_brick.CalcFingerLink2Orientation<symbolic::Expression>(
          finger, q_from(gripper_brick.finger_base_position_index(finger)),
          q_from(gripper_brick.finger_mid_position_index(finger)));
  const symbolic::Expression theta_to =
      gripper_brick.CalcFingerLink2Orientation<symbolic::Expression>(
          finger, q_to(gripper_brick.finger_base_position_index(finger)),
          q_to(gripper_brick.finger_mid_position_index(finger)));
  prog->AddLinearConstraint(theta_from - theta_to, -rolling_angle_bound,
                            rolling_angle_bound);
}

namespace internal {
FingerNoSlidingFromFixedPostureConstraint::
    FingerNoSlidingFromFixedPostureConstraint(
        const GripperBrickHelper<double>* gripper_brick, Finger finger,
        BrickFace face, const systems::Context<double>* from_context,
        systems::Context<double>* to_context)
    : solvers::Constraint(1, gripper_brick->plant().num_positions(),
                          Vector1d(0), Vector1d(0),
                          "finger_no_sliding_from_fixed_posture"),
      gripper_brick_{gripper_brick},
      finger_{finger},
      face_{face},
      from_context_{from_context},
      to_context_{to_context} {}

template <typename T>
void FingerNoSlidingFromFixedPostureConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y) const {
  y->resize(1);
  multibody::internal::UpdateContextConfiguration(to_context_,
                                                  gripper_brick_->plant(), x);
  Eigen::VectorXd q_from = gripper_brick_->plant().GetPositions(*from_context_);
  const auto& q_to = x;

  const Vector3<double> p_BTip_from = ComputeFingerTipInBrickFrame(
      *gripper_brick_, finger_, *from_context_, q_from);
  const Vector3<T> p_BTip_to =
      ComputeFingerTipInBrickFrame(*gripper_brick_, finger_, *to_context_, x);
  const T theta_from = gripper_brick_->CalcFingerLink2Orientation(
      finger_, T(q_from(gripper_brick_->finger_base_position_index(finger_))),
      T(q_from(gripper_brick_->finger_mid_position_index(finger_))));
  const T theta_to = gripper_brick_->CalcFingerLink2Orientation(
      finger_, T(q_to(gripper_brick_->finger_base_position_index(finger_))),
      T(q_to(gripper_brick_->finger_mid_position_index(finger_))));
  switch (face_) {
    case BrickFace::kPosY:
    case BrickFace::kNegY: {
      // rolling with positive delta_theta about x axis causes positive
      // translation along the z axis.
      (*y)(0) = p_BTip_to(2) - p_BTip_from(2) -
                gripper_brick_->finger_tip_radius() * (theta_to - theta_from);
      break;
    }
    case BrickFace::kPosZ:
    case BrickFace::kNegZ: {
      // rolling with positive delta_theta about x axis causes negative
      // translation along the y axis.
      (*y)(0) = -(p_BTip_to(1) - p_BTip_from(1)) -
                gripper_brick_->finger_tip_radius() * (theta_to - theta_from);
      break;
    }
  }
}

void FingerNoSlidingFromFixedPostureConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric<double>(x, y);
}

void FingerNoSlidingFromFixedPostureConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DoEvalGeneric<AutoDiffXd>(x, y);
}
}  // namespace internal

void AddFingerNoSlidingFromFixedPostureConstraint(
    const GripperBrickHelper<double>& gripper_brick, Finger finger,
    BrickFace face, double rolling_angle_lower, double rolling_angle_upper,
    solvers::MathematicalProgram* prog,
    const systems::Context<double>& from_context,
    systems::Context<double>* to_context,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_to,
    double face_shrink_factor, double depth) {
  AddFingerTipInContactWithBrickFaceConstraint(gripper_brick, finger, face,
                                               prog, q_to, to_context,
                                               face_shrink_factor, depth);

  const Eigen::VectorXd& q_from =
      gripper_brick.plant().GetPositions(from_context);
  prog->AddConstraint(
      std::make_shared<internal::FingerNoSlidingFromFixedPostureConstraint>(
          &gripper_brick, finger, face, &from_context, to_context),
      {q_to});
  const double theta_from = gripper_brick.CalcFingerLink2Orientation<double>(
      finger, q_from(gripper_brick.finger_base_position_index(finger)),
      q_from(gripper_brick.finger_mid_position_index(finger)));
  const symbolic::Expression theta_to =
      gripper_brick.CalcFingerLink2Orientation<symbolic::Expression>(
          finger, q_to(gripper_brick.finger_base_position_index(finger)),
          q_to(gripper_brick.finger_mid_position_index(finger)));
  prog->AddLinearConstraint(theta_from - theta_to, rolling_angle_lower,
                            rolling_angle_upper);
}

template void AddFrictionConeConstraint<double>(
    const GripperBrickHelper<double>&, Finger, BrickFace,
    const Eigen::Ref<const Vector2<symbolic::Variable>>&, double,
    solvers::MathematicalProgram*);
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
