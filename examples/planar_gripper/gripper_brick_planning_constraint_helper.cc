#include "drake/examples/planar_gripper/gripper_brick_planning_constraint_helper.h"

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
    solvers::MathematicalProgram* prog) {
  const auto& plant = gripper_brick_system.plant();
  const multibody::CoulombFriction<double>& brick_friction =
      plant.default_coulomb_friction(plant.GetCollisionGeometriesForBody(
          gripper_brick_system.brick_frame().body())[0]);
  const multibody::CoulombFriction<double>& finger_tip_friction =
      plant.default_coulomb_friction(plant.GetCollisionGeometriesForBody(
          gripper_brick_system.finger_link2_frame(finger).body())[0]);
  const multibody::CoulombFriction<double> combined_friction =
      multibody::CalcContactFrictionFromSurfaceProperties(brick_friction,
                                                          finger_tip_friction);
  const double mu = combined_friction.static_friction();
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
  prog->AddConstraint(
      std::make_shared<multibody::PositionConstraint>(
          &(gripper_brick_system.plant()), brick, p_BFingertip_lower,
          p_BFingertip_upper, finger_link2, p_L2Fingertip, plant_context),
      q_vars);
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
  return math::initializeAutoDiffGivenGradientMatrix(
      p_BFingertip, Jv_BF2_B * math::autoDiffToGradientMatrix(q));
}

template void AddFrictionConeConstraint<double>(
    const GripperBrickHelper<double>&, Finger, BrickFace,
    const Eigen::Ref<const Vector2<symbolic::Variable>>&,
    solvers::MathematicalProgram*);
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
