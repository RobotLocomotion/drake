#include "drake/examples/planar_gripper/gripper_brick_planning_constraint_helper.h"

#include <gtest/gtest.h>

#include "drake/solvers/solve.h"

namespace drake {
namespace examples {
namespace planar_gripper {
GTEST_TEST(AddFrictionConeConstraintTest, Test) {
  GripperBrickHelper<double> gripper_brick;
  solvers::MathematicalProgram prog;
  auto f_Cb_B = prog.NewContinuousVariables<2>();
  const double friction_cone_shrink_factor = 1;
  AddFrictionConeConstraint(gripper_brick, Finger::kFinger2, BrickFace::kNegY,
                            f_Cb_B, friction_cone_shrink_factor, &prog);

  auto check_force_in_cone = [&prog, &f_Cb_B](const Eigen::Vector2d& f_Cb_B_val,
                                              bool is_in_cone) {
    bool is_satisfied = true;
    for (const auto& binding : prog.GetAllConstraints()) {
      Eigen::VectorXd bound_var_val(binding.variables().rows());
      for (int i = 0; i < bound_var_val.rows(); ++i) {
        for (int j = 0; j < 2; ++j) {
          if (binding.variables()(i).get_id() == f_Cb_B(j).get_id()) {
            bound_var_val(i) = f_Cb_B_val(j);
          }
        }
      }
      if (!binding.evaluator()->CheckSatisfied(bound_var_val)) {
        is_satisfied = false;
        break;
      }
    }
    EXPECT_EQ(is_satisfied, is_in_cone);
  };

  const multibody::CoulombFriction<double> combined_friction =
      gripper_brick.GetFingerTipBrickCoulombFriction(Finger::kFinger2);

  const double mu = combined_friction.static_friction();
  // Test a force that points in the neg Y direction. This force is not in the
  // friction cone.
  check_force_in_cone(Eigen::Vector2d(-1, 0), false);
  // Test a force that points to the pos Y direction. This force is in the
  // friction cone.
  check_force_in_cone(Eigen::Vector2d(1, 0), true);

  check_force_in_cone(Eigen::Vector2d(1, 2 * mu), false);
  check_force_in_cone(Eigen::Vector2d(1, 0.9 * mu), true);
}

GTEST_TEST(AddFingerTipInContactWithBrickFaceTest, Test) {
  GripperBrickHelper<double> gripper_brick;
  solvers::MathematicalProgram prog;
  auto diagram_context = gripper_brick.diagram().CreateDefaultContext();
  systems::Context<double>* plant_context =
      &(gripper_brick.diagram().GetMutableSubsystemContext(
          gripper_brick.plant(), diagram_context.get()));

  auto q_vars =
      prog.NewContinuousVariables(gripper_brick.plant().num_positions());

  const double depth = 2e-3;
  AddFingerTipInContactWithBrickFaceConstraint(gripper_brick, Finger::kFinger2,
                                               BrickFace::kNegY, &prog, q_vars,
                                               plant_context, 0.8, depth);

  const auto result = solvers::Solve(prog);
  EXPECT_TRUE(result.is_success());
  const Eigen::VectorXd q_sol = result.GetSolution(q_vars);
  gripper_brick.plant().SetPositions(plant_context, q_sol);
  Eigen::Vector3d p_BFingertip;
  gripper_brick.plant().CalcPointsPositions(
      *plant_context, gripper_brick.finger_link2_frame(Finger::kFinger2),
      gripper_brick.p_L2Fingertip(), gripper_brick.brick_frame(),
      &p_BFingertip);

  EXPECT_LE(p_BFingertip(2), 0.4 * gripper_brick.brick_size()(2) + 1E-5);
  EXPECT_GE(p_BFingertip(2), -0.4 * gripper_brick.brick_size()(2) - 1E-5);
  EXPECT_NEAR(p_BFingertip(1) + gripper_brick.finger_tip_radius(),
              -0.5 * gripper_brick.brick_size()(1) + depth, 1E-5);
}

GTEST_TEST(TestAddFingerNoSlidingConstraint, Test) {
  GripperBrickHelper<double> gripper_brick;
  const Finger finger{Finger::kFinger1};
  const BrickFace face{BrickFace::kNegZ};
  solvers::MathematicalProgram prog;
  auto q_from =
      prog.NewContinuousVariables(gripper_brick.plant().num_positions());
  auto q_to =
      prog.NewContinuousVariables(gripper_brick.plant().num_positions());
  auto diagram_context_from = gripper_brick.diagram().CreateDefaultContext();
  auto diagram_context_to = gripper_brick.diagram().CreateDefaultContext();
  systems::Context<double>* plant_context_from =
      &(gripper_brick.diagram().GetMutableSubsystemContext(
          gripper_brick.plant(), diagram_context_from.get()));
  systems::Context<double>* plant_context_to =
      &(gripper_brick.diagram().GetMutableSubsystemContext(
          gripper_brick.plant(), diagram_context_to.get()));
  const double face_shrink_factor{0.8};
  const double depth{0.001};
  AddFingerTipInContactWithBrickFaceConstraint(
      gripper_brick, finger, face, &prog, q_from, plant_context_from,
      face_shrink_factor, depth);
  const double rolling_angle_bound{0.1 * M_PI};
  AddFingerNoSlidingConstraint(gripper_brick, finger, face, rolling_angle_bound,
                               &prog, plant_context_from, plant_context_to,
                               q_from, q_to, face_shrink_factor, depth);

  // Now solve the problem.
  const auto result = solvers::Solve(prog);
  EXPECT_TRUE(result.is_success());

  // Make sure both "from" posture and "to" postures are in contact with the
  // brick face.
  auto check_finger_in_contact =
      [&gripper_brick, depth, &result, face_shrink_factor](
          const VectorX<symbolic::Variable>& q,
          systems::Context<double>* plant_context) -> math::RigidTransformd {
    const Eigen::VectorXd q_sol = result.GetSolution(q);
    gripper_brick.plant().SetPositions(plant_context, q_sol);
    const math::RigidTransform<double> X_BL2 =
        gripper_brick.plant().CalcRelativeTransform(
            *plant_context, gripper_brick.brick_frame(),
            gripper_brick.finger_link2_frame(finger));
    const Eigen::Vector3d p_BTip = X_BL2 * gripper_brick.p_L2Fingertip();
    const Eigen::Vector3d brick_size = gripper_brick.brick_size();
    EXPECT_NEAR(p_BTip(2),
                -brick_size(2) / 2 - gripper_brick.finger_tip_radius() + depth,
                1E-4);
    EXPECT_GE(p_BTip(1), -brick_size(1) / 2 * face_shrink_factor - 1E-4);
    EXPECT_LE(p_BTip(1), brick_size(1) / 2 * face_shrink_factor + 1E-4);
    return X_BL2;
  };

  const math::RigidTransformd X_BL2_from =
      check_finger_in_contact(q_from, plant_context_from);
  const math::RigidTransformd X_BL2_to =
      check_finger_in_contact(q_to, plant_context_to);

  // Check the orientation difference between "from" posture and "to" posture.
  const Eigen::AngleAxisd angle_axis =
      (X_BL2_from.rotation().inverse() * X_BL2_to.rotation()).ToAngleAxis();
  double theta;
  if (angle_axis.axis().dot(Eigen::Vector3d::UnitX()) > 1 - 1E-4) {
    theta = angle_axis.angle();
  } else if (angle_axis.axis().dot(-Eigen::Vector3d::UnitX()) > 1 - 1E-4) {
    theta = -angle_axis.angle();
  } else {
    throw std::runtime_error("The axis should be either x axis or -x axis.");
  }
  EXPECT_GE(theta, -rolling_angle_bound - 1E-5);
  EXPECT_LE(theta, rolling_angle_bound + 1E-5);

  const Eigen::Vector3d p_BFingertip_from =
      X_BL2_from * gripper_brick.p_L2Fingertip();
  const Eigen::Vector3d p_BFingertip_to =
      X_BL2_to * gripper_brick.p_L2Fingertip();
  EXPECT_NEAR(p_BFingertip_to(1) - p_BFingertip_from(1),
              -gripper_brick.finger_tip_radius() * theta, 1E-5);
}
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
