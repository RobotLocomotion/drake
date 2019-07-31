#include "drake/examples/planar_gripper/gripper_brick_planning_utils.h"

#include <gtest/gtest.h>

#include "drake/solvers/solve.h"

namespace drake {
namespace examples {
namespace planar_gripper {
GTEST_TEST(AddFrictionConeConstraintTest, Test) {
  GripperBrickHelper<double> gripper_brick;
  solvers::MathematicalProgram prog;
  auto f_Cb_B = prog.NewContinuousVariables<2>();
  AddFrictionConeConstraint(gripper_brick, Finger::kFinger2, BrickFace::kNegY,
                            f_Cb_B, &prog);

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

  const auto& plant = gripper_brick.plant();
  const multibody::CoulombFriction<double>& brick_friction =
      plant.default_coulomb_friction(plant.GetCollisionGeometriesForBody(
          gripper_brick.brick_frame().body())[0]);
  const multibody::CoulombFriction<double>& finger_tip_friction =
      plant.default_coulomb_friction(plant.GetCollisionGeometriesForBody(
          gripper_brick.finger_link2_frame(Finger::kFinger2).body())[0]);
  const multibody::CoulombFriction<double> combined_friction =
      multibody::CalcContactFrictionFromSurfaceProperties(brick_friction,
                                                          finger_tip_friction);
  const double mu = combined_friction.static_friction();
  // Test a force that points in neg Y direction. This force is not in the
  // friction cone.
  check_force_in_cone(Eigen::Vector2d(-1, 0), false);
  // Test a force that points to pos Y direction. This force is in the friction
  // cone.
  check_force_in_cone(Eigen::Vector2d(1, 0), true);

  check_force_in_cone(Eigen::Vector2d(1, 2 * mu), false);
  check_force_in_cone(Eigen::Vector2d(1, 0.9 * mu), true);
}
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
