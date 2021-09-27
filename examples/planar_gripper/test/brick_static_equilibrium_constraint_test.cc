#include "drake/examples/planar_gripper/brick_static_equilibrium_constraint.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/compute_numerical_gradient.h"

namespace drake {
namespace examples {
namespace planar_gripper {
GTEST_TEST(BrickStaticEquilibriumNonlinearConstraint, TestEval) {
  GripperBrickHelper<double> gripper_brick_system;
  std::vector<std::pair<Finger, BrickFace>> finger_face_contacts;
  finger_face_contacts.emplace_back(Finger::kFinger1, BrickFace::kPosZ);
  finger_face_contacts.emplace_back(Finger::kFinger3, BrickFace::kPosY);
  auto diagram_context = gripper_brick_system.diagram().CreateDefaultContext();
  const auto& plant = gripper_brick_system.plant();
  systems::Context<double>* plant_mutable_context =
      &(gripper_brick_system.diagram().GetMutableSubsystemContext(
          gripper_brick_system.plant(), diagram_context.get()));

  BrickStaticEquilibriumNonlinearConstraint constraint(
      gripper_brick_system, finger_face_contacts, plant_mutable_context);

  EXPECT_EQ(constraint.num_outputs(), 3);
  EXPECT_EQ(constraint.num_vars(), plant.num_positions() + 2 * 2);
  EXPECT_TRUE(
      CompareMatrices(constraint.lower_bound(), Eigen::Vector3d::Zero()));
  EXPECT_TRUE(
      CompareMatrices(constraint.upper_bound(), Eigen::Vector3d::Zero()));

  // Test Eval for an arbitrary x.
  Eigen::VectorXd x(13);
  x << 0.1, 0.3, 0.3, -0.4, -1.2, 0.5, 1.3, -0.2, 1.8, 2.5, -0.4, -0.2, -0.6;
  Eigen::VectorXd y;
  constraint.Eval(x, &y);

  Eigen::Matrix<double, 9, 1> q = x.head<9>();
  plant.SetPositions(plant_mutable_context, q);
  const math::RigidTransform<double> X_BW = plant.CalcRelativeTransform(
      *plant_mutable_context, gripper_brick_system.brick_frame(),
      plant.world_frame());
  Eigen::Vector3d y_expected;
  y_expected.head<2>() =
      x.segment<2>(9) + x.segment<2>(11) +
      (X_BW.rotation() *
       Eigen::Vector3d(
           0, 0,
           -gripper_brick_system.brick_frame().body().get_default_mass() *
               multibody::UniformGravityFieldElement<double>::kDefaultStrength))
          .tail<2>();
  Eigen::Vector3d total_torque(0, 0, 0);
  Eigen::Vector3d p_BFingertip0;
  plant.CalcPointsPositions(
      *plant_mutable_context,
      gripper_brick_system.finger_link2_frame(finger_face_contacts[0].first),
      gripper_brick_system.p_L2Fingertip(), gripper_brick_system.brick_frame(),
      &p_BFingertip0);
  Eigen::Vector3d p_BC0 = p_BFingertip0;
  p_BC0(2) -= gripper_brick_system.finger_tip_radius();
  Eigen::Vector3d p_BFingertip1;
  plant.CalcPointsPositions(
      *plant_mutable_context,
      gripper_brick_system.finger_link2_frame(finger_face_contacts[1].first),
      gripper_brick_system.p_L2Fingertip(), gripper_brick_system.brick_frame(),
      &p_BFingertip1);
  Eigen::Vector3d p_BC1 = p_BFingertip1;
  p_BC1(1) -= gripper_brick_system.finger_tip_radius();
  total_torque += p_BC0.cross(Eigen::Vector3d(0, x(9), x(10))) +
                  p_BC1.cross(Eigen::Vector3d(0, x(11), x(12)));
  y_expected(2) = total_torque(0);
  EXPECT_TRUE(CompareMatrices(y, y_expected, 1E-10));

  // Now check Eval with autodiff.
  auto x_ad = math::InitializeAutoDiff(x);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(x_ad, &y_autodiff);

  std::function<void(const Eigen::Ref<const Eigen::VectorXd>&,
                     Eigen::VectorXd*)>
      evaluator = [&constraint](const Eigen::Ref<const Eigen::VectorXd>& x_val,
                                Eigen::VectorXd* y_val) {
        return constraint.Eval(x_val, y_val);
      };
  const auto J = math::ComputeNumericalGradient(evaluator, x);
  EXPECT_TRUE(CompareMatrices(math::ExtractGradient(y_autodiff), J, 1E-5));
}
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
