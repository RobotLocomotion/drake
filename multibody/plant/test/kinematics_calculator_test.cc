#include "drake/multibody/plant/kinematics_calculator.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace {

const char kIiwaFilePath[] =
    "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf";

GTEST_TEST(KinematicsCalculatorTest, Test) {
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelFromFile(FindResourceOrThrow(kIiwaFilePath));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"));
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  Eigen::VectorXd x(plant.num_multibody_states());
  for (int i = 0; i < x.size(); i++) {
    x[i] = static_cast<double>(i) / 10.0;
  }
  plant.SetPositionsAndVelocities(plant_context.get(), x);

  KinematicsCalculator<double> dut(&plant);
  auto context = dut.CreateDefaultContext();
  dut.get_input_port(0).FixValue(context.get(), x);
  const auto& output =
      dut.get_output_port(0).Eval<systems::Context<double>>(*context);

  const Frame<double>& link_7 = plant.GetFrameByName("iiwa_link_7");

  // Check position.
  math::RigidTransform<double> X_W7_expected =
      plant.CalcRelativeTransform(*plant_context, plant.world_frame(), link_7);
  math::RigidTransform<double> X_W7_result =
      plant.CalcRelativeTransform(output, plant.world_frame(), link_7);

  EXPECT_TRUE(X_W7_expected.IsExactlyEqualTo(X_W7_result));

  // Check velocity.
  const SpatialVelocity<double>& V_W7_expected =
      plant.EvalBodySpatialVelocityInWorld(*plant_context, link_7.body());
  const SpatialVelocity<double>& V_W7_result =
      plant.EvalBodySpatialVelocityInWorld(output, link_7.body());

  EXPECT_TRUE(CompareMatrices(V_W7_expected.rotational(),
                              V_W7_result.rotational(), 1e-14));
  EXPECT_TRUE(CompareMatrices(V_W7_expected.translational(),
                              V_W7_result.translational(), 1e-14));

  // Check Jacobian.
  Eigen::MatrixXd J_expected(6, plant.num_velocities());
  plant.CalcJacobianSpatialVelocity(
      *plant_context, multibody::JacobianWrtVariable::kV, link_7,
      Vector3<double>::Zero(), plant.world_frame(), plant.world_frame(),
      &J_expected);
  Eigen::MatrixXd J_result(6, plant.num_velocities());
  plant.CalcJacobianSpatialVelocity(output, multibody::JacobianWrtVariable::kV,
                                    link_7, Vector3<double>::Zero(),
                                    plant.world_frame(), plant.world_frame(),
                                    &J_result);
  EXPECT_TRUE(CompareMatrices(J_expected, J_result, 1e-14));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
