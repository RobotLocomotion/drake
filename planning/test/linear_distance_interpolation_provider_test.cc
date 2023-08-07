#include "drake/planning/linear_distance_interpolation_provider.h"

#include <common_robotics_utilities/math.hpp>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/math/rigid_transform.h"
#include "drake/planning/test/planning_test_helpers.h"

namespace drake {
namespace planning {
namespace test {
namespace {
using common_robotics_utilities::math::Distance;
using common_robotics_utilities::math::Interpolate;

multibody::parsing::ModelDirectives MakeFixedIiwaDirectives() {
  // Assemble model directives.
  multibody::parsing::ModelDirective add_env_model;
  add_env_model.add_model = multibody::parsing::AddModel{
      "package://drake/planning/test_utilities/collision_ground_plane.sdf",
      "ground_plane_box"};
  multibody::parsing::ModelDirective add_env_weld;
  add_env_weld.add_weld = multibody::parsing::AddWeld{
      "world", "ground_plane_box::ground_plane_box"};

  multibody::parsing::ModelDirective add_robot_model;
  add_robot_model.add_model = multibody::parsing::AddModel{
      "package://drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_collision.urdf",
      "iiwa"};
  multibody::parsing::ModelDirective add_robot_weld;
  add_robot_weld.add_weld = multibody::parsing::AddWeld{"world", "iiwa::base"};

  const multibody::parsing::ModelDirectives directives{
      .directives = {add_env_model, add_env_weld, add_robot_model,
                     add_robot_weld}};
  return directives;
}

multibody::parsing::ModelDirectives MakeFloatingIiwaDirectives() {
  // Assemble model directives.
  multibody::parsing::ModelDirective add_env_model;
  add_env_model.add_model = multibody::parsing::AddModel{
      "package://drake/planning/test_utilities/collision_ground_plane.sdf",
      "ground_plane_box"};
  multibody::parsing::ModelDirective add_env_weld;
  add_env_weld.add_weld = multibody::parsing::AddWeld{
      "world", "ground_plane_box::ground_plane_box"};

  multibody::parsing::ModelDirective add_arm_model;
  add_arm_model.add_model = multibody::parsing::AddModel{
      "package://drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_collision.urdf",
      "iiwa"};

  multibody::parsing::ModelDirective add_base_model;
  add_base_model.add_model = multibody::parsing::AddModel{
      "package://drake/planning/test_utilities/flying_robot_base.sdf",
      "flying_robot_base"};
  multibody::parsing::ModelDirective add_base_arm_weld;
  add_base_arm_weld.add_weld = multibody::parsing::AddWeld{
      "flying_robot_base::flying_robot_base", "iiwa::base"};

  const multibody::parsing::ModelDirectives directives{
      .directives = {add_env_model, add_env_weld, add_arm_model, add_base_model,
                     add_base_arm_weld}};
  return directives;
}

void DoFixedIiwaTest(const RobotDiagram<double>& model,
                     const LinearDistanceAndInterpolationProvider& provider,
                     const Eigen::VectorXd& expected_weights) {
  // Test Clone().
  const auto cloned = provider.Clone();
  EXPECT_NE(cloned, nullptr);

  // Make sure weights match expected values.
  EXPECT_TRUE(CompareMatrices(provider.distance_weights(), expected_weights));

  // Define test configurations.
  const auto zero_q = Eigen::VectorXd::Zero(model.plant().num_positions());
  const auto ones_q = Eigen::VectorXd::Ones(model.plant().num_positions());
  const Eigen::VectorXd half_q = ones_q * 0.5;

  // Test that every joint has the expected distance weight.
  for (int i = 0; i < model.plant().num_positions(); ++i) {
    const double expected_weight = expected_weights(i);

    Eigen::VectorXd pos_test_q = zero_q;
    pos_test_q(i) = 1.0;
    Eigen::VectorXd neg_test_q = zero_q;
    neg_test_q(i) = -1.0;

    EXPECT_EQ(provider.ComputeConfigurationDistance(zero_q, pos_test_q),
              expected_weight);
    EXPECT_EQ(provider.ComputeConfigurationDistance(zero_q, neg_test_q),
              expected_weight);
  }

  // Test full-q distance.
  {
    const Eigen::VectorXd weighted_deltas =
        (ones_q - zero_q).cwiseProduct(expected_weights);
    const double expected_distance = weighted_deltas.norm();
    EXPECT_EQ(provider.ComputeConfigurationDistance(zero_q, ones_q),
              expected_distance);
  }

  // Interpolation queries.
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(zero_q, ones_q, 0.0), zero_q));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(zero_q, ones_q, 1.0), ones_q));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(zero_q, ones_q, 0.5), half_q));
}

void DoFloatingIiwaTest(const RobotDiagram<double>& model,
                        const LinearDistanceAndInterpolationProvider& provider,
                        const Eigen::VectorXd& expected_weights) {
  const auto model_context = model.CreateDefaultContext();
  auto& plant_context = model.mutable_plant_context(model_context.get());
  const auto arm_model_instance = model.plant().GetModelInstanceByName("iiwa");
  const int num_arm_positions = model.plant().num_positions(arm_model_instance);
  const auto& base_body = model.plant().GetBodyByName("flying_robot_base");

  // Helper to assemble a full configuration from arm q and arm base pose.
  const auto make_full_q = [&](const Eigen::VectorXd& arm_q,
                               const math::RigidTransformd& X_WBase) {
    DRAKE_THROW_UNLESS(arm_q.size() == num_arm_positions);
    model.plant().SetPositions(&plant_context, arm_model_instance, arm_q);
    model.plant().SetFreeBodyPose(&plant_context, base_body, X_WBase);
    const Eigen::VectorXd full_q = model.plant().GetPositions(plant_context);
    return full_q;
  };

  // Define test configurations.
  const auto arm_zero_q = Eigen::VectorXd::Zero(num_arm_positions);
  const auto arm_ones_q = Eigen::VectorXd::Ones(num_arm_positions);
  const auto arm_half_q = Eigen::VectorXd::Constant(num_arm_positions, 0.5);

  const auto X_WBase_identity = math::RigidTransformd::Identity();
  const math::RigidTransformd X_WBase_test(
      math::RollPitchYawd(M_PI, -M_PI_2, M_PI_4),
      Eigen::Vector3d(1.0, 2.0, 3.0));
  const math::RigidTransformd X_WBase_half(Interpolate(
      X_WBase_identity.GetAsIsometry3(), X_WBase_test.GetAsIsometry3(), 0.5));

  const Eigen::VectorXd full_zero_q = make_full_q(arm_zero_q, X_WBase_identity);
  const Eigen::VectorXd full_test_q = make_full_q(arm_ones_q, X_WBase_test);
  const Eigen::VectorXd full_half_q = make_full_q(arm_half_q, X_WBase_half);

  // Test Clone().
  const auto cloned = provider.Clone();
  EXPECT_NE(cloned, nullptr);

  // Make sure weights match expected values.
  EXPECT_TRUE(CompareMatrices(provider.distance_weights(), expected_weights));

  const Eigen::VectorXd expected_arm_weights =
      model.plant().GetPositionsFromArray(arm_model_instance, expected_weights);

  // Test that every arm joint has the expected distance weight.
  for (int i = 0; i < num_arm_positions; ++i) {
    const double expected_weight = expected_arm_weights(i);

    Eigen::VectorXd arm_pos_test_q = arm_zero_q;
    arm_pos_test_q(i) = 1.0;
    Eigen::VectorXd arm_neg_test_q = arm_zero_q;
    arm_neg_test_q(i) = -1.0;

    for (const auto& X_WBase : {X_WBase_identity, X_WBase_half, X_WBase_test}) {
      const Eigen::VectorXd full_zero_test_q = make_full_q(arm_zero_q, X_WBase);
      const Eigen::VectorXd full_pos_test_q =
          make_full_q(arm_pos_test_q, X_WBase);
      const Eigen::VectorXd full_neg_test_q =
          make_full_q(arm_neg_test_q, X_WBase);

      EXPECT_EQ(provider.ComputeConfigurationDistance(full_zero_test_q,
                                                      full_pos_test_q),
                expected_weight);
      EXPECT_EQ(provider.ComputeConfigurationDistance(full_zero_test_q,
                                                      full_neg_test_q),
                expected_weight);
    }
  }

  // Test full-q distance.
  {
    const double orientation_delta =
        Distance(X_WBase_identity.rotation().ToQuaternion(),
                 X_WBase_test.rotation().ToQuaternion());

    const Eigen::Vector3d translation_deltas =
        X_WBase_test.translation() - X_WBase_identity.translation();

    const Eigen::VectorXd arm_deltas = arm_ones_q - arm_zero_q;

    Eigen::VectorXd deltas(14);
    deltas << orientation_delta, 0.0, 0.0, 0.0, translation_deltas, arm_deltas;

    const Eigen::VectorXd weighted_deltas =
        deltas.cwiseProduct(expected_weights);
    const double expected_distance = weighted_deltas.norm();
    EXPECT_EQ(provider.ComputeConfigurationDistance(full_zero_q, full_test_q),
              expected_distance);
  }

  // Interpolation queries.
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(full_zero_q, full_test_q, 0.0),
      full_zero_q));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(full_zero_q, full_test_q, 1.0),
      full_test_q));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(full_zero_q, full_test_q, 0.5),
      full_half_q, 1e-10));
}

GTEST_TEST(FixedIiwaTest, Test) {
  const auto model = MakePlanningTestModel(MakeFixedIiwaDirectives());

  // Default weights.
  {
    const LinearDistanceAndInterpolationProvider provider(
        model->plant(), std::map<std::string, double>{});

    Eigen::VectorXd expected_weights(7);
    expected_weights << 1, 1, 1, 1, 1, 1, 1;

    DoFixedIiwaTest(*model, provider, expected_weights);
  }

  // Non-default weights (map).
  {
    std::map<std::string, double> named_weights;
    named_weights["iiwa_joint_1"] = 1.0;
    named_weights["iiwa_joint_2"] = 2.0;
    named_weights["iiwa_joint_3"] = 3.0;
    named_weights["iiwa_joint_4"] = 4.0;
    named_weights["iiwa_joint_5"] = 5.0;
    named_weights["iiwa_joint_6"] = 6.0;
    named_weights["iiwa_joint_7"] = 7.0;

    const LinearDistanceAndInterpolationProvider provider(model->plant(),
                                                          named_weights);

    Eigen::VectorXd expected_weights(7);
    expected_weights << 1, 2, 3, 4, 5, 6, 7;

    DoFixedIiwaTest(*model, provider, expected_weights);
  }

  // Non-default weights (vector).
  {
    Eigen::VectorXd weights(7);
    weights << 1, 2, 3, 4, 5, 6, 7;

    const LinearDistanceAndInterpolationProvider provider(model->plant(),
                                                          weights);

    DoFixedIiwaTest(*model, provider, weights);
  }

  // Invalid weights.
  {
    const auto too_few_weights = Eigen::VectorXd::Ones(4);
    DRAKE_EXPECT_THROWS_MESSAGE(
        LinearDistanceAndInterpolationProvider(model->plant(), too_few_weights),
        "Provided distance weights size 4 does not match num_positions 7");

    const auto too_many_weights = Eigen::VectorXd::Ones(10);
    DRAKE_EXPECT_THROWS_MESSAGE(
        LinearDistanceAndInterpolationProvider(model->plant(),
                                               too_many_weights),
        "Provided distance weights size 10 does not match num_positions 7");

    Eigen::VectorXd has_negative_weights(7);
    has_negative_weights << -1, 2, -3, 4, -5, 6, -7;
    DRAKE_EXPECT_THROWS_MESSAGE(
        LinearDistanceAndInterpolationProvider(model->plant(),
                                               has_negative_weights),
        "Provided distance weight 0 with value -1 is less than zero");

    Eigen::VectorXd has_non_finite_weights(7);
    has_non_finite_weights << 1, 2, 3, 4, 5, 6,
        std::numeric_limits<double>::infinity();
    DRAKE_EXPECT_THROWS_MESSAGE(
        LinearDistanceAndInterpolationProvider(model->plant(),
                                               has_non_finite_weights),
        "Provided distance weight 6 with value inf is not finite");
  }
}

GTEST_TEST(FloatingIiwaTest, Test) {
  const auto model = MakePlanningTestModel(MakeFloatingIiwaDirectives());

  // Default weights.
  {
    const LinearDistanceAndInterpolationProvider provider(
        model->plant(), std::map<std::string, double>{});

    Eigen::VectorXd expected_weights(14);
    expected_weights << 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;

    DoFloatingIiwaTest(*model, provider, expected_weights);
  }

  // Non-default weights (map).
  {
    std::map<std::string, double> named_weights;
    named_weights["iiwa_joint_1"] = 1.0;
    named_weights["iiwa_joint_2"] = 2.0;
    named_weights["iiwa_joint_3"] = 3.0;
    named_weights["iiwa_joint_4"] = 4.0;
    named_weights["iiwa_joint_5"] = 5.0;
    named_weights["iiwa_joint_6"] = 6.0;
    named_weights["iiwa_joint_7"] = 7.0;

    const LinearDistanceAndInterpolationProvider provider(model->plant(),
                                                          named_weights);

    Eigen::VectorXd expected_weights(14);
    expected_weights << 1, 0, 0, 0, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7;

    DoFloatingIiwaTest(*model, provider, expected_weights);
  }

  // Non-default weights (vector).
  {
    Eigen::VectorXd weights(14);
    weights << 11, 0, 0, 0, 12, 13, 14, 1, 2, 3, 4, 5, 6, 7;

    const LinearDistanceAndInterpolationProvider provider(model->plant(),
                                                          weights);

    DoFloatingIiwaTest(*model, provider, weights);
  }

  // Invalid weights.
  {
    std::map<std::string, double> invalid_named_weights;
    invalid_named_weights["$world_flying_robot_base"] = 2.0;

    DRAKE_EXPECT_THROWS_MESSAGE(
        LinearDistanceAndInterpolationProvider(model->plant(),
                                               invalid_named_weights),
        ".* \\[quaternion_floating\\] not supported when using named.*");

    Eigen::VectorXd invalid_quat_weights(14);
    invalid_quat_weights << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14;

    DRAKE_EXPECT_THROWS_MESSAGE(
        LinearDistanceAndInterpolationProvider(model->plant(),
                                               invalid_quat_weights),
        ".* for quaternion dof .* must be .* instead.*");
  }
}

}  // namespace
}  // namespace test
}  // namespace planning
}  // namespace drake
