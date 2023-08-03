#include "drake/planning/linear_distance_interpolation_provider.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/unused.h"
#include "drake/math/rigid_transform.h"
#include "drake/planning/test/planning_test_helpers.h"

namespace drake {
namespace planning {
namespace test {
namespace {

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

  multibody::parsing::ModelDirective add_robot_model;
  add_robot_model.add_model = multibody::parsing::AddModel{
      "package://drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_spheres_dense_collision.urdf",
      "iiwa"};

  const multibody::parsing::ModelDirectives directives{
      .directives = {add_env_model, add_env_weld, add_robot_model}};
  return directives;
}

GTEST_TEST(FixedNoWeightsIiwaTest, Test) {
  const auto model = MakePlanningTestModel(MakeFixedIiwaDirectives());
  const LinearDistanceAndInterpolationProvider provider(
      model->plant(), std::map<std::string, double>{});

  // Define test configurations.
  const auto zero_q = Eigen::VectorXd::Zero(model->plant().num_positions());
  const auto ones_q = Eigen::VectorXd::Ones(model->plant().num_positions());
  const Eigen::VectorXd half_q = ones_q * 0.5;

  // Test Clone().
  const auto cloned = provider.Clone();
  EXPECT_NE(cloned, nullptr);

  // Test that every joint has a distance weight of 1.
  for (int i = 0; i < model->plant().num_positions(); ++i) {
    Eigen::VectorXd pos_test_q = zero_q;
    pos_test_q(i) = 1.0;
    Eigen::VectorXd neg_test_q = zero_q;
    neg_test_q(i) = -1.0;

    EXPECT_EQ(provider.ComputeConfigurationDistance(zero_q, pos_test_q), 1.0);
    EXPECT_EQ(provider.ComputeConfigurationDistance(zero_q, neg_test_q), 1.0);
  }

  // Test full-q distance.
  EXPECT_EQ(provider.ComputeConfigurationDistance(zero_q, ones_q),
            std::sqrt(static_cast<double>(model->plant().num_positions())));

  // Interpolation queries.
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(zero_q, ones_q, 0.0), zero_q));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(zero_q, ones_q, 1.0), ones_q));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(zero_q, ones_q, 0.5), half_q));
}

GTEST_TEST(FloatingNoWeightsIiwaTest, Test) {
  const auto model = MakePlanningTestModel(MakeFloatingIiwaDirectives());
  const LinearDistanceAndInterpolationProvider provider(
      model->plant(), std::map<std::string, double>{});

  const auto model_context = model->CreateDefaultContext();



  // Define test configurations.
  // auto zero_q = Eigen::VectorXd::Zero(model->plant().num_positions());
  // auto ones_q = Eigen::VectorXd::Ones(model->plant().num_positions());
  // Eigen::VectorXd half_q = ones_q * 0.5;
  // model->plant().Set

  // Test Clone().
  const auto cloned = provider.Clone();
  EXPECT_NE(cloned, nullptr);

  // // Test that every joint has a distance weight of 1.
  // for (int i = 0; i < model->plant().num_positions(); ++i) {
  //   auto pos_test_q = zero_q;
  //   pos_test_q(i) = 1.0;
  //   auto neg_test_q = zero_q;
  //   neg_test_q(i) = -1.0;

  //   EXPECT_EQ(provider.ComputeConfigurationDistance(zero_q,
  //   pos_test_q), 1.0);
  //   EXPECT_EQ(provider.ComputeConfigurationDistance(zero_q,
  //   neg_test_q), 1.0);
  // }

  // // Test full-q distance.
  // EXPECT_EQ(
  //     provider.ComputeConfigurationDistance(zero_q, ones_q),
  //     std::sqrt(static_cast<double>(model->plant().num_positons())));

  // // Interpolation queries.
  // EXPECT_TRUE(CompareMatrices(
  //     provider.InterpolateBetweenConfigurations(zero_q, ones_q, 0.0),
  //     zero_q));
  // EXPECT_TRUE(CompareMatrices(
  //     provider.InterpolateBetweenConfigurations(zero_q, ones_q, 1.0),
  //     ones_q));
  // EXPECT_TRUE(CompareMatrices(
  //     provider.InterpolateBetweenConfigurations(zero_q, ones_q, 0.5),
  //     half_q));
}

}  // namespace
}  // namespace test
}  // namespace planning
}  // namespace drake
