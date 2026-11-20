#include "drake/multibody/optimization/spatial_velocity_constraint.h"

#include <optional>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Checks that the constraints built from the double and AutoDiffXd plants
// agree in their bounds, values, and gradients at state x.
void CompareDoubleAndAutoDiffPlants(const MultibodyPlant<double>& plant,
                                    systems::Context<double>* context,
                                    const MultibodyPlant<AutoDiffXd>& plant_ad,
                                    systems::Context<AutoDiffXd>* context_ad,
                                    const std::string& frameA_name,
                                    const std::string& frameB_name,
                                    const VectorXd& x) {
  const Vector3d p_BC(0.1, 0.2, 0.3);
  const Vector3d v_AC_lower(-0.2, -0.3, -0.4);
  const Vector3d v_AC_upper(0.2, 0.3, 0.4);
  const SpatialVelocityConstraint::AngularVelocityBounds w_AC_bounds{
      .magnitude_lower = 0.1,
      .magnitude_upper = 2.0,
      .reference_direction = {0.1, 0.2, 0.3},
      .theta_bound = 0.5};
  const double kTol = 1e-12;

  for (const auto& bounds :
       {std::optional<SpatialVelocityConstraint::AngularVelocityBounds>{},
        std::optional{w_AC_bounds}}) {
    SCOPED_TRACE(fmt::format("A = {}, B = {}, angular bounds = {}", frameA_name,
                             frameB_name, bounds.has_value()));
    const SpatialVelocityConstraint dut(
        &plant, plant.GetFrameByName(frameA_name), v_AC_lower, v_AC_upper,
        plant.GetFrameByName(frameB_name), p_BC, context, bounds);
    const SpatialVelocityConstraint expected(
        &plant_ad, plant_ad.GetFrameByName(frameA_name), v_AC_lower, v_AC_upper,
        plant_ad.GetFrameByName(frameB_name), p_BC, context_ad, bounds);

    EXPECT_TRUE(CompareMatrices(dut.lower_bound(), expected.lower_bound()));
    EXPECT_TRUE(CompareMatrices(dut.upper_bound(), expected.upper_bound()));

    VectorXd y, y_expected;
    dut.Eval(x, &y);
    expected.Eval(x, &y_expected);
    EXPECT_TRUE(CompareMatrices(y, y_expected, 1e-14));

    // Identity seed, and a seed with fewer derivatives than x.
    for (const MatrixXd& dx : {MatrixXd(MatrixXd::Identity(x.size(), x.size())),
                               MatrixXd(MatrixXd::Random(x.size(), 3))}) {
      const AutoDiffVecXd x_ad = math::InitializeAutoDiff(x, dx);
      AutoDiffVecXd y_ad, y_ad_expected;
      dut.Eval(x_ad, &y_ad);
      expected.Eval(x_ad, &y_ad_expected);
      CompareAutoDiffVectors(y_ad, y_ad_expected, kTol);
    }
  }
}

TEST_F(IiwaKinematicConstraintTest, DoublePlantMatchesAutoDiffPlant) {
  VectorXd x(14);
  x << 0.1, 0.2, 0.25, -0.6, 0.26, 0.12, 0.15, 0.36, 0.73, 0.5, -0.62, 0.243,
      0.245, 0.55;
  // A is a descendant of B.
  CompareDoubleAndAutoDiffPlants(*plant_, plant_context_, *plant_autodiff_,
                                 plant_context_autodiff_.get(), "iiwa_link_7",
                                 "iiwa_link_3", x);
  // A is the world.
  CompareDoubleAndAutoDiffPlants(*plant_, plant_context_, *plant_autodiff_,
                                 plant_context_autodiff_.get(), "world",
                                 "iiwa_link_7", x);
  // B is a descendant of A.
  CompareDoubleAndAutoDiffPlants(*plant_, plant_context_, *plant_autodiff_,
                                 plant_context_autodiff_.get(), "iiwa_link_2",
                                 "iiwa_link_5", x);
}

// Sets the floating bodies' poses so that x has unit quaternions.
VectorXd MakeStateWithUnitQuaternions(
    const MultibodyPlant<double>& plant, systems::Context<double>* context,
    const std::vector<const RigidBody<double>*>& floating_bodies) {
  plant.SetPositionsAndVelocities(
      context, VectorXd::LinSpaced(plant.num_multibody_states(), -0.7, 0.9));
  double angle = 0.3;
  for (const RigidBody<double>* body : floating_bodies) {
    plant.SetFreeBodyPose(
        context, *body,
        math::RigidTransformd(math::RollPitchYawd(angle, -0.2, 0.5),
                              Vector3d(0.1, -0.4, angle)));
    angle += 0.4;
  }
  return plant.GetPositionsAndVelocities(*context);
}

GTEST_TEST(SpatialVelocityConstraintTest, FloatingIiwa) {
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  const auto plant_ad = systems::System<double>::ToAutoDiffXd(plant);
  auto context_ad = plant_ad->CreateDefaultContext();
  const VectorXd x = MakeStateWithUnitQuaternions(
      plant, context.get(), {&plant.GetBodyByName("iiwa_link_0")});

  CompareDoubleAndAutoDiffPlants(plant, context.get(), *plant_ad,
                                 context_ad.get(), "world", "iiwa_link_7", x);
  // A is a descendant of B.
  CompareDoubleAndAutoDiffPlants(plant, context.get(), *plant_ad,
                                 context_ad.get(), "iiwa_link_6", "iiwa_link_2",
                                 x);
}

TEST_F(TwoFreeBodiesConstraintTest, DoublePlantMatchesAutoDiffPlant) {
  const VectorXd x = MakeStateWithUnitQuaternions(
      *plant_, plant_context_,
      {&plant_->GetBodyByName("body1"), &plant_->GetBodyByName("body2")});
  CompareDoubleAndAutoDiffPlants(*plant_, plant_context_, *plant_autodiff_,
                                 plant_context_autodiff_.get(), "body1",
                                 "body2", x);
}

TEST_F(IiwaKinematicConstraintTest, SpatialVelocityConstraint) {
  const Vector3d p_BC(0.1, 0.2, 0.3);
  const Vector3d v_AC_lower(-0.2, -0.3, -0.4);
  const Vector3d v_AC_upper(0.2, 0.3, 0.4);
  const auto& frameA = plant_autodiff_->GetFrameByName("iiwa_link_7");
  const auto& frameB = plant_autodiff_->GetFrameByName("iiwa_link_3");

  SpatialVelocityConstraint constraint_translation_only(
      plant_autodiff_.get(), frameA, v_AC_lower, v_AC_upper, frameB, p_BC,
      plant_context_autodiff_.get());

  EXPECT_TRUE(
      CompareMatrices(constraint_translation_only.lower_bound(), v_AC_lower));
  EXPECT_TRUE(
      CompareMatrices(constraint_translation_only.upper_bound(), v_AC_upper));

  const SpatialVelocityConstraint::AngularVelocityBounds w_AC_bounds{
      .magnitude_lower = 0.1,
      .magnitude_upper = 2.0,
      .reference_direction = {0.1, 0.2, 0.3},
      .theta_bound = 0.5};

  SpatialVelocityConstraint constraint(
      plant_autodiff_.get(), frameA, v_AC_lower, v_AC_upper, frameB, p_BC,
      plant_context_autodiff_.get(), w_AC_bounds);

  VectorXd expected_lower(5), expected_upper(5);
  expected_lower << v_AC_lower, std::pow(w_AC_bounds.magnitude_lower, 2),
      std::cos(w_AC_bounds.theta_bound);
  expected_upper << v_AC_upper, std::pow(w_AC_bounds.magnitude_upper, 2), 1;
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), expected_lower));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), expected_upper));

  // Set arbitrary positions and velocities.
  VectorXd x(14);
  x << 0.1, 0.2, 0.25, -0.6, 0.26, 0.12, 0.15, 0.36, 0.73, 0.5, -0.62, 0.243,
      0.245, 0.55;

  VectorXd y;
  constraint.Eval(x, &y);

  // Now compute the expected quantities (a different way). We only check that
  // the double values match; the gradients are computed via a straight-forward
  // application of autodiff.
  plant_->SetPositionsAndVelocities(plant_context_, x);
  const RigidBody<double>& bodyA = plant_->GetBodyByName("iiwa_link_7");
  const RigidBody<double>& bodyB = plant_->GetBodyByName("iiwa_link_3");
  const auto V_AW_W = plant_->world_frame().CalcSpatialVelocity(
      *plant_context_, bodyA.body_frame(), plant_->world_frame());
  const auto V_WB_W = bodyB.EvalSpatialVelocityInWorld(*plant_context_);
  const auto R_WB = bodyB.EvalPoseInWorld(*plant_context_).rotation();
  const auto V_WC_W = V_WB_W.Shift(R_WB * p_BC);
  const auto R_AW = bodyA.EvalPoseInWorld(*plant_context_).rotation().inverse();
  Vector3d p_WC_W;
  plant_->CalcPointsPositions(*plant_context_, bodyB.body_frame(), p_BC,
                              plant_->world_frame(), &p_WC_W);
  const auto V_AC_W = V_AW_W.ComposeWithMovingFrameVelocity(p_WC_W, V_WC_W);
  const auto V_AC_A = R_AW * V_AC_W;

  EXPECT_TRUE(CompareMatrices(V_AC_A.translational(), y.head<3>(), 1e-14));
  EXPECT_NEAR(y[3], V_AC_A.rotational().squaredNorm(), 1e-14);
  EXPECT_NEAR(y[4],
              V_AC_A.rotational().normalized().dot(
                  w_AC_bounds.reference_direction.normalized()),
              1e-14);
}

TEST_F(IiwaKinematicConstraintTest, InvalidInputs) {
  const Vector3d p_BC(0.1, 0.2, 0.3);
  const Vector3d v_AC_lower(-0.2, -0.3, -0.4);
  const Vector3d v_AC_upper(0.2, 0.3, 0.4);
  const auto& frameA = plant_autodiff_->GetFrameByName("iiwa_link_7");
  const auto& frameB = plant_autodiff_->GetFrameByName("iiwa_link_3");

  SpatialVelocityConstraint::AngularVelocityBounds w_AC_bounds{
      .magnitude_lower = 0.1,
      .magnitude_upper = 2.0,
      .reference_direction = {0.1, 0.2, 0.3},
      .theta_bound = 0.5};

  // This version works.
  SpatialVelocityConstraint constraint(
      plant_autodiff_.get(), frameA, v_AC_lower, v_AC_upper, frameB, p_BC,
      plant_context_autodiff_.get(), w_AC_bounds);

  // @throws std::exception if `plant` is nullptr.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialVelocityConstraint(nullptr, frameA, v_AC_lower, v_AC_upper, frameB,
                                p_BC, plant_context_autodiff_.get(),
                                w_AC_bounds),
      "plant is nullptr.");

  // @throws std::exception if `plant_context` is nullptr.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialVelocityConstraint(plant_autodiff_.get(), frameA, v_AC_lower,
                                v_AC_upper, frameB, p_BC, nullptr, w_AC_bounds),
      "plant_context is nullptr.");

  // @throws std::exception if invalid w_AC_bounds are provided.
  w_AC_bounds.magnitude_upper = 0.01;
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialVelocityConstraint(plant_autodiff_.get(), frameA, v_AC_lower,
                                v_AC_upper, frameB, p_BC, nullptr, w_AC_bounds),
      ".*w_AC_bounds->magnitude_lower <= w_AC_bounds->magnitude_upper.*");
}

TEST_F(IiwaKinematicConstraintTest, InvalidInputsDoublePlant) {
  const Vector3d p_BC(0.1, 0.2, 0.3);
  const Vector3d v_AC_lower(-0.2, -0.3, -0.4);
  const Vector3d v_AC_upper(0.2, 0.3, 0.4);
  const auto& frameA = plant_->GetFrameByName("iiwa_link_7");
  const auto& frameB = plant_->GetFrameByName("iiwa_link_3");

  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialVelocityConstraint(
          static_cast<const MultibodyPlant<double>*>(nullptr), frameA,
          v_AC_lower, v_AC_upper, frameB, p_BC, plant_context_),
      "plant is nullptr.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialVelocityConstraint(plant_, frameA, v_AC_lower, v_AC_upper, frameB,
                                p_BC, nullptr),
      "plant_context is nullptr.");
}

}  // namespace
}  // namespace multibody
}  // namespace drake
