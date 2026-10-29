#include "drake/multibody/inverse_kinematics/manipulability_cost.h"

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using math::InitializeAutoDiff;
using math::RigidTransformd;
using math::RollPitchYawd;

// The index computed independently: J on the AutoDiffXd plant at a point Q
// away from Bo and expressed in world, which the index must not depend on.
AutoDiffXd CalcManipulabilityIndexOnAutoDiffPlant(
    const MultibodyPlant<AutoDiffXd>& plant,
    systems::Context<AutoDiffXd>* context, const std::string& frameA_name,
    const std::string& frameB_name, const VectorXd& q) {
  plant.SetPositions(context, InitializeAutoDiff(q));
  MatrixX<AutoDiffXd> J(6, plant.num_velocities());
  plant.CalcJacobianSpatialVelocity(
      *context, JacobianWrtVariable::kV, plant.GetFrameByName(frameB_name),
      Vector3<AutoDiffXd>(0.1, -0.2, 0.3), plant.GetFrameByName(frameA_name),
      plant.world_frame(), &J);
  // The columns off the path are zero, so all of them can stay.
  return sqrt((J * J.transpose()).determinant());
}

class IiwaManipulabilityCostTest : public IiwaKinematicConstraintTest {};

TEST_F(IiwaManipulabilityCostTest, ValueAndGradient) {
  const double c = 2.5;
  const ManipulabilityCost cost(plant_, plant_->world_frame(),
                                plant_->GetFrameByName("iiwa_link_7"), c,
                                plant_context_);
  EXPECT_EQ(cost.num_vars(), plant_->num_positions());

  const VectorXd q =
      (VectorXd(7) << 0.1, 0.4, -0.3, -1.2, 0.5, 0.8, -0.2).finished();
  const AutoDiffXd w = CalcManipulabilityIndexOnAutoDiffPlant(
      *plant_autodiff_, plant_context_autodiff_.get(), "world", "iiwa_link_7",
      q);
  ASSERT_GT(w.value(), 1e-3);

  VectorXd y;
  cost.Eval(q, &y);
  EXPECT_NEAR(y(0), -c * w.value(), 1e-12);

  // Seed x with a non-identity gradient to check the chain rule.
  const MatrixXd dq_dz = MatrixXd::Random(7, 3);
  AutoDiffVecXd x = q.cast<AutoDiffXd>();
  for (int i = 0; i < 7; ++i) x(i).derivatives() = dq_dz.row(i).transpose();
  AutoDiffVecXd y_ad;
  cost.Eval(x, &y_ad);
  EXPECT_NEAR(y_ad(0).value(), -c * w.value(), 1e-12);
  EXPECT_TRUE(CompareMatrices(y_ad(0).derivatives(),
                              -c * dq_dz.transpose() * w.derivatives(), 1e-10));
}

TEST_F(IiwaManipulabilityCostTest, Singular) {
  const ManipulabilityCost cost(plant_, plant_->world_frame(),
                                plant_->GetFrameByName("iiwa_link_7"), 1.0,
                                plant_context_);
  // Joints 1, 3, 5 and 7 are collinear at q = 0; rounding leaves w tiny but
  // not exactly zero.
  AutoDiffVecXd y_ad;
  cost.Eval(InitializeAutoDiff(VectorXd::Zero(7)), &y_ad);
  EXPECT_NEAR(y_ad(0).value(), 0.0, 1e-12);
  EXPECT_TRUE(CompareMatrices(y_ad(0).derivatives(), VectorXd::Zero(7), 1e-12));
}

// Near the elbow singularity (joint 4 angle δ), w and ∂w/∂q stay accurate,
// which forming J Jᵀ would prevent. w = aδ + O(δ²), so w/δ and ∂w/∂q at
// δ = 1e-9 match their values at δ = 1e-5 up to O(1e-5).
TEST_F(IiwaManipulabilityCostTest, NearSingular) {
  const ManipulabilityCost cost(plant_, plant_->world_frame(),
                                plant_->GetFrameByName("iiwa_link_7"), 1.0,
                                plant_context_);
  auto eval = [&](double delta) {
    VectorXd q(7);
    q << 0.3, -0.5, 0.8, delta, 0.4, -0.9, 0.2;
    AutoDiffVecXd y;
    cost.Eval(InitializeAutoDiff(q), &y);
    return y(0);
  };
  const AutoDiffXd y_tiny = eval(1e-9);
  const AutoDiffXd y_small = eval(1e-5);
  const double slope = y_small.value() / 1e-5;
  EXPECT_NEAR(y_tiny.value() / 1e-9, slope, 1e-4 * std::abs(slope));
  EXPECT_TRUE(CompareMatrices(y_tiny.derivatives(), y_small.derivatives(),
                              1e-4 * y_small.derivatives().norm()));
}

// Neither frame's pose on its body affects the index.
GTEST_TEST(ManipulabilityCostTest, OffsetFramesMatchBodyFrames) {
  MultibodyPlant<double> plant(0.0);
  AddTwoFreeBodiesToPlant(&plant);
  const Frame<double>& body1 = plant.GetFrameByName("body1");
  const Frame<double>& body2 = plant.GetFrameByName("body2");
  const Frame<double>& offset1 =
      plant.AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          "offset1", body1,
          RigidTransformd(RollPitchYawd(0.7, -0.3, 1.2),
                          Vector3d(0.4, -1.1, 0.6))));
  const Frame<double>& offset2 =
      plant.AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          "offset2", body2,
          RigidTransformd(RollPitchYawd(-0.5, 0.9, 0.2),
                          Vector3d(-0.8, 0.3, 1.5))));
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  plant.SetFreeBodyPose(
      context.get(), body1.body(),
      RigidTransformd(RollPitchYawd(0.3, -0.2, 0.5), Vector3d(0.1, 0.3, 0.7)));
  plant.SetFreeBodyPose(
      context.get(), body2.body(),
      RigidTransformd(RollPitchYawd(-0.4, 0.6, 0.1), Vector3d(-0.8, 0.2, 1.4)));
  const VectorXd q = plant.GetPositions(*context);

  const ManipulabilityCost on_bodies(&plant, body1, body2, 1.0, context.get());
  const ManipulabilityCost on_offsets(&plant, offset1, offset2, 1.0,
                                      context.get());
  AutoDiffVecXd y_bodies;
  AutoDiffVecXd y_offsets;
  on_bodies.Eval(InitializeAutoDiff(q), &y_bodies);
  on_offsets.Eval(InitializeAutoDiff(q), &y_offsets);
  ASSERT_LT(y_bodies(0).value(), -1e-3);
  EXPECT_NEAR(y_offsets(0).value(), y_bodies(0).value(), 1e-12);
  EXPECT_TRUE(CompareMatrices(y_offsets(0).derivatives(),
                              y_bodies(0).derivatives(), 1e-10));
}

// Joints off the path (here, all of the second arm's) get exactly zero
// gradient.
GTEST_TEST(ManipulabilityCostTest, OffPathGradientIsZero) {
  auto plant = ConstructIiwaPlant(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf",
      0.1, 2 /* num_iiwa */);
  auto context = plant->CreateDefaultContext();
  const ModelInstanceIndex arm1 = plant->GetModelInstanceByName("iiwa14");
  const ManipulabilityCost cost(plant.get(), plant->world_frame(),
                                plant->GetFrameByName("iiwa_link_7", arm1), 1.0,
                                context.get());
  VectorXd q(14);
  q << 0.1, 0.4, -0.3, -1.2, 0.5, 0.8, -0.2, 0.3, -0.6, 0.9, -1.1, 0.2, 0.7,
      0.4;
  AutoDiffVecXd y;
  cost.Eval(InitializeAutoDiff(q), &y);
  ASSERT_LT(y(0).value(), -1e-3);

  std::vector<bool> on_path(plant->num_positions(), false);
  for (const JointIndex& joint_index : plant->GetJointIndices(arm1)) {
    const Joint<double>& joint = plant->get_joint(joint_index);
    for (int k = 0; k < joint.num_positions(); ++k) {
      on_path[joint.position_start() + k] = true;
    }
  }
  double on_path_norm_squared = 0;
  for (int i = 0; i < plant->num_positions(); ++i) {
    const double dy_dqi = y(0).derivatives()(i);
    if (on_path[i]) {
      on_path_norm_squared += dy_dqi * dy_dqi;
    } else {
      EXPECT_EQ(dy_dqi, 0.0);
    }
  }
  EXPECT_GT(on_path_norm_squared, 1e-6);
}

// A planar 6R chain: J's rows for ωx, ωy and vz are exactly zero at every q.
GTEST_TEST(ManipulabilityCostTest, PlanarChainIsSingular) {
  MultibodyPlant<double> plant(0.0);
  const RigidBody<double>* parent = &plant.world_body();
  for (int i = 0; i < 6; ++i) {
    const RigidBody<double>& child = plant.AddRigidBody(
        fmt::format("link{}", i), SpatialInertia<double>::MakeUnitary());
    plant.AddJoint<RevoluteJoint>(
        fmt::format("joint{}", i), *parent,
        RigidTransformd(Vector3d(i == 0 ? 0.0 : 0.5, 0, 0)), child,
        std::nullopt, Vector3d::UnitZ());
    parent = &child;
  }
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  const ManipulabilityCost cost(&plant, plant.world_frame(),
                                parent->body_frame(), 1.0, context.get());
  AutoDiffVecXd y_ad;
  cost.Eval(InitializeAutoDiff(
                (VectorXd(6) << 0.1, -0.7, 0.3, 1.1, -0.4, 0.9).finished()),
            &y_ad);
  EXPECT_EQ(y_ad(0).value(), 0.0);
  EXPECT_TRUE(CompareMatrices(y_ad(0).derivatives(), VectorXd::Zero(6)));
}

TEST_F(IiwaManipulabilityCostTest, Throws) {
  const Frame<double>& world = plant_->world_frame();
  const Frame<double>& link7 = plant_->GetFrameByName("iiwa_link_7");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ManipulabilityCost(nullptr, world, link7, 1.0, plant_context_),
      "plant is nullptr.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ManipulabilityCost(plant_, world, link7, 1.0, nullptr),
      ".*plant_context is nullptr");
  for (const double bad_c : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()}) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        ManipulabilityCost(plant_, world, link7, bad_c, plant_context_),
        ".*c > 0 && std::isfinite\\(c\\).*c = .*");
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      ManipulabilityCost(plant_, plant_->GetFrameByName("iiwa_link_3"), link7,
                         1.0, plant_context_),
      ".*only 4 generalized velocities.*'iiwa_link_3'.*'iiwa_link_7'.*");

  const ManipulabilityCost cost(plant_, world, link7, 1.0, plant_context_);
  VectorX<symbolic::Expression> y;
  DRAKE_EXPECT_THROWS_MESSAGE(
      cost.Eval(symbolic::MakeVectorContinuousVariable(7, "q"), &y),
      ".*does not work for symbolic variables.");
}

class TwoFreeBodiesManipulabilityCostTest : public TwoFreeBodiesConstraintTest {
};

// Here w depends only on the distance between the body origins, so the
// gradient's quaternion entries are zero; QuaternionGradient covers them.
TEST_F(TwoFreeBodiesManipulabilityCostTest, ValueAndGradient) {
  const Frame<double>& body1 = plant_->get_frame(body1_index_);
  const Frame<double>& body2 = plant_->get_frame(body2_index_);
  const ManipulabilityCost cost(plant_, body1, body2, 1.0, plant_context_);
  plant_->SetFreeBodyPose(
      plant_context_, body1.body(),
      RigidTransformd(RollPitchYawd(0.3, -0.2, 0.5), Vector3d(0.1, 0.3, 0.7)));
  plant_->SetFreeBodyPose(
      plant_context_, body2.body(),
      RigidTransformd(RollPitchYawd(-0.4, 0.6, 0.1), Vector3d(-0.8, 0.2, 1.4)));
  VectorXd q = plant_->GetPositions(*plant_context_);
  q.segment<4>(body1.body().floating_positions_start()) *= 1.3;
  q.segment<4>(body2.body().floating_positions_start()) *= 0.8;
  const AutoDiffXd w = CalcManipulabilityIndexOnAutoDiffPlant(
      *plant_autodiff_, plant_context_autodiff_.get(), "body1", "body2", q);
  ASSERT_GT(w.value(), 1e-3);

  AutoDiffVecXd y_ad;
  cost.Eval(InitializeAutoDiff(q), &y_ad);
  EXPECT_NEAR(y_ad(0).value(), -w.value(), 1e-12);
  EXPECT_TRUE(CompareMatrices(y_ad(0).derivatives(), -w.derivatives(), 1e-10));

  // Swapping A and B leaves the index unchanged.
  const ManipulabilityCost swapped(plant_, body2, body1, 1.0, plant_context_);
  AutoDiffVecXd y_swapped;
  swapped.Eval(InitializeAutoDiff(q), &y_swapped);
  EXPECT_NEAR(y_swapped(0).value(), y_ad(0).value(), 1e-12);
  EXPECT_TRUE(CompareMatrices(y_swapped(0).derivatives(), y_ad(0).derivatives(),
                              1e-10));
}

// A floating joint mid-chain makes w depend on its quaternion, so this
// exercises the quaternion blocks of ∂w/∂q = N⁺(q)ᵀ⋅∂w/∂v, at unit and
// non-unit quaternions (as solvers visit).
GTEST_TEST(ManipulabilityCostTest, QuaternionGradient) {
  MultibodyPlant<double> plant(0.0);
  const auto M = SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1);
  const RigidBody<double>& L1 = plant.AddRigidBody("L1", M);
  const RigidBody<double>& L2 = plant.AddRigidBody("L2", M);
  const RigidBody<double>& L3 = plant.AddRigidBody("L3", M);
  const auto& j1 =
      plant.AddJoint<RevoluteJoint>("j1", plant.world_body(), std::nullopt, L1,
                                    std::nullopt, Vector3d::UnitZ());
  const auto& floating = plant.AddJoint<QuaternionFloatingJoint>(
      "floating", L1, RigidTransformd(Vector3d(0, 0, 0.5)), L2, std::nullopt);
  const auto& j3 = plant.AddJoint<RevoluteJoint>(
      "j3", L2, RigidTransformd(Vector3d(0.4, 0, 0)), L3, std::nullopt,
      Vector3d::UnitX());
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  j1.set_angle(context.get(), 0.7);
  j3.set_angle(context.get(), -0.4);
  floating.SetPose(context.get(), RigidTransformd(RollPitchYawd(0.3, -0.5, 0.8),
                                                  Vector3d(0.2, -0.1, 0.3)));
  auto plant_autodiff = systems::System<double>::ToAutoDiffXd(plant);
  auto context_autodiff = plant_autodiff->CreateDefaultContext();
  const ManipulabilityCost cost(&plant, plant.world_frame(), L3.body_frame(),
                                1.0, context.get());
  const int start = floating.position_start();
  for (const double scale : {1.0, 1.3}) {
    VectorXd q = plant.GetPositions(*context);
    q.segment<4>(start) *= scale;
    const AutoDiffXd w = CalcManipulabilityIndexOnAutoDiffPlant(
        *plant_autodiff, context_autodiff.get(), "world", "L3", q);
    AutoDiffVecXd y_ad;
    cost.Eval(InitializeAutoDiff(q), &y_ad);
    ASSERT_GT(y_ad(0).derivatives().segment<4>(start).norm(), 0.1);
    EXPECT_NEAR(y_ad(0).value(), -w.value(), 1e-12);
    EXPECT_TRUE(
        CompareMatrices(y_ad(0).derivatives(), -w.derivatives(), 1e-10));
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
