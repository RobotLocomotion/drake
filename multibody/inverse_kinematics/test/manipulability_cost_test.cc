#include "drake/multibody/inverse_kinematics/manipulability_cost.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
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
AutoDiffXd CalcIndexOnAutoDiffPlant(const MultibodyPlant<AutoDiffXd>& plant,
                                    systems::Context<AutoDiffXd>* context,
                                    const std::string& frameA_name,
                                    const std::string& frameB_name,
                                    const VectorXd& q) {
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
                                plant_->GetFrameByName("iiwa_link_7"),
                                plant_context_, c);
  EXPECT_EQ(cost.num_vars(), plant_->num_positions());

  const VectorXd q =
      (VectorXd(7) << 0.1, 0.4, -0.3, -1.2, 0.5, 0.8, -0.2).finished();
  const AutoDiffXd w =
      CalcIndexOnAutoDiffPlant(*plant_autodiff_, plant_context_autodiff_.get(),
                               "world", "iiwa_link_7", q);
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

TEST_F(IiwaManipulabilityCostTest, DefaultWeight) {
  const ManipulabilityCost cost(plant_, plant_->world_frame(),
                                plant_->GetFrameByName("iiwa_link_7"),
                                plant_context_);
  const VectorXd q =
      (VectorXd(7) << 0.1, 0.4, -0.3, -1.2, 0.5, 0.8, -0.2).finished();
  const AutoDiffXd w =
      CalcIndexOnAutoDiffPlant(*plant_autodiff_, plant_context_autodiff_.get(),
                               "world", "iiwa_link_7", q);
  VectorXd y;
  cost.Eval(q, &y);
  EXPECT_NEAR(y(0), -w.value(), 1e-12);
}

TEST_F(IiwaManipulabilityCostTest, Singular) {
  const ManipulabilityCost cost(plant_, plant_->world_frame(),
                                plant_->GetFrameByName("iiwa_link_7"),
                                plant_context_);
  // Joints 1, 3, 5 and 7 are collinear at q = 0; rounding leaves w tiny but
  // not exactly zero.
  AutoDiffVecXd y_ad;
  cost.Eval(InitializeAutoDiff(VectorXd::Zero(7)), &y_ad);
  EXPECT_NEAR(y_ad(0).value(), 0.0, 1e-12);
  EXPECT_TRUE(CompareMatrices(y_ad(0).derivatives(), VectorXd::Zero(7), 1e-12));
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
                                parent->body_frame(), context.get());
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
      ManipulabilityCost(nullptr, world, link7, plant_context_),
      "plant is nullptr.");
  DRAKE_EXPECT_THROWS_MESSAGE(ManipulabilityCost(plant_, world, link7, nullptr),
                              ".*plant_context is nullptr");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ManipulabilityCost(plant_, world, link7, plant_context_, 0.0),
      ".*c > 0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      ManipulabilityCost(plant_, plant_->GetFrameByName("iiwa_link_3"), link7,
                         plant_context_),
      ".*only 4 generalized velocities.*'iiwa_link_3'.*'iiwa_link_7'.*");

  const ManipulabilityCost cost(plant_, world, link7, plant_context_);
  VectorX<symbolic::Expression> y;
  DRAKE_EXPECT_THROWS_MESSAGE(
      cost.Eval(symbolic::MakeVectorContinuousVariable(7, "q"), &y),
      ".*does not work for symbolic variables.");
}

class TwoFreeBodiesManipulabilityCostTest : public TwoFreeBodiesConstraintTest {
};

// A floating joint has q̇ ≠ v, so this exercises ∂w/∂q = N⁺(q)ᵀ⋅∂w/∂v. The
// quaternions are non-unit, as solvers visit, and the whole gradient must
// match.
TEST_F(TwoFreeBodiesManipulabilityCostTest, ValueAndGradient) {
  const Frame<double>& body1 = plant_->get_frame(body1_index_);
  const Frame<double>& body2 = plant_->get_frame(body2_index_);
  const ManipulabilityCost cost(plant_, body1, body2, plant_context_);
  plant_->SetFreeBodyPose(
      plant_context_, body1.body(),
      RigidTransformd(RollPitchYawd(0.3, -0.2, 0.5), Vector3d(0.1, 0.3, 0.7)));
  plant_->SetFreeBodyPose(
      plant_context_, body2.body(),
      RigidTransformd(RollPitchYawd(-0.4, 0.6, 0.1), Vector3d(-0.8, 0.2, 1.4)));
  VectorXd q = plant_->GetPositions(*plant_context_);
  q.segment<4>(body1.body().floating_positions_start()) *= 1.3;
  q.segment<4>(body2.body().floating_positions_start()) *= 0.8;
  const AutoDiffXd w = CalcIndexOnAutoDiffPlant(
      *plant_autodiff_, plant_context_autodiff_.get(), "body1", "body2", q);
  ASSERT_GT(w.value(), 1e-3);

  AutoDiffVecXd y_ad;
  cost.Eval(InitializeAutoDiff(q), &y_ad);
  EXPECT_NEAR(y_ad(0).value(), -w.value(), 1e-12);
  EXPECT_TRUE(CompareMatrices(y_ad(0).derivatives(), -w.derivatives(), 1e-10));

  // Swapping A and B leaves the index unchanged.
  const ManipulabilityCost swapped(plant_, body2, body1, plant_context_);
  AutoDiffVecXd y_swapped;
  swapped.Eval(InitializeAutoDiff(q), &y_swapped);
  EXPECT_NEAR(y_swapped(0).value(), y_ad(0).value(), 1e-12);
  EXPECT_TRUE(CompareMatrices(y_swapped(0).derivatives(), y_ad(0).derivatives(),
                              1e-10));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
