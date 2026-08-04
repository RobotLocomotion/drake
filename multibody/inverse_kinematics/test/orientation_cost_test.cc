#include "drake/multibody/inverse_kinematics/orientation_cost.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/math/wrap_to.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using math::InitializeAutoDiff;
using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;
using systems::Context;

template <typename T>
auto ConstructTwoFreeBodiesCost(const math::RotationMatrix<double>& R_AbarA,
                                const math::RotationMatrix<double>& R_BbarB,
                                double c) {
  struct ReturnValues {
    std::unique_ptr<MultibodyPlant<T>> plant;
    std::unique_ptr<Context<T>> context;
    const RigidBody<T>* body1{};
    const RigidBody<T>* body2{};
    std::unique_ptr<OrientationCost> cost;
  };
  ReturnValues v;

  v.plant = ConstructTwoFreeBodiesPlant<T>();
  v.context = v.plant->CreateDefaultContext();

  v.body1 = &v.plant->GetBodyByName("body1");
  v.body2 = &v.plant->GetBodyByName("body2");

  v.cost = std::make_unique<OrientationCost>(
      v.plant.get(), v.body1->body_frame(), R_AbarA, v.body2->body_frame(),
      R_BbarB, c, v.context.get());

  return v;
}

// Given two free bodies with some given poses, evaluate the position cost.
GTEST_TEST(OrientationCostTest, TwoFreeBodies) {
  const RotationMatrix<double> R_AbarA(RollPitchYaw<double>(1, 2, 3));
  const RotationMatrix<double> R_BbarB(RollPitchYaw<double>(4, 5, 6));
  const double c{2.4};

  auto [plant, context, body1, body2, cost] =
      ConstructTwoFreeBodiesCost<double>(R_AbarA, R_BbarB, c);
  auto [plant_ad, context_ad, body1_ad, body2_ad, cost_ad] =
      ConstructTwoFreeBodiesCost<AutoDiffXd>(R_AbarA, R_BbarB, c);

  // Note: We are passing OrientationCost here instead of capturing them due to
  // a known issue in C++:
  // https://stackoverflow.com/questions/46114214/lambda-implicit-capture-fails-with-variable-declared-from-structured-binding
  auto CheckCases = [](const OrientationCost& orientation_cost,
                       const OrientationCost& orientation_cost_ad,
                       const VectorXd& q, double y_expected) {
    // MultibodyPlant double, Eval double.
    VectorXd y(1);
    orientation_cost.Eval(q, &y);
    EXPECT_NEAR(y[0], y_expected, 1e-12);

    // MultibodyPlant AutoDiffXd, Eval double.
    VectorXd y_ad_d(1);
    orientation_cost_ad.Eval(q, &y_ad_d);
    EXPECT_NEAR(y_ad_d[0], y_expected, 1e-12);

    // MultibodyPlant double, Eval AutoDiffXd
    const VectorX<AutoDiffXd> q_ad = InitializeAutoDiff(q);
    VectorX<AutoDiffXd> y_d_ad(1);
    orientation_cost.Eval(q_ad, &y_d_ad);
    EXPECT_NEAR(y_d_ad[0].value(), y_expected, 1e-12);

    // MultibodyPlant AutoDiffXd, Eval AutoDiffXd
    VectorX<AutoDiffXd> y_ad_ad(1);
    orientation_cost_ad.Eval(q_ad, &y_ad_ad);
    EXPECT_NEAR(y_ad_ad[0].value(), y_expected, 1e-12);

    // Check that the two gradients match.
    EXPECT_EQ(y_d_ad[0].derivatives().size(), q.size());
    EXPECT_TRUE(CompareMatrices(y_d_ad[0].derivatives(),
                                y_ad_ad[0].derivatives(), 1e-11));
  };

  // X_WA = X_WB = Identity;
  plant->SetFreeBodyPose(context.get(), *body1, RigidTransform<double>());
  plant->SetFreeBodyPose(context.get(), *body2, RigidTransform<double>());
  VectorXd q = plant->GetPositions(*context);
  RotationMatrix<double> R_AB = R_AbarA.inverse() * R_BbarB;  // because A == B.
  // We use the explicit formula in the Cost implementation; we test this
  // against Eigen's conversion from matrices to AngleAxis:
  double theta = R_AB.ToAngleAxis().angle();
  CheckCases(*cost, *cost_ad, q, c * (1.0 - cos(theta)));

  // X_WA, X_WB are arbitrary non-identity transforms.
  RigidTransform<double> X_WAbar(RollPitchYaw<double>(0.32, -0.24, -0.51),
                                 Vector3d(0.1, 0.3, 0.72));
  RigidTransform<double> X_WBbar(RollPitchYaw<double>(8.1, 0.42, -0.2),
                                 Vector3d(-0.84, 0.2, 1.4));
  plant->SetFreeBodyPose(context.get(), *body1, X_WAbar);
  plant->SetFreeBodyPose(context.get(), *body2, X_WBbar);
  q = plant->GetPositions(*context);

  R_AB =
      (X_WAbar.rotation() * R_AbarA).inverse() * X_WBbar.rotation() * R_BbarB;
  theta = R_AB.ToAngleAxis().angle();
  CheckCases(*cost, *cost_ad, q, c * (1.0 - cos(theta)));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
