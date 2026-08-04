#include "drake/multibody/inverse_kinematics/angle_between_vectors_cost.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;

using math::InitializeAutoDiff;
using math::RigidTransform;
using math::RollPitchYaw;
using systems::Context;

template <typename T>
auto ConstructTwoFreeBodiesCost(const Eigen::Ref<const Vector3d>& a_A,
                                const Eigen::Ref<const Vector3d>& b_B,
                                double c) {
  struct ReturnValues {
    std::unique_ptr<MultibodyPlant<T>> plant;
    std::unique_ptr<Context<T>> context;
    const RigidBody<T>* body1{};
    const RigidBody<T>* body2{};
    std::unique_ptr<AngleBetweenVectorsCost> cost;
  };
  ReturnValues v;

  v.plant = ConstructTwoFreeBodiesPlant<T>();
  v.context = v.plant->CreateDefaultContext();

  v.body1 = &v.plant->GetBodyByName("body1");
  v.body2 = &v.plant->GetBodyByName("body2");

  v.cost = std::make_unique<AngleBetweenVectorsCost>(
      v.plant.get(), v.body1->body_frame(), a_A, v.body2->body_frame(), b_B, c,
      v.context.get());

  return v;
}

// Given two free bodies with some given poses, evaluate the angle between
// vectors cost.
GTEST_TEST(AngleBetweenVectorsCostTest, TwoFreeBodies) {
  const Eigen::Vector3d a_A(1, 2, 3);
  const Eigen::Vector3d b_B(4, 5, 6);
  double c = 10;

  auto [plant, context, body1, body2, cost] =
      ConstructTwoFreeBodiesCost<double>(a_A, b_B, c);
  auto [plant_ad, context_ad, body1_ad, body2_ad, cost_ad] =
      ConstructTwoFreeBodiesCost<AutoDiffXd>(a_A, b_B, c);
  // Note: We are passing AngleBetweenVectorsCost here instead of capturing them
  // due to a known issue in C++:
  // https://stackoverflow.com/questions/46114214/lambda-implicit-capture-fails-with-variable-declared-from-structured-binding
  auto CheckCases = [](const AngleBetweenVectorsCost& angle_cost,
                       const AngleBetweenVectorsCost& angle_cost_ad,
                       const VectorXd& q, double y_expected) {
    // MultibodyPlant double, Eval double.
    VectorXd y(1);
    angle_cost.Eval(q, &y);
    EXPECT_NEAR(y[0], y_expected, 1e-12);

    // MultibodyPlant AutoDiffXd, Eval double.
    VectorXd y_ad_d(1);
    angle_cost_ad.Eval(q, &y_ad_d);
    EXPECT_NEAR(y_ad_d[0], y_expected, 1e-12);

    // MultibodyPlant double, Eval AutoDiffXd
    const VectorX<AutoDiffXd> q_ad = InitializeAutoDiff(q);
    VectorX<AutoDiffXd> y_d_ad(1);
    angle_cost.Eval(q_ad, &y_d_ad);
    EXPECT_NEAR(y_d_ad[0].value(), y_expected, 1e-12);

    // MultibodyPlant AutoDiffXd, Eval AutoDiffXd
    VectorX<AutoDiffXd> y_ad_ad(1);
    angle_cost_ad.Eval(q_ad, &y_ad_ad);
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
  double cos_theta =
      a_A.dot(b_B) / (a_A.norm() * b_B.norm());  // because A == B.
  CheckCases(*cost, *cost_ad, q, c * (1 - cos_theta));

  // X_WA, X_WB are arbitrary non-indentity transforms.
  RigidTransform<double> X_WA(RollPitchYaw<double>(0.32, -0.24, -0.51),
                              Vector3d(0.1, 0.3, 0.72));
  RigidTransform<double> X_WB(RollPitchYaw<double>(8.1, 0.42, -0.2),
                              Vector3d(-0.84, 0.2, 1.4));
  plant->SetFreeBodyPose(context.get(), *body1, X_WA);
  plant->SetFreeBodyPose(context.get(), *body2, X_WB);
  q = plant->GetPositions(*context);

  const Eigen::Vector3d a_W = X_WA.rotation() * a_A;
  const Eigen::Vector3d b_W = X_WB.rotation() * b_B;
  cos_theta = a_W.dot(b_W) / (a_W.norm() * b_W.norm());
  CheckCases(*cost, *cost_ad, q, c * (1 - cos_theta));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
