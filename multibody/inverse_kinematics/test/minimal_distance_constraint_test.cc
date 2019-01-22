#include "drake/multibody/inverse_kinematics/minimal_distance_constraint.h"

#include <limits>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
const double kEps = std::numeric_limits<double>::epsilon();
// The penalty function explained in section II.C of Whole-body Motion planning
// with Centroidal dynamics and full kinematics by Hongkai Dai, Andres
// Valenzuela and Russ Tedrake.
template <typename T>
T Penalty(const T& distance, double minimal_distance) {
  if (distance >= minimal_distance) {
    return 0;
  } else {
    const T x = distance / minimal_distance - 1;
    using std::exp;
    const T penalty = -x * exp(1.0 / x);
    return penalty;
  }
}

AutoDiffVecXd EvalMinimalDistanceConstraintAutoDiff(
    const systems::Context<AutoDiffXd>& context,
    const MultibodyPlant<AutoDiffXd>& plant, double minimal_distance) {
  AutoDiffVecXd y(1);
  y(0).value() = 0;
  y(0).derivatives().resize(
      math::autoDiffToGradientMatrix(plant.GetPositions(context)).cols());
  y(0).derivatives().setZero();
  const geometry::QueryObject<AutoDiffXd>& query_object =
      plant
          .EvalAbstractInput(context,
                             plant.get_geometry_query_input_port().get_index())
          ->GetValue<geometry::QueryObject<AutoDiffXd>>();

  const std::vector<geometry::SignedDistancePair<double>>
      signed_distance_pairs =
          query_object.ComputeSignedDistancePairwiseClosestPoints();

  for (const auto& signed_distance_pair : signed_distance_pairs) {
    const double distance = signed_distance_pair.distance;
    if (distance < minimal_distance) {
      const double sign = distance > 0 ? 1 : -1;

      Vector3<AutoDiffXd> p_WCa, p_WCb;
      const geometry::SceneGraphInspector<AutoDiffXd>& inspector =
          query_object.inspector();
      const geometry::FrameId frame_A_id =
          inspector.GetFrameId(signed_distance_pair.id_A);
      const geometry::FrameId frame_B_id =
          inspector.GetFrameId(signed_distance_pair.id_B);
      plant.CalcPointsPositions(
          context, plant.GetBodyFromFrameId(frame_A_id)->body_frame(),
          signed_distance_pair.p_ACa.cast<AutoDiffXd>(), plant.world_frame(),
          &p_WCa);
      plant.CalcPointsPositions(
          context, plant.GetBodyFromFrameId(frame_B_id)->body_frame(),
          signed_distance_pair.p_BCb.cast<AutoDiffXd>(), plant.world_frame(),
          &p_WCb);

      const AutoDiffXd distance_autodiff = sign * (p_WCa - p_WCb).norm();

      const Vector1<AutoDiffXd> penalty_autodiff =
          Vector1<AutoDiffXd>(Penalty(distance_autodiff, minimal_distance));

      y += penalty_autodiff;
    }
  }
  return y;
}
// Compare the result between Eval<double> and Eval<AutoDiffXd>. Also compare
// the gradient in Eval<AutoDiffXd> with the result in
// EvalMinimalDistanceConstraintAutoDiff.
void CheckMinimalDistanceConstraintEval(
    const MinimalDistanceConstraint& constraint,
    const Eigen::Ref<const AutoDiffVecXd>& x_autodiff,
    const MultibodyPlant<AutoDiffXd>& plant_autodiff,
    systems::Context<AutoDiffXd>* context_autodiff, double tol) {
  Eigen::VectorXd y_double;
  constraint.Eval(math::autoDiffToValueMatrix(x_autodiff), &y_double);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(x_autodiff, &y_autodiff);
  EXPECT_TRUE(
      CompareMatrices(y_double, math::autoDiffToValueMatrix(y_autodiff), tol));
  plant_autodiff.SetPositions(context_autodiff, x_autodiff);
  const AutoDiffVecXd y_autodiff_expected =
      EvalMinimalDistanceConstraintAutoDiff(*context_autodiff, plant_autodiff,
                                            constraint.minimal_distance());
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);
}

TEST_F(TwoFreeSpheresTest, MinimalDistanceConstraint) {
  const double minimal_distance(0.1);
  const MinimalDistanceConstraint constraint(plant_double_, minimal_distance,
                                             plant_context_double_);
  EXPECT_EQ(constraint.num_constraints(), 1);
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), Vector1d(0)));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), Vector1d(0)));

  const Eigen::Quaterniond sphere1_quaternion(1, 0, 0, 0);
  const Eigen::Quaterniond sphere2_quaternion(1, 0, 0, 0);
  // distance larger than minimal_distance;
  // The penalty should be 0, so is the gradient.
  Eigen::Vector3d sphere1_position(0.2, 1.2, 0.3);
  Eigen::Vector3d sphere2_position(1.2, -0.4, 2.3);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(sphere1_quaternion), sphere1_position,
      QuaternionToVectorWxyz(sphere2_quaternion), sphere2_position;
  Eigen::VectorXd y_double(1);
  constraint.Eval(q, &y_double);
  Eigen::Matrix<AutoDiffXd, 14, 1> q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y_autodiff(1);
  constraint.Eval(q_autodiff, &y_autodiff);

  EXPECT_EQ(y_double(0), 0);
  EXPECT_TRUE(CompareMatrices(y_autodiff(0).derivatives(),
                              Eigen::Matrix<double, 14, 1>::Zero()));
  CheckMinimalDistanceConstraintEval(constraint, q_autodiff, *plant_autodiff_,
                                     plant_context_autodiff_, 1E-12);

  // distance smaller than the minimal_distance, we can compute the penalty and
  // the gradient by hand.
  sphere1_position << 0.1, 0.2, 0.3;
  sphere2_position << 0.11, 0.21, 0.31;
  q << QuaternionToVectorWxyz(sphere1_quaternion), sphere1_position,
      QuaternionToVectorWxyz(sphere2_quaternion), sphere2_position;
  const double distance_expected =
      (sphere1_position - sphere2_position).norm() - radius1_ - radius2_;
  constraint.Eval(q, &y_double);
  const double penalty_expected = Penalty(distance_expected, minimal_distance);
  const double tol{5 * kEps};
  EXPECT_NEAR(y_double(0), penalty_expected, tol);
  const Eigen::Vector3d distance_normal =
      (sphere1_position - sphere2_position).normalized();
  Eigen::RowVectorXd ddistance_dq(14);
  ddistance_dq << Eigen::RowVector4d::Zero(), distance_normal.transpose(),
      Eigen::RowVector4d::Zero(), -distance_normal.transpose();
  const auto distance_autodiff = math::initializeAutoDiffGivenGradientMatrix(
      Vector1d(distance_expected), ddistance_dq);
  const auto penalty_autodiff = Penalty(distance_autodiff(0), minimal_distance);
  q_autodiff = math::initializeAutoDiff(q);
  constraint.Eval(q_autodiff, &y_autodiff);
  const double gradient_tol = 5 * kEps;
  EXPECT_TRUE(CompareMatrices(y_autodiff(0).derivatives(),
                              penalty_autodiff.derivatives(), gradient_tol));
  CheckMinimalDistanceConstraintEval(constraint, q_autodiff, *plant_autodiff_,
                                     plant_context_autodiff_, 10 * kEps);
}

GTEST_TEST(MinimalDistanceConstraintTest,
           MultibodyPlantWithouthGeometrySource) {
  auto plant = ConstructTwoFreeBodiesPlant<double>();
  auto context = plant->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimalDistanceConstraint(plant.get(), 0.1, context.get()),
      std::invalid_argument,
      "MinimalDistanceConstraint: MultibodyPlant has not registered its "
      "geometry source with SceneGraph yet. Please refer to "
      "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
      "SceneGraph.");
}

TEST_F(TwoFreeSpheresTest, NonpositiveMinimalDistance) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimalDistanceConstraint(plant_double_, 0, plant_context_double_),
      std::invalid_argument,
      "MinimalDistanceConstraint: minimal_distance should be positive.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimalDistanceConstraint(plant_double_, -0.1, plant_context_double_),
      std::invalid_argument,
      "MinimalDistanceConstraint: minimal_distance should be positive.");
}
}  // namespace multibody
}  // namespace drake
