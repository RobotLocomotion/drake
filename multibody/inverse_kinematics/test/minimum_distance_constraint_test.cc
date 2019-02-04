#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

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
T Penalty(const T& distance, double minimum_distance,
          MinimumDistancePenaltyType penalty_type) {
  if (distance >= minimum_distance) {
    return 0;
  } else {
    const T x = distance / minimum_distance - 1;
    switch (penalty_type) {
      case MinimumDistancePenaltyType::kExponentiallySmoothedHinge: {
        using std::exp;
        const T penalty = -x * exp(1.0 / x);
        return penalty;
      }
      case MinimumDistancePenaltyType::kQuadraticallySmoothedHinge: {
        if (x > -1) {
          return x * x / 2;
        } else {
          return -0.5 - x;
        }
      }
      default:
        throw std::runtime_error("Should not reach here.");
    }
  }
}

AutoDiffVecXd EvalMinimumDistanceConstraintAutoDiff(
    const systems::Context<AutoDiffXd>& context,
    const MultibodyPlant<AutoDiffXd>& plant, double minimum_distance,
    MinimumDistancePenaltyType penalty_type) {
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
    if (distance < minimum_distance) {
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
          (inspector.X_FG(signed_distance_pair.id_A) *
           signed_distance_pair.p_ACa)
              .cast<AutoDiffXd>(),
          plant.world_frame(), &p_WCa);
      plant.CalcPointsPositions(
          context, plant.GetBodyFromFrameId(frame_B_id)->body_frame(),
          (inspector.X_FG(signed_distance_pair.id_B) *
           signed_distance_pair.p_BCb)
              .cast<AutoDiffXd>(),
          plant.world_frame(), &p_WCb);

      const AutoDiffXd distance_autodiff = sign * (p_WCa - p_WCb).norm();

      const Vector1<AutoDiffXd> penalty_autodiff = Vector1<AutoDiffXd>(
          Penalty(distance_autodiff, minimum_distance, penalty_type));

      y += penalty_autodiff;
    }
  }
  return y;
}
// Compare the result between Eval<double> and Eval<AutoDiffXd>. Also compare
// the gradient in Eval<AutoDiffXd> with the result in
// EvalMinimumDistanceConstraintAutoDiff.
void CheckMinimumDistanceConstraintEval(
    const MinimumDistanceConstraint& constraint,
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
      EvalMinimumDistanceConstraintAutoDiff(*context_autodiff, plant_autodiff,
                                            constraint.minimum_distance(),
                                            constraint.penalty_type());
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);
}

template <typename T>
Vector3<T> ComputeCollisionSphereCenterPosition(const Vector3<T>& p_WB,
                                                const Quaternion<T>& quat_WB,
                                                const Eigen::Isometry3d& X_BS) {
  Isometry3<T> X_WB;
  X_WB.linear() = quat_WB.toRotationMatrix();
  X_WB.translation() = p_WB;
  return X_WB * (X_BS.translation().cast<T>());
}

class TwoFreeSpheresMinimumDistanceTest : public TwoFreeSpheresTest {
 public:
  void CheckConstraintBounds(
      const MinimumDistanceConstraint& constraint) const {
    EXPECT_EQ(constraint.num_constraints(), 1);
    EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), Vector1d(0)));
    EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), Vector1d(0)));
  }

  void CheckConstraintEvalLargerThanMinimumDistance(
      const MinimumDistanceConstraint& constraint, const Eigen::Vector3d& p_WB1,
      const Eigen::Vector3d p_WB2) const {
    // distance larger than minimum_distance;
    // The penalty should be 0, so is the gradient.
    const Eigen::Quaterniond sphere1_quaternion(1, 0, 0, 0);
    const Eigen::Quaterniond sphere2_quaternion(1, 0, 0, 0);
    Eigen::Matrix<double, 14, 1> q;
    q << QuaternionToVectorWxyz(sphere1_quaternion), p_WB1,
        QuaternionToVectorWxyz(sphere2_quaternion), p_WB2;
    Eigen::VectorXd y_double(1);
    constraint.Eval(q, &y_double);
    Eigen::Matrix<AutoDiffXd, 14, 1> q_autodiff = math::initializeAutoDiff(q);
    AutoDiffVecXd y_autodiff(1);
    constraint.Eval(q_autodiff, &y_autodiff);

    EXPECT_EQ(y_double(0), 0);
    EXPECT_TRUE(CompareMatrices(y_autodiff(0).derivatives(),
                                Eigen::Matrix<double, 14, 1>::Zero()));
    CheckMinimumDistanceConstraintEval(constraint, q_autodiff, *plant_autodiff_,
                                       plant_context_autodiff_, 1E-12);
  }

  void CheckConstraintEvalSmallerThanMinimumDistance(
      const MinimumDistanceConstraint& constraint, const Eigen::Vector3d& p_WB1,
      const Eigen::Vector3d& p_WB2) const {
    // distance smaller than the minimum_distance, we can compute the penalty
    // and
    // the gradient by hand.
    const Eigen::Quaterniond sphere1_quaternion(1, 0, 0, 0);
    const Eigen::Quaterniond sphere2_quaternion(1, 0, 0, 0);
    Eigen::Matrix<double, 14, 1> q;
    q << QuaternionToVectorWxyz(sphere1_quaternion), p_WB1,
        QuaternionToVectorWxyz(sphere2_quaternion), p_WB2;
    const Eigen::Vector3d p_WS1 = ComputeCollisionSphereCenterPosition(
        p_WB1, sphere1_quaternion, X_B1S1_);
    const Eigen::Vector3d p_WS2 = ComputeCollisionSphereCenterPosition(
        p_WB2, sphere2_quaternion, X_B2S2_);
    const double distance_expected =
        (p_WS1 - p_WS2).norm() - radius1_ - radius2_;
    Eigen::VectorXd y_double(1);
    constraint.Eval(q, &y_double);
    const double penalty_expected =
        Penalty(distance_expected, constraint.minimum_distance(),
                constraint.penalty_type());
    const double tol{5 * kEps};
    EXPECT_NEAR(y_double(0), penalty_expected, tol);
    // Test with a non-ideneity gradient.
    Eigen::Matrix<double, 14, Eigen::Dynamic> dqdz(14, 2);
    for (int i = 0; i < 14; ++i) {
      dqdz(i, 0) = i;
      dqdz(i, 1) = i + 1;
    }
    Eigen::Matrix<AutoDiffXd, 14, 1> q_autodiff =
        math::initializeAutoDiffGivenGradientMatrix(q, dqdz);
    Vector3<AutoDiffXd> p_WB1_autodiff = q_autodiff.segment<3>(4);
    Vector3<AutoDiffXd> p_WB2_autodiff = q_autodiff.segment<3>(11);
    Quaternion<AutoDiffXd> quat_WB1_autodiff(q_autodiff(0), q_autodiff(1),
                                             q_autodiff(2), q_autodiff(3));
    Quaternion<AutoDiffXd> quat_WB2_autodiff(q_autodiff(7), q_autodiff(8),
                                             q_autodiff(9), q_autodiff(10));
    Vector3<AutoDiffXd> p_WS1_autodiff = ComputeCollisionSphereCenterPosition(
        p_WB1_autodiff, quat_WB1_autodiff, X_B1S1_);
    Vector3<AutoDiffXd> p_WS2_autodiff = ComputeCollisionSphereCenterPosition(
        p_WB2_autodiff, quat_WB2_autodiff, X_B2S2_);
    const AutoDiffXd distance_autodiff =
        (p_WS1_autodiff - p_WS2_autodiff).norm() - AutoDiffXd(radius1_) -
        AutoDiffXd(radius2_);
    const auto penalty_autodiff =
        Penalty(distance_autodiff, constraint.minimum_distance(),
                constraint.penalty_type());
    AutoDiffVecXd y_autodiff;
    constraint.Eval(q_autodiff, &y_autodiff);
    // The exponential penalty function -x * e^{1/x} can introduce relatively
    // large numerical error, when the distance is close to minimum_distance (
    // namely when x is close to 0).
    const double gradient_tol = 3E-12;
    EXPECT_TRUE(CompareMatrices(y_autodiff(0).derivatives(),
                                penalty_autodiff.derivatives(), gradient_tol));
    // Now evaluate the constraint using autodiff from start.
    CheckMinimumDistanceConstraintEval(constraint, q_autodiff, *plant_autodiff_,
                                       plant_context_autodiff_, gradient_tol);
  }

  void CheckConstraintEval(const MinimumDistanceConstraint& constraint) {
    // Distance between the spheres are larger than minimum_distance
    Eigen::Vector3d p_WB1(0.2, 1.2, 0.3);
    Eigen::Vector3d p_WB2(1.2, -0.4, 2.3);
    CheckConstraintEvalLargerThanMinimumDistance(constraint, p_WB1, p_WB2);

    // Two spheres are colliding.
    p_WB1 << 0.1, 0.2, 0.3;
    p_WB2 << 0.11, 0.21, 0.31;
    CheckConstraintEvalSmallerThanMinimumDistance(constraint, p_WB1, p_WB2);

    // Two spheres are separated, but their distance is smaller than
    // minimum_distance.
    p_WB1 << 0.1, 0.2, 0.3;
    p_WB2 = p_WB1 +
            Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3) *
                (radius1_ + radius2_ + 0.5 * constraint.minimum_distance());

    CheckConstraintEvalSmallerThanMinimumDistance(constraint, p_WB1, p_WB2);
  }
};

TEST_F(TwoFreeSpheresMinimumDistanceTest, ExponentialPenalty) {
  const double minimum_distance(0.1);
  const MinimumDistanceConstraint constraint(
      plant_double_, minimum_distance, plant_context_double_,
      MinimumDistancePenaltyType::kExponentiallySmoothedHinge);
  CheckConstraintBounds(constraint);

  CheckConstraintEval(constraint);
}

TEST_F(TwoFreeSpheresMinimumDistanceTest, SmoothedHingePenalty) {
  const double minimum_distance(0.1);
  const MinimumDistanceConstraint constraint(plant_double_, minimum_distance,
                                             plant_context_double_);
  // The default penalty type is smoothed hinge.
  EXPECT_EQ(constraint.penalty_type(),
            MinimumDistancePenaltyType::kQuadraticallySmoothedHinge);

  CheckConstraintBounds(constraint);

  CheckConstraintEval(constraint);
}

GTEST_TEST(MinimumDistanceConstraintTest,
           MultibodyPlantWithouthGeometrySource) {
  auto plant = ConstructTwoFreeBodiesPlant<double>();
  auto context = plant->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(plant.get(), 0.1, context.get()),
      std::invalid_argument,
      "MinimumDistanceConstraint: MultibodyPlant has not registered its "
      "geometry source with SceneGraph yet. Please refer to "
      "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
      "SceneGraph.");
}

TEST_F(TwoFreeSpheresTest, NonpositiveMinimalDistance) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(plant_double_, 0, plant_context_double_),
      std::invalid_argument,
      "MinimumDistanceConstraint: minimum_distance should be positive.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(plant_double_, -0.1, plant_context_double_),
      std::invalid_argument,
      "MinimumDistanceConstraint: minimum_distance should be positive.");
}
}  // namespace multibody
}  // namespace drake
