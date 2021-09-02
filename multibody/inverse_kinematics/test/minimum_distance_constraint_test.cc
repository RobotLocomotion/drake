#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"

#include <limits>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/solvers/test_utilities/check_constraint_eval_nonsymbolic.h"

namespace drake {
namespace multibody {

using solvers::test::CheckConstraintEvalNonsymbolic;

constexpr int kNumPositionsForTwoFreeBodies{14};

template <typename T>
Vector3<T> ComputeCollisionSphereCenterPosition(
    const Vector3<T>& p_WB, const Quaternion<T>& quat_WB,
    const math::RigidTransformd& X_BS) {
  math::RigidTransform<T> X_WB{quat_WB, p_WB};
  return X_WB * (X_BS.translation().cast<T>());
}

class TwoFreeSpheresMinimumDistanceTest : public TwoFreeSpheresTest {
 public:
  void CheckConstraintBounds(
      const MinimumDistanceConstraint& constraint) const {
    EXPECT_EQ(constraint.num_constraints(), 1);
    EXPECT_TRUE(
        CompareMatrices(constraint.lower_bound(),
                        Vector1d(-std::numeric_limits<double>::infinity())));
    EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), Vector1d(1)));
  }

  void CheckConstraintEvalLargerThanInfluenceDistance(
      const MinimumDistanceConstraint& constraint, const Eigen::Vector3d& p_WB1,
      const Eigen::Vector3d p_WB2) const {
    // Distance larger than influence_distance;
    // The penalty should be 0, so is the gradient.
    const Eigen::Quaterniond sphere1_quaternion(1, 0, 0, 0);
    const Eigen::Quaterniond sphere2_quaternion(1, 0, 0, 0);
    Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1> q;
    q << QuaternionToVectorWxyz(sphere1_quaternion), p_WB1,
        QuaternionToVectorWxyz(sphere2_quaternion), p_WB2;
    Eigen::VectorXd y_double(1);
    constraint.Eval(q, &y_double);
    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff =
        math::InitializeAutoDiff(q);
    AutoDiffVecXd y_autodiff(1);
    constraint.Eval(q_autodiff, &y_autodiff);

    EXPECT_TRUE(constraint.CheckSatisfied(q));
    EXPECT_TRUE(constraint.CheckSatisfied(q_autodiff));
    EXPECT_EQ(y_double(0), 0);
    EXPECT_TRUE(CompareMatrices(
        y_autodiff(0).derivatives(),
        Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1>::Zero()));
    CheckConstraintEvalNonsymbolic(constraint, q_autodiff, 1E-12);
  }

  void CheckConstraintEvalBetweenMinimumAndInfluenceDistance(
      const MinimumDistanceConstraint& constraint, const Eigen::Vector3d& p_WB1,
      const Eigen::Vector3d p_WB2) const {
    const Eigen::Quaterniond sphere1_quaternion(1, 0, 0, 0);
    const Eigen::Quaterniond sphere2_quaternion(1, 0, 0, 0);
    Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1> q;
    q << QuaternionToVectorWxyz(sphere1_quaternion), p_WB1,
        QuaternionToVectorWxyz(sphere2_quaternion), p_WB2;
    Eigen::VectorXd y_double(1);
    constraint.Eval(q, &y_double);
    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff =
        math::InitializeAutoDiff(q);
    AutoDiffVecXd y_autodiff(1);
    constraint.Eval(q_autodiff, &y_autodiff);

    EXPECT_TRUE(constraint.CheckSatisfied(q));
    EXPECT_TRUE(constraint.CheckSatisfied(q_autodiff));
    // Check that the value is strictly greater than 0.
    EXPECT_GT(y_double(0), 0);
    CheckConstraintEvalNonsymbolic(constraint, q_autodiff, 1E-12);
  }

  void CheckConstraintEvalSmallerThanMinimumDistance(
      const MinimumDistanceConstraint& constraint, const Eigen::Vector3d& p_WB1,
      const Eigen::Vector3d p_WB2) const {
    const Eigen::Quaterniond sphere1_quaternion(1, 0, 0, 0);
    const Eigen::Quaterniond sphere2_quaternion(1, 0, 0, 0);
    Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1> q;
    q << QuaternionToVectorWxyz(sphere1_quaternion), p_WB1,
        QuaternionToVectorWxyz(sphere2_quaternion), p_WB2;
    Eigen::VectorXd y_double(1);
    constraint.Eval(q, &y_double);
    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff =
        math::InitializeAutoDiff(q);
    AutoDiffVecXd y_autodiff(1);
    constraint.Eval(q_autodiff, &y_autodiff);

    EXPECT_FALSE(constraint.CheckSatisfied(q));
    EXPECT_FALSE(constraint.CheckSatisfied(q_autodiff));
    // Check that the value is strictly greater than 1.
    EXPECT_GT(y_double(0), 1);
    CheckConstraintEvalNonsymbolic(constraint, q_autodiff, 1E-12);
  }

  void CheckConstraintEval(const MinimumDistanceConstraint& constraint) {
    // Distance between the spheres is larger than influence_distance
    Eigen::Vector3d p_WB1(0.2, 1.2, 0.3);
    Eigen::Vector3d p_WB2(1.2, -0.4, 2.3);
    CheckConstraintEvalLargerThanInfluenceDistance(constraint, p_WB1, p_WB2);

    // Distance between the spheres is larger than minimum_distance but less
    // than influence_distance
    p_WB1 << 0.1, 0.2, 0.3;
    p_WB2 = p_WB1 + Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3) *
                        (radius1_ + radius2_ +
                         0.5 * (constraint.influence_distance() +
                                constraint.minimum_distance()));
    CheckConstraintEvalBetweenMinimumAndInfluenceDistance(constraint, p_WB1,
                                                          p_WB2);

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
      solvers::ExponentiallySmoothedHingeLoss);
  CheckConstraintBounds(constraint);

  CheckConstraintEval(constraint);
}

TEST_F(TwoFreeSpheresMinimumDistanceTest, QuadraticallySmoothedHingePenalty) {
  const double minimum_distance(0.1);
  // The default penalty type is smoothed hinge.
  const MinimumDistanceConstraint constraint(plant_double_, minimum_distance,
                                             plant_context_double_);

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
      "Kinematic constraint: MultibodyPlant has not registered "
      "with a SceneGraph yet. Please refer to "
      "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
      "SceneGraph.");
}

GTEST_TEST(MinimumDistanceConstraintTest, MultibodyPlantWithoutCollisionPairs) {
  // Make sure MinimumDistanceConstraint evaluation works when
  // no collisions are possible in an MBP with no collision geometry.

  systems::DiagramBuilder<double> builder{};
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.0);
  AddTwoFreeBodiesToPlant(&plant);
  plant.Finalize();
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto plant_context =
      &diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  const double minimum_distance(0.1);
  const MinimumDistanceConstraint constraint(&plant, minimum_distance,
                                             plant_context);

  Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff =
      math::InitializeAutoDiff(
          Eigen::VectorXd::Zero(kNumPositionsForTwoFreeBodies));
  AutoDiffVecXd y_autodiff(1);
  constraint.Eval(q_autodiff, &y_autodiff);
}

TEST_F(TwoFreeSpheresTest, NonpositiveInfluenceDistanceOffset) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(plant_double_, 0.1, plant_context_double_, {},
                                0.0),
      std::invalid_argument,
      "MinimumDistanceConstraint: influence_distance_offset must be positive.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(plant_double_, 0.1, plant_context_double_, {},
                                -0.1),
      std::invalid_argument,
      "MinimumDistanceConstraint: influence_distance_offset must be positive.");
}

TEST_F(TwoFreeSpheresTest, NonfiniteInfluenceDistanceOffset) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(plant_double_, 0.1, plant_context_double_, {},
                                std::numeric_limits<double>::infinity()),
      std::invalid_argument,
      "MinimumDistanceConstraint: influence_distance_offset must be finite.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceConstraint(plant_double_, 0.1, plant_context_double_, {},
                                std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument,
      "MinimumDistanceConstraint: influence_distance_offset must be finite.");
}

template <typename T>
T BoxSphereSignedDistance(const Eigen::Vector3d& box_size, double radius,
                          const VectorX<T>& x) {
  const math::RigidTransform<T> X_WB(
      math::RotationMatrix<T>(Eigen::Quaternion<T>(x(0), x(1), x(2), x(3))),
      x.template segment<3>(4));
  const math::RigidTransform<T> X_WS(
      math::RotationMatrix<T>(Eigen::Quaternion<T>(x(7), x(8), x(9), x(10))),
      x.template tail<3>());
  return BoxSphereSignedDistance(box_size, radius, X_WB, X_WS);
}

TEST_F(BoxSphereTest, Test) {
  const double minimum_distance = 0.01;
  for (MinimumDistancePenaltyFunction penalty_function :
       {QuadraticallySmoothedHingeLoss, ExponentiallySmoothedHingeLoss}) {
    MinimumDistanceConstraint constraint(plant_double_, minimum_distance,
                                         plant_context_double_,
                                         penalty_function);

    auto check_eval_autodiff =
        [&constraint](const Eigen::VectorXd& q_val,
                      const Eigen::MatrixXd& q_gradient, double tol,
                      const Eigen::Vector3d& box_size, double radius) {
          AutoDiffVecXd x_autodiff =
              math::InitializeAutoDiff(q_val, q_gradient);

          CheckConstraintEvalNonsymbolic(constraint, x_autodiff, tol);
        };

    Eigen::VectorXd q(kNumPositionsForTwoFreeBodies);
    // First check q with normalized quaternion.
    q.head<4>() = Eigen::Vector4d(1, 0, 0, 0);
    q.segment<3>(4) = Eigen::Vector3d(0, 0, -5);
    q.segment<4>(7) = Eigen::Vector4d(0.1, 0.7, 0.8, 0.9).normalized();
    q.tail<3>() << 0, 0, 0;
    check_eval_autodiff(
        q,
        Eigen::MatrixXd::Identity(kNumPositionsForTwoFreeBodies,
                                  kNumPositionsForTwoFreeBodies),
        1E-13, box_size_, radius_);

    q.tail<3>() << 0, 0, 1.1;
    check_eval_autodiff(
        q,
        Eigen::MatrixXd::Identity(kNumPositionsForTwoFreeBodies,
                                  kNumPositionsForTwoFreeBodies),
        1E-13, box_size_, radius_);

    // box and sphere are separated, but the separation distance is smaller than
    // minimum_distance.
    q.tail<3>() << 0, 0, 1.005;
    check_eval_autodiff(
        q,
        Eigen::MatrixXd::Identity(kNumPositionsForTwoFreeBodies,
                                  kNumPositionsForTwoFreeBodies),
        1E-11, box_size_, radius_);

    q.tail<3>() << 0, 0, -1;
    check_eval_autodiff(
        q,
        Eigen::MatrixXd::Identity(kNumPositionsForTwoFreeBodies,
                                  kNumPositionsForTwoFreeBodies),
        1E-13, box_size_, radius_);

    // Test a q with unnormalized quaternion.
    q.head<4>() << 0.1, 1.7, 0.5, 0.3;
    q.segment<4>(7) << 1, 0.1, 0.3, 2;
    check_eval_autodiff(
        q,
        Eigen::MatrixXd::Identity(kNumPositionsForTwoFreeBodies,
                                  kNumPositionsForTwoFreeBodies),
        1E-12, box_size_, radius_);

    // Now check if constraint constructed from MBP<ADS> gives the same result
    // as that from MBP<double>
    const MinimumDistanceConstraint constraint_from_autodiff(
        plant_autodiff_, minimum_distance, plant_context_autodiff_,
        penalty_function);
    // Set dq to arbitrary values.
    Eigen::Matrix<double, 14, 2> dq;
    for (int i = 0; i < 14; ++i) {
      dq(i, 0) = std::sin(i + 1);
      dq(i, 1) = 2 * i - 1;
    }
    /* tolerance for checking numerical gradient vs analytical gradient. The
     * numerical gradient is only accurate up to 5E-6 */
    const double gradient_tol = 5E-6;
    TestKinematicConstraintEval(constraint, constraint_from_autodiff, q, dq,
                                gradient_tol);
  }
}
}  // namespace multibody
}  // namespace drake
