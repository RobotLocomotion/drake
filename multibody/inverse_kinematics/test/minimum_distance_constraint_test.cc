#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/solvers/minimum_value_constraint.h"
#include "drake/solvers/test_utilities/check_constraint_eval_nonsymbolic.h"

namespace drake {
namespace multibody {
namespace {
const double kInf = std::numeric_limits<double>::infinity();

constexpr int kNumPositionsForTwoFreeBodies{14};
}  // namespace

using solvers::ExponentiallySmoothedHingeLoss;
using solvers::QuadraticallySmoothedHingeLoss;
using solvers::test::CheckConstraintEvalNonsymbolic;

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
      const MinimumDistanceLowerBoundConstraint& constraint) const {
    EXPECT_EQ(constraint.num_constraints(), 1);
    EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), Vector1d(-kInf)));
    EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), Vector1d(1)));
  }

  void CheckConstraintBounds(
      const MinimumDistanceUpperBoundConstraint& constraint) const {
    EXPECT_EQ(constraint.num_constraints(), 1);
    EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), Vector1d(1)));
    EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), Vector1d(kInf)));
  }

  void ConstructQ(const Eigen::Vector3d& p_WB1, const Eigen::Vector3d& p_WB2,
                  Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1>* q,
                  Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1>*
                      q_autodiff) const {
    const Eigen::Quaterniond sphere1_quaternion =
        Eigen::Quaterniond::Identity();
    const Eigen::Quaterniond sphere2_quaternion =
        Eigen::Quaterniond::Identity();
    *q << QuaternionToVectorWxyz(sphere1_quaternion), p_WB1,
        QuaternionToVectorWxyz(sphere2_quaternion), p_WB2;
    *q_autodiff = math::InitializeAutoDiff(*q);
  }

  template <typename C>
  void CheckConstraintEvalLargerThanInfluenceDistance(
      const C& constraint, const Eigen::Vector3d& p_WB1,
      const Eigen::Vector3d& p_WB2, bool expect_satisfied) const {
    // Distance larger than influence_distance;
    // The penalty should be 0, so is the gradient.
    Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1> q;
    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff;
    ConstructQ(p_WB1, p_WB2, &q, &q_autodiff);
    Eigen::VectorXd y_double(constraint.num_constraints());
    constraint.Eval(q, &y_double);
    AutoDiffVecXd y_autodiff(constraint.num_constraints());
    constraint.Eval(q_autodiff, &y_autodiff);

    if (expect_satisfied) {
      EXPECT_TRUE(constraint.CheckSatisfied(q));
      EXPECT_TRUE(constraint.CheckSatisfied(q_autodiff));
    } else {
      EXPECT_FALSE(constraint.CheckSatisfied(q));
      EXPECT_FALSE(constraint.CheckSatisfied(q_autodiff));
    }
    for (int i = 0; i < constraint.num_constraints(); ++i) {
      EXPECT_TRUE(CompareMatrices(
          y_autodiff(i).derivatives(),
          Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1>::Zero()));
    }
    CheckConstraintEvalNonsymbolic(constraint, q_autodiff, 1E-12);
  }

  void CheckConstraintEvalBetweenMinimumUpperAndInfluenceDistance(
      const MinimumDistanceUpperBoundConstraint& constraint,
      const Eigen::Vector3d& p_WB1, const Eigen::Vector3d& p_WB2) const {
    DRAKE_DEMAND(std::isfinite(constraint.distance_bound()));
    Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1> q;
    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff;
    ConstructQ(p_WB1, p_WB2, &q, &q_autodiff);
    Eigen::VectorXd y_double(constraint.num_constraints());
    constraint.Eval(q, &y_double);
    AutoDiffVecXd y_autodiff(constraint.num_constraints());
    constraint.Eval(q_autodiff, &y_autodiff);

    EXPECT_FALSE(constraint.CheckSatisfied(q));
    EXPECT_FALSE(constraint.CheckSatisfied(q_autodiff));
    // Check that the value is strictly greater than 0.
    CheckConstraintEvalNonsymbolic(constraint, q_autodiff, 1E-12);
  }

  void CheckConstraintEvalBetweenMinimumLowerAndInfluenceDistance(
      const MinimumDistanceLowerBoundConstraint& constraint,
      const Eigen::Vector3d& p_WB1, const Eigen::Vector3d& p_WB2) const {
    DRAKE_DEMAND(std::isfinite(constraint.distance_bound()));
    Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1> q;
    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff;
    ConstructQ(p_WB1, p_WB2, &q, &q_autodiff);
    Eigen::VectorXd y_double(constraint.num_constraints());
    constraint.Eval(q, &y_double);
    AutoDiffVecXd y_autodiff(constraint.num_constraints());
    constraint.Eval(q_autodiff, &y_autodiff);

    EXPECT_TRUE(constraint.CheckSatisfied(q));
    EXPECT_TRUE(constraint.CheckSatisfied(q_autodiff));
    CheckConstraintEvalNonsymbolic(constraint, q_autodiff, 1E-12);
  }

  void CheckConstraintEvalSmallerThanMinimumLowerDistance(
      const MinimumDistanceLowerBoundConstraint& constraint,
      const Eigen::Vector3d& p_WB1, const Eigen::Vector3d& p_WB2) const {
    Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1> q;
    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff;
    ConstructQ(p_WB1, p_WB2, &q, &q_autodiff);
    Eigen::VectorXd y_double(constraint.num_constraints());
    constraint.Eval(q, &y_double);
    AutoDiffVecXd y_autodiff(constraint.num_constraints());
    constraint.Eval(q_autodiff, &y_autodiff);

    EXPECT_FALSE(constraint.CheckSatisfied(q));
    EXPECT_FALSE(constraint.CheckSatisfied(q_autodiff));
    CheckConstraintEvalNonsymbolic(constraint, q_autodiff, 1E-12);
  }

  void CheckConstraintEvalSmallerThanMinimumUpperDistance(
      const MinimumDistanceUpperBoundConstraint& constraint,
      const Eigen::Vector3d& p_WB1, const Eigen::Vector3d& p_WB2) const {
    Eigen::Matrix<double, kNumPositionsForTwoFreeBodies, 1> q;
    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff;
    ConstructQ(p_WB1, p_WB2, &q, &q_autodiff);
    Eigen::VectorXd y_double(constraint.num_constraints());
    constraint.Eval(q, &y_double);
    AutoDiffVecXd y_autodiff(constraint.num_constraints());
    constraint.Eval(q_autodiff, &y_autodiff);

    EXPECT_TRUE(constraint.CheckSatisfied(q));
    EXPECT_TRUE(constraint.CheckSatisfied(q_autodiff));
    CheckConstraintEvalNonsymbolic(constraint, q_autodiff, 1E-12);
  }

  void CheckConstraintEval(
      const MinimumDistanceLowerBoundConstraint& constraint) const {
    // Distance between the spheres is larger than influence_distance
    Eigen::Vector3d p_WB1(0.2, 1.2, 0.3);
    Eigen::Vector3d p_WB2(1.2, -0.4, 2.3);
    CheckConstraintEvalLargerThanInfluenceDistance(constraint, p_WB1, p_WB2,
                                                   true /* expect satisfied */);

    // Distance between the spheres is larger than minimum_distance_lower but
    // less than influence distance
    p_WB1 << 0.2, 1.2, 0.3;
    p_WB2 = p_WB1 + Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3) *
                        (radius1_ + radius2_ +
                         0.5 * (constraint.distance_bound() +
                                constraint.influence_distance()));
    CheckConstraintEvalBetweenMinimumLowerAndInfluenceDistance(constraint,
                                                               p_WB1, p_WB2);

    // Two spheres are colliding.
    p_WB1 << 0.1, 0.2, 0.3;
    p_WB2 << 0.11, 0.21, 0.31;
    CheckConstraintEvalSmallerThanMinimumLowerDistance(constraint, p_WB1,
                                                       p_WB2);

    // Two spheres are separated, but their distance is smaller than
    // minimum_distance_lower.
    p_WB1 << 0.1, 0.2, 0.3;
    p_WB2 =
        p_WB1 + Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3) *
                    (radius1_ + radius2_ + 0.5 * constraint.distance_bound());

    CheckConstraintEvalSmallerThanMinimumLowerDistance(constraint, p_WB1,
                                                       p_WB2);
  }

  void CheckConstraintEval(
      const MinimumDistanceUpperBoundConstraint& constraint) const {
    // Distance between the spheres is larger than influence_distance
    Eigen::Vector3d p_WB1(0.2, 1.2, 0.3);
    Eigen::Vector3d p_WB2(1.2, -0.4, 2.3);
    CheckConstraintEvalLargerThanInfluenceDistance(
        constraint, p_WB1, p_WB2, false /* expect satisfied */);

    // Distance between the spheres is larger than minimum_distance_upper but
    // less than influence distance
    p_WB1 << 0.2, 1.2, 0.3;
    p_WB2 = p_WB1 + Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3) *
                        (radius1_ + radius2_ +
                         0.5 * (constraint.distance_bound() +
                                constraint.influence_distance()));
    CheckConstraintEvalBetweenMinimumUpperAndInfluenceDistance(constraint,
                                                               p_WB1, p_WB2);

    // Two spheres are colliding.
    p_WB1 << 0.1, 0.2, 0.3;
    p_WB2 << 0.11, 0.21, 0.31;
    DRAKE_DEMAND((p_WB1 - p_WB2).norm() - radius1_ - radius2_ <
                 constraint.distance_bound());
    CheckConstraintEvalSmallerThanMinimumUpperDistance(constraint, p_WB1,
                                                       p_WB2);

    // Two spheres are separated, but their distance is smaller than
    // minimum_distance_lower.
    p_WB1 << 0.1, 0.2, 0.3;
    p_WB2 =
        p_WB1 + Eigen::Vector3d(1.0 / 3, 2.0 / 3, 2.0 / 3) *
                    (radius1_ + radius2_ + 0.5 * constraint.distance_bound());

    CheckConstraintEvalSmallerThanMinimumUpperDistance(constraint, p_WB1,
                                                       p_WB2);
  }
};

/**
 Constructs a diagram that contains N free-floating spheres.
 */
template <typename T>
class NFreeSpheresModel {
 public:
  explicit NFreeSpheresModel(int num_spheres) : num_spheres_{num_spheres} {
    systems::DiagramBuilder<T> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, 0.0);
    const double mass{1};
    const math::RigidTransformd X_BS{};
    const Eigen::Vector3d p_AoAcm_A = X_BS.translation();
    const RotationalInertia<double> I_AAcm_A =
        UnitInertia<double>::SolidSphere(radius_);
    const SpatialInertia<double> M_AAo_A =
        SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A,
                                                       I_AAcm_A);
    for (int i = 0; i < num_spheres; ++i) {
      auto& body = plant_->AddRigidBody("sphere" + std::to_string(i), M_AAo_A);
      plant_->RegisterCollisionGeometry(body, X_BS, geometry::Sphere(radius_),
                                        fmt::format("sphere{}_collision", i),
                                        multibody::CoulombFriction<double>());
      sphere_frame_indices_.push_back(body.body_frame().index());
    }
    plant_->Finalize();

    diagram_ = builder.Build();

    diagram_context_ = diagram_->CreateDefaultContext();
  }

  double radius() const { return radius_; }

  const systems::Diagram<T>& diagram() const { return *diagram_; }

  const MultibodyPlant<T>& plant() const { return *plant_; }

  const systems::Context<T>& plant_context() const {
    return diagram_->GetSubsystemContext(*plant_, *diagram_context_);
  }

  systems::Context<T>& get_mutable_plant_context() {
    return plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  }

 private:
  int num_spheres_;
  const double radius_{0.01};
  std::unique_ptr<systems::Diagram<T>> diagram_;
  // This plant points to a sub-system in diagram_.
  MultibodyPlant<T>* plant_{nullptr};
  // This scene_graph points to a sub-system in diagram_.
  geometry::SceneGraph<T>* scene_graph_{nullptr};

  std::vector<FrameIndex> sphere_frame_indices_;

  std::unique_ptr<systems::Context<T>> diagram_context_;
};

TEST_F(TwoFreeSpheresMinimumDistanceTest, ExponentialPenaltyLowerBound) {
  const double minimum_distance_lower{0.1};
  const double influence_distance_offset{1.0};
  const MinimumDistanceLowerBoundConstraint constraint(
      plant_double_, minimum_distance_lower, plant_context_double_,
      solvers::ExponentiallySmoothedHingeLoss, influence_distance_offset);
  CheckConstraintBounds(constraint);

  CheckConstraintEval(constraint);
}

TEST_F(TwoFreeSpheresMinimumDistanceTest, ExponentialPenaltyUpperBound) {
  const double minimum_distance_upper{0.2};
  const double influence_distance_offset{1.0};
  const MinimumDistanceUpperBoundConstraint constraint(
      plant_double_, minimum_distance_upper, plant_context_double_,
      influence_distance_offset, solvers::ExponentiallySmoothedHingeLoss);
  CheckConstraintBounds(constraint);

  CheckConstraintEval(constraint);
}

TEST_F(TwoFreeSpheresMinimumDistanceTest,
       QuadraticallySmoothedHingePenaltyLowerBound) {
  const double minimum_distance_lower{0.1};
  const double influence_distance_offset{1.0};
  // The default penalty type is smoothed hinge.
  const MinimumDistanceLowerBoundConstraint constraint(
      plant_double_, minimum_distance_lower, plant_context_double_,
      solvers::QuadraticallySmoothedHingeLoss, influence_distance_offset);

  CheckConstraintBounds(constraint);

  CheckConstraintEval(constraint);
}

TEST_F(TwoFreeSpheresMinimumDistanceTest,
       QuadraticallySmoothedHingePenaltyUpperBound) {
  const double minimum_distance_upper{0.2};
  const double influence_distance_offset{1.0};
  // The default penalty type is smoothed hinge.
  const MinimumDistanceUpperBoundConstraint constraint(
      plant_double_, minimum_distance_upper, plant_context_double_,
      influence_distance_offset, solvers::QuadraticallySmoothedHingeLoss);

  CheckConstraintBounds(constraint);

  CheckConstraintEval(constraint);
}

GTEST_TEST(MinimumDistanceLowerBoundConstraintTest,
           MultibodyPlantWithouthGeometrySource) {
  auto plant = ConstructTwoFreeBodiesPlant<double>();
  auto context = plant->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceLowerBoundConstraint(plant.get(), 0.1, context.get()),
      "Kinematic constraint: MultibodyPlant has not registered "
      "with a SceneGraph yet. Please refer to "
      "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
      "SceneGraph.");
}

GTEST_TEST(MinimumDistanceUpperBoundConstraintTest,
           MultibodyPlantWithouthGeometrySource) {
  auto plant = ConstructTwoFreeBodiesPlant<double>();
  auto context = plant->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceUpperBoundConstraint(plant.get(), 0.1, context.get(), 1),
      "Kinematic constraint: MultibodyPlant has not registered "
      "with a SceneGraph yet. Please refer to "
      "AddMultibodyPlantSceneGraph on how to connect MultibodyPlant to "
      "SceneGraph.");
}

GTEST_TEST(MinimumDistanceLowerUpperBoundConstraintTest,
           MultibodyPlantWithoutCollisionPairs) {
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

  {
    const double minimum_distance_lower(0.1);
    const MinimumDistanceLowerBoundConstraint lower_bound_constraint(
        &plant, minimum_distance_lower, plant_context);

    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff =
        math::InitializeAutoDiff(
            Eigen::VectorXd::Zero(kNumPositionsForTwoFreeBodies));
    AutoDiffVecXd y_autodiff(1);
    lower_bound_constraint.Eval(q_autodiff, &y_autodiff);
  }
  {
    const double minimum_distance_upper(0.1);
    const double influence_distance_offset(1);
    const MinimumDistanceUpperBoundConstraint upper_bound_constraint(
        &plant, minimum_distance_upper, plant_context,
        influence_distance_offset);

    Eigen::Matrix<AutoDiffXd, kNumPositionsForTwoFreeBodies, 1> q_autodiff =
        math::InitializeAutoDiff(
            Eigen::VectorXd::Zero(kNumPositionsForTwoFreeBodies));
    AutoDiffVecXd y_autodiff(1);
    upper_bound_constraint.Eval(q_autodiff, &y_autodiff);
  }
}

TEST_F(TwoFreeSpheresTest, NonpositiveInfluenceDistanceOffsetLowerBound) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceLowerBoundConstraint(plant_double_, 0.1,
                                          plant_context_double_, {}, -0.05),
      "MinimumDistanceLowerBoundConstraint: influence_distance=0.05, must be "
      "larger than bound=0.1; equivalently, influence_distance_offset=-0.05, "
      "but it "
      "needs to be positive.");
  // We've already tested the message contents for a *smaller* influence
  // distance. We'll assume the message is the same for an equal distance and
  // just confirm the throw.
  EXPECT_THROW(MinimumDistanceLowerBoundConstraint(
                   plant_double_, 0.1, plant_context_double_, {}, 0.0),
               std::exception);
}

TEST_F(TwoFreeSpheresTest, NonpositiveInfluenceDistanceOffsetUpperBound) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      MinimumDistanceUpperBoundConstraint(plant_double_, 0.1,
                                          plant_context_double_, -0.05, {}),
      "MinimumDistanceUpperBoundConstraint: influence_distance=0.05, must be "
      "larger than bound=0.1; equivalently, influence_distance_offset=-0.05, "
      "but it "
      "needs to be positive.");
  // We've already tested the message contents for a *smaller* influence
  // distance. We'll assume the message is the same for an equal distance and
  // just confirm the throw.
  DRAKE_EXPECT_THROWS_MESSAGE(MinimumDistanceLowerBoundConstraint(
                                  plant_double_, 0.1, plant_context_double_, {},
                                  std::numeric_limits<double>::infinity()),
                              "MinimumDistanceLowerBoundConstraint: "
                              "influence_distance must be finite.");
  EXPECT_THROW(MinimumDistanceLowerBoundConstraint(
                   plant_double_, 0.1, plant_context_double_, {}, 0.0),
               std::exception);
  DRAKE_EXPECT_THROWS_MESSAGE(MinimumDistanceUpperBoundConstraint(
                                  plant_double_, 0.1, plant_context_double_,
                                  std::numeric_limits<double>::infinity(), {}),
                              "MinimumDistanceUpperBoundConstraint: "
                              "influence_distance must be finite.");
  EXPECT_THROW(MinimumDistanceUpperBoundConstraint(
                   plant_double_, 0.1, plant_context_double_, 0.0, {}),
               std::exception);
}

TEST_F(BoxSphereTest, Test) {
  const double minimum_distance_lower = 0.01;
  const double minimum_distance_upper = 0.1;
  const double influence_distance_offset1 = 1;
  const double influence_distance_offset2 = 2;
  for (solvers::MinimumValuePenaltyFunction penalty_function :
       {QuadraticallySmoothedHingeLoss, ExponentiallySmoothedHingeLoss}) {
    MinimumDistanceLowerBoundConstraint lower_bound_constraint(
        plant_double_, minimum_distance_lower, plant_context_double_,
        penalty_function, influence_distance_offset1);
    MinimumDistanceUpperBoundConstraint upper_bound_constraint(
        plant_double_, minimum_distance_upper, plant_context_double_,
        influence_distance_offset2, penalty_function);

    auto check_eval_autodiff = [&lower_bound_constraint,
                                &upper_bound_constraint](
                                   const Eigen::VectorXd& q_val,
                                   const Eigen::MatrixXd& q_gradient,
                                   double tol, const Eigen::Vector3d& box_size,
                                   double radius) {
      AutoDiffVecXd x_autodiff = math::InitializeAutoDiff(q_val, q_gradient);

      CheckConstraintEvalNonsymbolic(lower_bound_constraint, x_autodiff, tol);
      CheckConstraintEvalNonsymbolic(upper_bound_constraint, x_autodiff, tol);
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
        1E-10, box_size_, radius_);

    q.tail<3>() << 0, 0, 1.1;
    check_eval_autodiff(
        q,
        Eigen::MatrixXd::Identity(kNumPositionsForTwoFreeBodies,
                                  kNumPositionsForTwoFreeBodies),
        1E-10, box_size_, radius_);

    // box and sphere are separated, but the separation distance is smaller than
    // minimum_distance.
    q.tail<3>() << 0, 0, 1.005;
    check_eval_autodiff(
        q,
        Eigen::MatrixXd::Identity(kNumPositionsForTwoFreeBodies,
                                  kNumPositionsForTwoFreeBodies),
        1E-10, box_size_, radius_);

    q.tail<3>() << 0, 0, -1;
    check_eval_autodiff(
        q,
        Eigen::MatrixXd::Identity(kNumPositionsForTwoFreeBodies,
                                  kNumPositionsForTwoFreeBodies),
        1E-10, box_size_, radius_);

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
    const MinimumDistanceLowerBoundConstraint
        lower_bound_constraint_from_autodiff(
            plant_autodiff_, minimum_distance_lower, plant_context_autodiff_,
            penalty_function, influence_distance_offset1);
    const MinimumDistanceUpperBoundConstraint
        upper_bound_constraint_from_autodiff(
            plant_autodiff_, minimum_distance_upper, plant_context_autodiff_,
            influence_distance_offset2, penalty_function);
    // Set dq to arbitrary values.
    Eigen::Matrix<double, 14, 2> dq;
    for (int i = 0; i < 14; ++i) {
      dq(i, 0) = std::sin(i + 1);
      dq(i, 1) = 2 * i - 1;
    }
    /* tolerance for checking numerical gradient vs analytical gradient. The
     * numerical gradient is only accurate up to 5E-6 */
    const double gradient_tol = 5E-6;
    TestKinematicConstraintEval(lower_bound_constraint,
                                lower_bound_constraint_from_autodiff, q, dq,
                                gradient_tol);
    TestKinematicConstraintEval(upper_bound_constraint,
                                upper_bound_constraint_from_autodiff, q, dq,
                                gradient_tol, 1E-11 /* tol */);
  }
}

GTEST_TEST(ThreeSpheresTest, SomeLargerThanInfluenceSomeSmallerThanLowerBound) {
  // Test the case with three spheres. Some pair of spheres have distance >
  // d_influence, and some pairs have distance < d_min_lower.
  NFreeSpheresModel<double> three_spheres(3);
  const double minimum_distance_lower = 0.05;
  const double d_influence = 0.06;
  // Indices into the q vector for each sphere's position.
  Eigen::Index kSpheres[] = {4, 11, 18};
  Eigen::VectorXd q =
      three_spheres.plant().GetPositions(three_spheres.plant_context());
  // Position for sphere 0.
  q.segment<3>(kSpheres[0]) << 0, 0, 0;
  // Position for sphere 1.
  q.segment<3>(kSpheres[1]) << 0, 0, 0.05;
  // Position for sphere 2.
  q.segment<3>(kSpheres[2]) << 0, 0, 0.1;
  // Make sure that distance(sphere0, sphere1) < d_min.
  ASSERT_LT((q.segment<3>(kSpheres[0]) - q.segment<3>(kSpheres[1])).norm() -
                2 * three_spheres.radius(),
            minimum_distance_lower);
  // Make sure distance(sphere0, sphere2) > d_influence.
  ASSERT_GT((q.segment<3>(kSpheres[2]) - q.segment<3>(kSpheres[0])).norm() -
                2 * three_spheres.radius(),
            d_influence);
  for (solvers::MinimumValuePenaltyFunction penalty_function :
       {QuadraticallySmoothedHingeLoss, ExponentiallySmoothedHingeLoss}) {
    MinimumDistanceLowerBoundConstraint dut(
        &(three_spheres.plant()), minimum_distance_lower,
        &(three_spheres.get_mutable_plant_context()), penalty_function,
        d_influence - minimum_distance_lower);
    Eigen::VectorXd y_val;
    dut.Eval(q, &y_val);
    EXPECT_TRUE((y_val.array() < dut.lower_bound().array()).any() ||
                (y_val.array() > dut.upper_bound().array()).any());
  }
}

GTEST_TEST(ThreeSpheresTest, SomeLargerThanInfluenceSomeSmallerThanUpperBound) {
  // Test the case with three spheres. Some pair of spheres have distance >
  // d_influence, and some pairs have distance < minimum-distance_upper.
  NFreeSpheresModel<double> three_spheres(3);
  const double minimum_distance_upper = 0.05;
  const double d_influence = 0.06;
  // Indices into the q vector for each sphere's position.
  Eigen::Index kSpheres[] = {4, 11, 18};
  Eigen::VectorXd q =
      three_spheres.plant().GetPositions(three_spheres.plant_context());
  // Position for sphere 0.
  q.segment<3>(kSpheres[0]) << 0, 0, 0;
  // Position for sphere 1.
  q.segment<3>(kSpheres[1]) << 0, 0, 0.05;
  // Position for sphere 2.
  q.segment<3>(kSpheres[2]) << 0, 0, 0.1;
  // Make sure that distance(sphere0, sphere1) < d_min.
  ASSERT_LT((q.segment<3>(kSpheres[0]) - q.segment<3>(kSpheres[1])).norm() -
                2 * three_spheres.radius(),
            minimum_distance_upper);
  // Make sure distance(sphere0, sphere2) > d_influence.
  ASSERT_GT((q.segment<3>(kSpheres[2]) - q.segment<3>(kSpheres[0])).norm() -
                2 * three_spheres.radius(),
            d_influence);
  for (solvers::MinimumValuePenaltyFunction penalty_function :
       {QuadraticallySmoothedHingeLoss, ExponentiallySmoothedHingeLoss}) {
    MinimumDistanceUpperBoundConstraint dut(
        &(three_spheres.plant()), minimum_distance_upper,
        &(three_spheres.get_mutable_plant_context()),
        d_influence - minimum_distance_upper, penalty_function);
    EXPECT_TRUE(dut.CheckSatisfied(q));
  }
}

TEST_F(SpheresAndWallsTest, TestWithCollisionCheckerLowerUpperBounds) {
  // Test MinimumDistanceLower/UpperBoundConstraint constructed with
  // CollisionChecker.
  planning::CollisionCheckerParams params;
  params.model = builder_.Build();
  params.robot_model_instances.push_back(multibody::default_model_instance());
  auto quaternion_difference = [](const Eigen::Ref<const Eigen::Vector4d>& q1,
                                  const Eigen::Ref<const Eigen::Vector4d>& q2) {
    // Compute 1-cos(theta/2) where theta is the angle between the two
    // quaternions.
    return 1 - (Eigen::Quaterniond(q1(0), q1(1), q1(2), q1(3)).conjugate() *
                Eigen::Quaterniond(q2(0), q2(1), q2(2), q2(3)))
                   .w();
  };
  params.configuration_distance_function = [quaternion_difference](
                                               const Eigen::VectorXd& q1,
                                               const Eigen::VectorXd& q2) {
    return quaternion_difference(q1.head<4>(), q2.head<4>()) +
           quaternion_difference(q1.segment<4>(7), q2.segment<4>(7)) +
           (q1.segment<3>(4) - q2.segment<3>(4)).norm() +
           (q1.segment<3>(11) - q2.segment<3>(11)).norm();
  };
  params.edge_step_size = 0.05;
  planning::SceneGraphCollisionChecker collision_checker(std::move(params));
  auto collision_checker_context =
      collision_checker.MakeStandaloneModelContext();
  double influence_distance = 0.2;

  double minimum_distance_lower = 0.05;
  double minimum_distance_upper = 0.15;
  MinimumDistanceLowerBoundConstraint dut_lower_bound(
      &collision_checker, minimum_distance_lower,
      collision_checker_context.get(), solvers::QuadraticallySmoothedHingeLoss,
      influence_distance - minimum_distance_lower);
  MinimumDistanceUpperBoundConstraint dut_upper_bound(
      &collision_checker, minimum_distance_upper,
      collision_checker_context.get(),
      influence_distance - minimum_distance_upper,
      solvers::QuadraticallySmoothedHingeLoss);
  EXPECT_EQ(dut_lower_bound.num_constraints(), 1);
  EXPECT_EQ(dut_lower_bound.num_vars(),
            collision_checker.plant().num_positions());
  EXPECT_EQ(dut_upper_bound.num_constraints(), 1);
  EXPECT_EQ(dut_upper_bound.num_vars(),
            collision_checker.plant().num_positions());

  auto check_constraint =
      [&collision_checker, &dut_lower_bound, &dut_upper_bound](
          const Eigen::Vector3d& p_WS1, const Eigen::Vector3d& p_WS2,
          bool is_lower_satisfied, bool is_upper_satisfied) {
        Eigen::VectorXd q(collision_checker.plant().num_positions());
        // sphere 1 quaternion.
        q.head<4>() << 1, 0, 0, 0;
        q.segment<3>(4) = p_WS1;
        // sphere 2 quaternion.
        q.segment<4>(7) << 1, 0, 0, 0;
        q.tail<3>() = p_WS2;
        EXPECT_EQ(dut_lower_bound.CheckSatisfied(q), is_lower_satisfied);
        EXPECT_EQ(dut_upper_bound.CheckSatisfied(q), is_upper_satisfied);
        // Make sure the gradient is correct.
        const AutoDiffVecXd q_ad = math::InitializeAutoDiff(q);

        auto check_eval = [&q, &q_ad](const solvers::Constraint& dut) {
          AutoDiffVecXd y_ad;
          dut.Eval(q_ad, &y_ad);

          math::NumericalGradientOption options(
              math::NumericalGradientMethod::kCentral);
          const Eigen::MatrixXd grad_numeric =
              math::ComputeNumericalGradient<Eigen::VectorXd, Eigen::VectorXd,
                                             Eigen::VectorXd>(
                  [&dut](const Eigen::VectorXd& x, Eigen::VectorXd* y) {
                    dut.Eval(x, y);
                  },
                  q, options);
          EXPECT_TRUE(
              CompareMatrices(math::ExtractGradient(y_ad), grad_numeric, 1E-7));
        };
        check_eval(dut_lower_bound);
        check_eval(dut_upper_bound);
      };

  // The two spheres almost coincide (do not set the two spheres to exactly
  // coincide as the distance jacobian is ill defined at that point).
  check_constraint(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1E-4, 0, 0), false,
                   true);

  // The two spheres are separated, but the distance is smaller than
  // minimum_distance_lower
  check_constraint(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(0.9 * minimum_distance_lower + 2 * radius_, 0, 0), false,
      true);
  // The distance between the two spheres is between minimum_distance_lower
  // and minimum_distance_upper.
  check_constraint(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(
          (minimum_distance_lower + minimum_distance_upper) / 2 + 2 * radius_,
          0, 0),
      true, true);
  // The distance between the two spheres is above minimum_distance_upper.
  check_constraint(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(minimum_distance_upper + 2 * radius_ + 1E-3, 0, 0), true,
      false);
  // The distance between the two spheres is above minimum_distance_upper, but
  // the distance between sphere 2 and the right wall is between
  // minimum_distance_lower and minimum_distance_upper.
  check_constraint(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(
          wall_length_ / 2 -
              (minimum_distance_lower + minimum_distance_upper) / 2 - radius_,
          0, 0),
      true, true);
}
}  // namespace multibody
}  // namespace drake
