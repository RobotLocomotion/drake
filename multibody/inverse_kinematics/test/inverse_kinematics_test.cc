#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/wrap_to.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/create_constraint.h"
#include "drake/solvers/solve.h"

// TODO(eric.cousineau): Replace manual coordinate indexing with more semantic
// operations (`CalcRelativeTransform`, `SetFreeBodyPose`).

namespace drake {
namespace multibody {

Eigen::Quaterniond Vector4ToQuaternion(
    const Eigen::Ref<const Eigen::Vector4d>& q) {
  return Eigen::Quaterniond(q(0), q(1), q(2), q(3));
}

class TwoFreeBodiesTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TwoFreeBodiesTest);

  TwoFreeBodiesTest()
      : two_bodies_plant_(ConstructTwoFreeBodiesPlant<double>()),
        // TODO(hongkai.dai) call GetFrameByName()
        body1_frame_(two_bodies_plant_->GetFrameByName("body1")),
        body2_frame_(two_bodies_plant_->GetFrameByName("body2")),
        ik_(*two_bodies_plant_) {}

  ~TwoFreeBodiesTest() override {}

  std::unique_ptr<systems::Context<double>> RetrieveSolution(
      const solvers::MathematicalProgramResult& result) {
    const auto q_sol = result.GetSolution(ik_.q());
    body1_quaternion_sol_ = Vector4ToQuaternion(q_sol.head<4>());
    body1_position_sol_ = q_sol.segment<3>(4);
    body2_quaternion_sol_ = Vector4ToQuaternion(q_sol.segment<4>(7));
    body2_position_sol_ = q_sol.tail<3>();

    auto context = two_bodies_plant_->CreateDefaultContext();
    two_bodies_plant_->SetPositions(context.get(), q_sol);
    return context;
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> two_bodies_plant_;
  const Frame<double>& body1_frame_;
  const Frame<double>& body2_frame_;
  InverseKinematics ik_;

  Eigen::Quaterniond body1_quaternion_sol_;
  Eigen::Quaterniond body2_quaternion_sol_;
  Eigen::Vector3d body1_position_sol_;
  Eigen::Vector3d body2_position_sol_;
};

GTEST_TEST(InverseKinematicsTest, ConstructorWithJointLimits) {
  // Constructs an inverse kinematics problem for IIWA robot, make sure that
  // the joint limits are imposed when with_joint_limits=true, and the joint
  // limits are ignored when with_joint_limits=false.
  auto plant = ConstructIiwaPlant(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf",
      0.01);

  InverseKinematics ik_with_joint_limits(*plant);
  InverseKinematics ik_without_joint_limits(*plant, false);
  // Now check the joint limits.
  const VectorX<double> lower_limits = plant->GetPositionLowerLimits();
  const VectorX<double> upper_limits = plant->GetPositionUpperLimits();
  // Check if q_test will satisfy the joint limit constraint imposed from the
  // IK constructor.
  auto q_test_with_joint_limits =
      ik_with_joint_limits.get_mutable_prog()->AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7),
          ik_with_joint_limits.q());
  auto q_test_without_joint_limits =
      ik_without_joint_limits.get_mutable_prog()->AddBoundingBoxConstraint(
          Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7),
          ik_without_joint_limits.q());
  auto check_q_with_joint_limits =
      [&ik_with_joint_limits,
       &q_test_with_joint_limits](const Eigen::VectorXd& q_test) {
        q_test_with_joint_limits.evaluator()->UpdateLowerBound(q_test);
        q_test_with_joint_limits.evaluator()->UpdateUpperBound(q_test);
        return Solve(ik_with_joint_limits.prog()).is_success();
      };
  auto check_q_without_joint_limits =
      [&ik_without_joint_limits,
       &q_test_without_joint_limits](const Eigen::VectorXd& q_test) {
        q_test_without_joint_limits.evaluator()->UpdateLowerBound(q_test);
        q_test_without_joint_limits.evaluator()->UpdateUpperBound(q_test);
        return Solve(ik_without_joint_limits.prog()).is_success();
      };
  for (int i = 0; i < 7; ++i) {
    Eigen::VectorXd q_good = Eigen::VectorXd::Zero(7);
    q_good(i) = lower_limits(i) * 0.01 + upper_limits(i) * 0.99;
    EXPECT_TRUE(check_q_with_joint_limits(q_good));
    EXPECT_TRUE(check_q_without_joint_limits(q_good));
    q_good(i) = lower_limits(i) * 0.99 + upper_limits(i) * 0.01;
    EXPECT_TRUE(check_q_with_joint_limits(q_good));
    EXPECT_TRUE(check_q_without_joint_limits(q_good));
    Eigen::VectorXd q_bad = q_good;
    q_bad(i) = -0.01 * lower_limits(i) + 1.01 * upper_limits(i);
    EXPECT_FALSE(check_q_with_joint_limits(q_bad));
    EXPECT_TRUE(check_q_without_joint_limits(q_bad));
    q_bad(i) = 1.01 * lower_limits(i) - 0.01 * upper_limits(i);
    EXPECT_FALSE(check_q_with_joint_limits(q_bad));
    EXPECT_TRUE(check_q_without_joint_limits(q_bad));
  }
}

TEST_F(TwoFreeBodiesTest, ConstructorAddsUnitQuaterionConstraints) {
  // By default, the initial guess was set to be [1, 0, 0, 0].
  EXPECT_TRUE(CompareMatrices(ik_.prog().GetInitialGuess(ik_.q().head<4>()),
                              Eigen::Vector4d(1, 0, 0, 0)));

  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().head<4>(),
                                          Eigen::Vector4d(1, 2, 3, 4));
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().segment<4>(7),
                                          Eigen::Vector4d(.1, .2, .3, .4));
  const auto result = Solve(ik_.prog());
  EXPECT_TRUE(result.is_success());
  Eigen::VectorXd q = result.GetSolution(ik_.q());
  EXPECT_NEAR(q.head<4>().squaredNorm(), 1.0, 1e-6);
  EXPECT_NEAR(q.segment<4>(7).squaredNorm(), 1.0, 1e-6);

  // Test the constructor which takes a Context.
  auto context = two_bodies_plant_->CreateDefaultContext();
  InverseKinematics ik2(*two_bodies_plant_, context.get());
  ik2.get_mutable_prog()->SetInitialGuess(ik2.q().head<4>(),
                                          Eigen::Vector4d(1, 2, 3, 4));
  ik2.get_mutable_prog()->SetInitialGuess(ik2.q().segment<4>(7),
                                          Eigen::Vector4d(.1, .2, .3, .4));
  const auto result2 = Solve(ik2.prog());
  EXPECT_TRUE(result2.is_success());
  q = result2.GetSolution(ik2.q());
  EXPECT_NEAR(q.head<4>().squaredNorm(), 1.0, 1e-6);
  EXPECT_NEAR(q.segment<4>(7).squaredNorm(), 1.0, 1e-6);
}

GTEST_TEST(InverseKinematicsTest, ConstructorLockedJoints) {
  MultibodyPlant<double> plant(0);

  // Create a plant with four bodies.
  const auto& world = plant.world_body();
  const auto M = SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1);
  const auto& body1 = plant.AddRigidBody("body1", M);
  const auto& body2 = plant.AddRigidBody("body2", M);
  const auto& body3 = plant.AddRigidBody("body3", M);
  const auto& body4 = plant.AddRigidBody("body4", M);

  // Attach a specific joint to each body:
  // (1) A quaternion floating joint that will not be locked.
  // (2) A quaternion floating joint that we'll lock to its initial position.
  // (3) A revolute joint that will not be locked.
  // (4) A revolute joint that we'll lock to its initial position.
  math::RigidTransform<double> I;
  Eigen::Vector3d X = Eigen::Vector3d::UnitX();
  const auto& joint1 =
      plant.AddJoint<QuaternionFloatingJoint>("joint1", world, I, body1, I);
  const auto& joint2 =
      plant.AddJoint<QuaternionFloatingJoint>("joint2", world, I, body2, I);
  const auto& joint3 =
      plant.AddJoint<RevoluteJoint>("joint3", world, I, body3, I, X);
  const auto& joint4 =
      plant.AddJoint<RevoluteJoint>("joint4", world, I, body4, I, X);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  // Leave joint1 unlocked.

  // Lock body2's floating joint to an un-normalized initial value.
  joint2.SetQuaternion(&*context, Eigen::Quaternion<double>(0, 3.0, 0, 0));
  joint2.Lock(&*context);

  // Set limits on joint3, but do not lock it.
  dynamic_cast<RevoluteJoint<double>&>(plant.get_mutable_joint(joint3.index()))
      .set_position_limits(Vector1d{-0.5}, Vector1d{0.5});

  // Lock body4's revolute joint beyond its limit.
  dynamic_cast<RevoluteJoint<double>&>(plant.get_mutable_joint(joint4.index()))
      .set_position_limits(Vector1d{-1}, Vector1d{1});
  joint4.set_angle(&*context, 1.1);
  joint4.Lock(&*context);

  // Initialize IK.
  const InverseKinematics ik(plant, &*context);
  const solvers::MathematicalProgram& prog = ik.prog();

  // The initial guess is set for the two quaternion floating joints.
  EXPECT_TRUE(CompareMatrices(
      ik.prog().GetInitialGuess(ik.q().segment(joint1.position_start(), 4)),
      Eigen::Vector4d(1, 0, 0, 0)));
  const Eigen::Vector4d joint2_position(0, 1, 0, 0);
  EXPECT_TRUE(CompareMatrices(
      prog.GetInitialGuess(ik.q().segment(joint2.position_start(), 4)),
      joint2_position));

  // We only expect one bounding box constraint, which is the joint limits.
  ASSERT_EQ(prog.bounding_box_constraints().size(), 1);
  const solvers::Binding<solvers::BoundingBoxConstraint>& limits =
      prog.bounding_box_constraints().front();

  // joint 1 is unlocked, we expect a unit quaternion constraint and limits of
  // [-1, 1].
  ASSERT_EQ(prog.generic_constraints().size(), 1);
  const solvers::Binding<solvers::Constraint>& unit_quat =
      prog.generic_constraints().front();
  ASSERT_EQ(unit_quat.variables().size(), 4);
  const int j1_start = joint1.position_start();
  EXPECT_EQ(symbolic::Variables(unit_quat.variables()),
            symbolic::Variables(ik.q().segment(j1_start, 4)));
  EXPECT_TRUE(
      CompareMatrices(limits.evaluator()->lower_bound().segment<4>(j1_start),
                      Eigen::Vector4d(-1, -1, -1, -1)));
  EXPECT_TRUE(
      CompareMatrices(limits.evaluator()->upper_bound().segment<4>(j1_start),
                      Eigen::Vector4d(1, 1, 1, 1)));

  // joint2 is locked, so we expect a limits == joint2_position.
  const int j2_start = joint2.position_start();
  EXPECT_TRUE(CompareMatrices(
      limits.evaluator()->lower_bound().segment<4>(j2_start), joint2_position));
  EXPECT_TRUE(CompareMatrices(
      limits.evaluator()->upper_bound().segment<4>(j2_start), joint2_position));

  // joint3 is unlocked, so we expect the joint limits to be enforced.
  const int j3_start = joint3.position_start();
  EXPECT_EQ(limits.evaluator()->lower_bound()[j3_start], -0.5);
  EXPECT_EQ(limits.evaluator()->upper_bound()[j3_start], +0.5);

  // joint4 is locked. Locked revolute joints obey their initial position,
  // ignoring limits.
  const int j4_start = joint4.position_start();
  EXPECT_EQ(limits.evaluator()->lower_bound()[j4_start], 1.1);
  EXPECT_EQ(limits.evaluator()->upper_bound()[j4_start], 1.1);
}

TEST_F(TwoFreeBodiesTest, PositionConstraint) {
  const Eigen::Vector3d p_BQ(0.2, 0.3, 0.5);
  const Eigen::Vector3d p_AQ_lower(-0.1, -0.2, -0.3);
  const Eigen::Vector3d p_AQ_upper(-0.05, -0.12, -0.28);
  ik_.AddPositionConstraint(body1_frame_, p_BQ, body2_frame_, p_AQ_lower,
                            p_AQ_upper);

  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().head<4>(),
                                          Eigen::Vector4d(1, 0, 0, 0));
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().segment<4>(7),
                                          Eigen::Vector4d(1, 0, 0, 0));
  const auto result = Solve(ik_.prog());
  EXPECT_TRUE(result.is_success());
  RetrieveSolution(result);
  const Eigen::Vector3d p_AQ = body2_quaternion_sol_.inverse() *
                               (body1_quaternion_sol_ * p_BQ +
                                body1_position_sol_ - body2_position_sol_);
  const double tol = 1E-6;
  EXPECT_TRUE(
      (p_AQ.array() <= p_AQ_upper.array() + Eigen::Array3d::Constant(tol))
          .all());
  EXPECT_TRUE(
      (p_AQ.array() >= p_AQ_lower.array() - Eigen::Array3d::Constant(tol))
          .all());
}

TEST_F(TwoFreeBodiesTest, PositionCost) {
  const Eigen::Vector3d p_AP(-0.1, -0.2, -0.3);
  const Eigen::Vector3d p_BQ(0.2, 0.3, 0.5);
  auto binding = ik_.AddPositionCost(body1_frame_, p_AP, body2_frame_, p_BQ,
                                     Eigen::Matrix3d::Identity());

  // We don't need to test the cost implementation, only that the arguments are
  // passed correctly.  Just set an arbitrary (but valid) q and evaluate the
  // cost.
  math::RigidTransform<double> X_WA(
      math::RollPitchYaw<double>(0.32, -0.24, -0.51),
      Eigen::Vector3d(0.1, 0.3, 0.72));
  math::RigidTransform<double> X_WB(math::RollPitchYaw<double>(8.1, 0.42, -0.2),
                                    Eigen::Vector3d(-0.84, 0.2, 1.4));
  auto context = two_bodies_plant_->CreateDefaultContext();
  two_bodies_plant_->SetFreeBodyPose(
      context.get(), two_bodies_plant_->GetBodyByName("body1"), X_WA);
  two_bodies_plant_->SetFreeBodyPose(
      context.get(), two_bodies_plant_->GetBodyByName("body2"), X_WB);
  ik_.get_mutable_prog()->SetInitialGuess(
      ik_.q(), two_bodies_plant_->GetPositions(*context));

  const Eigen::Vector3d p_AQ = X_WA.inverse() * X_WB * p_BQ;
  const Eigen::Vector3d err = p_AQ - p_AP;
  const double expected_cost = err.dot(err);

  EXPECT_NEAR(ik_.prog().EvalBindingAtInitialGuess(binding)[0], expected_cost,
              1e-12);
}

TEST_F(TwoFreeBodiesTest, OrientationConstraint) {
  const double angle_bound = 0.05 * M_PI;

  const math::RotationMatrix<double> R_AbarA(Eigen::AngleAxisd(
      0.1 * M_PI, Eigen::Vector3d(0.2, 0.4, 1.2).normalized()));
  const math::RotationMatrix<double> R_BbarB(Eigen::AngleAxisd(
      0.2 * M_PI, Eigen::Vector3d(1.2, 2.1, -0.2).normalized()));

  ik_.AddOrientationConstraint(body1_frame_, R_AbarA, body2_frame_, R_BbarB,
                               angle_bound);

  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().head<4>(),
                                          Eigen::Vector4d(1, 0, 0, 0));
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().segment<4>(7),
                                          Eigen::Vector4d(1, 0, 0, 0));
  const auto result = Solve(ik_.prog());
  EXPECT_TRUE(result.is_success());
  const auto q_sol = result.GetSolution(ik_.q());
  RetrieveSolution(result);
  const math::RotationMatrix<double> R_AbarBbar(
      body1_quaternion_sol_.inverse() * body2_quaternion_sol_);
  const math::RotationMatrix<double> R_AB =
      R_AbarA.transpose() * R_AbarBbar * R_BbarB;
  const double angle = R_AB.ToAngleAxis().angle();
  EXPECT_LE(angle, angle_bound + 1E-6);
}

TEST_F(TwoFreeBodiesTest, OrientationCost) {
  const math::RotationMatrix<double> R_AbarA(
      math::RollPitchYaw<double>(1, 2, 3));
  const math::RotationMatrix<double> R_BbarB(
      math::RollPitchYaw<double>(4, 5, 6));
  const double c{2.4};
  auto binding =
      ik_.AddOrientationCost(body1_frame_, R_AbarA, body2_frame_, R_BbarB, c);

  // We don't need to test the cost implementation, only that the arguments are
  // passed correctly.  Just set an arbitrary (but valid) q and evaluate the
  // cost.
  math::RigidTransform<double> X_WAbar(
      math::RollPitchYaw<double>(0.32, -0.24, -0.51),
      Eigen::Vector3d(0.1, 0.3, 0.72));
  math::RigidTransform<double> X_WBbar(
      math::RollPitchYaw<double>(8.1, 0.42, -0.2),
      Eigen::Vector3d(-0.84, 0.2, 1.4));
  auto context = two_bodies_plant_->CreateDefaultContext();
  two_bodies_plant_->SetFreeBodyPose(
      context.get(), two_bodies_plant_->GetBodyByName("body1"), X_WAbar);
  two_bodies_plant_->SetFreeBodyPose(
      context.get(), two_bodies_plant_->GetBodyByName("body2"), X_WBbar);
  ik_.get_mutable_prog()->SetInitialGuess(
      ik_.q(), two_bodies_plant_->GetPositions(*context));

  const math::RotationMatrix<double> R_AB =
      (X_WAbar.rotation() * R_AbarA).inverse() * X_WBbar.rotation() * R_BbarB;
  const double theta = R_AB.ToAngleAxis().angle();
  EXPECT_NEAR(ik_.prog().EvalBindingAtInitialGuess(binding)[0],
              c * (1.0 - cos(theta)), 1e-12);
}

TEST_F(TwoFreeBodiesTest, GazeTargetConstraint) {
  const Eigen::Vector3d p_AS(0.01, 0.2, 0.4);
  const Eigen::Vector3d n_A(0.2, 0.4, -0.1);
  const Eigen::Vector3d p_BT(0.4, -0.2, 1.5);
  const double cone_half_angle{0.2 * M_PI};

  ik_.AddGazeTargetConstraint(body1_frame_, p_AS, n_A, body2_frame_, p_BT,
                              cone_half_angle);

  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().head<4>(),
                                          Eigen::Vector4d(1, 0, 0, 0));
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().segment<4>(7),
                                          Eigen::Vector4d(1, 0, 0, 0));
  const auto result = Solve(ik_.prog());
  EXPECT_TRUE(result.is_success());

  RetrieveSolution(result);
  const Eigen::Vector3d p_WS =
      body1_quaternion_sol_ * p_AS + body1_position_sol_;
  const Eigen::Vector3d p_WT =
      body2_quaternion_sol_ * p_BT + body2_position_sol_;
  const Eigen::Vector3d p_ST_A =
      body1_quaternion_sol_.inverse() * (p_WT - p_WS);
  EXPECT_GE(p_ST_A.dot(n_A),
            std::cos(cone_half_angle) * p_ST_A.norm() * n_A.norm() - 1E-3);
}

TEST_F(TwoFreeBodiesTest, AngleBetweenVectorsConstraint) {
  const Eigen::Vector3d n_A(0.2, -0.4, 0.9);
  const Eigen::Vector3d n_B(1.4, -0.1, 1.8);

  const double angle_lower{0.2 * M_PI};
  const double angle_upper{0.2 * M_PI};

  ik_.AddAngleBetweenVectorsConstraint(body1_frame_, n_A, body2_frame_, n_B,
                                       angle_lower, angle_upper);
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().head<4>(),
                                          Eigen::Vector4d(1, 0, 0, 0));
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().segment<4>(7),
                                          Eigen::Vector4d(1, 0, 0, 0));
  const auto result = Solve(ik_.prog());
  EXPECT_TRUE(result.is_success());

  RetrieveSolution(result);

  const Eigen::Vector3d n_A_W = body1_quaternion_sol_ * n_A;
  const Eigen::Vector3d n_B_W = body2_quaternion_sol_ * n_B;

  const double angle =
      std::acos(n_A_W.dot(n_B_W) / (n_A_W.norm() * n_B_W.norm()));
  EXPECT_NEAR(angle, angle_lower, 1E-6);
}

TEST_F(TwoFreeBodiesTest, AngleBetweenVectorsCost) {
  const Eigen::Vector3d na_A(-0.1, -0.2, -0.3);
  const Eigen::Vector3d nb_B(0.2, 0.3, 0.5);
  const double c = 10;
  auto binding =
      ik_.AddAngleBetweenVectorsCost(body1_frame_, na_A, body2_frame_, nb_B, c);

  // We don't need to test the cost implementation, only that the arguments are
  // passed correctly.  Just set an arbitrary (but valid) q and evaluate the
  // cost.
  math::RigidTransform<double> X_WA(
      math::RollPitchYaw<double>(0.32, -0.24, -0.51),
      Eigen::Vector3d(0.1, 0.3, 0.72));
  math::RigidTransform<double> X_WB(math::RollPitchYaw<double>(8.1, 0.42, -0.2),
                                    Eigen::Vector3d(-0.84, 0.2, 1.4));
  auto context = two_bodies_plant_->CreateDefaultContext();
  two_bodies_plant_->SetFreeBodyPose(
      context.get(), two_bodies_plant_->GetBodyByName("body1"), X_WA);
  two_bodies_plant_->SetFreeBodyPose(
      context.get(), two_bodies_plant_->GetBodyByName("body2"), X_WB);
  ik_.get_mutable_prog()->SetInitialGuess(
      ik_.q(), two_bodies_plant_->GetPositions(*context));

  const Eigen::Vector3d na_W = X_WA.rotation() * na_A;
  const Eigen::Vector3d nb_W = X_WB.rotation() * nb_B;
  const double cos_theta = na_W.dot(nb_W) / (na_W.norm() * nb_W.norm());
  const double expected_cost = c * (1 - cos_theta);

  EXPECT_NEAR(ik_.prog().EvalBindingAtInitialGuess(binding)[0], expected_cost,
              1e-12);
}

TEST_F(TwoFreeBodiesTest, PointToPointDistanceConstraint) {
  const Eigen::Vector3d p_B1P1(0.2, -0.4, 0.9);
  const Eigen::Vector3d p_B2P2(1.4, -0.1, 1.8);

  const double distance_lower{0.2};
  const double distance_upper{0.25};

  ik_.AddPointToPointDistanceConstraint(body1_frame_, p_B1P1, body2_frame_,
                                        p_B2P2, distance_lower, distance_upper);
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().head<4>(),
                                          Eigen::Vector4d(1, 0, 0, 0));
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().segment<4>(7),
                                          Eigen::Vector4d(1, 0, 0, 0));
  const auto result = Solve(ik_.prog());
  EXPECT_TRUE(result.is_success());

  RetrieveSolution(result);

  const Eigen::Vector3d p_WP1 =
      body1_position_sol_ + body1_quaternion_sol_ * p_B1P1;
  const Eigen::Vector3d p_WP2 =
      body2_position_sol_ + body2_quaternion_sol_ * p_B2P2;
  const double distance_sol = (p_WP1 - p_WP2).norm();
  EXPECT_GE(distance_sol, distance_lower - 1e-6);
  EXPECT_LE(distance_sol, distance_upper + 1e-6);
}

TEST_F(TwoFreeBodiesTest, PointToLineDistanceConstraint) {
  const Eigen::Vector3d p_B1P(0.2, -0.4, 0.9);
  const Eigen::Vector3d p_B2Q(1.4, -0.1, 1.8);
  const Eigen::Vector3d n_B2(0.4, -0.5, 1.2);

  const double distance_lower{0.2};
  const double distance_upper{0.25};

  ik_.AddPointToLineDistanceConstraint(body1_frame_, p_B1P, body2_frame_, p_B2Q,
                                       n_B2, distance_lower, distance_upper);
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().head<4>(),
                                          Eigen::Vector4d(1, 0, 0, 0));
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().segment<4>(7),
                                          Eigen::Vector4d(1, 0, 0, 0));
  const auto result = Solve(ik_.prog());
  EXPECT_TRUE(result.is_success());

  RetrieveSolution(result);

  const Eigen::Vector3d p_WP =
      body1_position_sol_ + body1_quaternion_sol_ * p_B1P;
  const Eigen::Vector3d p_WQ =
      body2_position_sol_ + body2_quaternion_sol_ * p_B2Q;
  const Eigen::Vector3d n_W = body2_quaternion_sol_ * n_B2;
  const Eigen::Vector3d n_W_normalized = n_W.normalized();

  const double distance_sol =
      (p_WQ + (p_WP - p_WQ).dot(n_W_normalized) * n_W_normalized - p_WP).norm();
  EXPECT_GE(distance_sol, distance_lower - 2e-6);
  EXPECT_LE(distance_sol, distance_upper + 2e-6);
}

TEST_F(TwoFreeBodiesTest, PolyhedronConstraint) {
  const Frame<double>& frameF = body1_frame_;
  const Frame<double>& frameG = body2_frame_;
  Eigen::Matrix<double, 3, 2> p_GP;
  p_GP.col(0) << 0.1, 0.2, 0.3;
  p_GP.col(1) << 0.4, 0.5, 0.6;
  Eigen::Matrix<double, 1, 6> A;
  A << 1, 2, 3, 4, 5, 6;
  Vector1d b(10);

  ik_.AddPolyhedronConstraint(frameF, frameG, p_GP, A, b);
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().head<4>(),
                                          Eigen::Vector4d(1, 0, 0, 0));
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q().segment<4>(7),
                                          Eigen::Vector4d(1, 0, 0, 0));
  const auto result = Solve(ik_.prog());
  EXPECT_TRUE(result.is_success());

  auto context = RetrieveSolution(result);

  Eigen::Matrix3Xd p_FP(3, p_GP.cols());
  two_bodies_plant_->CalcPointsPositions(*context, frameG, p_GP, frameF, &p_FP);
  Eigen::Map<Eigen::VectorXd> p_FP_stack(p_FP.data(), 3 * p_FP.cols());
  const Eigen::VectorXd y_expected = A * p_FP_stack;
  EXPECT_EQ(y_expected.rows(), b.rows());
  for (int i = 0; i < b.rows(); ++i) {
    EXPECT_LE(y_expected(i), b(i) + 1E-5);
  }
}

TEST_F(TwoFreeSpheresTest, MinimumDistanceLowerBoundConstraintTest) {
  const double min_distance_lower = 0.1;

  InverseKinematics ik(*plant_double_, plant_context_double_);

  auto constraint =
      ik.AddMinimumDistanceLowerBoundConstraint(min_distance_lower);

  // The two spheres are colliding in the initial guess.
  ik.get_mutable_prog()->SetInitialGuess(ik.q().head<4>(),
                                         Eigen::Vector4d(1, 0, 0, 0));
  ik.get_mutable_prog()->SetInitialGuess(ik.q().segment<3>(4),
                                         Eigen::Vector3d(0, 0, 0.01));
  ik.get_mutable_prog()->SetInitialGuess(ik.q().segment<4>(7),
                                         Eigen::Vector4d(1, 0, 0, 0));
  ik.get_mutable_prog()->SetInitialGuess(ik.q().tail<3>(),
                                         Eigen::Vector3d(0, 0, -0.01));

  auto solve_and_check = [&]() {
    const solvers::MathematicalProgramResult result = Solve(ik.prog());
    EXPECT_TRUE(result.is_success());

    const Eigen::Vector3d p_WB1 = result.GetSolution(ik.q().segment<3>(4));
    const Eigen::Quaterniond quat_WB1(
        result.GetSolution(ik.q()(0)), result.GetSolution(ik.q()(1)),
        result.GetSolution(ik.q()(2)), result.GetSolution(ik.q()(3)));
    const Eigen::Vector3d p_WB2 = result.GetSolution(ik.q().tail<3>());
    const Eigen::Quaterniond quat_WB2(
        result.GetSolution(ik.q()(7)), result.GetSolution(ik.q()(8)),
        result.GetSolution(ik.q()(9)), result.GetSolution(ik.q()(10)));
    const Eigen::Vector3d p_WS1 =
        p_WB1 + quat_WB1.toRotationMatrix() * X_B1S1_.translation();
    const Eigen::Vector3d p_WS2 =
        p_WB2 + quat_WB2.toRotationMatrix() * X_B2S2_.translation();
    // This large error is due to the derivative of the penalty function(i.e.,
    // the gradient ∂penalty/∂distance) being small near minimum_distance. Hence
    // a small violation on the penalty leads to a large violation on the
    // minimum_distance.
    const double tol = 2e-4;
    EXPECT_GE((p_WS1 - p_WS2).norm() - radius1_ - radius2_,
              min_distance_lower - tol);
  };

  solve_and_check();

  // Now set the two spheres to coincide at the initial guess, and solve again.
  ik.get_mutable_prog()->SetInitialGuess(
      ik.q().tail<3>(), ik.prog().initial_guess().segment<3>(4));
  solve_and_check();
}

TEST_F(TwoFreeSpheresTest, MinimumDistanceUpperBoundConstraintTest) {
  const double min_distance_upper = 0.5;
  const double influence_distance_offset = 1;

  InverseKinematics ik(*plant_double_, plant_context_double_);

  auto constraint = ik.AddMinimumDistanceUpperBoundConstraint(
      min_distance_upper, influence_distance_offset);

  // The two spheres are colliding in the initial guess.
  ik.get_mutable_prog()->SetInitialGuess(ik.q().head<4>(),
                                         Eigen::Vector4d(1, 0, 0, 0));
  ik.get_mutable_prog()->SetInitialGuess(ik.q().segment<3>(4),
                                         Eigen::Vector3d(0, 0, 0.01));
  ik.get_mutable_prog()->SetInitialGuess(ik.q().segment<4>(7),
                                         Eigen::Vector4d(1, 0, 0, 0));
  ik.get_mutable_prog()->SetInitialGuess(ik.q().tail<3>(),
                                         Eigen::Vector3d(0, 0, -0.01));

  auto solve_and_check = [&]() {
    const solvers::MathematicalProgramResult result = Solve(ik.prog());
    EXPECT_TRUE(result.is_success());

    const Eigen::Vector3d p_WB1 = result.GetSolution(ik.q().segment<3>(4));
    const Eigen::Quaterniond quat_WB1(
        result.GetSolution(ik.q()(0)), result.GetSolution(ik.q()(1)),
        result.GetSolution(ik.q()(2)), result.GetSolution(ik.q()(3)));
    const Eigen::Vector3d p_WB2 = result.GetSolution(ik.q().tail<3>());
    const Eigen::Quaterniond quat_WB2(
        result.GetSolution(ik.q()(7)), result.GetSolution(ik.q()(8)),
        result.GetSolution(ik.q()(9)), result.GetSolution(ik.q()(10)));
    const Eigen::Vector3d p_WS1 =
        p_WB1 + quat_WB1.toRotationMatrix() * X_B1S1_.translation();
    const Eigen::Vector3d p_WS2 =
        p_WB2 + quat_WB2.toRotationMatrix() * X_B2S2_.translation();
    // This large error is due to the derivative of the penalty function(i.e.,
    // the gradient ∂penalty/∂distance) being small near minimum_distance. Hence
    // a small violation on the penalty leads to a large violation on the
    // minimum_distance.
    const double tol = 2e-4;
    EXPECT_LE((p_WS1 - p_WS2).norm() - radius1_ - radius2_,
              min_distance_upper + tol);
  };

  solve_and_check();

  // Now set the two spheres to coincide at the initial guess, and solve again.
  ik.get_mutable_prog()->SetInitialGuess(
      ik.q().tail<3>(), ik.prog().initial_guess().segment<3>(4));
  solve_and_check();

  // Now set the two spheres separated at the initial guess, and solve again.
  // TODO(hongkai.dai): SNOPT can solve the problem but IPOPT doesn't. I need to
  // investigate into it.
}
}  // namespace multibody
}  // namespace drake
