#include "drake/multibody/optimization/toppra.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace multibody {
namespace {

class IiwaToppraTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaToppraTest)

  IiwaToppraTest() : iiwa_plant_(std::make_unique<MultibodyPlant<double>>(0)) {
    std::string file_path = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/iiwa7/"
        "iiwa7_no_collision.sdf");
    const auto iiwa_instance = multibody::Parser(iiwa_plant_.get())
                                   .AddModelFromFile(file_path, "iiwa");
    iiwa_plant_->WeldFrames(
        iiwa_plant_->world_frame(),
        iiwa_plant_->GetFrameByName("iiwa_link_0", iiwa_instance));
    iiwa_plant_->Finalize();
    plant_context_ = iiwa_plant_->CreateDefaultContext();

    Eigen::MatrixXd knots(7, 2);
    knots.col(0) << -1.57, 0.1, 0, -1.2, 0, 1.6, 0;
    knots.col(1) << -0.8, -0.6, 0.5, 0.1, -0.5, -0.1, -1;
    path_ = PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
        Eigen::VectorXd::LinSpaced(2, 0, 1), knots, Eigen::VectorXd::Zero(7),
        Eigen::VectorXd::Zero(7));

    auto options = CalcGridPointsOptions();
    options.max_err = 1e-3;
    auto gridpts = Toppra::CalcGridPoints(path_, options);
    toppra_ = std::make_unique<Toppra>(path_, *iiwa_plant_, gridpts);
  }

  ~IiwaToppraTest() override {}

 protected:
  std::unique_ptr<MultibodyPlant<double>> iiwa_plant_;
  std::unique_ptr<systems::Context<double>> plant_context_;
  PiecewisePolynomial<double> path_;
  std::unique_ptr<Toppra> toppra_;
};

TEST_F(IiwaToppraTest, JointVelocityLimit) {
  Eigen::VectorXd lower_bound(7);
  lower_bound << -1.1, -1.2, -1.3, -1.4, -1.5, -1.6, -1.7;
  Eigen::VectorXd upper_bound(7);
  upper_bound << 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7;

  auto velocity_constraint =
      toppra_->AddJointVelocityLimit(lower_bound, upper_bound);
  toppra_->AddJointAccelerationLimit(Eigen::VectorXd::Constant(7, -10),
                                     Eigen::VectorXd::Constant(7, 10));

  auto result = toppra_->SolvePathParameterization();
  ASSERT_TRUE(result);
  auto s_path = result.value();

  // This tolerance was tuned to work with the given gridpoints.
  const double tol = 1e-14;
  for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
    const auto s = s_path.scalarValue(s_path.start_time(ii));
    const auto s_dot = s_path.EvalDerivative(s_path.start_time(ii), 1);
    const auto dq_ds = path_.EvalDerivative(s, 1);
    const auto velocity = dq_ds * s_dot;

    EXPECT_TRUE((velocity.array() >= lower_bound.array() - tol).all());
    EXPECT_TRUE((velocity.array() <= upper_bound.array() + tol).all());
  }

  const auto s = s_path.scalarValue(s_path.end_time());
  const auto s_dot = s_path.EvalDerivative(s_path.end_time(), 1);
  const auto dq_ds = path_.EvalDerivative(s, 1);
  const auto velocity = dq_ds * s_dot;

  EXPECT_TRUE((velocity.array() >= lower_bound.array() - tol).all());
  EXPECT_TRUE((velocity.array() <= upper_bound.array() + tol).all());
}

TEST_F(IiwaToppraTest, JointAccelerationLimit) {
  Eigen::VectorXd lower_bound(7);
  lower_bound << -1.1, -1.2, -1.3, -1.4, -1.5, -1.6, -1.7;
  Eigen::VectorXd upper_bound(7);
  upper_bound << 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7;

  auto acceleration_constraint =
      toppra_->AddJointAccelerationLimit(lower_bound, upper_bound);

  auto result = toppra_->SolvePathParameterization();
  ASSERT_TRUE(result);
  auto s_path = result.value();

  // This tolerance was tuned to work with the given gridpoints.
  const double tol = 1e-14;
  for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
    const auto s = s_path.scalarValue(s_path.start_time(ii));
    const auto s_dot = s_path.EvalDerivative(s_path.start_time(ii), 1);
    const auto s_ddot = s_path.EvalDerivative(s_path.start_time(ii), 2);
    const auto dq_ds = path_.EvalDerivative(s, 1);
    const auto ddq_dds = path_.EvalDerivative(s, 2);
    const auto acceleration = dq_ds * s_ddot + ddq_dds * s_dot * s_dot;

    EXPECT_TRUE((acceleration.array() >= lower_bound.array() - tol).all());
    EXPECT_TRUE((acceleration.array() <= upper_bound.array() + tol).all());
  }

  const auto s = s_path.scalarValue(s_path.end_time());
  const auto s_dot = s_path.EvalDerivative(s_path.end_time(), 1);
  const auto s_ddot = s_path.EvalDerivative(s_path.end_time(), 2);
  const auto dq_ds = path_.EvalDerivative(s, 1);
  const auto ddq_dds = path_.EvalDerivative(s, 2);
  const auto acceleration = dq_ds * s_ddot + ddq_dds * s_dot * s_dot;

  EXPECT_TRUE((acceleration.array() >= lower_bound.array() - tol).all());
  EXPECT_TRUE((acceleration.array() <= upper_bound.array() + tol).all());
}

TEST_F(IiwaToppraTest, JointTorqueLimit) {
  Eigen::VectorXd lower_bound = Eigen::VectorXd::Constant(7, -30);
  Eigen::VectorXd upper_bound = Eigen::VectorXd::Constant(7, 31);

  auto torque_constraint =
      toppra_->AddJointTorqueLimit(lower_bound, upper_bound);

  auto result = toppra_->SolvePathParameterization();
  ASSERT_TRUE(result);
  auto s_path = result.value();

  // This tolerance was tuned to work with the given gridpoints.
  const double tol = 1e-14;
  Eigen::MatrixXd M(7, 7);
  Eigen::VectorXd Cv(7);
  Eigen::VectorXd G(7);
  Eigen::VectorXd tau(7);
  for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
    const auto s = s_path.scalarValue(s_path.start_time(ii));
    const auto s_dot = s_path.EvalDerivative(s_path.start_time(ii), 1);
    const auto s_ddot = s_path.EvalDerivative(s_path.start_time(ii), 2);
    const auto dq_ds = path_.EvalDerivative(s, 1);
    const auto ddq_dds = path_.EvalDerivative(s, 2);
    const auto position = path_.value(s);
    const auto velocity = dq_ds * s_dot;
    const auto acceleration = dq_ds * s_ddot + ddq_dds * s_dot * s_dot;

    iiwa_plant_->SetPositions(plant_context_.get(), position);
    iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
    iiwa_plant_->CalcMassMatrix(*plant_context_, &M);
    iiwa_plant_->CalcBiasTerm(*plant_context_, &Cv);
    G = iiwa_plant_->CalcGravityGeneralizedForces(*plant_context_);
    tau = M * acceleration + Cv - G;

    EXPECT_TRUE((tau.array() >= lower_bound.array() - tol).all());
    EXPECT_TRUE((tau.array() <= upper_bound.array() + tol).all());
  }

  const auto s = s_path.scalarValue(s_path.end_time());
  const auto s_dot = s_path.EvalDerivative(s_path.end_time(), 1);
  const auto s_ddot = s_path.EvalDerivative(s_path.end_time(), 2);
  const auto dq_ds = path_.EvalDerivative(s, 1);
  const auto ddq_dds = path_.EvalDerivative(s, 2);
  const auto position = path_.value(s);
  const auto velocity = dq_ds * s_dot;
  const auto acceleration = dq_ds * s_ddot + ddq_dds * s_dot * s_dot;

  iiwa_plant_->SetPositions(plant_context_.get(), position);
  iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
  iiwa_plant_->CalcMassMatrix(*plant_context_, &M);
  iiwa_plant_->CalcBiasTerm(*plant_context_, &Cv);
  G = iiwa_plant_->CalcGravityGeneralizedForces(*plant_context_);
  tau = M * acceleration + Cv - G;

  EXPECT_TRUE((tau.array() >= lower_bound.array() - tol).all());
  EXPECT_TRUE((tau.array() <= upper_bound.array() + tol).all());
}

TEST_F(IiwaToppraTest, FrameVelocityLimit) {
  Eigen::VectorXd lower_bound(6);
  lower_bound << -1.1, -1.2, -1.3, -1.4, -1.5, -1.6;
  Eigen::VectorXd upper_bound(6);
  upper_bound << 2.1, 2.2, 2.3, 2.4, 2.5, 2.6;
  const auto& frame = iiwa_plant_->GetFrameByName("iiwa_link_7");

  toppra_->AddFrameVelocityLimit(frame, lower_bound, upper_bound);
  toppra_->AddJointAccelerationLimit(Eigen::VectorXd::Constant(7, -10),
                                     Eigen::VectorXd::Constant(7, 10));

  auto result = toppra_->SolvePathParameterization();
  ASSERT_TRUE(result);
  auto s_path = result.value();

  // This tolerance was tuned to work with the given gridpoints.
  const double tol = 1e-14;
  Eigen::VectorXd frame_velocity(6);
  for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
    const auto s = s_path.scalarValue(s_path.start_time(ii));
    const auto s_dot = s_path.EvalDerivative(s_path.start_time(ii), 1);
    const auto dq_ds = path_.EvalDerivative(s, 1);
    const auto position = path_.value(s);
    const auto velocity = dq_ds * s_dot;

    iiwa_plant_->SetPositions(plant_context_.get(), position);
    iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
    frame_velocity =
        frame.CalcSpatialVelocityInWorld(*plant_context_).get_coeffs();

    EXPECT_TRUE((frame_velocity.array() >= lower_bound.array() - tol).all());
    EXPECT_TRUE((frame_velocity.array() <= upper_bound.array() + tol).all());
  }

  const auto s = s_path.scalarValue(s_path.end_time());
  const auto s_dot = s_path.EvalDerivative(s_path.end_time(), 1);
  const auto dq_ds = path_.EvalDerivative(s, 1);
  const auto position = path_.value(s);
  const auto velocity = dq_ds * s_dot;

  iiwa_plant_->SetPositions(plant_context_.get(), position);
  iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
  frame_velocity =
      frame.CalcSpatialVelocityInWorld(*plant_context_).get_coeffs();

  EXPECT_TRUE((frame_velocity.array() >= lower_bound.array() - tol).all());
  EXPECT_TRUE((frame_velocity.array() <= upper_bound.array() + tol).all());
}

TEST_F(IiwaToppraTest, FrameTranslationalSpeedLimit) {
  const double upper_bound = 1.1;
  const auto& frame = iiwa_plant_->GetFrameByName("iiwa_link_7");

  toppra_->AddFrameTranslationalSpeedLimit(frame, upper_bound);
  toppra_->AddJointAccelerationLimit(Eigen::VectorXd::Constant(7, -10),
                                     Eigen::VectorXd::Constant(7, 10));

  auto result = toppra_->SolvePathParameterization();
  ASSERT_TRUE(result);
  auto s_path = result.value();

  // This tolerance was tuned to work with the given gridpoints.
  const double tol = 1e-14;
  for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
    const auto s = s_path.scalarValue(s_path.start_time(ii));
    const auto s_dot = s_path.EvalDerivative(s_path.start_time(ii), 1);
    const auto dq_ds = path_.EvalDerivative(s, 1);
    const auto position = path_.value(s);
    const auto velocity = dq_ds * s_dot;

    iiwa_plant_->SetPositions(plant_context_.get(), position);
    iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
    const auto frame_velocity =
        frame.CalcSpatialVelocityInWorld(*plant_context_);
    const double speed = frame_velocity.translational().norm();

    EXPECT_LT(speed, upper_bound + tol);
  }

  const auto s = s_path.scalarValue(s_path.end_time());
  const auto s_dot = s_path.EvalDerivative(s_path.end_time(), 1);
  const auto dq_ds = path_.EvalDerivative(s, 1);
  const auto position = path_.value(s);
  const auto velocity = dq_ds * s_dot;

  iiwa_plant_->SetPositions(plant_context_.get(), position);
  iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
  const auto frame_velocity = frame.CalcSpatialVelocityInWorld(*plant_context_);
  const double speed = frame_velocity.translational().norm();

  EXPECT_LT(speed, upper_bound + tol);
}

TEST_F(IiwaToppraTest, FrameAccelerationLimit) {
  Eigen::VectorXd lower_bound(6);
  lower_bound << -1.1, -1.2, -1.3, -1.4, -1.5, -1.6;
  Eigen::VectorXd upper_bound(6);
  upper_bound << 2.1, 2.2, 2.3, 2.4, 2.5, 2.6;
  const auto& frame = iiwa_plant_->GetFrameByName("iiwa_link_7");

  toppra_->AddFrameAccelerationLimit(frame, lower_bound, upper_bound);

  auto result = toppra_->SolvePathParameterization();
  ASSERT_TRUE(result);
  auto s_path = result.value();

  // This tolerance was tuned to work with the given gridpoints.
  const double tol = 1e-14;
  Eigen::MatrixXd J(6, 7);
  Eigen::VectorXd J_dot_v(6);
  Eigen::VectorXd frame_acceleration(6);
  for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
    const auto s = s_path.scalarValue(s_path.start_time(ii));
    const auto s_dot = s_path.EvalDerivative(s_path.start_time(ii), 1);
    const auto s_ddot = s_path.EvalDerivative(s_path.start_time(ii), 2);
    const auto dq_ds = path_.EvalDerivative(s, 1);
    const auto ddq_dds = path_.EvalDerivative(s, 2);
    const auto position = path_.value(s);
    const auto velocity = dq_ds * s_dot;
    const auto acceleration = dq_ds * s_ddot + ddq_dds * s_dot * s_dot;

    iiwa_plant_->SetPositions(plant_context_.get(), position);
    iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
    iiwa_plant_->CalcJacobianSpatialVelocity(
        *plant_context_, JacobianWrtVariable::kQDot, frame,
        Eigen::Vector3d::Zero(), iiwa_plant_->world_frame(),
        iiwa_plant_->world_frame(), &J);
    J_dot_v = iiwa_plant_
                  ->CalcBiasSpatialAcceleration(
                      *plant_context_, JacobianWrtVariable::kV, frame,
                      Eigen::Vector3d::Zero(), iiwa_plant_->world_frame(),
                      iiwa_plant_->world_frame())
                  .get_coeffs();
    frame_acceleration = J * acceleration + J_dot_v;

    EXPECT_TRUE(
        (frame_acceleration.array() >= lower_bound.array() - tol).all());
    EXPECT_TRUE(
        (frame_acceleration.array() <= upper_bound.array() + tol).all());
  }

  const auto s = s_path.scalarValue(s_path.end_time());
  const auto s_dot = s_path.EvalDerivative(s_path.end_time(), 1);
  const auto s_ddot = s_path.EvalDerivative(s_path.end_time(), 2);
  const auto dq_ds = path_.EvalDerivative(s, 1);
  const auto ddq_dds = path_.EvalDerivative(s, 2);
  const auto position = path_.value(s);
  const auto velocity = dq_ds * s_dot;
  const auto acceleration = dq_ds * s_ddot + ddq_dds * s_dot * s_dot;

  iiwa_plant_->SetPositions(plant_context_.get(), position);
  iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
  iiwa_plant_->CalcJacobianSpatialVelocity(
      *plant_context_, JacobianWrtVariable::kQDot, frame,
      Eigen::Vector3d::Zero(), iiwa_plant_->world_frame(),
      iiwa_plant_->world_frame(), &J);
  J_dot_v = iiwa_plant_
                ->CalcBiasSpatialAcceleration(
                    *plant_context_, JacobianWrtVariable::kV, frame,
                    Eigen::Vector3d::Zero(), iiwa_plant_->world_frame(),
                    iiwa_plant_->world_frame())
                .get_coeffs();
  frame_acceleration = J * acceleration + J_dot_v;

  EXPECT_TRUE((frame_acceleration.array() >= lower_bound.array() - tol).all());
  EXPECT_TRUE((frame_acceleration.array() <= upper_bound.array() + tol).all());
}

GTEST_TEST(ToppraTest, GridpointsTest) {
  Eigen::MatrixXd knots(3, 2);
  knots.col(0) << 0, 10, 20;
  knots.col(1) << 1, 11, 21;
  auto path = PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
      Eigen::VectorXd::LinSpaced(2, 0, 1), knots, Eigen::VectorXd::Zero(3),
      Eigen::VectorXd::Zero(3));

  // Check minimum points
  auto options = CalcGridPointsOptions();
  options.max_iter = 0;
  options.min_points = 10;
  auto gridpts = Toppra::CalcGridPoints(path, options);
  EXPECT_GT(gridpts.size(), options.min_points);

  // Check minimum segment length
  options = CalcGridPointsOptions();
  options.max_err = 100;
  options.max_seg_length = 0.05;
  options.min_points = 1;
  gridpts = Toppra::CalcGridPoints(path, options);
  EXPECT_GT(gridpts.size(), 20);

  // Check iteration limit
  options = CalcGridPointsOptions();
  options.max_iter = 3;
  options.min_points = 1;
  gridpts = Toppra::CalcGridPoints(path, options);
  EXPECT_EQ(gridpts.size(), 9);
}

GTEST_TEST(ToppraTest, TimeOptimalTest) {
  auto plant = std::make_unique<MultibodyPlant<double>>(0);
  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);
  auto& body = plant->AddRigidBody("body", M_AAo_A);
  plant->AddJoint<PrismaticJoint>("joint", plant->world_body(), std::nullopt,
                                  body, std::nullopt, Eigen::Vector3d::UnitX());
  plant->Finalize();

  auto path = PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
      Eigen::Vector2d(0, 1), Eigen::RowVector2d(0, 1), Vector1d(0),
      Vector1d(0));

  auto options = CalcGridPointsOptions();
  options.max_err = 1e-5;
  auto gridpts = Toppra::CalcGridPoints(path, options);
  auto toppra = std::make_unique<Toppra>(path, *plant, gridpts);

  auto acceleration_constraint =
      toppra->AddJointAccelerationLimit(Vector1d(-1), Vector1d(1));

  auto result = toppra->SolvePathParameterization();
  ASSERT_TRUE(result);
  auto trajectory = result.value();

  // The time optimal trajectory for a double integrator travelling 1 m with
  // 1 m/s^2 acceleration limits will be a bang-bang trajectory that lasts 2
  // seconds. This tolerance was tuned to work with the given gridpoints.
  const double tol = 0.01;
  EXPECT_LT(trajectory.end_time(), 2 + tol);
}

GTEST_TEST(ToppraTest, ZeroVelocityTest) {
  auto plant = std::make_unique<MultibodyPlant<double>>(0);
  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);
  auto& body = plant->AddRigidBody("body", M_AAo_A);
  plant->AddJoint<PrismaticJoint>("joint", plant->world_body(), std::nullopt,
                                  body, std::nullopt, Eigen::Vector3d::UnitX());
  plant->Finalize();

  auto path = PiecewisePolynomial<double>::FirstOrderHold(
      Eigen::Vector3d(0, 0.9, 2), Eigen::RowVector3d(0, 0, 1));

  auto options = CalcGridPointsOptions();
  options.max_err = 1e-5;
  auto gridpts = Toppra::CalcGridPoints(path, options);
  auto toppra = std::make_unique<Toppra>(path, *plant, gridpts);

  auto velocity_constraint =
      toppra->AddJointVelocityLimit(Vector1d(-1), Vector1d(1));
  auto acceleration_constraint =
      toppra->AddJointAccelerationLimit(Vector1d(-1), Vector1d(1));

  auto result = toppra->SolvePathParameterization();
  ASSERT_TRUE(result);
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main(int argc, char** argv) {
  // Ensure that we have the Gurobi license for the entire duration of this
  // test, so that we do not have to release and re-acquire the license for
  // every test.
  auto gurobi_license = drake::solvers::GurobiSolver::AcquireLicense();
  // Ensure that we have the MOSEK license for the entire duration of this test,
  // so that we do not have to release and re-acquire the license for every
  // test.
  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
