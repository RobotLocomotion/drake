#include "drake/multibody/optimization/toppra.h"

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace multibody {
namespace {

class IiwaToppraTest : public ::testing::Test {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaToppraTest);

  IiwaToppraTest() : iiwa_plant_(std::make_unique<MultibodyPlant<double>>(0)) {
    multibody::Parser(iiwa_plant_.get())
        .AddModelsFromUrl(
            "package://drake_models/iiwa_description/sdf/"
            "iiwa7_no_collision.sdf");
    iiwa_plant_->WeldFrames(iiwa_plant_->world_frame(),
                            iiwa_plant_->GetFrameByName("iiwa_link_0"));
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

  // TODO(hongkai-dai): Consider hoisting this function into the toppra API.
  // Perhaps with the sugar where s_path has the default implementation of being
  // s_path(t) = s.

  // Computes the spatial velocity of a given `frame`. The state of the iiwa
  // plant is defined by the underlying geometric path_ and the re-parameterized
  // trajectory as defined by s_path (mapping time t to path parameter s). This
  // writes to the test's allocated MbP Context.
  SpatialVelocity<double> CalcSpatialVelocity(
      double t, const PiecewisePolynomial<double>& s_path,
      const Frame<double>& frame) {
    const double s = s_path.scalarValue(t);
    const auto s_dot = s_path.EvalDerivative(t, 1);
    const auto dq_ds = path_.EvalDerivative(s, 1);
    const auto position = path_.value(s);
    const auto velocity = dq_ds * s_dot;

    iiwa_plant_->SetPositions(plant_context_.get(), position);
    iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
    return frame.CalcSpatialVelocityInWorld(*plant_context_);
  }

  // Computes the spatial acceleration of a given `frame`.  The state of the
  // iiwa plant is defined by the underlying geometric path_ and the
  // re-parameterized trajectory as defined by s_path (mapping time t to path
  // parameter s). This writes to the test's allocated MbP Context.
  Vector6d CalcSpatialAcceleration(double t,
                                   const PiecewisePolynomial<double>& s_path,
                                   const Frame<double>& frame) {
    const auto s_dot = s_path.EvalDerivative(t, 1);
    const auto s_ddot = s_path.EvalDerivative(t, 2);
    const double s = s_path.scalarValue(t);
    const auto dq_ds = path_.EvalDerivative(s, 1);
    const auto ddq_dds = path_.EvalDerivative(s, 2);
    const auto position = path_.value(s);
    const auto velocity = dq_ds * s_dot;
    const auto acceleration = dq_ds * s_ddot + ddq_dds * s_dot * s_dot;

    iiwa_plant_->SetPositions(plant_context_.get(), position);
    iiwa_plant_->SetVelocities(plant_context_.get(), velocity);
    Eigen::MatrixXd J(6, 7);
    iiwa_plant_->CalcJacobianSpatialVelocity(
        *plant_context_, JacobianWrtVariable::kQDot, frame,
        Eigen::Vector3d::Zero(), iiwa_plant_->world_frame(),
        iiwa_plant_->world_frame(), &J);
    Eigen::VectorXd J_dot_v(6);
    J_dot_v = iiwa_plant_
                  ->CalcBiasSpatialAcceleration(
                      *plant_context_, JacobianWrtVariable::kV, frame,
                      Eigen::Vector3d::Zero(), iiwa_plant_->world_frame(),
                      iiwa_plant_->world_frame())
                  .get_coeffs();
    return J * acceleration + J_dot_v;
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> iiwa_plant_;
  std::unique_ptr<systems::Context<double>> plant_context_;
  PiecewisePolynomial<double> path_;
  std::unique_ptr<Toppra> toppra_;
};

TEST_F(IiwaToppraTest, BoundaryConditions) {
  toppra_->AddJointVelocityLimit(Eigen::VectorXd::Constant(7, -1),
                                 Eigen::VectorXd::Constant(7, 1));
  toppra_->AddJointAccelerationLimit(Eigen::VectorXd::Constant(7, -10),
                                     Eigen::VectorXd::Constant(7, 10));
  const double s_dot_start = 0.7;
  const double s_dot_end = 0.4;
  auto result = toppra_->SolvePathParameterization(s_dot_start, s_dot_end);
  ASSERT_TRUE(result);
  auto s_path = result.value();
  auto s_path_dot = s_path.derivative();
  EXPECT_NEAR(s_path_dot.value(s_path.start_time())(0, 0), s_dot_start, 1e-6);
  EXPECT_NEAR(s_path_dot.value(s_path.end_time())(0, 0), s_dot_end, 1e-6);
}

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

  for (bool use_trajectory : {true, false}) {
    SCOPED_TRACE(
        fmt::format("Speed limit with trajectory: {}", use_trajectory));
    if (use_trajectory) {
      // A trajectory limit with a const image (defined over the domain of the
      // path) should be identical to the single, constant limit.
      const std::vector<double> breaks{path_.start_time(), path_.end_time()};
      const Eigen::MatrixXd upper_bound_mat =
          Vector1<double>::Constant(upper_bound);
      const std::vector samples{upper_bound_mat, upper_bound_mat};
      const auto upper_bound_traj =
          PiecewisePolynomial<double>::ZeroOrderHold(breaks, samples);

      toppra_->AddFrameTranslationalSpeedLimit(frame, upper_bound_traj);
    } else {
      toppra_->AddFrameTranslationalSpeedLimit(frame, upper_bound);
    }
    toppra_->AddJointAccelerationLimit(Eigen::VectorXd::Constant(7, -10),
                                       Eigen::VectorXd::Constant(7, 10));

    auto result = toppra_->SolvePathParameterization();
    ASSERT_TRUE(result);
    auto s_path = result.value();

    // This tolerance was tuned to work with the given gridpoints.
    const double tol = 1e-14;
    for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
      const auto frame_velocity =
          CalcSpatialVelocity(s_path.start_time(ii), s_path, frame);
      const double speed = frame_velocity.translational().norm();

      EXPECT_LT(speed, upper_bound + tol);
    }

    const auto frame_velocity =
        CalcSpatialVelocity(s_path.end_time(), s_path, frame);
    const double speed = frame_velocity.translational().norm();

    EXPECT_LT(speed, upper_bound + tol);
  }
}

// Tests (non-)error responses based on the (non-)overlapping of limit and
// path domains.
TEST_F(IiwaToppraTest, FrameTranslationalSpeedLimitTrajectoryOverlap) {
  const double upper_bound = 1.1;
  const Eigen::MatrixXd upper_bound_mat =
      Vector1<double>::Constant(upper_bound);
  const std::vector samples{upper_bound_mat, upper_bound_mat};
  const auto& frame = iiwa_plant_->GetFrameByName("iiwa_link_7");

  const double path_start = path_.start_time();
  const double path_end = path_.end_time();

  // Limit domain lies "above" path domain.
  {
    const std::vector<double> breaks{path_end + 0.1, path_end + 1.1};
    const auto upper_bound_traj =
        PiecewisePolynomial<double>::ZeroOrderHold(breaks, samples);
    DRAKE_EXPECT_THROWS_MESSAGE(
        toppra_->AddFrameTranslationalSpeedLimit(frame, upper_bound_traj),
        fmt::format(".* can't add a trajectory translational speed .* limit "
                    "domain \\[{}, {}\\].* path domain \\[{}, {}\\].",
                    breaks[0], breaks[1], path_start, path_end));
  }

  // Limit domain lies "below" path domain.
  {
    const std::vector<double> breaks{path_start - 1.1, path_start - 0.1};
    const auto upper_bound_traj =
        PiecewisePolynomial<double>::ZeroOrderHold(breaks, samples);
    DRAKE_EXPECT_THROWS_MESSAGE(
        toppra_->AddFrameTranslationalSpeedLimit(frame, upper_bound_traj),
        ".* can't add a trajectory translational speed .* limit .*");
  }

  // Overlap at the lower range of the path domain is valid.
  {
    const std::vector<double> breaks{path_start - 1.1,
                                     (path_start + path_end) / 2};
    const auto upper_bound_traj =
        PiecewisePolynomial<double>::ZeroOrderHold(breaks, samples);
    EXPECT_NO_THROW(
        toppra_->AddFrameTranslationalSpeedLimit(frame, upper_bound_traj));
  }

  // Overlap at the upper range of the path domain is valid.
  {
    const std::vector<double> breaks{(path_start + path_end) / 2,
                                     path_end + 1.1};
    const auto upper_bound_traj =
        PiecewisePolynomial<double>::ZeroOrderHold(breaks, samples);
    EXPECT_NO_THROW(
        toppra_->AddFrameTranslationalSpeedLimit(frame, upper_bound_traj));
  }
}

// Confirms that if the limit varies over the path, the observed speed respects
// it.
TEST_F(IiwaToppraTest, FrameTranslationalSpeedLimitVarying) {
  const auto& frame = iiwa_plant_->GetFrameByName("iiwa_link_7");

  // Create a trajectory with a lower speed for the first half and a higher
  // speed for the second half.
  const double s0 = path_.start_time();
  const double s2 = path_.end_time();
  const double s1 = (s0 + s2) / 2;
  const double limit0 = 1.1;  // bound for the interval [s0, s1).
  const double limit1 = 2.2;  // bound for the interval [s1, s2).
  Eigen::MatrixXd samples(1, 3);
  samples.row(0) << limit0, limit1, limit1;
  const auto bound_traj = PiecewisePolynomial<double>::ZeroOrderHold(
      Eigen::VectorXd::LinSpaced(3, s0, s2), samples);

  toppra_->AddFrameTranslationalSpeedLimit(frame, bound_traj);
  toppra_->AddJointAccelerationLimit(Eigen::VectorXd::Constant(7, -10),
                                     Eigen::VectorXd::Constant(7, 10));

  auto result = toppra_->SolvePathParameterization();
  ASSERT_TRUE(result);
  auto s_path = result.value();

  auto calc_speed_and_limit = [&s_path, &frame, &bound_traj,
                               this](double t) -> std::pair<double, double> {
    const double s = s_path.scalarValue(t);
    const SpatialVelocity<double> frame_velocity =
        CalcSpatialVelocity(t, s_path, frame);
    const double speed = frame_velocity.translational().norm();
    const double speed_limit = bound_traj.scalarValue(s);
    return {speed, speed_limit};
  };

  // This tolerance was tuned to work with the given gridpoints.
  bool second_half_sped_up = false;
  const double tol = 1e-14;
  for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
    const double t = s_path.start_time(ii);
    const auto [speed, speed_limit] = calc_speed_and_limit(t);

    // We should always respect the instantaneous speed limit.
    EXPECT_LT(speed, speed_limit + tol);

    // In the first half, we should remain below limit0. In the second half we
    // should rise above limit0. Because the trajectory is hard-coded to
    // terminate with zero velocity, we will simply detect that the speed
    // rises above the lower limit *after* the change and before the end.
    if (s_path.scalarValue(t) >= s1) {
      second_half_sped_up = second_half_sped_up || speed > limit0;
    }
  }

  const double t = s_path.end_time();
  const auto [speed, speed_limit] = calc_speed_and_limit(t);

  EXPECT_LT(speed, speed_limit + tol);
  EXPECT_TRUE(second_half_sped_up);
}

TEST_F(IiwaToppraTest, FrameAccelerationLimit) {
  Eigen::VectorXd lower_bound(6);
  lower_bound << -1.1, -1.2, -1.3, -1.4, -1.5, -1.6;
  Eigen::VectorXd upper_bound(6);
  upper_bound << 2.1, 2.2, 2.3, 2.4, 2.5, 2.6;
  const auto& frame = iiwa_plant_->GetFrameByName("iiwa_link_7");

  for (bool use_trajectory : {true, false}) {
    SCOPED_TRACE(fmt::format("Frame acceleration limit with trajectory: {}",
                             use_trajectory));
    if (use_trajectory) {
      const std::vector<double> breaks{path_.start_time(), path_.end_time()};
      const std::vector<MatrixX<double>> lower_samples{lower_bound,
                                                       lower_bound};
      const auto lower_bound_traj =
          PiecewisePolynomial<double>::ZeroOrderHold(breaks, lower_samples);

      const std::vector<MatrixX<double>> upper_samples{upper_bound,
                                                       upper_bound};
      const auto upper_bound_traj =
          PiecewisePolynomial<double>::ZeroOrderHold(breaks, upper_samples);

      toppra_->AddFrameAccelerationLimit(frame, lower_bound_traj,
                                         upper_bound_traj);
    } else {
      toppra_->AddFrameAccelerationLimit(frame, lower_bound, upper_bound);
    }

    auto result = toppra_->SolvePathParameterization();
    ASSERT_TRUE(result);
    auto s_path = result.value();

    // This tolerance was tuned to work with the given gridpoints.
    const double tol = 1e-14;
    for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
      Vector6d frame_acceleration =
          CalcSpatialAcceleration(s_path.start_time(ii), s_path, frame);

      EXPECT_TRUE(
          (frame_acceleration.array() >= lower_bound.array() - tol).all());
      EXPECT_TRUE(
          (frame_acceleration.array() <= upper_bound.array() + tol).all());
    }

    Vector6d frame_acceleration =
        CalcSpatialAcceleration(s_path.end_time(), s_path, frame);

    EXPECT_TRUE(
        (frame_acceleration.array() >= lower_bound.array() - tol).all());
    EXPECT_TRUE(
        (frame_acceleration.array() <= upper_bound.array() + tol).all());
  }
}

// Tests (non-)error responses based on the (non-)overlapping of limit and
// path domains.
TEST_F(IiwaToppraTest, FrameAccelerationLimitTrajectoryOverlap) {
  Eigen::VectorXd lower_bound(6);
  lower_bound << -1.1, -1.2, -1.3, -1.4, -1.5, -1.6;
  const std::vector<MatrixX<double>> lower_samples{lower_bound, lower_bound};

  Eigen::VectorXd upper_bound(6);
  upper_bound << 2.1, 2.2, 2.3, 2.4, 2.5, 2.6;
  const std::vector<MatrixX<double>> upper_samples{upper_bound, upper_bound};

  const auto& frame = iiwa_plant_->GetFrameByName("iiwa_link_7");

  const double path_start = path_.start_time();
  const double path_end = path_.end_time();
  const std::vector<double> big_breaks{path_start - 0.1, path_end + 0.1};

  const auto lower_bigger =
      PiecewisePolynomial<double>::ZeroOrderHold(big_breaks, lower_samples);
  const auto upper_bigger =
      PiecewisePolynomial<double>::ZeroOrderHold(big_breaks, upper_samples);

  {
    // Limit domain lies "above" path domain.
    const std::vector<double> bad_breaks{path_end + 0.1, path_end + 1.1};
    const auto lower_bad =
        PiecewisePolynomial<double>::ZeroOrderHold(bad_breaks, lower_samples);
    const auto upper_bad =
        PiecewisePolynomial<double>::ZeroOrderHold(bad_breaks, upper_samples);

    // Just for lower.
    DRAKE_EXPECT_THROWS_MESSAGE(
        toppra_->AddFrameAccelerationLimit(frame, lower_bad, upper_bigger),
        fmt::format(".* can't add a trajectory frame acceleration .* lower "
                    "limit domain \\[{}, {}\\].* upper limit domain "
                    "\\[{}, {}\\] .* path domain \\[{}, {}\\].",
                    bad_breaks[0], bad_breaks[1], big_breaks[0], big_breaks[1],
                    path_start, path_end));

    // Just for upper.
    DRAKE_EXPECT_THROWS_MESSAGE(
        toppra_->AddFrameAccelerationLimit(frame, lower_bigger, upper_bad),
        ".* can't add a trajectory frame acceleration .*");

    // Both.
    DRAKE_EXPECT_THROWS_MESSAGE(
        toppra_->AddFrameAccelerationLimit(frame, lower_bad, upper_bad),
        ".* can't add a trajectory frame acceleration .*");
  }

  {
    // Limit domain lies "below" path domain.
    const std::vector<double> bad_breaks{path_start - 1, path_start - 0.1};
    const auto lower_bad =
        PiecewisePolynomial<double>::ZeroOrderHold(bad_breaks, lower_samples);
    const auto upper_bad =
        PiecewisePolynomial<double>::ZeroOrderHold(bad_breaks, upper_samples);

    // Just for lower.
    DRAKE_EXPECT_THROWS_MESSAGE(
        toppra_->AddFrameAccelerationLimit(frame, lower_bad, upper_bigger),
        ".* can't add a trajectory frame acceleration .*");

    // Just for upper.
    DRAKE_EXPECT_THROWS_MESSAGE(
        toppra_->AddFrameAccelerationLimit(frame, lower_bigger, upper_bad),
        ".* can't add a trajectory frame acceleration .*");

    // Both.
    DRAKE_EXPECT_THROWS_MESSAGE(
        toppra_->AddFrameAccelerationLimit(frame, lower_bad, upper_bad),
        ".* can't add a trajectory frame acceleration .*");
  }

  {
    // We'll use a single test to infer that the limit domains can partially
    // overlap. We'll send each limit in different directions.
    const std::vector<double> low_breaks{path_start - 1,
                                         (path_start + path_end) / 2};
    const std::vector<double> high_breaks{(path_start + path_end) / 2,
                                          path_end + 1};
    const auto lower_traj =
        PiecewisePolynomial<double>::ZeroOrderHold(high_breaks, lower_samples);
    const auto upper_traj =
        PiecewisePolynomial<double>::ZeroOrderHold(low_breaks, upper_samples);
    EXPECT_NO_THROW(
        toppra_->AddFrameAccelerationLimit(frame, lower_traj, upper_traj));
  }
}

// Confirms that if the limit varies over the path, the observed acceleration
// respects it.
TEST_F(IiwaToppraTest, FrameAccelerationLimitVarying) {
  const auto& frame = iiwa_plant_->GetFrameByName("iiwa_link_7");

  // Create a trajectory with a lower speed for the first half and a higher
  // speed for the second half.
  const double s0 = path_.start_time();
  const double s2 = path_.end_time();
  const double s1 = (s0 + s2) / 2;

  // Lower limit for the interval [s0, s1).
  const Vector6d lower0 = Vector6d::Constant(-0.5);
  const Vector6d upper0 = -lower0;
  // Lower limit for the interval [s1, s2).
  const Vector6d lower1 = Vector6d::Constant(-7);
  const Vector6d upper1 = -lower1;

  Eigen::MatrixXd lower_samples(6, 3);
  lower_samples << lower0, lower1, lower1;
  const auto lower_traj = PiecewisePolynomial<double>::ZeroOrderHold(
      Eigen::VectorXd::LinSpaced(3, s0, s2), lower_samples);
  Eigen::MatrixXd upper_samples(6, 3);
  upper_samples << upper0, upper1, upper1;
  const auto upper_traj = PiecewisePolynomial<double>::ZeroOrderHold(
      Eigen::VectorXd::LinSpaced(3, s0, s2), upper_samples);

  toppra_->AddFrameAccelerationLimit(frame, lower_traj, upper_traj);

  auto result = toppra_->SolvePathParameterization();
  ASSERT_TRUE(result);
  auto s_path = result.value();

  auto calc_accel_lower_and_upper_limit =
      [&s_path, &frame, &lower_traj, &upper_traj,
       this](double t) -> std::tuple<Vector6d, Vector6d, Vector6d> {
    const double s = s_path.scalarValue(t);
    const Vector6d acceleration = CalcSpatialAcceleration(t, s_path, frame);
    return {acceleration, lower_traj.value(s), upper_traj.value(s)};
  };

  // This tolerance was tuned to work with the given gridpoints.
  bool second_half_increased_accleration = false;
  const double tol = 2e-14;
  for (int ii = 0; ii < s_path.get_number_of_segments(); ii++) {
    const double t = s_path.start_time(ii);
    auto [accel, lower, upper] = calc_accel_lower_and_upper_limit(t);

    // We should always respect the instantaneous acceleration limit.
    EXPECT_TRUE((accel.array() >= lower.array() - tol).all());
    EXPECT_TRUE((accel.array() <= upper.array() + tol).all());

    // In the first half, we should remain within the smaller [lower0, upper0]
    // constraint box. In the second half we should fill the larger box. We
    // can't assert the acceleration at each gridpoint; instead, we'll simply
    // assert that we *observed* a change in accleration magnitude in the
    // second half that is larger than the first half's small box.
    if (s_path.scalarValue(t) >= s1) {
      second_half_increased_accleration =
          second_half_increased_accleration ||
          (accel.array().abs() > upper1.array()).any();
    }
  }
  const double t = s_path.end_time();
  auto [accel, lower, upper] = calc_accel_lower_and_upper_limit(t);

  EXPECT_TRUE((accel.array() >= lower.array() - tol).all());
  EXPECT_TRUE((accel.array() <= upper.array() + tol).all());

  EXPECT_TRUE(second_half_increased_accleration);
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
