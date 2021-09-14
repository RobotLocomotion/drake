#include "drake/multibody/optimization/toppra.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"

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

    Eigen::MatrixXd knots(7, 2);
    knots.col(0) << -1.57, 0.1, 0, -1.2, 0, 1.6, 0;
    knots.col(1) << -0.8, -0.6, 0.5, 0.1, -0.5, -0.1, -1;
    path_ = PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
        Eigen::VectorXd::LinSpaced(2, 0, 1), knots, Eigen::VectorXd::Zero(7),
        Eigen::VectorXd::Zero(7));

    auto options = CalcGridPointsOptions();
    options.max_err = 1e-5;
    auto gridpts = Toppra::CalcGridpts(path_, options);
    toppra_ = std::make_unique<Toppra>(path_, *iiwa_plant_, gridpts);
  }

  ~IiwaToppraTest() override {}

 protected:
  std::unique_ptr<MultibodyPlant<double>> iiwa_plant_;
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

  auto result = toppra_->Solve();
  ASSERT_TRUE(result);
  auto trajectory = result.value();

  Eigen::VectorXd knot_lb, knot_ub;
  std::tie(knot_lb, knot_ub) = velocity_constraint;
  EXPECT_EQ(knot_lb.size(), trajectory.get_number_of_segments());
  EXPECT_EQ(knot_ub.size(), trajectory.get_number_of_segments());
  EXPECT_TRUE((knot_ub.array() >= knot_lb.array()).all());

  const double tol = 1e-6;
  for (int ii = 0; ii < trajectory.get_number_of_segments(); ii++) {
    const auto velocity =
        trajectory.EvalDerivative(trajectory.start_time(ii), 1);

    EXPECT_TRUE((velocity.array() >= lower_bound.array() + tol).all());
    EXPECT_TRUE((velocity.array() <= upper_bound.array() + tol).all());
  }

  const auto velocity = trajectory.EvalDerivative(trajectory.end_time(), 1);

  EXPECT_TRUE((velocity.array() >= lower_bound.array() + tol).all());
  EXPECT_TRUE((velocity.array() <= upper_bound.array() + tol).all());
}

TEST_F(IiwaToppraTest, JointAccelerationLimit) {
  Eigen::VectorXd lower_bound(7);
  lower_bound << -1.1, -1.2, -1.3, -1.4, -1.5, -1.6, -1.7;
  Eigen::VectorXd upper_bound(7);
  upper_bound << 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7;

  auto acceleration_constraint =
      toppra_->AddJointAccelerationLimit(lower_bound, upper_bound);

  auto result = toppra_->Solve();
  ASSERT_TRUE(result);
  auto trajectory = result.value();

  const double tol = 1e-6;
  for (int ii = 0; ii < trajectory.get_number_of_segments(); ii++) {
    const auto acceleration =
        trajectory.EvalDerivative(trajectory.start_time(ii), 2);

    EXPECT_TRUE((acceleration.array() >= lower_bound.array() + tol).all());
    EXPECT_TRUE((acceleration.array() <= upper_bound.array() + tol).all());
  }

  const auto acceleration = trajectory.EvalDerivative(trajectory.end_time(), 2);

  EXPECT_TRUE((acceleration.array() >= lower_bound.array() + tol).all());
  EXPECT_TRUE((acceleration.array() <= upper_bound.array() + tol).all());
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
  auto gridpts = Toppra::CalcGridpts(path, options);
  EXPECT_GT(gridpts.size(), options.min_points);

  // Check minimum segment length
  options = CalcGridPointsOptions();
  options.max_err = 100;
  options.max_seg_length = 0.05;
  options.min_points = 1;
  gridpts = Toppra::CalcGridpts(path, options);
  EXPECT_GT(gridpts.size(), 20);

  // Check iteration limit
  options = CalcGridPointsOptions();
  options.max_iter = 3;
  options.min_points = 1;
  gridpts = Toppra::CalcGridpts(path, options);
  EXPECT_EQ(gridpts.size(), 9);
}
}  // namespace
}  // namespace multibody
}  // namespace drake
