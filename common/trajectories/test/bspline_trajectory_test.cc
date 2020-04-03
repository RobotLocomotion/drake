#include "drake/common/trajectories/bspline_trajectory.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/proto/call_python.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/compute_numerical_gradient.h"
#include "drake/math/knot_vector_type.h"

DEFINE_bool(visualize, false,
            "If true, emit Python plotting commands using CallPython(). "
            "You must build and run //common/proto:call_python_client_cli "
            "first in order to see the resulting plots (you will also need to "
            "set up an rpc file - see the --help for call_python_client_cli "
            "for more information). This option does not work with "
            "`bazel run`. Run the binary from bazel-bin instead.");

namespace drake {
namespace trajectories {

using math::BsplineBasis;
using math::ComputeNumericalGradient;
using math::KnotVectorType;
using math::NumericalGradientMethod;
using math::NumericalGradientOption;

namespace {
BsplineTrajectory<double> MakeCircleTrajectory() {
  const int order = 4;
  const int num_control_points = 11;
  std::vector<MatrixX<double>> control_points{};
  const auto t_dummy =
      VectorX<double>::LinSpaced(num_control_points, 0, 2 * M_PI);
  for (int i = 0; i < num_control_points; ++i) {
    control_points.push_back(
        (MatrixX<double>(2, 1) << std::sin(t_dummy(i)), std::cos(t_dummy(i)))
            .finished());
  }
  return {BsplineBasis<double>{order, num_control_points}, control_points};
}
}  // namespace

// Verifies that the constructors work as expected.
GTEST_TEST(BsplineTrajectoryTests, ConstructorTest) {
  const int expected_order = 4;
  const int expected_num_control_points = 11;
  const std::vector<double> expected_knots{
      0, 0, 0, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1, 1, 1, 1};
  const int expected_rows = 2;
  const int expected_cols = 1;
  const double expected_start_time = 0;
  const double expected_end_time = 1;
  std::vector<MatrixX<double>> expected_control_points{};
  for (int i = 0; i < expected_num_control_points; ++i) {
    expected_control_points.push_back(
        i * MatrixX<double>::Ones(expected_rows, expected_cols));
  }
  BsplineBasis<double> bspline_basis_1{expected_order, expected_knots};
  BsplineTrajectory<double> trajectory{bspline_basis_1,
                                       expected_control_points};

  // Verify method return values.
  EXPECT_EQ(trajectory.rows(), expected_rows);
  EXPECT_EQ(trajectory.cols(), expected_cols);
  EXPECT_EQ(trajectory.start_time(), expected_start_time);
  EXPECT_EQ(trajectory.end_time(), expected_end_time);
  EXPECT_EQ(trajectory.control_points(), expected_control_points);
}

// Verifies that value() works as expected (i.e. as a thin wrapper on
// basis().EvaluateCurve() with clamping).
GTEST_TEST(BsplineTrajectoryTests, ValueTest) {
  BsplineTrajectory<double> trajectory = MakeCircleTrajectory();

  // Verify that value() returns the expected results.
  const int num_times = 100;
  VectorX<double> t = VectorX<double>::LinSpaced(
      num_times, trajectory.start_time() - 0.1, trajectory.end_time() + 0.1);
  for (int k = 0; k < num_times; ++k) {
    MatrixX<double> value = trajectory.value(t(k));
    double t_clamped = std::min(std::max(t(k), trajectory.start_time()),
                                trajectory.end_time());
    MatrixX<double> expected_value = trajectory.basis().EvaluateCurve(
        trajectory.control_points(), t_clamped);
    EXPECT_TRUE(CompareMatrices(value, expected_value,
                                std::numeric_limits<double>::epsilon()));
  }
}

// Verifies that MakeDerivative() works as expected.
GTEST_TEST(BsplineTrajectoryTests, MakeDerivativeTest) {
  BsplineTrajectory<double> trajectory = MakeCircleTrajectory();

  // Verify that MakeDerivative() returns the expected results.
  std::unique_ptr<Trajectory<double>> derivative_trajectory =
      trajectory.MakeDerivative();
  std::function<void(const Vector1<double>&, VectorX<double>*)> calc_value =
      [&trajectory](const Vector1<double>& t, VectorX<double>* value) {
        *value = trajectory.value(t(0));
      };
  const int num_times = 100;
  VectorX<double> t = VectorX<double>::LinSpaced(
      num_times, trajectory.start_time(), trajectory.end_time());
  for (int k = 0; k < num_times; ++k) {
    MatrixX<double> derivative = derivative_trajectory->value(t(k));
    // To avoid evaluating the B-spline trajectory outside of its domain, we
    // use forward/backward finite differences at the start/end points.
    NumericalGradientMethod method = NumericalGradientMethod::kCentral;
    double tolerance = 1e-7;
    if (k == 0) {
      method = NumericalGradientMethod::kForward;
      tolerance = 1e-5;
    } else if (k == num_times - 1) {
      method = NumericalGradientMethod::kBackward;
      tolerance = 1e-5;
    }
    MatrixX<double> expected_derivative = ComputeNumericalGradient(
        calc_value, Vector1<double>{t(k)}, NumericalGradientOption{method});
    EXPECT_TRUE(CompareMatrices(derivative, expected_derivative, tolerance));
  }
}

// Verifies that CopyBlock() works as expected.
GTEST_TEST(BsplineTrajectoryTests, CopyBlockTest) {
  const int order = 4;
  const int num_control_points = 11;
  const int rows = 3;
  const int cols = 3;
  std::vector<MatrixX<double>> control_points{};
  std::vector<MatrixX<double>> expected_control_points{};
  const auto t_dummy =
      VectorX<double>::LinSpaced(num_control_points, 0, 2 * M_PI);
  for (int i = 0; i < num_control_points; ++i) {
    double s = std::sin(t_dummy(i));
    double c = std::cos(t_dummy(i));
    // clang-format off
    control_points.push_back(
        (MatrixX<double>(rows, cols) <<
         s, c, s,
         s, s, c,
         c, s, s).finished());
    // clang-format on
    expected_control_points.push_back(control_points.back().block(0, 0, 2, 3));
  }
  for (const auto& knot_vector_type :
       {KnotVectorType::kUniform, KnotVectorType::kClampedUniform}) {
    BsplineTrajectory<double> trajectory =
        BsplineTrajectory<double>{
            BsplineBasis<double>{order, num_control_points, knot_vector_type},
            control_points}
            .CopyBlock(0, 0, 2, 3);
    BsplineTrajectory<double> expected_trajectory = BsplineTrajectory<double>{
        BsplineBasis<double>{order, num_control_points, knot_vector_type},
        expected_control_points};
    EXPECT_EQ(trajectory, expected_trajectory);
  }
}

// Verifies that InsertKnots() works as expected.
GTEST_TEST(BsplineTrajectoryTests, InsertKnotsTest) {
  using common::CallPython;
  BsplineTrajectory<double> original_trajectory = MakeCircleTrajectory();

  // Create a vector of new knots to add. Note that it contains a knot with a
  // multiplicity of 2.
  std::vector<double> new_knots{original_trajectory.start_time(), M_PI_4, 0.25,
                                0.25, original_trajectory.end_time()};
  // Add the new knots.
  BsplineTrajectory<double> trajectory_with_new_knots = original_trajectory;
  trajectory_with_new_knots.InsertKnots({new_knots});

  // Verify that the number of control points after insertion is equal to the
  // original number of control points plus the number of added knots.
  EXPECT_EQ(trajectory_with_new_knots.num_control_points(),
            original_trajectory.num_control_points() + new_knots.size());

  // Add the new knots again to verify that nothing goes wrong if the
  // trajectory into which knots are inserted has interior knots with
  // multiplicity greater than 1.
  trajectory_with_new_knots.InsertKnots({new_knots});

  // Verify that the number of control points after insertion is equal to the
  // original number of control points plus the number of added knots.
  EXPECT_EQ(trajectory_with_new_knots.num_control_points(),
            original_trajectory.num_control_points() + 2 * new_knots.size());

  // Verify that start_time() and end_time() return the same results for the
  // original trajectory and the one with additional knots.
  EXPECT_EQ(trajectory_with_new_knots.start_time(),
            original_trajectory.start_time());
  EXPECT_EQ(trajectory_with_new_knots.end_time(),
            original_trajectory.end_time());

  // Verify that value() returns the same result for the original trajectory
  // and the trajectory with additional knots for `num_times` sampled values
  // of `t` between start_time() and end_time().
  const int num_times = 100;
  VectorX<double> t =
      VectorX<double>::LinSpaced(num_times, original_trajectory.start_time(),
                                 original_trajectory.end_time());
  const double tolerance = 2 * std::numeric_limits<double>::epsilon();
  if (FLAGS_visualize) {
    CallPython("figure");
  }
  for (int k = 0; k < num_times; ++k) {
    MatrixX<double> value = trajectory_with_new_knots.value(t(k));
    MatrixX<double> expected_value = original_trajectory.value(t(k));
    EXPECT_TRUE(CompareMatrices(value, expected_value, tolerance));
    if (FLAGS_visualize) {
      CallPython(
          "plot", t(k), value.transpose(),
          CompareMatrices(value, expected_value, tolerance) ? "gs" : "rs");
      CallPython("plot", t(k), expected_value.transpose(), "ko");
    }
  }
  if (FLAGS_visualize) {
    CallPython("grid", true);
  }
}

}  // namespace trajectories
}  // namespace drake
