#include "drake/math/bspline_trajectory.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/proto/call_python.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/compute_numerical_gradient.h"

DEFINE_bool(visualize, false,
            "If true, emit Python plotting commands using CallPython().");

namespace drake {
namespace math {
namespace {
MatrixX<double> NaiveBsplineTrajectoryValue(
    const BsplineTrajectory<double>& trajectory, double parameter_value) {
  MatrixX<double> value =
      MatrixX<double>::Zero(trajectory.rows(), trajectory.cols());
  for (int i = 0; i < trajectory.num_control_points(); ++i) {
    value += trajectory.BasisFunctionValue(i, parameter_value) *
             trajectory.control_points()[i];
  }
  return value;
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

// Verifies that value() works as expected.
GTEST_TEST(BsplineTrajectoryTests, ValueTest) {
  const int order = 4;
  const int num_control_points = 11;
  const int rows = 2;
  const int cols = 1;
  std::vector<MatrixX<double>> control_points{};
  const auto t_dummy =
      VectorX<double>::LinSpaced(num_control_points, 0, 2 * M_PI);
  for (int i = 0; i < num_control_points; ++i) {
    control_points.push_back(
        (MatrixX<double>(rows, cols) << std::sin(t_dummy(i)),
         std::cos(t_dummy(i)))
            .finished());
  }
  for (const auto& knot_vector_type :
       {KnotVectorType::kUniform, KnotVectorType::kClampedUniform}) {
    BsplineTrajectory<double> trajectory{
        BsplineBasis<double>{order, num_control_points, knot_vector_type},
        control_points};

    // Verify that value() returns the expected results.
    const int num_times = 100;
    VectorX<double> t = VectorX<double>::LinSpaced(
        num_times, trajectory.start_time(), trajectory.end_time());
    for (int k = 0; k < num_times; ++k) {
      MatrixX<double> value = trajectory.value(t(k));
      MatrixX<double> expected_value =
          NaiveBsplineTrajectoryValue(trajectory, t(k));
      EXPECT_TRUE(CompareMatrices(value, expected_value,
                                  16 * std::numeric_limits<double>::epsilon()));
    }
  }
}

// Verifies that Derivative() works as expected.
GTEST_TEST(BsplineTrajectoryTests, DerivativeTest) {
  const int order = 4;
  const int num_control_points = 11;
  const int rows = 2;
  const int cols = 1;
  std::vector<MatrixX<double>> control_points{};
  const auto t_dummy =
      VectorX<double>::LinSpaced(num_control_points, 0, 2 * M_PI);
  for (int i = 0; i < num_control_points; ++i) {
    control_points.push_back(
        (MatrixX<double>(rows, cols) << std::sin(t_dummy(i)),
         std::cos(t_dummy(i)))
            .finished());
  }
  for (const auto& knot_vector_type :
       {KnotVectorType::kUniform, KnotVectorType::kClampedUniform}) {
    BsplineTrajectory<double> trajectory{
        BsplineBasis<double>{order, num_control_points, knot_vector_type},
        control_points};

    // Verify that Derivative() returns the expected results.
    BsplineTrajectory<double> derivative_trajectory = trajectory.Derivative();
    std::function<void(const Vector1<double>&, VectorX<double>*)> calc_value =
        [&trajectory](const Vector1<double> t, VectorX<double>* value) {
          *value = trajectory.value(t(0));
        };
    const int num_times = 100;
    VectorX<double> t = VectorX<double>::LinSpaced(
        num_times, trajectory.start_time(), trajectory.end_time());
    for (int k = 0; k < num_times; ++k) {
      MatrixX<double> derivative = derivative_trajectory.value(t(k));
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
    BsplineTrajectory<double> trajectory = BsplineTrajectory<double>{
      BsplineBasis<double>{order, num_control_points, knot_vector_type},
      control_points}.CopyBlock(0, 0, 2, 3);
    BsplineTrajectory<double> expected_trajectory = BsplineTrajectory<double>{
      BsplineBasis<double>{order, num_control_points, knot_vector_type},
      expected_control_points};
    EXPECT_EQ(trajectory, expected_trajectory);
  }
}

// Verifies that InsertKnots() works as expected.
GTEST_TEST(BsplineTrajectoryTests, InsertKnotsTest) {
  using common::CallPython;
  const int order = 4;
  const int num_control_points = 11;
  const int rows = 2;
  const int cols = 1;
  std::vector<MatrixX<double>> control_points{};
  const auto t_dummy =
      VectorX<double>::LinSpaced(num_control_points, 0, 2 * M_PI);
  for (int i = 0; i < num_control_points; ++i) {
    control_points.push_back(
        (MatrixX<double>(rows, cols) << std::sin(t_dummy(i)),
         std::cos(t_dummy(i)))
            .finished());
  }
  for (const auto& knot_vector_type :
    {KnotVectorType::kUniform, KnotVectorType::kClampedUniform}) {
    BsplineTrajectory<double> original_trajectory{
        BsplineBasis<double>{order, num_control_points, knot_vector_type},
        control_points};
    // Create a vector of new knots to add
    std::vector<double> new_knots{original_trajectory.start_time(), M_PI_4,
          0.25, 0.25, original_trajectory.end_time()};
    BsplineTrajectory<double> trajectory_with_new_knots = original_trajectory;
    trajectory_with_new_knots.InsertKnots({new_knots});
    EXPECT_EQ(trajectory_with_new_knots.num_control_points(),
              original_trajectory.num_control_points() + new_knots.size());
    // Verify that value() returns the expected results.
    const int num_times = 100;
    VectorX<double> t = VectorX<double>::LinSpaced(
        num_times, original_trajectory.start_time(),
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
}

}  // namespace math
}  // namespace drake
