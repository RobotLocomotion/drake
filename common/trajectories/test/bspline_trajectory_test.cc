#include "drake/common/trajectories/bspline_trajectory.h"

#include <algorithm>
#include <functional>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/proto/call_python.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/math/autodiff_gradient.h"
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

using common::CallPython;
using math::ExtractGradient;
using math::BsplineBasis;
using math::ComputeNumericalGradient;
using math::KnotVectorType;
using math::NumericalGradientMethod;
using math::NumericalGradientOption;
using symbolic::Expression;
using yaml::LoadYamlString;

namespace {
template <typename T = double>
BsplineTrajectory<T> MakeCircleTrajectory() {
  using std::cos;
  using std::sin;
  const int order = 4;
  const int num_control_points = 11;
  std::vector<MatrixX<T>> control_points{};
  const auto t_dummy = VectorX<T>::LinSpaced(num_control_points, 0, 2 * M_PI);
  for (int i = 0; i < num_control_points; ++i) {
    control_points.push_back(
        (MatrixX<T>(2, 1) << sin(t_dummy(i)), cos(t_dummy(i))).finished());
  }
  return {BsplineBasis<T>{order, num_control_points}, control_points};
}

}  // namespace

template <typename T>
class BsplineTrajectoryTests : public ::testing::Test {};

using DefaultScalars = ::testing::Types<double, AutoDiffXd, Expression>;
TYPED_TEST_SUITE(BsplineTrajectoryTests, DefaultScalars);

// Verifies that the constructors work as expected.
TYPED_TEST(BsplineTrajectoryTests, ConstructorTest) {
  using T = TypeParam;
  const int expected_order = 4;
  const int expected_num_control_points = 11;
  const std::vector<T> expected_knots{
      0, 0, 0, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1, 1, 1, 1};
  const int expected_rows = 2;
  const int expected_cols = 1;
  const T expected_start_time = 0;
  const T expected_end_time = 1;
  std::vector<MatrixX<T>> expected_control_points{};
  for (int i = 0; i < expected_num_control_points; ++i) {
    expected_control_points.push_back(
        i * MatrixX<T>::Ones(expected_rows, expected_cols));
  }
  BsplineBasis<T> bspline_basis_1{expected_order, expected_knots};
  BsplineTrajectory<T> trajectory{bspline_basis_1, expected_control_points};

  // Verify method return values.
  EXPECT_EQ(trajectory.rows(), expected_rows);
  EXPECT_EQ(trajectory.cols(), expected_cols);
  EXPECT_EQ(trajectory.start_time(), expected_start_time);
  EXPECT_EQ(trajectory.end_time(), expected_end_time);
  // Use std::equal (not EXPECT_EQ) to deal with symbolic::Formula.
  const auto& traj_control_points = trajectory.control_points();
  EXPECT_TRUE(std::equal(
      traj_control_points.begin(), traj_control_points.end(),
      expected_control_points.begin(), expected_control_points.end(),
      [](const auto& traj_matrix, const auto& expected_matrix) {
        return (traj_matrix - expected_matrix).norm() == 0.0;
      }));

  // Verify that construction from BsplineBasis<double> works.
  std::vector<double> knots_double{};
  for (const auto& knot : expected_knots) {
    knots_double.push_back(ExtractDoubleOrThrow(knot));
  }
  BsplineBasis<double> bspline_basis_2{expected_order, knots_double};
  BsplineTrajectory<T> trajectory_from_double{bspline_basis_2,
                                              expected_control_points};
  EXPECT_EQ(trajectory_from_double, trajectory);
}

// Verifies that value() works as expected (i.e. as a thin wrapper on
// basis().EvaluateCurve() with clamping).
TYPED_TEST(BsplineTrajectoryTests, ValueTest) {
  using T = TypeParam;
  BsplineTrajectory<T> trajectory = MakeCircleTrajectory<T>();

  // Verify that value() returns the expected results.
  const int num_times = 100;
  VectorX<T> t = VectorX<T>::LinSpaced(num_times, trajectory.start_time() - 0.1,
                                       trajectory.end_time() + 0.1);
  for (int k = 0; k < num_times; ++k) {
    MatrixX<T> value = trajectory.value(t(k));
    using std::max;
    using std::min;
    T t_clamped = min(max(t(k), trajectory.start_time()),
                      trajectory.end_time());
    MatrixX<T> expected_value = trajectory.basis().EvaluateCurve(
        trajectory.control_points(), t_clamped);
    EXPECT_TRUE(CompareMatrices(value, expected_value,
                                std::numeric_limits<T>::epsilon()));
  }
}

// Verifies that MakeDerivative() works as expected.
TYPED_TEST(BsplineTrajectoryTests, MakeDerivativeTest) {
  using T = TypeParam;
  BsplineTrajectory<T> trajectory = MakeCircleTrajectory<T>();

  // Verify that MakeDerivative() returns the expected results.
  std::unique_ptr<Trajectory<T>> derivative_trajectory =
      trajectory.MakeDerivative();
  std::function<void(const Vector1<T>&, VectorX<T>*)> calc_value =
      [&trajectory](const Vector1<T>& t, VectorX<T>* value) {
        *value = trajectory.value(t(0));
      };
  const int num_times = 100;
  VectorX<T> t = VectorX<T>::LinSpaced(num_times, trajectory.start_time(),
                                       trajectory.end_time());
  for (int k = 0; k < num_times; ++k) {
    MatrixX<T> derivative = derivative_trajectory->value(t(k));
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
    MatrixX<T> expected_derivative = ComputeNumericalGradient(
        calc_value, Vector1<T>{t(k)}, NumericalGradientOption{method});
    EXPECT_TRUE(CompareMatrices(derivative, expected_derivative, tolerance));
  }

  // Verify that MakeDerivative() returns 0 matrix for derivative of order
  // higher than basis degree
  derivative_trajectory = trajectory.MakeDerivative(trajectory.basis().order());
  MatrixX<T> expected_derivative = MatrixX<T>::Zero(trajectory.rows(),
                                                    trajectory.cols());
  for (int k = 0; k < num_times; ++k) {
    MatrixX<T> derivative = derivative_trajectory->value(t(k));
    EXPECT_TRUE(CompareMatrices(derivative, expected_derivative, 0.0));
  }
}

// Verifies that EvalDerivative() works as expected.
TYPED_TEST(BsplineTrajectoryTests, EvalDerivativeTest) {
  using T = TypeParam;
  BsplineTrajectory<T> trajectory = MakeCircleTrajectory<T>();

  // Verify that EvalDerivative() returns the consistent results.
  const int num_times = 20;
  VectorX<T> t = VectorX<T>::LinSpaced(num_times, trajectory.start_time(),
                                       trajectory.end_time());
  for (int o = 0; o < trajectory.basis().order(); ++o) {
    std::unique_ptr<Trajectory<T>> derivative_trajectory =
        trajectory.MakeDerivative(o);
    for (int k = 0; k < num_times; ++k) {
      MatrixX<T> derivative = trajectory.EvalDerivative(t(k), o);
      MatrixX<T> expected_derivative = derivative_trajectory->value(t(k));
      double tolerance = 1e-14;
      EXPECT_TRUE(CompareMatrices(derivative, expected_derivative, tolerance));
    }
  }
}

// Verifies that CopyBlock() works as expected.
TYPED_TEST(BsplineTrajectoryTests, CopyBlockTest) {
  using T = TypeParam;
  using std::cos;
  using std::sin;
  const int order = 4;
  const int num_control_points = 11;
  const int rows = 3;
  const int cols = 3;
  std::vector<MatrixX<T>> control_points{};
  std::vector<MatrixX<T>> expected_control_points{};
  const auto t_dummy = VectorX<T>::LinSpaced(num_control_points, 0, 2 * M_PI);
  for (int i = 0; i < num_control_points; ++i) {
    T s = sin(t_dummy(i));
    T c = cos(t_dummy(i));
    // clang-format off
    control_points.push_back(
        (MatrixX<T>(rows, cols) <<
         s, c, s,
         s, s, c,
         c, s, s).finished());
    // clang-format on
    expected_control_points.push_back(control_points.back().block(0, 0, 2, 3));
  }
  for (const auto& knot_vector_type :
       {KnotVectorType::kUniform, KnotVectorType::kClampedUniform}) {
    BsplineTrajectory<T> trajectory =
        BsplineTrajectory<T>{
            BsplineBasis<T>{order, num_control_points, knot_vector_type},
            control_points}
            .CopyBlock(0, 0, 2, 3);
    BsplineTrajectory<T> expected_trajectory = BsplineTrajectory<T>{
        BsplineBasis<T>{order, num_control_points, knot_vector_type},
        expected_control_points};
    EXPECT_EQ(trajectory, expected_trajectory);
  }
}

// Verifies that InsertKnots() works as expected.
TYPED_TEST(BsplineTrajectoryTests, InsertKnotsTest) {
  using T = TypeParam;
  BsplineTrajectory<T> original_trajectory = MakeCircleTrajectory<T>();

  // Create a vector of new knots to add. Note that it contains a knot with a
  // multiplicity of 2.
  std::vector<T> new_knots{original_trajectory.start_time(), M_PI_4, 0.25, 0.25,
                           original_trajectory.end_time()};
  // Add the new knots.
  BsplineTrajectory<T> trajectory_with_new_knots = original_trajectory;
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
  VectorX<T> t =
      VectorX<T>::LinSpaced(num_times, original_trajectory.start_time(),
                            original_trajectory.end_time());
  const double tolerance = 2 * std::numeric_limits<double>::epsilon();
  if constexpr (std::is_same_v<T, double>) {
    if (FLAGS_visualize) {
      CallPython("figure");
    }
  }
  for (int k = 0; k < num_times; ++k) {
    MatrixX<T> value = trajectory_with_new_knots.value(t(k));
    MatrixX<T> expected_value = original_trajectory.value(t(k));
    EXPECT_TRUE(CompareMatrices(value, expected_value, tolerance));
    if constexpr (std::is_same_v<T, double>) {
      if (FLAGS_visualize) {
        CallPython(
            "plot", t(k), value.transpose(),
            CompareMatrices(value, expected_value, tolerance) ? "gs" : "rs");
        CallPython("plot", t(k), expected_value.transpose(), "ko");
      }
    }
  }
  if constexpr (std::is_same_v<T, double>) {
    if (FLAGS_visualize) {
      CallPython("grid", true);
    }
  }
}

// Verifies that the derivatives obtained by evaluating a
// `BsplineTrajectory<AutoDiffXd>` and extracting the gradient of the result
// match those obtained by taking the derivative of the whole trajectory and
// evaluating it at the same point.
GTEST_TEST(BsplineTrajectoryDerivativeTests, AutoDiffTest) {
  BsplineTrajectory<AutoDiffXd> trajectory = MakeCircleTrajectory<AutoDiffXd>();
  std::unique_ptr<Trajectory<double>> derivative_trajectory =
      MakeCircleTrajectory<double>().MakeDerivative();
  const int num_times = 100;
  VectorX<double> t = VectorX<double>::LinSpaced(
      num_times, ExtractDoubleOrThrow(trajectory.start_time()),
      ExtractDoubleOrThrow(trajectory.end_time()));
  const double kTolerance = 20 * std::numeric_limits<double>::epsilon();
  for (int k = 0; k < num_times; ++k) {
    AutoDiffXd t_k = math::InitializeAutoDiff(Vector1d{t(k)})[0];
    MatrixX<double> derivative_value = ExtractGradient(trajectory.value(t_k));
    MatrixX<double> expected_derivative_value =
        derivative_trajectory->value(t(k));
    EXPECT_TRUE(CompareMatrices(derivative_value, expected_derivative_value,
                                kTolerance));
  }
}

const char* const good = R"""(
  basis: !BsplineBasis
    order: 2
    knots: [0., 1., 1.5, 1.6, 2.]
  control_points:
    -
      - [0.0, 0.1, 0.2]
      - [0.3, 0.4, 0.5]
    -
      - [1.0, 1.1, 1.2]
      - [1.3, 1.4, 1.5]
    -
      - [2.0, 2.1, 2.2]
      - [2.3, 2.4, 2.5]
)""";

GTEST_TEST(BsplineTrajectorySerializeTests, GoodTest) {
  const int kOrder{2};
  const std::vector<double> knots{0., 1., 1.5, 1.6, 2.};
  const std::vector<MatrixX<double>> control_points{
      (MatrixX<double>(2, 3) << 0.0, 0.1, 0.2,
                                0.3, 0.4, 0.5).finished(),
      (MatrixX<double>(2, 3) << 1.0, 1.1, 1.2,
                                1.3, 1.4, 1.5).finished(),
      (MatrixX<double>(2, 3) << 2.0, 2.1, 2.2,
                                2.3, 2.4, 2.5).finished(),
  };
  const auto dut = LoadYamlString<BsplineTrajectory<double>>(good);
  EXPECT_EQ(
      dut,
      BsplineTrajectory<double>(
          BsplineBasis<double>(kOrder, knots),
          control_points));
}

const char* const not_enough_control_points = R"""(
  basis: !BsplineBasis
    order: 2
    knots: [0., 1., 1.5, 1.6, 2.]
  control_points:
    -
      - [0.0, 0.1, 0.2]
      - [0.3, 0.4, 0.5]
    -
      - [1.0, 1.1, 1.2]
      - [1.3, 1.4, 1.5]
)""";
GTEST_TEST(BsplineTrajectorySerializeTests, NotEnoughControlPointsTest) {
    DRAKE_EXPECT_THROWS_MESSAGE(
      LoadYamlString<BsplineTrajectory<double>>(not_enough_control_points),
      ".*CheckInvariants.*");
}

}  // namespace trajectories
}  // namespace drake
