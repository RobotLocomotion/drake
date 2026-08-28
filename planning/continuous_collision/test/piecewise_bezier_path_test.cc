/* Acceptance tests for the curve module.

Every property test uses a fixed seed so the suite is reproducible and never
flaky. Reference values come from Drake's own trajectory classes, so these
tests check our conversions against an independent implementation rather than
against themselves. */

#include "drake/planning/continuous_collision/piecewise_bezier_path.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/math/bspline_basis.h"
#include "drake/math/knot_vector_type.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::copyable_unique_ptr;
using drake::math::BsplineBasis;
using drake::math::KnotVectorType;
using drake::trajectories::BezierCurve;
using drake::trajectories::BsplineTrajectory;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

constexpr double kTwoPi = 6.2831853071795864769252867665590;

/* A minimal Trajectory<double> subclass that the curve module does not
support; used to exercise the unknown-segment-type error path. */
class UnsupportedTrajectory final : public Trajectory<double> {
 public:
  UnsupportedTrajectory(int rows, double t_start, double t_end)
      : rows_(rows), t_start_(t_start), t_end_(t_end) {}

 private:
  std::unique_ptr<Trajectory<double>> DoClone() const final {
    return std::make_unique<UnsupportedTrajectory>(rows_, t_start_, t_end_);
  }
  Eigen::MatrixXd do_value(const double&) const final {
    return Eigen::MatrixXd::Zero(rows_, 1);
  }
  Eigen::Index do_rows() const final { return rows_; }
  Eigen::Index do_cols() const final { return 1; }
  double do_start_time() const final { return t_start_; }
  double do_end_time() const final { return t_end_; }

  int rows_{};
  double t_start_{};
  double t_end_{};
};

Eigen::MatrixXd RandomMatrix(int rows, int cols, std::mt19937_64* generator) {
  std::uniform_real_distribution<double> distribution(-1.0, 1.0);
  Eigen::MatrixXd result(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      result(i, j) = distribution(*generator);
    }
  }
  return result;
}

/* Independent Bézier evaluation via Drake, for use as ground truth. */
Eigen::VectorXd DrakeBezierValue(const Eigen::MatrixXd& control_points,
                                 double s) {
  return BezierCurve<double>(0.0, 1.0, control_points).value(s);
}

/* Builds a Bézier curve over [t_start, t_end] whose first control point is
`start` and whose remaining control points are random. */
BezierCurve<double> MakeBezierCurve(const Eigen::VectorXd& start, int order,
                                    double t_start, double t_end,
                                    std::mt19937_64* generator) {
  Eigen::MatrixXd control_points =
      RandomMatrix(static_cast<int>(start.size()), order + 1, generator);
  control_points.col(0) = start;
  return BezierCurve<double>(t_start, t_end, control_points);
}

/* Wraps a vector of trajectories into a CompositeTrajectory. */
CompositeTrajectory<double> MakeComposite(
    std::vector<std::unique_ptr<Trajectory<double>>> pieces) {
  std::vector<copyable_unique_ptr<Trajectory<double>>> segments;
  segments.reserve(pieces.size());
  for (auto& piece : pieces) {
    segments.emplace_back(std::move(piece));
  }
  return CompositeTrajectory<double>(std::move(segments));
}

/* Maximum absolute deviation between the path and `trajectory` over
`num_samples` uniformly spaced times covering the whole domain. */
double MaxSampledError(const PiecewiseBezierPath& path,
                       const Trajectory<double>& trajectory, int num_samples) {
  const double t0 = trajectory.start_time();
  const double tf = trajectory.end_time();
  double worst = 0.0;
  for (int i = 0; i < num_samples; ++i) {
    const double t = t0 + (tf - t0) * i / (num_samples - 1.0);
    const Eigen::VectorXd expected = trajectory.value(t);
    const Eigen::VectorXd actual = path.Value(t);
    worst = std::max(worst, (expected - actual).cwiseAbs().maxCoeff());
  }
  return worst;
}

// --------------------------------------------------------------------------
// Bézier evaluation and de Casteljau subdivision.
// --------------------------------------------------------------------------

/* Our de Casteljau evaluation must agree with BezierCurve::value to 1e-12 over
dense samples, for orders 1 through 5. */
GTEST_TEST(BezierEvaluation, MatchesDrakeBezierCurve) {
  std::mt19937_64 generator(1234);
  for (int order = 1; order <= 5; ++order) {
    for (int trial = 0; trial < 5; ++trial) {
      const int num_positions = 1 + (trial % 6);
      const Eigen::MatrixXd control_points =
          RandomMatrix(num_positions, order + 1, &generator);
      const double t_start = -0.75 + 0.4 * trial;
      const double t_end = t_start + 1.0 + 0.3 * trial;
      const BezierCurve<double> curve(t_start, t_end, control_points);

      const PiecewiseBezierPath path =
          PiecewiseBezierPath::FromTrajectory(curve, Options{});
      ASSERT_EQ(path.num_positions(), num_positions);
      ASSERT_EQ(path.segments().size(), 1u);
      EXPECT_EQ(path.start_time(), t_start);
      EXPECT_EQ(path.end_time(), t_end);
      EXPECT_TRUE(
          path.segments()[0].control_points.isApprox(control_points, 0.0));

      constexpr int kNumSamples = 1001;
      for (int i = 0; i < kNumSamples; ++i) {
        const double s = static_cast<double>(i) / (kNumSamples - 1);
        const double t = t_start + s * (t_end - t_start);
        const Eigen::VectorXd expected = curve.value(t);
        EXPECT_LT((path.Value(t) - expected).cwiseAbs().maxCoeff(), 1e-12)
            << "order " << order << " trial " << trial << " t " << t;
        EXPECT_LT((path.EvaluateSegment(0, s) - expected).cwiseAbs().maxCoeff(),
                  1e-12)
            << "order " << order << " trial " << trial << " s " << s;
      }
    }
  }
}

/* Property test: for >= 1000 random curves the two children produced by
splitting at 1/2 reproduce the parent exactly (to 1e-12) on their halves, and
the apex is the parent's midpoint value. */
GTEST_TEST(DeCasteljau, ChildrenReproduceParent) {
  std::mt19937_64 generator(20260826);
  std::uniform_int_distribution<int> rows_distribution(1, 7);
  std::uniform_int_distribution<int> order_distribution(0, 6);
  constexpr int kNumCases = 1000;
  constexpr int kNumSamples = 21;

  Eigen::MatrixXd left;
  Eigen::MatrixXd right;
  Eigen::VectorXd mid;
  double worst = 0.0;
  for (int trial = 0; trial < kNumCases; ++trial) {
    const int num_positions = rows_distribution(generator);
    const int order = order_distribution(generator);
    const Eigen::MatrixXd parent =
        RandomMatrix(num_positions, order + 1, &generator);

    DeCasteljauSplitAtHalf(parent, &left, &right, &mid);
    ASSERT_EQ(left.rows(), num_positions);
    ASSERT_EQ(left.cols(), order + 1);
    ASSERT_EQ(right.rows(), num_positions);
    ASSERT_EQ(right.cols(), order + 1);
    ASSERT_EQ(mid.size(), num_positions);

    // The apex is exactly q(1/2), and the children share the endpoints they
    // must.
    worst = std::max(
        worst, (mid - DrakeBezierValue(parent, 0.5)).cwiseAbs().maxCoeff());
    worst =
        std::max(worst, (left.col(0) - parent.col(0)).cwiseAbs().maxCoeff());
    worst = std::max(
        worst, (right.col(order) - parent.col(order)).cwiseAbs().maxCoeff());
    worst = std::max(worst, (left.col(order) - mid).cwiseAbs().maxCoeff());
    worst = std::max(worst, (right.col(0) - mid).cwiseAbs().maxCoeff());

    for (int i = 0; i < kNumSamples; ++i) {
      const double s = static_cast<double>(i) / (kNumSamples - 1);
      const Eigen::VectorXd expected = DrakeBezierValue(parent, s);
      const Eigen::VectorXd child_value =
          (s <= 0.5) ? DrakeBezierValue(left, 2.0 * s)
                     : DrakeBezierValue(right, 2.0 * s - 1.0);
      worst = std::max(worst, (child_value - expected).cwiseAbs().maxCoeff());
    }
  }
  EXPECT_LT(worst, 1e-12);
}

/* The hot loop pre-sizes its outputs; re-splitting into already-correctly
sized buffers must not allocate at all, so the steady-state recursion the
certifier runs is allocation-free. */
GTEST_TEST(DeCasteljau, PreSizedOutputsDoNotAllocate) {
  std::mt19937_64 generator(7);
  const Eigen::MatrixXd parent = RandomMatrix(6, 4, &generator);
  Eigen::MatrixXd left(6, 4);
  Eigen::MatrixXd right(6, 4);
  Eigen::VectorXd mid(6);
  DeCasteljauSplitAtHalf(parent, &left, &right, &mid);
  // Splitting a child in place into the same buffers is the recursion the
  // certifier runs; it must be allocation-free too.
  const Eigen::MatrixXd child = left;
  {
    drake::test::LimitMalloc guard;
    DeCasteljauSplitAtHalf(parent, &left, &right, &mid);
    DeCasteljauSplitAtHalf(child, &left, &right, &mid);
  }
}

/* Property test: after a random sequence of subdivisions, the node's
control-point box contains every sample of the sub-curve it represents, and is
contained in its parent's box (the two Bézier facts the normalization relies
on). */
GTEST_TEST(DeCasteljau, ControlBoxesContainCurveAfterRandomSubdivision) {
  std::mt19937_64 generator(99991);
  std::uniform_int_distribution<int> rows_distribution(1, 5);
  std::uniform_int_distribution<int> order_distribution(1, 6);
  std::uniform_int_distribution<int> depth_distribution(1, 6);
  std::uniform_int_distribution<int> coin(0, 1);
  constexpr int kNumCases = 1000;
  constexpr int kNumSamples = 41;
  constexpr double kTolerance = 1e-12;

  Eigen::MatrixXd left;
  Eigen::MatrixXd right;
  Eigen::VectorXd mid;
  for (int trial = 0; trial < kNumCases; ++trial) {
    const int num_positions = rows_distribution(generator);
    const int order = order_distribution(generator);
    const Eigen::MatrixXd root =
        RandomMatrix(num_positions, order + 1, &generator);
    const Eigen::VectorXd root_lower = root.rowwise().minCoeff();
    const Eigen::VectorXd root_upper = root.rowwise().maxCoeff();

    Eigen::MatrixXd node = root;
    double a = 0.0;
    double b = 1.0;
    const int depth = depth_distribution(generator);
    for (int level = 0; level < depth; ++level) {
      const Eigen::MatrixXd parent = node;
      const Eigen::VectorXd parent_lower = parent.rowwise().minCoeff();
      const Eigen::VectorXd parent_upper = parent.rowwise().maxCoeff();
      DeCasteljauSplitAtHalf(parent, &left, &right, &mid);
      const double midpoint = 0.5 * (a + b);
      if (coin(generator) == 0) {
        node = left;
        b = midpoint;
      } else {
        node = right;
        a = midpoint;
      }
      // Children are convex combinations of the parent's control points, so
      // each child's box is inside the parent's.
      const Eigen::VectorXd node_lower = node.rowwise().minCoeff();
      const Eigen::VectorXd node_upper = node.rowwise().maxCoeff();
      ASSERT_TRUE(
          ((node_lower.array() >= parent_lower.array() - kTolerance).all()))
          << "trial " << trial;
      ASSERT_TRUE(
          ((node_upper.array() <= parent_upper.array() + kTolerance).all()))
          << "trial " << trial;
    }

    const Eigen::VectorXd node_lower = node.rowwise().minCoeff();
    const Eigen::VectorXd node_upper = node.rowwise().maxCoeff();
    ASSERT_TRUE(
        ((node_lower.array() >= root_lower.array() - kTolerance).all()));
    ASSERT_TRUE(
        ((node_upper.array() <= root_upper.array() + kTolerance).all()));

    for (int i = 0; i < kNumSamples; ++i) {
      const double local = static_cast<double>(i) / (kNumSamples - 1);
      const double global = a + local * (b - a);
      const Eigen::VectorXd from_node = DrakeBezierValue(node, local);
      const Eigen::VectorXd from_root = DrakeBezierValue(root, global);
      // The child exactly represents the sub-curve ...
      ASSERT_LT((from_node - from_root).cwiseAbs().maxCoeff(), 1e-12)
          << "trial " << trial << " local " << local;
      // ... and the sub-curve lives in the child's control box.
      ASSERT_TRUE(
          ((from_node.array() >= node_lower.array() - kTolerance).all()))
          << "trial " << trial;
      ASSERT_TRUE(
          ((from_node.array() <= node_upper.array() + kTolerance).all()))
          << "trial " << trial;
    }
  }
}

// --------------------------------------------------------------------------
// B-spline → Bézier.
// --------------------------------------------------------------------------

BsplineTrajectory<double> MakeBsplineFromBasis(
    const BsplineBasis<double>& basis, int num_positions,
    std::mt19937_64* generator) {
  std::vector<Eigen::MatrixXd> control_points;
  control_points.reserve(basis.num_basis_functions());
  for (int i = 0; i < basis.num_basis_functions(); ++i) {
    control_points.push_back(RandomMatrix(num_positions, 1, generator));
  }
  return BsplineTrajectory<double>(basis, std::move(control_points));
}

/* Shared checker: the conversion must reproduce the B-spline to 1e-10 over
>= 1e4 dense samples, the segments must tile the domain, and there must be
`expected_segments` of them (or any number, when that is 0). */
void CheckBsplineEquivalence(const BsplineTrajectory<double>& bspline,
                             int expected_segments = 0) {
  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(bspline, Options{});
  EXPECT_EQ(path.num_positions(), bspline.rows());
  EXPECT_NEAR(path.start_time(), bspline.start_time(), 1e-14);
  EXPECT_NEAR(path.end_time(), bspline.end_time(), 1e-14);
  for (const BezierSegment& segment : path.segments()) {
    // Full interior multiplicity => every segment has exactly `order` control
    // points, i.e. the degree of the source spline.
    EXPECT_EQ(segment.control_points.cols(), bspline.basis().order());
  }
  if (expected_segments > 0) {
    EXPECT_EQ(static_cast<int>(path.segments().size()), expected_segments);
  }
  EXPECT_LT(MaxSampledError(path, bspline, 10001), 1e-10);
}

GTEST_TEST(BsplineConversion, ClampedUniformOrders2To6) {
  std::mt19937_64 generator(4242);
  for (int order = 2; order <= 6; ++order) {
    SCOPED_TRACE("order " + std::to_string(order));
    const int num_basis_functions = order + 4;
    // A clamped uniform basis has num_basis_functions - order + 1 spans.
    CheckBsplineEquivalence(
        MakeBsplineFromBasis(
            BsplineBasis<double>(order, num_basis_functions,
                                 KnotVectorType::kClampedUniform, 0.0, 3.0),
            3, &generator),
        num_basis_functions - order + 1);
  }
}

GTEST_TEST(BsplineConversion, NonUniformKnots) {
  std::mt19937_64 generator(515151);
  for (int order = 2; order <= 6; ++order) {
    SCOPED_TRACE("order " + std::to_string(order));
    // Clamped, but with irregular interior spacing.
    std::vector<double> knots(order, 0.0);
    for (double interior : {0.13, 0.29, 0.31, 1.70, 2.55}) {
      knots.push_back(interior);
    }
    knots.insert(knots.end(), order, 3.0);
    CheckBsplineEquivalence(
        MakeBsplineFromBasis(BsplineBasis<double>(order, knots), 4, &generator),
        6);
  }
}

GTEST_TEST(BsplineConversion, RepeatedInteriorKnots) {
  std::mt19937_64 generator(606060);
  // Order 4 (cubic): interior knot 1.0 with multiplicity 2 (C1 there) and
  // interior knot 2.0 with multiplicity 3 (C0 there, the extreme case that
  // still passes junction validation). Nonempty spans: [0,1], [1,2], [2,3],
  // [3,3.5], [3.5,4].
  const std::vector<double> knots{0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0,
                                  2.0, 3.0, 3.5, 4.0, 4.0, 4.0, 4.0};
  CheckBsplineEquivalence(
      MakeBsplineFromBasis(BsplineBasis<double>(4, knots), 2, &generator), 5);
}

/* The representation KinematicTrajectoryOptimization emits: clamped uniform,
order 4, one control point per decision-variable column. */
GTEST_TEST(BsplineConversion, KinematicTrajectoryOptimizationStyle) {
  std::mt19937_64 generator(777);
  CheckBsplineEquivalence(
      MakeBsplineFromBasis(
          BsplineBasis<double>(4, 10, KnotVectorType::kClampedUniform, 0.0,
                               5.0),
          7, &generator),
      7);
}

/* General (unclamped) knot vectors are supported too: the domain endpoints are
raised to full multiplicity by the same insertion pass. */
GTEST_TEST(BsplineConversion, UnclampedUniformKnots) {
  std::mt19937_64 generator(31337);
  for (int order = 2; order <= 5; ++order) {
    SCOPED_TRACE("order " + std::to_string(order));
    CheckBsplineEquivalence(MakeBsplineFromBasis(
        BsplineBasis<double>(order, order + 5, KnotVectorType::kUniform, 0.0,
                             2.0),
        3, &generator));
  }
}

GTEST_TEST(BsplineConversion, SegmentTimesMatchKnotSpans) {
  std::mt19937_64 generator(24680);
  const std::vector<double> knots{0.0, 0.0, 0.0, 0.5, 1.25, 2.0, 2.0, 2.0};
  const PiecewiseBezierPath path = PiecewiseBezierPath::FromTrajectory(
      MakeBsplineFromBasis(BsplineBasis<double>(3, knots), 2, &generator),
      Options{});
  ASSERT_EQ(path.segments().size(), 3u);
  const std::vector<double> expected{0.0, 0.5, 1.25, 2.0};
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(path.segments()[i].t_start, expected[i]);
    EXPECT_EQ(path.segments()[i].t_end, expected[i + 1]);
  }
}

GTEST_TEST(BsplineConversion, MatrixValuedThrows) {
  std::vector<Eigen::MatrixXd> control_points(6, Eigen::MatrixXd::Zero(2, 2));
  const BsplineTrajectory<double> bspline(
      BsplineBasis<double>(3, 6, KnotVectorType::kClampedUniform, 0.0, 1.0),
      control_points);
  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromTrajectory(bspline, Options{}),
      "[\\s\\S]*column-vector-valued[\\s\\S]*");
}

// --------------------------------------------------------------------------
// PiecewisePolynomial → Bernstein.
// --------------------------------------------------------------------------

GTEST_TEST(PiecewisePolynomialConversion, FirstOrderHold) {
  std::mt19937_64 generator(11235);
  const Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(6, 0.0, 2.5);
  const Eigen::MatrixXd samples = RandomMatrix(4, 6, &generator);
  const PiecewisePolynomial<double> pp =
      PiecewisePolynomial<double>::FirstOrderHold(times, samples);

  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(pp, Options{});
  ASSERT_EQ(path.segments().size(), 5u);
  for (int k = 0; k < 5; ++k) {
    // A first-order hold is exactly an order-1 Bézier per segment, whose
    // control points are the waypoints themselves.
    ASSERT_EQ(path.segments()[k].control_points.cols(), 2);
    EXPECT_LT((path.segments()[k].control_points.col(0) - samples.col(k))
                  .cwiseAbs()
                  .maxCoeff(),
              1e-14);
    EXPECT_LT((path.segments()[k].control_points.col(1) - samples.col(k + 1))
                  .cwiseAbs()
                  .maxCoeff(),
              1e-14);
  }
  EXPECT_LT(MaxSampledError(path, pp, 10001), 1e-10);
}

GTEST_TEST(PiecewisePolynomialConversion, CubicSplines) {
  std::mt19937_64 generator(626262);
  const Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(7, -1.0, 3.0);
  const Eigen::MatrixXd samples = RandomMatrix(3, 7, &generator);

  const PiecewisePolynomial<double> continuous_second =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          times, samples);
  const PiecewiseBezierPath path_a =
      PiecewiseBezierPath::FromTrajectory(continuous_second, Options{});
  EXPECT_EQ(path_a.segments().size(), 6u);
  for (const BezierSegment& segment : path_a.segments()) {
    EXPECT_EQ(segment.control_points.cols(), 4);
  }
  EXPECT_LT(MaxSampledError(path_a, continuous_second, 10001), 1e-10);

  const PiecewisePolynomial<double> shape_preserving =
      PiecewisePolynomial<double>::CubicShapePreserving(times, samples);
  const PiecewiseBezierPath path_b =
      PiecewiseBezierPath::FromTrajectory(shape_preserving, Options{});
  EXPECT_LT(MaxSampledError(path_b, shape_preserving, 10001), 1e-10);
}

/* A single high-degree polynomial segment, from degree 1 up to the default cap
and one past it. */
GTEST_TEST(PiecewisePolynomialConversion, LagrangeUpToDegreeCapAndBeyond) {
  const Options options;
  ASSERT_EQ(options.max_conversion_degree, 10);
  for (int degree = 1; degree <= options.max_conversion_degree + 1; ++degree) {
    SCOPED_TRACE("degree " + std::to_string(degree));
    const int num_points = degree + 1;
    Eigen::VectorXd times(num_points);
    Eigen::MatrixXd samples(2, num_points);
    for (int i = 0; i < num_points; ++i) {
      // A non-unit segment duration: the monomial coefficients must be
      // rescaled by (t_end - t_start)^a before the change of basis.
      times[i] = 0.3 + 1.7 * static_cast<double>(i) / degree;
      samples(0, i) = std::sin(3.0 * times[i]);
      samples(1, i) = std::cos(2.0 * times[i]) - 0.25 * times[i];
    }
    const PiecewisePolynomial<double> pp =
        PiecewisePolynomial<double>::LagrangeInterpolatingPolynomial(times,
                                                                     samples);
    Options relaxed = options;
    if (degree > options.max_conversion_degree) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          PiecewiseBezierPath::FromTrajectory(pp, options),
          "[\\s\\S]*max_conversion_degree[\\s\\S]*");
      // Raising the cap is the documented escape hatch.
      relaxed.max_conversion_degree = degree;
    }
    const PiecewiseBezierPath path =
        PiecewiseBezierPath::FromTrajectory(pp, relaxed);
    ASSERT_EQ(path.segments().size(), 1u);
    EXPECT_EQ(path.segments()[0].control_points.cols(), degree + 1);
    EXPECT_LT(MaxSampledError(path, pp, 10001), 1e-10);
  }
}

GTEST_TEST(PiecewisePolynomialConversion, MatrixValuedThrows) {
  std::vector<Eigen::MatrixXd> samples;
  samples.push_back(Eigen::MatrixXd::Zero(2, 2));
  samples.push_back(Eigen::MatrixXd::Ones(2, 2));
  const std::vector<double> times{0.0, 1.0};
  const PiecewisePolynomial<double> pp =
      PiecewisePolynomial<double>::FirstOrderHold(times, samples);
  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromTrajectory(pp, Options{}),
      "[\\s\\S]*column-vector-valued[\\s\\S]*");
}

// --------------------------------------------------------------------------
// Junction (C0) validation.
// --------------------------------------------------------------------------

/* Builds a two-segment composite whose second segment starts at the first
segment's endpoint plus `offset`. */
CompositeTrajectory<double> MakeJunctionCase(const Eigen::VectorXd& offset,
                                             std::mt19937_64* generator) {
  const int num_positions = static_cast<int>(offset.size());
  Eigen::MatrixXd first = RandomMatrix(num_positions, 4, generator);
  Eigen::MatrixXd second = RandomMatrix(num_positions, 3, generator);
  second.col(0) = first.col(3) + offset;
  std::vector<std::unique_ptr<Trajectory<double>>> pieces;
  pieces.push_back(std::make_unique<BezierCurve<double>>(0.0, 1.0, first));
  pieces.push_back(std::make_unique<BezierCurve<double>>(1.0, 2.5, second));
  return MakeComposite(std::move(pieces));
}

GTEST_TEST(JunctionValidation, InjectedDiscontinuityThrows) {
  std::mt19937_64 generator(90210);
  Eigen::VectorXd offset = Eigen::VectorXd::Zero(3);
  offset[1] = 1e-3;
  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromTrajectory(MakeJunctionCase(offset, &generator),
                                          Options{}),
      "[\\s\\S]*C0 discontinuity[\\s\\S]*coordinate 1[\\s\\S]*");

  // A gap just under the tolerance is accepted.
  Eigen::VectorXd tiny = Eigen::VectorXd::Zero(3);
  tiny[2] = 9e-8;
  EXPECT_NO_THROW(PiecewiseBezierPath::FromTrajectory(
      MakeJunctionCase(tiny, &generator), Options{}));
}

GTEST_TEST(JunctionValidation, TwoPiOffsetAcceptedOnlyWhenDeclaredRevolute) {
  std::mt19937_64 generator(1357);
  Eigen::VectorXd offset = Eigen::VectorXd::Zero(3);
  offset[1] = kTwoPi;
  const CompositeTrajectory<double> trajectory =
      MakeJunctionCase(offset, &generator);

  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromTrajectory(trajectory, Options{}),
      "[\\s\\S]*C0 discontinuity[\\s\\S]*");

  // Declaring the *wrong* coordinate does not help.
  Options wrong;
  wrong.continuous_revolute_indices = {0, 2};
  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromTrajectory(trajectory, wrong),
      "[\\s\\S]*C0 discontinuity[\\s\\S]*");

  Options right;
  right.continuous_revolute_indices = {1};
  EXPECT_NO_THROW(PiecewiseBezierPath::FromTrajectory(trajectory, right));

  // Any integer multiple of 2π is fine ...
  Eigen::VectorXd big_offset = Eigen::VectorXd::Zero(3);
  big_offset[1] = -3.0 * kTwoPi;
  EXPECT_NO_THROW(PiecewiseBezierPath::FromTrajectory(
      MakeJunctionCase(big_offset, &generator), right));

  // ... but an offset that is not one is still a discontinuity.
  Eigen::VectorXd off_by = Eigen::VectorXd::Zero(3);
  off_by[1] = kTwoPi + 1e-3;
  DRAKE_EXPECT_THROWS_MESSAGE(PiecewiseBezierPath::FromTrajectory(
                                  MakeJunctionCase(off_by, &generator), right),
                              "[\\s\\S]*C0 discontinuity[\\s\\S]*");
}

/* Forward kinematics is 2π-periodic, so a legitimate 2πk junction offset must
be left exactly as it is: the segments are NOT re-aligned. */
GTEST_TEST(JunctionValidation,
           ControlPointsAreNotRealignedAcrossTwoPiJunction) {
  std::mt19937_64 generator(864213);
  Eigen::VectorXd offset = Eigen::VectorXd::Zero(2);
  offset[0] = kTwoPi;
  const CompositeTrajectory<double> trajectory =
      MakeJunctionCase(offset, &generator);
  Options options;
  options.continuous_revolute_indices = {0};
  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(trajectory, options);
  ASSERT_EQ(path.segments().size(), 2u);

  const Eigen::MatrixXd& first = path.segments()[0].control_points;
  const Eigen::MatrixXd& second = path.segments()[1].control_points;
  EXPECT_NEAR(second(0, 0) - first(0, first.cols() - 1), kTwoPi, 1e-14);
  // The path reproduces the source trajectory verbatim on both sides.
  EXPECT_LT(MaxSampledError(path, trajectory, 501), 1e-12);
  // The two sides of the junction are distinct representatives of the same
  // configuration; Value() reports the later segment's, matching
  // PiecewiseTrajectory::get_segment_index().
  EXPECT_NEAR(path.Value(1.0)[0], second(0, 0), 1e-14);
  EXPECT_NEAR(path.EvaluateSegment(0, 1.0)[0], first(0, first.cols() - 1),
              1e-14);
  // The global control box therefore spans the 2π jump, as intended.
  EXPECT_GT(path.global_upper_bound()[0] - path.global_lower_bound()[0], 5.0);
}

GTEST_TEST(JunctionValidation, OutOfRangeRevoluteIndexThrows) {
  Eigen::MatrixXd waypoints(2, 3);
  waypoints << 0.0, 1.0, 2.0, 0.0, 0.0, 0.0;
  Options options;
  options.continuous_revolute_indices = {2};
  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromWaypoints(waypoints, options),
      "[\\s\\S]*continuous_revolute_indices[\\s\\S]*");
}

/* A zero-order hold genuinely teleports at every break; certifying it
per-segment would silently skip the jumps, so it is rejected. */
GTEST_TEST(JunctionValidation, ZeroOrderHoldIsRejected) {
  const Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(4, 0.0, 3.0);
  Eigen::MatrixXd samples(2, 4);
  samples << 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0;
  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromTrajectory(
          PiecewisePolynomial<double>::ZeroOrderHold(times, samples),
          Options{}),
      "[\\s\\S]*C0 discontinuity[\\s\\S]*");
}

// --------------------------------------------------------------------------
// Metadata: global control box and constant coordinates.
// --------------------------------------------------------------------------

GTEST_TEST(Metadata, GlobalControlBox) {
  std::mt19937_64 generator(5150);
  std::vector<std::unique_ptr<Trajectory<double>>> pieces;
  Eigen::VectorXd start = RandomMatrix(3, 1, &generator).col(0);
  double t = 0.0;
  std::vector<Eigen::MatrixXd> all_control_points;
  for (int i = 0; i < 3; ++i) {
    BezierCurve<double> curve =
        MakeBezierCurve(start, 3, t, t + 1.0, &generator);
    all_control_points.push_back(curve.control_points());
    start = curve.control_points().col(3);
    t += 1.0;
    pieces.push_back(std::make_unique<BezierCurve<double>>(curve));
  }
  const CompositeTrajectory<double> trajectory =
      MakeComposite(std::move(pieces));
  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(trajectory, Options{});

  Eigen::VectorXd expected_lower =
      Eigen::VectorXd::Constant(3, std::numeric_limits<double>::infinity());
  Eigen::VectorXd expected_upper =
      Eigen::VectorXd::Constant(3, -std::numeric_limits<double>::infinity());
  for (const Eigen::MatrixXd& cps : all_control_points) {
    expected_lower = expected_lower.cwiseMin(cps.rowwise().minCoeff());
    expected_upper = expected_upper.cwiseMax(cps.rowwise().maxCoeff());
  }
  EXPECT_TRUE(path.global_lower_bound().isApprox(expected_lower, 0.0));
  EXPECT_TRUE(path.global_upper_bound().isApprox(expected_upper, 0.0));

  // The convex-hull property: dense samples stay inside the global box.
  for (int i = 0; i <= 1000; ++i) {
    const Eigen::VectorXd q = path.Value(3.0 * i / 1000.0);
    EXPECT_TRUE(
        ((q.array() >= path.global_lower_bound().array() - 1e-12).all()));
    EXPECT_TRUE(
        ((q.array() <= path.global_upper_bound().array() + 1e-12).all()));
  }
}

GTEST_TEST(Metadata, ConstantCoordinateFlags) {
  Eigen::MatrixXd waypoints(4, 4);
  // Coordinate 0 moves; 1 is exactly constant; 2 wobbles below the tolerance;
  // 3 moves by just above the tolerance.
  waypoints.row(0) << 0.0, 0.5, -0.25, 1.0;
  waypoints.row(1) << 2.0, 2.0, 2.0, 2.0;
  waypoints.row(2) << 1.0, 1.0 + 5e-9, 1.0 - 2e-8, 1.0;
  waypoints.row(3) << 0.0, 0.0, 2e-7, 0.0;

  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromWaypoints(waypoints, Options{});
  ASSERT_EQ(path.constant_coordinates().size(), 4u);
  EXPECT_FALSE(path.constant_coordinates()[0]);
  EXPECT_TRUE(path.constant_coordinates()[1]);
  EXPECT_TRUE(path.constant_coordinates()[2]);
  EXPECT_FALSE(path.constant_coordinates()[3]);

  // A looser tolerance sweeps coordinate 3 in as well.
  Options loose;
  loose.continuity_tolerance = 1e-5;
  const PiecewiseBezierPath loose_path =
      PiecewiseBezierPath::FromWaypoints(waypoints, loose);
  EXPECT_TRUE(loose_path.constant_coordinates()[3]);
  EXPECT_FALSE(loose_path.constant_coordinates()[0]);
}

// --------------------------------------------------------------------------
// CompositeTrajectory handling (the GcsTrajectoryOptimization output shape).
// --------------------------------------------------------------------------

GTEST_TEST(Composite, BezierSegmentsRoundTrip) {
  std::mt19937_64 generator(31415);
  const int num_positions = 5;
  const std::vector<int> orders{3, 5, 2, 1};
  const std::vector<double> breaks{0.0, 0.4, 1.9, 2.0, 4.25};

  Eigen::VectorXd start = RandomMatrix(num_positions, 1, &generator).col(0);
  std::vector<std::unique_ptr<Trajectory<double>>> pieces;
  for (std::size_t i = 0; i < orders.size(); ++i) {
    BezierCurve<double> curve =
        MakeBezierCurve(start, orders[i], breaks[i], breaks[i + 1], &generator);
    start = curve.control_points().col(orders[i]);
    pieces.push_back(std::make_unique<BezierCurve<double>>(curve));
  }
  const CompositeTrajectory<double> trajectory =
      MakeComposite(std::move(pieces));

  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(trajectory, Options{});
  ASSERT_EQ(path.segments().size(), orders.size());
  for (std::size_t i = 0; i < orders.size(); ++i) {
    EXPECT_EQ(path.segments()[i].control_points.cols(), orders[i] + 1);
    EXPECT_EQ(path.segments()[i].t_start, breaks[i]);
    EXPECT_EQ(path.segments()[i].t_end, breaks[i + 1]);
  }
  EXPECT_EQ(path.start_time(), breaks.front());
  EXPECT_EQ(path.end_time(), breaks.back());
  EXPECT_LT(MaxSampledError(path, trajectory, 10001), 1e-12);
}

GTEST_TEST(Composite, NestedCompositeRecursion) {
  std::mt19937_64 generator(2718);
  const int num_positions = 2;
  Eigen::VectorXd start = RandomMatrix(num_positions, 1, &generator).col(0);

  BezierCurve<double> a = MakeBezierCurve(start, 2, 0.0, 1.0, &generator);
  start = a.control_points().col(2);
  BezierCurve<double> b = MakeBezierCurve(start, 3, 1.0, 2.0, &generator);
  start = b.control_points().col(3);
  BezierCurve<double> c = MakeBezierCurve(start, 1, 2.0, 3.0, &generator);

  std::vector<std::unique_ptr<Trajectory<double>>> inner_pieces;
  inner_pieces.push_back(std::make_unique<BezierCurve<double>>(b));
  inner_pieces.push_back(std::make_unique<BezierCurve<double>>(c));
  auto inner = std::make_unique<CompositeTrajectory<double>>(
      MakeComposite(std::move(inner_pieces)));

  std::vector<std::unique_ptr<Trajectory<double>>> outer_pieces;
  outer_pieces.push_back(std::make_unique<BezierCurve<double>>(a));
  outer_pieces.push_back(std::move(inner));
  const CompositeTrajectory<double> trajectory =
      MakeComposite(std::move(outer_pieces));

  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(trajectory, Options{});
  ASSERT_EQ(path.segments().size(), 3u);
  EXPECT_EQ(path.segments()[0].control_points.cols(), 3);
  EXPECT_EQ(path.segments()[1].control_points.cols(), 4);
  EXPECT_EQ(path.segments()[2].control_points.cols(), 2);
  EXPECT_LT(MaxSampledError(path, trajectory, 5001), 1e-12);
}

/* A CompositeTrajectory whose segments are B-splines and PiecewisePolynomials
recurses through the same rules. */
GTEST_TEST(Composite, MixedSegmentTypes) {
  std::mt19937_64 generator(11111);
  const int num_positions = 2;

  const BsplineBasis<double> basis(4, 8, KnotVectorType::kClampedUniform, 0.0,
                                   1.0);
  BsplineTrajectory<double> bspline =
      MakeBsplineFromBasis(basis, num_positions, &generator);

  // Continue with a first-order hold that starts exactly where the B-spline
  // ends, so the junction is C0.
  const Eigen::VectorXd end_value = bspline.FinalValue();
  Eigen::MatrixXd samples(num_positions, 3);
  samples.col(0) = end_value;
  samples.col(1) = end_value + Eigen::VectorXd::Constant(num_positions, 0.3);
  samples.col(2) = end_value - Eigen::VectorXd::Constant(num_positions, 0.1);
  Eigen::VectorXd times(3);
  times << 1.0, 1.5, 2.0;
  const PiecewisePolynomial<double> pp =
      PiecewisePolynomial<double>::FirstOrderHold(times, samples);

  std::vector<std::unique_ptr<Trajectory<double>>> pieces;
  pieces.push_back(std::make_unique<BsplineTrajectory<double>>(bspline));
  pieces.push_back(std::make_unique<PiecewisePolynomial<double>>(pp));
  const CompositeTrajectory<double> trajectory =
      MakeComposite(std::move(pieces));

  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(trajectory, Options{});
  // 5 Bézier segments from the clamped order-4 B-spline plus 2 from the FOH.
  EXPECT_EQ(path.segments().size(), 7u);
  EXPECT_LT(MaxSampledError(path, trajectory, 10001), 1e-10);
}

GTEST_TEST(Composite, UnknownSegmentTypeThrowsWithIndexAndTypeName) {
  std::mt19937_64 generator(4321);
  const int num_positions = 3;
  Eigen::VectorXd start = RandomMatrix(num_positions, 1, &generator).col(0);
  BezierCurve<double> first = MakeBezierCurve(start, 2, 0.0, 1.0, &generator);

  std::vector<std::unique_ptr<Trajectory<double>>> pieces;
  pieces.push_back(std::make_unique<BezierCurve<double>>(first));
  pieces.push_back(
      std::make_unique<UnsupportedTrajectory>(num_positions, 1.0, 2.0));
  const CompositeTrajectory<double> trajectory =
      MakeComposite(std::move(pieces));

  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromTrajectory(trajectory, Options{}),
      "[\\s\\S]*UnsupportedTrajectory[\\s\\S]*segment index 1[\\s\\S]*");

  // At the top level the offending segment index is 0.
  const UnsupportedTrajectory bare(num_positions, 0.0, 1.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromTrajectory(bare, Options{}),
      "[\\s\\S]*segment index 0[\\s\\S]*");
}

// --------------------------------------------------------------------------
// Waypoints and evaluation domain handling.
// --------------------------------------------------------------------------

GTEST_TEST(Waypoints, OrderOneSegmentsAreExact) {
  std::mt19937_64 generator(19191);
  const int num_positions = 6;
  const int num_waypoints = 5;
  const Eigen::MatrixXd waypoints =
      RandomMatrix(num_positions, num_waypoints, &generator);
  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromWaypoints(waypoints, Options{});

  ASSERT_EQ(path.num_positions(), num_positions);
  ASSERT_EQ(static_cast<int>(path.segments().size()), num_waypoints - 1);
  EXPECT_EQ(path.start_time(), 0.0);
  EXPECT_EQ(path.end_time(), num_waypoints - 1);
  for (int k = 0; k + 1 < num_waypoints; ++k) {
    const BezierSegment& segment = path.segments()[k];
    EXPECT_EQ(segment.t_start, k);
    EXPECT_EQ(segment.t_end, k + 1);
    ASSERT_EQ(segment.control_points.cols(), 2);
    EXPECT_TRUE(segment.control_points.col(0).isApprox(waypoints.col(k), 0.0));
    EXPECT_TRUE(
        segment.control_points.col(1).isApprox(waypoints.col(k + 1), 0.0));
    // Straight-line interpolation is exact at every parameter.
    for (int i = 0; i <= 100; ++i) {
      const double s = i / 100.0;
      const Eigen::VectorXd expected =
          (1.0 - s) * waypoints.col(k) + s * waypoints.col(k + 1);
      EXPECT_LT((path.Value(k + s) - expected).cwiseAbs().maxCoeff(), 1e-15);
      EXPECT_LT((path.EvaluateSegment(k, s) - expected).cwiseAbs().maxCoeff(),
                1e-15);
    }
  }
}

GTEST_TEST(Waypoints, TooFewWaypointsThrows) {
  DRAKE_EXPECT_THROWS_MESSAGE(PiecewiseBezierPath::FromWaypoints(
                                  Eigen::MatrixXd::Zero(3, 1), Options{}),
                              "[\\s\\S]*at least 2 waypoints[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      PiecewiseBezierPath::FromWaypoints(Eigen::MatrixXd(0, 4), Options{}),
      "[\\s\\S]*zero rows[\\s\\S]*");
}

GTEST_TEST(Evaluation, DomainEdgesClampAndOutsideThrows) {
  Eigen::MatrixXd waypoints(2, 3);
  waypoints << 0.0, 1.0, 3.0, -1.0, 0.0, 1.0;
  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromWaypoints(waypoints, Options{});

  EXPECT_TRUE(path.Value(0.0).isApprox(waypoints.col(0), 0.0));
  EXPECT_TRUE(path.Value(2.0).isApprox(waypoints.col(2), 0.0));
  // Within the clamping slack.
  EXPECT_NO_THROW(path.Value(2.0 + 1e-13));
  EXPECT_TRUE(path.Value(-1e-13).isApprox(waypoints.col(0), 0.0));
  EXPECT_NO_THROW(path.EvaluateSegment(0, 1.0 + 1e-13));

  for (const double t : {-1e-3, 2.5}) {
    DRAKE_EXPECT_THROWS_MESSAGE(path.Value(t),
                                "[\\s\\S]*outside the path's domain[\\s\\S]*");
  }
  for (const double bad_s : {-0.5, 1.5}) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        path.EvaluateSegment(0, bad_s),
        "[\\s\\S]*outside the segment's domain[\\s\\S]*");
  }
  for (const int k : {-1, 2}) {
    DRAKE_EXPECT_THROWS_MESSAGE(path.EvaluateSegment(k, 0.5),
                                "[\\s\\S]*out of range[\\s\\S]*");
  }
}

/* Segment-time bookkeeping contract for downstream modules: at a junction time
shared by two segments, Value() evaluates the LATER segment, exactly as
drake::trajectories::PiecewiseTrajectory::get_segment_index() does; at the
domain end it evaluates the last segment. */
GTEST_TEST(Evaluation, JunctionTimeSelectsTheLaterSegment) {
  Eigen::MatrixXd waypoints(1, 4);
  waypoints << 0.0, 1.0, 3.0, 6.0;
  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromWaypoints(waypoints, Options{});
  ASSERT_EQ(path.segments().size(), 3u);
  // Segment k spans [k, k+1]; at t = 1 both segment 0's end and segment 1's
  // start are the value 1.0, and the lookup lands on segment 1.
  EXPECT_EQ(path.Value(0.0)[0], 0.0);
  EXPECT_EQ(path.Value(1.0)[0], 1.0);
  EXPECT_EQ(path.Value(2.0)[0], 3.0);
  EXPECT_EQ(path.Value(3.0)[0], 6.0);
  // Interior samples resolve to the expected segment.
  EXPECT_NEAR(path.Value(1.5)[0], 2.0, 1e-15);
  EXPECT_NEAR(path.Value(2.5)[0], 4.5, 1e-15);
}

/* Junction times are shared by two segments; Value() must agree with the source
trajectory there regardless of which side the lookup lands on. */
GTEST_TEST(Evaluation, JunctionTimesAreConsistent) {
  std::mt19937_64 generator(606);
  const int num_positions = 3;
  Eigen::VectorXd start = RandomMatrix(num_positions, 1, &generator).col(0);
  std::vector<std::unique_ptr<Trajectory<double>>> pieces;
  double t = 0.0;
  for (int i = 0; i < 4; ++i) {
    BezierCurve<double> curve =
        MakeBezierCurve(start, 3, t, t + 0.75, &generator);
    start = curve.control_points().col(3);
    t += 0.75;
    pieces.push_back(std::make_unique<BezierCurve<double>>(curve));
  }
  const CompositeTrajectory<double> trajectory =
      MakeComposite(std::move(pieces));
  const PiecewiseBezierPath path =
      PiecewiseBezierPath::FromTrajectory(trajectory, Options{});
  for (int i = 0; i < 4; ++i) {
    const double junction = 0.75 * i;
    EXPECT_LT((path.Value(junction) - trajectory.value(junction))
                  .cwiseAbs()
                  .maxCoeff(),
              1e-13)
        << "junction " << junction;
  }
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
