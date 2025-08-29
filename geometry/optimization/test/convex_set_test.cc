#include "drake/geometry/optimization/convex_set.h"

#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

// N.B. See also convex_set_limit_malloc_test for additional unit test cases.

GTEST_TEST(ConvexSetsTest, BasicTest) {
  ConvexSets sets;

  const ConvexSet& a = *sets.emplace_back(Point(Vector2d{1., 2.}));
  const Vector3d b_point{3., 4., 5.};
  std::unique_ptr<Point> b_original = std::make_unique<Point>(b_point);
  Point* b_pointer = b_original.get();
  const ConvexSet& b = *sets.emplace_back(std::move(b_original));

  EXPECT_EQ(a.ambient_dimension(), 2);
  EXPECT_EQ(b.ambient_dimension(), 3);

  EXPECT_EQ(sets.size(), 2);
  EXPECT_EQ(sets[0]->ambient_dimension(), 2);
  EXPECT_EQ(sets[1]->ambient_dimension(), 3);

  // Confirm that a const reference to the container provides only const access
  // to the set.
  const ConvexSets& const_sets = sets;
  static_assert(std::is_same_v<const ConvexSet&, decltype(*const_sets[0])>);

  // Confirm that I can move sets without copying the underlying data.
  // Note: jwnimmer-tri argued that this should not be a strong requirement.
  // Derived ConvexSets with substantial memory footprint could implement
  // Clone() using a shared_ptr on their data.  It may be fine to remove this if
  // a different pattern requires it.
  ConvexSets moved = std::move(sets);
  EXPECT_EQ(moved.size(), 2);
  EXPECT_EQ(moved[0]->ambient_dimension(), 2);
  EXPECT_EQ(moved[1]->ambient_dimension(), 3);
  EXPECT_TRUE(moved[1]->PointInSet(b_point));
  const Vector3d new_point{6., 7., 8.};
  EXPECT_FALSE(moved[1]->PointInSet(new_point));
  b_pointer->set_x(new_point);
  EXPECT_TRUE(moved[1]->PointInSet(new_point));
}

GTEST_TEST(ConvexSetTest, IntersectsWithTest) {
  /* Test that IntersectsWith() yields correct results for the following
  arrangement of boxes:
     5                ┏━━━━━━━━━┓
                      ┃      C  ┃
     4      ┏━━━━━━━━━┃━━━━┓    ┃
            ┃         ┃    ┃    ┃
     3      ┃         ┗━━━━━━━━━┛
            ┃      B       ┃
     2 ┏━━━━┃━━━━┓         ┃
       ┃    ┃    ┃         ┃
     1 ┃    ┗━━━━━━━━━━━━━━┛
       ┃  A      ┃
     0 ┗━━━━━━━━━┛
       0    1    2    3    4    5
  */
  HPolyhedron set_A = HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(2, 2));
  HPolyhedron set_B = HPolyhedron::MakeBox(Vector2d(1, 1), Vector2d(4, 4));
  HPolyhedron set_C = HPolyhedron::MakeBox(Vector2d(3, 3), Vector2d(5, 5));
  EXPECT_TRUE(set_A.IntersectsWith(set_B));
  EXPECT_TRUE(set_B.IntersectsWith(set_A));
  EXPECT_TRUE(set_B.IntersectsWith(set_C));
  EXPECT_TRUE(set_C.IntersectsWith(set_B));
  EXPECT_FALSE(set_A.IntersectsWith(set_C));
  EXPECT_FALSE(set_C.IntersectsWith(set_A));
}

GTEST_TEST(MakeConvexSetsTest, Basic) {
  HPolyhedron box = HPolyhedron::MakeUnitBox(2);
  ConvexSets sets =
      MakeConvexSets(box, box.Clone(), Point(Vector3d(1.0, 2.0, 3.0)));

  EXPECT_EQ(sets.size(), 3);
  EXPECT_EQ(sets[0]->ambient_dimension(), 2);
  EXPECT_EQ(sets[1]->ambient_dimension(), 2);
  EXPECT_EQ(sets[2]->ambient_dimension(), 3);
}

// A mutable lvalue reference is copied, not moved.
GTEST_TEST(MakeConvexSetsTest, MutableLvalueReference) {
  const HPolyhedron box = HPolyhedron::MakeUnitBox(2);
  std::unique_ptr<ConvexSet> box_clone = box.Clone();
  ConvexSets sets = MakeConvexSets(box_clone);
  EXPECT_EQ(sets.size(), 1);
  EXPECT_NE(box_clone.get(), nullptr);
}

// Minimum implementation of a ConvexSet.
class DummyVolumeSet : public ConvexSet {
 public:
  explicit DummyVolumeSet(int dim, bool can_calc_volume)
      : ConvexSet(dim, can_calc_volume) {}

 protected:
  std::unique_ptr<ConvexSet> DoClone() const override { return nullptr; }
  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>&,
                    double) const override {
    return false;
  }
  std::pair<VectorX<symbolic::Variable>,
            std::vector<solvers::Binding<solvers::Constraint>>>
  DoAddPointInSetConstraints(
      solvers::MathematicalProgram*,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&)
      const override {
    return {{}, {}};
  }
  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram*,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&,
      const symbolic::Variable&) const override {
    return {};
  }
  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram*, const Eigen::Ref<const Eigen::MatrixXd>&,
      const Eigen::Ref<const Eigen::VectorXd>&,
      const Eigen::Ref<const Eigen::VectorXd>&, double,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&)
      const override {
    return {};
  }
  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const override {
    return {nullptr, math::RigidTransformd()};
  }
};

// A convex set that doesn't implement DoCalcVolume but can erroneously report
// that it can compute an exact volume.
class NoImplSet final : public DummyVolumeSet {
 public:
  explicit NoImplSet(int dim, bool can_calc_volume)
      : DummyVolumeSet(dim, can_calc_volume) {}
};

// A convex set that has implemented DoCalcVolume(), but can arbitrarily
// indicate whether it has an exact volume. The value returned by DoCalcVolume()
// depends on whether the constructor's `can_calc_volume` is true or false.
// If true, `DoCalcVolume()` returns a positive value, if `false`, a negative
// value. We should never get a negative value because CalcVolume() should throw
// base on `has_exact_value()`.
class HasImplSet final : public DummyVolumeSet {
 public:
  explicit HasImplSet(int dim, bool can_calc_volume)
      : DummyVolumeSet(dim, can_calc_volume),
        can_calc_volume_(can_calc_volume) {}

 private:
  double DoCalcVolume() const final { return can_calc_volume_ ? 1.5 : -1; }
  bool can_calc_volume_{};
};

// Confirms that CalcVolume() respects has_exact_volume() and the ambient
// dimension before invoking DoCalcVolume() and that errors in derived classes
// are detected and reported.
GTEST_TEST(ConvexSetTest, CalcVolume) {
  // CalcVolume() correctly avoids calling DoCalcVolume().
  DRAKE_EXPECT_THROWS_MESSAGE(
      NoImplSet(1, false).CalcVolume(),
      ".*NoImplSet reports that it cannot report an exact volume.*");

  // CalcVolume() calls DoCalcVolume(), revealing the class has lied.
  DRAKE_EXPECT_THROWS_MESSAGE(
      NoImplSet(1, true).CalcVolume(),
      ".*NoImplSet has a defect -- has_exact_volume.. is reporting true.*");

  // CalcVolume() correctly avoids calling the implemented DoCalcVolume().
  DRAKE_EXPECT_THROWS_MESSAGE(
      HasImplSet(1, false).CalcVolume(),
      ".*HasImplSet reports that it cannot report an exact volume.*");

  // CalcVolume() called DoCalcVolume() correctly, and it returned a positive
  // value.
  EXPECT_GT(HasImplSet(1, true).CalcVolume(), 0);

  // In the case of zero dimension, the exception happens after checking
  // has_exact_volume.
  DRAKE_EXPECT_THROWS_MESSAGE(NoImplSet(0, false).CalcVolume(),
                              ".*an exact volume.*");
  DRAKE_EXPECT_THROWS_MESSAGE(NoImplSet(0, true).CalcVolume(),
                              ".*NoImplSet is a zero-dimensional set.*");
  DRAKE_EXPECT_THROWS_MESSAGE(HasImplSet(0, false).CalcVolume(),
                              ".*an exact volume.*");
  DRAKE_EXPECT_THROWS_MESSAGE(HasImplSet(0, true).CalcVolume(),
                              ".*HasImplSet is a zero-dimensional set.*");
}

// Compute the value of pi via sampling the unit circle.
GTEST_TEST(ConvexSetTest, CalcVolumeViaSampling) {
  Hyperellipsoid unit_circle = Hyperellipsoid::MakeUnitBall(2);
  RandomGenerator generator(1234);
  const double desired_rel_accuracy = 1e-3;
  // We need 250K hits, which means about 318K samples. Most likely will not
  // achieve this with 100K samples.
  const int max_num_samples_low = 1e5;
  const int max_num_samples_high = 1e6;
  const SampledVolume bad_result = unit_circle.CalcVolumeViaSampling(
      &generator, desired_rel_accuracy, max_num_samples_low);
  const SampledVolume good_result = unit_circle.CalcVolumeViaSampling(
      &generator, desired_rel_accuracy, max_num_samples_high);
  EXPECT_GT(bad_result.rel_accuracy, desired_rel_accuracy);
  EXPECT_LE(good_result.rel_accuracy, desired_rel_accuracy);
  // We must get close enough to pi with good estimate
  EXPECT_FALSE(std::abs(bad_result.volume - M_PI) / M_PI <
               desired_rel_accuracy);
  EXPECT_NEAR(good_result.volume, M_PI, M_PI * desired_rel_accuracy);
  // We reach the max_num_samples in the bad estimate, but not in the good one.
  EXPECT_EQ(bad_result.num_samples, max_num_samples_low);
  EXPECT_LT(good_result.num_samples, max_num_samples_high);
};

// Compute the projection of a point onto a set that has no point in set
// shortcut and no projection shortcut.
GTEST_TEST(ConvexSetTest, GenericProjection) {
  const VPolytope vpolytope = VPolytope::MakeUnitBox(2);
  // Each column is a test point.
  // clang-format off
  const Eigen::Matrix2Xd test_points{{0.5, 2, -1.1, 2},
                                     {0.5, 0, -3.0,   2}};
  const Eigen::Matrix2Xd expected_projection{{0.5, 1, -1, 1},
                                             {0.5, 0, -1, 1}};
  // clang-format on

  const std::vector<double> expected_distances{0, 1, sqrt(4.01), sqrt(2)};
  const auto projection_result = vpolytope.Projection(test_points);
  ASSERT_TRUE(projection_result.has_value());
  const auto& [distances, projections] = projection_result.value();
  const double kTol = 1e-6;
  for (int i = 0; i < test_points.cols(); ++i) {
    EXPECT_TRUE(
        CompareMatrices(projections.col(i), expected_projection.col(i), kTol));
    EXPECT_NEAR(distances.at(i), expected_distances.at(i), kTol);
  }
}

// Compute the projection of a point onto a set that does have point in set
// shortcut, but no projection shortcut.
GTEST_TEST(ConvexSetTest, ProjectionWithShortcutPoint) {
  const HPolyhedron polytope = HPolyhedron::MakeUnitBox(2);
  // Each column is a test point.
  // clang-format off
  const Eigen::Matrix2Xd test_points{{0.5, 2, -1.1, 2},
                                     {0.5, 0, -3.0,   2}};
  const Eigen::Matrix2Xd expected_projection{{0.5, 1, -1, 1},
                                             {0.5, 0, -1, 1}};
  // clang-format on

  const std::vector<double> expected_distances{0, 1, sqrt(4.01), sqrt(2)};
  const auto projection_result = polytope.Projection(test_points);
  ASSERT_TRUE(projection_result.has_value());
  const auto& [distances, projections] = projection_result.value();
  const double kTol = 1e-7;
  for (int i = 0; i < test_points.cols(); ++i) {
    EXPECT_TRUE(
        CompareMatrices(projections.col(i), expected_projection.col(i), kTol));
    EXPECT_NEAR(distances.at(i), expected_distances.at(i), kTol);
  }
}

// Compute the projection of a point onto a set that has a projection shortcut.
GTEST_TEST(ConvexSetTest, ShortcutProjection) {
  Eigen::Matrix<double, 2, 1> basis;
  basis << 1, 1;
  Eigen::VectorXd translation(2);
  translation << 1, 0;
  const AffineSubspace as(basis, translation);

  // Each column is a test point.
  // clang-format off
  const Eigen::Matrix2Xd test_points{{0.5, 2, -1.1, 1},
                                     {0.5, 0, -3.0, 0}};
  const Eigen::Matrix2Xd expected_projection{{1, 1.5, -1.55, 1},
                                             {0, 0.5, -2.55,   0}};
  // clang-format on

  const std::vector<double> expected_distances{sqrt(2) / 2, sqrt(2) / 2,
                                               9 / (10 * sqrt(2)), 0.0};
  const auto projection_result = as.Projection(test_points);
  ASSERT_TRUE(projection_result.has_value());
  const auto& [distances, projections] = projection_result.value();
  const double kTol = 1e-7;
  for (int i = 0; i < test_points.cols(); ++i) {
    EXPECT_TRUE(
        CompareMatrices(projections.col(i), expected_projection.col(i), kTol));
    EXPECT_NEAR(distances.at(i), expected_distances.at(i), kTol);
  }
}

GTEST_TEST(ConvexSetTest, Projection0Dim) {
  const HPolyhedron polytope = HPolyhedron::MakeUnitBox(0);
  const Eigen::VectorXd test_point(0);
  const auto projection_result = polytope.Projection(test_point);
  ASSERT_TRUE(projection_result.has_value());
  const auto& [distances, projections] = projection_result.value();
  EXPECT_EQ(distances.at(0), 0);
  EXPECT_EQ(projections, test_point);
}

GTEST_TEST(ConvexSetTest, Projection0DimEmpty) {
  const VPolytope polytope{};
  const Eigen::VectorXd test_point(0);
  const auto projection_result = polytope.Projection(test_point);
  EXPECT_FALSE(projection_result.has_value());
}

GTEST_TEST(ConvexSetTest, ProjectionEmptySet) {
  const Eigen::Matrix<double, 2, 1> A{{1}, {-1}};
  const Eigen::Vector<double, 2> b{-1, -1};
  // The polytope encoding that x ≤ -1, x ≥ 1 is empty.
  const HPolyhedron polytope(A, b);
  EXPECT_TRUE(polytope.IsEmpty());
  const Eigen::Matrix<double, 1, 2> test_point{{0.5, 2}};
  EXPECT_FALSE(polytope.Projection(test_point).has_value());
}

GTEST_TEST(ConvexSetTest, ProjectionError) {
  const HPolyhedron polytope = HPolyhedron::MakeUnitBox(2);
  const Eigen::Vector3d test_point{0.5, 2, -1.1};
  // Wrong dimension of the test point.
  EXPECT_THROW(polytope.Projection(test_point), std::exception);
}

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
