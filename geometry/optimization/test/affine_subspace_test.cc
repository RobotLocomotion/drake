#include "drake/geometry/optimization/affine_subspace.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

GTEST_TEST(AffineSubspaceTest, DefaultCtor) {
  const AffineSubspace dut;
  EXPECT_EQ(dut.basis().cols(), 0);
  EXPECT_EQ(dut.basis().rows(), 0);
  EXPECT_EQ(dut.translation().size(), 0);
  EXPECT_EQ(dut.ambient_dimension(), 0);
  EXPECT_NO_THROW(dut.Clone());
  EXPECT_TRUE(dut.IsBounded());
  EXPECT_FALSE(dut.IsEmpty());
  EXPECT_TRUE(dut.MaybeGetPoint().has_value());
  ASSERT_TRUE(dut.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(dut.PointInSet(dut.MaybeGetFeasiblePoint().value()));
  EXPECT_TRUE(dut.PointInSet(Eigen::VectorXd::Zero(0)));
  EXPECT_TRUE(dut.IntersectsWith(dut));
  EXPECT_TRUE(dut.ContainedIn(AffineSubspace()));
  EXPECT_TRUE(dut.IsNearlyEqualTo(AffineSubspace()));
}

GTEST_TEST(AffineSubspaceTest, Point) {
  Eigen::Matrix<double, 3, 0> basis;
  Eigen::VectorXd translation(3);
  translation << 1, 2, 3;
  const AffineSubspace as(basis, translation);

  EXPECT_EQ(as.basis().cols(), 0);
  EXPECT_EQ(as.basis().rows(), 3);
  EXPECT_EQ(as.translation().size(), 3);
  EXPECT_EQ(as.ambient_dimension(), 3);
  EXPECT_NO_THROW(as.Clone());
  EXPECT_TRUE(as.IsBounded());
  EXPECT_FALSE(as.IsEmpty());
  EXPECT_TRUE(as.MaybeGetPoint().has_value());
  ASSERT_TRUE(as.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(as.PointInSet(as.MaybeGetFeasiblePoint().value()));
  EXPECT_TRUE(as.PointInSet(translation));
  EXPECT_FALSE(as.PointInSet(Eigen::VectorXd::Zero(3)));
  EXPECT_TRUE(as.IntersectsWith(as));
}

GTEST_TEST(AffineSubspaceTest, Line) {
  Eigen::Matrix<double, 3, 1> basis;
  basis << 1, 1, 1;
  Eigen::VectorXd translation(3);
  translation << 1, 0, 0;
  const AffineSubspace as(basis, translation);

  EXPECT_EQ(as.basis().cols(), 1);
  EXPECT_EQ(as.basis().rows(), 3);
  EXPECT_EQ(as.translation().size(), 3);
  EXPECT_EQ(as.ambient_dimension(), 3);
  EXPECT_NO_THROW(as.Clone());
  EXPECT_FALSE(as.IsBounded());
  EXPECT_FALSE(as.IsEmpty());
  EXPECT_FALSE(as.MaybeGetPoint().has_value());
  ASSERT_TRUE(as.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(as.PointInSet(as.MaybeGetFeasiblePoint().value()));
  EXPECT_TRUE(as.PointInSet(translation));
  Eigen::VectorXd test_point(3);
  test_point << 2, 1, 1;
  EXPECT_TRUE(as.PointInSet(test_point));
  EXPECT_FALSE(as.PointInSet(Eigen::VectorXd::Zero(3)));
  EXPECT_TRUE(as.IntersectsWith(as));
}

GTEST_TEST(AffineSubspaceTest, Plane) {
  Eigen::Matrix<double, 3, 2> basis;
  // clang-format off
  basis << 1, 0,
           0, 1,
           0, 0;
  // clang-format on
  Eigen::VectorXd translation(3);
  translation << 0, 0, 1;
  const AffineSubspace as(basis, translation);

  EXPECT_EQ(as.basis().cols(), 2);
  EXPECT_EQ(as.basis().rows(), 3);
  EXPECT_EQ(as.translation().size(), 3);
  EXPECT_EQ(as.ambient_dimension(), 3);
  EXPECT_NO_THROW(as.Clone());
  EXPECT_FALSE(as.IsBounded());
  EXPECT_FALSE(as.IsEmpty());
  EXPECT_FALSE(as.MaybeGetPoint().has_value());
  ASSERT_TRUE(as.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(as.PointInSet(as.MaybeGetFeasiblePoint().value()));
  EXPECT_TRUE(as.PointInSet(translation));
  Eigen::VectorXd test_point(3);
  test_point << 43, -7, 1;
  EXPECT_TRUE(as.PointInSet(test_point));
  EXPECT_FALSE(as.PointInSet(Eigen::VectorXd::Zero(3)));
  EXPECT_TRUE(as.IntersectsWith(as));
}

GTEST_TEST(AffineSubspaceTest, VolumeInR3) {
  Eigen::Matrix<double, 3, 3> basis;
  // clang-format off
  basis << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
  // clang-format on
  Eigen::VectorXd translation(3);
  translation << 0, 0, 1;
  const AffineSubspace as(basis, translation);

  EXPECT_EQ(as.basis().cols(), 3);
  EXPECT_EQ(as.basis().rows(), 3);
  EXPECT_EQ(as.translation().size(), 3);
  EXPECT_EQ(as.ambient_dimension(), 3);
  EXPECT_NO_THROW(as.Clone());
  EXPECT_FALSE(as.IsBounded());
  EXPECT_FALSE(as.IsEmpty());
  EXPECT_FALSE(as.MaybeGetPoint().has_value());
  ASSERT_TRUE(as.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(as.PointInSet(as.MaybeGetFeasiblePoint().value()));
  EXPECT_TRUE(as.PointInSet(translation));
  Eigen::VectorXd test_point(3);
  test_point << 43, -7, 1;
  EXPECT_TRUE(as.PointInSet(test_point));
  EXPECT_TRUE(as.PointInSet(Eigen::VectorXd::Zero(3)));
  EXPECT_TRUE(as.IntersectsWith(as));
}

GTEST_TEST(AffineSubspaceTest, VolumeInR4) {
  Eigen::Matrix<double, 4, 3> basis;
  // clang-format off
  basis << 1, 0, 0,
           0, 1, 0,
           0, 0, 1,
           0, 0, 0;
  // clang-format on
  Eigen::VectorXd translation(4);
  translation << 0, 0, 0, 1;
  const AffineSubspace as(basis, translation);

  EXPECT_EQ(as.basis().cols(), 3);
  EXPECT_EQ(as.basis().rows(), 4);
  EXPECT_EQ(as.translation().size(), 4);
  EXPECT_EQ(as.ambient_dimension(), 4);
  EXPECT_NO_THROW(as.Clone());
  EXPECT_FALSE(as.IsBounded());
  EXPECT_FALSE(as.IsEmpty());
  EXPECT_FALSE(as.MaybeGetPoint().has_value());
  ASSERT_TRUE(as.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(as.PointInSet(as.MaybeGetFeasiblePoint().value()));
  EXPECT_TRUE(as.PointInSet(translation));
  Eigen::VectorXd test_point(4);
  test_point << 43, -7, 1, 1;
  EXPECT_TRUE(as.PointInSet(test_point));
  EXPECT_FALSE(as.PointInSet(Eigen::VectorXd::Zero(4)));
  EXPECT_TRUE(as.IntersectsWith(as));
}

GTEST_TEST(AffineSubspaceTest, NotABasis1) {
  // If we try to construct with vectors that don't form a basis,
  // we should get an error.
  // 3 vectors in R^2 (not a basis)
  Eigen::Matrix<double, 2, 3> basis;
  // clang-format off
  basis << 1, 0, 1,
           0, 1, 1;
  // clang-format on

  Eigen::VectorXd translation(2);
  translation << 0, 1;
  EXPECT_THROW(AffineSubspace(basis, translation), std::exception);
}

GTEST_TEST(AffineSubspaceTest, NotABasis2) {
  // If we try to construct with vectors that don't form a basis,
  // we should get an error.
  // 2 linearly dependent vectors (not a basis)
  Eigen::Matrix<double, 2, 2> basis;
  // clang-format off
  basis << 1, 2,
           0, 0;
  // clang-format on

  Eigen::VectorXd translation(2);
  translation << 0, 1;
  EXPECT_THROW(AffineSubspace(basis, translation), std::exception);
}

GTEST_TEST(AffineSubspaceTest, Serialize) {
  Eigen::Matrix<double, 3, 2> basis;
  // clang-format off
  basis << 1, 0,
           0, 1,
           0, 0;
  // clang-format on
  Eigen::VectorXd translation(3);
  translation << 0, 0, 1;
  const AffineSubspace as(basis, translation);
  const std::string yaml = yaml::SaveYamlString(as);
  const auto as2 = yaml::LoadYamlString<AffineSubspace>(yaml);
  EXPECT_EQ(as.ambient_dimension(), as2.ambient_dimension());
  EXPECT_TRUE(CompareMatrices(as.basis(), as2.basis()));
  EXPECT_TRUE(CompareMatrices(as.translation(), as2.translation()));
}

GTEST_TEST(AffineSubspaceTest, Move) {
  Eigen::Matrix<double, 3, 2> basis;
  // clang-format off
  basis << 1, 0,
           0, 1,
           0, 0;
  // clang-format on
  Eigen::VectorXd translation(3);
  translation << 0, 0, 1;
  const AffineSubspace orig(basis, translation);

  // A move-constructed AffineSubspace takes over the original data.
  AffineSubspace dut(std::move(orig));
  EXPECT_EQ(dut.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(dut.basis(), basis));
  EXPECT_TRUE(CompareMatrices(dut.translation(), translation));

  // The old AffineSubspace is in a valid but unspecified state.
  EXPECT_EQ(orig.basis().rows(), orig.ambient_dimension());
  EXPECT_NO_THROW(orig.Clone());
}

GTEST_TEST(AffineSubspaceTest, CloneTest) {
  Eigen::Matrix<double, 3, 2> basis;
  // clang-format off
  basis << 1, 0,
           0, 1,
           0, 0;
  // clang-format on
  Eigen::VectorXd translation(3);
  translation << 0, 0, 1;
  const AffineSubspace as(basis, translation);

  std::unique_ptr<ConvexSet> clone = as.Clone();
  EXPECT_EQ(clone->ambient_dimension(), as.ambient_dimension());
  AffineSubspace* pointer = dynamic_cast<AffineSubspace*>(clone.get());
  ASSERT_NE(pointer, nullptr);
  EXPECT_TRUE(CompareMatrices(as.basis(), pointer->basis()));
  EXPECT_TRUE(CompareMatrices(as.translation(), pointer->translation()));
}

GTEST_TEST(AffineSubspaceTest, PointInSetConstraints) {
  Eigen::Matrix<double, 3, 2> basis;
  // clang-format off
  basis << 1, 0,
           0, 1,
           0, 0;
  // clang-format on
  Eigen::VectorXd translation(3);
  translation << 0, 0, 1;
  const AffineSubspace as(basis, translation);

  const double kTol = 1e-11;

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto [new_vars, new_constraints] = as.AddPointInSetConstraints(&prog, x);

  // It has to contain 1 linear equality constraint, and 2 new decision
  // variables, corresponding to the two basis vectors.
  EXPECT_EQ(new_constraints.size(), 1);
  EXPECT_EQ(new_vars.rows(), as.basis().cols());

  auto result = solvers::Solve(prog);
  ASSERT_TRUE(result.is_success());
  const auto new_vars_val = result.GetSolution(new_vars);
  const Eigen::Vector3d x_val = result.GetSolution(x);
  EXPECT_TRUE(as.PointInSet(x_val, kTol));
  EXPECT_TRUE(
      CompareMatrices(x_val, as.basis() * new_vars_val + translation, kTol));
}

GTEST_TEST(AffineSubspaceTest, ContainmentTest) {
  Eigen::Matrix<double, 3, 1> basis1;
  // clang-format off
  basis1 << 1,
            1,
            0;
  // clang-format on
  Eigen::VectorXd translation1(3);
  translation1 << 0, 0, 1;
  const AffineSubspace as1(basis1, translation1);

  Eigen::Matrix<double, 3, 2> basis2;
  // clang-format off
  basis2 << 1, 0,
            0, 1,
            0, 0;
  // clang-format on
  Eigen::VectorXd translation2(3);
  translation2 << 1, 1, 1;
  const AffineSubspace as2(basis2, translation2);

  EXPECT_TRUE(as1.ContainedIn(as2));
  EXPECT_FALSE(as2.ContainedIn(as1));
  EXPECT_FALSE(as1.IsNearlyEqualTo(as2));
}

GTEST_TEST(AffineSubspaceTest, CompareDifferentDimensions) {
  // The containment, equality, and inequality methods must handle
  // the case where the AffineSubspaces are of differing ambient
  // dimension.
  Eigen::Matrix<double, 2, 1> basis1;
  // clang-format off
  basis1 << 1,
            0;
  // clang-format on
  Eigen::VectorXd translation1(2);
  translation1 << 0, 1;
  const AffineSubspace as1(basis1, translation1);

  Eigen::Matrix<double, 3, 2> basis2;
  // clang-format off
  basis2 << 1, 0,
            0, 1,
            0, 0;
  // clang-format on
  Eigen::VectorXd translation2(3);
  translation2 << 0, 0, 1;
  const AffineSubspace as2(basis2, translation2);

  EXPECT_FALSE(as1.ContainedIn(as2));
  EXPECT_FALSE(as2.ContainedIn(as1));
  EXPECT_FALSE(as1.IsNearlyEqualTo(as2));
}

GTEST_TEST(AffineSubspaceTest, EqualityTest) {
  // An affine subspace is invariant under change of basis, and any choice of
  // point in the affine subspace can be used as the translation. This test
  // verifies that these properties hold.
  Eigen::Matrix<double, 3, 2> basis1;
  // clang-format off
  basis1 << 1, 0,
            0, 1,
            0, 0;
  // clang- format on
  Eigen::VectorXd translation1(3);
  translation1 << 0, 0, 1;

  Eigen::Matrix<double, 3, 2> basis2;
  // clang-format off
  basis2 << 1, 1,
            1, -1,
            0, 0;
  // clang-format on
  Eigen::VectorXd translation2(3);
  translation2 << 1, 1, 1;

  const AffineSubspace as1(basis1, translation1);
  const AffineSubspace as2(basis1, translation1);
  const AffineSubspace as3(basis1, translation2);
  const AffineSubspace as4(basis2, translation1);
  const AffineSubspace as5(basis2, translation2);

  const double kTol = 1e-15;

  EXPECT_TRUE(as1.IsNearlyEqualTo(as2, kTol));
  EXPECT_TRUE(as2.IsNearlyEqualTo(as3, kTol));
  EXPECT_TRUE(as3.IsNearlyEqualTo(as4, kTol));
  EXPECT_TRUE(as4.IsNearlyEqualTo(as5, kTol));
}

GTEST_TEST(AffineSubspaceTest, EqualityTest2) {
  // This test checks to make sure that IsNearlyEqualTo still works when the
  // translations are clearly different, and the AffineSubspaces aren't
  // axis-aligned. This subspace is the plane passing through the points
  // (1, 0, 0); (0, 1, 0); and (0, 0, 1). We give two different bases and two
  // different translations -- note that the basis vectors are always orthogonal
  // to the vector (1, 1, 1).
  Eigen::Matrix<double, 3, 2> basis1;
  // clang-format off
  basis1 << -1,  0.5,
            0.5, -1,
            0.5, 0.5;
  // clang- format on
  Eigen::Matrix<double, 3, 2> basis2;
  // clang-format off
  basis2 << 1,  0,
            -1, 1,
            0, -1;
  // clang- format on
  Eigen::VectorXd translation1(3);
  translation1 << 0, 0, 1;
  Eigen::VectorXd translation2(3);
  translation2 << 0, 1, 0;

  const AffineSubspace as1(basis1, translation1);
  const AffineSubspace as2(basis1, translation2);
  const AffineSubspace as3(basis2, translation1);
  const AffineSubspace as4(basis2, translation2);

  const double kTol = 1e-15;

  EXPECT_TRUE(as1.IsNearlyEqualTo(as2, kTol));
  EXPECT_TRUE(as2.IsNearlyEqualTo(as3, kTol));
  EXPECT_TRUE(as3.IsNearlyEqualTo(as4, kTol));
}

GTEST_TEST(AffineSubspaceTest, DeliberatelyLooseTolerance) {
  // Verify that we can set the tolerance very high, and make it look like two
  // affine subspaces are equivalent (when they aren't).
  Eigen::Matrix<double, 2, 1> basis1;
  basis1 << 1, 0;
  Eigen::Matrix<double, 2, 1> basis2;
  basis2 << 0, 1;
  Eigen::VectorXd translation(2);
  translation << 0, 0;

  const AffineSubspace as1(basis1, translation);
  const AffineSubspace as2(basis2, translation);

  const double almost_too_high_tol = 1. - 1e-12;
  const double too_high_tol = 1. + 1e-12;

  EXPECT_FALSE(as1.ContainedIn(as2, almost_too_high_tol));
  EXPECT_TRUE(as1.ContainedIn(as2, too_high_tol));
  EXPECT_FALSE(as2.ContainedIn(as1, almost_too_high_tol));
  EXPECT_TRUE(as2.ContainedIn(as1, too_high_tol));
  EXPECT_FALSE(as1.IsNearlyEqualTo(as2, almost_too_high_tol));
  EXPECT_TRUE(as1.IsNearlyEqualTo(as2, too_high_tol));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
