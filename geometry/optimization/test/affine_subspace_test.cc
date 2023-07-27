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
  EXPECT_NO_THROW(dut.Project(Eigen::VectorXd::Zero(0)));
  EXPECT_EQ(dut.AffineDimension(), 0);
  Eigen::VectorXd test_point(0);
  EXPECT_EQ(dut.ToLocalCoordinates(test_point).size(), 0);
  EXPECT_TRUE(CompareMatrices(dut.ToLocalCoordinates(test_point), test_point));
  EXPECT_TRUE(CompareMatrices(
      dut.ToGlobalCoordinates(dut.ToLocalCoordinates(test_point)),
      dut.Project(test_point)));
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
  EXPECT_TRUE(as.PointInSet(as.Project(Eigen::VectorXd::Zero(3))));

  // Should throw because the ambient dimension is wrong.
  EXPECT_THROW(as.Project(Eigen::VectorXd::Zero(1)), std::exception);

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 0);
  Eigen::VectorXd test_point(3);
  test_point << 42, 27, 0;
  EXPECT_EQ(as.ToLocalCoordinates(test_point).size(), 0);
  EXPECT_TRUE(CompareMatrices(as.ToLocalCoordinates(test_point),
                              Eigen::VectorXd::Zero(0)));
  EXPECT_TRUE(
      CompareMatrices(as.ToGlobalCoordinates(as.ToLocalCoordinates(test_point)),
                      as.Project(test_point)));
  EXPECT_TRUE(CompareMatrices(
      as.ToLocalCoordinates(as.ToGlobalCoordinates(Eigen::VectorXd::Zero(0))),
      Eigen::VectorXd::Zero(0)));
}

GTEST_TEST(AffineSubspaceTest, Line) {
  Eigen::Matrix<double, 3, 1> basis;
  basis << 1, 1, 0;
  Eigen::VectorXd translation(3);
  translation << 1, 0, 0;
  const AffineSubspace as(basis, translation);

  const double kTol = 1e-15;

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
  test_point << 2, 1, 0;
  EXPECT_TRUE(as.PointInSet(test_point, kTol));
  EXPECT_FALSE(as.PointInSet(Eigen::VectorXd::Zero(3), kTol));
  EXPECT_TRUE(as.IntersectsWith(as));
  EXPECT_TRUE(as.PointInSet(as.Project(Eigen::VectorXd::Zero(3)), kTol));

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 1);
  Eigen::VectorXd test_point2(3);
  test_point2 << 2, 1, 1;
  Eigen::VectorXd expected_project(3);
  expected_project << 2, 1, 0;
  Eigen::VectorXd expected_local_coords(1);
  expected_local_coords << 1;
  EXPECT_EQ(as.ToLocalCoordinates(test_point2).size(), 1);
  EXPECT_TRUE(CompareMatrices(as.ToLocalCoordinates(test_point2),
                              expected_local_coords, kTol));
  EXPECT_TRUE(CompareMatrices(
      as.ToGlobalCoordinates(as.ToLocalCoordinates(test_point2)),
      as.Project(test_point2), kTol));
  EXPECT_TRUE(CompareMatrices(
      as.ToLocalCoordinates(as.ToGlobalCoordinates(expected_local_coords)),
      expected_local_coords, kTol));
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

  const double kTol = 1e-15;

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
  EXPECT_TRUE(as.PointInSet(as.Project(Eigen::VectorXd::Zero(3))));

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 2);
  Eigen::VectorXd test_point2(3);
  test_point2 << 42, 27, 0;
  Eigen::VectorXd expected_project(3);
  expected_project << 42, 27, 1;
  Eigen::VectorXd expected_local_coords(2);
  expected_local_coords << 42, 27;
  EXPECT_EQ(as.ToLocalCoordinates(test_point2).size(), 2);
  EXPECT_TRUE(CompareMatrices(as.ToLocalCoordinates(test_point2),
                              expected_local_coords, kTol));
  EXPECT_TRUE(CompareMatrices(
      as.ToGlobalCoordinates(as.ToLocalCoordinates(test_point2)),
      as.Project(test_point2), kTol));
  EXPECT_TRUE(CompareMatrices(
      as.ToLocalCoordinates(as.ToGlobalCoordinates(expected_local_coords)),
      expected_local_coords, kTol));
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

  const double kTol = 1e-15;

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
  EXPECT_TRUE(as.PointInSet(as.Project(Eigen::VectorXd::Zero(3))));

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 3);
  Eigen::VectorXd test_point2(3);
  test_point2 << 42, 27, 1;
  Eigen::VectorXd expected_project(3);
  expected_project << 42, 27, 1;
  Eigen::VectorXd expected_local_coords(3);
  expected_local_coords << 42, 27, 0;
  EXPECT_EQ(as.ToLocalCoordinates(test_point2).size(), 3);
  EXPECT_TRUE(CompareMatrices(as.ToLocalCoordinates(test_point2),
                              expected_local_coords, kTol));
  EXPECT_TRUE(CompareMatrices(
      as.ToGlobalCoordinates(as.ToLocalCoordinates(test_point2)),
      as.Project(test_point2), kTol));
  EXPECT_TRUE(CompareMatrices(
      as.ToLocalCoordinates(as.ToGlobalCoordinates(expected_local_coords)),
      expected_local_coords, kTol));
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

  const double kTol = 1e-15;

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
  EXPECT_TRUE(as.PointInSet(as.Project(Eigen::VectorXd::Zero(4))));

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 3);
  Eigen::VectorXd test_point2(4);
  test_point2 << 42, 27, -7, 0;
  Eigen::VectorXd expected_project(4);
  expected_project << 42, 27, -7, 1;
  Eigen::VectorXd expected_local_coords(3);
  expected_local_coords << 42, 27, -7;
  EXPECT_EQ(as.ToLocalCoordinates(test_point2).size(), 3);
  EXPECT_TRUE(CompareMatrices(as.ToLocalCoordinates(test_point2),
                              expected_local_coords, kTol));
  EXPECT_TRUE(CompareMatrices(
      as.ToGlobalCoordinates(as.ToLocalCoordinates(test_point2)),
      as.Project(test_point2), kTol));
  EXPECT_TRUE(CompareMatrices(
      as.ToLocalCoordinates(as.ToGlobalCoordinates(expected_local_coords)),
      expected_local_coords, kTol));
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

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
