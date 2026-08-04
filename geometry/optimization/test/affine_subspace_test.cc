#include "drake/geometry/optimization/affine_subspace.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/spectrahedron.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::Solve;
using solvers::VectorXDecisionVariable;
using symbolic::Variable;

void CheckOrthogonalComplementBasis(const AffineSubspace& as) {
  MatrixXd perpendicular_basis = as.OrthogonalComplementBasis();
  EXPECT_EQ(perpendicular_basis.cols(),
            as.ambient_dimension() - as.AffineDimension());
  EXPECT_EQ(perpendicular_basis.rows(), as.ambient_dimension());
  // Check that every perpendicular basis vector is orthogonal to
  // every basis vector, and that no basis vector has norm zero.
  const double kTol = 1e-15;
  for (int i = 0; i < perpendicular_basis.cols(); ++i) {
    EXPECT_GE(perpendicular_basis.col(i).norm(), kTol);
    for (int j = 0; j < as.basis().cols(); ++j) {
      EXPECT_NEAR(0, as.basis().col(j).dot(perpendicular_basis.col(i)), kTol);
    }
  }
}

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
  EXPECT_TRUE(dut.PointInSet(VectorXd::Zero(0)));
  EXPECT_TRUE(dut.IntersectsWith(dut));
  EXPECT_EQ(dut.AffineDimension(), 0);
  VectorXd test_point(0);
  EXPECT_EQ(dut.ToLocalCoordinates(test_point).size(), 0);
  EXPECT_TRUE(CompareMatrices(dut.ToLocalCoordinates(test_point), test_point));
  const auto projection_result = dut.Projection(test_point);
  const auto& [distances, projections] = projection_result.value();
  EXPECT_TRUE(CompareMatrices(
      dut.ToGlobalCoordinates(dut.ToLocalCoordinates(test_point)),
      projections));
  EXPECT_TRUE(dut.ContainedIn(AffineSubspace()));
  EXPECT_TRUE(dut.IsNearlyEqualTo(AffineSubspace()));
  CheckOrthogonalComplementBasis(dut);
  EXPECT_TRUE(dut.has_exact_volume());
  EXPECT_THROW(dut.CalcVolume(), std::exception);
}

GTEST_TEST(AffineSubspaceTest, Point) {
  Eigen::Matrix<double, 3, 0> basis;
  VectorXd translation(3);
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
  EXPECT_FALSE(as.PointInSet(VectorXd::Zero(3)));
  EXPECT_TRUE(as.IntersectsWith(as));
  auto projection_result = as.Projection(VectorXd::Zero(3));
  EXPECT_TRUE(as.PointInSet(std::get<1>(projection_result.value())));
  CheckOrthogonalComplementBasis(as);
  EXPECT_EQ(as.CalcVolume(), 0);

  // Should throw because the ambient dimension is wrong.
  EXPECT_THROW(as.Projection(VectorXd::Zero(1)), std::exception);

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 0);
  VectorXd test_point(3);
  test_point << 42, 27, 0;
  EXPECT_EQ(as.ToLocalCoordinates(test_point).size(), 0);
  EXPECT_TRUE(
      CompareMatrices(as.ToLocalCoordinates(test_point), VectorXd::Zero(0)));
  projection_result = as.Projection(test_point);
  const auto& [distances, projections] = projection_result.value();
  EXPECT_TRUE(CompareMatrices(
      as.ToGlobalCoordinates(as.ToLocalCoordinates(test_point)), projections));
  EXPECT_TRUE(CompareMatrices(
      as.ToLocalCoordinates(as.ToGlobalCoordinates(VectorXd::Zero(0))),
      VectorXd::Zero(0)));
}

GTEST_TEST(AffineSubspaceTest, Line) {
  Eigen::Matrix<double, 3, 1> basis;
  basis << 1, 1, 0;
  VectorXd translation(3);
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
  VectorXd test_point(3);
  test_point << 2, 1, 0;
  EXPECT_TRUE(as.PointInSet(test_point, kTol));
  EXPECT_FALSE(as.PointInSet(VectorXd::Zero(3), kTol));
  EXPECT_TRUE(as.IntersectsWith(as));
  const auto projection_result = as.Projection(VectorXd::Zero(3));
  ASSERT_TRUE(projection_result.has_value());
  const auto& [distances, projections] = projection_result.value();
  EXPECT_TRUE(as.PointInSet(projections, kTol));
  CheckOrthogonalComplementBasis(as);
  EXPECT_EQ(as.CalcVolume(), 0);

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 1);
  VectorXd test_point2(3);
  test_point2 << 2, 1, 1;
  VectorXd expected_project(3);
  expected_project << 2, 1, 0;
  VectorXd expected_local_coords(1);
  expected_local_coords << 1;
  EXPECT_EQ(as.ToLocalCoordinates(test_point2).size(), 1);
  EXPECT_TRUE(CompareMatrices(as.ToLocalCoordinates(test_point2),
                              expected_local_coords, kTol));
  const auto projection_result2 = as.Projection(test_point2);
  ASSERT_TRUE(projection_result2.has_value());
  const auto& [distances2, projections2] = projection_result2.value();
  EXPECT_TRUE(CompareMatrices(
      as.ToGlobalCoordinates(as.ToLocalCoordinates(test_point2)), projections2,
      kTol));
  EXPECT_TRUE(CompareMatrices(
      as.ToLocalCoordinates(as.ToGlobalCoordinates(expected_local_coords)),
      expected_local_coords, kTol));
  EXPECT_TRUE(as.PointInSet(projections, kTol));
  EXPECT_TRUE(as.PointInSet(projections2, kTol));
}

GTEST_TEST(AffineSubspaceTest, Plane) {
  Eigen::Matrix<double, 3, 2> basis;
  // clang-format off
  basis << 1, 0,
           0, 1,
           0, 0;
  // clang-format on
  VectorXd translation(3);
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
  VectorXd test_point(3);
  test_point << 43, -7, 1;
  EXPECT_TRUE(as.PointInSet(test_point));
  EXPECT_FALSE(as.PointInSet(VectorXd::Zero(3)));
  EXPECT_TRUE(as.IntersectsWith(as));
  const auto projection_result = as.Projection(VectorXd::Zero(3));
  ASSERT_TRUE(projection_result.has_value());
  EXPECT_TRUE(as.PointInSet(std::get<1>(projection_result.value())));
  CheckOrthogonalComplementBasis(as);
  EXPECT_EQ(as.CalcVolume(), 0);

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 2);
  VectorXd test_point2(3);
  test_point2 << 42, 27, 0;
  VectorXd expected_project(3);
  expected_project << 42, 27, 1;
  VectorXd expected_local_coords(2);
  expected_local_coords << 42, 27;
  EXPECT_EQ(as.ToLocalCoordinates(test_point2).size(), 2);
  EXPECT_TRUE(CompareMatrices(as.ToLocalCoordinates(test_point2),
                              expected_local_coords, kTol));
  const auto projection_result2 = as.Projection(test_point2);
  ASSERT_TRUE(projection_result.has_value());
  const auto& [distances2, projections2] = projection_result2.value();
  EXPECT_TRUE(CompareMatrices(
      as.ToGlobalCoordinates(as.ToLocalCoordinates(test_point2)), projections2,
      kTol));
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
  VectorXd translation(3);
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
  VectorXd test_point(3);
  test_point << 43, -7, 1;
  EXPECT_TRUE(as.PointInSet(test_point));
  EXPECT_TRUE(as.PointInSet(VectorXd::Zero(3)));
  EXPECT_TRUE(as.IntersectsWith(as));
  const auto projection_result = as.Projection(VectorXd::Zero(3));
  ASSERT_TRUE(projection_result.has_value());
  EXPECT_TRUE(as.PointInSet(std::get<1>(projection_result.value())));
  CheckOrthogonalComplementBasis(as);
  EXPECT_EQ(as.CalcVolume(), std::numeric_limits<double>::infinity());

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 3);
  VectorXd test_point2(3);
  test_point2 << 42, 27, 1;
  VectorXd expected_project(3);
  expected_project << 42, 27, 1;
  VectorXd expected_local_coords(3);
  expected_local_coords << 42, 27, 0;
  EXPECT_EQ(as.ToLocalCoordinates(test_point2).size(), 3);
  EXPECT_TRUE(CompareMatrices(as.ToLocalCoordinates(test_point2),
                              expected_local_coords, kTol));
  const auto projection_result2 = as.Projection(test_point2);
  ASSERT_TRUE(projection_result2.has_value());
  const auto& [_, projections2] = projection_result2.value();
  EXPECT_TRUE(CompareMatrices(projections2, expected_project, kTol));
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
  VectorXd translation(4);
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
  VectorXd test_point(4);
  test_point << 43, -7, 1, 1;
  EXPECT_TRUE(as.PointInSet(test_point));
  EXPECT_FALSE(as.PointInSet(VectorXd::Zero(4)));
  EXPECT_TRUE(as.IntersectsWith(as));
  const auto projection_result = as.Projection(VectorXd::Zero(4));
  ASSERT_TRUE(projection_result.has_value());
  EXPECT_TRUE(as.PointInSet(std::get<1>(projection_result.value())));
  CheckOrthogonalComplementBasis(as);
  EXPECT_EQ(as.CalcVolume(), 0);

  // Test local coordinates
  EXPECT_EQ(as.AffineDimension(), 3);
  VectorXd test_point2(4);
  test_point2 << 42, 27, -7, 0;
  VectorXd expected_project(4);
  expected_project << 42, 27, -7, 1;
  VectorXd expected_local_coords(3);
  expected_local_coords << 42, 27, -7;
  EXPECT_EQ(as.ToLocalCoordinates(test_point2).size(), 3);
  EXPECT_TRUE(CompareMatrices(as.ToLocalCoordinates(test_point2),
                              expected_local_coords, kTol));
  const auto projection_result2 = as.Projection(test_point2);
  ASSERT_TRUE(projection_result2.has_value());
  const auto& [_, projections2] = projection_result2.value();
  EXPECT_TRUE(CompareMatrices(projections2, expected_project, kTol));
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

  VectorXd translation(2);
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

  VectorXd translation(2);
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
  VectorXd translation(3);
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
  VectorXd translation(3);
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
  VectorXd translation(3);
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
  VectorXd translation(3);
  translation << 0, 0, 1;
  const AffineSubspace as(basis, translation);

  const double kTol = 1e-11;

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto [new_vars, new_constraints] = as.AddPointInSetConstraints(&prog, x);

  // It has to contain 1 linear equality constraint, and 2 new decision
  // variables, corresponding to the two basis vectors.
  EXPECT_EQ(new_constraints.size(), 1);
  EXPECT_EQ(new_vars.rows(), as.basis().cols());

  auto result = Solve(prog);
  ASSERT_TRUE(result.is_success());
  const auto new_vars_val = result.GetSolution(new_vars);
  const Vector3d x_val = result.GetSolution(x);
  EXPECT_TRUE(as.PointInSet(x_val, kTol));
  EXPECT_TRUE(
      CompareMatrices(x_val, as.basis() * new_vars_val + translation, kTol));
}

GTEST_TEST(AffineSubspaceTest, PointInNonnegativeScalingConstraints) {
  Eigen::Matrix<double, 3, 2> basis;
  // clang-format off
  basis << 1, 0,
           0, 1,
           0, 0;
  // clang-format on
  VectorXd translation(3);
  translation << 0, 0, 1;
  const AffineSubspace as(basis, translation);

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];

  std::vector<Binding<Constraint>> constraints =
      as.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  EXPECT_EQ(constraints.size(), 2);

  // Because the nonnegative scaling constraints rely on an additional (hidden)
  // variable y, to check the constraint, we solve an optimization problem,
  // fixing the values of x and t. Success implies a feasible value was found
  // for the unconstrained variable y.

  Vector3d x_test_value = Vector3d::Zero();
  double t_test_value = 0;
  auto x_constraint = prog.AddLinearEqualityConstraint(MatrixXd::Identity(3, 3),
                                                       x_test_value, x);
  auto t_constraint = prog.AddLinearEqualityConstraint(
      MatrixXd::Identity(1, 1), Vector1d(t_test_value), Vector1<Variable>(t));

  // Test values for x, t, and whether the constraint is satisfied.
  const std::vector<std::tuple<Vector3d, double, bool>> test_x_t{
      {Vector3d(-43.0, 43.0, 0.0), 0.0, true},
      {Vector3d(-43.0, 43.0, 0.5), 0.5, true},
      {Vector3d(-43.0, 43.0, 3.0), 3.0, true},
      {Vector3d(-43.0, 43.0, -1.0), 0.0, false},
      {Vector3d(-43.0, 43.0, -1.0), 1.0, false},
      {Vector3d(-43.0, 43.0, 0.0), -1.0, false},
      {Vector3d(-43.0, 43.0, 5.0), 0.0, false},
      {Vector3d(-43.0, 43.0, 1.0), 0.5, false}};

  for (const auto& [x_val, t_val, expect_success] : test_x_t) {
    x_constraint.evaluator()->UpdateCoefficients(MatrixXd::Identity(3, 3),
                                                 x_val);
    t_constraint.evaluator()->UpdateCoefficients(MatrixXd::Identity(1, 1),
                                                 Vector1d(t_val));
    auto result = Solve(prog);
    EXPECT_EQ(result.is_success(), expect_success);
  }

  MatrixXd A(3, 2);
  // clang-format off
  A << 1, 0,
       0, 1,
       2, 0;
  // clang-format on
  Vector3d b = Vector3d::Zero();
  Vector2d c(1, -1);
  double d = 0;

  MathematicalProgram prog2;
  auto x2 = prog2.NewContinuousVariables(2, "x");
  auto t2 = prog2.NewContinuousVariables(2, "t");

  std::vector<Binding<Constraint>> constraints2 =
      as.AddPointInNonnegativeScalingConstraints(&prog2, A, b, c, d, x2, t2);

  EXPECT_EQ(constraints2.size(), 2);

  Vector2d x2_test_value = Vector2d::Zero();
  Vector2d t2_test_value = Vector2d::Zero();
  auto x2_constraint = prog2.AddLinearEqualityConstraint(
      MatrixXd::Identity(2, 2), x2_test_value, x2);
  auto t2_constraint = prog2.AddLinearEqualityConstraint(
      MatrixXd::Identity(2, 2), t2_test_value, t2);

  const std::vector<std::tuple<Vector2d, Vector2d, bool>> test_x2_t2{
      {Vector2d{1.0, 1.0}, Vector2d{2.0, 0.0}, true},
      {Vector2d(2.0, 1.0), Vector2d(4.0, 0.0), true},
      {Vector2d(0.0, 1.0), Vector2d(0.0, 0.0), true},
      {Vector2d(0.0, 1.0), Vector2d(1.0, 1.0), true},
      {Vector2d(2.0, 1.0), Vector2d(0.0, -4.0), true},
      {Vector2d{1.0, 1.0}, Vector2d{0.0, 0.0}, false},
      {Vector2d{1.0, 1.0}, Vector2d{1.0, 0.0}, false},
      {Vector2d{-1.0, 1.0}, Vector2d{0.0, 2.0}, false},
      {Vector2d{-1.0, 1.0}, Vector2d{-2.0, 0.0}, false}};

  for (const auto& [x2_val, t2_val, expect_success] : test_x2_t2) {
    x2_constraint.evaluator()->UpdateCoefficients(MatrixXd::Identity(2, 2),
                                                  x2_val);
    t2_constraint.evaluator()->UpdateCoefficients(MatrixXd::Identity(2, 2),
                                                  t2_val);
    auto result2 = Solve(prog2);
    EXPECT_EQ(result2.is_success(), expect_success);
  }
}

// Check that the ConvexSet is contained in an AffineSubspace
bool CheckAffineSubspaceSetContainment(const AffineSubspace& as,
                                       const ConvexSet& set, double tol = 0) {
  if (!set.MaybeGetFeasiblePoint().has_value()) {
    // The empty set is contained in every affine subspace.
    return true;
  }
  VectorXd feasible_point = set.MaybeGetFeasiblePoint().value();
  if (!as.PointInSet(feasible_point, tol)) {
    return false;
  }

  MathematicalProgram prog;

  // x represents the offset relative to the feasible point. We add a bounding
  // box constraint to ensure the problem is not unbounded.
  VectorXDecisionVariable x =
      prog.NewContinuousVariables(set.ambient_dimension(), "x");
  prog.AddBoundingBoxConstraint(-1, 1, x);

  // y represents the actual point we are finding, so as to constrain it
  // to lie within the convex set.
  VectorXDecisionVariable y =
      prog.NewContinuousVariables(set.ambient_dimension(), "y");
  prog.AddLinearConstraint(y == x + feasible_point);
  set.AddPointInSetConstraints(&prog, y);

  // This is the objective we use. We will iteratively try to minimize and
  // maximize the ith component of x for each dimension to find a feasible point
  // along that axis, and then check that it's contained in the affine hull.
  VectorXd new_objective_vector = VectorXd::Zero(set.ambient_dimension());
  Binding<solvers::LinearCost> objective =
      prog.AddLinearCost(new_objective_vector, x);

  for (int i = 0; i < set.ambient_dimension(); ++i) {
    new_objective_vector.setZero();
    new_objective_vector[i] = 1;
    objective.evaluator()->UpdateCoefficients(new_objective_vector);

    auto result = Solve(prog);
    DRAKE_DEMAND(result.is_success());
    if (!as.PointInSet(result.GetSolution(y), tol)) {
      return false;
    }

    new_objective_vector[i] = -1;
    objective.evaluator()->UpdateCoefficients(new_objective_vector);

    result = Solve(prog);
    DRAKE_DEMAND(result.is_success());
    if (!as.PointInSet(result.GetSolution(y), tol)) {
      return false;
    }
  }

  // If we've made it to the end, then we've successfully checked containment.
  return true;
}

// Check that an AffineSubspace doesn't contain any extra points besides
// what's necessary for the ConvexSet. This function loops through each
// basis vector of the AffineSubspace, removes it, and then sees if the
// ConvexSet is a subset of the new AffineSubspace. If this is not the case,
// then the basis vector was necessary. If it is still a subset, then we
// don't have the affine hull, since it's not the smallest affine set
// containing the ConvexSet.
void CheckAffineHullTightness(const AffineSubspace& as, const ConvexSet& set,
                              double tol = 1e-12) {
  ASSERT_TRUE(as.ambient_dimension() == set.ambient_dimension());
  ASSERT_FALSE(set.IsEmpty());

  const VectorXd translation = as.translation();
  const MatrixXd basis = as.basis();

  for (int i = 0; i < basis.cols(); ++i) {
    const int right_cols_num = basis.cols() - i - 1;
    MatrixXd new_basis(basis.rows(), basis.cols() - 1);
    new_basis << basis.leftCols(i), basis.rightCols(right_cols_num);
    const AffineSubspace new_as(new_basis, translation);
    EXPECT_FALSE(CheckAffineSubspaceSetContainment(new_as, set, tol));
  }
}

GTEST_TEST(AffineSubspaceTest, AffineHulToleranceAPI) {
  // Don't accept a negative tolerance (even if it will be ignored by the
  // function based on the subclass of ConvexSet).
  Point p(Vector1d(1.0));
  HPolyhedron h = HPolyhedron::MakeUnitBox(1);
  EXPECT_THROW(AffineSubspace(p, -1.0), std::exception);
  EXPECT_THROW(AffineSubspace(h, -1.0), std::exception);
}

GTEST_TEST(AffineSubspaceTest, AffineHullCartesianProduct) {
  // Point VPolytope
  VPolytope point(Vector2d(2, -1));

  // Line segment VPolytope
  Eigen::Matrix<double, 3, 2> line_segment_points;
  // clang-format off
  line_segment_points << 0, 1,
                         0, 1,
                         0, 1;
  // clang-format on
  VPolytope line_segment(line_segment_points);

  CartesianProduct c(point, line_segment);
  AffineSubspace as(c);

  EXPECT_EQ(as.basis().cols(), 1);
  EXPECT_EQ(as.basis().rows(), 5);
  EXPECT_EQ(as.translation().size(), 5);
  EXPECT_EQ(as.ambient_dimension(), 5);

  const double kTol = 1e-15;

  VectorXd test_point(5);
  test_point << 2, -1, 2, 2, 2;
  EXPECT_TRUE(as.PointInSet(test_point, kTol));

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as, c, kTol));
  CheckAffineHullTightness(as, c, kTol);
}

GTEST_TEST(AffineSubspaceTest, AffineHullHPolyhedron) {
  // Test a full-dimensional HPolyhedron.
  HPolyhedron h1 = HPolyhedron::MakeUnitBox(3);
  AffineSubspace as1(h1);

  EXPECT_EQ(as1.basis().cols(), 3);
  EXPECT_EQ(as1.basis().rows(), 3);
  EXPECT_EQ(as1.translation().size(), 3);
  EXPECT_EQ(as1.ambient_dimension(), 3);

  const double kTol = 1e-15;

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as1, h1, kTol));
  CheckAffineHullTightness(as1, h1, kTol);

  // Test a not-full-dimensional HPolyhedron.
  MatrixXd A2(6, 3);
  VectorXd b2(6);

  // clang-format off
  A2 <<  1,  0,  0,
        -1,  0,  0,
         0,  1,  0,
         0, -1,  0,
         0,  0,  1,
         0,  0, -1;
  b2 << 1, 0, 1, 0, 0, 0;
  // clang-format on
  HPolyhedron h2(A2, b2);

  const double kTol2 = 1e-6;

  AffineSubspace as2(h2, kTol2);

  EXPECT_EQ(as2.basis().cols(), 2);
  EXPECT_EQ(as2.basis().rows(), 3);
  EXPECT_EQ(as2.translation().size(), 3);
  EXPECT_EQ(as2.ambient_dimension(), 3);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as2, h2, kTol2));
  CheckAffineHullTightness(as2, h2, kTol2);

  // Numerically-challenging HPolyhedron from #20985.
  Eigen::Matrix<double, 6, 3> A3;
  Eigen::Vector<double, 6> b3;
  // clang-format off
  A3 <<  1,  0,  0,
         0,  1,  0,
         0,  0,  1,
        -1,  0,  0,
         0, -1,  0,
         0,  0, -1;
  // clang-format on
  b3 << 0.03, 0.03, 0.075, 0.03, 0.03, 0.075;
  const HPolyhedron h3(A3, b3);
  const AffineSubspace as3(h3, 0);

  EXPECT_EQ(as3.basis().cols(), 3);
  EXPECT_EQ(as3.basis().rows(), 3);
  EXPECT_EQ(as3.translation().size(), 3);
  EXPECT_EQ(as3.ambient_dimension(), 3);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as3, h3, kTol2));
  CheckAffineHullTightness(as3, h3, kTol2);
}

GTEST_TEST(AffineSubspaceTest, AffineHullHyperellipsoid) {
  // Taken from hyperellipsoid_test.cc
  const Eigen::Matrix3d D = Eigen::DiagonalMatrix<double, 3>(1.0, 2.0, 3.0);
  const math::RotationMatrixd R =
      math::RotationMatrixd::MakeZRotation(M_PI / 2.0);
  const Eigen::Matrix3d A = D * R.matrix();
  const Vector3d center{4.0, 5.0, 6.0};

  Hyperellipsoid E(A, center);
  AffineSubspace as(E);

  EXPECT_EQ(as.basis().cols(), 3);
  EXPECT_EQ(as.basis().rows(), 3);
  EXPECT_EQ(as.translation().size(), 3);
  EXPECT_EQ(as.ambient_dimension(), 3);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as, E));
  CheckAffineHullTightness(as, E);
}

GTEST_TEST(AffineSubspaceTest, AffineHullHyperrectangle) {
  const Eigen::Vector3d lb{0, -1, -1};
  const Eigen::Vector3d ub{0, 1, 1};

  Hyperrectangle H(lb, ub);
  AffineSubspace as(H);

  EXPECT_EQ(as.basis().cols(), 2);
  EXPECT_EQ(as.basis().rows(), 3);
  EXPECT_EQ(as.translation().size(), 3);
  EXPECT_EQ(as.ambient_dimension(), 3);

  const double kTol = 1e-15;
  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as, H, kTol));
  CheckAffineHullTightness(as, H);
}

GTEST_TEST(AffineSubspaceTest, AffineHullIntersection) {
  // Point-Line intersection

  // Point VPolytope
  VPolytope point(Vector3d(0.5, 0.5, 0.5));

  // Line segment VPolytope
  Eigen::Matrix<double, 3, 2> line_segment_points;
  // clang-format off
  line_segment_points << 0, 1,
                         0, 1,
                         0, 1;
  // clang-format on
  VPolytope line_segment(line_segment_points);

  Intersection i1(point, line_segment);
  AffineSubspace as1(i1);

  const double kTol1 = 1e-6;
  EXPECT_TRUE(i1.PointInSet(Vector3d(0.5, 0.5, 0.5), kTol1));
  EXPECT_TRUE(as1.PointInSet(Vector3d(0.5, 0.5, 0.5), kTol1));

  EXPECT_EQ(as1.basis().cols(), 0);
  EXPECT_EQ(as1.basis().rows(), 3);
  EXPECT_EQ(as1.translation().size(), 3);
  EXPECT_EQ(as1.ambient_dimension(), 3);

  // Line-Plane intersection (works out to a point)
  Eigen::Matrix<double, 3, 3> triangle1_points;
  // clang-format off
  triangle1_points << 1, 0, 0,
                      0, 1, 0,
                      0, 0, 1;
  // clang-format on
  VPolytope triangle1(triangle1_points);

  Intersection i2(line_segment, triangle1);
  AffineSubspace as2(i2);

  const double kTol2 = 1e-6;
  const Vector3d one_third(1. / 3., 1. / 3., 1. / 3.);
  EXPECT_TRUE(i2.PointInSet(one_third, kTol2));
  EXPECT_TRUE(as2.PointInSet(one_third, kTol2));

  EXPECT_EQ(as2.basis().cols(), 0);
  EXPECT_EQ(as2.basis().rows(), 3);
  EXPECT_EQ(as2.translation().size(), 3);
  EXPECT_EQ(as2.ambient_dimension(), 3);

  // Line-Plane intersection (works out to a line)
  Eigen::Matrix<double, 3, 3> triangle2_points;
  // clang-format off
  triangle2_points << 0, 1, 1,
                      0, 1, 1,
                      0, 0, 1;
  // clang-format on
  VPolytope triangle2(triangle2_points);

  Intersection i3(line_segment, triangle2);
  AffineSubspace as3(i3);

  const double kTol3 = 1e-6;
  EXPECT_TRUE(i3.PointInSet(Vector3d(0, 0, 0), kTol3));
  EXPECT_TRUE(i3.PointInSet(Vector3d(1, 1, 1), kTol3));

  EXPECT_TRUE(as3.PointInSet(Vector3d(0, 0, 0), kTol3));
  EXPECT_TRUE(as3.PointInSet(Vector3d(1, 1, 1), kTol3));

  EXPECT_EQ(as3.basis().cols(), 1);
  EXPECT_EQ(as3.basis().rows(), 3);
  EXPECT_EQ(as3.translation().size(), 3);
  EXPECT_EQ(as3.ambient_dimension(), 3);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as3, i3, kTol3));
  CheckAffineHullTightness(as3, i3, kTol3);

  // Plane-Plane intersection (works out to a line)
  Eigen::Matrix<double, 3, 3> triangle3_points;
  // clang-format off
  triangle3_points << 0, 1, 1,
                      0, 1, 0,
                      0, 0, 5;
  // clang-format on
  VPolytope triangle3(triangle3_points);

  Intersection i4(triangle2, triangle3);
  AffineSubspace as4(i4);

  // The numerics are much better with commerical solvers.
  const double kTol4 = 1e-6;
  EXPECT_TRUE(i4.PointInSet(Vector3d(0, 0, 0), kTol4));
  EXPECT_TRUE(i4.PointInSet(Vector3d(1, 1, 0), kTol4));

  EXPECT_TRUE(as4.PointInSet(Vector3d(0, 0, 0), kTol4));
  EXPECT_TRUE(as4.PointInSet(Vector3d(1, 1, 0), kTol4));

  EXPECT_EQ(as4.basis().cols(), 1);
  EXPECT_EQ(as4.basis().rows(), 3);
  EXPECT_EQ(as4.translation().size(), 3);
  EXPECT_EQ(as4.ambient_dimension(), 3);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as4, i4, kTol4));
  CheckAffineHullTightness(as4, i4, kTol4);

  // Plane-Plane intersection (works out to a plane)
  Eigen::Matrix<double, 3, 3> triangle4_points;
  // clang-format off
  triangle4_points << 0, 1, 1,
                      0, 1, 1,
                      0, 0, 5;
  // clang-format on
  VPolytope triangle4(triangle4_points);

  Intersection i5(triangle2, triangle4);
  AffineSubspace as5(i5);

  EXPECT_EQ(as5.basis().cols(), 2);

  const double kTol5 = 1e-6;
  EXPECT_TRUE(i5.PointInSet(Vector3d(0, 0, 0), kTol5));
  EXPECT_TRUE(i5.PointInSet(Vector3d(1, 1, 0), kTol5));
  EXPECT_TRUE(i5.PointInSet(Vector3d(1, 1, 1), kTol5));

  EXPECT_TRUE(as5.PointInSet(Vector3d(0, 0, 0), kTol5));
  EXPECT_TRUE(as5.PointInSet(Vector3d(1, 1, 0), kTol5));
  EXPECT_TRUE(as5.PointInSet(Vector3d(1, 1, 1), kTol5));

  EXPECT_EQ(as5.basis().cols(), 2);
  EXPECT_EQ(as5.basis().rows(), 3);
  EXPECT_EQ(as5.translation().size(), 3);
  EXPECT_EQ(as5.ambient_dimension(), 3);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as5, i5, kTol5));

  // For this test case, a tighter tolerance is needed to check
  // that the affine hull is the miminmal affine set that
  // contains the intersection. May be related to the tolerance
  // issues for VPolytope described in issue #17197.
  const double tightness_tol = 1e-15;
  CheckAffineHullTightness(as5, i5, tightness_tol);
}

GTEST_TEST(AffineSubspaceTest, AffineHullMinkowskiSum) {
  // Two lines that add to a line
  Eigen::Matrix<double, 3, 2> line_segment1_points;
  // clang-format off
  line_segment1_points << 0, 1,
                          0, 1,
                          0, 1;
  // clang-format on
  VPolytope line_segment1(line_segment1_points);
  Eigen::Matrix<double, 3, 2> line_segment2_points;
  // clang-format off
  line_segment2_points << 2, 3,
                          2, 3,
                          2, 3;
  // clang-format on
  VPolytope line_segment2(line_segment2_points);

  MinkowskiSum ms1(line_segment1, line_segment2);
  AffineSubspace as1(ms1);

  const double kTol = 1e-12;
  EXPECT_TRUE(ms1.PointInSet(Vector3d(2, 2, 2), kTol));
  EXPECT_TRUE(ms1.PointInSet(Vector3d(4, 4, 4), kTol));

  EXPECT_TRUE(as1.PointInSet(Vector3d(2, 2, 2), kTol));
  EXPECT_TRUE(as1.PointInSet(Vector3d(4, 4, 4), kTol));

  EXPECT_EQ(as1.basis().cols(), 1);
  EXPECT_EQ(as1.basis().rows(), 3);
  EXPECT_EQ(as1.translation().size(), 3);
  EXPECT_EQ(as1.ambient_dimension(), 3);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as1, ms1, kTol));
  CheckAffineHullTightness(as1, ms1, kTol);

  // Two lines that add to a plane
  Eigen::Matrix<double, 3, 2> line_segment3_points;
  // clang-format off
  line_segment3_points << 0, 1,
                          0, 0,
                          0, 0;
  // clang-format on
  VPolytope line_segment3(line_segment3_points);

  MinkowskiSum ms2(line_segment1, line_segment3);
  AffineSubspace as2(ms2);

  EXPECT_TRUE(ms2.PointInSet(Vector3d(0, 0, 0), kTol));
  EXPECT_TRUE(ms2.PointInSet(Vector3d(1, 0, 0), kTol));
  EXPECT_TRUE(ms2.PointInSet(Vector3d(1, 1, 1), kTol));

  EXPECT_TRUE(as2.PointInSet(Vector3d(0, 0, 0), kTol));
  EXPECT_TRUE(as2.PointInSet(Vector3d(1, 0, 0), kTol));
  EXPECT_TRUE(as2.PointInSet(Vector3d(1, 1, 1), kTol));

  EXPECT_EQ(as2.basis().cols(), 2);
  EXPECT_EQ(as2.basis().rows(), 3);
  EXPECT_EQ(as2.translation().size(), 3);
  EXPECT_EQ(as2.ambient_dimension(), 3);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as2, ms2, kTol));
  CheckAffineHullTightness(as2, ms2, kTol);

  // A line and a plane that add to a 3d space
  Eigen::Matrix<double, 3, 3> triangle_points;
  // clang-format off
  triangle_points << 0, 0, 0,
                     1, 0, 1,
                     0, 1, 1;
  // clang-format on
  VPolytope triangle(triangle_points);

  MinkowskiSum ms3(line_segment3, triangle);
  AffineSubspace as3(ms3);

  EXPECT_TRUE(ms3.PointInSet(Vector3d(0, 1, 0), kTol));
  EXPECT_TRUE(ms3.PointInSet(Vector3d(0, 0, 1), kTol));
  EXPECT_TRUE(ms3.PointInSet(Vector3d(0, 1, 1), kTol));
  EXPECT_TRUE(ms3.PointInSet(Vector3d(1, 1, 1), kTol));

  EXPECT_TRUE(as3.PointInSet(Vector3d(0, 1, 0), kTol));
  EXPECT_TRUE(as3.PointInSet(Vector3d(0, 0, 1), kTol));
  EXPECT_TRUE(as3.PointInSet(Vector3d(0, 1, 1), kTol));
  EXPECT_TRUE(as3.PointInSet(Vector3d(1, 1, 1), kTol));

  EXPECT_EQ(as3.basis().cols(), 3);
  EXPECT_EQ(as3.basis().rows(), 3);
  EXPECT_EQ(as3.translation().size(), 3);
  EXPECT_EQ(as3.ambient_dimension(), 3);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as3, ms3, kTol));
  CheckAffineHullTightness(as3, ms3, kTol);
}

GTEST_TEST(AffineSubspaceTest, AffineHullPoint) {
  const Vector3d p_value{4.2, 2.7, 0.0};
  Point p(p_value);

  // The tolerance is ignored, but including it verifies that we can call the
  // constructor with a double (instead of std::optional<double>).
  double kTol = 43.0;
  AffineSubspace as(p, kTol);

  EXPECT_EQ(as.basis().cols(), 0);
  EXPECT_EQ(as.basis().rows(), 3);
  EXPECT_EQ(as.translation().size(), 3);
  EXPECT_EQ(as.ambient_dimension(), 3);

  EXPECT_TRUE(as.IsBounded());
  ASSERT_TRUE(p.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(as.PointInSet(p.MaybeGetFeasiblePoint().value()));
}

GTEST_TEST(AffineSubspaceTest, AffineHullSpectrahedron) {
  // Test the default constructor
  Spectrahedron spect1;
  AffineSubspace as1(spect1);

  EXPECT_EQ(as1.basis().cols(), 0);
  EXPECT_EQ(as1.basis().rows(), 0);
  EXPECT_EQ(as1.translation().size(), 0);
  EXPECT_EQ(as1.ambient_dimension(), 0);

  /*
   * A trivial SDP
   * max X1(0, 1) + X1(1, 2),
   * s.t X1 ∈ ℝ³ˣ³ is psd,
   *     X1(0, 0) + X1(1, 1) + X1(2, 2) = 1,
   *     -2 ≤ X1(0, 1) + X1(1, 2) - 2 * X1(0, 2) ≤ 0,
   *     X1(2, 2) ∈ [-1, 1].
   */

  // Taken from spectrahedron_test.cc
  // The ambient dimension of this spectrahedron is 6. (Because a 3x3 symmetric
  // matrix has 6 unique entries, and the remaining 3 are not considered part
  // of the ambient space.) This spectrahedron has 1 additional equality
  // constraint, so its affine dimension should be 5.
  MathematicalProgram prog;
  auto X1 = prog.NewSymmetricContinuousVariables<3>();
  prog.AddLinearCost(-(X1(0, 1) + X1(1, 2)));
  prog.AddPositiveSemidefiniteConstraint(X1);
  prog.AddLinearEqualityConstraint(X1(0, 0) + X1(1, 1) + X1(2, 2), 1);
  prog.AddLinearConstraint(X1(0, 1) + X1(1, 2) - 2 * X1(0, 2), -2, 0);
  prog.AddBoundingBoxConstraint(-1, 1, X1(2, 2));

  Spectrahedron spect2(prog);
  EXPECT_EQ(spect2.ambient_dimension(), 6);

  const double kTol = 1e-8;

  AffineSubspace as2(spect2, kTol);

  EXPECT_EQ(as2.basis().cols(), 5);
  EXPECT_EQ(as2.basis().rows(), 6);
  EXPECT_EQ(as2.translation().size(), 6);
  EXPECT_EQ(as2.ambient_dimension(), 6);

  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as2, spect2, kTol));
  CheckAffineHullTightness(as2, spect2, kTol);
}

GTEST_TEST(AffineSubspaceTest, AffineHullVPolytope) {
  const double kTol = 1e-12;
  // Check that computing the affine hull of an empty set throws an error
  const VPolytope dut;
  EXPECT_THROW(AffineSubspace{dut}, std::exception);

  // Check a point as a VPolytope
  Vector3d point(2, -1, 0);
  VPolytope v(point);
  AffineSubspace as1(v);

  EXPECT_EQ(as1.basis().cols(), 0);
  EXPECT_EQ(as1.basis().rows(), 3);
  EXPECT_EQ(as1.translation().size(), 3);
  EXPECT_EQ(as1.ambient_dimension(), 3);

  EXPECT_TRUE(as1.PointInSet(Vector3d(2, -1, 0), kTol));
  EXPECT_FALSE(as1.PointInSet(Vector3d(2, -1, 1), kTol));
  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as1, v));
  CheckAffineHullTightness(as1, v);

  // Check a line segment as a VPolytope
  Eigen::Matrix<double, 3, 2> line_segment_points;
  // clang-format off
  line_segment_points << 0, 1,
                         0, 1,
                         0, 1;
  // clang-format on
  VPolytope line_segment(line_segment_points);
  AffineSubspace as2(line_segment);

  EXPECT_EQ(as2.basis().cols(), 1);
  EXPECT_EQ(as2.basis().rows(), 3);
  EXPECT_EQ(as2.translation().size(), 3);
  EXPECT_EQ(as2.ambient_dimension(), 3);

  EXPECT_TRUE(as2.PointInSet(Vector3d(2, 2, 2), kTol));
  EXPECT_FALSE(as2.PointInSet(Vector3d(2, 2, 0), kTol));
  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as2, line_segment, kTol));
  CheckAffineHullTightness(as2, line_segment);

  // Check a triangle in 3D as a VPolytope
  Eigen::Matrix<double, 3, 3> triangle_points;
  // clang-format off
  triangle_points <<  0, 1, 0,
                      0, 0, 1,
                      0, 0, 0;
  // clang-format on
  VPolytope triangle(triangle_points);
  AffineSubspace as3(triangle);

  EXPECT_EQ(as3.basis().cols(), 2);
  EXPECT_EQ(as3.basis().rows(), 3);
  EXPECT_EQ(as3.translation().size(), 3);
  EXPECT_EQ(as3.ambient_dimension(), 3);

  EXPECT_TRUE(as3.PointInSet(Vector3d(42, 27, 0), kTol));
  EXPECT_FALSE(as3.PointInSet(Vector3d(42, 27, 1), kTol));
  EXPECT_TRUE(CheckAffineSubspaceSetContainment(as3, triangle));
  CheckAffineHullTightness(as3, triangle);
}

GTEST_TEST(AffineSubspaceTest, BatchChangeOfCoordinates) {
  // Test that the projection and transforms to and from local coordinates can
  // handle batches of points.
  Eigen::Matrix<double, 3, 2> basis;
  // clang-format off
  basis << 1, 0,
           0, 1,
           0, 0;
  // clang-format on
  VectorXd translation(3);
  translation << 0, 0, 1;
  const AffineSubspace as(basis, translation);

  Eigen::Matrix<double, 3, 5> points;
  // clang-format off
  points << 1, 2, 3, 4, 5,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0;
  // clang-format on
  const auto projection_result = as.Projection(points);
  ASSERT_TRUE(projection_result.has_value());
  const auto& [distances, projections] = projection_result.value();
  for (int i = 0; i < points.cols(); ++i) {
    EXPECT_TRUE(CompareMatrices(
        projections.col(i), std::get<1>(as.Projection(points.col(i)).value())));
  }

  MatrixXd local = as.ToLocalCoordinates(points);
  EXPECT_EQ(local.rows(), 2);
  EXPECT_EQ(local.cols(), 5);
  for (int i = 0; i < points.cols(); ++i) {
    EXPECT_TRUE(
        CompareMatrices(local.col(i), as.ToLocalCoordinates(points.col(i))));
  }

  MatrixXd global = as.ToGlobalCoordinates(local);
  EXPECT_EQ(global.rows(), 3);
  EXPECT_EQ(global.cols(), 5);
  for (int i = 0; i < local.cols(); ++i) {
    EXPECT_TRUE(
        CompareMatrices(global.col(i), as.ToGlobalCoordinates(local.col(i))));
  }
}

GTEST_TEST(AffineSubspaceTest, ContainmentTest) {
  Eigen::Matrix<double, 3, 1> basis1;
  // clang-format off
  basis1 << 1,
            1,
            0;
  // clang-format on
  VectorXd translation1(3);
  translation1 << 0, 0, 1;
  const AffineSubspace as1(basis1, translation1);

  Eigen::Matrix<double, 3, 2> basis2;
  // clang-format off
  basis2 << 1, 0,
            0, 1,
            0, 0;
  // clang-format on
  VectorXd translation2(3);
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
  VectorXd translation1(2);
  translation1 << 0, 1;
  const AffineSubspace as1(basis1, translation1);

  Eigen::Matrix<double, 3, 2> basis2;
  // clang-format off
  basis2 << 1, 0,
            0, 1,
            0, 0;
  // clang-format on
  VectorXd translation2(3);
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
  // clang-format on
  VectorXd translation1(3);
  translation1 << 0, 0, 1;

  Eigen::Matrix<double, 3, 2> basis2;
  // clang-format off
  basis2 << 1, 1,
            1, -1,
            0, 0;
  // clang-format on
  VectorXd translation2(3);
  translation2 << 1, 1, 1;

  const AffineSubspace as1(basis1, translation1);
  const AffineSubspace as2(basis1, translation1);
  const AffineSubspace as3(basis1, translation2);
  const AffineSubspace as4(basis2, translation1);
  const AffineSubspace as5(basis2, translation2);

  const double kTol = 1e-14;

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
  // clang-format on
  Eigen::Matrix<double, 3, 2> basis2;
  // clang-format off
  basis2 << 1,  0,
            -1, 1,
            0, -1;
  // clang-format on
  VectorXd translation1(3);
  translation1 << 0, 0, 1;
  VectorXd translation2(3);
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
  VectorXd translation(2);
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
