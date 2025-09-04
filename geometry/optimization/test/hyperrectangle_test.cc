#include "drake/geometry/optimization/hyperrectangle.h"

#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;

bool PointInScaledSet(const solvers::VectorXDecisionVariable& x_vars,
                      const solvers::VectorXDecisionVariable& t_vars,
                      const VectorXd& x, const VectorXd& t,
                      solvers::MathematicalProgram* prog,
                      const std::vector<Binding<Constraint>>& constraints) {
  const double tol = 0;
  prog->SetInitialGuess(x_vars, x);
  prog->SetInitialGuess(t_vars, t);
  return prog->CheckSatisfiedAtInitialGuess(constraints, tol);
}

GTEST_TEST(HyperrectangleTest, Default) {
  const Hyperrectangle dut;
  EXPECT_EQ(dut.ambient_dimension(), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(dut.CalcVolume(), ".*zero.*");
  RandomGenerator gen(1234);
  DRAKE_EXPECT_THROWS_MESSAGE(dut.CalcVolumeViaSampling(&gen), ".*zero.*");
  EXPECT_EQ(dut.Center().size(), 0);
  EXPECT_TRUE(dut.MaybeGetPoint().has_value());
  EXPECT_EQ(dut.MaybeGetPoint()->size(), 0);
  EXPECT_TRUE(dut.MaybeGetFeasiblePoint().has_value());
  EXPECT_EQ(dut.MaybeGetFeasiblePoint()->size(), 0);
  EXPECT_TRUE(dut.PointInSet(VectorXd()));
  const auto hpolyhedron = dut.MakeHPolyhedron();
  EXPECT_EQ(hpolyhedron.ambient_dimension(), 0);
}

// Test functions on hyperrectangle.
GTEST_TEST(HyperrectangleTest, BasicTests) {
  const Vector3d lb{-1, -2, -3};
  const Vector3d ub{3, 2, 1};
  const Hyperrectangle hyperrectangle(lb, ub);
  EXPECT_EQ(hyperrectangle.ambient_dimension(), 3);
  // The volume of the hyperrectangle is 4*4*4 = 64.
  EXPECT_NEAR(hyperrectangle.CalcVolume(), 64, 1e-6);
  // Check the center of the hyperrectangle.
  const auto center = hyperrectangle.Center();
  EXPECT_TRUE(CompareMatrices(center, (lb + ub) / 2.0, 1e-6));
  // Getting the unique point in the set.
  EXPECT_FALSE(hyperrectangle.MaybeGetPoint().has_value());
  // Getting a feasible point in the set and it is the center
  const auto feasible_point = hyperrectangle.MaybeGetFeasiblePoint();
  EXPECT_TRUE(feasible_point.has_value());
  EXPECT_TRUE(CompareMatrices(feasible_point.value(), center, 1e-6));
  // Make it a HPolyhedron and compare the Chebyshev center
  const auto hpolyhedron = hyperrectangle.MakeHPolyhedron();
  const auto chebyshev_center = hpolyhedron.ChebyshevCenter();
  EXPECT_TRUE(CompareMatrices(chebyshev_center, center, 1e-6));
  // Can not make an invalid hyperrectangle.
  EXPECT_THROW(Hyperrectangle(Vector3d{1, 2, 3}, Vector3d{1, 2, 1}),
               std::exception);
  // It is ok to make a point hyperrectangle.
  const Hyperrectangle point_hyperrectangle(Vector3d{1, 2, 3},
                                            Vector3d{1, 2, 3});
  EXPECT_EQ(point_hyperrectangle.ambient_dimension(), 3);
  EXPECT_TRUE(point_hyperrectangle.IsBounded());
  EXPECT_TRUE(point_hyperrectangle.PointInSet(Vector3d{1, 2, 3}));
  EXPECT_TRUE(point_hyperrectangle.MaybeGetPoint().has_value());
  EXPECT_TRUE(point_hyperrectangle.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(CompareMatrices(
      point_hyperrectangle.MaybeGetFeasiblePoint().value(), Vector3d{1, 2, 3}));
}

GTEST_TEST(HyperrectangleTest, Sampling) {
  const Hyperrectangle hyperrectangle(Eigen::Vector2d{-1, -2},
                                      Eigen::Vector2d{3, 2});
  const int n_samples = 1000;
  RandomGenerator generator(1234);
  Eigen::VectorXd sum_samples = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd sum_samples_squared = Eigen::VectorXd::Zero(2);
  for (int i = 0; i < n_samples; ++i) {
    const auto sample = hyperrectangle.UniformSample(&generator);
    sum_samples += sample;
    sum_samples_squared += sample.array().square().matrix();
    EXPECT_TRUE(hyperrectangle.PointInSet(sample, 1e-12));
  }
  // Check the mean of the samples.
  EXPECT_TRUE(
      CompareMatrices(sum_samples / n_samples, hyperrectangle.Center(), 1e-1));
  // The variance of the samples.
  const auto variance = sum_samples_squared / n_samples -
                        (sum_samples / n_samples).array().square().matrix();
  EXPECT_TRUE(
      CompareMatrices(variance, Eigen::Vector2d{16.0 / 12, 16.0 / 12}, 1e-1));
}

GTEST_TEST(HyperrectangleTest, InvariantTest) {
  // Cannot make a hyperrectangle with nan.
  EXPECT_THROW(
      Hyperrectangle(Vector3d{1, 2, 3},
                     Vector3d{1, 2, std::numeric_limits<double>::quiet_NaN()}),
      std::exception);
  EXPECT_THROW(
      Hyperrectangle(Vector3d{std::numeric_limits<double>::quiet_NaN(), 2, 3},
                     Vector3d{1, 2, 3}),
      std::exception);
  // Cannot make an empty hyperrectangle
  EXPECT_THROW(Hyperrectangle(Vector3d{1, 2, 3}, Vector3d{1, 2, -1}),
               std::exception);

  // Cannot make an unbounded hyperrectangle
  EXPECT_THROW(Hyperrectangle(
                   Vector3d::Constant(-std::numeric_limits<double>::infinity()),
                   Vector3d::Constant(1)),
               std::exception);
  EXPECT_THROW(Hyperrectangle(
                   -Vector3d::Constant(1),
                   Vector3d::Constant(std::numeric_limits<double>::infinity())),
               std::exception);
}

GTEST_TEST(HyperrectangleTest, IsBoundedTest) {
  EXPECT_TRUE(
      Hyperrectangle(Vector3d{-1, -2, -3}, Vector3d{3, 2, 1}).IsBounded());
}

GTEST_TEST(HyperrectangleTest, IsEmptyTest) {
  EXPECT_FALSE(
      Hyperrectangle(Vector3d{-1, -2, -3}, Vector3d{3, 2, 1}).IsEmpty());
}

GTEST_TEST(HyperrectangleTest, MaybeGetIntersection) {
  const Vector3d lb1{-1, -2, -3};
  const Vector3d ub1{3, 2, 1};
  const Vector3d lb2{-2, 1, -2};
  const Vector3d ub2{4, 1, 0};
  const Hyperrectangle h1(lb1, ub1);
  const Hyperrectangle h2(lb2, ub2);
  const std::optional<Hyperrectangle> intersection12 =
      h1.MaybeGetIntersection(h2);
  const std::optional<Hyperrectangle> intersection21 =
      h2.MaybeGetIntersection(h1);
  EXPECT_TRUE(intersection12->IsBounded());
  EXPECT_FALSE(intersection12->IsEmpty());
  EXPECT_TRUE(CompareMatrices(intersection12->lb(), Vector3d{-1, 1, -2}));
  EXPECT_TRUE(CompareMatrices(intersection12->ub(), Vector3d{3, 1, 0}));
  EXPECT_TRUE(CompareMatrices(intersection12->lb(), intersection21->lb()));
  EXPECT_TRUE(CompareMatrices(intersection12->ub(), intersection21->ub()));

  // Empty intersection yields nullopt.
  const Vector3d lb3{-30, -20, -10};
  const Vector3d ub3{-20, -11, -8};
  const Hyperrectangle h3(lb3, ub3);
  const std::optional<Hyperrectangle> intersection13 =
      h1.MaybeGetIntersection(h3);
  const std::optional<Hyperrectangle> intersection31 =
      h3.MaybeGetIntersection(h1);
  EXPECT_FALSE(intersection13.has_value());

  // Error: mismatched dimension of h4 and h1.
  const Vector2d lb4{-1, 2};
  const Vector2d ub4{20, 11};
  const Hyperrectangle h4(lb4, ub4);
  EXPECT_THROW(unused(h4.MaybeGetIntersection(h1)), std::exception);
}

GTEST_TEST(HyperrectangleTest, AddPointInSetConstraints) {
  const Vector3d lb{-1, -2, -3};
  const Vector3d ub{3, 2, 1};
  const Hyperrectangle hyperrectangle(lb, ub);
  // Verify the point in set program is feasible.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables(3, "x");
  auto [new_vars, con] = hyperrectangle.AddPointInSetConstraints(&prog, x);
  // There should be no new variables.
  EXPECT_EQ(new_vars.size(), 0);
  // Inside the rectangle.
  prog.SetInitialGuess(x, Vector3d::Zero());
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(con[0]));
  // At the edge of the rectangle.
  prog.SetInitialGuess(x, Vector3d(-1, -2, 1));
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(con[0]));
  // outside the rectangle
  prog.SetInitialGuess(x, Vector3d(3, 2, -4));
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(con[0]));
}

// Test AddPointInNonnegativeScalingConstraints.
GTEST_TEST(HyperrectangleTest, AddPointInNonnegativeScalingConstraints) {
  // Test AddPointInNonnegativeScalingConstraints
  const Vector3d lb{-1, -2, -3};
  const Vector3d ub{3, 2, 1};
  const Hyperrectangle hyperrectangle(lb, ub);
  MathematicalProgram prog;
  const auto t = prog.NewContinuousVariables(1, "t")[0];
  const auto x = prog.NewContinuousVariables(3, "x");
  auto scaled_con =
      hyperrectangle.AddPointInNonnegativeScalingConstraints(&prog, x, t);
  // 2 constraints + 1 added for t>0
  EXPECT_EQ(scaled_con.size(), 3);
  // Inside the zero rectangle.
  prog.SetInitialGuess(x, Vector3d::Zero());
  prog.SetInitialGuess(t, 0);
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(scaled_con));
  // At the edge of the rectangle.
  prog.SetInitialGuess(x, Vector3d(0.2, -0.8, 0.4));
  prog.SetInitialGuess(t, 0.4);
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(scaled_con));
  // Outside of the rectangle from upper bound side
  prog.SetInitialGuess(x, Vector3d(1, 1, 1));
  prog.SetInitialGuess(t, 0.5);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(scaled_con));
}

// Test AddPointInNonnegativeScalingConstraints with matrices.
GTEST_TEST(HyperrectangleTest,
           AddPointInNonnegativeScalingConstraintsWithMatrices) {
  const Vector3d lb{-1, -2, -3};
  const Vector3d ub{3, 2, 1};
  const Hyperrectangle hyperrectangle(lb, ub);
  MathematicalProgram prog;
  Eigen::MatrixXd A(3, 2), b(3, 1), c(3, 1);
  A << 1, 2, 0, 1, -2, 1;
  b << 5, 1, -2;
  c << 1, -1, 2;
  const double d = 2;
  const auto t = prog.NewContinuousVariables(3, "t");
  const auto x = prog.NewContinuousVariables(2, "x");
  auto scaled_matrix_con =
      hyperrectangle.AddPointInNonnegativeScalingConstraints(&prog, A, b, c, d,
                                                             x, t);
  EXPECT_EQ(scaled_matrix_con.size(), 3);
  // Ax + b \in  (c't + d) * hyperrectangle or  test_point := (Ax + b) / (c't +
  // d) \in hyperrectangle
  {
    // test_point = b / 2 = 2.5, 0.5, -1. Inside the hyperrectangle
    const Eigen::Vector2d x_value{0, 0};
    const Eigen::Vector3d t_value{0, 0, 0};
    const auto test_point =
        1 / ((c.transpose() * t_value)[0] + d) * (A * x_value + b);
    EXPECT_TRUE(CompareMatrices(test_point, Eigen::Vector3d(2.5, 0.5, -1)));
    EXPECT_EQ(
        PointInScaledSet(x, t, x_value, t_value, &prog, scaled_matrix_con),
        hyperrectangle.PointInSet(test_point));
  }
  {
    // test_point = (8, 2, -3) / 5. Inside the hyperrectangle
    const Eigen::Vector2d x_value{1, 1};
    const Eigen::Vector3d t_value{4, 2, 0.5};
    const auto test_point =
        1 / ((c.transpose() * t_value)[0] + d) * (A * x_value + b);
    EXPECT_TRUE(CompareMatrices(test_point, 1 / 5.0 * Eigen::Vector3d(8, 2, -3),
                                std::numeric_limits<double>::epsilon()));
    EXPECT_EQ(
        PointInScaledSet(x, t, x_value, t_value, &prog, scaled_matrix_con),
        hyperrectangle.PointInSet(test_point));
  }
  {
    // test_point = (8, 2, -3) / 1. Outside the hyperrectangle
    const Eigen::Vector2d x_value{1, 1};
    const Eigen::Vector3d t_value{4, 5, 0};
    const auto test_point =
        1 / ((c.transpose() * t_value)[0] + d) * (A * x_value + b);
    EXPECT_TRUE(CompareMatrices(test_point, Eigen::Vector3d(8, 2, -3)));
    EXPECT_EQ(
        PointInScaledSet(x, t, x_value, t_value, &prog, scaled_matrix_con),
        hyperrectangle.PointInSet(test_point));
  }
}

// Tests the computation of the minimal axis-aligned bounding box of a
// polyhedron.
GTEST_TEST(HyperrectangleTest, AxisAlignedBoundingBox) {
  // Case: Unbounded.
  {
    Matrix<double, 3, 2> A;
    Matrix<double, 3, 1> b;
    // clang-format off
    A <<  1,  1,  // x + y ≤ 1
         -1,  0,  // x ≥ -2
         -1, -1;  // x+y ≥ -1
    b << 1, 2, 1;
    // clang-format on
    HPolyhedron H(A, b);
    EXPECT_FALSE(
        Hyperrectangle::MaybeCalcAxisAlignedBoundingBox(H).has_value());
  }

  // Case: Bounded parallelogram.
  {
    Matrix<double, 4, 2> A;
    Matrix<double, 4, 1> b;
    // clang-format off
    A <<  1,  0,  // x ≤ 1
          1,  1,  // x + y ≤ 1
         -1,  0,  // x ≥ -2
         -1, -1;  // x+y ≥ -1
    b << 1, 1, 2, 1;
    // clang-format on
    HPolyhedron H(A, b);
    std::optional<Hyperrectangle> aabb_opt =
        Hyperrectangle::MaybeCalcAxisAlignedBoundingBox(H);
    EXPECT_TRUE(aabb_opt.has_value());
    const auto& aabb = aabb_opt.value();
    EXPECT_NEAR(aabb.lb()(0), -2, 1e-6);
    EXPECT_NEAR(aabb.ub()(0), 1, 1e-6);
    EXPECT_NEAR(aabb.lb()(1), -2, 1e-6);
    EXPECT_NEAR(aabb.ub()(1), 3, 1e-6);
  }
}

GTEST_TEST(HyperrectangleTest, UnboundedEllipsoid) {
  // Zero matrix in hyperellipsoid is unbounded.
  Hyperellipsoid H(Eigen::MatrixXd::Zero(2, 2), Eigen::VectorXd::Zero(2));
  const auto aabb = Hyperrectangle::MaybeCalcAxisAlignedBoundingBox(H);
  EXPECT_FALSE(aabb.has_value());
}

GTEST_TEST(HyperrectangleTest, Serialize) {
  const Vector3d lb{-1, -2, -3};
  const Vector3d ub{3, 2, 1};
  const Hyperrectangle hyperrectangle(lb, ub);
  const std::string yaml = yaml::SaveYamlString(hyperrectangle);
  const auto hyperrectangle2 = yaml::LoadYamlString<Hyperrectangle>(yaml);
  EXPECT_EQ(hyperrectangle.ambient_dimension(),
            hyperrectangle2.ambient_dimension());
  EXPECT_TRUE(CompareMatrices(hyperrectangle.lb(), hyperrectangle2.lb()));
  EXPECT_TRUE(CompareMatrices(hyperrectangle.ub(), hyperrectangle2.ub()));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
