#include "drake/geometry/optimization/convex_hull.h"

#include <gtest/gtest.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

GTEST_TEST(ConvexHullTest, BasicTests) {
  // Empty convex hull without any sets.
  ConvexHull null_hull({});
  EXPECT_TRUE(null_hull.IsEmpty());
  EXPECT_TRUE(null_hull.IsBounded());
  EXPECT_FALSE(null_hull.MaybeGetPoint().has_value());
  EXPECT_EQ(null_hull.num_elements(), 0);
  EXPECT_EQ(null_hull.ambient_dimension(), 0);
  // Convex hull with a point and a rectangle.
  const Point point(Eigen::Vector2d(1.0, 2.0));
  const Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                                 Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(point, rectangle));
  EXPECT_EQ(hull.num_elements(), 2);
  EXPECT_EQ(hull.ambient_dimension(), 2);
  EXPECT_FALSE(hull.IsEmpty());
  EXPECT_TRUE(hull.IsBounded());
  EXPECT_FALSE(hull.MaybeGetPoint().has_value());
  // Inppropriate dimensions.
  Point point_3d = Point(Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_THROW(ConvexHull(MakeConvexSets(point, point_3d)), std::runtime_error);
}

GTEST_TEST(ConvexHullTest, CheckEmpty) {
  const Point point(Eigen::Vector2d(1.0, 2.0));
  const Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                                 Eigen::Vector2d(1.0, 1.0));
  // Add an HPolyhedron that is empty.
  Eigen::MatrixXd A(2, 2);
  A << 1, 0, -1, 0;
  Eigen::VectorXd b(2);
  b << -1, -1;
  HPolyhedron empty_hpolyhedron(A, b);
  ConvexHull hull_1(MakeConvexSets(point, rectangle, empty_hpolyhedron));
  EXPECT_FALSE(hull_1.IsEmpty());
  EXPECT_TRUE(hull_1.empty_sets_removed());
  EXPECT_EQ(hull_1.participating_sets().size(), 2);
  EXPECT_TRUE(hull_1.PointInSet(Eigen::Vector2d(0.0, 1.5), 1e-6));
  // [0, 3] is not in the convex hull.
  EXPECT_FALSE(hull_1.PointInSet(Eigen::Vector2d(0.0, 3.0), 1e-6));
  ConvexHull hull_2(MakeConvexSets(point, rectangle, empty_hpolyhedron), false);
  EXPECT_FALSE(hull_2.IsEmpty());
  EXPECT_FALSE(hull_2.empty_sets_removed());
  EXPECT_EQ(hull_2.participating_sets().size(), 3);
  EXPECT_TRUE(hull_2.PointInSet(Eigen::Vector2d(0.0, 1.5), 1e-6));
  // Unexpected behavior because the check was bypassed.
  // [0, 3] is not in the convex hull, but it says it is because
  // 0*empty_hpolyhedron contains the line.
  EXPECT_TRUE(hull_2.PointInSet(Eigen::Vector2d(0.0, 3.0), 1e-6));
  // If only the empty_hpolyhedron is added, the convex hull should be empty.
  ConvexHull hull_3(MakeConvexSets(empty_hpolyhedron));
  EXPECT_TRUE(hull_3.IsEmpty());
}

GTEST_TEST(ConvexHullTest, PointInSet1) {
  // Case with 1 set in 2D.
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(rectangle));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(0.0, 1.0), 1e-6));
  EXPECT_FALSE(hull.PointInSet(Eigen::Vector2d(1.0, 1.1), 1e-6));
  // Test tolerance.
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(1.0, 1.1), 0.1));
}

GTEST_TEST(ConvexHullTest, PointInSet2) {
  // Case with 3 sets in 2D.
  Point point1(Eigen::Vector2d(0.0, 0.0));
  Point point2(Eigen::Vector2d(1.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(point1, point2, rectangle));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(0.0, 0.0), 1e-6));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(1.0, 1.0), 1e-6));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(0.9, 0.5), 1e-6));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(-0.5, 0.5), 1e-6));
  EXPECT_FALSE(hull.PointInSet(Eigen::Vector2d(1.1, 0.5), 1e-6));
  // Test tolerances.
  EXPECT_FALSE(hull.PointInSet(Eigen::Vector2d(-0.501, 0.5), 1e-4));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(-0.501, 0.5), 1e-2));
}

GTEST_TEST(ConvexHullTest, AddPointInSetConstraints1) {
  Point point(Eigen::Vector2d(0.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(point, rectangle));
  EXPECT_TRUE(
      internal::CheckAddPointInSetConstraints(hull, Eigen::Vector2d(0.4, 0.4)));
  EXPECT_FALSE(
      internal::CheckAddPointInSetConstraints(hull, Eigen::Vector2d(0.6, 0.4)));
}

GTEST_TEST(ConvexHullTest, AddPointInSetConstraints2) {
  Point point(Eigen::Vector2d(0.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(point, rectangle));
  // Solves a mathematical program that finds a point in the convex hull with
  // least L2 distance to (0.8,0) The result should be (0.4, 0.4).
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  auto [new_vars, new_constraints] = hull.AddPointInSetConstraints(&prog, x);
  // How many new variables are added?
  // 2 alphas, 2*2 x variables. Total 6.
  EXPECT_EQ(new_vars.size(), 6);
  prog.AddQuadraticCost((x - Eigen::Vector2d(0.8, 0.0)).squaredNorm());
  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  const Eigen::VectorXd x_sol = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(x_sol, Eigen::Vector2d(0.4, 0.4), 1e-6));
}

GTEST_TEST(ConvexHullTest, AddPointInSetConstraints3) {
  // Makes convex hull from a point and another convex hull. Calls
  // AddPointInNonnegativeScalingConstraints for the second convex hull.
  Point point1(Eigen::Vector2d(0.0, 0.0));
  Point point2(Eigen::Vector2d(0.5, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull1(MakeConvexSets(point1, rectangle));
  ConvexHull hull2(MakeConvexSets(hull1, point2));
  // We know that (0.5, 0) to (1.0, 1.0) becomes a face
  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(
      hull2, Eigen::Vector2d(0.3, 0.0)));
  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(
      hull2, Eigen::Vector2d(0.6, 0.2)));
  EXPECT_FALSE(internal::CheckAddPointInSetConstraints(
      hull2, Eigen::Vector2d(0.6, 0.1)));
}

GTEST_TEST(ConvexHullTest, AddPointInNonnegativeScalingConstraints1) {
  Point point(Eigen::Vector2d(0.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  // Add an HPolyhedron that is empty.
  Eigen::MatrixXd A(2, 2);
  A << 1, 0, -1, 0;
  Eigen::VectorXd b(2);
  b << 1, -2;
  HPolyhedron empty_hpolyhedron(A, b);
  ConvexHull hull(MakeConvexSets(point, rectangle, empty_hpolyhedron));
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  auto t = prog.NewContinuousVariables(1, "t");
  auto new_constraints =
      hull.AddPointInNonnegativeScalingConstraints(&prog, x, t(0));
  EXPECT_GT(new_constraints.size(), 0);
  // Solve the closest point to (2.0, 1.0). The closest point is (1.5, 1.5) when
  // t = 1.5.
  prog.AddQuadraticCost((x - Eigen::Vector2d(2.0, 1.0)).squaredNorm());
  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  const Eigen::VectorXd x_sol = result.GetSolution(x);
  const Eigen::VectorXd t_sol = result.GetSolution(t);
  EXPECT_TRUE(CompareMatrices(x_sol, Eigen::Vector2d(1.5, 1.5), 1e-4));
  EXPECT_GE(t_sol(0), 1.5);
  // Adding negative constraints on t leads to infeasibility.
  prog.AddLinearConstraint(t(0) <= -1.0);
  const auto result2 = Solve(prog);
  EXPECT_FALSE(result2.is_success());
}

GTEST_TEST(ConvexHullTest, AddPointInNonnegativeScalingConstraints2) {
  // Verify by solving a 2D problem and verify the solution.
  Point point1(Eigen::Vector2d(0.0, 0.0));
  Point point2(Eigen::Vector2d(0.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 2.0));
  ConvexHull hull(MakeConvexSets(point1, point2, rectangle));
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  auto t = prog.NewContinuousVariables(3, "t");
  // CCW 90 degree rotation matrix + shift y by 2.0.
  Eigen::MatrixXd A(2, 2);
  A << 0, -1, 1, 0;
  Eigen::Vector2d b(0.0, 2.0);
  // Select a 3d vector c = [1, 2, -1] and d = 5.0, just to make the problem
  // more interesting.
  Eigen::Vector3d c(1.0, 1.0, 1.0);
  const double d = 0.4;
  auto new_constraints =
      hull.AddPointInNonnegativeScalingConstraints(&prog, A, b, c, d, x, t);
  EXPECT_GT(new_constraints.size(), 0);
  // Pick a point: (-1.7 + a, -0.6). Ax+b will be (0.6, 0.3 + a). It would not
  // be in the convex hull for a = 0.
  auto a = prog.NewContinuousVariables(1, "a");
  prog.AddLinearEqualityConstraint(x(0) == -1.7 + a(0));
  prog.AddLinearEqualityConstraint(x(1) == -0.6);
  // It would be in the convex hull for a = 0.3. The smallest (c't + d) that
  // would allow this is 0.6.
  prog.AddLinearCost(a(0));
  prog.AddL2NormCost(Eigen::MatrixXd::Identity(3, 3), Eigen::Vector3d::Zero(),
                     t);
  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  const Eigen::VectorXd t_sol = result.GetSolution(t);
  EXPECT_NEAR(result.GetSolution(a)(0), 0.3, 1e-6);
  EXPECT_NEAR(c.transpose() * t_sol + d, 0.6, 1e-6);
  // We know the t solution, all elements will be 0.2/3.
  EXPECT_TRUE(CompareMatrices(t_sol, 0.2 / 3 * Eigen::Vector3d::Ones(), 1e-6));
}

GTEST_TEST(ConvexHullTest, BoundedTest) {
  HPolyhedron H1 = HPolyhedron::MakeL1Ball(3);
  HPolyhedron H2 =
      HPolyhedron(-Eigen::Matrix3d::Identity(), Eigen::Vector3d{1, 1, 1});
  ConvexHull C1(MakeConvexSets(H1, H2));

  EXPECT_FALSE(C1.IsBounded(Parallelism::None()));
  EXPECT_FALSE(C1.IsBounded(Parallelism::Max()));

  ConvexHull C2(MakeConvexSets(H1, H1));
  EXPECT_TRUE(C2.IsBounded(Parallelism::None()));
  EXPECT_TRUE(C2.IsBounded(Parallelism::Max()));

  // Check a high dimensional example.
  HPolyhedron H3 = HPolyhedron::MakeUnitBox(100);
  Eigen::MatrixXd A = H3.A();
  Eigen::VectorXd b = H3.b();

  ASSERT_EQ(A.rows(), 200);
  HPolyhedron H4(A.topRows(199), b.head(199));

  ConvexHull C3(MakeConvexSets(H3, H3));  // bounded
  ConvexHull C4(MakeConvexSets(H3, H4));  // unbounded

  EXPECT_TRUE(C3.IsBounded(Parallelism::Max()));
  EXPECT_FALSE(C4.IsBounded(Parallelism::Max()));

  // See also intersection_test.cc for more extensive testing.
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
