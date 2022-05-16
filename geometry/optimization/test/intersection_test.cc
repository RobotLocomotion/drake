#include "drake/geometry/optimization/intersection.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using math::RigidTransformd;

GTEST_TEST(IntersectionTest, BasicTest) {
  const Point P1(Vector2d{0.1, 1.2});
  HPolyhedron H1 = HPolyhedron::MakeBox(Vector2d{0, 0}, Vector2d{2, 2});
  Intersection S(P1, H1);
  EXPECT_EQ(S.num_elements(), 2);
  EXPECT_EQ(S.ambient_dimension(), 2);

  // Test PointInSet.
  Vector2d in, out;
  in << P1.x();
  out << P1.x() + Vector2d::Constant(0.01);
  EXPECT_TRUE(S.PointInSet(in));
  EXPECT_FALSE(S.PointInSet(out));

  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(S, in));
  EXPECT_FALSE(internal::CheckAddPointInSetConstraints(S, out));

  // Test IsBounded.
  EXPECT_TRUE(S.IsBounded());

  // Test ConvexSets constructor.
  ConvexSets sets;
  sets.emplace_back(P1);
  sets.emplace_back(H1);
  const Intersection S2(sets);
  EXPECT_EQ(S2.num_elements(), 2);
  EXPECT_EQ(S2.ambient_dimension(), 2);
  EXPECT_TRUE(S2.PointInSet(in));
  EXPECT_FALSE(S2.PointInSet(out));
}

GTEST_TEST(IntersectionTest, TwoBoxes) {
  HPolyhedron H1 = HPolyhedron::MakeBox(Vector2d{0, 0}, Vector2d{2, 2});
  HPolyhedron H2 = HPolyhedron::MakeBox(Vector2d{1, -1}, Vector2d{3, 1});
  Intersection S(H1, H2);
  EXPECT_TRUE(S.PointInSet(Vector2d{1.9, 0.9}));
  EXPECT_FALSE(S.PointInSet(Vector2d{1.9, 1.1}));
  EXPECT_FALSE(S.PointInSet(Vector2d{2.1, 0.9}));
}

GTEST_TEST(IntersectionTest, BoundedTest) {
  HPolyhedron H1 = HPolyhedron(Matrix2d::Identity(), Vector2d{1, 1});
  HPolyhedron H2 = HPolyhedron(-Matrix2d::Identity(), Vector2d{1, 1});
  Intersection S(H1, H2);

  EXPECT_FALSE(H1.IsBounded());
  EXPECT_FALSE(H2.IsBounded());
  DRAKE_EXPECT_THROWS_MESSAGE(
      S.IsBounded(),
      "Determining the boundedness of an Intersection made up of unbounded "
      "elements is not currently supported.");
}

GTEST_TEST(IntersectionTest, CloneTest) {
  const Point P1(Vector2d{0.1, 1.2});
  HPolyhedron H1 = HPolyhedron::MakeBox(Vector2d{0, 0}, Vector2d{2, 2});
  Intersection S(P1, H1);
  std::unique_ptr<ConvexSet> clone = S.Clone();
  EXPECT_EQ(clone->ambient_dimension(), 2);

  Intersection* s = dynamic_cast<Intersection*>(clone.get());
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(s->num_elements(), 2);
  const Point* p = dynamic_cast<const Point*>(&s->element(0));
  ASSERT_NE(p, nullptr);
  EXPECT_TRUE(CompareMatrices(P1.x(), p->x()));
}

GTEST_TEST(IntersectionTest, NonnegativeScalingTest) {
  const Point P1(Vector2d{0.1, 1.2});
  HPolyhedron H1 = HPolyhedron::MakeBox(Vector2d{0, 0}, Vector2d{2, 2});
  Intersection S(P1, H1);

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];

  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      S.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  // 1 vector constraint from P1, 1 from H1, and t>=0 3 times.
  EXPECT_EQ(constraints.size(), 5);

  const double tol = 1e-16;
  for (const double scale : {0.5, 1.0, 2.0}) {
    prog.SetInitialGuess(x, scale * P1.x());
    prog.SetInitialGuess(t, scale);
    EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
    prog.SetInitialGuess(x, 0.99 * scale * P1.x());
    EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
  }
  prog.SetInitialGuess(x, P1.x());
  prog.SetInitialGuess(t, -1.0);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
}

bool PointInScaledSet(const solvers::VectorXDecisionVariable& x_vars,
                      const solvers::VectorXDecisionVariable& t_var,
                      const Vector2d& x, const Vector2d& t,
                      solvers::MathematicalProgram* prog) {
  auto b1 = prog->AddBoundingBoxConstraint(x, x, x_vars);
  auto b2 = prog->AddBoundingBoxConstraint(t, t, t_var);
  auto result = solvers::Solve(*prog);
  prog->RemoveConstraint(b1);
  prog->RemoveConstraint(b2);
  return result.is_success();
}

GTEST_TEST(IntersectionTest, NonnegativeScalingTest2) {
  const Point P1(Vector2d{0.1, 1.2});
  HPolyhedron H1 = HPolyhedron::MakeBox(Vector2d{0, 0}, Vector2d{2, 2});
  Intersection S(P1, H1);

  solvers::MathematicalProgram prog;
  Eigen::MatrixXd A(2, 2);
  // clang-format off
  A << 0, 1,
       1, 0;
  // clang-format on
  Eigen::Vector2d b = Eigen::Vector2d::Zero();
  auto x = prog.NewContinuousVariables(2, "x");
  Eigen::Vector2d c(1, -1);
  double d = 0;
  auto t = prog.NewContinuousVariables(2, "t");

  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      S.AddPointInNonnegativeScalingConstraints(&prog, A, b, c, d, x, t);

  // 1 vector constraint from P1, 1 from H1, and c'*t+d>=0 3 times.
  EXPECT_EQ(constraints.size(), 5);

  Eigen::Vector2d x_solution(1.2, 0.1);
  Eigen::Vector3d scale(0.5, 1.0, 2.0);
  Eigen::MatrixXd t_solutions(2, 9);
  // clang-format off
  t_solutions << 0.5,  0,  0.25, 1,  0,  0.5, 2,  0,  1,
                 0, -0.5, -0.25, 0, -1, -0.5, 0, -2, -1;
  // clang-format on
  for (int ii = 0; ii < t_solutions.cols(); ++ii) {
    EXPECT_TRUE(PointInScaledSet(x, t, scale[ii / 3] * x_solution,
                                 t_solutions.col(ii), &prog));
    EXPECT_FALSE(PointInScaledSet(x, t, 0.99 * scale[ii / 3] * x_solution,
                                  t_solutions.col(ii), &prog));
  }

  EXPECT_FALSE(
      PointInScaledSet(x, t, x_solution, Eigen::Vector2d(0, 1), &prog));
  EXPECT_FALSE(
      PointInScaledSet(x, t, x_solution, Eigen::Vector2d(-1, 0), &prog));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
