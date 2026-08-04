#include "drake/geometry/optimization/intersection.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

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

  // Test MaybeGetPoint. Even though the logical intersection *does* represent a
  // point, our implementation doesn't yet have any implementation tactic that
  // would identify that condition quickly.
  EXPECT_FALSE(S.MaybeGetPoint().has_value());

  // Test PointInSet.
  Vector2d in, out;
  in << P1.x();
  out << P1.x() + Vector2d::Constant(0.01);
  EXPECT_TRUE(S.PointInSet(in));
  EXPECT_FALSE(S.PointInSet(out));

  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(S, in));
  EXPECT_FALSE(internal::CheckAddPointInSetConstraints(S, out));
  {
    solvers::MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<2>();
    const auto [new_vars, new_constraints] =
        S.AddPointInSetConstraints(&prog, x);
    EXPECT_GE(new_constraints.size(), 2);
  }

  // Test IsBounded.
  EXPECT_TRUE(S.IsBounded(Parallelism::None()));
  EXPECT_TRUE(S.IsBounded(Parallelism::Max()));

  // Test IsEmpty
  EXPECT_FALSE(S.IsEmpty());

  // Test MaybeGetFeasiblePoint.
  ASSERT_TRUE(S.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(S.PointInSet(S.MaybeGetFeasiblePoint().value()));

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

GTEST_TEST(IntersectionTest, TwoIdenticalPoints) {
  const Point P1(Vector2d{0.1, 1.2});
  Intersection S(P1, P1);
  EXPECT_EQ(S.ambient_dimension(), 2);
  ASSERT_TRUE(S.MaybeGetPoint().has_value());
  EXPECT_TRUE(CompareMatrices(S.MaybeGetPoint().value(), P1.x()));
  EXPECT_TRUE(S.PointInSet(P1.x()));
  EXPECT_FALSE(S.IsEmpty());
  ASSERT_TRUE(S.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(S.PointInSet(S.MaybeGetFeasiblePoint().value()));
}

GTEST_TEST(IntersectionTest, DefaultCtor) {
  const Intersection dut;
  EXPECT_EQ(dut.num_elements(), 0);
  EXPECT_NO_THROW(dut.Clone());
  EXPECT_EQ(dut.ambient_dimension(), 0);
  EXPECT_TRUE(dut.IntersectsWith(dut));
  EXPECT_TRUE(dut.IsBounded(Parallelism::None()));
  EXPECT_TRUE(dut.IsBounded(Parallelism::Max()));
  EXPECT_FALSE(dut.IsEmpty());
  EXPECT_TRUE(dut.MaybeGetPoint().has_value());
  EXPECT_TRUE(dut.PointInSet(Eigen::VectorXd::Zero(0)));
  ASSERT_TRUE(dut.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(dut.PointInSet(dut.MaybeGetFeasiblePoint().value()));
}

GTEST_TEST(IntersectionTest, Move) {
  const Point P1(Vector2d{0.1, 1.2});
  HPolyhedron H1 = HPolyhedron::MakeBox(Vector2d{0, 0}, Vector2d{2, 2});
  Intersection orig(P1, H1);

  // A move-constructed Intersection takes over the original data.
  Intersection dut(std::move(orig));
  EXPECT_EQ(dut.num_elements(), 2);
  EXPECT_EQ(dut.ambient_dimension(), 2);

  // The old set is in a valid but unspecified state. For convenience we'll
  // assert that it's empty, but that's not the only valid implementation,
  // just the one we happen to currently use.
  EXPECT_EQ(orig.num_elements(), 0);
  EXPECT_EQ(orig.ambient_dimension(), 0);
  EXPECT_NO_THROW(orig.Clone());
}

GTEST_TEST(IntersectionTest, TwoBoxes) {
  HPolyhedron H1 = HPolyhedron::MakeBox(Vector2d{0, 0}, Vector2d{2, 2});
  HPolyhedron H2 = HPolyhedron::MakeBox(Vector2d{1, -1}, Vector2d{3, 1});
  Intersection S(H1, H2);
  EXPECT_TRUE(S.PointInSet(Vector2d{1.9, 0.9}));
  EXPECT_FALSE(S.PointInSet(Vector2d{1.9, 1.1}));
  EXPECT_FALSE(S.PointInSet(Vector2d{2.1, 0.9}));
  EXPECT_FALSE(S.MaybeGetPoint().has_value());
  EXPECT_FALSE(S.IsEmpty());
  ASSERT_TRUE(S.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(S.PointInSet(S.MaybeGetFeasiblePoint().value()));
}

GTEST_TEST(IntersectionTest, BoundedTest) {
  HPolyhedron H1 = HPolyhedron(Matrix2d::Identity(), Vector2d{1, 1});
  HPolyhedron H2 = HPolyhedron(-Matrix2d::Identity(), Vector2d{1, 1});
  Intersection S1(H1, H2);

  EXPECT_FALSE(H1.IsBounded());
  EXPECT_FALSE(H2.IsBounded());
  EXPECT_TRUE(S1.IsBounded(Parallelism::None()));
  EXPECT_TRUE(S1.IsBounded(Parallelism::Max()));

  Intersection S2(H1, H1);
  EXPECT_FALSE(S2.IsBounded(Parallelism::None()));
  EXPECT_FALSE(S2.IsBounded(Parallelism::Max()));
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

GTEST_TEST(IntersectionTest, EmptyIntersectionTest1) {
  Eigen::MatrixXd A{5, 3};
  Eigen::VectorXd b{5};
  // Rows 1-3 define an infeasible set of inequalities.
  // clang-format off
  A << 1, 0, 0,
       1, -1, 0,
       -1, 0, 1,
       0, 1, -1,
       0, 0, -1;
  b << 1, -1, -1, -1, 0;
  // clang-format on

  HPolyhedron H1{A, b};
  const Point P1(Vector3d{0.1, 1.2, 0.3});
  Intersection S(P1, H1);

  EXPECT_TRUE(S.IsEmpty());
}

GTEST_TEST(IntersectionTest, EmptyIntersectionTest2) {
  const Point P1(Vector2d{0.1, 1.2});
  const Point P2(Vector2d{0.1, 4.3});
  Intersection S(P1, P2);

  EXPECT_TRUE(S.IsEmpty());
  EXPECT_FALSE(S.MaybeGetFeasiblePoint().has_value());
}

GTEST_TEST(IntersectionTest, EmptyInput) {
  const Point P(Vector2d{1.2, 3.4});
  // A VPolytope constructed from an empty list of vertices is empty.
  Eigen::Matrix<double, 2, 0> vertices;
  const VPolytope V = VPolytope(vertices);
  Intersection S(P, V);
  EXPECT_TRUE(S.IsEmpty());
}

GTEST_TEST(IntersectionTest, BoundedTest2) {
  // A higher-dimensional, more complicated boundedness check that can't use the
  // shortcut, since both constituent sets are unbounded. Note that this is
  // implicitly testing ConvexSet::GenericDoIsBounded, since we can't easily
  // test that functionality in convex_set_test.cc.
  HPolyhedron l1 = HPolyhedron::MakeUnitBox(100);
  Eigen::MatrixXd A = l1.A();
  Eigen::VectorXd b = l1.b();

  // Put half of the rows in one HPolyhedron, and the other half in another.
  // Also make a variant where one row is skipped, to make it unbounded. Because
  // each of these three sets are unbounded, the shortcut boundedness check
  // cannot be called, so the generic check has to be performed. Note: we cannot
  // guarantee that we've fallen back to the GenericDoIsBounded method, or that
  // multiple threads are actually being used by that method.
  ASSERT_EQ(A.rows(), 200);
  HPolyhedron half1(A.topRows(100), b.head(100));
  HPolyhedron half2(A.bottomRows(100), b.tail(100));
  HPolyhedron half2_unbounded(A.bottomRows(99), b.tail(99));

  // Cross-check that our BUILD file allows parallelism.
  DRAKE_DEMAND(Parallelism::Max().num_threads() > 1);

  Intersection bounded(half1, half2);
  Intersection unbounded(half1, half2_unbounded);

  EXPECT_TRUE(bounded.IsBounded(Parallelism::Max()));
  EXPECT_FALSE(unbounded.IsBounded(Parallelism::Max()));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
