#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <forward_list>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::RowVector2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using solvers::Binding;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::MathematicalProgramResult;
using solvers::QuadraticCost;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Substitution;
using symbolic::Variables;

using Edge = GraphOfConvexSets::Edge;
using Vertex = GraphOfConvexSets::Vertex;
using VertexId = GraphOfConvexSets::VertexId;

using ::testing::AllOf;
using ::testing::HasSubstr;
using ::testing::Not;

namespace {
  bool MixedIntegerSolverAvailable() {
    return (solvers::MosekSolver::is_available() &&
            solvers::MosekSolver::is_enabled()) ||
           (solvers::GurobiSolver::is_available() &&
            solvers::GurobiSolver::is_enabled());
  }
}

GTEST_TEST(GraphOfConvexSetsTest, AddVertex) {
  GraphOfConvexSets g;
  Point p(Vector3d(1.0, 2.0, 3.0));
  Vertex* v = g.AddVertex(p, "point");

  EXPECT_EQ(v->ambient_dimension(), 3);
  EXPECT_EQ(v->name(), "point");

  EXPECT_EQ(v->x().size(), 3);
  EXPECT_EQ(v->x()[0].get_name(), "point(0)");
  EXPECT_TRUE(v->set().PointInSet(p.x()));

  // Confirm that the vertex set does not refer to the original memory.
  p.set_x(Vector3d(4., 5., 6));
  EXPECT_FALSE(v->set().PointInSet(p.x()));

  auto vertices = g.Vertices();
  EXPECT_EQ(vertices.size(), 1);
  EXPECT_EQ(vertices.at(0), v);

  const GraphOfConvexSets* const_g = &g;
  const auto const_vertices = const_g->Vertices();
  EXPECT_EQ(const_vertices.size(), 1);
  EXPECT_EQ(const_vertices.at(0), v);
}

GTEST_TEST(GraphOfConvexSetsTest, GetVertexSolution) {
  GraphOfConvexSets g;
  Point p(Vector3d(1.0, 2.0, 3.0));
  Vertex* v = g.AddVertex(p, "point");

  MathematicalProgramResult result;
  std::unordered_map<symbolic::Variable::Id, int> map;
  for (int i = 0; i < 3; ++i) {
    map.emplace(v->x()[i].get_id(), i);
  }
  result.set_decision_variable_index(map);
  result.set_x_val(p.x());
  EXPECT_TRUE(CompareMatrices(v->GetSolution(result), p.x()));
}

GTEST_TEST(GraphOfConvexSetsTest, AddEdge) {
  GraphOfConvexSets g;
  Point pu(Vector3d(1.0, 2.0, 3.0));
  Point pv(Vector2d(4., 5.));
  Vertex* u = g.AddVertex(pu, "u");
  Vertex* v = g.AddVertex(pv, "v");
  Edge* e = g.AddEdge(u->id(), v->id(), "e");

  EXPECT_EQ(e->u().name(), u->name());
  EXPECT_EQ(e->v().name(), v->name());

  EXPECT_EQ(Variables(e->xu()), Variables(u->x()));
  EXPECT_EQ(Variables(e->xv()), Variables(v->x()));

  auto edges = g.Edges();
  EXPECT_EQ(edges.size(), 1);
  EXPECT_EQ(edges.at(0), e);

  const GraphOfConvexSets* const_g = &g;
  const auto const_edges = const_g->Edges();
  EXPECT_EQ(const_edges.size(), 1);
  EXPECT_EQ(const_edges.at(0), e);
}

GTEST_TEST(GraphOfConvexSetsTest, AddEdge2) {
  GraphOfConvexSets g;
  Point pu(Vector3d(1.0, 2.0, 3.0));
  Point pv(Vector2d(4., 5.));
  Vertex* u = g.AddVertex(pu, "u");
  Vertex* v = g.AddVertex(pv, "v");
  Edge* e = g.AddEdge(*u, *v, "e");

  EXPECT_EQ(e->u().name(), u->name());
  EXPECT_EQ(e->v().name(), v->name());

  EXPECT_EQ(Variables(e->xu()), Variables(u->x()));
  EXPECT_EQ(Variables(e->xv()), Variables(v->x()));
}


/*
┌───┐       ┌───┐
│ u ├───e──►│ v │
└───┘       └───┘
*/
class TwoPoints : public ::testing::Test {
 protected:
  TwoPoints()
      : pu_{Vector2d(1., 2.)},
        pv_{Vector3d(3., 4., 5.)} {
    u_ = g_.AddVertex(pu_, "u");
    v_ = g_.AddVertex(pv_, "v");
    e_ = g_.AddEdge(*u_, *v_, "e");
  }

  GraphOfConvexSets g_{};
  Point pu_;
  Point pv_;
  Vertex* u_;
  Vertex* v_;
  Edge* e_;
};

TEST_F(TwoPoints, Basic) {
  EXPECT_EQ(e_->name(), "e");
  EXPECT_EQ(e_->u().name(), u_->name());
  EXPECT_EQ(e_->v().name(), v_->name());

  EXPECT_EQ(Variables(e_->xu()), Variables(u_->x()));
  EXPECT_EQ(Variables(e_->xv()), Variables(v_->x()));

  auto vertices = g_.Vertices();
  EXPECT_EQ(vertices.at(0), u_);
  EXPECT_EQ(vertices.at(1), v_);

  EXPECT_EQ(g_.Edges().at(0), e_);
}

// Confirms that I can add costs (both ways) and get the solution.
// The correctness of the added costs will be established by the solution tests.
TEST_F(TwoPoints, AddCost) {
  auto [ell0, b0] = e_->AddCost((e_->xv().head<2>() - e_->xu()).squaredNorm());
  auto cost = std::make_shared<LinearCost>(Vector2d::Zero(), 0.1);
  auto [ell1, b1] = e_->AddCost(Binding(cost, e_->xu()));

  // Confirm that they are down-castable.
  auto quadratic = dynamic_cast<QuadraticCost*>(b0.evaluator().get());
  EXPECT_TRUE(quadratic != nullptr);
  auto linear = dynamic_cast<LinearCost*>(b1.evaluator().get());
  EXPECT_TRUE(linear != nullptr);

  MathematicalProgramResult result;
  std::unordered_map<symbolic::Variable::Id, int> map;
  map.emplace(ell0.get_id(), 0);
  map.emplace(ell1.get_id(), 1);
  const Vector2d ell{1.2, 3.4};
  result.set_decision_variable_index(map);
  result.set_x_val(ell);

  EXPECT_NEAR(e_->GetSolutionCost(result), ell.sum(), 1e-16);

  symbolic::Variable other_var("x");
  DRAKE_EXPECT_THROWS_MESSAGE(e_->AddCost(other_var), ".*IsSubsetOf.*");
}

// Confirms that I can add constraints (both ways).
// The correctness of the added constraints will be established by the solution
// tests.
TEST_F(TwoPoints, AddConstraint) {
  auto b0 = e_->AddConstraint(e_->xv().head<2>() == e_->xu());
  auto constraint = std::make_shared<LinearConstraint>(
      Matrix2d::Identity(), Vector2d::Zero(), Vector2d{1.2, 3.4});
  auto b1 = e_->AddConstraint(Binding(constraint, e_->xu()));

  // Confirm that they are down-castable.
  auto linear_equality =
      dynamic_cast<LinearEqualityConstraint*>(b0.evaluator().get());
  EXPECT_TRUE(linear_equality != nullptr);
  auto linear = dynamic_cast<LinearConstraint*>(b1.evaluator().get());
  EXPECT_TRUE(linear != nullptr);

  symbolic::Variable other_var("x");
  DRAKE_EXPECT_THROWS_MESSAGE(e_->AddConstraint(other_var == 1),
                              ".*IsSubsetOf.*");
}

/*
Let's me test with one edge defintely on the optimal path, and one definitely
off it.
┌──────┐         ┌──────┐
│source├──e_on──►│target│
└───┬──┘         └──────┘
    │e_off
    │
┌───▼──┐
│ sink │
└──────┘
*/
class ThreePoints : public ::testing::Test {
 protected:
  ThreePoints()
      : p_source_{Vector2d(3., 5.)},
        p_target_{Vector2d(-2., 4.)},
        p_sink_{Vector2d(5., -2.3)} {
    source_ = g_.AddVertex(p_source_);
    target_ = g_.AddVertex(p_target_);
    sink_ = g_.AddVertex(p_sink_);
    e_on_ = g_.AddEdge(*source_, *target_);
    e_off_ = g_.AddEdge(*source_, *sink_);

    subs_on_off_.emplace(e_on_->xv()[0], e_off_->xv()[0]);
    subs_on_off_.emplace(e_on_->xv()[1], e_off_->xv()[1]);
  }

  GraphOfConvexSets g_;
  Point p_source_;
  Point p_target_;
  Point p_sink_;
  Vertex* source_{nullptr};
  Vertex* target_{nullptr};
  Vertex* sink_{nullptr};
  Edge* e_on_{nullptr};
  Edge* e_off_{nullptr};
  Substitution subs_on_off_{};
};

TEST_F(ThreePoints, LinearCost1) {
  e_on_->AddCost(1.0);
  e_off_->AddCost(1.0);
  auto result = g_.SolveShortestPath(source_->id(), target_->id(), true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), 1.0, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);

  EXPECT_TRUE(
      CompareMatrices(e_on_->GetSolutionPhiXu(result), p_source_.x(), 1e-6));
  EXPECT_TRUE(
      CompareMatrices(e_on_->GetSolutionPhiXv(result), p_target_.x(), 1e-6));
  EXPECT_TRUE(CompareMatrices(e_off_->GetSolutionPhiXu(result),
                              0 * p_source_.x(), 1e-6));
  EXPECT_TRUE(
      CompareMatrices(e_off_->GetSolutionPhiXv(result), 0 * p_sink_.x(), 1e-6));

  // Alternative signature.
  auto result2 = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result2.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result2), 1.0, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result2), 0.0, 1e-6);
}

TEST_F(ThreePoints, ConvexRelaxation) {
  e_on_->AddCost(1.0);
  e_off_->AddCost(1.0);
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(e_on_->phi()), 1.0, 1e-6);
  EXPECT_NEAR(result.GetSolution(e_off_->phi()), 0.0, 1e-6);

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  auto result2 = g_.SolveShortestPath(*source_, *target_, false);
  ASSERT_TRUE(result2.is_success());
  EXPECT_NEAR(result2.GetSolution(e_on_->phi()), 1.0, 1e-6);
  EXPECT_NEAR(result2.GetSolution(e_off_->phi()), 0.0, 1e-6);

  EXPECT_NEAR(e_on_->GetSolutionCost(result), e_on_->GetSolutionCost(result2),
              1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), e_off_->GetSolutionCost(result2),
              1e-6);
}

TEST_F(ThreePoints, LinearCost2) {
  const Vector0<double> a;
  const double b = 1.23;
  auto cost = std::make_shared<solvers::LinearCost>(a, b);
  e_on_->AddCost(solvers::Binding(cost, {}));
  e_off_->AddCost(solvers::Binding(cost, {}));
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), b, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, LinearCost3) {
  const Vector2d a{2.3, 4.5};
  const double b = 1.23;
  e_on_->AddCost(a.dot(e_on_->xu()) + b);
  DRAKE_EXPECT_THROWS_MESSAGE(
      g_.SolveShortestPath(*source_, *target_, true),
      ".*costs cannot have linear terms.*");
}

TEST_F(ThreePoints, LinearCost4) {
  const double b = -1.23;
  e_on_->AddCost(b);
  DRAKE_EXPECT_THROWS_MESSAGE(
      g_.SolveShortestPath(*source_, *target_, true),
      "Costs must be non-negative.*");
}

TEST_F(ThreePoints, QuadraticCost) {
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm());
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm());

  auto result = g_.SolveShortestPath(*source_, *target_, true);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              (p_source_.x() - p_target_.x()).squaredNorm(), 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, QuadraticCost2) {
  Matrix2d A;
  A << 4.3, .5, -.4, 1.2;
  // Make an arbitrary non-negative quadratic form, which goes to zero at a
  // point.
  Expression cost = (A * (e_on_->xu() - Vector2d{.54, -.23}) +
                     A * (e_on_->xv() - Vector2d{7.2, -.73}))
                        .squaredNorm();
  e_on_->AddCost(cost);
  e_off_->AddCost(cost.Substitute(subs_on_off_));
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  Environment env{};
  env.insert(e_on_->xu(), p_source_.x());
  env.insert(e_on_->xv(), p_target_.x());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), cost.Evaluate(env), 1e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 4e-6);
}

TEST_F(ThreePoints, QuadraticCost3) {
  Matrix2d A;
  A << 4.3, .5, -.4, 1.2;
  // Make an arbitrary positive quadratic form (by adding a constant).
  Expression cost = (A * (e_on_->xu() - Vector2d{.54, -.23}) +
                     A * (e_on_->xv() - Vector2d{7.2, -.73}))
                        .squaredNorm() +
                    4.2;
  e_on_->AddCost(cost);
  e_off_->AddCost(cost.Substitute(subs_on_off_));
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result.is_success());
  Environment env{};
  env.insert(e_on_->xu(), p_source_.x());
  env.insert(e_on_->xv(), p_target_.x());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), cost.Evaluate(env), 1e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 4e-6);
}

TEST_F(ThreePoints, QuadraticCost4) {
  const Matrix4d R = Vector4d(3.2, 4.3, 6.4, 7.1).asDiagonal();
  const Vector4d d = Vector4d(0.1, 0.2, 0.3, 0.4);
  auto cost = std::make_shared<solvers::QuadraticCost>(
      2.0 * R.transpose() * R, 2.0 * R.transpose() * d, d.dot(d));
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  Vector4d x;
  x << p_source_.x(), p_target_.x();
  EXPECT_NEAR(e_on_->GetSolutionCost(result), (R * x + d).squaredNorm(), 1e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

// Costs must be strictly positive.
TEST_F(ThreePoints, QuadraticCost5) {
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm() - 2.0);
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm() - 2.0);

  DRAKE_EXPECT_THROWS_MESSAGE(
    g_.SolveShortestPath(*source_, *target_, true),
    ".* must be strictly non-negative.*");
}

TEST_F(ThreePoints, L2NormCost) {
  // |xu - xv|₂
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L2NormCost>(A, Vector2d::Zero());
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              (p_source_.x() - p_target_.x()).norm(), 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, L2NormCost2) {
  // L2-norm of an arbitrary transformation of xu and xv.
  Matrix<double, 2, 4> A;
  // clang-format off
  A << 4.3, .5, -.4, 1.2,
       0.1, .2, -2., -.34;
  // clang-format on
  const Vector2d b{.5, .3};
  auto cost = std::make_shared<solvers::L2NormCost>(A, b);
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(
      e_on_->GetSolutionCost(result),
      (A.leftCols(2) * p_source_.x() + A.rightCols(2) * p_target_.x() + b)
          .norm(),
      1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
}

// Like the ThreePoints, but with boxes for each vertex instead of points.
class ThreeBoxes : public ::testing::Test {
 protected:
  ThreeBoxes() {
    auto box = HPolyhedron::MakeUnitBox(2);
    source_ = g_.AddVertex(box, "source");
    target_ = g_.AddVertex(box, "target");
    sink_ = g_.AddVertex(box, "sink");
    e_on_ = g_.AddEdge(*source_, *target_);
    e_off_ = g_.AddEdge(*source_, *sink_);

    subs_on_off_.emplace(e_on_->xv()[0], e_off_->xv()[0]);
    subs_on_off_.emplace(e_on_->xv()[1], e_off_->xv()[1]);
  }

  GraphOfConvexSets g_;
  Edge* e_on_{nullptr};
  Edge* e_off_{nullptr};
  Vertex* source_{nullptr};
  Vertex* target_{nullptr};
  Vertex* sink_{nullptr};
  Substitution subs_on_off_{};
};

// Ipopt fails to solve the QuadraticCost and L2NormCost tests above (both of
// which use Lorentz cone constraints), complaining that the problem has "Too
// few degrees of freedom"; in other words that it has redundant constraints. Of
// course that is true (since the sets were just Points), but Gurobi, Mosek, and
// Snopt all solve it just fine.  This test confirms that Ipopt can solve
// shortest path problems with the QuadraticCost when the points are changed to
// a boxes.
//
// Adding a similar test corresponding to the L2NormCost test caused
// "IPOPT terminated after exceeding the maximum iteration limit" on mac CI
// which was using IPOPT 3.13.4.  Upgrading to IPOPT 3.14.2 resolved the
// problem.
//
// Note: Ipopt will be used in some of the build configurations in CI, even
// though I cannot (yet) request it specifically here.
TEST_F(ThreeBoxes, IpoptTest) {
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm());
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm());
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result.is_success());
}

TEST_F(ThreeBoxes, LinearEqualityConstraint) {
  const Vector2d b{.5, .3};
  e_on_->AddConstraint(e_on_->xv() == b);
  e_off_->AddConstraint(e_off_->xv() == b);
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(target_->GetSolution(result), b, 1e-6));
  EXPECT_TRUE(CompareMatrices(sink_->GetSolution(result), 0 * b, 1e-6));
}

TEST_F(ThreeBoxes, LinearEqualityConstraint2) {
  Matrix<double, 2, 4> Aeq;
  // clang-format off
  Aeq << 4.3, .5, -.4, 1.2,
       0.1, .2, -2., -.34;
  // clang-format on
  const Vector2d beq{.5, .3};
  auto constraint =
      std::make_shared<solvers::LinearEqualityConstraint>(Aeq, beq);
  e_on_->AddConstraint(
      solvers::Binding(constraint, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddConstraint(
      solvers::Binding(constraint, {e_off_->xu(), e_off_->xv()}));
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(
      CompareMatrices(Aeq.leftCols(2) * source_->GetSolution(result) +
                          Aeq.rightCols(2) * target_->GetSolution(result),
                      beq, 1e-6));
  EXPECT_TRUE(
      CompareMatrices(sink_->GetSolution(result), Vector2d::Zero(), 1e-6));
}

TEST_F(ThreeBoxes, LinearConstraint) {
  const Vector2d b{.5, .3};
  e_on_->AddConstraint(e_on_->xv() >= b);
  e_off_->AddConstraint(e_off_->xv() >= b);
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE((target_->GetSolution(result).array() >= b.array() - 1e-6).all());
  EXPECT_TRUE(
      CompareMatrices(sink_->GetSolution(result), Vector2d::Zero(), 1e-6));
}

TEST_F(ThreeBoxes, LinearConstraint2) {
  Matrix<double, 2, 4> A;
  // clang-format off
  A << 4.3, .5, -.4, 1.2,
       0.1, .2, -2., -.34;
  // clang-format on
  const Vector2d lb{.5, .3}, ub{1.0, .4};
  auto constraint = std::make_shared<solvers::LinearConstraint>(A, lb, ub);
  e_on_->AddConstraint(
      solvers::Binding(constraint, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddConstraint(
      solvers::Binding(constraint, {e_off_->xu(), e_off_->xv()}));
  auto result = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(((A.leftCols(2) * source_->GetSolution(result) +
                A.rightCols(2) * target_->GetSolution(result))
                   .array() <= ub.array() + 1e-6)
                  .all());
  EXPECT_TRUE(((A.leftCols(2) * source_->GetSolution(result) +
                A.rightCols(2) * target_->GetSolution(result))
                   .array() >= lb.array() - 1e-6)
                  .all());
  EXPECT_TRUE(
      CompareMatrices(sink_->GetSolution(result), Vector2d::Zero(), 1e-6));

  // Confirm that my linear constraint resulted in a non-trivial solution.
  EXPECT_FALSE(
      CompareMatrices(source_->GetSolution(result), Vector2d::Zero(), 1e-6));
  EXPECT_FALSE(
      CompareMatrices(target_->GetSolution(result), Vector2d::Zero(), 1e-6));
}

// A simple shortest-path problem where the continuous variables do not effect
// the problem (they are all equality constrained).  The GraphOfConvexSets class
// should still solve the problem, and the convex relaxation should be optimal.
GTEST_TEST(ShortestPathTest, ClassicalShortestPath) {
  GraphOfConvexSets spp;

  std::vector<VertexId> vid(5);
  for (int i = 0; i < 5; ++i) {
    vid[i] = spp.AddVertex(Point(Vector1d{0.0}))->id();
  }

  spp.AddEdge(vid[0], vid[1])->AddCost(3.0);
  spp.AddEdge(vid[1], vid[0])->AddCost(1.0);
  spp.AddEdge(vid[0], vid[2])->AddCost(4.0);
  spp.AddEdge(vid[1], vid[2])->AddCost(1.0);
  spp.AddEdge(vid[0], vid[3])->AddCost(1.0);
  spp.AddEdge(vid[3], vid[2])->AddCost(1.0);
  spp.AddEdge(vid[1], vid[4])
      ->AddCost(2.5);  // Updated from original to break symmetry.
  spp.AddEdge(vid[2], vid[4])->AddCost(3.0);
  spp.AddEdge(vid[0], vid[4])->AddCost(6.0);

  auto result = spp.SolveShortestPath(vid[0], vid[4], true);
  ASSERT_TRUE(result.is_success());

  for (const auto& e : spp.Edges()) {
    double expected_cost = 0.0;
    // Only expect non-zero costs on the shortest path.
    if (e->u().id() == vid[0] && e->v().id() == vid[3]) {
      expected_cost = 1.0;
    } else if (e->u().id() == vid[3] && e->v().id() == vid[2]) {
      expected_cost = 1.0;
    } else if (e->u().id() == vid[2] && e->v().id() == vid[4]) {
      expected_cost = 3.0;
    }
    EXPECT_NEAR(e->GetSolutionCost(result), expected_cost, 1e-6);
  }
}

GTEST_TEST(ShortestPathTest, TobiasToyExample) {
  GraphOfConvexSets spp;

  Vertex* source = spp.AddVertex(Point(Vector2d(0, 0)), "source");
  Eigen::MatrixXd vertices(2, 4);
  // clang-format off
  vertices << 1,  1,  3,  3,
              0, -2, -2, -1;
  // clang-format on
  Vertex* p1 = spp.AddVertex(VPolytope(vertices), "p1");
  // clang-format off
  vertices <<  4,  5,  3,  2,
              -2, -4, -4, -3;
  // clang-format on
  Vertex* p2 = spp.AddVertex(VPolytope(vertices), "p2");
  vertices.resize(2, 5);
  // clang-format off
  vertices <<  2, 1, 2, 4, 4,
               2, 3, 4, 4, 3;
  // clang-format on
  Vertex* p3 = spp.AddVertex(VPolytope(vertices), "p3");
  Vertex* e1 =
      spp.AddVertex(Hyperellipsoid(Matrix2d::Identity(), Vector2d(4, 1)), "e1");
  Vertex* e2 = spp.AddVertex(
      Hyperellipsoid(Matrix2d(Vector2d(.25, 1).asDiagonal()), Vector2d(7, -2)),
      "e2");
  // clang-format off
  vertices.resize(2, 3);
  vertices <<  5, 7, 6,
               4, 4, 3;
  // clang-format on
  Vertex* p4 = spp.AddVertex(VPolytope(vertices), "p4");
  vertices.resize(2, 4);
  // clang-format off
  vertices <<  7, 8, 9, 8,
               2, 2, 3, 4;
  // clang-format on
  Vertex* p5 = spp.AddVertex(VPolytope(vertices), "p5");
  Vertex* target = spp.AddVertex(Point(Vector2d(9, 0)), "target");

  Edge* source_to_p1 = spp.AddEdge(*source, *p1);
  Edge* source_to_p2 = spp.AddEdge(*source, *p2);
  spp.AddEdge(*source, *p3);
  spp.AddEdge(*p1, *e2);
  spp.AddEdge(*p2, *p3);
  spp.AddEdge(*p2, *e1);
  spp.AddEdge(*p2, *e2);
  spp.AddEdge(*p3, *p2);  // removing this changes the asymptotic behavior.
  spp.AddEdge(*p3, *e1);
  spp.AddEdge(*p3, *p4);
  spp.AddEdge(*e1, *e2);
  spp.AddEdge(*e1, *p4);
  spp.AddEdge(*e1, *p5);
  spp.AddEdge(*e2, *e1);
  spp.AddEdge(*e2, *p5);
  spp.AddEdge(*e2, *target);
  spp.AddEdge(*p4, *p3);
  spp.AddEdge(*p4, *e2);
  spp.AddEdge(*p4, *p5);
  spp.AddEdge(*p4, *target);
  spp.AddEdge(*p5, *e1);
  spp.AddEdge(*p5, *target);

  // |xu - xv|₂
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L2NormCost>(A, Vector2d::Zero());
  for (const auto& e : spp.Edges()) {
    e->AddCost(solvers::Binding(cost, {e->xu(), e->xv()}));
  }

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  auto result = spp.SolveShortestPath(source->id(), target->id(), false);
  ASSERT_TRUE(result.is_success());

  const std::forward_list<Vertex*> shortest_path{source, p1, e2, target};
  for (const auto& e : spp.Edges()) {
    auto iter = std::find(shortest_path.begin(), shortest_path.end(), &e->u());
    if (iter != shortest_path.end() && &e->v() == *(++iter)) {
      // Then it's on the shortest path; cost should be non-zero.
      EXPECT_GE(e->GetSolutionCost(result), 1.0);
    } else {
      EXPECT_NEAR(e->GetSolutionCost(result), 0.0, 1e-5);
    }
  }

  // Test that forcing an edge not on the shortest path to be active yields a
  // higher cost.
  source_to_p2->AddPhiConstraint(true);
  {
    auto new_result = spp.SolveShortestPath(source->id(), target->id(), false);
    ASSERT_TRUE(new_result.is_success());

    const std::forward_list<Vertex*> new_shortest_path{source, p2, e2, target};
    for (const auto& e : spp.Edges()) {
      auto iter = std::find(new_shortest_path.begin(), new_shortest_path.end(),
                            &e->u());
      // All costs should be non-negative.
      EXPECT_GT(e->GetSolutionCost(new_result), -1e-6);
      if (iter != new_shortest_path.end() && &e->v() == *(++iter)) {
        // Then it's on the shortest path; phi should be 1.
        EXPECT_NEAR(new_result.GetSolution(e->phi()), 1.0, 1e-5);
      } else {
        EXPECT_NEAR(new_result.GetSolution(e->phi()), 0.0, 1e-5);
      }
    }
    EXPECT_GT(new_result.get_optimal_cost(), result.get_optimal_cost());
  }
  source_to_p2->ClearPhiConstraints();

  // Test that forcing an edge on the shortest path to be in-active yields a
  // higher cost.
  source_to_p1->AddPhiConstraint(false);
  {
    auto new_result = spp.SolveShortestPath(source->id(), target->id(), false);
    ASSERT_TRUE(new_result.is_success());

    const std::forward_list<Vertex*> new_shortest_path{source, p2, e2, target};
    for (const auto& e : spp.Edges()) {
      auto iter = std::find(new_shortest_path.begin(), new_shortest_path.end(),
                            &e->u());
      // All costs should be non-negative (to within optimizer tolerance).
      EXPECT_GT(e->GetSolutionCost(new_result), -1e-6);
      if (iter != new_shortest_path.end() && &e->v() == *(++iter)) {
        // Then it's on the shortest path; phi should be 1.
        EXPECT_NEAR(new_result.GetSolution(e->phi()), 1.0, 1e-5);
      } else {
        EXPECT_NEAR(new_result.GetSolution(e->phi()), 0.0, 1e-5);
      }
    }
    EXPECT_GT(new_result.get_optimal_cost(), result.get_optimal_cost());
  }
}

GTEST_TEST(ShortestPathTest, Graphviz) {
  GraphOfConvexSets g;
  auto source = g.AddVertex(Point(Vector2d{1.0, 2.}), "source");
  auto target = g.AddVertex(Point(Vector1d{1e-8}), "target");
  g.AddEdge(*source, *target, "edge");

  // Note: Testing the entire string against a const string is too fragile,
  // since the VertexIds are Identifier<> and increment on a global counter.
  EXPECT_THAT(
      g.GetGraphvizString(),
      AllOf(HasSubstr("source"), HasSubstr("target"), HasSubstr("edge")));
  auto result = g.SolveShortestPath(*source, *target, true);
  EXPECT_THAT(g.GetGraphvizString(result),
              AllOf(HasSubstr("x ="), HasSubstr("cost ="), HasSubstr("ϕ ="),
                    HasSubstr("ϕ xᵤ ="), HasSubstr("ϕ xᵥ =")));
  // No slack variables.
  EXPECT_THAT(
      g.GetGraphvizString(result, false),
      AllOf(HasSubstr("x ="), HasSubstr("cost ="), Not(HasSubstr("ϕ =")),
            Not(HasSubstr("ϕ xᵤ =")), Not(HasSubstr("ϕ xᵥ ="))));
  // Precision and scientific.
  EXPECT_THAT(g.GetGraphvizString(result, false, 2, false),
              AllOf(HasSubstr("x = [1.00 2.00]"), HasSubstr("x = [0.00]")));
  EXPECT_THAT(g.GetGraphvizString(result, false, 2, true),
              AllOf(HasSubstr("x = [1 2]"), HasSubstr("x = [1e-08]")));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
