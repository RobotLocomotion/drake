#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <forward_list>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solver_options.h"

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
using solvers::SolutionResult;
using solvers::SolverOptions;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Substitution;
using symbolic::Variables;

using Edge = GraphOfConvexSets::Edge;
using EdgeId = GraphOfConvexSets::EdgeId;
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

GTEST_TEST(GraphOfConvexSetsTest, RemoveEdge) {
  GraphOfConvexSets g;
  Point pu(Vector3d(1.0, 2.0, 3.0));
  Point pv(Vector2d(4., 5.));
  Vertex* u = g.AddVertex(pu, "u");
  Vertex* v = g.AddVertex(pv, "v");
  Edge* e1 = g.AddEdge(*u, *v, "e1");
  Edge* e2 = g.AddEdge(*v, *u, "e2");

  EXPECT_EQ(g.Edges().size(), 2);

  g.RemoveEdge(e1->id());
  auto edges = g.Edges();
  EXPECT_EQ(edges.size(), 1);
  EXPECT_EQ(edges.at(0), e2);

  g.RemoveEdge(*e2);
  EXPECT_EQ(g.Edges().size(), 0);
}

GTEST_TEST(GraphOfConvexSetsTest, RemoveVertex) {
  GraphOfConvexSets g;
  Vertex* v1 = g.AddVertex(Point(Vector2d(3., 5.)));
  Vertex* v2 = g.AddVertex(Point(Vector2d(-2., 4.)));
  Vertex* v3 = g.AddVertex(Point(Vector2d(5., -2.3)));
  Edge* e1 = g.AddEdge(*v1, *v2);
  g.AddEdge(*v1, *v3);
  g.AddEdge(*v3, *v1);

  EXPECT_EQ(g.Vertices().size(), 3);
  EXPECT_EQ(g.Edges().size(), 3);

  g.RemoveVertex(v3->id());
  EXPECT_EQ(g.Vertices().size(), 2);
  auto edges = g.Edges();
  EXPECT_EQ(edges.size(), 1);
  EXPECT_EQ(edges.at(0), e1);

  g.RemoveVertex(*v2);
  auto vertices = g.Vertices();
  EXPECT_EQ(vertices.size(), 1);
  EXPECT_EQ(vertices.at(0), v1);
  EXPECT_EQ(g.Edges().size(), 0);
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
  auto [v_ell0, v_b0] = v_->AddCost((v_->x() + Vector3d::Ones()).squaredNorm());
  auto [v_ell1, v_b1] = v_->AddCost(Binding(cost, v_->x().head<2>()));

  // Confirm that they are down-castable.
  auto quadratic = dynamic_cast<QuadraticCost*>(b0.evaluator().get());
  EXPECT_TRUE(quadratic != nullptr);
  auto linear = dynamic_cast<LinearCost*>(b1.evaluator().get());
  EXPECT_TRUE(linear != nullptr);
  quadratic = dynamic_cast<QuadraticCost*>(v_b0.evaluator().get());
  EXPECT_TRUE(quadratic != nullptr);
  linear = dynamic_cast<LinearCost*>(v_b1.evaluator().get());
  EXPECT_TRUE(linear != nullptr);

  // Confirm that they are all accessible.
  const auto& edge_costs = e_->GetCosts();
  EXPECT_EQ(edge_costs[0], b0);
  EXPECT_EQ(edge_costs[1], b1);
  const auto& vertex_costs = v_->GetCosts();
  EXPECT_EQ(vertex_costs[0], v_b0);
  EXPECT_EQ(vertex_costs[1], v_b1);

  MathematicalProgramResult result;
  std::unordered_map<symbolic::Variable::Id, int> map;
  map.emplace(ell0.get_id(), 0);
  map.emplace(ell1.get_id(), 1);
  map.emplace(v_ell0.get_id(), 2);
  map.emplace(v_ell1.get_id(), 3);
  const Vector4d ell{1.2, 3.4, 5.6, 7.8};
  result.set_decision_variable_index(map);
  result.set_x_val(ell);

  EXPECT_NEAR(e_->GetSolutionCost(result), ell.head<2>().sum(), 1e-16);
  EXPECT_NEAR(v_->GetSolutionCost(result), ell.tail<2>().sum(), 1e-16);

  symbolic::Variable other_var("x");
  DRAKE_EXPECT_THROWS_MESSAGE(e_->AddCost(other_var), ".*IsSubsetOf.*");
  DRAKE_EXPECT_THROWS_MESSAGE(v_->AddCost(other_var), ".*IsSubsetOf.*");
}

// Confirms that I can add constraints (both ways).
// The correctness of the added constraints will be established by the solution
// tests.
TEST_F(TwoPoints, AddConstraint) {
  auto b0 = e_->AddConstraint(e_->xv().head<2>() == e_->xu());
  auto constraint = std::make_shared<LinearConstraint>(
      Matrix2d::Identity(), Vector2d::Zero(), Vector2d{1.2, 3.4});
  auto b1 = e_->AddConstraint(Binding(constraint, e_->xu()));
  auto u_b0 = u_->AddConstraint(u_->x() == pu_.x());
  auto u_b1 = u_->AddConstraint(Binding(constraint, u_->x()));

  // Confirm that they are down-castable.
  auto linear_equality =
      dynamic_cast<LinearEqualityConstraint*>(b0.evaluator().get());
  EXPECT_TRUE(linear_equality != nullptr);
  auto linear = dynamic_cast<LinearConstraint*>(b1.evaluator().get());
  EXPECT_TRUE(linear != nullptr);
  linear_equality =
      dynamic_cast<LinearEqualityConstraint*>(u_b0.evaluator().get());
  EXPECT_TRUE(linear_equality != nullptr);
  linear = dynamic_cast<LinearConstraint*>(u_b1.evaluator().get());
  EXPECT_TRUE(linear != nullptr);

  // Confirm that they are all accessible.
  const auto& edge_constraints = e_->GetConstraints();
  EXPECT_EQ(edge_constraints[0], b0);
  EXPECT_EQ(edge_constraints[1], b1);
  const auto& vertex_constraints = u_->GetConstraints();
  EXPECT_EQ(vertex_constraints[0], u_b0);
  EXPECT_EQ(vertex_constraints[1], u_b1);

  symbolic::Variable other_var("x");
  DRAKE_EXPECT_THROWS_MESSAGE(e_->AddConstraint(other_var == 1),
                              ".*IsSubsetOf.*");
  DRAKE_EXPECT_THROWS_MESSAGE(u_->AddConstraint(other_var == 1),
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

    options.preprocessing = false;
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
  GraphOfConvexSetsOptions options;
};

TEST_F(ThreePoints, LinearCost1) {
  e_on_->AddCost(1.0);
  e_off_->AddCost(1.0);
  source_->AddCost(1.0);
  auto result = g_.SolveShortestPath(source_->id(), target_->id(), options);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), 1.0, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), 1.0, 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);

  EXPECT_TRUE(
      CompareMatrices(e_on_->GetSolutionPhiXu(result), p_source_.x(), 1e-6));
  EXPECT_TRUE(
      CompareMatrices(e_on_->GetSolutionPhiXv(result), p_target_.x(), 1e-6));
  EXPECT_TRUE(CompareMatrices(e_off_->GetSolutionPhiXu(result),
                              0 * p_source_.x(), 1e-6));
  EXPECT_TRUE(
      CompareMatrices(e_off_->GetSolutionPhiXv(result), 0 * p_sink_.x(), 1e-6));

  // Alternative signatures.
  auto result2 = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result2.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result2), 1.0, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result2), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result2), 1.0, 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result2), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result2), 0.0, 1e-6);

  options.solver_options = SolverOptions();
  auto result4 = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result4.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result4), 1.0, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result4), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result4), 1.0, 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result4), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result4), 0.0, 1e-6);

  EXPECT_TRUE(
      CompareMatrices(source_->GetSolution(result4), p_source_.x(), 1e-6));
  EXPECT_TRUE(
      CompareMatrices(target_->GetSolution(result4), p_target_.x(), 1e-6));
  EXPECT_TRUE(sink_->GetSolution(result4).hasNaN());

  if (solvers::ClpSolver::is_available()) {
    solvers::ClpSolver clp;
    options.solver = &clp;
    auto result3 = g_.SolveShortestPath(*source_, *target_, options);
    ASSERT_TRUE(result3.is_success());
    EXPECT_NEAR(e_on_->GetSolutionCost(result3), 1.0, 1e-6);
    EXPECT_NEAR(e_off_->GetSolutionCost(result3), 0.0, 1e-6);
    EXPECT_NEAR(source_->GetSolutionCost(result3), 1.0, 1e-6);
    EXPECT_NEAR(target_->GetSolutionCost(result3), 0.0, 1e-6);
    EXPECT_NEAR(sink_->GetSolutionCost(result3), 0.0, 1e-6);
  }
}

TEST_F(ThreePoints, ConvexRelaxation) {
  e_on_->AddCost(1.0);
  e_off_->AddCost(1.0);
  source_->AddCost(1.0);
  e_on_->AddConstraint(e_on_->xv()[0] <= 0.0);
  source_->AddConstraint(source_->x()[0] >= 1.0);
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(e_on_->phi()), 1.0, 1e-6);
  EXPECT_NEAR(result.GetSolution(e_off_->phi()), 0.0, 1e-6);

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  options.convex_relaxation = false;
  auto result2 = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result2.is_success());
  EXPECT_NEAR(result2.GetSolution(e_on_->phi()), 1.0, 1e-6);
  EXPECT_NEAR(result2.GetSolution(e_off_->phi()), 0.0, 1e-6);

  EXPECT_NEAR(e_on_->GetSolutionCost(result), e_on_->GetSolutionCost(result2),
              1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), e_off_->GetSolutionCost(result2),
              1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result),
              source_->GetSolutionCost(result2), 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result),
              target_->GetSolutionCost(result2), 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), sink_->GetSolutionCost(result2),
              1e-6);
}

TEST_F(ThreePoints, LinearCost2) {
  const Vector0<double> a;
  const double b = 1.23;
  auto cost = std::make_shared<solvers::LinearCost>(a, b);
  e_on_->AddCost(solvers::Binding(cost, {}));
  e_off_->AddCost(solvers::Binding(cost, {}));
  source_->AddCost(solvers::Binding(cost, {}));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), b, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), b, 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, LinearCost3) {
  const Vector2d a{2.3, 4.5};
  const double b = 1.23;
  e_on_->AddCost(a.dot(e_on_->xu()) + b);
  e_off_->AddCost(a.dot(e_off_->xu()) + b);
  source_->AddCost(a.dot(source_->x()) + b);
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), a.dot(p_source_.x()) + b, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), a.dot(p_source_.x()) + b, 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, LinearCost4) {
  const double b = -1.23;
  e_on_->AddCost(b);
  DRAKE_EXPECT_THROWS_MESSAGE(g_.SolveShortestPath(*source_, *target_, options),
                              "Constant costs must be non-negative.*");
}

TEST_F(ThreePoints, LinearCost5) {
  const double b = -1.23;
  source_->AddCost(b);
  DRAKE_EXPECT_THROWS_MESSAGE(g_.SolveShortestPath(*source_, *target_, options),
                              "Constant costs must be non-negative.*");
}

TEST_F(ThreePoints, MultipleVertexCosts) {
  source_->AddCost(1.0);
  source_->AddCost(1.0);
  DRAKE_EXPECT_NO_THROW(g_.SolveShortestPath(*source_, *target_, options));
}

TEST_F(ThreePoints, QuadraticCost) {
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm());
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm());
  source_->AddCost(
      static_cast<const VectorX<Expression>>(source_->x()).squaredNorm());

  auto result = g_.SolveShortestPath(*source_, *target_, options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              (p_source_.x() - p_target_.x()).squaredNorm(), 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), p_source_.x().squaredNorm(),
              1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
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
  Expression vertex_cost =
      (A * (source_->x() - Vector2d{.54, -.23})).squaredNorm();
  source_->AddCost(vertex_cost);
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  Environment env{};
  env.insert(e_on_->xu(), p_source_.x());
  env.insert(e_on_->xv(), p_target_.x());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), cost.Evaluate(env), 1e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 4e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), vertex_cost.Evaluate(env),
              1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
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
  Expression vertex_cost =
      (A * (source_->x() - Vector2d{.54, -.23})).squaredNorm() + 4.2;
  source_->AddCost(vertex_cost);
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result.is_success());
  Environment env{};
  env.insert(e_on_->xu(), p_source_.x());
  env.insert(e_on_->xv(), p_target_.x());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), cost.Evaluate(env), 1e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 4e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), vertex_cost.Evaluate(env),
              1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, QuadraticCost4) {
  const Matrix4d R = Vector4d(1.2, 2.3, 3.4, 4.5).asDiagonal();
  const Vector4d d = Vector4d(0.1, 0.2, 0.3, 0.4);
  auto cost = std::make_shared<solvers::QuadraticCost>(
      2.0 * R.transpose() * R, 2.0 * R.transpose() * d, d.dot(d));
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  const Matrix2d R_v = R.topLeftCorner<2, 2>();
  const Vector2d d_v = d.head<2>();
  auto vertex_cost = std::make_shared<solvers::QuadraticCost>(
      2.0 * R_v.transpose() * R_v, 2.0 * R_v.transpose() * d_v, d_v.dot(d_v));
  source_->AddCost(solvers::Binding(vertex_cost, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  Vector4d x;
  x << p_source_.x(), p_target_.x();
  EXPECT_NEAR(e_on_->GetSolutionCost(result), (R * x + d).squaredNorm(), 2e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result),
              (R_v * p_source_.x() + d_v).squaredNorm(), 1e-5);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
}

// Costs must be strictly positive.
TEST_F(ThreePoints, QuadraticCost5) {
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm() - 2.0);
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm() - 2.0);

  DRAKE_EXPECT_THROWS_MESSAGE(g_.SolveShortestPath(*source_, *target_, options),
                              ".* must be strictly non-negative.*");
}

// Costs must be strictly positive.
TEST_F(ThreePoints, QuadraticCost6) {
  source_->AddCost(
      static_cast<const VectorX<Expression>>(source_->x()).squaredNorm() - 2.0);

  DRAKE_EXPECT_THROWS_MESSAGE(g_.SolveShortestPath(*source_, *target_, options),
                              ".* must be strictly non-negative.*");
}

TEST_F(ThreePoints, L1NormCost) {
  // |xu - xv|₁
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L1NormCost>(A, Vector2d::Zero());
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  RowVector2d A_v(1, -1);
  auto vertex_cost =
      std::make_shared<solvers::L1NormCost>(A_v, Vector1d::Zero());
  source_->AddCost(solvers::Binding(vertex_cost, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              (p_source_.x() - p_target_.x()).cwiseAbs().sum(), 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), abs(A_v * p_source_.x()), 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, L1NormCost2) {
  // L1-norm of an arbitrary transformation of xu and xv.
  Matrix<double, 2, 4> A;
  // clang-format off
  A << 4.3, .5, -.4, 1.2,
       0.1, .2, -2., -.34;
  // clang-format on
  const Vector2d b{.5, .3};
  auto cost = std::make_shared<solvers::L1NormCost>(A, b);
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  RowVector2d A_v(4.3, -0.4);
  auto vertex_cost =
      std::make_shared<solvers::L1NormCost>(A_v, Vector1d::Zero());
  source_->AddCost(solvers::Binding(vertex_cost, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(
      e_on_->GetSolutionCost(result),
      (A.leftCols(2) * p_source_.x() + A.rightCols(2) * p_target_.x() + b)
          .cwiseAbs()
          .sum(),
      1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), abs(A_v * p_source_.x()), 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, L2NormCost) {
  // |xu - xv|₂
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L2NormCost>(A, Vector2d::Zero());
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  RowVector2d A_v(1, -1);
  auto vertex_cost =
      std::make_shared<solvers::L2NormCost>(A_v, Vector1d::Zero());
  source_->AddCost(solvers::Binding(vertex_cost, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              (p_source_.x() - p_target_.x()).norm(), 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), (A_v * p_source_.x()).norm(),
              1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
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
  RowVector2d A_v(4.3, -0.4);
  auto vertex_cost =
      std::make_shared<solvers::L2NormCost>(A_v, Vector1d::Zero());
  source_->AddCost(solvers::Binding(vertex_cost, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
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
  EXPECT_NEAR(source_->GetSolutionCost(result), (A_v * p_source_.x()).norm(),
              1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, LInfNormCost) {
  // |xu - xv|∞
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::LInfNormCost>(A, Vector2d::Zero());
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  RowVector2d A_v(1, -1);
  auto vertex_cost =
      std::make_shared<solvers::LInfNormCost>(A_v, Vector1d::Zero());
  source_->AddCost(solvers::Binding(vertex_cost, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result),
              (p_source_.x() - p_target_.x()).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), abs(A_v * p_source_.x()), 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, LInfNormCost2) {
  // LInfinity-norm of an arbitrary transformation of xu and xv.
  Matrix<double, 2, 4> A;
  // clang-format off
  A << 4.3, .5, -.4, 1.2,
       0.1, .2, -2., -.34;
  // clang-format on
  const Vector2d b{.5, .3};
  auto cost = std::make_shared<solvers::LInfNormCost>(A, b);
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  RowVector2d A_v(4.3, -0.4);
  auto vertex_cost =
      std::make_shared<solvers::LInfNormCost>(A_v, Vector1d::Zero());
  source_->AddCost(solvers::Binding(vertex_cost, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(
      e_on_->GetSolutionCost(result),
      (A.leftCols(2) * p_source_.x() + A.rightCols(2) * p_target_.x() + b)
          .cwiseAbs()
          .maxCoeff(),
      1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), abs(A_v * p_source_.x()), 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
}

TEST_F(ThreePoints, PerspectiveQuadraticCost) {
  Matrix<double, 2, 4> A;
  // clang-format off
  A << -1, 1, 0, 0,
       0, 0, 1, 1;
  // clang-format on
  const Vector2d b{0.5, 0.3};
  auto cost = std::make_shared<solvers::PerspectiveQuadraticCost>(A, b);
  e_on_->AddCost(solvers::Binding(cost, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddCost(solvers::Binding(cost, {e_off_->xu(), e_off_->xv()}));
  Matrix<double, 2, 2> A_v;
  A_v << -1, 1, 1, 1;
  auto vertex_cost =
      std::make_shared<solvers::PerspectiveQuadraticCost>(A_v, b);
  source_->AddCost(solvers::Binding(vertex_cost, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  const double expected_cost =
      std::pow(p_target_.x()(0) + p_target_.x()(1) + b(1), 2) /
      (p_source_.x()(1) - p_source_.x()(0) + b(0));
  EXPECT_NEAR(e_on_->GetSolutionCost(result), expected_cost, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  const double vertex_expected_cost =
      std::pow(p_source_.x()(0) + p_source_.x()(1) + b(1), 2) /
      (p_source_.x()(1) - p_source_.x()(0) + b(0));
  EXPECT_NEAR(source_->GetSolutionCost(result), vertex_expected_cost, 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
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

    options.preprocessing = true;
  }

  GraphOfConvexSets g_;
  Edge* e_on_{nullptr};
  Edge* e_off_{nullptr};
  Vertex* source_{nullptr};
  Vertex* target_{nullptr};
  Vertex* sink_{nullptr};
  Substitution subs_on_off_{};
  GraphOfConvexSetsOptions options;
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
TEST_F(ThreeBoxes, IpoptTest) {
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm());
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm());

  solvers::IpoptSolver ipopt;
  options.solver = &ipopt;
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result.is_success());
}

TEST_F(ThreeBoxes, LinearEqualityConstraint) {
  const Vector2d b{.5, .3};
  e_on_->AddConstraint(e_on_->xv() == b);
  e_off_->AddConstraint(e_off_->xv() == b);
  source_->AddConstraint(source_->x() == -b);
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(source_->GetSolution(result), -b, 1e-6));
  EXPECT_TRUE(CompareMatrices(target_->GetSolution(result), b, 1e-6));
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
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
  Matrix<double, 2, 2> Aeq_v;
  Aeq_v << 4.5, 0.7, 0.3, 0.6;
  auto vertex_constraint =
      std::make_shared<solvers::LinearEqualityConstraint>(Aeq_v, beq);
  source_->AddConstraint(solvers::Binding(vertex_constraint, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(Aeq_v * source_->GetSolution(result), beq, 1e-6));
  EXPECT_TRUE(
      CompareMatrices(Aeq.leftCols(2) * source_->GetSolution(result) +
                          Aeq.rightCols(2) * target_->GetSolution(result),
                      beq, 1e-6));
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
}

TEST_F(ThreeBoxes, LinearConstraint) {
  const Vector2d b{.5, .3};
  e_on_->AddConstraint(e_on_->xv() >= b);
  e_off_->AddConstraint(e_off_->xv() >= b);
  source_->AddConstraint(source_->x() <= -b);
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE((source_->GetSolution(result).array() <= b.array() - 1e-6).all());
  EXPECT_TRUE((target_->GetSolution(result).array() >= b.array() - 1e-6).all());
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
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
  Matrix<double, 2, 2> A_v;
  A_v << 4.5, 0.7, 0.3, 0.6;
  auto vertex_constraint =
      std::make_shared<solvers::LinearConstraint>(A_v, lb, ub);
  source_->AddConstraint(solvers::Binding(vertex_constraint, source_->x()));
  auto result = g_.SolveShortestPath(*source_, *target_, options);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(
      ((A_v * source_->GetSolution(result)).array() <= ub.array() + 1e-6)
          .all());
  EXPECT_TRUE(
      ((A_v * source_->GetSolution(result)).array() >= lb.array() - 1e-6)
          .all());
  EXPECT_TRUE(((A.leftCols(2) * source_->GetSolution(result) +
                A.rightCols(2) * target_->GetSolution(result))
                   .array() <= ub.array() + 1e-6)
                  .all());
  EXPECT_TRUE(((A.leftCols(2) * source_->GetSolution(result) +
                A.rightCols(2) * target_->GetSolution(result))
                   .array() >= lb.array() - 1e-6)
                  .all());
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());

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

  GraphOfConvexSetsOptions options;
  options.preprocessing = false;

  auto result = spp.SolveShortestPath(vid[0], vid[4], options);
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

GTEST_TEST(ShortestPathTest, InfeasibleProblem) {
  GraphOfConvexSets spp;

  Vertex* source = spp.AddVertex(Point(Vector1d(-1)));
  Vertex* v1 = spp.AddVertex(Point(Vector1d(-0.5)));
  Vertex* v2 = spp.AddVertex(Point(Vector1d(0.5)));
  Vertex* target = spp.AddVertex(Point(Vector1d(1)));

  spp.AddEdge(*source, *v1);
  spp.AddEdge(*v2, *target);

  auto result = spp.SolveShortestPath(*source, *target);
  ASSERT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);

  GraphOfConvexSetsOptions options;
  options.max_rounded_paths = 1;
  result = spp.SolveShortestPath(*source, *target, options);
  ASSERT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  options.convex_relaxation = false;
  result = spp.SolveShortestPath(*source, *target, options);
  ASSERT_FALSE(result.is_success());
}

GTEST_TEST(ShortestPathTest, TwoStepLoopConstraint) {
  GraphOfConvexSets spp;

  spp.AddVertex(Point(Vector2d(0, 0)));
  spp.AddVertex(HPolyhedron::MakeBox(Vector2d(1.3, 1.3), Vector2d(2.7, 2.7)));
  spp.AddVertex(HPolyhedron::MakeBox(Vector2d(3.3, 1.3), Vector2d(4.7, 2.7)));
  spp.AddVertex(HPolyhedron::MakeBox(Vector2d(1.3, -2.7), Vector2d(2.7, -1.3)));
  spp.AddVertex(HPolyhedron::MakeBox(Vector2d(3.3, -2.7), Vector2d(4.7, -1.3)));
  spp.AddVertex(Point(Vector2d(6, 0)));

  auto v = spp.Vertices();

  // |xu - xv|₂
  Matrix<double, 4, 4> A = Matrix<double, 4, 4>::Identity();
  A.block(0, 2, 2, 2) = -Matrix2d::Identity();
  A.block(2, 0, 2, 2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::QuadraticCost>(A, Vector4d::Zero());

  spp.AddEdge(*v[0], *v[1])
      ->AddCost(solvers::Binding(cost, {v[0]->x(), v[1]->x()}));
  spp.AddEdge(*v[1], *v[0])
      ->AddCost(solvers::Binding(cost, {v[1]->x(), v[0]->x()}));
  spp.AddEdge(*v[1], *v[2])
      ->AddCost(solvers::Binding(cost, {v[1]->x(), v[2]->x()}));
  spp.AddEdge(*v[2], *v[1])
      ->AddCost(solvers::Binding(cost, {v[2]->x(), v[1]->x()}));
  spp.AddEdge(*v[2], *v[5])
      ->AddCost(solvers::Binding(cost, {v[2]->x(), v[5]->x()}));
  spp.AddEdge(*v[5], *v[2])
      ->AddCost(solvers::Binding(cost, {v[5]->x(), v[2]->x()}));

  spp.AddEdge(*v[0], *v[3])
      ->AddCost(solvers::Binding(cost, {v[0]->x(), v[3]->x()}));
  spp.AddEdge(*v[3], *v[0])
      ->AddCost(solvers::Binding(cost, {v[3]->x(), v[0]->x()}));
  spp.AddEdge(*v[3], *v[4])
      ->AddCost(solvers::Binding(cost, {v[3]->x(), v[4]->x()}));
  spp.AddEdge(*v[4], *v[3])
      ->AddCost(solvers::Binding(cost, {v[4]->x(), v[3]->x()}));
  spp.AddEdge(*v[4], *v[5])
      ->AddCost(solvers::Binding(cost, {v[4]->x(), v[5]->x()}));
  spp.AddEdge(*v[5], *v[4])
      ->AddCost(solvers::Binding(cost, {v[5]->x(), v[4]->x()}));

  GraphOfConvexSetsOptions options;
  options.preprocessing = false;

  auto result = spp.SolveShortestPath(*v[0], *v[5], options);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  EXPECT_TRUE(result.is_success());

  int non_zero_edges = 0;
  for (const auto& e : spp.Edges()) {
    // Tuned so that off edges are below this value for all solvers
    if (result.GetSolution(e->phi()) > 1e-5) {
      ++non_zero_edges;
    }
  }
  EXPECT_EQ(non_zero_edges, 6);
}

// Test that all optimization variables are properly set, even when constrained
// to be on or off.
GTEST_TEST(ShortestPathTest, PhiConstraint) {
  GraphOfConvexSets spp;

  spp.AddVertex(Point(Vector2d(0, 0)));
  spp.AddVertex(Point(Vector2d(1, -1)));
  spp.AddVertex(Point(Vector2d(1, 1)));
  spp.AddVertex(Point(Vector2d(2, 0)));

  auto v = spp.Vertices();

  Edge* edge_01 = spp.AddEdge(*v[0], *v[1]);
  Edge* edge_02 = spp.AddEdge(*v[0], *v[2]);
  Edge* edge_13 = spp.AddEdge(*v[1], *v[3]);
  Edge* edge_23 = spp.AddEdge(*v[2], *v[3]);

  // |xu - xv|₂
  Matrix<double, 4, 4> A = Matrix<double, 4, 4>::Identity();
  A.block(0, 2, 2, 2) = -Matrix2d::Identity();
  A.block(2, 0, 2, 2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::QuadraticCost>(A, Vector4d::Zero());

  edge_01->AddCost(solvers::Binding(cost, {v[0]->x(), v[1]->x()}));
  edge_02->AddCost(solvers::Binding(cost, {v[0]->x(), v[2]->x()}));
  edge_13->AddCost(solvers::Binding(cost, {v[1]->x(), v[3]->x()}));
  edge_23->AddCost(solvers::Binding(cost, {v[2]->x(), v[3]->x()}));

  GraphOfConvexSetsOptions options;
  options.preprocessing = false;

  // Confirm that variables for edges are set when no on/off constraint is
  // imposed.
  {
    auto result = spp.SolveShortestPath(*v[0], *v[3], options);
    if (result.get_solver_id() == solvers::IpoptSolver::id()) {
      return;  // See IpoptTest for details.
    }
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXu(result),
                                0.5 * Vector2d(1, -1), 1e-6));
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXv(result),
                                0.5 * Vector2d(2, 0), 1e-6));
    EXPECT_NEAR(edge_13->GetSolutionCost(result), 0.5 * 1, 1e-6);
    EXPECT_NEAR(result.GetSolution(edge_13->phi()), 0.5, 1e-6);
    EXPECT_TRUE(
        CompareMatrices(v[1]->GetSolution(result), Vector2d(1, -1), 1e-6));
  }

  // Confirm that variables for edges that are turned off are properly set.
  // This check is necessary now that these variables have been removed from the
  // optimization problem.
  edge_13->AddPhiConstraint(false);
  {
    auto result = spp.SolveShortestPath(*v[0], *v[3], options);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXu(result),
                                Vector2d::Zero(), 1e-6));
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXv(result),
                                Vector2d::Zero(), 1e-6));
    EXPECT_NEAR(edge_13->GetSolutionCost(result), 0, 1e-6);
    EXPECT_NEAR(result.GetSolution(edge_13->phi()), 0, 1e-6);
  }

  // Confirm that variables for edges that are turned on are properly set.
  edge_13->AddPhiConstraint(true);
  {
    auto result = spp.SolveShortestPath(*v[0], *v[3], options);
    if (result.get_solver_id() == solvers::IpoptSolver::id()) {
      return;  // See IpoptTest for details.
    }
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXu(result),
                                Vector2d(1, -1), 1e-6));
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXv(result),
                                Vector2d(2, 0), 1e-6));
    EXPECT_NEAR(edge_13->GetSolutionCost(result), 1, 1e-6);
    EXPECT_NEAR(result.GetSolution(edge_13->phi()), 1, 1e-6);
  }
}

// Confirms that preprocessing removes edges that cannot be on the shortest path
// and does not change the solution to a shortest path query.
class PreprocessShortestPathTest : public ::testing::Test {
 protected:
  PreprocessShortestPathTest() {
    vid_.reserve(7);
    for (int i = 0; i < 7; ++i) {
      vid_[i] = g_.AddVertex(Point(Vector1d{1.0}))->id();
    }

    edges_.reserve(13);
    edges_.push_back(g_.AddEdge(vid_[0], vid_[2]));
    edges_.push_back(g_.AddEdge(vid_[2], vid_[3]));
    edges_.push_back(g_.AddEdge(vid_[2], vid_[4]));
    edges_.push_back(g_.AddEdge(vid_[3], vid_[4]));
    edges_.push_back(g_.AddEdge(vid_[4], vid_[3]));
    edges_.push_back(g_.AddEdge(vid_[3], vid_[5]));
    edges_.push_back(g_.AddEdge(vid_[4], vid_[5]));
    // Useless backtracking edges
    edges_.push_back(g_.AddEdge(vid_[3], vid_[2]));
    edges_.push_back(g_.AddEdge(vid_[5], vid_[4]));
    // Useless nodes off source and target
    edges_.push_back(g_.AddEdge(vid_[0], vid_[1]));
    edges_.push_back(g_.AddEdge(vid_[1], vid_[0]));
    edges_.push_back(g_.AddEdge(vid_[5], vid_[6]));
    edges_.push_back(g_.AddEdge(vid_[6], vid_[5]));

    for (Edge* e : edges_) {
      e->AddCost(1.0);
    }

    // Break symmetry of graph.
    edges_[2]->AddCost(0.1);
  }
  std::set<EdgeId> PreprocessShortestPath(VertexId source_id,
                                          VertexId target_id) {
    return g_.PreprocessShortestPath(source_id, target_id);
  }

  GraphOfConvexSets g_;
  std::vector<VertexId> vid_;
  std::vector<Edge*> edges_;
  GraphOfConvexSetsOptions options_;
};

TEST_F(PreprocessShortestPathTest, CheckEdges) {
  std::set<EdgeId> removed_edges = PreprocessShortestPath(vid_[0], vid_[5]);

  for (size_t ii = 0; ii < edges_.size(); ii++) {
    if (ii < 7) {
      EXPECT_FALSE(removed_edges.find(edges_[ii]->id()) != removed_edges.end());
    } else {
      EXPECT_TRUE(removed_edges.find(edges_[ii]->id()) != removed_edges.end());
    }
  }
}

TEST_F(PreprocessShortestPathTest, CheckResults) {
  options_.preprocessing = false;
  auto result1 = g_.SolveShortestPath(vid_[0], vid_[5], options_);
  ASSERT_TRUE(result1.is_success());

  options_.preprocessing = true;
  auto result2 = g_.SolveShortestPath(vid_[0], vid_[5], options_);
  ASSERT_TRUE(result2.is_success());

  for (Edge* e : edges_) {
    EXPECT_NEAR(result1.GetSolution(e->phi()), result2.GetSolution(e->phi()),
                1e-10);
    EXPECT_TRUE(CompareMatrices(result1.GetSolution(e->xu()),
                                result2.GetSolution(e->xu()), 1e-12));
    EXPECT_TRUE(CompareMatrices(result1.GetSolution(e->xv()),
                                result2.GetSolution(e->xv()), 1e-12));
    EXPECT_NEAR(e->GetSolutionCost(result1), e->GetSolutionCost(result2),
                1e-10);
  }
}

/* This test rounds the shortest path on a graph with two paths around an
obstacle.
┌──────┐     ┌────┐     ┌────┐
|source├────►│ p1 │◄───►│ p3 │─────────┐
└───┬──┘     └─▲──┘     └─▲──┘         |
    │          |          |            |
    │        ┌─▼──┐     ┌─▼──┐     ┌───▼────┐
    └───────►│ p2 │◄───►│ p4 │────►│ target │
             └────┘     └────┘     └────────┘

*/
GTEST_TEST(ShortestPathTest, RoundedSolution) {
  GraphOfConvexSets spp;

  Vertex* source = spp.AddVertex(Point(Vector2d(-1.5, -1.5)));
  Vertex* target = spp.AddVertex(Point(Vector2d(1.5, 1.5)));
  Vertex* p1 =
      spp.AddVertex(HPolyhedron::MakeBox(Vector2d(-2, -2), Vector2d(2, -1)));
  Vertex* p2 =
      spp.AddVertex(HPolyhedron::MakeBox(Vector2d(-2, -2), Vector2d(-1, 2)));
  Vertex* p3 =
      spp.AddVertex(HPolyhedron::MakeBox(Vector2d(1, -2), Vector2d(2, 2)));
  Vertex* p4 =
      spp.AddVertex(HPolyhedron::MakeBox(Vector2d(-2, 1), Vector2d(2, 2)));

  // Edges pointing towards target
  spp.AddEdge(*source, *p1);
  spp.AddEdge(*source, *p2);
  spp.AddEdge(*p1, *p3);
  spp.AddEdge(*p2, *p4);
  spp.AddEdge(*p3, *target);
  spp.AddEdge(*p4, *target);

  // Edges between parallel vertices
  spp.AddEdge(*p1, *p2);
  spp.AddEdge(*p2, *p1);
  spp.AddEdge(*p3, *p4);
  spp.AddEdge(*p4, *p3);

  // Edges pointing towards source
  spp.AddEdge(*p3, *p1);
  spp.AddEdge(*p4, *p2);

  // |xu - xv|₂
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L2NormCost>(A, Vector2d::Zero());

  for (const auto& e : spp.Edges()) {
    if (e->u().id() != source->id()) {
      e->AddCost(solvers::Binding(cost, {e->xu(), e->xv()}));
    }
  }

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;
  options.preprocessing = false;
  options.max_rounded_paths = 0;
  auto relaxed_result =
      spp.SolveShortestPath(source->id(), target->id(), options);
  ASSERT_TRUE(relaxed_result.is_success());

  options.preprocessing = true;
  options.max_rounded_paths = 10;
  auto rounded_result =
      spp.SolveShortestPath(source->id(), target->id(), options);
  ASSERT_TRUE(rounded_result.is_success());

  EXPECT_LT(relaxed_result.get_optimal_cost(),
            rounded_result.get_optimal_cost());

  const auto& edges = spp.Edges();
  for (size_t ii = 0; ii < edges.size(); ++ii) {
    if (ii < 6) {
      // Some solvers do not balance the two paths as closely as other solvers.
      const double tol =
          (relaxed_result.get_solver_id() == solvers::GurobiSolver::id()) ? 1e-1
          : (relaxed_result.get_solver_id() == solvers::CsdpSolver::id()) ? 1e-2
          : 1e-5;
      EXPECT_NEAR(relaxed_result.GetSolution(edges[ii]->phi()), 0.5, tol);
    } else if (ii < 10) {
      EXPECT_LT(relaxed_result.GetSolution(edges[ii]->phi()), 0.5);
      EXPECT_GT(relaxed_result.GetSolution(edges[ii]->phi()), 0);
    } else {
      EXPECT_NEAR(relaxed_result.GetSolution(edges[ii]->phi()), 0, 1e-6);
    }
    EXPECT_TRUE(rounded_result.GetSolution(edges[ii]->phi()) == 0 ||
                rounded_result.GetSolution(edges[ii]->phi()) == 1);
  }

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  options.convex_relaxation = false;
  options.preprocessing = false;
  options.max_rounded_paths = 0;
  auto mip_result = spp.SolveShortestPath(source->id(), target->id(), options);
  EXPECT_NEAR(rounded_result.get_optimal_cost(), mip_result.get_optimal_cost(),
              2e-6);
}

// In some cases, the depth first search performed in rounding will lead to a
// dead end. This test confirms that the search can backtrack to explore a new
// branch. Note that this test is only effective when Mosek or Gurobi is
// enabled. Otherwise, the failure mode that required backtracking in the depth
// first search is not triggered.
GTEST_TEST(ShortestPathTest, RoundingBacktrack) {
  GraphOfConvexSets spp;

  Vertex* source = spp.AddVertex(Point(Vector2d(1, -2)), "source");
  Vertex* target = spp.AddVertex(Point(Vector2d(0.4, 2.5)), "target");

  Vector3d b(3, 3, -1);
  Vertex* v0 = spp.AddVertex(
      HPolyhedron((Matrix<double, 3, 2>() << -1, 0, 0, -1, 1, 1).finished(), b)
          .CartesianPower(2));
  Vertex* v1 = spp.AddVertex(
      HPolyhedron((Matrix<double, 3, 2>() << 1, 0, 0, -1, -1, 1).finished(), b)
          .CartesianPower(2));
  Vertex* v2 = spp.AddVertex(
      HPolyhedron((Matrix<double, 3, 2>() << -1, 0, 0, 1, 1, -1).finished(), b)
          .CartesianPower(2));
  Vertex* v3 = spp.AddVertex(
      HPolyhedron((Matrix<double, 3, 2>() << 1, 0, 0, 1, -1, -1).finished(), b)
          .CartesianPower(2));

  std::vector<Edge*> source_edges;
  std::vector<Edge*> region_edges;
  std::vector<Edge*> target_edges;

  source_edges.push_back(spp.AddEdge(*source, *v0));
  source_edges.push_back(spp.AddEdge(*source, *v1));

  region_edges.push_back(spp.AddEdge(*v0, *v1));
  region_edges.push_back(spp.AddEdge(*v1, *v0));
  region_edges.push_back(spp.AddEdge(*v0, *v2));
  region_edges.push_back(spp.AddEdge(*v2, *v0));
  region_edges.push_back(spp.AddEdge(*v1, *v3));
  region_edges.push_back(spp.AddEdge(*v3, *v1));
  region_edges.push_back(spp.AddEdge(*v2, *v3));
  region_edges.push_back(spp.AddEdge(*v3, *v2));

  target_edges.push_back(spp.AddEdge(*v2, *target));
  target_edges.push_back(spp.AddEdge(*v3, *target));

  Matrix<double, 2, 4> A_diff;
  A_diff << -Matrix2d::Identity(), Matrix2d::Identity();
  auto continuity_con = std::make_shared<solvers::LinearEqualityConstraint>(
      A_diff, Vector2d::Zero());
  for (Edge* e : source_edges) {
    e->AddConstraint(
        solvers::Binding(continuity_con, {e->xu(), e->xv().head<2>()}));
  }
  for (Edge* e : source_edges) {
    e->AddConstraint(solvers::Binding(continuity_con,
                                      {e->xu().tail<2>(), e->xv().head<2>()}));
  }
  for (Edge* e : target_edges) {
    e->AddConstraint(
        solvers::Binding(continuity_con, {e->xu().tail<2>(), e->xv()}));
  }

  auto cost = std::make_shared<solvers::L2NormCost>(A_diff, Vector2d::Zero());
  v0->AddCost(solvers::Binding(cost, v0->x()));
  v1->AddCost(solvers::Binding(cost, v1->x()));
  v2->AddCost(solvers::Binding(cost, v2->x()));
  v3->AddCost(solvers::Binding(cost, v3->x()));

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;
  options.preprocessing = false;
  options.max_rounded_paths = 10;
  auto result = spp.SolveShortestPath(source->id(), target->id(), options);
  ASSERT_TRUE(result.is_success());
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

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = false;
  options.preprocessing = false;
  auto result = spp.SolveShortestPath(source->id(), target->id(), options);
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
    auto new_result =
        spp.SolveShortestPath(source->id(), target->id(), options);
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
    auto new_result =
        spp.SolveShortestPath(source->id(), target->id(), options);
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

// Section 11.4.1 (and the corresponding Figure 9) from the original Graph Of
// Convex Sets paper (https://arxiv.org/abs/2101.11565) gives an instance where
// the convex relaxation is loose.
GTEST_TEST(ShortestPathTest, Figure9) {
  GraphOfConvexSets spp;

  const Vertex* source = spp.AddVertex(Point(Vector2d::Zero()), "source");
  const Vertex* v1 = spp.AddVertex(Point(Vector2d(0, 2)));
  const Vertex* v2 = spp.AddVertex(Point(Vector2d(0, -2)));
  const Vertex* v3 =
      spp.AddVertex(HPolyhedron::MakeBox(Vector2d(2, -2), Vector2d(4, 2)));
  const Vertex* target = spp.AddVertex(Point(Vector2d(5, 0)), "target");

  Edge* e01 = spp.AddEdge(*source, *v1);
  Edge* e02 = spp.AddEdge(*source, *v2);
  Edge* e13 = spp.AddEdge(*v1, *v3);
  Edge* e23 = spp.AddEdge(*v2, *v3);
  Edge* e34 = spp.AddEdge(*v3, *target);

  // Edge length is distance for all edges.
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L2NormCost>(A, Vector2d::Zero());

  for (const auto& e : spp.Edges()) {
    e->AddCost(solvers::Binding(cost, {e->xu(), e->xv()}));
  }

  auto result = spp.SolveShortestPath(source->id(), target->id());
  ASSERT_TRUE(result.is_success());

  const double kTol = 2e-4;  // Gurobi required this large tolerance.
  EXPECT_NEAR(result.GetSolution(e01->phi()), 0.5, kTol);
  EXPECT_NEAR(result.GetSolution(e02->phi()), 0.5, kTol);
  EXPECT_NEAR(result.GetSolution(e13->phi()), 0.5, kTol);
  EXPECT_NEAR(result.GetSolution(e23->phi()), 0.5, kTol);
  EXPECT_NEAR(result.GetSolution(e34->phi()), 1.0, kTol);

  EXPECT_TRUE(CompareMatrices(v1->GetSolution(result), Vector2d(0, 2), kTol));
  EXPECT_TRUE(CompareMatrices(v2->GetSolution(result), Vector2d(0, -2), kTol));
  EXPECT_TRUE(v3->GetSolution(result)[0] > 2.0 - kTol);
  EXPECT_TRUE(v3->GetSolution(result)[0] < 4.0 - kTol);
  EXPECT_NEAR(v3->GetSolution(result)[1], 0, kTol);

  // Test that rounding returns the convex relaxation when relaxation is
  // feasible but integer solution is not.
  //
  // CSDP crashes when fed an infeasible problem so don't test behaviour under
  // CSDP when problem is not integer feasible.
  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  e13->AddConstraint(e13->xu()[1] == e13->xv()[1]);
  e23->AddConstraint(e23->xu()[1] == e23->xv()[1]);
  e34->AddConstraint(e34->xu()[1] == e34->xv()[1]);

  auto relaxed_result = spp.SolveShortestPath(source->id(), target->id());
  GraphOfConvexSetsOptions options;
  options.max_rounded_paths = 1;
  auto rounded_result =
      spp.SolveShortestPath(source->id(), target->id(), options);

  ASSERT_TRUE(relaxed_result.is_success());
  EXPECT_EQ(rounded_result.get_solution_result(),
            SolutionResult::kIterationLimit);
  for (const auto& e : spp.Edges()) {
    EXPECT_NEAR(relaxed_result.GetSolution(e->phi()),
                rounded_result.GetSolution(e->phi()), 1e-10);
    EXPECT_TRUE(CompareMatrices(relaxed_result.GetSolution(e->xu()),
                                rounded_result.GetSolution(e->xu()), 1e-12));
    EXPECT_TRUE(CompareMatrices(relaxed_result.GetSolution(e->xv()),
                                rounded_result.GetSolution(e->xv()), 1e-12));
    EXPECT_NEAR(e->GetSolutionCost(relaxed_result),
                e->GetSolutionCost(rounded_result), 1e-10);
  }
}

GTEST_TEST(ShortestPathTest, Graphviz) {
  GraphOfConvexSets g;
  auto source = g.AddVertex(Point(Vector2d{1.0, 2.}), "source");
  auto target = g.AddVertex(Point(Vector1d{1e-6}), "target");
  g.AddEdge(*source, *target, "edge");

  GraphOfConvexSetsOptions options;
  options.preprocessing = true;

  // Note: Testing the entire string against a const string is too fragile,
  // since the VertexIds are Identifier<> and increment on a global counter.
  EXPECT_THAT(
      g.GetGraphvizString(),
      AllOf(HasSubstr("source"), HasSubstr("target"), HasSubstr("edge")));
  auto result = g.SolveShortestPath(*source, *target, options);
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
              AllOf(HasSubstr("x = [1 2]"), HasSubstr("x = [1e-06]")));
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
TEST_F(ThreePoints, DeprecatedLinearCost1) {
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

  // Alternative signatures.
  auto result2 = g_.SolveShortestPath(*source_, *target_, true);
  ASSERT_TRUE(result2.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result2), 1.0, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result2), 0.0, 1e-6);

  if (solvers::ClpSolver::is_available()) {
    solvers::ClpSolver solver;
    auto result3 = g_.SolveShortestPath(*source_, *target_, true, &solver);
    ASSERT_TRUE(result3.is_success());
    EXPECT_NEAR(e_on_->GetSolutionCost(result3), 1.0, 1e-6);
    EXPECT_NEAR(e_off_->GetSolutionCost(result3), 0.0, 1e-6);
  }

  SolverOptions solver_options;
  auto result4 =
      g_.SolveShortestPath(*source_, *target_, true, nullptr, solver_options);
  ASSERT_TRUE(result4.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result4), 1.0, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result4), 0.0, 1e-6);

  EXPECT_TRUE(
      CompareMatrices(source_->GetSolution(result4), p_source_.x(), 1e-6));
  EXPECT_TRUE(
      CompareMatrices(target_->GetSolution(result4), p_target_.x(), 1e-6));
  EXPECT_TRUE(sink_->GetSolution(result4).hasNaN());
}

TEST_F(ThreePoints, DeprecatedConvexRelaxation) {
  e_on_->AddCost(1.0);
  e_off_->AddCost(1.0);
  auto result = g_.SolveShortestPath(source_->id(), target_->id(), true);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(e_on_->phi()), 1.0, 1e-6);
  EXPECT_NEAR(result.GetSolution(e_off_->phi()), 0.0, 1e-6);

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  auto result2 = g_.SolveShortestPath(source_->id(), target_->id(), false);
  ASSERT_TRUE(result2.is_success());
  EXPECT_NEAR(result2.GetSolution(e_on_->phi()), 1.0, 1e-6);
  EXPECT_NEAR(result2.GetSolution(e_off_->phi()), 0.0, 1e-6);

  EXPECT_NEAR(e_on_->GetSolutionCost(result), e_on_->GetSolutionCost(result2),
              1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), e_off_->GetSolutionCost(result2),
              1e-6);
}

// A simple shortest-path problem where the continuous variables do not effect
// the problem (they are all equality constrained).  The GraphOfConvexSets class
// should still solve the problem, and the convex relaxation should be optimal.
GTEST_TEST(ShortestPathTest, DeprecatedClassicalShortestPath) {
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

GTEST_TEST(ShortestPathTest, DeprecatedTwoStepLoopConstraint) {
  GraphOfConvexSets spp;

  spp.AddVertex(Point(Vector2d(0, 0)));
  spp.AddVertex(HPolyhedron::MakeBox(Vector2d(1.3, 1.3), Vector2d(2.7, 2.7)));
  spp.AddVertex(HPolyhedron::MakeBox(Vector2d(3.3, 1.3), Vector2d(4.7, 2.7)));
  spp.AddVertex(HPolyhedron::MakeBox(Vector2d(1.3, -2.7), Vector2d(2.7, -1.3)));
  spp.AddVertex(HPolyhedron::MakeBox(Vector2d(3.3, -2.7), Vector2d(4.7, -1.3)));
  spp.AddVertex(Point(Vector2d(6, 0)));

  auto v = spp.Vertices();

  // |xu - xv|₂
  Matrix<double, 4, 4> A = Matrix<double, 4, 4>::Identity();
  A.block(0, 2, 2, 2) = -Matrix2d::Identity();
  A.block(2, 0, 2, 2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::QuadraticCost>(A, Vector4d::Zero());

  spp.AddEdge(*v[0], *v[1])
      ->AddCost(solvers::Binding(cost, {v[0]->x(), v[1]->x()}));
  spp.AddEdge(*v[1], *v[0])
      ->AddCost(solvers::Binding(cost, {v[1]->x(), v[0]->x()}));
  spp.AddEdge(*v[1], *v[2])
      ->AddCost(solvers::Binding(cost, {v[1]->x(), v[2]->x()}));
  spp.AddEdge(*v[2], *v[1])
      ->AddCost(solvers::Binding(cost, {v[2]->x(), v[1]->x()}));
  spp.AddEdge(*v[2], *v[5])
      ->AddCost(solvers::Binding(cost, {v[2]->x(), v[5]->x()}));
  spp.AddEdge(*v[5], *v[2])
      ->AddCost(solvers::Binding(cost, {v[5]->x(), v[2]->x()}));

  spp.AddEdge(*v[0], *v[3])
      ->AddCost(solvers::Binding(cost, {v[0]->x(), v[3]->x()}));
  spp.AddEdge(*v[3], *v[0])
      ->AddCost(solvers::Binding(cost, {v[3]->x(), v[0]->x()}));
  spp.AddEdge(*v[3], *v[4])
      ->AddCost(solvers::Binding(cost, {v[3]->x(), v[4]->x()}));
  spp.AddEdge(*v[4], *v[3])
      ->AddCost(solvers::Binding(cost, {v[4]->x(), v[3]->x()}));
  spp.AddEdge(*v[4], *v[5])
      ->AddCost(solvers::Binding(cost, {v[4]->x(), v[5]->x()}));
  spp.AddEdge(*v[5], *v[4])
      ->AddCost(solvers::Binding(cost, {v[5]->x(), v[4]->x()}));

  auto result = spp.SolveShortestPath(*v[0], *v[5], true);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  EXPECT_TRUE(result.is_success());

  int non_zero_edges = 0;
  for (const auto& e : spp.Edges()) {
    // Tuned so that off edges are below this value for all solvers
    if (result.GetSolution(e->phi()) > 1e-5) {
      ++non_zero_edges;
    }
  }
  EXPECT_EQ(non_zero_edges, 6);
}

// Test that all optimization variables are properly set, even when constrained
// to be on or off.
GTEST_TEST(ShortestPathTest, DeprecatedPhiConstraint) {
  GraphOfConvexSets spp;

  spp.AddVertex(Point(Vector2d(0, 0)));
  spp.AddVertex(Point(Vector2d(1, -1)));
  spp.AddVertex(Point(Vector2d(1, 1)));
  spp.AddVertex(Point(Vector2d(2, 0)));

  auto v = spp.Vertices();

  Edge* edge_01 = spp.AddEdge(*v[0], *v[1]);
  Edge* edge_02 = spp.AddEdge(*v[0], *v[2]);
  Edge* edge_13 = spp.AddEdge(*v[1], *v[3]);
  Edge* edge_23 = spp.AddEdge(*v[2], *v[3]);

  // |xu - xv|₂
  Matrix<double, 4, 4> A = Matrix<double, 4, 4>::Identity();
  A.block(0, 2, 2, 2) = -Matrix2d::Identity();
  A.block(2, 0, 2, 2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::QuadraticCost>(A, Vector4d::Zero());

  edge_01->AddCost(solvers::Binding(cost, {v[0]->x(), v[1]->x()}));
  edge_02->AddCost(solvers::Binding(cost, {v[0]->x(), v[2]->x()}));
  edge_13->AddCost(solvers::Binding(cost, {v[1]->x(), v[3]->x()}));
  edge_23->AddCost(solvers::Binding(cost, {v[2]->x(), v[3]->x()}));

  // Confirm that variables for edges are set when no on/off constraint is
  // imposed.
  {
    auto result = spp.SolveShortestPath(*v[0], *v[3], true);
    if (result.get_solver_id() == solvers::IpoptSolver::id()) {
      return;  // See IpoptTest for details.
    }
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXu(result),
                                0.5 * Vector2d(1, -1), 1e-6));
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXv(result),
                                0.5 * Vector2d(2, 0), 1e-6));
    EXPECT_NEAR(edge_13->GetSolutionCost(result), 0.5 * 1, 1e-6);
    EXPECT_NEAR(result.GetSolution(edge_13->phi()), 0.5, 1e-6);
    EXPECT_TRUE(
        CompareMatrices(v[1]->GetSolution(result), Vector2d(1, -1), 1e-6));
  }

  // Confirm that variables for edges that are turned off are properly set.
  // This check is necessary now that these variables have been removed from the
  // optimization problem.
  edge_13->AddPhiConstraint(false);
  {
    auto result = spp.SolveShortestPath(*v[0], *v[3], true);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXu(result),
                                Vector2d::Zero(), 1e-6));
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXv(result),
                                Vector2d::Zero(), 1e-6));
    EXPECT_NEAR(edge_13->GetSolutionCost(result), 0, 1e-6);
    EXPECT_NEAR(result.GetSolution(edge_13->phi()), 0, 1e-6);
  }

  // Confirm that variables for edges that are turned on are properly set.
  edge_13->AddPhiConstraint(true);
  {
    auto result = spp.SolveShortestPath(*v[0], *v[3], true);
    if (result.get_solver_id() == solvers::IpoptSolver::id()) {
      return;  // See IpoptTest for details.
    }
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXu(result),
                                Vector2d(1, -1), 1e-6));
    EXPECT_TRUE(CompareMatrices(edge_13->GetSolutionPhiXv(result),
                                Vector2d(2, 0), 1e-6));
    EXPECT_NEAR(edge_13->GetSolutionCost(result), 1, 1e-6);
    EXPECT_NEAR(result.GetSolution(edge_13->phi()), 1, 1e-6);
  }
}

GTEST_TEST(ShortestPathTest, DeprecatedTobiasToyExample) {
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

// Section 11.4.1 (and the corresponding Figure 9) from the original Graph Of
// Convex Sets paper (https://arxiv.org/abs/2101.11565) gives an instance where
// the convex relaxation is loose.
GTEST_TEST(ShortestPathTest, DeprecatedFigure9) {
  GraphOfConvexSets spp;

  const Vertex* source = spp.AddVertex(Point(Vector2d::Zero()), "source");
  const Vertex* v1 = spp.AddVertex(Point(Vector2d(0, 2)));
  const Vertex* v2 = spp.AddVertex(Point(Vector2d(0, -2)));
  const Vertex* v3 =
      spp.AddVertex(HPolyhedron::MakeBox(Vector2d(2, -2), Vector2d(4, 2)));
  const Vertex* target = spp.AddVertex(Point(Vector2d(5, 0)), "target");

  const Edge* e01 = spp.AddEdge(*source, *v1);
  const Edge* e02 = spp.AddEdge(*source, *v2);
  const Edge* e13 = spp.AddEdge(*v1, *v3);
  const Edge* e23 = spp.AddEdge(*v2, *v3);
  const Edge* e34 = spp.AddEdge(*v3, *target);

  // Edge length is distance for all edges.
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L2NormCost>(A, Vector2d::Zero());

  for (const auto& e : spp.Edges()) {
    e->AddCost(solvers::Binding(cost, {e->xu(), e->xv()}));
  }

  auto result = spp.SolveShortestPath(source->id(), target->id(), true);
  ASSERT_TRUE(result.is_success());

  const double kTol = 2e-4;  // Gurobi required this large tolerance.
  EXPECT_NEAR(result.GetSolution(e01->phi()), 0.5, kTol);
  EXPECT_NEAR(result.GetSolution(e02->phi()), 0.5, kTol);
  EXPECT_NEAR(result.GetSolution(e13->phi()), 0.5, kTol);
  EXPECT_NEAR(result.GetSolution(e23->phi()), 0.5, kTol);
  EXPECT_NEAR(result.GetSolution(e34->phi()), 1.0, kTol);

  EXPECT_TRUE(CompareMatrices(v1->GetSolution(result), Vector2d(0, 2), kTol));
  EXPECT_TRUE(CompareMatrices(v2->GetSolution(result), Vector2d(0, -2), kTol));
  EXPECT_TRUE(v3->GetSolution(result)[0] > 2.0 - kTol);
  EXPECT_TRUE(v3->GetSolution(result)[0] < 4.0 - kTol);
  EXPECT_NEAR(v3->GetSolution(result)[1], 0, kTol);
}
#pragma GCC diagnostic pop

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
