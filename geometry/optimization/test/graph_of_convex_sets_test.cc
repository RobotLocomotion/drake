#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <forward_list>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/linear_system_solver.h"
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
using solvers::internal::CreateBinding;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Substitution;
using symbolic::Variables;

using Edge = GraphOfConvexSets::Edge;
using EdgeId = GraphOfConvexSets::EdgeId;
using Transcription = GraphOfConvexSets::Transcription;
using Vertex = GraphOfConvexSets::Vertex;
using VertexId = GraphOfConvexSets::VertexId;

using ::testing::AllOf;
using ::testing::HasSubstr;
using ::testing::Not;

const double kInf = std::numeric_limits<double>::infinity();

namespace {
bool MixedIntegerSolverAvailable() {
  return (solvers::MosekSolver::is_available() &&
          solvers::MosekSolver::is_enabled()) ||
         (solvers::GurobiSolver::is_available() &&
          solvers::GurobiSolver::is_enabled());
}
}  // namespace

GTEST_TEST(GraphOfConvexSetsOptionsTest, Serialize) {
  GraphOfConvexSetsOptions options;
  options.convex_relaxation = false;
  options.max_rounded_paths = false;
  options.preprocessing = true;
  options.max_rounding_trials = 5;
  options.flow_tolerance = 0.01;
  options.rounding_seed = 5;
  solvers::MosekSolver mosek_solver;
  options.solver = &mosek_solver;
  options.solver_options = solvers::SolverOptions();
  solvers::IpoptSolver ipopt_solver;
  options.restriction_solver = &ipopt_solver;
  options.restriction_solver_options = solvers::SolverOptions();
  const std::string serialized = yaml::SaveYamlString(options);
  const auto deserialized =
      yaml::LoadYamlString<GraphOfConvexSetsOptions>(serialized);
  EXPECT_EQ(deserialized.convex_relaxation, options.convex_relaxation);
  EXPECT_EQ(deserialized.preprocessing, options.preprocessing);
  EXPECT_EQ(deserialized.max_rounded_paths, options.max_rounded_paths);
  EXPECT_EQ(deserialized.max_rounding_trials, options.max_rounding_trials);
  EXPECT_EQ(deserialized.flow_tolerance, options.flow_tolerance);
  EXPECT_EQ(deserialized.rounding_seed, options.rounding_seed);
  // The non-built-in types are not serialized.
  EXPECT_EQ(deserialized.solver, nullptr);
  EXPECT_EQ(deserialized.restriction_solver, nullptr);
  EXPECT_EQ(deserialized.solver_options, solvers::SolverOptions());
  EXPECT_FALSE(deserialized.restriction_solver_options.has_value());
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
  Edge* e = g.AddEdge(u, v, "e");

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
  Edge* e1 = g.AddEdge(u, v, "e1");
  Edge* e2 = g.AddEdge(v, u, "e2");

  EXPECT_EQ(g.Edges().size(), 2);

  EXPECT_EQ(u->incoming_edges().size(), 1);
  EXPECT_EQ(u->incoming_edges()[0], e2);
  EXPECT_EQ(u->outgoing_edges().size(), 1);
  EXPECT_EQ(u->outgoing_edges()[0], e1);
  EXPECT_EQ(v->incoming_edges().size(), 1);
  EXPECT_EQ(v->incoming_edges()[0], e1);
  EXPECT_EQ(v->outgoing_edges().size(), 1);
  EXPECT_EQ(v->outgoing_edges()[0], e2);

  g.RemoveEdge(e1);
  auto edges = g.Edges();
  EXPECT_EQ(edges.size(), 1);
  EXPECT_EQ(edges.at(0), e2);
  EXPECT_EQ(u->incoming_edges().size(), 1);
  EXPECT_EQ(u->incoming_edges()[0], e2);
  EXPECT_EQ(u->outgoing_edges().size(), 0);
  EXPECT_EQ(v->incoming_edges().size(), 0);
  EXPECT_EQ(v->outgoing_edges().size(), 1);
  EXPECT_EQ(v->outgoing_edges()[0], e2);

  g.RemoveEdge(e2);
  EXPECT_EQ(g.Edges().size(), 0);
  EXPECT_EQ(u->incoming_edges().size(), 0);
  EXPECT_EQ(u->outgoing_edges().size(), 0);
  EXPECT_EQ(v->incoming_edges().size(), 0);
  EXPECT_EQ(v->outgoing_edges().size(), 0);
}

GTEST_TEST(GraphOfConvexSetsTest, RemoveVertex) {
  GraphOfConvexSets g;
  Vertex* v1 = g.AddVertex(Point(Vector2d(3., 5.)));
  Vertex* v2 = g.AddVertex(Point(Vector2d(-2., 4.)));
  Vertex* v3 = g.AddVertex(Point(Vector2d(5., -2.3)));
  Edge* e1 = g.AddEdge(v1, v2);
  g.AddEdge(v1, v3);
  g.AddEdge(v3, v1);
  EXPECT_EQ(v1->incoming_edges().size(), 1);
  EXPECT_EQ(v1->outgoing_edges().size(), 2);

  EXPECT_EQ(g.Vertices().size(), 3);
  EXPECT_EQ(g.Edges().size(), 3);

  g.RemoveVertex(v3);
  EXPECT_EQ(g.Vertices().size(), 2);
  auto edges = g.Edges();
  EXPECT_EQ(edges.size(), 1);
  EXPECT_EQ(edges.at(0), e1);
  EXPECT_EQ(v1->incoming_edges().size(), 0);

  g.RemoveVertex(v2);
  auto vertices = g.Vertices();
  EXPECT_EQ(vertices.size(), 1);
  EXPECT_EQ(vertices.at(0), v1);
  EXPECT_EQ(g.Edges().size(), 0);
  EXPECT_EQ(v1->outgoing_edges().size(), 0);
}

/*
┌───┐       ┌───┐
│ u ├───e──►│ v │
└───┘       └───┘
*/
class TwoPoints : public ::testing::Test {
 protected:
  TwoPoints() : pu_{Vector2d(1., 2.)}, pv_{Vector3d(3., 4., 5.)} {
    u_ = g_.AddVertex(pu_, "u");
    v_ = g_.AddVertex(pv_, "v");
    e_ = g_.AddEdge(u_, v_, "e");
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

// Confirms that we can add costs (both ways) and get the solution.
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

// Confirms that we can add constraints (both ways).
// The correctness of the added constraints will be established by the solution
// tests.
TEST_F(TwoPoints, AddConstraint) {
  auto b0 = e_->AddConstraint(e_->xv().head<2>() == e_->xu());
  auto constraint = std::make_shared<LinearConstraint>(
      Matrix2d::Identity(), Vector2d::Zero(), Vector2d{1.2, 3.4});
  auto b1 = e_->AddConstraint(Binding(constraint, e_->xu()));
  auto u_b0 = u_->AddConstraint(u_->x() == pu_.x());
  auto u_b1 = u_->AddConstraint(Binding(constraint, u_->x()));
  // If no transcription is specified, the constraint won't be added.
  EXPECT_THROW(e_->AddConstraint(e_->xv().head<2>() == e_->xu(), {}),
               std::exception);
  EXPECT_THROW(e_->AddConstraint(Binding(constraint, e_->xu()), {}),
               std::exception);
  EXPECT_THROW(u_->AddConstraint(u_->x() == pu_.x(), {}), std::exception);
  EXPECT_THROW(u_->AddConstraint(Binding(constraint, u_->x()), {}),
               std::exception);

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
  const auto& all_edge_constraints = e_->GetConstraints();
  EXPECT_EQ(all_edge_constraints.size(), 2);
  EXPECT_EQ(all_edge_constraints[0], b0);
  EXPECT_EQ(all_edge_constraints[1], b1);
  const auto& all_vertex_constraints = u_->GetConstraints();
  EXPECT_EQ(all_vertex_constraints.size(), 2);
  EXPECT_EQ(all_vertex_constraints[0], u_b0);
  EXPECT_EQ(all_vertex_constraints[1], u_b1);

  // By default Edge::AddConstraint or Vertex::AddConstraint is adding the
  // binding to all transcriptions.
  for (const auto& transcription :
       {Transcription::kMIP, Transcription::kRelaxation,
        Transcription::kRestriction}) {
    const auto& edge_constraints = e_->GetConstraints({transcription});
    EXPECT_EQ(edge_constraints.size(), 2);
    EXPECT_EQ(edge_constraints[0], b0);
    EXPECT_EQ(edge_constraints[1], b1);
    const auto& vertex_constraints = u_->GetConstraints({transcription});
    EXPECT_EQ(vertex_constraints.size(), 2);
    EXPECT_EQ(vertex_constraints[0], u_b0);
    EXPECT_EQ(vertex_constraints[1], u_b1);
  }
  // If no transcription is specified, nothing will be returned.
  EXPECT_THROW(e_->GetConstraints({}), std::exception);
  EXPECT_THROW(u_->GetConstraints({}), std::exception);

  symbolic::Variable other_var("x");
  DRAKE_EXPECT_THROWS_MESSAGE(e_->AddConstraint(other_var == 1),
                              ".*IsSubsetOf.*");
  DRAKE_EXPECT_THROWS_MESSAGE(u_->AddConstraint(other_var == 1),
                              ".*IsSubsetOf.*");
}

// Verifies that the correct solver is used for the MIP, relaxation and the
// restriction.
TEST_F(TwoPoints, ReportCorrectSolverId) {
  e_->AddCost((e_->xv().head<2>() - e_->xu()).squaredNorm());
  GraphOfConvexSetsOptions options;
  // Define a different solver for the restriction.
  solvers::ClarabelSolver clarabel;
  options.solver = &clarabel;
  solvers::ClpSolver clp;
  options.restriction_solver = &clp;
  options.convex_relaxation = true;
  options.preprocessing = false;

  // When solving the convex relaxation with no rounded paths, we expect the
  // options.solver to be used.
  options.max_rounded_paths = 0;
  auto result = g_.SolveShortestPath(*u_, *v_, options);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(result.get_solver_id(), options.solver->solver_id());

  // The convex restriction should use the restriction solver.
  result = g_.SolveConvexRestriction({e_}, options);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(result.get_solver_id(), options.restriction_solver->solver_id());

  // With the rounding on, the reported solver should be the restriciton solver
  options.max_rounded_paths = 1;
  result = g_.SolveShortestPath(*u_, *v_, options);
  EXPECT_TRUE(result.is_success());
  EXPECT_EQ(result.get_solver_id(), options.restriction_solver->solver_id());

  // Even if we add a constraint that makes only the rounding fail, the
  // reported solver should still be the restriction solver.
  u_->AddConstraint(u_->x()[0] == 0.0, {Transcription::kRestriction});
  result = g_.SolveShortestPath(*u_, *v_, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solver_id(), options.restriction_solver->solver_id());

  // Adding infeasible constraints to the relaxation should fail the relaxation,
  // hence solving the restriction will never be reached and the reported solver
  // should be the solver used in the relaxation.
  e_->AddConstraint(e_->xv()[0] == 0.0, {Transcription::kRelaxation});
  result = g_.SolveShortestPath(*u_, *v_, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solver_id(), options.solver->solver_id());

  // Since we haven't added any infeasible constraints to the MIP transcription,
  // we expect the solve to be successful and the reported solver to be the MIP
  // solver.
  if (solvers::MosekSolver::is_available() &&
      solvers::MosekSolver::is_enabled()) {
    solvers::MosekSolver mosek;
    options.solver = &mosek;
    options.convex_relaxation = false;
    result = g_.SolveShortestPath(*u_, *v_, options);
    EXPECT_TRUE(result.is_success());
    EXPECT_EQ(result.get_solver_id(), options.solver->solver_id());
  }
}

// Verify that assigning a transcription to a constraint adds it to the correct
// problem. This test will be split into multiple cases.
TEST_F(TwoPoints, VerifyTranscriptionAssignmentBaseline) {
  // We will be adding a few feasible constraints to all transcriptions, even
  // though there are redundant with the set constraint. It is expected that
  // solving the individual problems will be successful.
  e_->AddCost((e_->xv().head<2>() - e_->xu()).squaredNorm());
  e_->AddConstraint(e_->xv()[0] == 3.0,
                    {Transcription::kMIP, Transcription::kRelaxation,
                     Transcription::kRestriction});
  u_->AddConstraint(u_->x()[0] == 1.0,
                    {Transcription::kMIP, Transcription::kRelaxation,
                     Transcription::kRestriction});

  GraphOfConvexSetsOptions options;
  options.preprocessing = false;

  // The convex relaxation should be successful.
  options.convex_relaxation = true;
  options.max_rounded_paths = 0;
  EXPECT_TRUE(g_.SolveShortestPath(*u_, *v_, options).is_success());

  // The restriction, also in the rounding, should be successful.
  options.convex_relaxation = true;
  options.max_rounded_paths = 1;
  EXPECT_TRUE(g_.SolveConvexRestriction({e_}, options).is_success());
  EXPECT_TRUE(g_.SolveShortestPath(*u_, *v_, options).is_success());

  // The MIP should be successful as well.
  options.convex_relaxation = false;
  if (MixedIntegerSolverAvailable()) {
    EXPECT_TRUE(g_.SolveShortestPath(*u_, *v_, options).is_success());
  }
}

TEST_F(TwoPoints, VerifyTranscriptionAssignmentkMIP) {
  // We will be adding a few feasible constraints to all transcriptions, even
  // though there are redundant with the set constraint.
  // Further, we will add an infeasible constraint to the relaxation and the
  // restriction, but not the MIP.
  e_->AddCost((e_->xv().head<2>() - e_->xu()).squaredNorm());
  e_->AddConstraint(e_->xv()[0] == 3.0,
                    {Transcription::kMIP, Transcription::kRelaxation,
                     Transcription::kRestriction});
  u_->AddConstraint(u_->x()[0] == 1.0,
                    {Transcription::kMIP, Transcription::kRelaxation,
                     Transcription::kRestriction});

  GraphOfConvexSetsOptions options;
  options.preprocessing = false;

  // The MIP should be successful.
  e_->AddConstraint(e_->xv()[0] == 0.0,
                    {Transcription::kRelaxation, Transcription::kRestriction});
  u_->AddConstraint(u_->x()[0] == 0.0,
                    {Transcription::kRelaxation, Transcription::kRestriction});
  if (MixedIntegerSolverAvailable()) {
    options.convex_relaxation = false;
    EXPECT_TRUE(g_.SolveShortestPath(*u_, *v_, options).is_success());
  }
  // Ensure the relaxation fails.
  options.convex_relaxation = true;
  options.max_rounded_paths = 0;
  EXPECT_FALSE(g_.SolveShortestPath(*u_, *v_, options).is_success());

  // Ensure the restriction fails.
  EXPECT_FALSE(g_.SolveConvexRestriction({e_}, options).is_success());
}

TEST_F(TwoPoints, VerifyTranscriptionAssignmentkRelaxation) {
  // We will be adding a few feasible constraints to all transcriptions, even
  // though there are redundant with the set constraint.
  // Further, we will add an infeasible constraint to the MIP and the
  // restriction, but not the relaxation.
  e_->AddCost((e_->xv().head<2>() - e_->xu()).squaredNorm());
  e_->AddConstraint(e_->xv()[0] == 3.0,
                    {Transcription::kMIP, Transcription::kRelaxation,
                     Transcription::kRestriction});
  u_->AddConstraint(u_->x()[0] == 1.0,
                    {Transcription::kMIP, Transcription::kRelaxation,
                     Transcription::kRestriction});

  GraphOfConvexSetsOptions options;
  options.preprocessing = false;

  // The relaxation should be successful.
  e_->AddConstraint(e_->xv()[0] == 0.0,
                    {Transcription::kMIP, Transcription::kRestriction});
  u_->AddConstraint(u_->x()[0] == 0.0,
                    {Transcription::kMIP, Transcription::kRestriction});
  options.convex_relaxation = true;
  options.max_rounded_paths = 0;
  EXPECT_TRUE(g_.SolveShortestPath(*u_, *v_, options).is_success());

  // Ensure the mip fails.
  if (MixedIntegerSolverAvailable()) {
    options.convex_relaxation = false;
    EXPECT_FALSE(g_.SolveShortestPath(*u_, *v_, options).is_success());
  }

  // Ensure the restriction fails.
  EXPECT_FALSE(g_.SolveConvexRestriction({e_}, options).is_success());

  // Ensure the restriction called in the rounding fails.
  options.convex_relaxation = true;
  options.max_rounded_paths = 1;
  EXPECT_FALSE(g_.SolveShortestPath(*u_, *v_, options).is_success());
}
TEST_F(TwoPoints, VerifyTranscriptionAssignmentkRestriction) {
  // We will be adding a few feasible constraints to all transcriptions, even
  // though there are redundant with the set constraint.
  // Further, we will add an infeasible constraint to the MIP and the
  // relaxation, but not the restriction.
  e_->AddCost((e_->xv().head<2>() - e_->xu()).squaredNorm());
  e_->AddConstraint(e_->xv()[0] == 3.0,
                    {Transcription::kMIP, Transcription::kRelaxation,
                     Transcription::kRestriction});
  u_->AddConstraint(u_->x()[0] == 1.0,
                    {Transcription::kMIP, Transcription::kRelaxation,
                     Transcription::kRestriction});

  GraphOfConvexSetsOptions options;
  options.preprocessing = false;

  // The restriction should be successful.
  e_->AddConstraint(e_->xv()[0] == 0.0,
                    {Transcription::kMIP, Transcription::kRelaxation});
  u_->AddConstraint(u_->x()[0] == 0.0,
                    {Transcription::kMIP, Transcription::kRelaxation});
  EXPECT_TRUE(g_.SolveConvexRestriction({e_}, options).is_success());

  // Ensure the mip fails.
  if (MixedIntegerSolverAvailable()) {
    options.convex_relaxation = false;
    EXPECT_FALSE(g_.SolveShortestPath(*u_, *v_, options).is_success());
  }

  // Ensure the and relaxation fails.
  options.convex_relaxation = true;
  options.max_rounded_paths = 0;
  EXPECT_FALSE(g_.SolveShortestPath(*u_, *v_, options).is_success());
}

GTEST_TEST(GraphOfConvexSetsTest, TwoNullPointsConstraint) {
  GraphOfConvexSets g;
  Point pu(Eigen::VectorXd::Zero(0));
  Point pv(Eigen::VectorXd::Zero(0));
  Vertex* u = g.AddVertex(pu, "u");
  Vertex* v = g.AddVertex(pv, "v");
  Edge* e = g.AddEdge(u, v, "e");
  DRAKE_EXPECT_THROWS_MESSAGE(e->AddConstraint(symbolic::Formula::True()),
                              ".*total.*ambient.*dimension.*");
}

/* A graph with one edge definitely on the optimal path, and one definitely off
it.
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
    e_on_ = g_.AddEdge(source_, target_);
    e_off_ = g_.AddEdge(source_, sink_);

    subs_on_off_.emplace(e_on_->xv()[0], e_off_->xv()[0]);
    subs_on_off_.emplace(e_on_->xv()[1], e_off_->xv()[1]);

    options_.preprocessing = false;
    options_.convex_relaxation = true;
  }

  void CheckConvexRestriction(const MathematicalProgramResult& result) {
    MathematicalProgramResult restriction_result =
        g_.SolveConvexRestriction(std::vector<const Edge*>({e_on_}), options_);
    log()->info("Solved convex restriction with {}",
                restriction_result.get_solver_id().name());
    // Confirm that we get a convex solver (not an NLP solver).
    if (MixedIntegerSolverAvailable()) {
      EXPECT_TRUE(
          restriction_result.get_solver_id() == solvers::MosekSolver::id() ||
          restriction_result.get_solver_id() == solvers::GurobiSolver::id());
    }
    EXPECT_TRUE(restriction_result.is_success());
    EXPECT_NEAR(result.get_optimal_cost(),
                restriction_result.get_optimal_cost(), 1e-4);
    for (const auto* v : {source_, target_, sink_}) {
      EXPECT_TRUE(CompareMatrices(result.GetSolution(v->x()),
                                  restriction_result.GetSolution(v->x()),
                                  1e-6));
    }
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
  GraphOfConvexSetsOptions options_;
};

// Confirms that we get a helpful error message when we try to solve a problem
// and no MIP solver is available.
TEST_F(ThreePoints, NoMixedIntegerSolverAvailable) {
  if (MixedIntegerSolverAvailable()) {
    return;
  }

  options_.convex_relaxation = false;

  // Note: DRAKE_EXPECT_THROWS_MESSAGE fails to match the error message even
  // when the regex is ".*". So we implement a simpler a regex-free version
  // here.
  try {
    g_.SolveShortestPath(*source_, *target_, options_);
    GTEST_NONFATAL_FAILURE_("Should have thrown.");
  } catch (const std::exception& err) {
    EXPECT_NE(
        std::string(err.what())
            .find(
                "no solver available that can solve the mixed-integer version"),
        std::string::npos);
  } catch (...) {
    GTEST_NONFATAL_FAILURE_("Should have thrown std::exception.");
  }
}

TEST_F(ThreePoints, LinearCost1) {
  e_on_->AddCost(1.0);
  e_off_->AddCost(1.0);
  source_->AddCost(1.0);
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);

  options_.solver_options = SolverOptions();
  auto result4 = g_.SolveShortestPath(*source_, *target_, options_);
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
    options_.solver = &clp;
    auto result3 = g_.SolveShortestPath(*source_, *target_, options_);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(e_on_->phi()), 1.0, 1e-6);
  EXPECT_NEAR(result.GetSolution(e_off_->phi()), 0.0, 1e-6);

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  options_.convex_relaxation = false;
  auto result2 = g_.SolveShortestPath(*source_, *target_, options_);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), b, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), b, 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
  CheckConvexRestriction(result);
}

TEST_F(ThreePoints, LinearCost3) {
  const Vector2d a{2.3, 4.5};
  const double b = 1.23;
  e_on_->AddCost(a.dot(e_on_->xu()) + b);
  e_off_->AddCost(a.dot(e_off_->xu()) + b);
  source_->AddCost(a.dot(source_->x()) + b);
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), a.dot(p_source_.x()) + b, 1e-6);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), a.dot(p_source_.x()) + b, 1e-6);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
  CheckConvexRestriction(result);
}

TEST_F(ThreePoints, LinearCost4) {
  const double b = -1.23;
  e_on_->AddCost(b);
  DRAKE_EXPECT_THROWS_MESSAGE(
      g_.SolveShortestPath(*source_, *target_, options_),
      "Constant costs must be non-negative.*");
}

TEST_F(ThreePoints, LinearCost5) {
  const double b = -1.23;
  source_->AddCost(b);
  DRAKE_EXPECT_THROWS_MESSAGE(
      g_.SolveShortestPath(*source_, *target_, options_),
      "Constant costs must be non-negative.*");
}

TEST_F(ThreePoints, MultipleVertexCosts) {
  source_->AddCost(1.0);
  source_->AddCost(1.0);
  DRAKE_EXPECT_NO_THROW(g_.SolveShortestPath(*source_, *target_, options_));
}

TEST_F(ThreePoints, QuadraticCost) {
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm());
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm());
  source_->AddCost(
      static_cast<const VectorX<Expression>>(source_->x()).squaredNorm());

  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  if (result.get_solver_id() == solvers::IpoptSolver::id()) {
    return;  // See IpoptTest for details.
  }
  ASSERT_TRUE(result.is_success());
  Environment env{};
  env.insert(e_on_->xu(), p_source_.x());
  env.insert(e_on_->xv(), p_target_.x());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), cost.Evaluate(env), 2e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 4e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), vertex_cost.Evaluate(env),
              2e-5);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  Environment env{};
  env.insert(e_on_->xu(), p_source_.x());
  env.insert(e_on_->xv(), p_target_.x());
  EXPECT_NEAR(e_on_->GetSolutionCost(result), cost.Evaluate(env), 5e-5);
  EXPECT_NEAR(e_off_->GetSolutionCost(result), 0.0, 4e-6);
  EXPECT_NEAR(source_->GetSolutionCost(result), vertex_cost.Evaluate(env),
              2e-4);
  EXPECT_NEAR(target_->GetSolutionCost(result), 0.0, 1e-6);
  EXPECT_NEAR(sink_->GetSolutionCost(result), 0.0, 1e-6);
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);
}

// Costs must be strictly positive.
TEST_F(ThreePoints, QuadraticCost5) {
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm() - 2.0);
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm() - 2.0);

  DRAKE_EXPECT_THROWS_MESSAGE(
      g_.SolveShortestPath(*source_, *target_, options_),
      ".* must be strictly non-negative.*");
}

// Costs must be strictly positive.
TEST_F(ThreePoints, QuadraticCost6) {
  source_->AddCost(
      static_cast<const VectorX<Expression>>(source_->x()).squaredNorm() - 2.0);

  DRAKE_EXPECT_THROWS_MESSAGE(
      g_.SolveShortestPath(*source_, *target_, options_),
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  if (result.get_solver_id() == solvers::CsdpSolver::id()) {
    // CSDP 6.2.0 gets the wrong cost -- but correct solution -- in the convex
    // restriction (on linux only).
    return;
  }
  CheckConvexRestriction(result);
}

TEST_F(ThreePoints, GetSolutionPath) {
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  const auto path = g_.GetSolutionPath(*source_, *target_, result);
  ASSERT_EQ(path.size(), 1);
  EXPECT_EQ(path[0], e_on_);

  GraphOfConvexSets other;
  Vertex* v = other.AddVertex(HPolyhedron::MakeUnitBox(2), "other");

  // Bad source.
  DRAKE_EXPECT_THROWS_MESSAGE(g_.GetSolutionPath(*v, *target_, result),
                              ".*Source.*is not a vertex.*");
  // Bad target.
  DRAKE_EXPECT_THROWS_MESSAGE(g_.GetSolutionPath(*source_, *v, result),
                              ".*Target.*is not a vertex.*");

  // is_success is false.
  result.set_solution_result(solvers::SolutionResult::kInfeasibleConstraints);
  DRAKE_EXPECT_THROWS_MESSAGE(g_.GetSolutionPath(*source_, *target_, result),
                              ".*Cannot extract a solution.*");
}

// Like the ThreePoints, but with boxes for each vertex instead of points.
class ThreeBoxes : public ::testing::Test {
 protected:
  ThreeBoxes() {
    auto box = HPolyhedron::MakeUnitBox(2);
    source_ = g_.AddVertex(box, "source");
    target_ = g_.AddVertex(box, "target");
    sink_ = g_.AddVertex(box, "sink");
    e_on_ = g_.AddEdge(source_, target_);
    e_off_ = g_.AddEdge(source_, sink_);

    subs_on_off_.emplace(e_on_->xv()[0], e_off_->xv()[0]);
    subs_on_off_.emplace(e_on_->xv()[1], e_off_->xv()[1]);

    options_.preprocessing = true;
    options_.convex_relaxation = true;
  }

  void CheckConvexRestriction(const MathematicalProgramResult& result) {
    MathematicalProgramResult restriction_result =
        g_.SolveConvexRestriction(std::vector<const Edge*>({e_on_}), options_);
    // Confirm that we get a convex solver (not an NLP solver).
    if (MixedIntegerSolverAvailable()) {
      EXPECT_TRUE(
          restriction_result.get_solver_id() == solvers::MosekSolver::id() ||
          restriction_result.get_solver_id() == solvers::GurobiSolver::id());
    }
    EXPECT_TRUE(restriction_result.is_success());
    EXPECT_NEAR(result.get_optimal_cost(),
                restriction_result.get_optimal_cost(), 1e-6);
    for (const auto* v : {source_, target_, sink_}) {
      EXPECT_TRUE(CompareMatrices(result.GetSolution(v->x()),
                                  restriction_result.GetSolution(v->x()),
                                  1e-6));
    }
  }

  GraphOfConvexSets g_;
  Edge* e_on_{nullptr};
  Edge* e_off_{nullptr};
  Vertex* source_{nullptr};
  Vertex* target_{nullptr};
  Vertex* sink_{nullptr};
  Substitution subs_on_off_{};
  GraphOfConvexSetsOptions options_;
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
  options_.solver = &ipopt;
  options_.convex_relaxation = true;
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
}

TEST_F(ThreeBoxes, NonConvexRounding) {
  /* Consider the following problem:
  - Each of the sets source, target and sink are unit boxes in 2D.
  - They further constrain the feasible set to be in between the border of the
    unit box and the exterior of the unit circle.
  - In addition, the point in the source region must be in the upper left
    quadrant, the point in the target region must be in the upper right quadrant
    and the point in the sink region must be in the lower left quadrant.
    The quadrants are padded by 0.1 to exclude 0.0 from the feasible set.
  - We wish to minimize the squared L2 norm of the difference between the
    points. We expect that source to target will be the shortest path since the
    optimal difference is zero, while the distance between source and sink will
    be greater than one.

        Source                            Target
  ┌────────────┐                     ┌────────────┐
  │xxxxx___    │                     │    ___xxxxx│
  │xxx/    \   │        e_on         │   /    \xxx│
  │xx|      |  │  ────────────────►  │  |      |xx│
  │  |      |  │                     │  |      |  │
  │   \____/   │                     │   \____/   │
  │            │                     │            │
  └────────────┘                     └────────────┘

        │
        │  e_off
        │
        │
        │
        ▼
      Sink
  ┌────────────┐
  │    ____    │
  │   /    \   │
  │  |      |  │
  │  |      |xx│
  │   \____/xxx│
  │       xxxxx│
  └────────────┘

  We will approach this by formulating a convex surrogate of the problem and
  solving the non-convex problem in the rounding stage.
  */

  // Add minimum distance cost.
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm());
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm());

  /* Source: The relaxation of the problem simply requires the point to be
  strictly in the top left corner.

    Relaxation        Rounding
  ┌────────────┐   ┌────────────┐
  │xxx         │   │xxxxx___    │
  │xxx         │   │xxx/    \   │
  │            │   │xx|      |  │
  │            │   │  |      |  │
  │            │   │   \____/   │
  │            │   │            │
  └────────────┘   └────────────┘
  */
  std::vector<solvers::Binding<solvers::Constraint>> constraints_relaxation;
  std::vector<solvers::Binding<solvers::Constraint>> constraints_restriction;

  constraints_relaxation.push_back(source_->AddConstraint(
      source_->x()[0] <= -0.5, {Transcription::kRelaxation}));
  constraints_relaxation.push_back(source_->AddConstraint(
      source_->x()[1] >= 0.5, {Transcription::kRelaxation}));

  constraints_restriction.push_back(source_->AddConstraint(
      pow(source_->x()[0], 2) + pow(source_->x()[1], 2) >= 1.0,
      {Transcription::kRestriction}));
  constraints_restriction.push_back(source_->AddConstraint(
      source_->x()[0] <= -0.1, {Transcription::kRestriction}));
  constraints_restriction.push_back(source_->AddConstraint(
      source_->x()[1] >= 0.1, {Transcription::kRestriction}));

  // Verify the constraints in the relaxation only contain the relaxed
  // constraints, but not the resctriction constraints.
  for (const auto& constraint :
       source_->GetConstraints({Transcription::kRelaxation})) {
    EXPECT_TRUE(std::find(constraints_relaxation.begin(),
                          constraints_relaxation.end(),
                          constraint) != constraints_relaxation.end());
    EXPECT_FALSE(std::find(constraints_restriction.begin(),
                           constraints_restriction.end(),
                           constraint) != constraints_restriction.end());
  }

  // Verify the constraints in the restriction only contain the appropiate
  // constraints, but not the relaxed constraints.
  for (const auto& constraint :
       source_->GetConstraints({Transcription::kRestriction})) {
    EXPECT_FALSE(std::find(constraints_relaxation.begin(),
                           constraints_relaxation.end(),
                           constraint) != constraints_relaxation.end());
    EXPECT_TRUE(std::find(constraints_restriction.begin(),
                          constraints_restriction.end(),
                          constraint) != constraints_restriction.end());
  }

  /* Target: The relaxation of the problem simply requires the point to be
  strictly in the top right corner.

    Relaxation        Rounding
  ┌────────────┐   ┌────────────┐
  │         xxx│   │    ___xxxxx│
  │         xxx│   │   /    \xxx│
  │            │   │  |      |xx│
  │            │   │  |      |  │
  │            │   │   \____/   │
  │            │   │            │
  └────────────┘   └────────────┘
  */
  constraints_relaxation.push_back(target_->AddConstraint(
      target_->x()[0] >= 0.5, {Transcription::kRelaxation}));
  constraints_relaxation.push_back(target_->AddConstraint(
      target_->x()[1] >= 0.5, {Transcription::kRelaxation}));

  constraints_restriction.push_back(target_->AddConstraint(
      pow(target_->x()[0], 2) + pow(target_->x()[1], 2) >= 1.0,
      {Transcription::kRestriction}));
  constraints_restriction.push_back(target_->AddConstraint(
      target_->x()[0] >= 0.1, {Transcription::kRestriction}));
  constraints_restriction.push_back(target_->AddConstraint(
      target_->x()[1] >= 0.1, {Transcription::kRestriction}));

  // Verify the constraints in the relaxation only contain the relaxed
  // constraints, but not the restriction constraints.
  for (const auto& constraint :
       target_->GetConstraints({Transcription::kRelaxation})) {
    EXPECT_TRUE(std::find(constraints_relaxation.begin(),
                          constraints_relaxation.end(),
                          constraint) != constraints_relaxation.end());
    EXPECT_FALSE(std::find(constraints_restriction.begin(),
                           constraints_restriction.end(),
                           constraint) != constraints_restriction.end());
  }

  // Verify the constraints in the restriction only contain the appropiate
  // constraints, but not the relaxed constraints.
  for (const auto& constraint :
       target_->GetConstraints({Transcription::kRestriction})) {
    EXPECT_FALSE(std::find(constraints_relaxation.begin(),
                           constraints_relaxation.end(),
                           constraint) != constraints_relaxation.end());
    EXPECT_TRUE(std::find(constraints_restriction.begin(),
                          constraints_restriction.end(),
                          constraint) != constraints_restriction.end());
  }

  /* Sink: The relaxation of the problem simply requires the point to be
  strictly in the bottom right corner.

    Relaxation        Rounding
  ┌────────────┐   ┌────────────┐
  │            │   │    ____    │
  │            │   │   /    \   │
  │            │   │  |      |  │
  │            │   │  |      |xx│
  │         xxx│   │   \____/xxx│
  │         xxx│   │       xxxxx│
  └────────────┘   └────────────┘
  */

  constraints_relaxation.push_back(
      sink_->AddConstraint(sink_->x()[0] >= 0.5, {Transcription::kRelaxation}));
  constraints_relaxation.push_back(sink_->AddConstraint(
      sink_->x()[1] <= -0.5, {Transcription::kRelaxation}));

  constraints_restriction.push_back(
      sink_->AddConstraint(pow(sink_->x()[0], 2) + pow(sink_->x()[1], 2) >= 1.0,
                           {Transcription::kRestriction}));
  constraints_restriction.push_back(sink_->AddConstraint(
      sink_->x()[0] >= 0.1, {Transcription::kRestriction}));
  constraints_restriction.push_back(sink_->AddConstraint(
      sink_->x()[1] <= -0.1, {Transcription::kRestriction}));

  // Verify the constraints in the relaxation only contain the relaxed
  // constraints, but not the restriction constraints.
  for (const auto& constraint :
       sink_->GetConstraints({Transcription::kRelaxation})) {
    EXPECT_TRUE(std::find(constraints_relaxation.begin(),
                          constraints_relaxation.end(),
                          constraint) != constraints_relaxation.end());
    EXPECT_FALSE(std::find(constraints_restriction.begin(),
                           constraints_restriction.end(),
                           constraint) != constraints_restriction.end());
  }

  // Verify the constraints in the restriction only contain the appropiate
  // constraints, but not the relaxed constraints.
  for (const auto& constraint :
       sink_->GetConstraints({Transcription::kRestriction})) {
    EXPECT_FALSE(std::find(constraints_relaxation.begin(),
                           constraints_relaxation.end(),
                           constraint) != constraints_relaxation.end());
    EXPECT_TRUE(std::find(constraints_restriction.begin(),
                          constraints_restriction.end(),
                          constraint) != constraints_restriction.end());
  }

  // Since all constraints were added to the relaxation and restriction
  // transcription, we expect kMIP to be empty.
  EXPECT_EQ(source_->GetConstraints({Transcription::kMIP}).size(), 0);
  EXPECT_EQ(target_->GetConstraints({Transcription::kMIP}).size(), 0);
  EXPECT_EQ(sink_->GetConstraints({Transcription::kMIP}).size(), 0);

  solvers::IpoptSolver ipopt;
  options_.restriction_solver = &ipopt;
  options_.convex_relaxation = true;
  options_.max_rounded_paths = 2;
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  // Make sure it used the correct solver.
  EXPECT_EQ(result.get_solver_id(), solvers::IpoptSolver::id());

  const double kExpectedDistance = 0.2;
  const double distance =
      (source_->GetSolution(result) - target_->GetSolution(result)).norm();
  EXPECT_NEAR(distance, kExpectedDistance, 1e-6);

  // Verify that the solution includes source to target.
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
}

TEST_F(ThreeBoxes, LinearEqualityConstraint) {
  const Vector2d b{.5, .3};
  e_on_->AddConstraint(e_on_->xv() == b);
  e_off_->AddConstraint(e_off_->xv() == b);
  source_->AddConstraint(source_->x() == -b);
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(source_->GetSolution(result), -b, 1e-6));
  EXPECT_TRUE(CompareMatrices(target_->GetSolution(result), b, 1e-6));
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(Aeq_v * source_->GetSolution(result), beq, 1e-6));
  EXPECT_TRUE(
      CompareMatrices(Aeq.leftCols(2) * source_->GetSolution(result) +
                          Aeq.rightCols(2) * target_->GetSolution(result),
                      beq, 1e-6));
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
  CheckConvexRestriction(result);
}

// Test linear constraints, including infinite lower and upper bounds.
TEST_F(ThreeBoxes, LinearConstraint) {
  const Matrix2d A = Matrix2d::Identity();
  const Vector2d b{.5, .3};
  e_on_->AddConstraint(CreateBinding(
      std::make_shared<LinearConstraint>(A, b, Vector2d::Constant(kInf)),
      e_on_->xv()));  // b ≤ e_on_->xv() ≤ ∞.
  e_off_->AddConstraint(e_off_->xv() >= b);
  source_->AddConstraint(CreateBinding(
      std::make_shared<LinearConstraint>(A, Vector2d::Constant(-kInf), -b),
      source_->x()));  // -∞ ≤ source->() ≤ -b.
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(
      (source_->GetSolution(result).array() <= -b.array() + 1e-6).all());
  EXPECT_TRUE((target_->GetSolution(result).array() >= b.array() - 1e-6).all());
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
  CheckConvexRestriction(result);
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
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
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
  CheckConvexRestriction(result);
}

// Test constraints that have lower bounds and upper bounds that are a
// combination of finite and infinite.
TEST_F(ThreeBoxes, LinearConstraint3) {
  const Matrix2d A = Matrix2d::Identity();
  const Vector2d b{.5, -kInf};  // One finite and one infinite.
  e_on_->AddConstraint(CreateBinding(
      std::make_shared<LinearConstraint>(A, b, Vector2d::Constant(kInf)),
      e_on_->xv()));  // b ≤ e_on_->xv() ≤ ∞.
  e_off_->AddConstraint(CreateBinding(
      std::make_shared<LinearConstraint>(A, b, Vector2d::Constant(kInf)),
      e_off_->xv()));  // b ≤ e_off_->xv() ≤ ∞.
  source_->AddConstraint(CreateBinding(
      std::make_shared<LinearConstraint>(A, Vector2d::Constant(-kInf), -b),
      source_->x()));  // -∞ ≤ source->() ≤ -b.
  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(
      (source_->GetSolution(result).array() <= -b.array() + 1e-6).all());
  EXPECT_TRUE((target_->GetSolution(result).array() >= b.array() - 1e-6).all());
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
  CheckConvexRestriction(result);
}

TEST_F(ThreeBoxes, LorentzConeConstraint) {
  Eigen::MatrixXd A(5, 4);
  // clang-format off
  A << 1, 2, 3, 4,
       0, 1, 2, 3,
       3, 0, 1, 0,
       0, 3, 2, 1,
       0, 0, 0, 0;
  // clang-format on
  Eigen::VectorXd b(5);
  b << 1, 0, 2, 0, 1;

  auto constraint = std::make_shared<solvers::LorentzConeConstraint>(A, b);
  e_on_->AddConstraint(
      solvers::Binding(constraint, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddConstraint(
      solvers::Binding(constraint, {e_off_->xu(), e_off_->xv()}));

  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());

  Eigen::VectorXd res(4);
  res << source_->GetSolution(result), target_->GetSolution(result);
  auto z = A * res + b;
  EXPECT_GE(z[0], std::sqrt(std::pow(z[1], 2) + std::pow(z[2], 2) +
                            std::pow(z[3], 2) + std::pow(z[4], 2)));
  EXPECT_GE(z[0], 0);
}

TEST_F(ThreeBoxes, PositiveSemidefiniteConstraint1) {
  auto constraint =
      std::make_shared<solvers::PositiveSemidefiniteConstraint>(2);
  solvers::VectorXDecisionVariable psd_x_on(4);
  // [ xᵤ[0], xᵤ[1]; xᵤ[1], xᵥ[0]] ≽ 0.
  psd_x_on << e_on_->xu()[0], e_on_->xu()[1], e_on_->xu()[1], e_on_->xv()[0];
  e_on_->AddConstraint(solvers::Binding(constraint, psd_x_on));
  // Prevent the matrix from collapsing to zero.
  e_on_->AddConstraint(psd_x_on[0] + psd_x_on[3] == 1);

  // Make sure the PSD constraint is inactive for the off edge.
  solvers::VectorXDecisionVariable psd_x_off(4);
  psd_x_off << e_off_->xu()[0], e_off_->xu()[1], e_off_->xu()[1],
      e_off_->xv()[0];
  e_off_->AddConstraint(solvers::Binding(constraint, psd_x_off));
  e_off_->AddConstraint(psd_x_off[0] + psd_x_off[3] == 1);

  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());

  Eigen::Matrix2d mat;
  mat << source_->GetSolution(result)[0], source_->GetSolution(result)[1],
      source_->GetSolution(result)[1], target_->GetSolution(result)[0];
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(mat);
  EXPECT_GE(solver.eigenvalues()[0], 0);
  EXPECT_GE(solver.eigenvalues()[1], 0);
  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
}

// This test has linear constraints to prevent the matrix from being positive
// semidefinite. The solver should fail.
TEST_F(ThreeBoxes, PositiveSemidefiniteConstraint2) {
  auto constraint =
      std::make_shared<solvers::PositiveSemidefiniteConstraint>(2);
  solvers::VectorXDecisionVariable psd_x_on(4);
  // [ xᵤ[0], xᵤ[1]; xᵤ[1], xᵥ[0]] ≽ 0.
  psd_x_on << e_on_->xu()[0], e_on_->xu()[1], e_on_->xu()[1], e_on_->xv()[0];
  e_on_->AddConstraint(solvers::Binding(constraint, psd_x_on));
  e_on_->AddConstraint(psd_x_on[0] + psd_x_on[3] == -1);

  solvers::VectorXDecisionVariable psd_x_off(4);
  psd_x_off << e_off_->xu()[0], e_off_->xu()[1], e_off_->xu()[1],
      e_off_->xv()[0];
  e_off_->AddConstraint(solvers::Binding(constraint, psd_x_off));
  e_off_->AddConstraint(psd_x_off[0] + psd_x_off[3] == -1);

  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_FALSE(result.is_success());

  EXPECT_TRUE(sink_->GetSolution(result).hasNaN());
}

TEST_F(ThreeBoxes, RotatedLorentzConeConstraint) {
  Eigen::MatrixXd A(5, 4);
  // clang-format off
  A << 1, 2, 3, 4,
       0, 1, 2, 3,
       3, 0, 1, 0,
       0, 3, 2, 1,
       0, 0, 0, 0;
  // clang-format on
  Eigen::VectorXd b(5);
  b << 1, 0, 2, 0, 1;

  auto constraint =
      std::make_shared<solvers::RotatedLorentzConeConstraint>(A, b);
  e_on_->AddConstraint(
      solvers::Binding(constraint, {e_on_->xu(), e_on_->xv()}));
  e_off_->AddConstraint(
      solvers::Binding(constraint, {e_off_->xu(), e_off_->xv()}));

  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());

  Eigen::VectorXd res(4);
  res << source_->GetSolution(result), target_->GetSolution(result);
  auto z = A * res + b;
  EXPECT_GE(z[0] * z[1], std::sqrt(std::pow(z[2], 2) + std::pow(z[3], 2) +
                                   std::pow(z[4], 2)));
  EXPECT_GE(z[0], 0);
  EXPECT_GE(z[1], 0);
}

TEST_F(ThreeBoxes, SolveConvexRestriction) {
  const Vector2d b{.5, .3};

  // Vertex cost.
  source_->AddCost(
      static_cast<const VectorX<Expression>>(source_->x()).squaredNorm());
  // Vertex constraint.
  source_->AddConstraint(source_->x() <= -b);

  // Edge costs.
  e_on_->AddCost((e_on_->xu() - e_on_->xv()).squaredNorm());
  e_off_->AddCost((e_off_->xu() - e_off_->xv()).squaredNorm());

  // Edge constraints.
  e_on_->AddConstraint(e_on_->xv() >= b);
  e_off_->AddConstraint(e_off_->xv() >= b);

  auto result = g_.SolveShortestPath(*source_, *target_, options_);
  ASSERT_TRUE(result.is_success());

  auto restriction_result =
      g_.SolveConvexRestriction(std::vector<const Edge*>{e_on_}, options_);
  ASSERT_TRUE(restriction_result.is_success());

  EXPECT_NEAR(result.get_optimal_cost(), restriction_result.get_optimal_cost(),
              1e-6);
  for (const auto* v : g_.Vertices()) {
    EXPECT_TRUE(CompareMatrices(result.GetSolution(v->x()),
                                restriction_result.GetSolution(v->x()), 1e-6));
  }
}

// A simple shortest-path problem where the continuous variables do not affect
// the problem (they are all equality constrained).  The GraphOfConvexSets class
// should still solve the problem, and the convex relaxation should be optimal.
GTEST_TEST(ShortestPathTest, ClassicalShortestPath) {
  GraphOfConvexSets spp;

  std::vector<Vertex*> v(5);
  for (int i = 0; i < 5; ++i) {
    v[i] = spp.AddVertex(Point(Vector1d{0.0}));
  }

  Edge* v0_to_v1 = spp.AddEdge(v[0], v[1]);
  v0_to_v1->AddCost(3.0);
  spp.AddEdge(v[1], v[0])->AddCost(1.0);
  spp.AddEdge(v[0], v[2])->AddCost(4.0);
  spp.AddEdge(v[1], v[2])->AddCost(1.0);
  Edge* v0_to_v3 = spp.AddEdge(v[0], v[3]);
  v0_to_v3->AddCost(1.0);
  Edge* v3_to_v2 = spp.AddEdge(v[3], v[2]);
  v3_to_v2->AddCost(1.0);
  spp.AddEdge(v[1], v[4])
      ->AddCost(2.5);  // Updated from original to break symmetry.
  Edge* v2_to_v4 = spp.AddEdge(v[2], v[4]);
  v2_to_v4->AddCost(3.0);
  spp.AddEdge(v[0], v[4])->AddCost(6.0);

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;
  options.preprocessing = false;

  auto result = spp.SolveShortestPath(*v[0], *v[4], options);
  ASSERT_TRUE(result.is_success());

  EXPECT_EQ(spp.GetSolutionPath(*v[0], *v[4], result),
            std::vector<const Edge*>({v0_to_v3, v3_to_v2, v2_to_v4}));
  for (const auto* e : spp.Edges()) {
    double expected_cost = 0.0;
    // Only expect non-zero costs on the shortest path.
    if (e == v0_to_v3) {
      expected_cost = 1.0;
    } else if (e == v3_to_v2) {
      expected_cost = 1.0;
    } else if (e == v2_to_v4) {
      expected_cost = 3.0;
    }
    EXPECT_NEAR(e->GetSolutionCost(result), expected_cost, 1e-6);
  }

  // Now we artificially change the binaries in the result to cause
  // GetSolutionPath to backtrack.
  result.SetSolution(v0_to_v3->phi(), 0.8);
  result.SetSolution(v0_to_v1->phi(), 1.0);
  EXPECT_EQ(spp.GetSolutionPath(*v[0], *v[4], result, 0.2 /* tolerance */),
            std::vector<const Edge*>({v0_to_v3, v3_to_v2, v2_to_v4}));
}

GTEST_TEST(ShortestPathTest, InfeasibleProblem) {
  GraphOfConvexSets spp;

  Vertex* source = spp.AddVertex(Point(Vector1d(-1)));
  Vertex* v1 = spp.AddVertex(Point(Vector1d(-0.5)));
  Vertex* v2 = spp.AddVertex(Point(Vector1d(0.5)));
  Vertex* target = spp.AddVertex(Point(Vector1d(1)));

  spp.AddEdge(source, v1);
  spp.AddEdge(v2, target);

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;
  auto result = spp.SolveShortestPath(*source, *target, options);
  ASSERT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);

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

  DRAKE_EXPECT_THROWS_MESSAGE(spp.GetSolutionPath(*source, *target, result),
                              ".*is_success.*");
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

  spp.AddEdge(v[0], v[1])
      ->AddCost(solvers::Binding(cost, {v[0]->x(), v[1]->x()}));
  spp.AddEdge(v[1], v[0])
      ->AddCost(solvers::Binding(cost, {v[1]->x(), v[0]->x()}));
  spp.AddEdge(v[1], v[2])
      ->AddCost(solvers::Binding(cost, {v[1]->x(), v[2]->x()}));
  spp.AddEdge(v[2], v[1])
      ->AddCost(solvers::Binding(cost, {v[2]->x(), v[1]->x()}));
  spp.AddEdge(v[2], v[5])
      ->AddCost(solvers::Binding(cost, {v[2]->x(), v[5]->x()}));
  spp.AddEdge(v[5], v[2])
      ->AddCost(solvers::Binding(cost, {v[5]->x(), v[2]->x()}));

  spp.AddEdge(v[0], v[3])
      ->AddCost(solvers::Binding(cost, {v[0]->x(), v[3]->x()}));
  spp.AddEdge(v[3], v[0])
      ->AddCost(solvers::Binding(cost, {v[3]->x(), v[0]->x()}));
  spp.AddEdge(v[3], v[4])
      ->AddCost(solvers::Binding(cost, {v[3]->x(), v[4]->x()}));
  spp.AddEdge(v[4], v[3])
      ->AddCost(solvers::Binding(cost, {v[4]->x(), v[3]->x()}));
  spp.AddEdge(v[4], v[5])
      ->AddCost(solvers::Binding(cost, {v[4]->x(), v[5]->x()}));
  spp.AddEdge(v[5], v[4])
      ->AddCost(solvers::Binding(cost, {v[5]->x(), v[4]->x()}));

  GraphOfConvexSetsOptions options;
  options.preprocessing = false;
  options.convex_relaxation = true;

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

// Tests that all optimization variables are properly set, even when constrained
// to be on or off.
//            ┌──┐
//   ┌───────►│v2├──────┐
//   │        └──┘      │
//   │                  ▼
// ┌─┴┐                ┌──┐
// │v0│                │v3│
// └─┬┘                └──┘
//   │                  ▲
//   │        ┌──┐      │
//   └───────►│v1├──────┘
//            └──┘
GTEST_TEST(ShortestPathTest, PhiConstraint) {
  GraphOfConvexSets spp;

  spp.AddVertex(Point(Vector2d(0, 0)));
  spp.AddVertex(Point(Vector2d(1, -1)));
  spp.AddVertex(Point(Vector2d(1, 1)));
  spp.AddVertex(Point(Vector2d(2, 0)));

  auto v = spp.Vertices();

  spp.AddEdge(v[0], v[1]);
  spp.AddEdge(v[0], v[2]);
  Edge* edge_13 = spp.AddEdge(v[1], v[3]);
  spp.AddEdge(v[2], v[3]);

  // |xu - xv|₂²
  Matrix<double, 4, 4> A = Matrix<double, 4, 4>::Identity();
  A.block(0, 2, 2, 2) = -Matrix2d::Identity();
  A.block(2, 0, 2, 2) = -Matrix2d::Identity();
  auto cost =
      std::make_shared<solvers::QuadraticCost>(2.0 * A, Vector4d::Zero());
  for (auto e : spp.Edges()) {
    e->AddCost(solvers::Binding(cost, {e->xu(), e->xv()}));
  }

  GraphOfConvexSetsOptions options;
  options.preprocessing = false;
  options.convex_relaxation = true;

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
    // Gurobi's error is *slightly* larger than 1e-6. This puts in a healthy
    // margin to account for it.
    EXPECT_NEAR(edge_13->GetSolutionCost(result), 1.0, 1.1e-6);
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
    EXPECT_NEAR(edge_13->GetSolutionCost(result), 2.0, 1e-4);
    EXPECT_NEAR(result.GetSolution(edge_13->phi()), 1.0, 1e-6);
  }
}

// Confirms that preprocessing removes edges that cannot be on the shortest path
// and does not change the solution to a shortest path query.
class PreprocessShortestPathTest : public ::testing::Test {
 protected:
  PreprocessShortestPathTest() {
    v_.reserve(7);
    for (int i = 0; i < 7; ++i) {
      v_[i] = g_.AddVertex(Point(Vector1d{1.0}));
    }

    edges_.reserve(13);
    edges_.push_back(g_.AddEdge(v_[0], v_[2]));
    edges_.push_back(g_.AddEdge(v_[2], v_[3]));
    edges_.push_back(g_.AddEdge(v_[2], v_[4]));
    edges_.push_back(g_.AddEdge(v_[3], v_[4]));
    edges_.push_back(g_.AddEdge(v_[4], v_[3]));
    edges_.push_back(g_.AddEdge(v_[3], v_[5]));
    edges_.push_back(g_.AddEdge(v_[4], v_[5]));
    // Useless backtracking edges
    edges_.push_back(g_.AddEdge(v_[3], v_[2]));
    edges_.push_back(g_.AddEdge(v_[5], v_[4]));
    // Useless nodes off source and target
    edges_.push_back(g_.AddEdge(v_[0], v_[1]));
    edges_.push_back(g_.AddEdge(v_[1], v_[0]));
    edges_.push_back(g_.AddEdge(v_[5], v_[6]));
    edges_.push_back(g_.AddEdge(v_[6], v_[5]));

    for (Edge* e : edges_) {
      e->AddCost(1.0);
    }

    // Break symmetry of graph.
    edges_[2]->AddCost(0.1);

    options_.convex_relaxation = true;
  }

  std::set<EdgeId> PreprocessShortestPath(VertexId source_id,
                                          VertexId target_id) {
    return g_.PreprocessShortestPath(source_id, target_id, options_);
  }

  GraphOfConvexSets g_;
  std::vector<Vertex*> v_;
  std::vector<Edge*> edges_;
  GraphOfConvexSetsOptions options_;
};

TEST_F(PreprocessShortestPathTest, CheckEdges) {
  std::set<EdgeId> removed_edges =
      PreprocessShortestPath(v_[0]->id(), v_[5]->id());

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
  auto result1 = g_.SolveShortestPath(*v_[0], *v_[5], options_);
  ASSERT_TRUE(result1.is_success());

  options_.preprocessing = true;
  auto result2 = g_.SolveShortestPath(*v_[0], *v_[5], options_);
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

// PreprocessShortestPath() is supposed to consume the solver options. More
// particularly, those options must be passed to it from SolveShortestPath().
// We'll exploit friend access to confirm PreporocessShortestPath() makes use
// of the options. Confirming the options are *passed* is trickier. This is
// because both PreprocessShortestPath() and SolveShortestPath() invoke Solve
// with the same options. It is difficult to pass in a set of options such that
// we can discen the exercise in PreprocessShortestPath strictly from looking
// at the result. For example, passing an incompatible solver (as we do below)
// will throw an exception no matter what, even if SolveShortestPath() skips
// calling PreprocessShortestPath(). So, for now, we'll directly test that
// PreprocessShortestPath() depends on the options and leave the confirmation of
// SolveShortestPath() correctly passing those options as a future exercise.
TEST_F(PreprocessShortestPathTest, DependsOnOptions) {
  // Intentionally choose a solver that cannot run the preprocessing. Throwing
  // an exception is proof that the function relied on the options.
  solvers::LinearSystemSolver solver;
  options_.solver = &solver;
  DRAKE_EXPECT_THROWS_MESSAGE(PreprocessShortestPath(v_[0]->id(), v_[5]->id()),
                              ".*LinearSystemSolver is unable to solve.*");

  // TODO(SeanCurtis-TRI): Figure out a way to tell that SolveShortestPath
  // invokes PreporcessShortestPath with the given options as documented above.
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
  spp.AddEdge(source, p1);
  spp.AddEdge(source, p2);
  spp.AddEdge(p1, p3);
  spp.AddEdge(p2, p4);
  spp.AddEdge(p3, target);
  spp.AddEdge(p4, target);

  // Edges between parallel vertices
  spp.AddEdge(p1, p2);
  spp.AddEdge(p2, p1);
  spp.AddEdge(p3, p4);
  spp.AddEdge(p4, p3);

  // Edges pointing towards source
  spp.AddEdge(p3, p1);
  spp.AddEdge(p4, p2);

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
  auto relaxed_result = spp.SolveShortestPath(*source, *target, options);
  ASSERT_TRUE(relaxed_result.is_success());

  // We do not expect to find a path in the solution with zero tolerance.
  DRAKE_EXPECT_THROWS_MESSAGE(
      spp.GetSolutionPath(*source, *target, relaxed_result, 0 /* tolerance*/),
      ".*No path.*");
  // However, relaxing the tolerance (significantly) will find a path.
  EXPECT_NO_THROW(spp.GetSolutionPath(*source, *target, relaxed_result,
                                      0.9 /* tolerance*/));

  options.preprocessing = true;
  options.max_rounded_paths = 10;
  auto rounded_result = spp.SolveShortestPath(*source, *target, options);
  ASSERT_TRUE(rounded_result.is_success());

  EXPECT_LT(relaxed_result.get_optimal_cost(),
            rounded_result.get_optimal_cost());

  const auto& edges = spp.Edges();
  for (size_t ii = 0; ii < edges.size(); ++ii) {
    if (ii < 6) {
      // Some solvers do not balance the two paths as closely as other solvers.
      // I am not sure why these solvers perform badly on this problem.
      const double tol =
          (relaxed_result.get_solver_id() == solvers::GurobiSolver::id()) ? 1e-1
          : (relaxed_result.get_solver_id() == solvers::CsdpSolver::id()) ? 1e-2
          : (relaxed_result.get_solver_id() == solvers::ClarabelSolver::id())
              ? 1e-3  // We tried to tighten the optimality/feasibility
                      // tolerance of Clarabel but the optimal solution still
                      // match with the balanced solution very precisely.
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
  auto mip_result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_NEAR(rounded_result.get_optimal_cost(), mip_result.get_optimal_cost(),
              2e-6);

  if (solvers::MosekSolver::is_available() &&
      solvers::MosekSolver::is_enabled()) {
    // Test restriction_solver_options by setting the maximum iterations to 0,
    // which is equivalent to not solving the rounding problem. Thus it should
    // fail.
    solvers::MosekSolver mosek_solver;
    options.solver = &mosek_solver;
    options.convex_relaxation = true;
    options.preprocessing = false;
    options.max_rounded_paths = 10;

    options.restriction_solver_options = SolverOptions();
    options.restriction_solver_options->SetOption(
        solvers::MosekSolver::id(), "MSK_IPAR_INTPNT_MAX_ITERATIONS", 0);

    auto failed_result = spp.SolveShortestPath(*source, *target, options);
    EXPECT_FALSE(failed_result.is_success());

    // Without the convex relaxation, the solver should ignore the
    // restriction_solver_options and succeed.
    options.convex_relaxation = false;
    auto successful_result = spp.SolveShortestPath(*source, *target, options);
    EXPECT_TRUE(successful_result.is_success());
  }
}

/*
┌──────┐     ┌────┐     ┌────┐
|source├────►│ p1 │◄───►│ p3 │─────────┐
└───┬──┘     └─▲──┘     └─▲──┘         |
    │          |          |            |
    │        ┌─▼──┐     ┌─▼──┐     ┌───▼────┐
    └───────►│ p2 │◄───►│ p4 │────►│ target │
             └────┘     └────┘     └────────┘

*/
GTEST_TEST(ShortestPathTest, SamplePaths) {
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
  spp.AddEdge(source, p1);
  spp.AddEdge(source, p2);
  spp.AddEdge(p1, p3);
  spp.AddEdge(p2, p4);
  spp.AddEdge(p3, target);
  spp.AddEdge(p4, target);

  // Edges between parallel vertices
  spp.AddEdge(p1, p2);
  spp.AddEdge(p2, p1);
  spp.AddEdge(p3, p4);
  spp.AddEdge(p4, p3);

  // Edges pointing towards source
  spp.AddEdge(p3, p1);
  spp.AddEdge(p4, p2);

  const int kNumPaths = 4;

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;
  options.preprocessing = false;
  options.max_rounded_paths = 0;
  // We won't care about this result, but we solve to obtain a result we can
  // later modify.
  auto relaxed_result = spp.SolveShortestPath(*source, *target, options);
  ASSERT_TRUE(relaxed_result.is_success());

  // Set a high number of rounding trials so we find at least kNumPaths paths.
  options.max_rounded_paths = kNumPaths;
  options.max_rounding_trials = 100;

  // Set all the flow variables to 0.5 so we will sample multiple random paths.
  for (const auto& e : spp.Edges()) {
    relaxed_result.SetSolution(e->phi(), 0.5);
  }

  auto paths = spp.SamplePaths(*source, *target, relaxed_result, options);
  ASSERT_EQ(paths.size(), kNumPaths);

  // Check that all the paths start at the source and end at the target
  for (const auto& p : paths) {
    ASSERT_GE(p.size(), 0);
    ASSERT_EQ(p.front()->u().id(), source->id());
    ASSERT_EQ(p.back()->v().id(), target->id());
  }

  // Make sure no paths are equal
  for (std::size_t i = 0; i < paths.size(); ++i) {
    const auto& curr_path = paths[i];

    for (std::size_t other_idx = 0; other_idx < paths.size(); ++other_idx) {
      if (other_idx == i) continue;

      std::vector<EdgeId> curr_path_ids;
      for (const auto& edge : curr_path) curr_path_ids.push_back(edge->id());

      std::vector<EdgeId> other_path_ids;
      for (const auto& edge : paths[other_idx])
        other_path_ids.push_back(edge->id());

      // Sort the IDs
      std::sort(curr_path_ids.begin(), curr_path_ids.end());
      std::sort(other_path_ids.begin(), other_path_ids.end());
      bool paths_equal = curr_path_ids == other_path_ids;

      ASSERT_FALSE(paths_equal);
    }
  }
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

  source_edges.push_back(spp.AddEdge(source, v0));
  source_edges.push_back(spp.AddEdge(source, v1));

  region_edges.push_back(spp.AddEdge(v0, v1));
  region_edges.push_back(spp.AddEdge(v1, v0));
  region_edges.push_back(spp.AddEdge(v0, v2));
  region_edges.push_back(spp.AddEdge(v2, v0));
  region_edges.push_back(spp.AddEdge(v1, v3));
  region_edges.push_back(spp.AddEdge(v3, v1));
  region_edges.push_back(spp.AddEdge(v2, v3));
  region_edges.push_back(spp.AddEdge(v3, v2));

  target_edges.push_back(spp.AddEdge(v2, target));
  target_edges.push_back(spp.AddEdge(v3, target));

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
  auto result = spp.SolveShortestPath(*source, *target, options);
  ASSERT_TRUE(result.is_success());
}

// Cover the case where there is no path from source to target.
GTEST_TEST(ShortestPathTest, NoPath) {
  GraphOfConvexSets spp;
  auto source = spp.AddVertex(Point(Vector2d(0, 0)));
  auto v1 = spp.AddVertex(Point(Vector2d(0, 1)));
  auto v2 = spp.AddVertex(Point(Vector2d(1, 0)));
  auto target = spp.AddVertex(Point(Vector2d(1, 1)));
  spp.AddEdge(source, v1);
  spp.AddEdge(v2, target);

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;
  options.preprocessing = false;
  options.max_rounded_paths = 10;
  auto result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);

  options.preprocessing = true;
  options.max_rounded_paths = 10;
  result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);

  DRAKE_EXPECT_THROWS_MESSAGE(spp.GetSolutionPath(*source, *target, result),
                              ".*is_success.*");

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  options.convex_relaxation = false;
  result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
}

// Cover the special case of the source not having any outgoing edges.
GTEST_TEST(ShortestPathTest, NoPathDetachedSource) {
  GraphOfConvexSets spp;
  auto source = spp.AddVertex(Point(Vector2d(0, 0)));
  spp.AddVertex(Point(Vector2d(0, 1)));
  auto v2 = spp.AddVertex(Point(Vector2d(1, 0)));
  auto target = spp.AddVertex(Point(Vector2d(1, 1)));
  spp.AddEdge(v2, target);

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;
  options.preprocessing = false;
  options.max_rounded_paths = 10;
  auto result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);

  options.preprocessing = true;
  options.max_rounded_paths = 10;
  result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  options.convex_relaxation = false;
  result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
}

// Cover the special case of the target not having any incoming edges.
GTEST_TEST(ShortestPathTest, NoPathDetachedTarget) {
  GraphOfConvexSets spp;
  auto source = spp.AddVertex(Point(Vector2d(0, 0)));
  auto v1 = spp.AddVertex(Point(Vector2d(0, 1)));
  spp.AddVertex(Point(Vector2d(1, 0)));
  auto target = spp.AddVertex(Point(Vector2d(1, 1)));
  spp.AddEdge(source, v1);

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;
  options.preprocessing = false;
  options.max_rounded_paths = 10;
  auto result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);

  options.preprocessing = true;
  options.max_rounded_paths = 10;
  result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  options.convex_relaxation = false;
  result = spp.SolveShortestPath(*source, *target, options);
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
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

  spp.AddEdge(source, p1);
  Edge* source_to_p2 = spp.AddEdge(source, p2);
  Edge* source_to_p3 = spp.AddEdge(source, p3);
  spp.AddEdge(p1, e2);
  spp.AddEdge(p2, p3);
  spp.AddEdge(p2, e1);
  spp.AddEdge(p2, e2);
  spp.AddEdge(p3, p2);  // removing this changes the asymptotic behavior.
  Edge* p3_to_e1 = spp.AddEdge(p3, e1);
  spp.AddEdge(p3, p4);
  spp.AddEdge(e1, e2);
  Edge* e1_to_p4 = spp.AddEdge(e1, p4);
  spp.AddEdge(e1, p5);
  spp.AddEdge(e2, e1);
  spp.AddEdge(e2, p5);
  spp.AddEdge(e2, target);
  spp.AddEdge(p4, p3);
  spp.AddEdge(p4, e2);
  Edge* p4_to_p5 = spp.AddEdge(p4, p5);
  spp.AddEdge(p4, target);
  spp.AddEdge(p5, e1);
  Edge* p5_to_target = spp.AddEdge(p5, target);

  // |xu - xv|₂²
  for (const auto& e : spp.Edges()) {
    e->AddCost((e->xu() - e->xv()).squaredNorm());
  }

  if (!MixedIntegerSolverAvailable()) {
    return;
  }

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = false;
  options.preprocessing = false;
  auto result = spp.SolveShortestPath(*source, *target, options);
  ASSERT_TRUE(result.is_success());
  EXPECT_EQ(spp.GetSolutionPath(*source, *target, result),
            std::vector<const Edge*>(
                {source_to_p3, p3_to_e1, e1_to_p4, p4_to_p5, p5_to_target}));

  const std::forward_list<Vertex*> shortest_path{source, p3, e1,
                                                 p4,     p5, target};
  for (const auto& e : spp.Edges()) {
    auto iter = std::find(shortest_path.begin(), shortest_path.end(), &e->u());
    if (iter != shortest_path.end() && &e->v() == *(++iter)) {
      // Then it's on the shortest path; cost should be non-zero.
      EXPECT_GE(e->GetSolutionCost(result), 1.0);
    } else {
      EXPECT_NEAR(e->GetSolutionCost(result), 0.0, 1e-5);
    }
  }

  // Test that solving with the known shortest path returns the same results.
  std::vector<const Edge*> path = spp.GetSolutionPath(*source, *target, result);
  auto active_edges_result = spp.SolveConvexRestriction(path, options);
  ASSERT_TRUE(active_edges_result.is_success());
  // The optimal costs should match.
  EXPECT_NEAR(result.get_optimal_cost(), active_edges_result.get_optimal_cost(),
              3e-5);
  // The vertex solutions should match on the shortest path.
  EXPECT_TRUE(CompareMatrices(result.GetSolution(source->x()),
                              active_edges_result.GetSolution(source->x()),
                              2e-3));
  for (const auto* e : path) {
    EXPECT_TRUE(CompareMatrices(result.GetSolution(e->xv()),
                                active_edges_result.GetSolution(e->xv()),
                                2e-3));
  }

  // Test that forcing an edge not on the shortest path to be active yields a
  // higher cost.
  source_to_p2->AddPhiConstraint(true);
  {
    auto new_result = spp.SolveShortestPath(*source, *target, options);
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
  source_to_p3->AddPhiConstraint(false);
  {
    auto new_result = spp.SolveShortestPath(*source, *target, options);
    ASSERT_TRUE(new_result.is_success());

    const std::forward_list<Vertex*> new_shortest_path{source, p1, e2,
                                                       e1,     p5, target};
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

  Vertex* source = spp.AddVertex(Point(Vector2d::Zero()), "source");
  Vertex* v1 = spp.AddVertex(Point(Vector2d(0, 2)));
  Vertex* v2 = spp.AddVertex(Point(Vector2d(0, -2)));
  Vertex* v3 =
      spp.AddVertex(HPolyhedron::MakeBox(Vector2d(2, -2), Vector2d(4, 2)));
  Vertex* target = spp.AddVertex(Point(Vector2d(5, 0)), "target");

  Edge* e01 = spp.AddEdge(source, v1);
  Edge* e02 = spp.AddEdge(source, v2);
  Edge* e13 = spp.AddEdge(v1, v3);
  Edge* e23 = spp.AddEdge(v2, v3);
  Edge* e34 = spp.AddEdge(v3, target);

  // Edge length is distance for all edges.
  Matrix<double, 2, 4> A;
  A.leftCols(2) = Matrix2d::Identity();
  A.rightCols(2) = -Matrix2d::Identity();
  auto cost = std::make_shared<solvers::L2NormCost>(A, Vector2d::Zero());

  for (const auto& e : spp.Edges()) {
    e->AddCost(solvers::Binding(cost, {e->xu(), e->xv()}));
  }

  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;
  auto result = spp.SolveShortestPath(*source, *target, options);
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

  auto relaxed_result = spp.SolveShortestPath(*source, *target, options);
  options.max_rounded_paths = 1;
  auto rounded_result = spp.SolveShortestPath(*source, *target, options);

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

// A simple path planning example, where the environment is in the box (0,0)
// to (6,6), and there is an obstacle in the box (2,2) to (4,4).
GTEST_TEST(ShortestPathTest, SavvaBoxExample) {
  auto add_edge_and_constraints = [](GraphOfConvexSets* gcs, Vertex* v_left,
                                     Vertex* v_right) {
    Edge* edge =
        gcs->AddEdge(v_left, v_right, v_left->name() + "-" + v_right->name());
    // Second point of left vertex is equal to first point of right vertex.
    edge->AddConstraint(edge->xu().tail<2>() == edge->xv().head<2>());
    return edge;
  };

  auto add_quadratic_cost_between_consecutive_points = [](Vertex* v) {
    auto x = v->x();
    v->AddCost((x[2] - x[0]) * (x[2] - x[0]) + (x[3] - x[1]) * (x[3] - x[1]));
  };

  // Construct a GCS.
  GraphOfConvexSets gcs;

  // Define vertices.
  Vertex* start = gcs.AddVertex(Point(Vector2d{4, 5}), "start");
  Vertex* target = gcs.AddVertex(Point(Vector2d{3, 0}), "target");

  Vertex* v_left = gcs.AddVertex(
      HPolyhedron::MakeBox(Vector4d{0, 0, 0, 0}, Vector4d{2, 6, 2, 6}), "left");
  Vertex* v_above = gcs.AddVertex(
      HPolyhedron::MakeBox(Vector4d{0, 4, 0, 4}, Vector4d{6, 6, 6, 6}),
      "above");
  Vertex* v_right = gcs.AddVertex(
      HPolyhedron::MakeBox(Vector4d{4, 0, 4, 0}, Vector4d{6, 6, 6, 6}),
      "right");
  Vertex* v_below = gcs.AddVertex(
      HPolyhedron::MakeBox(Vector4d{0, 0, 0, 0}, Vector4d{6, 2, 6, 2}),
      "below");

  // Add costs and constraints.
  add_quadratic_cost_between_consecutive_points(v_left);
  add_quadratic_cost_between_consecutive_points(v_above);
  add_quadratic_cost_between_consecutive_points(v_right);
  add_quadratic_cost_between_consecutive_points(v_below);

  // From above you can go left or right,
  // from left or right you can go below.
  add_edge_and_constraints(&gcs, start, v_above);
  add_edge_and_constraints(&gcs, v_above, v_right);
  add_edge_and_constraints(&gcs, v_above, v_left);
  add_edge_and_constraints(&gcs, v_right, v_below);
  add_edge_and_constraints(&gcs, v_left, v_below);
  add_edge_and_constraints(&gcs, v_below, target);

  // Solve convex relaxation so that the flows are split.
  GraphOfConvexSetsOptions options;
  options.convex_relaxation = true;

  auto result = gcs.SolveShortestPath(*start, *target, options);
  ASSERT_TRUE(result.is_success());

  for (auto* e : gcs.Edges()) {
    if (e->name().find("start") == std::string::npos &&
        e->name().find("target") == std::string::npos) {
      // The flows are split, so recovering the solution via e->xu() does not
      // return the expected result (it looks like the equality constraint was
      // not enforced).
      EXPECT_FALSE(CompareMatrices(result.GetSolution(e->xu()).tail<2>(),
                                   result.GetSolution(e->xv()).head<2>(),
                                   1e-4));
      // But the underlying edge decision variables *are* equal.
      EXPECT_TRUE(CompareMatrices(e->GetSolutionPhiXu(result).tail<2>(),
                                  e->GetSolutionPhiXv(result).head<2>(), 1e-4));
    }
  }
}

GTEST_TEST(ShortestPathTest, Graphviz) {
  GraphOfConvexSets g;
  auto source = g.AddVertex(Point(Vector2d{1.0, 2.}), "source");
  auto target = g.AddVertex(Point(Vector1d{1e-5}), "target");
  g.AddEdge(source, target, "source_to_target")->AddCost(1.23);
  auto other = g.AddVertex(Point(Vector1d{4.0}), "other");
  g.AddEdge(source, other, "source_to_other")->AddCost(3.45);
  g.AddEdge(other, target, "other_to_target");  // No cost from other to target.

  GraphOfConvexSetsOptions options;
  options.preprocessing = true;
  options.convex_relaxation = true;

  // Note: Testing the entire string against a const string is too fragile,
  // since the VertexIds are Identifier<> and increment on a global counter.
  EXPECT_THAT(g.GetGraphvizString(),
              AllOf(HasSubstr("source"), HasSubstr("target"),
                    HasSubstr("source_to_target")));
  auto result = g.SolveShortestPath(*source, *target, options);
  EXPECT_THAT(g.GetGraphvizString(result),
              AllOf(HasSubstr("x ="), HasSubstr("cost ="), HasSubstr("ϕ ="),
                    HasSubstr("ϕ xᵤ ="), HasSubstr("ϕ xᵥ =")));

  // With a rounded result.
  options.max_rounded_paths = 1;
  result = g.SolveShortestPath(*source, *target, options);
  // Note: The cost here only comes from the cost=0 on other_to_target until
  // SolveConvexRestriction provides the rewritten costs.
  EXPECT_THAT(g.GetGraphvizString(result),
              AllOf(HasSubstr("x ="), HasSubstr("cost ="), HasSubstr("ϕ =")));

  // No slack variables.
  EXPECT_THAT(
      g.GetGraphvizString(result, false),
      AllOf(HasSubstr("x ="), HasSubstr("cost ="), Not(HasSubstr("ϕ =")),
            Not(HasSubstr("ϕ xᵤ =")), Not(HasSubstr("ϕ xᵥ ="))));
  // Precision and scientific.
  EXPECT_THAT(g.GetGraphvizString(result, false, 2, false),
              AllOf(HasSubstr("x = [1.00 2.00]"), HasSubstr("x = [0.00]")));
  EXPECT_THAT(g.GetGraphvizString(result, false, 2, true),
              AllOf(HasSubstr("x = [1 2]"), HasSubstr("x = [1e-05]")));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
