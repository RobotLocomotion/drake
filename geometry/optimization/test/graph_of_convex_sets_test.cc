#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/vpolytope.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using solvers::Binding;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::LinearEqualityConstraint;
using solvers::MathematicalProgramResult;
using solvers::QuadraticCost;
using symbolic::Variables;
typedef GraphOfConvexSets::Vertex Vertex;
typedef GraphOfConvexSets::Edge Edge;

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
}

// Confirms that I can add costs (both ways) and get the solution.
// The correctness of the added costs will be established by the solution tests.
TEST_F(TwoPoints, AddCost) {
  auto [ell0, b0] = e_->AddCost((e_->xv().head<2>() - e_->xu()).squaredNorm());
  auto cost = std::make_shared<LinearCost>(Vector2d{1.2, 3.4}, 0.1);
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

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
