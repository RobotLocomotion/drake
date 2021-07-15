#include "drake/geometry/optimization/graph_of_convex_sets.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
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

GTEST_TEST(VertexTest, Basic) {
  Point p(Vector3d(1.0, 2.0, 3.0));
  Vertex v(p, "point");

  EXPECT_EQ(v.ambient_dimension(), 3);
  EXPECT_EQ(v.name(), "point");

  EXPECT_EQ(v.x().size(), 3);
  EXPECT_EQ(v.x()[0].get_name(), "point(0)");
  EXPECT_TRUE(v.set().PointInSet(p.x()));
}

GTEST_TEST(VertexTest, GetSolution) {
  Point p(Vector3d(1.0, 2.0, 3.0));
  Vertex v(p, "point");
  MathematicalProgramResult result;
  std::unordered_map<symbolic::Variable::Id, int> map;
  for (int i = 0; i < 3; ++i) {
    map.emplace(v.x()[i].get_id(), i);
  }
  result.set_decision_variable_index(map);
  result.set_x_val(p.x());
  EXPECT_TRUE(CompareMatrices(v.GetSolution(result), p.x()));
}

GTEST_TEST(GraphOfConvexSetsTest, AddVertexMakeCopy) {
  GraphOfConvexSets g;
  Point p(Vector3d(1.0, 2.0, 3.0));
  Vertex* v = g.AddVertex(p, "point");

  EXPECT_EQ(v->ambient_dimension(), 3);
  EXPECT_EQ(v->name(), "point");
  EXPECT_TRUE(v->set().PointInSet(p.x()));
}

GTEST_TEST(GraphOfConvexSetsTest, AddVertexTakeOwnership) {
  GraphOfConvexSets g;
  auto p = std::make_unique<Point>(Vector3d(1.0, 2.0, 3.0));
  Point* pointer = p.get();
  Vertex* v = g.AddVertex(std::move(p), "point");

  EXPECT_EQ(v->ambient_dimension(), 3);
  EXPECT_EQ(v->name(), "point");
  EXPECT_TRUE(v->set().PointInSet(pointer->x()));

  // Confirm that the owned set refers to the original memory.
  const Vector3d x2{4., 5., 6.};
  pointer->set_x(x2);
  EXPECT_TRUE(v->set().PointInSet(x2));
}

/*
┌───┐       ┌───┐
│ u ├───e──►│ v │
└───┘       └───┘
*/
class TwoPointsNoGraph : public ::testing::Test {
 protected:
  TwoPointsNoGraph()
      : pu{Vector2d(1., 2.)},
        pv{Vector3d(3., 4., 5.)},
        u(pu, "u"),
        v(pv, "v"),
        e(u, v, "e") {}

  Point pu;
  Point pv;
  Vertex u;
  Vertex v;
  Edge e;
};

TEST_F(TwoPointsNoGraph, Basic) {
  EXPECT_EQ(e.name(), "e");
  EXPECT_EQ(e.u().name(), u.name());
  EXPECT_EQ(e.v().name(), v.name());

  EXPECT_EQ(Variables(e.xu()), Variables(u.x()));
  EXPECT_EQ(Variables(e.xv()), Variables(v.x()));
}

// Confirms that I can add costs (both ways) and get the solution.
// The correctness of the added costs will be established by the solution tests.
TEST_F(TwoPointsNoGraph, AddCost) {
  auto [ell0, b0] = e.AddCost((e.xv() - e.xu()).squaredNorm());
  auto cost = std::make_shared<LinearCost>(Vector2d{1.2, 3.4}, 0.1);
  auto [ell1, b1] = e.AddCost(Binding(cost, e.xu()));

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

  EXPECT_NEAR(e.GetSolutionCost(result), ell.sum(), 1e-16);
}

// Confirms that I can add constraints (both ways).
// The correctness of the added constraints will be established by the solution
// tests.
TEST_F(TwoPointsNoGraph, AddConstraint) {
  auto b0 = e.AddConstraint(e.xv()[0] == e.xu()[0]);
  auto constraint = std::make_shared<LinearConstraint>(
      Matrix2d::Identity(), Vector2d::Zero(), Vector2d{1.2, 3.4});
  auto b1 = e.AddConstraint(Binding(constraint, e.xu()));

  // Confirm that they are down-castable.
  auto linear_equality =
      dynamic_cast<LinearEqualityConstraint*>(b0.evaluator().get());
  EXPECT_TRUE(linear_equality != nullptr);
  auto linear = dynamic_cast<LinearConstraint*>(b1.evaluator().get());
  EXPECT_TRUE(linear != nullptr);
}

GTEST_TEST(GraphOfConvexSetsTest, AddEdge) {
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

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
