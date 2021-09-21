#include "drake/geometry/optimization/cartesian_product.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;

GTEST_TEST(CartesianProductTest, BasicTest) {
  const Point P1(Vector2d{1.2, 3.4}), P2(Vector2d{5.6, 7.8});
  const CartesianProduct S(P1, P2);
  EXPECT_EQ(S.num_factors(), 2);
  EXPECT_EQ(S.ambient_dimension(), 4);

  // Test PointInSet.
  Vector4d in, out;
  in << P1.x(), P2.x();
  out << P1.x(), P2.x() + Vector2d::Constant(0.01);
  EXPECT_TRUE(S.PointInSet(in));
  EXPECT_FALSE(S.PointInSet(out));

  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(S, in));
  EXPECT_FALSE(internal::CheckAddPointInSetConstraints(S, out));

  // Test IsBounded.
  EXPECT_TRUE(S.IsBounded());

  // Test ConvexSets constructor.
  ConvexSets sets;
  sets.emplace_back(P1);
  sets.emplace_back(P2);
  const CartesianProduct S2(sets);
  EXPECT_EQ(S2.num_factors(), 2);
  EXPECT_EQ(S2.ambient_dimension(), 4);
  EXPECT_TRUE(S2.PointInSet(in));
  EXPECT_FALSE(S2.PointInSet(out));
}

GTEST_TEST(CartesianProductTest, FromSceneGraph) {
  const RigidTransformd X_WG{math::RollPitchYawd(.1, .2, 3),
                             Vector3d{.5, .87, .1}};

  // Test SceneGraph constructor.
  const double kRadius = 0.2;
  const double kLength = 0.5;
  auto [scene_graph, geom_id] =
      internal::MakeSceneGraphWithShape(Cylinder(kRadius, kLength), X_WG);
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  CartesianProduct S(query, geom_id, std::nullopt);
  Matrix<double, 3, 4> in_G, out_G;
  // clang-format off
  in_G << 0, 0,           kRadius,      kRadius/std::sqrt(2.0),
          0, 0,           0,            kRadius/std::sqrt(2.0),
          0, kLength/2.0, -kLength/2.0, kLength/2.0;
  out_G << kRadius+.01, 0,     kRadius,          kRadius/std::sqrt(2.0),
           0, 0,               0,                kRadius/std::sqrt(2.0),
           0, kLength/2.0+.01, -kLength/2.0-.01, kLength/2.0+.01;
  // clang-format on
  const Matrix<double, 3, 4> in_W = X_WG * in_G, out_W = X_WG * out_G;
  const double kTol = 1e-14;
  for (int i = 0; i < 4; ++i) {
    EXPECT_LE(query.ComputeSignedDistanceToPoint(in_W.col(i))[0].distance,
              kTol);
    EXPECT_GE(query.ComputeSignedDistanceToPoint(out_W.col(i))[0].distance,
              -kTol);

    EXPECT_TRUE(S.PointInSet(in_W.col(i), kTol));
    EXPECT_FALSE(S.PointInSet(out_W.col(i), kTol));
  }

  // Test reference_frame frame.
  SourceId source_id = scene_graph->RegisterSource("F");
  FrameId frame_id = scene_graph->RegisterFrame(source_id, GeometryFrame("F"));
  auto context2 = scene_graph->CreateDefaultContext();
  const RigidTransformd X_WF{math::RollPitchYawd(.5, .26, -3),
                             Vector3d{.9, -2., .12}};
  const FramePoseVector<double> pose_vector{{frame_id, X_WF}};
  scene_graph->get_source_pose_port(source_id).FixValue(context2.get(),
                                                        pose_vector);
  auto query2 =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context2);
  CartesianProduct S2(query2, geom_id, frame_id);

  const RigidTransformd X_FW = X_WF.inverse();
  const Matrix<double, 3, 4> in_F = X_FW * in_W, out_F = X_FW * out_W;
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(S2.PointInSet(in_F.col(i), kTol));
    EXPECT_FALSE(S2.PointInSet(out_F.col(i), kTol));
  }
}

GTEST_TEST(CartesianProductTest, TwoBoxes) {
  HPolyhedron H1 = HPolyhedron::MakeBox(Vector2d{1, 1}, Vector2d{2, 2});
  HPolyhedron H2 = HPolyhedron::MakeBox(Vector2d{-2, 2}, Vector2d{0, 4});
  CartesianProduct S(H1, H2);
  EXPECT_TRUE(S.PointInSet(Vector4d{1.9, 1.9, -.1, 3.9}));
  EXPECT_FALSE(S.PointInSet(Vector4d{1.9, 1.9, -.1, 4.1}));
  EXPECT_FALSE(S.PointInSet(Vector4d{2.1, 1.9, -.1, 3.9}));
}

GTEST_TEST(CartesianProductTest, CloneTest) {
  const Point P1(Vector2d{1.2, 3.4}), P2(Vector2d{5.6, 7.8});
  const CartesianProduct S(P1, P2);
  std::unique_ptr<ConvexSet> clone = S.Clone();
  EXPECT_EQ(clone->ambient_dimension(), 4);

  CartesianProduct* s = dynamic_cast<CartesianProduct*>(clone.get());
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(s->num_factors(), 2);
  const Point* p = dynamic_cast<const Point*>(&s->factor(0));
  ASSERT_NE(p, nullptr);
  EXPECT_TRUE(CompareMatrices(P1.x(), p->x()));
}

GTEST_TEST(CartesianProductTest, NonnegativeScalingTest) {
  const Point P1(Vector2d{1.2, 3.4}), P2(Vector2d{5.6, 7.8});
  const CartesianProduct S(P1, P2);

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];

  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      S.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  // 1 vector constraint from P1, 1 from P2, and t>=0 3 times.
  EXPECT_EQ(constraints.size(), 5);

  Vector4d p;
  p << P1.x(), P2.x();
  for (const double scale : {0.5, 1.0, 2.0}) {
    prog.SetInitialGuess(x, scale * p);
    prog.SetInitialGuess(t, scale);
    EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, 1e-16));
    prog.SetInitialGuess(x, 0.99 * scale * p);
    EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, 1e-16));
  }
  prog.SetInitialGuess(x, p);
  prog.SetInitialGuess(t, -1.0);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, 1e-16));
}

// Test the case where A and b are set via the constructor.  Note that the
// SceneGraph test also operates on sets that have A and b set.  In particular,
// this test exercises the case of a non-square A.
GTEST_TEST(CartesianProductTest, Rotated) {
  // Denote the ambient space of S with _S.
  const Vector2d in_S{1.23, 4.5};
  const Vector2d out_S = in_S + Vector2d{0.0, 0.01};

  Matrix<double, 4, 2> A;
  A << 1.2, 4, 2, -1.2, 4.3, -2, 7.1, 1.4;
  Vector4d b{.64, 9.2, 7.2, 1.};
  // Denote the "rotated" space, where we take product of points P, with _P.
  const Vector4d in_P = A * in_S + b;
  const Point P1(in_P.head<2>()), P2(in_P.tail<2>());

  // The set S is equivalent to a Point(in_S).
  ConvexSets sets;
  sets.emplace_back(P1);
  sets.emplace_back(P2);
  const CartesianProduct S(sets, A, b);

  EXPECT_EQ(S.ambient_dimension(), 2);
  EXPECT_EQ(S.num_factors(), 2);

  // The set S is equivalent to Point(in_S).
  const double kTol = 1e-14;
  EXPECT_TRUE(S.PointInSet(in_S, kTol));
  EXPECT_FALSE(S.PointInSet(out_S, kTol));

  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(S, in_S));
  EXPECT_FALSE(internal::CheckAddPointInSetConstraints(S, out_S));

  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];
  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      S.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  // The rotation terms cause an additional y=Ax+b constraint to be added inside
  // the method, so I must solve the problem now with constraints to set x and t
  // now, instead of just checking the declared constraints.

  // Test that 0.5*in_S is in the set scaled by 0.5.
  prog.AddBoundingBoxConstraint(.5, .5, t);
  solvers::Binding<solvers::BoundingBoxConstraint> x_constraint =
      prog.AddBoundingBoxConstraint(.5 * in_S, .5 * in_S, x);
  EXPECT_TRUE(Solve(prog).is_success());

  // Test that 0.5*out_S is *not* in the set scaled by 0.5.
  prog.RemoveConstraint(x_constraint);
  prog.AddBoundingBoxConstraint(.5 * out_S, .5 * out_S, x);
  EXPECT_FALSE(Solve(prog).is_success());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
