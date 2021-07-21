#include "drake/geometry/optimization/point.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Vector2d;
using Eigen::Vector3d;
using internal::CheckAddPointInSetConstraints;
using internal::MakeSceneGraphWithShape;
using math::RigidTransformd;
using math::RotationMatrixd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;

GTEST_TEST(PointTest, BasicTest) {
  // Test constructor.
  Vector3d p_W{1.2, 4.5, -2.8};
  Point P(p_W);
  EXPECT_EQ(P.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(p_W, P.x()));

  // Test PointInSet.
  EXPECT_TRUE(P.PointInSet(p_W));
  EXPECT_FALSE(P.PointInSet(p_W + Vector3d::Constant(0.01)));
  EXPECT_TRUE(P.PointInSet(p_W + Vector3d::Constant(0.01), 0.01 + 1e-16));

  EXPECT_TRUE(CheckAddPointInSetConstraints(P, p_W));
  EXPECT_FALSE(
      CheckAddPointInSetConstraints(P, p_W + Vector3d::Constant(0.01)));

  // Test ToShapeWithPose.
  auto [shape, X_WG] = P.ToShapeWithPose();
  Sphere* sphere = dynamic_cast<Sphere*>(shape.get());
  EXPECT_TRUE(sphere != NULL);
  EXPECT_NEAR(sphere->radius(), 0.0, 0.0);
  EXPECT_TRUE(CompareMatrices(X_WG.translation(), p_W, 1e-16));

  // Test IsBounded (which is trivially true for Point).
  EXPECT_TRUE(P.IsBounded());

  // Test set_x().
  const Vector3d p2_W{6.2, -.23, -8.2};
  P.set_x(p2_W);
  EXPECT_TRUE(CompareMatrices(p2_W, P.x()));
}

GTEST_TEST(PointTest, FromSceneGraphTest) {
  Vector3d p_W{1.2, 4.5, -2.8};

  // Test SceneGraph constructor.
  const double kRadius = 0.2;
  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Sphere(kRadius), RigidTransformd(p_W));
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  EXPECT_NEAR(query.ComputeSignedDistanceToPoint(p_W)[0].distance, -kRadius,
              1e-16);

  const double kAllowableRadius = 0.5;
  Point P(query, geom_id, std::nullopt, kAllowableRadius);
  EXPECT_TRUE(CompareMatrices(p_W, P.x()));

  // Check maximum_allowable_radius argument.
  EXPECT_THROW(Point(query, geom_id, std::nullopt, kRadius / 2.0),
               std::exception);

  // Test reference_frame frame.
  SourceId source_id = scene_graph->RegisterSource("F");
  FrameId frame_id = scene_graph->RegisterFrame(source_id, GeometryFrame("F"));
  auto context2 = scene_graph->CreateDefaultContext();
  RigidTransformd X_WF(Vector3d{5., -2, 3.1});
  const FramePoseVector<double> pose_vector{{frame_id, X_WF}};
  scene_graph->get_source_pose_port(source_id).FixValue(context2.get(),
                                                        pose_vector);
  auto query2 =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context2);
  Point P2(query2, geom_id, frame_id, kAllowableRadius);

  EXPECT_TRUE(CompareMatrices(P2.x(), X_WF.inverse() * p_W, 1e-16));
}

GTEST_TEST(PointTest, 6DTest) {
  Eigen::VectorXd x = Eigen::VectorXd::Ones(6);
  Point P(x);
  EXPECT_EQ(P.ambient_dimension(), 6);
  EXPECT_EQ(P.x(), x);
}

GTEST_TEST(PointTest, CloneTest) {
  Point P(Vector6d::Zero());
  std::unique_ptr<ConvexSet> clone = P.Clone();
  EXPECT_EQ(clone->ambient_dimension(), P.ambient_dimension());
  Point* pointer = dynamic_cast<Point*>(clone.get());
  ASSERT_NE(pointer, nullptr);
  EXPECT_TRUE(CompareMatrices(P.x(), pointer->x()));
}

GTEST_TEST(PointTest, NonnegativeScalingTest) {
  Eigen::Vector4d p{1., 2., 3., 4.};
  Point P(p);

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];

  std::vector<Binding<Constraint>> constraints =
      P.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  EXPECT_EQ(constraints.size(), 2);

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

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
