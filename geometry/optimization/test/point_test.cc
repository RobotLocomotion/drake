#include "drake/geometry/optimization/point.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
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

  // Test MaybeGetPoint.
  ASSERT_TRUE(P.MaybeGetPoint().has_value());
  EXPECT_TRUE(CompareMatrices(P.MaybeGetPoint().value(), p_W));

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

  // Test IsEmpty (which is trivially false for Point).
  EXPECT_FALSE(P.IsEmpty());

  // Test MaybeGetFeasiblePoint.
  ASSERT_TRUE(P.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(P.PointInSet(P.MaybeGetFeasiblePoint().value()));

  // Test set_x().
  const Vector3d p2_W{6.2, -.23, -8.2};
  P.set_x(p2_W);
  EXPECT_TRUE(CompareMatrices(p2_W, P.x()));
  EXPECT_TRUE(CompareMatrices(P.MaybeGetPoint().value(), p2_W));
}

GTEST_TEST(PointTest, DefaultCtor) {
  const Point dut;
  EXPECT_EQ(dut.x().size(), 0);
  EXPECT_NO_THROW(dut.Clone());
  EXPECT_EQ(dut.ambient_dimension(), 0);
  EXPECT_TRUE(dut.IntersectsWith(dut));
  EXPECT_TRUE(dut.IsBounded());
  EXPECT_TRUE(dut.MaybeGetPoint().has_value());
  EXPECT_TRUE(dut.PointInSet(Eigen::VectorXd::Zero(0)));
  EXPECT_FALSE(dut.IsEmpty());
  ASSERT_TRUE(dut.MaybeGetFeasiblePoint().has_value());
  EXPECT_TRUE(dut.PointInSet(dut.MaybeGetFeasiblePoint().value()));

  Point P;
  EXPECT_NO_THROW(P.set_x(Eigen::VectorXd::Zero(0)));
}

GTEST_TEST(PointTest, Move) {
  const Vector3d p_W{1.2, 4.5, -2.8};
  Point orig(p_W);

  // A move-constructed Point takes over the original data.
  Point dut(std::move(orig));
  EXPECT_EQ(dut.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(dut.x(), p_W));

  // The old Point is in a valid but unspecified state.
  EXPECT_EQ(orig.x().size(), orig.ambient_dimension());
  EXPECT_NO_THROW(orig.Clone());
}

GTEST_TEST(PointTest, FromSceneGraphTest) {
  Vector3d p_W{1.2, 4.5, -2.8};

  // Test SceneGraph constructor.
  const double kRadius = 0.2;
  auto [scene_graph, geom_id, context, query] =
      MakeSceneGraphWithShape(Sphere(kRadius), RigidTransformd(p_W));

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

GTEST_TEST(PointTest, FromSceneGraphBad) {
  auto [scene_graph, geom_id, context, query] =
      MakeSceneGraphWithShape(HalfSpace(), RigidTransformd());
  DRAKE_EXPECT_THROWS_MESSAGE(Point(query, geom_id),
                              ".*Point.*cannot.*HalfSpace.*");
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
  Eigen::Vector4d p{1.0, 2.0, 3.0, 4.0};
  Point P(p);

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];

  std::vector<Binding<Constraint>> constraints =
      P.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  EXPECT_EQ(constraints.size(), 2);
  const double tol = 1e-16;

  for (const double scale : {0.5, 1.0, 2.0}) {
    prog.SetInitialGuess(x, scale * p);
    prog.SetInitialGuess(t, scale);
    EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
    prog.SetInitialGuess(x, 0.99 * scale * p);
    EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
  }

  prog.SetInitialGuess(x, p);
  prog.SetInitialGuess(t, -1.0);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
}

GTEST_TEST(PointTest, NonnegativeScalingTest2) {
  Eigen::Vector4d p{1.0, 2.0, 3.0, 4.0};
  Point P(p);

  MathematicalProgram prog;
  Eigen::MatrixXd A(4, 3);
  // clang-format off
  A << 1, 0, 0,
       0, 1, 0,
       0, 0, 3,
       0, 2, 0;
  // clang-format on
  Eigen::Vector4d b = Eigen::Vector4d::Zero();
  auto x = prog.NewContinuousVariables(3, "x");
  Eigen::Vector2d c(1, -1);
  double d = 0;
  auto t = prog.NewContinuousVariables(2, "t");

  std::vector<Binding<Constraint>> constraints =
      P.AddPointInNonnegativeScalingConstraints(&prog, A, b, c, d, x, t);

  EXPECT_EQ(constraints.size(), 2);

  Eigen::Vector3d x_solution(1, 2, 1);
  Eigen::Vector3d scale(0.5, 1.0, 2.0);
  Eigen::MatrixXd t_solutions(2, 9);
  // clang-format off
  t_solutions << 0.5,  0,  0.25, 1,  0,  0.5, 2,  0,  1,
                 0, -0.5, -0.25, 0, -1, -0.5, 0, -2, -1;
  // clang-format on
  const double tol = 1e-16;
  for (int ii = 0; ii < t_solutions.cols(); ii++) {
    prog.SetInitialGuess(x, scale[ii / 3] * x_solution);
    prog.SetInitialGuess(t, t_solutions.col(ii));
    EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
    prog.SetInitialGuess(x, 0.99 * scale[ii / 3] * x_solution);
    EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
  }

  prog.SetInitialGuess(x, x_solution);
  prog.SetInitialGuess(t, Eigen::Vector2d(0, 1));
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));

  prog.SetInitialGuess(x, x_solution);
  prog.SetInitialGuess(t, Eigen::Vector2d(-1, 0));
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
}

GTEST_TEST(PointTest, Projection) {
  Eigen::Vector4d p{1.0, 2.0, 3.0, 4.0};
  Point P(p);

  // clang-format off
  const Eigen::Matrix4d test_points{
      { 0.12,  0.41, -2.81, 2.17},
      { 3.06, -0.75, -1.16, 0.55},
      { 0.54,  4.26,  0.85, 0.02},
      { 2.99,  0.47,  3.99, 3.29}
  };
  const Eigen::Matrix4d expected_projection{
      { 1.0, 1.0, 1.0, 1.0},
      { 2.0, 2.0, 2.0, 2.0},
      { 3.0, 3.0, 3.0, 3.0},
      { 4.0, 4.0, 4.0, 4.0}
  };
  // clang-format on
  std::vector<double> expected_distances;
  for (int i = 0; i < test_points.cols(); ++i) {
    double expected_distance = 0.0;
    for (int j = 0; j < p.rows(); ++j) {
      expected_distance += std::pow((p[j] - test_points(j, i)), 2);
    }
    expected_distances.push_back(std::sqrt(expected_distance));
  }
  const auto projection_result = P.Projection(test_points);
  ASSERT_TRUE(projection_result.has_value());
  const auto& [distances, projections] = projection_result.value();
  const double kTol = 1e-12;
  for (int i = 0; i < test_points.cols(); ++i) {
    EXPECT_TRUE(
        CompareMatrices(projections.col(i), expected_projection.col(i), kTol));
    EXPECT_NEAR(distances.at(i), expected_distances.at(i), kTol);
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
