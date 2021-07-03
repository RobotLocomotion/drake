#include "drake/geometry/optimization/vpolytope.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Vector3d;
using internal::CheckAddPointInSetConstraints;
using internal::MakeSceneGraphWithShape;
using math::RigidTransformd;
using math::RotationMatrixd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;

GTEST_TEST(VPolytopeTest, TriangleTest) {
  Eigen::Matrix<double, 2, 3> triangle;
  // clang-format off
  triangle <<  2, 4, 3,
              -1, 2, 5;
  // clang-format on

  VPolytope V(triangle);
  EXPECT_EQ(V.ambient_dimension(), 2);
  EXPECT_TRUE(CompareMatrices(V.vertices(), triangle));

  // Check IsBounded (which is trivially true for V Polytopes).
  EXPECT_TRUE(V.IsBounded());

  const Eigen::Vector2d center = triangle.rowwise().mean();
  // Mosek worked with 1e-15.
  // Gurobi worked with 1e-15 (see the note about alpha_sol in the method).
  // CLP needed 1e-11.
  const double kTol = 1e-11;
  EXPECT_TRUE(V.PointInSet(center, kTol));
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector2d v = triangle.col(i);
    const Eigen::Vector2d at_tol =
        v + kTol * (v - center) / ((v - center).lpNorm<Eigen::Infinity>());
    EXPECT_TRUE(CompareMatrices(v, at_tol, 2.0 * kTol));
    EXPECT_FALSE(CompareMatrices(v, at_tol, .5 * kTol));

    EXPECT_TRUE(V.PointInSet(v, kTol));
    EXPECT_TRUE(V.PointInSet(at_tol, 2.0 * kTol));
    EXPECT_FALSE(V.PointInSet(at_tol, 0.5 * kTol));
  }
}

GTEST_TEST(VPolytopeTest, UnitBoxTest) {
  VPolytope V = VPolytope::MakeUnitBox(3);
  EXPECT_EQ(V.ambient_dimension(), 3);

  const Vector3d in1_W{.8, .3, -.9}, in2_W{-1.0, 1.0, 1.0},
      out_W{1.1, 1.2, 0.4};

  // Test PointInSet.
  // Mosek worked with 1e-14; CLP needed 1e-11.
  const double kTol = 1e-11;
  EXPECT_TRUE(V.PointInSet(in1_W, kTol));
  EXPECT_TRUE(V.PointInSet(in2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out_W, kTol));

  // Test AddPointInSetConstraints.
  EXPECT_TRUE(CheckAddPointInSetConstraints(V, in1_W));
  EXPECT_TRUE(CheckAddPointInSetConstraints(V, in2_W));
  EXPECT_FALSE(CheckAddPointInSetConstraints(V, out_W));

  // Test SceneGraph constructor.
  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Box(2.0, 2.0, 2.0), RigidTransformd::Identity());
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  VPolytope V_scene_graph(query, geom_id);
  EXPECT_EQ(V.ambient_dimension(), 3);
  EXPECT_TRUE(V.PointInSet(in1_W, kTol));
  EXPECT_TRUE(V.PointInSet(in2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out_W, kTol));
}

GTEST_TEST(VPolytopeTest, ArbitraryBoxTest) {
  const RigidTransformd X_WG(math::RollPitchYawd(.1, .2, 3),
                             Vector3d(-4.0, -5.0, -6.0));
  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Box(1.0, 2.0, 3.0), X_WG);
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);
  VPolytope V(query, geom_id);

  EXPECT_EQ(V.ambient_dimension(), 3);
  const Vector3d in1_G{.48, .98, -1.49}, in2_G{-0.49, .99, 1.49},
      out_G{0.51, .9, 0.4};
  const Vector3d in1_W = X_WG * in1_G, in2_W = X_WG * in2_G,
                 out_W = X_WG * out_G;

  EXPECT_LE(query.ComputeSignedDistanceToPoint(in1_W)[0].distance, 0.0);
  EXPECT_LE(query.ComputeSignedDistanceToPoint(in2_W)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out_W)[0].distance, 0.0);

  const double kTol = 1e-14;
  EXPECT_TRUE(V.PointInSet(in1_W, kTol));
  EXPECT_TRUE(V.PointInSet(in2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out_W, kTol));

  EXPECT_TRUE(CheckAddPointInSetConstraints(V, in1_W));
  EXPECT_TRUE(CheckAddPointInSetConstraints(V, in2_W));
  EXPECT_FALSE(CheckAddPointInSetConstraints(V, out_W));

  // Test reference_frame frame.
  SourceId source_id = scene_graph->RegisterSource("F");
  FrameId frame_id = scene_graph->RegisterFrame(source_id, GeometryFrame("F"));
  auto context2 = scene_graph->CreateDefaultContext();
  const RigidTransformd X_WF{math::RollPitchYawd(5.1, 3.2, -3),
                             Vector3d{.5, .87, .1}};
  const FramePoseVector<double> pose_vector{{frame_id, X_WF}};
  scene_graph->get_source_pose_port(source_id).FixValue(context2.get(),
                                                        pose_vector);
  auto query2 =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context2);
  VPolytope V_F(query2, geom_id, frame_id);

  const RigidTransformd X_FW = X_WF.inverse();
  EXPECT_TRUE(V_F.PointInSet(X_FW * in1_W, kTol));
  EXPECT_TRUE(V_F.PointInSet(X_FW * in2_W, kTol));
  EXPECT_FALSE(V_F.PointInSet(X_FW * out_W, kTol));
}

GTEST_TEST(VPolytopeTest, UnitBox6DTest) {
  VPolytope V = VPolytope::MakeUnitBox(6);
  EXPECT_EQ(V.ambient_dimension(), 6);

  Vector6d in1_W{Vector6d::Constant(-.99)}, in2_W{Vector6d::Constant(.99)},
      out1_W{Vector6d::Constant(-1.01)}, out2_W{Vector6d::Constant(1.01)};

  const double kTol = 1e-11;
  EXPECT_TRUE(V.PointInSet(in1_W, kTol));
  EXPECT_TRUE(V.PointInSet(in2_W, kTol));
  EXPECT_FALSE(V.PointInSet(out1_W, kTol));
  EXPECT_FALSE(V.PointInSet(out2_W, kTol));
}

GTEST_TEST(VPolytopeTest, CloneTest) {
  VPolytope V = VPolytope::MakeBox(Vector3d{-3, -4, -5}, Vector3d{6, 7, 8});
  std::unique_ptr<ConvexSet> clone = V.Clone();
  EXPECT_EQ(clone->ambient_dimension(), V.ambient_dimension());
  VPolytope* pointer = dynamic_cast<VPolytope*>(clone.get());
  ASSERT_NE(pointer, nullptr);
  EXPECT_TRUE(CompareMatrices(V.vertices(), pointer->vertices()));
}

// Returns true iff a scaled version of the upper corner is confirmed to be
// inside the scaled box.
bool PointInScaledSet(double x_scale, double t_scale) {
  const Vector3d lb{1, 1, 1}, ub{2, 3, 4};
  VPolytope V = VPolytope::MakeBox(lb, ub);

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];

  std::vector<Binding<Constraint>> constraints =
      V.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  EXPECT_EQ(constraints.size(), 4);

  prog.AddBoundingBoxConstraint(x_scale * ub, x_scale * ub, x);
  prog.AddBoundingBoxConstraint(t_scale, t_scale, t);
  auto result = solvers::Solve(prog);
  return result.is_success();
}

GTEST_TEST(VPolytopeTest, NonnegativeScalingTest) {
  // Note: I have to call Solve on these (not just check the constraints)
  // because of the slack variables.
  EXPECT_TRUE(PointInScaledSet(.99, 1.0));
  EXPECT_FALSE(PointInScaledSet(1.01, 1.0));
  EXPECT_FALSE(PointInScaledSet(.99, -0.01));
  EXPECT_TRUE(PointInScaledSet(.49, .5));
  EXPECT_FALSE(PointInScaledSet(.51, .5));
  EXPECT_TRUE(PointInScaledSet(1.99, 2.0));
  EXPECT_FALSE(PointInScaledSet(2.01, 2.0));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
