#include "drake/geometry/optimization/hpolyhedron.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Vector3d;
using internal::CheckAddPointInSetConstraint;
using internal::MakeSceneGraphWithShape;
using math::RigidTransformd;
using math::RotationMatrixd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;

GTEST_TEST(HPolyhedronTest, UnitBoxTest) {
  Eigen::Matrix<double, 6, 3> A;
  A << Eigen::Matrix3d::Identity(), -Eigen::Matrix3d::Identity();
  Vector6d b = Vector6d::Ones();

  // Test constructor.
  HPolyhedron H(A, b);
  EXPECT_EQ(H.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(A, H.A()));
  EXPECT_TRUE(CompareMatrices(b, H.b()));

  // Test MakeUnitBox method.
  HPolyhedron Hbox = HPolyhedron::MakeUnitBox(3);
  EXPECT_EQ(Hbox.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(A, Hbox.A()));
  EXPECT_TRUE(CompareMatrices(b, Hbox.b()));

  // Test PointInSet.
  EXPECT_TRUE(H.PointInSet(Vector3d(.8, .3, -.9)));
  EXPECT_TRUE(H.PointInSet(Vector3d(-1.0, 1.0, 1.0)));
  EXPECT_FALSE(H.PointInSet(Vector3d(1.1, 1.2, 0.4)));

  // Test AddPointInSetConstraint.
  EXPECT_TRUE(CheckAddPointInSetConstraint(H, Vector3d(.8, .3, -.9)));
  EXPECT_TRUE(CheckAddPointInSetConstraint(H, Vector3d(-1.0, 1.0, 1.0)));
  EXPECT_FALSE(CheckAddPointInSetConstraint(H, Vector3d(1.1, 1.2, 0.4)));

  // Test SceneGraph constructor.
  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Box(2.0, 2.0, 2.0), RigidTransformd::Identity());
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  HPolyhedron H_scene_graph(query, geom_id);
  EXPECT_TRUE(CompareMatrices(A, H_scene_graph.A()));
  EXPECT_TRUE(CompareMatrices(b, H_scene_graph.b()));
}

GTEST_TEST(HPolyhedronTest, ArbitraryBoxTest) {
  RigidTransformd X_WG(RotationMatrixd::MakeZRotation(M_PI / 2.0),
                       Vector3d(-4.0, -5.0, -6.0));
  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Box(1.0, 2.0, 3.0), X_WG);
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);
  HPolyhedron H(query, geom_id);

  EXPECT_EQ(H.ambient_dimension(), 3);
  // Rotated box should end up with lb=[-5,-5.5,-7.5], ub=[-3,-4.5,-4.5].
  Vector3d in1{-4.9, -5.4, -7.4}, in2{-3.1, -4.6, -4.6}, out1{-5.1, -5.6, -7.6},
      out2{-2.9, -4.4, -4.4};

  EXPECT_LE(query.ComputeSignedDistanceToPoint(in1)[0].distance, 0.0);
  EXPECT_LE(query.ComputeSignedDistanceToPoint(in2)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out1)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out2)[0].distance, 0.0);

  EXPECT_TRUE(H.PointInSet(in1));
  EXPECT_TRUE(H.PointInSet(in2));
  EXPECT_FALSE(H.PointInSet(out1));
  EXPECT_FALSE(H.PointInSet(out2));

  EXPECT_TRUE(CheckAddPointInSetConstraint(H, in1));
  EXPECT_TRUE(CheckAddPointInSetConstraint(H, in2));
  EXPECT_FALSE(CheckAddPointInSetConstraint(H, out1));
  EXPECT_FALSE(CheckAddPointInSetConstraint(H, out2));

  // Test expressed_in frame.
  SourceId source_id = scene_graph->RegisterSource("new_frame");
  FrameId frame_id =
      scene_graph->RegisterFrame(source_id, GeometryFrame("new_frame"));
  auto context2 = scene_graph->CreateDefaultContext();
  // Set X_WF to X_WG to simplify PointInSet checks.
  const FramePoseVector<double> pose_vector{{frame_id, X_WG}};
  scene_graph->get_source_pose_port(source_id).FixValue(context2.get(),
                                                        pose_vector);
  auto query2 =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context2);
  HPolyhedron H_P(query2, geom_id, frame_id);

  EXPECT_TRUE(H_P.PointInSet(Vector3d::Zero()));
  EXPECT_FALSE(H_P.PointInSet(in1));
  EXPECT_FALSE(H_P.PointInSet(in2));
}

GTEST_TEST(HPolyhedronTest, HalfSpaceTest) {
  RigidTransformd X_WG(RotationMatrixd::MakeYRotation(M_PI / 2.0),
                       Vector3d(-1.2, -2.1, -6.4));
  auto [scene_graph, geom_id] = MakeSceneGraphWithShape(HalfSpace(), X_WG);
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);
  HPolyhedron H(query, geom_id);

  EXPECT_EQ(H.ambient_dimension(), 3);

  // Rotated HalfSpace should be x <= -1.2.
  Vector3d in1{-1.21, 0.0, 0.0}, in2{-1.21, 2., 3.}, out1{-1.19, 0, 0},
      out2{-1.19, 2., 3.};

  EXPECT_LE(query.ComputeSignedDistanceToPoint(in1)[0].distance, 0.0);
  EXPECT_LE(query.ComputeSignedDistanceToPoint(in2)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out1)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out2)[0].distance, 0.0);

  EXPECT_TRUE(H.PointInSet(in1));
  EXPECT_TRUE(H.PointInSet(in2));
  EXPECT_FALSE(H.PointInSet(out1));
  EXPECT_FALSE(H.PointInSet(out2));
}

GTEST_TEST(HPolyhedronTest, UnitBox6DTest) {
  HPolyhedron H = HPolyhedron::MakeUnitBox(6);
  EXPECT_EQ(H.ambient_dimension(), 6);

  Vector6d in1{Vector6d::Constant(-.99)}, in2{Vector6d::Constant(.99)},
      out1{Vector6d::Constant(-1.01)}, out2{Vector6d::Constant(1.01)};

  EXPECT_TRUE(H.PointInSet(in1));
  EXPECT_TRUE(H.PointInSet(in2));
  EXPECT_FALSE(H.PointInSet(out1));
  EXPECT_FALSE(H.PointInSet(out2));
}

GTEST_TEST(HPolyhedronTest, InscribedEllipsoidTest) {
  // Test a unit box.
  HPolyhedron H = HPolyhedron::MakeUnitBox(3);
  HyperEllipsoid E = H.MaximumVolumeInscribedEllipsoid();
  // The exact tolerance will be solver dependent; this is (hopefully)
  // conservative enough.
  const double kTol = 1e-6;
  EXPECT_TRUE(CompareMatrices(E.center(), Vector3d::Zero(), kTol));
  EXPECT_TRUE(CompareMatrices(E.A().transpose() * E.A(),
                              Eigen::Matrix3d::Identity(3, 3), kTol));

  // A non-trivial example, taken some real problem data.  The addition of the
  // extra half-plane constraints cause the optimal ellipsoid to be far from
  // axis-aligned.
  Eigen::Matrix<double, 8, 3> A;
  Eigen::Matrix<double, 8, 1> b;
  // clang-format off
  A << Eigen::Matrix3d::Identity(),
       -Eigen::Matrix3d::Identity(),
       .9, -.3, .1,
       .9, -.3, .1;
  b << 2.1, 2.1, 2.1, 2.1, 2.1, 2.1, 1.3, 0.8;
  // clang-format on
  HPolyhedron H2(A, b);
  HyperEllipsoid E2 = H2.MaximumVolumeInscribedEllipsoid();
  // Check that points just inside the boundary of the ellipsoid are inside the
  // polytope.
  Eigen::Matrix3d C = E2.A().inverse();
  RandomGenerator generator;
  for (int i = 0; i < 10; ++i) {
    const RotationMatrixd R = math::UniformlyRandomRotationMatrix(&generator);
    SCOPED_TRACE(fmt::format("With random rotation matrix\n{}", R.matrix()));
    Vector3d x = C * R.matrix() * Vector3d(0.99, 0.0, 0.0) + E2.center();
    EXPECT_TRUE(E2.PointInSet(x));
    EXPECT_TRUE(H2.PointInSet(x));
  }

  // Make sure the ellipsoid touches the polytope, by checking that the minimum
  // residual, bᵢ − aᵢd − |aᵢC|₂, is zero.
  const Eigen::VectorXd polytope_halfspace_residue =
      b - A * E2.center() - ((A * C).rowwise().lpNorm<2>());
  EXPECT_NEAR(polytope_halfspace_residue.minCoeff(), 0, kTol);
}

GTEST_TEST(HPolyhedronTest, CloneTest) {
  HPolyhedron H = HPolyhedron::MakeBox(Vector3d{-3, -4, -5}, Vector3d{6, 7, 8});
  std::unique_ptr<ConvexSet> clone = H.Clone();
  EXPECT_EQ(clone->ambient_dimension(), H.ambient_dimension());
  HPolyhedron* pointer = dynamic_cast<HPolyhedron*>(clone.get());
  ASSERT_NE(pointer, nullptr);
  EXPECT_TRUE(CompareMatrices(H.A(), pointer->A()));
  EXPECT_TRUE(CompareMatrices(H.b(), pointer->b()));
}

GTEST_TEST(HPolyhedronTest, NonnegativeScalingTest) {
  const Vector3d lb{1, 1, 1}, ub{2, 3, 4};
  HPolyhedron H = HPolyhedron::MakeBox(lb, ub);

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto t = prog.NewContinuousVariables(1, "t")[0];

  std::vector<Binding<Constraint>> constraints =
      H.AddPointInNonnegativeScalingConstraints(&prog, x, t);

  EXPECT_EQ(constraints.size(), 2);

  prog.SetInitialGuess(x, .99 * ub);
  prog.SetInitialGuess(t, 1.0);
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, 0));

  prog.SetInitialGuess(x, 1.01 * ub);
  prog.SetInitialGuess(t, 1.0);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, 0));

  prog.SetInitialGuess(x, .99 * ub);
  prog.SetInitialGuess(t, -0.01);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, 0));

  prog.SetInitialGuess(x, .49 * ub);
  prog.SetInitialGuess(t, 0.5);
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, 0));

  prog.SetInitialGuess(x, .51 * ub);
  prog.SetInitialGuess(t, 0.5);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, 0));

  prog.SetInitialGuess(x, 1.99 * ub);
  prog.SetInitialGuess(t, 2.0);
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, 0));

  prog.SetInitialGuess(x, 2.01 * ub);
  prog.SetInitialGuess(t, 2.0);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, 0));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
