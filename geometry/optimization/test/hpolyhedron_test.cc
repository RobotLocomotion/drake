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
#include "drake/math/roll_pitch_yaw.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using internal::CheckAddPointInSetConstraints;
using internal::MakeSceneGraphWithShape;
using math::RigidTransformd;
using math::RotationMatrixd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;

GTEST_TEST(HPolyhedronTest, UnitBoxTest) {
  Matrix<double, 6, 3> A;
  A << Matrix3d::Identity(), -Matrix3d::Identity();
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

  // Test AddPointInSetConstraints.
  EXPECT_TRUE(CheckAddPointInSetConstraints(H, Vector3d(.8, .3, -.9)));
  EXPECT_TRUE(CheckAddPointInSetConstraints(H, Vector3d(-1.0, 1.0, 1.0)));
  EXPECT_FALSE(CheckAddPointInSetConstraints(H, Vector3d(1.1, 1.2, 0.4)));

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
  Vector3d in1_W{-4.9, -5.4, -7.4}, in2_W{-3.1, -4.6, -4.6},
      out1_W{-5.1, -5.6, -7.6}, out2_W{-2.9, -4.4, -4.4};

  EXPECT_LE(query.ComputeSignedDistanceToPoint(in1_W)[0].distance, 0.0);
  EXPECT_LE(query.ComputeSignedDistanceToPoint(in2_W)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out1_W)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out2_W)[0].distance, 0.0);

  EXPECT_TRUE(H.PointInSet(in1_W));
  EXPECT_TRUE(H.PointInSet(in2_W));
  EXPECT_FALSE(H.PointInSet(out1_W));
  EXPECT_FALSE(H.PointInSet(out2_W));

  EXPECT_TRUE(CheckAddPointInSetConstraints(H, in1_W));
  EXPECT_TRUE(CheckAddPointInSetConstraints(H, in2_W));
  EXPECT_FALSE(CheckAddPointInSetConstraints(H, out1_W));
  EXPECT_FALSE(CheckAddPointInSetConstraints(H, out2_W));

  // Test reference_frame frame.
  SourceId source_id = scene_graph->RegisterSource("F");
  FrameId frame_id = scene_graph->RegisterFrame(source_id, GeometryFrame("F"));
  auto context2 = scene_graph->CreateDefaultContext();
  const RigidTransformd X_WF{math::RollPitchYawd(.1, .2, 3),
                             Vector3d{.5, .87, .1}};
  const FramePoseVector<double> pose_vector{{frame_id, X_WF}};
  scene_graph->get_source_pose_port(source_id).FixValue(context2.get(),
                                                        pose_vector);
  auto query2 =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context2);
  HPolyhedron H_F(query2, geom_id, frame_id);

  const RigidTransformd X_FW = X_WF.inverse();
  EXPECT_TRUE(H_F.PointInSet(X_FW * in1_W));
  EXPECT_TRUE(H_F.PointInSet(X_FW * in2_W));
  EXPECT_FALSE(H_F.PointInSet(X_FW * out1_W));
  EXPECT_FALSE(H_F.PointInSet(X_FW * out2_W));
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
  Vector3d in1_W{-1.21, 0.0, 0.0}, in2_W{-1.21, 2., 3.}, out1_W{-1.19, 0, 0},
      out2_W{-1.19, 2., 3.};

  EXPECT_LE(query.ComputeSignedDistanceToPoint(in1_W)[0].distance, 0.0);
  EXPECT_LE(query.ComputeSignedDistanceToPoint(in2_W)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out1_W)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out2_W)[0].distance, 0.0);

  EXPECT_TRUE(H.PointInSet(in1_W));
  EXPECT_TRUE(H.PointInSet(in2_W));
  EXPECT_FALSE(H.PointInSet(out1_W));
  EXPECT_FALSE(H.PointInSet(out2_W));
}

GTEST_TEST(HPolyhedronTest, UnitBox6DTest) {
  HPolyhedron H = HPolyhedron::MakeUnitBox(6);
  EXPECT_EQ(H.ambient_dimension(), 6);

  Vector6d in1_W{Vector6d::Constant(-.99)}, in2_W{Vector6d::Constant(.99)},
      out1_W{Vector6d::Constant(-1.01)}, out2_W{Vector6d::Constant(1.01)};

  EXPECT_TRUE(H.PointInSet(in1_W));
  EXPECT_TRUE(H.PointInSet(in2_W));
  EXPECT_FALSE(H.PointInSet(out1_W));
  EXPECT_FALSE(H.PointInSet(out2_W));
}

GTEST_TEST(HPolyhedronTest, InscribedEllipsoidTest) {
  // Test a unit box.
  HPolyhedron H = HPolyhedron::MakeUnitBox(3);
  Hyperellipsoid E = H.MaximumVolumeInscribedEllipsoid();
  // The exact tolerance will be solver dependent; this is (hopefully)
  // conservative enough.
  const double kTol = 1e-4;
  EXPECT_TRUE(CompareMatrices(E.center(), Vector3d::Zero(), kTol));
  EXPECT_TRUE(CompareMatrices(E.A().transpose() * E.A(),
                              Matrix3d::Identity(3, 3), kTol));

  // A non-trivial example, taken some real problem data.  The addition of the
  // extra half-plane constraints cause the optimal ellipsoid to be far from
  // axis-aligned.
  Matrix<double, 8, 3> A;
  Matrix<double, 8, 1> b;
  // clang-format off
  A << Matrix3d::Identity(),
       -Matrix3d::Identity(),
       .9, -.3, .1,
       .9, -.3, .1;
  b << 2.1, 2.1, 2.1, 2.1, 2.1, 2.1, 1.3, 0.8;
  // clang-format on
  HPolyhedron H2(A, b);
  Hyperellipsoid E2 = H2.MaximumVolumeInscribedEllipsoid();
  // Check that points just inside the boundary of the ellipsoid are inside the
  // polytope.
  Matrix3d C = E2.A().inverse();
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
  const VectorXd polytope_halfspace_residue =
      b - A * E2.center() - ((A * C).rowwise().lpNorm<2>());
  EXPECT_NEAR(polytope_halfspace_residue.minCoeff(), 0, kTol);
}

GTEST_TEST(HPolyhedronTest, ChebyshevCenter) {
  HPolyhedron box = HPolyhedron::MakeUnitBox(6);
  EXPECT_TRUE(CompareMatrices(box.ChebyshevCenter(), Vector6d::Zero(), 1e-6));
}

// A rotated long thin rectangle in 2 dimensions.
GTEST_TEST(HPolyhedronTest, ChebyshevCenter2) {
  Matrix<double, 4, 2> A;
  Vector4d b;
  // clang-format off
  A << -2, -1,  // 2x + y ≥ 4
        2,  1,  // 2x + y ≤ 6
       -1,  2,  // x - 2y ≥ 2
        1, -2;  // x - 2y ≤ 8
  b << -4, 6, -2, 8;
  // clang-format on
  HPolyhedron H(A, b);
  const VectorXd center = H.ChebyshevCenter();
  EXPECT_TRUE(H.PointInSet(center));
  // For the rectangle, the center should have distance = 1.0 from the first
  // two half-planes, and ≥ 1.0 for the other two.
  const VectorXd distance = b - A*center;
  EXPECT_NEAR(distance[0], 1.0, 1e-6);
  EXPECT_NEAR(distance[1], 1.0, 1e-6);
  EXPECT_GE(distance[2], 1.0 - 1e-6);
  EXPECT_GE(distance[3], 1.0 - 1e-6);
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

GTEST_TEST(HPolyhedronTest, IsBounded) {
  Vector4d lb, ub;
  lb << -1, -3, -5, -2;
  ub << 2, 4, 5.4, 3;
  HPolyhedron H = HPolyhedron::MakeBox(lb, ub);
  EXPECT_TRUE(H.IsBounded());
}

GTEST_TEST(HPolyhedronTest, IsBounded2) {
  // Box with zero volume.
  const Vector2d lb{1, -3}, ub{1, 3};
  HPolyhedron H = HPolyhedron::MakeBox(lb, ub);
  EXPECT_TRUE(H.IsBounded());
}

GTEST_TEST(HPolyhedronTest, IsBounded3) {
  // Unbounded (2 inequalities in 3 dimensions).
  HPolyhedron H(MatrixXd::Identity(2, 3), Vector2d::Ones());
  EXPECT_FALSE(H.IsBounded());
}

GTEST_TEST(HPolyhedronTest, IsBounded4) {
  // Unbounded (A is low rank).
  Matrix3d A;
  // clang-format off
  A << 1, 2, 3,
       1, 2, 3,
       0, 0, 1;
  // clang-format on
  HPolyhedron H(A, Vector3d::Ones());
  EXPECT_FALSE(H.IsBounded());
}

GTEST_TEST(HPolyhedronTest, CartesianPowerTest) {
  // First test the concept. If x ∈ H, then [x; x]  ∈ H x H and
  // [x; x; x]  ∈ H x H x H.
  MatrixXd A{4, 2};
  A << MatrixXd::Identity(2, 2), -MatrixXd::Identity(2, 2);
  VectorXd b = VectorXd::Ones(4);
  HPolyhedron H(A, b);
  VectorXd x = VectorXd::Zero(2);
  EXPECT_TRUE(H.PointInSet(x));
  EXPECT_TRUE(H.CartesianPower(2).PointInSet((VectorXd(4) << x, x).finished()));
  EXPECT_TRUE(
      H.CartesianPower(3).PointInSet((VectorXd(6) << x, x, x).finished()));

  // Now test the HPolyhedron-specific behavior.
  MatrixXd A_1{2, 3};
  MatrixXd A_2{4, 6};
  MatrixXd A_3{6, 9};
  VectorXd b_1{2};
  VectorXd b_2{4};
  VectorXd b_3{6};
  MatrixXd zero = MatrixXd::Zero(2, 3);
  // clang-format off
  A_1 << 1, 2, 3,
         4, 5, 6;
  b_1 << 1, 2;
  A_2 <<  A_1, zero,
         zero,  A_1;
  b_2 << b_1, b_1;
  A_3 <<  A_1, zero, zero,
         zero,  A_1, zero,
         zero, zero,  A_1;
  b_3 << b_1, b_1, b_1;
  // clang-format on
  HPolyhedron H_1(A_1, b_1);
  HPolyhedron H_2 = H_1.CartesianPower(2);
  HPolyhedron H_3 = H_1.CartesianPower(3);
  EXPECT_TRUE(CompareMatrices(H_2.A(), A_2));
  EXPECT_TRUE(CompareMatrices(H_2.b(), b_2));
  EXPECT_TRUE(CompareMatrices(H_3.A(), A_3));
  EXPECT_TRUE(CompareMatrices(H_3.b(), b_3));
}

GTEST_TEST(HPolyhedronTest, CartesianProductTest) {
  HPolyhedron H_A = HPolyhedron::MakeUnitBox(2);
  VectorXd x_A = VectorXd::Zero(2);
  EXPECT_TRUE(H_A.PointInSet(x_A));

  HPolyhedron H_B = HPolyhedron::MakeBox(Vector2d(2, 2), Vector2d(4, 4));
  VectorXd x_B = 3 * VectorXd::Ones(2);
  EXPECT_TRUE(H_B.PointInSet(x_B));

  HPolyhedron H_C = H_A.CartesianProduct(H_B);
  VectorXd x_C{x_A.size() + x_B.size()};
  x_C << x_A, x_B;
  EXPECT_TRUE(H_C.PointInSet(x_C));
}


}  // namespace optimization
}  // namespace geometry
}  // namespace drake
