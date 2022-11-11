#include "drake/geometry/optimization/hpolyhedron.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/perception/point_cloud.h"
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

GTEST_TEST(HPolyhedronTest, DefaultConstructor) {
  HPolyhedron H;
  EXPECT_EQ(H.ambient_dimension(), 0);
  EXPECT_EQ(H.A().size(), 0);
  EXPECT_EQ(H.b().size(), 0);
}

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

GTEST_TEST(HPolyhedronTest, L1BallTest) {
  Matrix<double, 8, 3> A;
  VectorXd b = VectorXd::Ones(8);
  // clang-format off
  A <<   1,  1,  1,
        -1,  1,  1,
         1, -1,  1,
        -1, -1,  1,
         1,  1, -1,
        -1,  1, -1,
         1, -1, -1,
        -1, -1, -1;
  // clang-format on

  // Test MakeL1Ball method.
  HPolyhedron H_L1_box = HPolyhedron::MakeL1Ball(3);
  EXPECT_EQ(H_L1_box.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(A, H_L1_box.A()));
  EXPECT_TRUE(CompareMatrices(b, H_L1_box.b()));
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
  const VectorXd distance = b - A * center;
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

  const double tol = 0;

  prog.SetInitialGuess(x, 0.99 * ub);
  prog.SetInitialGuess(t, 1.0);
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));

  prog.SetInitialGuess(x, 1.01 * ub);
  prog.SetInitialGuess(t, 1.0);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));

  prog.SetInitialGuess(x, 0.99 * ub);
  prog.SetInitialGuess(t, -0.01);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));

  prog.SetInitialGuess(x, 0.49 * ub);
  prog.SetInitialGuess(t, 0.5);
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));

  prog.SetInitialGuess(x, 0.51 * ub);
  prog.SetInitialGuess(t, 0.5);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));

  prog.SetInitialGuess(x, 1.99 * ub);
  prog.SetInitialGuess(t, 2.0);
  EXPECT_TRUE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));

  prog.SetInitialGuess(x, 2.01 * ub);
  prog.SetInitialGuess(t, 2.0);
  EXPECT_FALSE(prog.CheckSatisfiedAtInitialGuess(constraints, tol));
}

bool PointInScaledSet(const solvers::VectorXDecisionVariable& x_vars,
                      const solvers::VectorXDecisionVariable& t_vars,
                      const Vector2d& x, const Vector2d& t,
                      solvers::MathematicalProgram* prog,
                      const std::vector<Binding<Constraint>>& constraints) {
  const double tol = 0;
  prog->SetInitialGuess(x_vars, x);
  prog->SetInitialGuess(t_vars, t);
  return prog->CheckSatisfiedAtInitialGuess(constraints, tol);
}

GTEST_TEST(HPolyhedronTest, NonnegativeScalingTest2) {
  const Vector3d lb{1, 1, 1}, ub{2, 3, 4};
  HPolyhedron H = HPolyhedron::MakeBox(lb, ub);

  MathematicalProgram prog;
  MatrixXd A(3, 2);
  // clang-format off
  A << 1, 0,
       0, 1,
       2, 0;
  // clang-format on
  Vector3d b = Vector3d::Zero();
  auto x = prog.NewContinuousVariables(2, "x");
  Vector2d c(1, -1);
  double d = 0;
  auto t = prog.NewContinuousVariables(2, "t");

  std::vector<Binding<Constraint>> constraints =
      H.AddPointInNonnegativeScalingConstraints(&prog, A, b, c, d, x, t);

  EXPECT_EQ(constraints.size(), 2);

  EXPECT_TRUE(PointInScaledSet(x, t, 0.99 * ub.head(2), Vector2d(1.0, 0), &prog,
                               constraints));
  EXPECT_TRUE(PointInScaledSet(x, t, 0.99 * ub.head(2), Vector2d(0, -1.0),
                               &prog, constraints));
  EXPECT_FALSE(PointInScaledSet(x, t, 1.01 * ub.head(2), Vector2d(1.0, 0),
                                &prog, constraints));
  EXPECT_FALSE(PointInScaledSet(x, t, 1.01 * ub.head(2), Vector2d(0, -1.0),
                                &prog, constraints));
  EXPECT_FALSE(PointInScaledSet(x, t, 0.99 * ub.head(2), Vector2d(-0.01, 0),
                                &prog, constraints));
  EXPECT_FALSE(PointInScaledSet(x, t, 0.99 * ub.head(2), Vector2d(0, -0.01),
                                &prog, constraints));
  EXPECT_TRUE(PointInScaledSet(x, t, 0.49 * ub.head(2), Vector2d(0.5, 0), &prog,
                               constraints));
  EXPECT_TRUE(PointInScaledSet(x, t, 0.49 * ub.head(2), Vector2d(0, -0.5),
                               &prog, constraints));
  EXPECT_FALSE(PointInScaledSet(x, t, 0.51 * ub.head(2), Vector2d(0.5, 0),
                                &prog, constraints));
  EXPECT_FALSE(PointInScaledSet(x, t, 0.51 * ub.head(2), Vector2d(0, -0.5),
                                &prog, constraints));
  EXPECT_TRUE(PointInScaledSet(x, t, 1.99 * ub.head(2), Vector2d(2.0, 0), &prog,
                               constraints));
  EXPECT_TRUE(PointInScaledSet(x, t, 1.99 * ub.head(2), Vector2d(0, -2.0),
                               &prog, constraints));
  EXPECT_FALSE(PointInScaledSet(x, t, 2.01 * ub.head(2), Vector2d(2.0, 0),
                                &prog, constraints));
  EXPECT_FALSE(PointInScaledSet(x, t, 2.01 * ub.head(2), Vector2d(0, -2.0),
                                &prog, constraints));
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

GTEST_TEST(HPolyhedronTest, IsBoundedEmptyPolyhedron) {
  Eigen::MatrixXd A_infeasible{3, 3};
  // clang-format off
  A_infeasible << 1, -1, 0,
                  -1, 0, 1,
                  0, 1, -1;
  // clang-format on
  HPolyhedron H(A_infeasible, -Vector3d::Ones());
  EXPECT_TRUE(H.IsEmpty());
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

GTEST_TEST(HPolyhedronTest, AxisAlignedContainment) {
  Vector2d lower_limit = -Vector2d::Ones();
  Vector2d upper_limit = Vector2d::Ones();
  double scale{0.25};

  HPolyhedron outer = HPolyhedron::MakeBox(lower_limit, upper_limit);
  HPolyhedron inner =
      HPolyhedron::MakeBox(scale * lower_limit, scale * upper_limit);

  EXPECT_TRUE(inner.ContainedIn(outer));
  EXPECT_FALSE(outer.ContainedIn(inner));
}

GTEST_TEST(HPolyhedronTest, L1BallContainsInfinityBall3D) {
  HPolyhedron L1_ball = HPolyhedron::MakeL1Ball(3);
  HPolyhedron Linfty_ball = HPolyhedron::MakeUnitBox(3);

  EXPECT_TRUE(L1_ball.ContainedIn(Linfty_ball));
  EXPECT_FALSE(Linfty_ball.ContainedIn(L1_ball));
}

GTEST_TEST(HPolyhedronTest, L1BallIrredundantIntersectionInfinityBall3D) {
  HPolyhedron L1_ball = HPolyhedron::MakeL1Ball(3);
  HPolyhedron Linfty_ball = HPolyhedron::MakeUnitBox(3);

  HPolyhedron IntersectionBall = L1_ball.Intersection(Linfty_ball, true);
  EXPECT_TRUE(CompareMatrices(L1_ball.A(), IntersectionBall.A()));
  EXPECT_TRUE(CompareMatrices(L1_ball.b(), IntersectionBall.b()));
}

GTEST_TEST(HPolyhedronTest, OffsetIrredundantBoxes) {
  Vector2d left_box_lower = {-1, -1};
  Vector2d left_box_upper = {0.25, 1};
  HPolyhedron left_box = HPolyhedron::MakeBox(left_box_lower, left_box_upper);

  Vector2d right_box_lower = {-0.25, -1};
  Vector2d right_box_upper = {1, 1};
  HPolyhedron right_box =
      HPolyhedron::MakeBox(right_box_lower, right_box_upper);

  HPolyhedron intersection_right_into_left =
      left_box.Intersection(right_box, true);
  HPolyhedron intersection_left_into_right =
      right_box.Intersection(left_box, true);

  MatrixXd A_right_into_left_expected(5, 2);
  VectorXd b_right_into_left_expected(5);
  MatrixXd A_left_into_right_expected(5, 2);
  VectorXd b_left_into_right_expected(5);

  A_right_into_left_expected.topRows(4) = left_box.A();
  b_right_into_left_expected.topRows(4) = left_box.b();
  A_left_into_right_expected.topRows(4) = right_box.A();
  b_left_into_right_expected.topRows(4) = right_box.b();

  A_right_into_left_expected.row(4) = right_box.A().row(2);
  b_right_into_left_expected.row(4) = right_box.b().row(2);

  A_left_into_right_expected.row(4) = left_box.A().row(0);
  b_left_into_right_expected.row(4) = left_box.b().row(0);

  EXPECT_TRUE(CompareMatrices(A_right_into_left_expected,
                              intersection_right_into_left.A()));
  EXPECT_TRUE(CompareMatrices(b_right_into_left_expected,
                              intersection_right_into_left.b()));

  EXPECT_TRUE(CompareMatrices(A_left_into_right_expected,
                              intersection_left_into_right.A()));
  EXPECT_TRUE(CompareMatrices(b_left_into_right_expected,
                              intersection_left_into_right.b()));
}

GTEST_TEST(HPolyhedronTest, ContainedIn) {
  // Checks Contained in with tolerance.
  const HPolyhedron small_polyhedron(Eigen::RowVector2d(1, 1), Vector1d(2));
  const HPolyhedron large_polyhedron(Eigen::RowVector2d(1, 1), Vector1d(3));
  EXPECT_FALSE(large_polyhedron.ContainedIn(small_polyhedron, 0));
  // We think the containment is true if we relax the tolerance.
  EXPECT_TRUE(large_polyhedron.ContainedIn(small_polyhedron, 1.1));
}

GTEST_TEST(HPolyhedronTest, IrredundantBallIntersectionContainsBothOriginal) {
  HPolyhedron L1_ball = HPolyhedron::MakeL1Ball(3);
  HPolyhedron Linfty_ball = HPolyhedron::MakeUnitBox(3);

  // clang-format on
  HPolyhedron IrredL1intoLinf = Linfty_ball.Intersection(L1_ball, true);
  HPolyhedron IrredLinfintoL1 = L1_ball.Intersection(Linfty_ball, true);

  EXPECT_TRUE(IrredL1intoLinf.ContainedIn(L1_ball, 3E-7));
  EXPECT_TRUE(IrredL1intoLinf.ContainedIn(Linfty_ball));
  EXPECT_TRUE(IrredLinfintoL1.ContainedIn(L1_ball));
  EXPECT_TRUE(IrredLinfintoL1.ContainedIn(Linfty_ball));
}

GTEST_TEST(HPolyhedronTest, ReduceL1LInfBallIntersection) {
  HPolyhedron L1_ball = HPolyhedron::MakeL1Ball(3);
  HPolyhedron Linfty_ball = HPolyhedron::MakeUnitBox(3);

  MatrixXd A_int(L1_ball.A().rows() + Linfty_ball.A().rows(), 3);
  MatrixXd b_int(A_int.rows(), 1);
  A_int.topRows(L1_ball.A().rows()) = L1_ball.A();
  b_int.topRows(L1_ball.b().rows()) = L1_ball.b();
  A_int.bottomRows(Linfty_ball.A().rows()) = Linfty_ball.A();
  b_int.bottomRows(Linfty_ball.b().rows()) = Linfty_ball.b();
  HPolyhedron polyhedron_to_reduce(A_int, b_int);
  HPolyhedron reduced_polyhedron = polyhedron_to_reduce.ReduceInequalities();

  EXPECT_TRUE(CompareMatrices(reduced_polyhedron.A(), L1_ball.A()));
  EXPECT_TRUE(CompareMatrices(reduced_polyhedron.b(), L1_ball.b()));
}

GTEST_TEST(HPolyhedronTest, ReduceToInfeasibleSet) {
  Eigen::MatrixXd A{5, 3};
  Eigen::VectorXd b{5};
  // Rows 1-3 define an infeasible set of inequalities.
  // clang-format off
  A << 1, 0, 0,
       1, -1, 0,
       -1, 0, 1,
       0, 1, -1,
       0, 0, -1;
  b << 1, -1, -1, -1, 0;
  // clang-format off

  HPolyhedron H{A, b};
  HPolyhedron H_reduced = H.ReduceInequalities();

  EXPECT_TRUE(H.IsEmpty());
  EXPECT_TRUE(H_reduced.IsEmpty());
}

GTEST_TEST(HPolyhedronTest, IsEmptyMinimalInequalitySet) {
  Eigen::MatrixXd A_infeasible{3, 3};
  Eigen::VectorXd b_infeasible{3};
  // clang-format off
  A_infeasible << 1, -1, 0,
                  -1, 0, 1,
                  0, 1, -1;
  b_infeasible << -1, -1, -1;
  // clang-format on

  HPolyhedron H{A_infeasible, b_infeasible};
  EXPECT_TRUE(H.IsEmpty());
}

GTEST_TEST(HPolyhedronTest, IsEmptyNonMinimalInequalitySet) {
  Eigen::MatrixXd A{5, 3};
  Eigen::VectorXd b{5};
  // clang-format off
  A << 1, 0, 0,
       0, 0, -1,
       1, -1, 0,
       -1, 0, 1,
       0, 1, -1;;
  b << 1, 0, -1, -1, -1;;
  // clang-format off

  HPolyhedron H{A, b};
  EXPECT_TRUE(H.IsEmpty());
}

GTEST_TEST(HPolyhedronTest, IsEmptyUnboundedHPolyhedron) {
  Eigen::MatrixXd A{2, 2};
  Eigen::VectorXd b{2};
  A << 1, 0, -1, 0;  // only restrict the first coordinate
  b << 1, 1;
  HPolyhedron H{A, b};
  EXPECT_FALSE(H.IsEmpty());
}

GTEST_TEST(HPolyhedronTest, IsEmptyBoundedHPolyhedron) {
  HPolyhedron H = HPolyhedron::MakeUnitBox(2);
  EXPECT_FALSE(H.IsEmpty());
}



GTEST_TEST(HPolyhedronTest, IntersectionTest) {
  HPolyhedron H_A = HPolyhedron::MakeUnitBox(2);
  HPolyhedron H_B = HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(2, 2));
  HPolyhedron H_C = H_A.Intersection(H_B);

  Vector2d x_C(0.5, 0.5);
  EXPECT_TRUE(H_A.PointInSet(x_C));
  EXPECT_TRUE(H_B.PointInSet(x_C));
  EXPECT_TRUE(H_C.PointInSet(x_C));

  Vector2d x_A(-0.5, -0.5);
  EXPECT_TRUE(H_A.PointInSet(x_A));
  EXPECT_FALSE(H_B.PointInSet(x_A));
  EXPECT_FALSE(H_C.PointInSet(x_A));

  Vector2d x_B(1.5, 1.5);
  EXPECT_FALSE(H_A.PointInSet(x_B));
  EXPECT_TRUE(H_B.PointInSet(x_B));
  EXPECT_FALSE(H_C.PointInSet(x_B));
}

GTEST_TEST(HPolyhedronTest, PontryaginDifferenceTestAxisAligned) {
  const HPolyhedron H_A = HPolyhedron::MakeUnitBox(2);
  const HPolyhedron H_B = HPolyhedron::MakeBox(Vector2d(0, 0), Vector2d(1, 1));
  const HPolyhedron H_C = H_A.PontryaginDifference(H_B);
  const HPolyhedron H_C_expected =
      HPolyhedron::MakeBox(Vector2d{-1, -1}, Vector2d{0, 0});

  EXPECT_TRUE(CompareMatrices(H_C.A(), H_C_expected.A(), 1e-8));
  EXPECT_TRUE(CompareMatrices(H_C.b(), H_C_expected.b(), 1e-8));
}

GTEST_TEST(HPolyhedronTest, PontryaginDifferenceTestSquareTriangle) {
  HPolyhedron H_A = HPolyhedron::MakeUnitBox(2);

  Matrix<double, 3, 2> A_B;
  Vector<double, 3> b_B;
  // clang-format off
  A_B << -1, 0,
          0, -1,
          1, 1;
  b_B << 0, 0, 1;
  // clang-format on
  // right triangle with vertices [0,0], [1,0], [0,1]
  const HPolyhedron H_B{A_B, b_B};

  const HPolyhedron H_C = H_A.PontryaginDifference(H_B);

  const HPolyhedron H_C_expected =
      HPolyhedron::MakeBox(Vector2d{-1, -1}, Vector2d{0, 0});

  EXPECT_TRUE(CompareMatrices(H_C.A(), H_C_expected.A(), 1e-8));
  EXPECT_TRUE(CompareMatrices(H_C.b(), H_C_expected.b(), 1e-8));
}

GTEST_TEST(HPolyhedronTest, PontryaginDifferenceTestNonAxisAligned) {
  // L1 box scaled to have corners at 0.5 instead of 1; it is intentionally not
  // axis aligned in this test
  HPolyhedron L1_ball = HPolyhedron::MakeL1Ball(3);
  const HPolyhedron H_A = HPolyhedron::MakeUnitBox(3);

  const HPolyhedron H_B{L1_ball.A(), 0.5 * L1_ball.b()};

  const HPolyhedron H_C = H_A.PontryaginDifference(H_B);

  const HPolyhedron H_C_expected =
      HPolyhedron::MakeBox(Vector3d::Constant(-0.5), Vector3d::Constant(0.5));

  EXPECT_TRUE(CompareMatrices(H_C.A(), H_C_expected.A(), 1e-8));
  EXPECT_TRUE(CompareMatrices(H_C.b(), H_C_expected.b(), 1e-8));
}

GTEST_TEST(HPolyhedronTest, UniformSampleTest) {
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

  // Draw random samples.
  RandomGenerator generator(1234);
  const int N{10000};
  MatrixXd samples(2, N);
  samples.col(0) = H.UniformSample(&generator);
  for (int i = 1; i < N; ++i) {
    samples.col(i) = H.UniformSample(&generator, samples.col(i - 1));
  }

  // Provide a visualization of the points.
  {
    std::shared_ptr<Meshcat> meshcat = geometry::GetTestEnvironmentMeshcat();
    meshcat->SetProperty("/Background", "visible", false);
    perception::PointCloud cloud(N);
    cloud.mutable_xyzs().topRows<2>() = samples.cast<float>();
    cloud.mutable_xyzs().bottomRows<1>().setZero();
    meshcat->SetObject("samples", cloud, 0.01, Rgba(0, 0, 1));

    // Note: This will not pause execution when running as a bazel test.
    std::cout << "[Press RETURN to continue]." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  // Check that they are all in the polyhedron.
  for (int i = 0; i < A.rows(); ++i) {
    EXPECT_LE((A.row(i) * samples).maxCoeff(), b(i));
  }

  const double kTol = 0.05 * N;
  // Check that approximately half of them satisfy 2x+y ≥ 5.
  EXPECT_NEAR(((2 * samples.row(0) + samples.row(1)).array() >= 5.0).count(),
              0.5 * N, kTol);

  // Check that approximately half of them satisfy x - 2y ≥ 5.
  EXPECT_NEAR(((samples.row(0) - 2 * samples.row(1)).array() >= 5.0).count(),
              0.5 * N, kTol);

  // Check that an off-center box gets the number of samples proportional to
  // its (relative) volume. H is a rotated box with volume 1 x 2.5 = 2.5. We'll
  // check the box: 3 ≤ x ≤ 3.5, -1.5 ≤ y ≤ -1, which has volume .5 x .5 = .25.
  EXPECT_NEAR((samples.row(0).array() >= 3 && samples.row(0).array() <= 3.5 &&
               samples.row(1).array() >= -1.5 && samples.row(1).array() <= -1)
                  .count(),
              N / 10, kTol);
}

// Test the case where the sample point is outside the region, but the max
// threshold can be smaller than the min threshold. (This was a bug uncovered
// by hammering on this code from IRIS).
GTEST_TEST(HPolyhedronTest, UniformSampleTest2) {
  Matrix<double, 5, 2> A;
  Matrix<double, 5, 1> b;
  // clang-format off
  A <<  1,  0,  // x ≤ 1
        0,  1,  // y ≤ 1
       -1,  0,  // x ≥ -1
        0, -1,  // y ≥ -1
       -1,  0,  // x ≥ 0
  b << 1, 1, 1, 1, 0;
  // clang-format on
  HPolyhedron H(A, b);

  // Draw random samples.
  RandomGenerator generator(1234);
  // Use a seed that is outside the set (because x ≤ 0), but still inside the
  // [-1, 1] unit box (so the line search in all directions returns finite
  // values). It throws when the hit and run direction intersects x=0 outside
  // of the unit box.
  const Vector2d seed{-0.5, 0.9};
  // Make sure that random samples either return a point in the set (because
  // they were lucky) or throw.  Previously, the method could return a point
  // outside the set.
  int num_throws = 0;
  int num_success = 0;
  for (int i = 0; i < 10; ++i) {
    try {
      const Vector2d sample = H.UniformSample(&generator, seed);
      EXPECT_TRUE(H.PointInSet(sample, 1e-12));
      ++num_success;
    } catch (const std::exception& err) {
      ++num_throws;
      EXPECT_NE(
          std::string(err.what())
              .find("Hit and Run algorithm failed to find a feasible point"),
          std::string::npos);
    }
  }
  // Make sure both paths were touched.
  EXPECT_GT(num_throws, 0);
  EXPECT_GT(num_success, 0);
}

GTEST_TEST(HPolyhedronTest, Serialize) {
  const HPolyhedron H = HPolyhedron::MakeL1Ball(3);
  const std::string yaml = yaml::SaveYamlString(H);
  const auto H2 = yaml::LoadYamlString<HPolyhedron>(yaml);
  EXPECT_EQ(H.ambient_dimension(), H2.ambient_dimension());
  EXPECT_TRUE(CompareMatrices(H.A(), H2.A()));
  EXPECT_TRUE(CompareMatrices(H.b(), H2.b()));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
