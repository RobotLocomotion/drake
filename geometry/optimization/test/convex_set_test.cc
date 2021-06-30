#include "drake/geometry/optimization/convex_set.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

std::tuple<std::unique_ptr<SceneGraph<double>>, GeometryId>
MakeSceneGraphWithShape(const Shape& shape, const RigidTransformd& X_WG) {
  auto scene_graph = std::make_unique<SceneGraph<double>>();
  SourceId source = scene_graph->RegisterSource("test");
  auto instance =
      std::make_unique<GeometryInstance>(X_WG, shape.Clone(), "test");
  instance->set_proximity_properties(ProximityProperties());
  GeometryId geom_id =
      scene_graph->RegisterAnchoredGeometry(source, std::move(instance));
  return std::tuple<std::unique_ptr<SceneGraph<double>>, GeometryId>(
      std::move(scene_graph), geom_id);
}

// Returns true iff the convex optimization can make the `point` be in the set.
bool CheckAddPointInSetConstraint(
    const ConvexSet& set, const Eigen::Ref<const Eigen::VectorXd>& point) {
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(point.size());
  set.AddPointInSetConstraint(&prog, x);
  // x = point.
  prog.AddBoundingBoxConstraint(point, point, x);
  return solvers::Solve(prog).is_success();
}

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
  HPolyhedron* ptr = dynamic_cast<HPolyhedron*>(clone.get());
  EXPECT_NE(ptr, nullptr);
  EXPECT_TRUE(CompareMatrices(H.A(), ptr->A()));
  EXPECT_TRUE(CompareMatrices(H.b(), ptr->b()));
}

GTEST_TEST(HyperEllipsoidTest, UnitSphereTest) {
  // Test constructor.
  const Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  const Vector3d center = Vector3d::Zero();
  HyperEllipsoid E(A, center);
  EXPECT_EQ(E.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(A, E.A()));
  EXPECT_TRUE(CompareMatrices(center, E.center()));

  // Test SceneGraph constructor.
  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Sphere(1.0), RigidTransformd::Identity());
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  HyperEllipsoid E_scene_graph(query, geom_id);
  EXPECT_TRUE(CompareMatrices(A, E_scene_graph.A()));
  EXPECT_TRUE(CompareMatrices(center, E_scene_graph.center()));

  // Test PointInSet.
  const Vector3d in1{.99, 0, 0}, in2{.5, .5, .5}, out1{1.01, 0, 0},
      out2{1.0, 1.0, 1.0};

  EXPECT_LE(query.ComputeSignedDistanceToPoint(in1)[0].distance, 0.0);
  EXPECT_LE(query.ComputeSignedDistanceToPoint(in2)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out1)[0].distance, 0.0);
  EXPECT_GE(query.ComputeSignedDistanceToPoint(out2)[0].distance, 0.0);

  EXPECT_TRUE(E.PointInSet(in1));
  EXPECT_TRUE(E.PointInSet(in2));
  EXPECT_FALSE(E.PointInSet(out1));
  EXPECT_FALSE(E.PointInSet(out2));

  EXPECT_TRUE(CheckAddPointInSetConstraint(E, in1));
  EXPECT_TRUE(CheckAddPointInSetConstraint(E, in2));
  EXPECT_FALSE(CheckAddPointInSetConstraint(E, out1));
  EXPECT_FALSE(CheckAddPointInSetConstraint(E, out2));

  // Test ToShapeWithPose.
  auto [shape, X_WG] = E.ToShapeWithPose();
  Ellipsoid* ellipsoid = dynamic_cast<Ellipsoid*>(shape.get());
  EXPECT_TRUE(ellipsoid != NULL);
  EXPECT_NEAR(ellipsoid->a(), 1.0, 1e-16);
  EXPECT_NEAR(ellipsoid->b(), 1.0, 1e-16);
  EXPECT_NEAR(ellipsoid->c(), 1.0, 1e-16);
  EXPECT_TRUE(CompareMatrices(X_WG.translation(), center, 1e-16));
  // Note: Any rotation in X_WG yields a correct solution; since the unit
  // sphere is rotationally symmetric.
}

GTEST_TEST(HyperEllipsoidTest, ScaledSphereTest) {
  const double radius = 0.1;
  auto [scene_graph, geom_id] =
      MakeSceneGraphWithShape(Sphere(radius), RigidTransformd::Identity());
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  HyperEllipsoid E(query, geom_id);
  EXPECT_TRUE(
      CompareMatrices(E.A(), Eigen::Matrix3d::Identity() / radius, 1e-16));
  EXPECT_TRUE(E.center().isZero(1e-16));

  auto [shape, X_WS] = E.ToShapeWithPose();

  Ellipsoid* ellipsoid = dynamic_cast<Ellipsoid*>(shape.get());
  EXPECT_TRUE(ellipsoid != nullptr);
  EXPECT_NEAR(ellipsoid->a(), radius, 1e-16);
  EXPECT_NEAR(ellipsoid->b(), radius, 1e-16);
  EXPECT_NEAR(ellipsoid->c(), radius, 1e-16);

  EXPECT_TRUE(X_WS.IsIdentityToEpsilon(1e-16));
}

GTEST_TEST(HyperEllipsoidTest, ArbitraryEllipsoidTest) {
  const Eigen::Matrix3d D = Eigen::DiagonalMatrix<double, 3>(1.0, 2.0, 3.0);
  const RotationMatrixd R = RotationMatrixd::MakeZRotation(M_PI / 2.0);
  const Eigen::Matrix3d A = D * R.matrix();
  const Vector3d center{4.0, 5.0, 6.0};

  HyperEllipsoid E(A, center);
  EXPECT_EQ(E.ambient_dimension(), 3);
  EXPECT_TRUE(CompareMatrices(A, E.A()));
  EXPECT_TRUE(CompareMatrices(center, E.center()));

  // Test SceneGraph constructor.
  auto [scene_graph, geom_id] = MakeSceneGraphWithShape(
      Ellipsoid(1.0 / D(0, 0), 1.0 / D(1, 1), 1.0 / D(2, 2)),
      RigidTransformd{R.inverse(), center});
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);

  HyperEllipsoid E_scene_graph(query, geom_id);
  EXPECT_TRUE(CompareMatrices(A, E_scene_graph.A()));
  EXPECT_TRUE(CompareMatrices(center, E_scene_graph.center(), 1e-15));

  // Test PointInSet.
  Vector3d in1{.99, 0, 0}, in2{.5, .5, .5}, out1{1.01, 0, 0},
      out2{1.0, 1.0, 1.0};
  const Eigen::Matrix3d B = R.inverse() * D.inverse();
  in1 = B * in1 + center;
  in2 = B * in2 + center;
  out1 = B * out1 + center;
  out2 = B * out2 + center;

  // TODO(russt): Add ComputeSignedDistanceToPoint queries once HyperEllipsoids
  // are supported.

  EXPECT_TRUE(E.PointInSet(in1));
  EXPECT_TRUE(E.PointInSet(in2));
  EXPECT_FALSE(E.PointInSet(out1));
  EXPECT_FALSE(E.PointInSet(out2));

  EXPECT_TRUE(CheckAddPointInSetConstraint(E, in1));
  EXPECT_TRUE(CheckAddPointInSetConstraint(E, in2));
  EXPECT_FALSE(CheckAddPointInSetConstraint(E, out1));
  EXPECT_FALSE(CheckAddPointInSetConstraint(E, out2));

  // Test expressed_in frame.
  SourceId source_id = scene_graph->RegisterSource("new_frame");
  FrameId frame_id =
      scene_graph->RegisterFrame(source_id, GeometryFrame("new_frame"));
  auto context2 = scene_graph->CreateDefaultContext();
  const FramePoseVector<double> pose_vector{
      {frame_id, RigidTransformd(center)}};
  scene_graph->get_source_pose_port(source_id).FixValue(context2.get(),
                                                        pose_vector);
  auto query2 =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context2);
  HyperEllipsoid E_P(query2, geom_id, frame_id);

  EXPECT_TRUE(E_P.PointInSet(Vector3d::Zero()));
  EXPECT_FALSE(E_P.PointInSet(in1));
  EXPECT_FALSE(E_P.PointInSet(in2));

  // Test ToShapeWithPose.
  auto [shape, X_WG] = E.ToShapeWithPose();
  Ellipsoid* ellipsoid = dynamic_cast<Ellipsoid*>(shape.get());
  EXPECT_TRUE(ellipsoid != NULL);
  // Confirm that the axes are ordered by magnitude as documented.
  EXPECT_GE(ellipsoid->a(), ellipsoid->b());
  EXPECT_GE(ellipsoid->b(), ellipsoid->c());
  // Note: D was intentionally ordered by magnitude, also.
  EXPECT_NEAR(ellipsoid->a(), 1.0 / D(0, 0), 1e-16);
  EXPECT_NEAR(ellipsoid->b(), 1.0 / D(1, 1), 1e-16);
  EXPECT_NEAR(ellipsoid->c(), 1.0 / D(2, 2), 1e-16);
  EXPECT_TRUE(CompareMatrices(X_WG.translation(), center, 1e-16));
  const Eigen::Matrix3d R_WG = X_WG.rotation().matrix();
  const Eigen::Matrix3d R_WG_expected = R.inverse().matrix();
  // We can only expect the transforms to match up to a symmetry.
  // Check that the columns are either aligned or antipolar.
  for (int col = 0; col < 3; col++) {
    EXPECT_GE(std::abs(R_WG.col(col).dot(R_WG_expected.col(col))), 1.0 - 1e-16);
  }
}

GTEST_TEST(HyperEllipsoidTest, UnitSphere6DTest) {
  HyperEllipsoid E(Eigen::Matrix<double, 6, 6>::Identity(), Vector6d::Zero());
  EXPECT_EQ(E.ambient_dimension(), 6);

  const double kScale = sqrt(1.0 / 6.0);
  Vector6d in1{Vector6d::Constant(-.99 * kScale)},
      in2{Vector6d::Constant(.99 * kScale)},
      out1{Vector6d::Constant(-1.01 * kScale)},
      out2{Vector6d::Constant(1.01 * kScale)};

  EXPECT_TRUE(E.PointInSet(in1));
  EXPECT_TRUE(E.PointInSet(in2));
  EXPECT_FALSE(E.PointInSet(out1));
  EXPECT_FALSE(E.PointInSet(out2));
}

GTEST_TEST(HyperEllipsoidTest, CloneTest) {
  HyperEllipsoid E(Eigen::Matrix<double, 6, 6>::Identity(), Vector6d::Zero());
  std::unique_ptr<ConvexSet> clone = E.Clone();
  EXPECT_EQ(clone->ambient_dimension(), E.ambient_dimension());
  HyperEllipsoid* ptr = dynamic_cast<HyperEllipsoid*>(clone.get());
  EXPECT_NE(ptr, nullptr);
  EXPECT_TRUE(CompareMatrices(E.A(), ptr->A()));
  EXPECT_TRUE(CompareMatrices(E.center(), ptr->center()));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
