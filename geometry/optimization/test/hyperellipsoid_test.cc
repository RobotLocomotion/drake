#include "drake/geometry/optimization/hyperellipsoid.h"

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
  HyperEllipsoid* pointer = dynamic_cast<HyperEllipsoid*>(clone.get());
  ASSERT_NE(pointer, nullptr);
  EXPECT_TRUE(CompareMatrices(E.A(), pointer->A()));
  EXPECT_TRUE(CompareMatrices(E.center(), pointer->center()));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
