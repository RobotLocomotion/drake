#include "drake/planning/iris/test/iris_test_utilities.h"

#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {

using common::MaybePauseForUser;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::Meshcat;
using geometry::Rgba;
using geometry::Sphere;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::VPolytope;

void IrisTestFixture::SetUpEnvironment(const std::string& urdf) {
  CollisionCheckerParams params;
  RobotDiagramBuilder<double> builder(0.0);

  params.robot_model_instances =
      builder.parser().AddModelsFromString(urdf, "urdf");

  plant_ptr_ = &(builder.plant());
  plant_ptr_->Finalize();

  params.model = builder.Build();
  params.edge_step_size = 0.01;
  checker_ = std::make_unique<SceneGraphCollisionChecker>(std::move(params));
}

JointLimits1D::JointLimits1D() {
  SetUpEnvironment(urdf_);

  Vector1d sample = Vector1d::Zero(1);
  starting_ellipsoid_ = Hyperellipsoid::MakeHypersphere(1e-2, sample);
  domain_ = HPolyhedron::MakeBox(plant_ptr_->GetPositionLowerLimits(),
                                 plant_ptr_->GetPositionUpperLimits());
}

void JointLimits1D::CheckRegion(const HPolyhedron& region) {
  EXPECT_EQ(region.ambient_dimension(), 1);

  const double eps = 1e-5;
  const double qmin = -2.0, qmax = 2.0;
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + eps}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - eps}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - eps}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + eps}));
}

DoublePendulum::DoublePendulum() {
  urdf_ = fmt::format(
      R"(
<robot name="double_pendulum">
  <link name="fixed">
    <collision name="right">
      <origin rpy="0 0 0" xyz="{w_plus_one_half} 0 0"/>
      <geometry><box size="1 1 10"/></geometry>
    </collision>
    <collision name="left">
      <origin rpy="0 0 0" xyz="-{w_plus_one_half} 0 0"/>
      <geometry><box size="1 1 10"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="world"/>
    <child link="link1"/>
  </joint>
  <link name="link2">
    <collision name="ball">
      <origin rpy="0 0 0" xyz="0 0 -{l2}"/>
      <geometry><sphere radius="{r}"/></geometry>
    </collision>
  </link>
  <joint name="joint2" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -{l1}"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
)",
      fmt::arg("w_plus_one_half", physical_param_w_ + .5),
      fmt::arg("l1", physical_param_l1_), fmt::arg("l2", physical_param_l2_),
      fmt::arg("r", physical_param_r_));

  SetUpEnvironment(urdf_);

  meshcat_ = geometry::GetTestEnvironmentMeshcat();

  Vector2d sample = Vector2d::Zero(2);
  starting_ellipsoid_ = Hyperellipsoid::MakeHypersphere(1e-2, sample);
  domain_ = HPolyhedron::MakeBox(plant_ptr_->GetPositionLowerLimits(),
                                 plant_ptr_->GetPositionUpperLimits());

  starting_ellipsoid_rational_forward_kinematics_ = starting_ellipsoid_;
  domain_rational_forward_kinematics_ =
      HPolyhedron::MakeBox(Vector2d(-1.0, -1.0), Vector2d(1.0, 1.0));
}

void DoublePendulum::CheckRegion(const HPolyhedron& region) {
  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 2.0);

  EXPECT_TRUE(region.PointInSet(Vector2d{.4, 0.0}));
  EXPECT_FALSE(region.PointInSet(Vector2d{.5, 0.0}));
  EXPECT_TRUE(region.PointInSet(Vector2d{.3, .3}));
  EXPECT_FALSE(region.PointInSet(Vector2d{.4, .3}));
  EXPECT_TRUE(region.PointInSet(Vector2d{-.4, 0.0}));
  EXPECT_FALSE(region.PointInSet(Vector2d{-.5, 0.0}));
  EXPECT_TRUE(region.PointInSet(Vector2d{-.3, -.3}));
  EXPECT_FALSE(region.PointInSet(Vector2d{-.4, -.3}));
}

void DoublePendulum::PlotEnvironmentAndRegion(
    const geometry::optimization::HPolyhedron& region) {
  meshcat_->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}),
                            -3.25, 3.25, -3.25, 3.25);
  meshcat_->SetProperty("/Grid", "visible", true);
  Eigen::RowVectorXd theta2s = Eigen::RowVectorXd::LinSpaced(100, -1.57, 1.57);
  Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(3, 2 * theta2s.size() + 1);
  const double c = -physical_param_w_ + physical_param_r_;
  for (int i = 0; i < theta2s.size(); ++i) {
    const double a = physical_param_l1_ +
                     physical_param_l2_ * std::cos(theta2s[i]),
                 b = physical_param_l2_ * std::sin(theta2s[i]);
    // wolfram solve a*sin(q) + b*cos(q) = c for q
    points(0, i) =
        2 * std::atan((std::sqrt(a * a + b * b - c * c) + a) / (b + c)) + M_PI;
    points(1, i) = theta2s[i];
    points(0, points.cols() - i - 2) =
        2 * std::atan((std::sqrt(a * a + b * b - c * c) + a) / (b - c)) - M_PI;
    points(1, points.cols() - i - 2) = theta2s[i];
  }
  points.col(points.cols() - 1) = points.col(0);
  meshcat_->SetLine("True C_free", points, 2.0, Rgba(0, 0, 1));
  VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
  points.resize(3, vregion.vertices().cols() + 1);
  points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
  points.topRightCorner(2, 1) = vregion.vertices().col(0);
  points.bottomRows<1>().setZero();
  meshcat_->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

  MaybePauseForUser();
}

void DoublePendulum::PlotEnvironmentAndRegionRationalForwardKinematics(
    const geometry::optimization::HPolyhedron& region,
    const std::function<VectorXd(const Eigen::VectorXd&)>& parameterization,
    const Eigen::Vector2d& region_query_point) {
  VPolytope vregion = VPolytope(region).GetMinimalRepresentation();

  // Region boundaries appear "curved" in the ambient space, so we use many
  // points per boundary segment to make a more faithful visualization.
  int n_points_per_edge = 10;
  Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(
      3, n_points_per_edge * vregion.vertices().cols() + 1);
  int next_point_index = 0;

  // Order vertices in counterclockwise order.
  Vector2d centroid = vregion.vertices().rowwise().mean();
  Eigen::Matrix2Xd centered = vregion.vertices().colwise() - centroid;
  VectorXd angles = centered.row(1).array().binaryExpr(
      centered.row(0).array(), [](double y, double x) {
        return std::atan2(y, x);
      });
  Eigen::VectorXi indices = Eigen::VectorXi::LinSpaced(
      vregion.vertices().cols(), 0, vregion.vertices().cols() - 1);
  std::sort(indices.data(), indices.data() + vregion.vertices().cols(),
            [&angles](int i1, int i2) {
              return angles(i1) < angles(i2);
            });
  Eigen::Matrix2Xd sorted_vertices = vregion.vertices()(Eigen::all, indices);

  for (int i1 = 0; i1 < sorted_vertices.cols(); ++i1) {
    int i2 = i1 + 1;
    if (i2 == sorted_vertices.cols()) {
      i2 = 0;
    }
    Vector2d q1 = sorted_vertices.col(i1);
    Vector2d q2 = sorted_vertices.col(i2);
    for (int j = 0; j < n_points_per_edge; ++j) {
      double t =
          static_cast<double>(j) / static_cast<double>(n_points_per_edge);
      Vector2d q = t * q2 + (1 - t) * q1;
      points.col(next_point_index).head(2) = parameterization(q);
      ++next_point_index;
    }
  }
  points.topRightCorner(2, 1) = parameterization(sorted_vertices.col(0));
  points.bottomRows<1>().setZero();
  meshcat_->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

  meshcat_->SetObject("Test point", Sphere(0.03), Rgba(1, 0, 0));

  Vector2d ambient_query_point = parameterization(region_query_point);
  meshcat_->SetTransform(
      "Test point", math::RigidTransform(Vector3d(ambient_query_point[0],
                                                  ambient_query_point[1], 0)));

  MaybePauseForUser();
}

BlockOnGround::BlockOnGround() {
  SetUpEnvironment(urdf_);

  meshcat_ = geometry::GetTestEnvironmentMeshcat();

  Vector2d sample(1.0, 0.0);
  starting_ellipsoid_ = Hyperellipsoid::MakeHypersphere(1e-2, sample);
  domain_ = HPolyhedron::MakeBox(plant_ptr_->GetPositionLowerLimits(),
                                 plant_ptr_->GetPositionUpperLimits());
}

void BlockOnGround::CheckRegion(const HPolyhedron& region) {
  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 2.0);
}

void BlockOnGround::PlotEnvironmentAndRegion(const HPolyhedron& region) {
  meshcat_->Set2dRenderMode(math::RigidTransformd(Eigen::Vector3d{0, 0, 1}), 0,
                            3.25, -3.25, 3.25);
  meshcat_->SetProperty("/Grid", "visible", true);
  Eigen::RowVectorXd thetas = Eigen::RowVectorXd::LinSpaced(100, -M_PI, M_PI);
  const double w = 2, h = 1;
  Eigen::Matrix3Xd points = Eigen::Matrix3Xd::Zero(3, 2 * thetas.size() + 1);
  for (int i = 0; i < thetas.size(); ++i) {
    const double a = 0.5 * (-w * std::sin(thetas[i]) - h * std::cos(thetas[i])),
                 b = 0.5 * (-w * std::sin(thetas[i]) + h * std::cos(thetas[i])),
                 c = 0.5 * (+w * std::sin(thetas[i]) - h * std::cos(thetas[i])),
                 d = 0.5 * (+w * std::sin(thetas[i]) + h * std::cos(thetas[i]));
    points(0, i) = std::max({a, b, c, d});
    points(1, i) = thetas[i];
    points(0, points.cols() - i - 2) = 3.0;
    points(1, points.cols() - i - 2) = thetas[i];
  }
  points.col(points.cols() - 1) = points.col(0);
  meshcat_->SetLine("True C_free", points, 2.0, Rgba(0, 0, 1));
  VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
  points.resize(3, vregion.vertices().cols() + 1);
  points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
  points.topRightCorner(2, 1) = vregion.vertices().col(0);
  points.bottomRows<1>().setZero();
  meshcat_->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

  MaybePauseForUser();
}

}  // namespace planning
}  // namespace drake
