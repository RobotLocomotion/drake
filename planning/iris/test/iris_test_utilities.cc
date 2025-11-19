#include "drake/planning/iris/test/iris_test_utilities.h"

#include <algorithm>
#include <utility>

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
using Eigen::Matrix2Xd;
using Eigen::Matrix3Xd;
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
      fmt::arg("w_plus_one_half", kPhysicalParamW + .5),
      fmt::arg("l1", kPhysicalParamL1), fmt::arg("l2", kPhysicalParamL2),
      fmt::arg("r", kPhysicalParamR));

  SetUpEnvironment(urdf_);

  meshcat_ = geometry::GetTestEnvironmentMeshcat();

  Vector2d sample = Vector2d::Zero(2);
  starting_ellipsoid_ = Hyperellipsoid::MakeHypersphere(1e-2, sample);
  domain_ = HPolyhedron::MakeBox(plant_ptr_->GetPositionLowerLimits(),
                                 plant_ptr_->GetPositionUpperLimits());
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
  meshcat_->Set2dRenderMode(math::RigidTransformd(Vector3d{0, 0, 1}), -3.25,
                            3.25, -3.25, 3.25);
  meshcat_->SetProperty("/Grid", "visible", true);
  Eigen::RowVectorXd theta2s = Eigen::RowVectorXd::LinSpaced(100, -1.57, 1.57);
  Matrix3Xd points = Matrix3Xd::Zero(3, 2 * theta2s.size() + 1);
  const double c = -kPhysicalParamW + kPhysicalParamR;
  for (int i = 0; i < theta2s.size(); ++i) {
    const double a = kPhysicalParamL1 + kPhysicalParamL2 * std::cos(theta2s[i]),
                 b = kPhysicalParamL2 * std::sin(theta2s[i]);
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

DoublePendulumRationalForwardKinematics::
    DoublePendulumRationalForwardKinematics()
    : rational_kinematics_(plant_ptr_) {
  starting_ellipsoid_rational_forward_kinematics_ =
      Hyperellipsoid::MakeHypersphere(1e-2, Vector2d::Zero(2));
  domain_rational_forward_kinematics_ =
      HPolyhedron::MakeBox(Vector2d(-1.0, -1.0), Vector2d(1.0, 1.0));

  region_query_point_1_ = Vector2d(-0.1, 0.3);
  region_query_point_2_ = Vector2d(0.1, -0.3);
}

void DoublePendulumRationalForwardKinematics::CheckParameterization(
    const std::function<VectorXd(const VectorXd&)>& parameterization) {
  const Vector2d output = parameterization(Vector2d(0.0, 0.0));
  EXPECT_NEAR(output[0], 0.0, 1e-15);
  EXPECT_NEAR(output[1], 0.0, 1e-15);
}

void DoublePendulumRationalForwardKinematics::
    CheckRegionRationalForwardKinematics(const HPolyhedron& region) {
  EXPECT_EQ(region.ambient_dimension(), 2);
  EXPECT_TRUE(region.PointInSet(region_query_point_1_));
  EXPECT_TRUE(region.PointInSet(region_query_point_2_));
}

void DoublePendulumRationalForwardKinematics::
    PlotEnvironmentAndRegionRationalForwardKinematics(
        const HPolyhedron& region,
        const std::function<VectorXd(const VectorXd&)>& parameterization,
        const Vector2d& region_query_point) {
  VPolytope vregion = VPolytope(region).GetMinimalRepresentation();

  // Region boundaries appear "curved" in the ambient space, so we use many
  // points per boundary segment to make a more faithful visualization.
  int n_points_per_edge = 10;
  Matrix3Xd points =
      Matrix3Xd::Zero(3, n_points_per_edge * vregion.vertices().cols() + 1);
  int next_point_index = 0;

  // Order vertices in counterclockwise order.
  Vector2d centroid = vregion.vertices().rowwise().mean();
  Matrix2Xd centered = vregion.vertices().colwise() - centroid;
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
  Matrix2Xd sorted_vertices = vregion.vertices()(eigen_all, indices);

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
  meshcat_->Set2dRenderMode(math::RigidTransformd(Vector3d{0, 0, 1}), 0, 3.25,
                            -3.25, 3.25);
  meshcat_->SetProperty("/Grid", "visible", true);
  Eigen::RowVectorXd thetas = Eigen::RowVectorXd::LinSpaced(100, -M_PI, M_PI);
  const double w = 2, h = 1;
  Matrix3Xd points = Matrix3Xd::Zero(3, 2 * thetas.size() + 1);
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

ConvexConfigurationSpace::ConvexConfigurationSpace() {
  urdf_ = fmt::format(
      R"(
<robot name="pendulum_on_vertical_track">
  <link name="fixed">
    <collision name="ground">
      <origin rpy="0 0 0" xyz="0 0 -1"/>
      <geometry><box size="10 10 2"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="cart">
  </link>
  <joint name="track" type="prismatic">
    <axis xyz="0 0 1"/>
    <limit lower="-{l}" upper="0"/>
    <parent link="world"/>
    <child link="cart"/>
  </joint>
  <link name="pendulum">
    <collision name="ball">
      <origin rpy="0 0 0" xyz="0 0 {l}"/>
      <geometry><sphere radius="{r}"/></geometry>
    </collision>
  </link>
  <joint name="pendulum" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="cart"/>
    <child link="pendulum"/>
  </joint>
</robot>
)",
      fmt::arg("l", kPhysicalParamL), fmt::arg("r", kPhysicalParamR));

  SetUpEnvironment(urdf_);

  meshcat_ = geometry::GetTestEnvironmentMeshcat();

  Vector2d sample{-0.5, 0.0};
  starting_ellipsoid_ = Hyperellipsoid::MakeHypersphere(1e-2, sample);
  domain_ = HPolyhedron::MakeBox(plant_ptr_->GetPositionLowerLimits(),
                                 plant_ptr_->GetPositionUpperLimits());
}

void ConvexConfigurationSpace::CheckRegion(const HPolyhedron& region) {
  // Confirm that the pendulum is colliding with the wall with true kinematics:
  EXPECT_LE(kZTest + kPhysicalParamL * std::cos(kThetaTest), kPhysicalParamR);

  EXPECT_FALSE(region.PointInSet(Vector2d{kZTest, kThetaTest}));

  EXPECT_EQ(region.ambient_dimension(), 2);
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 0.5);
}

void ConvexConfigurationSpace::PlotEnvironment() {
  meshcat_->Set2dRenderMode(math::RigidTransformd(Vector3d{0, 0, 1}), -3.25,
                            3.25, -3.25, 3.25);
  meshcat_->SetProperty("/Grid", "visible", true);
  Eigen::RowVectorXd theta1s = Eigen::RowVectorXd::LinSpaced(100, -1.5, 1.5);
  Matrix3Xd points = Matrix3Xd::Zero(3, 2 * theta1s.size());
  for (int i = 0; i < theta1s.size(); ++i) {
    points(0, i) = kPhysicalParamR - kPhysicalParamL * cos(theta1s[i]);
    points(1, i) = theta1s[i];
    points(0, points.cols() - i - 1) = 0;
    points(1, points.cols() - i - 1) = theta1s[i];
  }
  meshcat_->SetLine("True C_free", points, 2.0, Rgba(0, 0, 1));
}

void ConvexConfigurationSpace::PlotRegion(const HPolyhedron& region) {
  VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
  Matrix3Xd points = Matrix3Xd::Zero(3, vregion.vertices().cols() + 1);
  points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
  points.topRightCorner(2, 1) = vregion.vertices().col(0);
  points.bottomRows<1>().setZero();
  meshcat_->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

  meshcat_->SetObject("Test point", Sphere(0.03), Rgba(1, 0, 0));
  meshcat_->SetTransform("Test point",
                         math::RigidTransform(Vector3d(kZTest, kThetaTest, 0)));

  MaybePauseForUser();
}

void ConvexConfigurationSpace::PlotEnvironmentAndRegion(
    const HPolyhedron& region) {
  PlotEnvironment();
  PlotRegion(region);
}

ConvexConfigurationSpaceWithThreadsafeConstraint::
    ConvexConfigurationSpaceWithThreadsafeConstraint() {
  // Create the MathematicalProgram with the additional constraint.
  auto q = prog_.NewContinuousVariables(2, "q");
  Eigen::RowVectorXd a(2);
  a << 1, 0;
  double lb = -std::numeric_limits<double>::infinity();
  double ub = -0.3;
  prog_.AddLinearConstraint(a, lb, ub, q);

  // Due to the configuration space margin, this point can never be in the
  // region.
  query_point_in_set_ = Vector2d(-0.31, 0.0);
  query_point_not_in_set_ = Vector2d(-0.29, 0.0);
}

void ConvexConfigurationSpaceWithThreadsafeConstraint::CheckRegion(
    const HPolyhedron& region) {
  EXPECT_FALSE(region.PointInSet(query_point_not_in_set_));
  EXPECT_TRUE(region.PointInSet(query_point_in_set_));
}

namespace {
struct IdentityConstraint {
  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 2; }
  template <typename ScalarType>
  void eval(const Eigen::Ref<const VectorX<ScalarType>>& x,
            VectorX<ScalarType>* y) const {
    (*y) = x;
  }
};
}  // namespace

ConvexConfigurationSpaceWithNotThreadsafeConstraint::
    ConvexConfigurationSpaceWithNotThreadsafeConstraint() {
  // Create the MathematicalProgram with the additional constraint.
  VectorXd simple_constraint_lb = Vector2d(-2.0, -0.5);
  VectorXd simple_constraint_ub = Vector2d(0.0, 1.5);
  std::shared_ptr<solvers::Constraint> simple_constraint =
      std::make_shared<solvers::EvaluatorConstraint<
          solvers::FunctionEvaluator<IdentityConstraint>>>(
          std::make_shared<solvers::FunctionEvaluator<IdentityConstraint>>(
              IdentityConstraint{}),
          simple_constraint_lb, simple_constraint_ub);
  prog_.AddConstraint(simple_constraint, prog_.decision_variables());

  query_point_in_set_ = Vector2d(-1.0, -0.45);
  query_point_not_in_set_ = Vector2d(-1.0, -0.55);
}

ConvexConfigurationSubspace::ConvexConfigurationSubspace() {
  const Vector1d sample2{-0.5};
  starting_ellipsoid_ = Hyperellipsoid::MakeHypersphere(1e-2, sample2);
  // This domain matches the "x" dimension of C-space, so the region generated
  // will respect the joint limits.
  domain_ = HPolyhedron::MakeBox(Vector1d(-1.5), Vector1d(0));

  region_query_point_1_ = Vector1d(-0.75);
  region_query_point_2_ = Vector1d(-0.1);
}

void ConvexConfigurationSubspace::CheckRegion(const HPolyhedron& region) {
  EXPECT_EQ(region.ambient_dimension(), 1);
  EXPECT_TRUE(region.PointInSet(region_query_point_1_));
  EXPECT_TRUE(region.PointInSet(region_query_point_2_));
}

void ConvexConfigurationSubspace::PlotEnvironmentAndRegionSubspace(
    const HPolyhedron& region,
    const std::function<VectorXd(const VectorXd&)>& parameterization) {
  PlotEnvironment();

  VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
  Matrix3Xd points = Matrix3Xd::Zero(3, vregion.vertices().cols() + 1);
  for (int i = 0; i < vregion.vertices().cols(); ++i) {
    Vector2d point = parameterization(vregion.vertices().col(i));
    points.col(i).head(2) = point;
    if (i == 0) {
      points.topRightCorner(2, 1) = point;
    }
  }
  points.bottomRows<1>().setZero();
  meshcat_->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));

  meshcat_->SetObject("Test point", Sphere(0.03), Rgba(1, 0, 0));

  Vector2d ambient_query_point = parameterization(region_query_point_1_);
  meshcat_->SetTransform(
      "Test point", math::RigidTransform(Vector3d(ambient_query_point[0],
                                                  ambient_query_point[1], 0)));

  MaybePauseForUser();
}

FourCornersBoxes::FourCornersBoxes() {
  SetUpEnvironment(urdf_);

  meshcat_ = geometry::GetTestEnvironmentMeshcat();

  Vector2d sample{0.0, 0.0};
  starting_ellipsoid_ = Hyperellipsoid::MakeHypersphere(1e-2, sample);
  domain_ = HPolyhedron::MakeBox(plant_ptr_->GetPositionLowerLimits(),
                                 plant_ptr_->GetPositionUpperLimits());
}

void FourCornersBoxes::CheckRegion(const HPolyhedron&) {}

void FourCornersBoxes::CheckRegionContainsPoints(
    const HPolyhedron& region, const Matrix2Xd& containment_points) {
  EXPECT_EQ(region.ambient_dimension(), 2);
  for (int i = 0; i < containment_points.cols(); ++i) {
    EXPECT_TRUE(region.PointInSet(containment_points.col(i)));
  }
}

void FourCornersBoxes::PlotEnvironmentAndRegion(const HPolyhedron& region) {
  meshcat_->Set2dRenderMode(math::RigidTransformd(Vector3d{0, 0, 1}), -3.25,
                            3.25, -3.25, 3.25);
  meshcat_->SetProperty("/Grid", "visible", true);
  // Draw the true cspace.

  Matrix3Xd env_points(3, 5);
  // clang-format off
        env_points << -2, 2,  2, -2, -2,
                        2, 2, -2, -2,  2,
                        0, 0,  0,  0,  0;
  // clang-format on
  meshcat_->SetLine("Domain", env_points, 8.0, Rgba(0, 0, 0));
  Matrix3Xd centers(3, 4);
  double c = 1.0;
  // clang-format off
        centers << -c, c,  c, -c,
                    c, c, -c, -c,
                    0, 0,  0,  0;
  // clang-format on
  Matrix3Xd obs_points(3, 5);
  // Adding 0.01 offset to obstacles to acommodate for the radius of the
  // spherical robot.
  double s = 0.7 + 0.01;
  // clang-format off
        obs_points << -s, s,  s, -s, -s,
                        s, s, -s, -s, s,
                        s, 0,  0,  0,  0;
  // clang-format on
  for (int obstacle_idx = 0; obstacle_idx < 4; ++obstacle_idx) {
    Matrix3Xd obstacle = obs_points;
    obstacle.colwise() += centers.col(obstacle_idx);
    meshcat_->SetLine(fmt::format("/obstacles/obs_{}", obstacle_idx), obstacle,
                      8.0, Rgba(0, 0, 0));
  }

  VPolytope vregion = VPolytope(region).GetMinimalRepresentation();
  Matrix3Xd points;
  points.resize(3, vregion.vertices().cols() + 1);
  points.topLeftCorner(2, vregion.vertices().cols()) = vregion.vertices();
  points.topRightCorner(2, 1) = vregion.vertices().col(0);
  points.bottomRows<1>().setZero();
  meshcat_->SetLine("IRIS Region", points, 2.0, Rgba(0, 1, 0));
}

void FourCornersBoxes::PlotContainmentPoints(
    const Matrix2Xd& containment_points) {
  Matrix3Xd point_to_draw = Vector3d::Zero();
  for (int i = 0; i < containment_points.cols(); ++i) {
    std::string path = fmt::format("cont_pt/{}", i);
    meshcat_->SetObject(path, Sphere(0.04), geometry::Rgba(1, 0, 0.0, 1.0));
    point_to_draw(0) = containment_points(0, i);
    point_to_draw(1) = containment_points(1, i);
    meshcat_->SetTransform(path, math::RigidTransform<double>(point_to_draw));
  }
}

}  // namespace planning
}  // namespace drake
