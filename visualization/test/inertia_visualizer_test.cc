#include "drake/visualization/inertia_visualizer.h"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

using drake::geometry::Ellipsoid;
using drake::geometry::Meshcat;
using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraphResult;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;
using Matrix34d = Eigen::Matrix<double, 3, 4>;

namespace drake {
namespace visualization {
namespace internal {
namespace {

// See visualization_config_functions_test for tests that InertiaVisualizers
// are created in ApplyVisualizationConfig, as well as various parameter
// setting tests.

// This class allows testing that inertia visualization is being published.
class InertiaVisualizerConfigTest : public ::testing::Test {
 public:
  InertiaVisualizerConfigTest()
      : meshcat_{geometry::GetTestEnvironmentMeshcat()},
        builder_{},
        plant_and_scene_graph_{AddMultibodyPlantSceneGraph(&builder_, 0.0)},
        plant_{plant_and_scene_graph_.plant},
        scene_graph_{plant_and_scene_graph_.scene_graph} {}

  void SetUpDiagramWithConfig(const VisualizationConfig& config) {
    Parser parser(&plant_);
    meshcat_->Delete();
    parser.AddModels(
        FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf"));
    plant_.Finalize();

    ApplyVisualizationConfig(config, &builder_, nullptr, &plant_, &scene_graph_,
                             meshcat_);

    diagram_ = builder_.Build();
    context_ = diagram_->CreateDefaultContext();
    diagram_->ForcedPublish(*context_);
  }

 protected:
  std::shared_ptr<Meshcat> meshcat_;
  systems::DiagramBuilder<double> builder_;
  AddMultibodyPlantSceneGraphResult<double> plant_and_scene_graph_;
  MultibodyPlant<double>& plant_;
  SceneGraph<double>& scene_graph_;
  std::unique_ptr<systems::Diagram<double>> diagram_{};
  std::unique_ptr<systems::Context<double>> context_{};
};

// Test that an added model has both illustration and inertia geometry by
// default.
TEST_F(InertiaVisualizerConfigTest, TestDefaultVisualizationConfig) {
  const VisualizationConfig config;
  SetUpDiagramWithConfig(config);

  EXPECT_TRUE(meshcat_->HasPath("/drake/illustration/acrobot"));
  EXPECT_TRUE(meshcat_->HasPath("/drake/inertia/InertiaVisualizer/acrobot"));
}

// Test that we can show illustration geometry without inertia.
TEST_F(InertiaVisualizerConfigTest, IllustrationButNoInertiaConfig) {
  VisualizationConfig config;
  config.publish_inertia = false;
  SetUpDiagramWithConfig(config);

  EXPECT_TRUE(meshcat_->HasPath("/drake/illustration/acrobot"));
  EXPECT_FALSE(meshcat_->HasPath("/drake/inertia/InertiaVisualizer/acrobot"));
}

// Test that we can show inertia geometry without illustration.
TEST_F(InertiaVisualizerConfigTest, InertiaButNoIllustrationConfig) {
  VisualizationConfig config;
  config.publish_illustration = false;
  SetUpDiagramWithConfig(config);

  EXPECT_FALSE(meshcat_->HasPath("/drake/illustration/acrobot"));
  EXPECT_TRUE(meshcat_->HasPath("/drake/inertia/InertiaVisualizer/acrobot"));
}

// This class allows testing the results of inertia geometry calculation.
//
// The CalculateInertiaGeometryForPose() method returns the inertia ellipsoid
// and transform when passed inertial pose args, if a subclass provides valid
// values for the mass_, moment_i..._, and visual_sdf_ members.
class InertiaVisualizerGeometryTest : public ::testing::Test {
 public:
  InertiaVisualizerGeometryTest()
      : builder_{},
        plant_and_scene_graph_{AddMultibodyPlantSceneGraph(&builder_, 0.001)},
        plant_{plant_and_scene_graph_.plant},
        scene_graph_{plant_and_scene_graph_.scene_graph},
        eps_{std::numeric_limits<double>::epsilon()} {};

  struct GeometryTestCase {
    Vector3d pose{Vector3d::Zero()};
    double mass{0};
    double moment_ixx{0};
    double moment_iyy{0};
    double moment_izz{0};
    // The visual_sdf_ should contain the visual element of the test fixture
    // to clarify the geometry being modeled.
    std::string visual_sdf;
  };

  // TODO(trowell-tri) Update to a Vector6d argument when rpy is supported.
  // Returns the value of CalculateInertiaGeometry for the given input.
  std::pair<geometry::Ellipsoid, math::RigidTransform<double>>
  CalculateInertiaGeometryFor(const GeometryTestCase& input) {
    // TODO(trowell-tri) Extend to allow configuring pose rpy.
    static constexpr char kModelTemplate[] = R"""(
          <?xml version='1.0'?>
          <sdf version='1.7'>
            <model name='test_model'>
              <link name='test_body'>
                <inertial>
                  <mass>{}</mass>
                  <pose>{} {} {} 0 0 0</pose>
                  <inertia>
                    <ixx>{}</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>{}</iyy>
                    <iyz>0</iyz>
                    <izz>{}</izz>
                  </inertia>
                </inertial>
                <visual name="test_visual">
                  <geometry>
                    {}
                  </geometry>
                </visual>
              </link>
            </model>
          </sdf>
          )""";
    const std::string model =
        fmt::format(kModelTemplate, input.mass, input.pose.x(), input.pose.y(),
                    input.pose.z(), input.moment_ixx, input.moment_iyy,
                    input.moment_izz, input.visual_sdf);
    drake::log()->info("Using SDF model: {}", model);

    multibody::Parser(&plant_).AddModelsFromString(model, "sdf");
    plant_.Finalize();

    const multibody::Body<double>& body = plant_.GetBodyByName("test_body");
    return CalculateInertiaGeometry(body, *plant_.CreateDefaultContext());
  }

 protected:
  systems::DiagramBuilder<double> builder_;
  AddMultibodyPlantSceneGraphResult<double> plant_and_scene_graph_;
  MultibodyPlant<double>& plant_;
  SceneGraph<double>& scene_graph_;
  const double eps_;
};

// Test inertia geometry computations for the massless body, posed without
// translation or rotation.
TEST_F(InertiaVisualizerGeometryTest, ZeroMassTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.visual_sdf = "<sphere><radius>1.0</radius></sphere>"});

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.c(), 0.001, eps_);
}

// Test inertia geometry computations for the massless body, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerGeometryTest, PosedZeroMassTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.pose = Vector3d{10, 20, 30},
       .visual_sdf = "<sphere><radius>1.0</radius></sphere>"});

  // Pose for zero inertia is ignored.
  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.c(), 0.001, eps_);
}

// Test inertia geometry computations for the box model, posed without
// translation or rotation.
TEST_F(InertiaVisualizerGeometryTest, BoxTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.mass = 2000,
       .moment_ixx = 833.33333,
       .moment_iyy = 833.33333,
       .moment_izz = 333.33333,
       .visual_sdf = "<box><size>1.0 1.0 2.0</size></box>"});

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.6203504901239619, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.6203504901239619, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009849005526, eps_);
}

// Test inertia geometry computations for the box model, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerGeometryTest, PosedBoxTest) {
  const Vector3d pose{100, 200, 300};
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.pose = pose,
       .mass = 2000,
       .moment_ixx = 833.33333,
       .moment_iyy = 833.33333,
       .moment_izz = 333.33333,
       .visual_sdf = "<box><size>1.0 1.0 2.0</size></box>"});

  EXPECT_TRUE(
      CompareMatrices(transform.GetAsMatrix34().block(0, 3, 3, 1), pose));

  // The ellipsoid radii here are subtly different from the unposed case.
  EXPECT_NEAR(ellipsoid.a(), 0.62035049012869525, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.62035049012869525, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009848816188, eps_);
}

// Test inertia geometry computations for the thin cylinder model, posed without
// translation or rotation.
TEST_F(InertiaVisualizerGeometryTest, ThinCylinderTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.mass = 20,
       .moment_ixx = 166.666666,
       .moment_iyy = 166.666666,
       .moment_izz = 1.0e-3,
       .visual_sdf =
           "<cylinder><radius>0.01</radius><length>10.0</length></cylinder>"});

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.020222471033590449, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.020222471033590449, eps_);
  EXPECT_NEAR(ellipsoid.c(), 11.675431558385144, eps_);
}

// Test inertia geometry computations for the thin cylinder model, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerGeometryTest, PosedThinCylinderTest) {
  const Vector3d pose{50, 20, 1};
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.pose = pose,
       .mass = 20,
       .moment_ixx = 166.666666,
       .moment_iyy = 166.666666,
       .moment_izz = 1.0e-3,
       .visual_sdf =
           "<cylinder><radius>0.01</radius><length>10.0</length></cylinder>"});

  EXPECT_TRUE(
      CompareMatrices(transform.GetAsMatrix34().block(0, 3, 3, 1), pose));

  // The ellipsoid radii here are subtly different from the unposed case.
  EXPECT_NEAR(ellipsoid.a(), 0.020222471021249189, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.020222470998258898, eps_);
  EXPECT_NEAR(ellipsoid.c(), 11.675431585909024, eps_);
}

// Test inertia geometry computations for the ellipsoid model, posed without
// translation or rotation.
TEST_F(InertiaVisualizerGeometryTest, EllipsoidTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.mass = 2000,
       .moment_ixx = 2000,
       .moment_iyy = 2000,
       .moment_izz = 800,
       .visual_sdf = "<ellipsoid><radii>1.0 1.0 2.0</radii></ellipsoid>"});

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.6203504908994, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.6203504908994, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009817988, eps_);
}

// Test inertia geometry computations for the ellipsoid model, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerGeometryTest, PosedEllipsoidTest) {
  Vector3d pose{10, 200, 50};
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.pose = pose,
       .mass = 2000,
       .moment_ixx = 2000,
       .moment_iyy = 2000,
       .moment_izz = 800,
       .visual_sdf = "<ellipsoid><radii>1.0 1.0 2.0</radii></ellipsoid>"});

  EXPECT_TRUE(
      CompareMatrices(transform.GetAsMatrix34().block(0, 3, 3, 1), pose));

  // The ellipsoid radii here are subtly different from the unposed case
  EXPECT_NEAR(ellipsoid.a(), 0.62035049089987004, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.62035049089987004, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.240700981796919, eps_);
}

}  // namespace
}  // namespace internal
}  // namespace visualization
}  // namespace drake
