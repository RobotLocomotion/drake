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

  // TODO(trowell-tri) Update to a Vector6d argument when rpy is supported.
  std::pair<geometry::Ellipsoid, math::RigidTransform<double>>
  CalculateInertiaGeometryForPose(const Vector3d pose) {
    const std::string model = fmt::vformat(
        model_template_,
        fmt::make_format_args(mass_, pose(0), pose(1), pose(2), moment_ixx_,
                              moment_iyy_, moment_izz_, visual_sdf_));
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

  // TODO(trowell-tri) Extend to allow configuring pose rpy.
  const std::string model_template_ = R"""(
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

  // Subclasses should set the mass_, moment_i..._, and visual_sdf_ members.
  double mass_;
  double moment_ixx_;
  double moment_iyy_;
  double moment_izz_;
  // The visual_sdf_ should contain the visual element of the test fixture
  // to clarify the geometry being modeled.
  std::string visual_sdf_;
};

// A geometry test class which uses a massless body.
class InertiaVisualizerZeroMassTest : public InertiaVisualizerGeometryTest {
 public:
  InertiaVisualizerZeroMassTest() {
    mass_ = 0;
    moment_ixx_ = 0;
    moment_iyy_ = 0;
    moment_izz_ = 0;
    visual_sdf_ = "<sphere><radius>1.0</radius></sphere>";
  }
};

// Test inertia geometry computations for the massless body, posed without
// translation or rotation.
TEST_F(InertiaVisualizerZeroMassTest, UnmodifiedZeroMassTest) {
  Vector3d pose{0, 0, 0};
  auto [ellipsoid, transform] = CalculateInertiaGeometryForPose(pose);

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.c(), 0.001, eps_);
}

// Test inertia geometry computations for the massless body, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerZeroMassTest, ModifiedZeroMassTest) {
  Vector3d pose{10, 20, 30};
  auto [ellipsoid, transform] = CalculateInertiaGeometryForPose(pose);

  // Pose for zero inertia is ignored.
  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.c(), 0.001, eps_);
}

// A geometry test class which uses a simple box model.
class InertiaVisualizerBoxTest : public InertiaVisualizerGeometryTest {
 public:
  InertiaVisualizerBoxTest() {
    mass_ = 2000;
    moment_ixx_ = 833.33333;
    moment_iyy_ = 833.33333;
    moment_izz_ = 333.33333;
    visual_sdf_ = "<box><size>1.0 1.0 2.0</size></box>";
  }
};

// Test inertia geometry computations for the box model, posed without
// translation or rotation.
TEST_F(InertiaVisualizerBoxTest, UnmodifiedBoxTest) {
  Vector3d pose{0, 0, 0};
  auto [ellipsoid, transform] = CalculateInertiaGeometryForPose(pose);

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.6203504901239619, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.6203504901239619, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009849005526, eps_);
}

// Test inertia geometry computations for the box model, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerBoxTest, ModifiedBoxTest) {
  Vector3d pose{100, 200, 300};
  auto [ellipsoid, transform] = CalculateInertiaGeometryForPose(pose);

  EXPECT_TRUE(
      CompareMatrices(transform.GetAsMatrix34().block(0, 3, 3, 1), pose));

  // The ellipsoid radii here are subtly different from the unmodified case.
  EXPECT_NEAR(ellipsoid.a(), 0.62035049012869525, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.62035049012869525, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009848816188, eps_);
}

// A geometry test class which uses a thin cylinder model.
class InertiaVisualizerThinCylinderTest : public InertiaVisualizerGeometryTest {
 public:
  InertiaVisualizerThinCylinderTest() {
    mass_ = 20;
    moment_ixx_ = 166.666666;
    moment_iyy_ = 166.666666;
    moment_izz_ = 1.0e-3;
    visual_sdf_ = R"""(
      <cylinder>
        <radius>0.01</radius><length>10.0</length>
      </cylinder>
      )""";
  }
};

// Test inertia geometry computations for the thin cylinder model, posed without
// translation or rotation.
TEST_F(InertiaVisualizerThinCylinderTest, UnmodifiedThinCylinderTest) {
  Vector3d pose{0, 0, 0};
  auto [ellipsoid, transform] = CalculateInertiaGeometryForPose(pose);

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.020222471033590449, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.020222471033590449, eps_);
  EXPECT_NEAR(ellipsoid.c(), 11.675431558385144, eps_);
}

// Test inertia geometry computations for the thin cylinder model, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerThinCylinderTest, ModifiedThinCylinderTest) {
  Vector3d pose{50, 20, 1};
  auto [ellipsoid, transform] = CalculateInertiaGeometryForPose(pose);

  EXPECT_TRUE(
      CompareMatrices(transform.GetAsMatrix34().block(0, 3, 3, 1), pose));

  // The ellipsoid radii here are subtly different from the unmodified case.
  EXPECT_NEAR(ellipsoid.a(), 0.020222471021249189, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.020222470998258898, eps_);
  EXPECT_NEAR(ellipsoid.c(), 11.675431585909024, eps_);
}

// A geometry test class which uses an ellipsoid model.
class InertiaVisualizerEllipsoidTest : public InertiaVisualizerGeometryTest {
 public:
  InertiaVisualizerEllipsoidTest() {
    mass_ = 2000;
    moment_ixx_ = 2000;
    moment_iyy_ = 2000;
    moment_izz_ = 800;
    visual_sdf_ = "<ellipsoid><radii>1.0 1.0 2.0</radii></ellipsoid>";
  }
};

// Test inertia geometry computations for the ellipsoid model, posed without
// translation or rotation.
TEST_F(InertiaVisualizerEllipsoidTest, UnmodifiedEllipsoidTest) {
  Vector3d pose{0, 0, 0};
  auto [ellipsoid, transform] = CalculateInertiaGeometryForPose(pose);

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.6203504908994, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.6203504908994, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009817988, eps_);
}

// Test inertia geometry computations for the ellipsoid model, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerEllipsoidTest, ModifiedEllipsoidTest) {
  Vector3d pose{10, 200, 50};
  auto [ellipsoid, transform] = CalculateInertiaGeometryForPose(pose);

  EXPECT_TRUE(
      CompareMatrices(transform.GetAsMatrix34().block(0, 3, 3, 1), pose));

  // The ellipsoid radii here are subtly different from the unmodified case
  EXPECT_NEAR(ellipsoid.a(), 0.62035049089987004, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.62035049089987004, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.240700981796919, eps_);
}

}  // namespace
}  // namespace internal
}  // namespace visualization
}  // namespace drake
