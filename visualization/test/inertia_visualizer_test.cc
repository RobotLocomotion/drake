#include "drake/visualization/inertia_visualizer.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
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

namespace drake {
namespace visualization {
namespace internal {
namespace {

// See visualization_config_functions_test for tests that InertiaVisualizers
// are created in ApplyVisualizationConfig, as well as various parameter
// setting tests.

class InertiaVisualizerTest : public ::testing::Test {
 public:
  InertiaVisualizerTest()
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
TEST_F(InertiaVisualizerTest, TestDefaultVisualizationConfig) {
  const VisualizationConfig config;
  SetUpDiagramWithConfig(config);

  EXPECT_TRUE(meshcat_->HasPath("/drake/illustration/acrobot"));
  EXPECT_TRUE(meshcat_->HasPath("/drake/inertia/InertiaVisualizer/acrobot"));
}

// Test that we can show illustration geometry without inertia.
TEST_F(InertiaVisualizerTest, IllustrationButNoInertiaConfig) {
  VisualizationConfig config;
  config.publish_inertia = false;
  SetUpDiagramWithConfig(config);

  EXPECT_TRUE(meshcat_->HasPath("/drake/illustration/acrobot"));
  EXPECT_FALSE(meshcat_->HasPath("/drake/inertia/InertiaVisualizer/acrobot"));
}

// Test that we can show inertia geometry without illustration.
TEST_F(InertiaVisualizerTest, InertiaButNoIllustrationConfig) {
  VisualizationConfig config;
  config.publish_illustration = false;
  SetUpDiagramWithConfig(config);

  EXPECT_FALSE(meshcat_->HasPath("/drake/illustration/acrobot"));
  EXPECT_TRUE(meshcat_->HasPath("/drake/inertia/InertiaVisualizer/acrobot"));
}

// Test inertia geometry computations for a massless body.
GTEST_TEST(InertiaGeometryCalculationTest, ZeroMass) {
  const std::string model = R"""(
  <?xml version='1.0'?>
  <sdf version='1.7'>
    <model name='zero_mass'>
      <pose>0 0 0 0 0 0</pose>
      <link name='ball'>
        <inertial>
          <mass>0.0</mass>
          <inertia>
            <ixx>0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0</iyy>
            <iyz>0</iyz>
            <izz>0</izz>
          </inertia>
        </inertial>
        <visual name='ball_visual'>
          <geometry>
            <sphere><radius>1.0</radius></sphere>
          </geometry>
        </visual>
      </link>
    </model>
  </sdf>
  )""";

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
  multibody::Parser(&plant).AddModelsFromString(model, "sdf");
  plant.Finalize();

  const multibody::Body<double>& body = plant.GetBodyByName("ball");
  auto [ellipsoid, transform] =
      CalculateInertiaGeometry(body, *plant.CreateDefaultContext());

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  const double eps = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(ellipsoid.a(), 0.001, eps);
  EXPECT_NEAR(ellipsoid.b(), 0.001, eps);
  EXPECT_NEAR(ellipsoid.c(), 0.001, eps);
}

// Test inertia geometry computations for a simple box model.
GTEST_TEST(InertiaGeometryCalculationTest, Box) {
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
  multibody::Parser(&plant).AddModelsFromUrl(
      "package://drake/geometry/render/test/box.sdf");
  plant.Finalize();

  const multibody::Body<double>& body = plant.GetBodyByName("box");
  auto [ellipsoid, transform] =
      CalculateInertiaGeometry(body, *plant.CreateDefaultContext());

  EXPECT_EQ(transform.IsExactlyIdentity(), true);
  // TODO(trowell-tri) need to test alternately posed bodies.
  // Add these tests once math is correct.

  const double eps = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(ellipsoid.a(), 0.13365045454007368, eps);
  EXPECT_NEAR(ellipsoid.b(), 0.13365045454007368, eps);
  EXPECT_NEAR(ellipsoid.c(), 0.013365047619144647, eps);
}

}  // namespace
}  // namespace internal
}  // namespace visualization
}  // namespace drake
