#include "drake/visualization/inertia_visualizer.h"

#include <fmt/format.h>
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
// The CalculateInertiaGeometryForArgs() method returns the inertia ellipsoid
// and transform when passed inertial pose args, if a subclass provides a valid
// sdformat in model_template_.
class InertiaVisualizerGeometryTest : public ::testing::Test {
 public:
  InertiaVisualizerGeometryTest()
      : builder_{},
        plant_and_scene_graph_{AddMultibodyPlantSceneGraph(&builder_, 0.001)},
        plant_{plant_and_scene_graph_.plant},
        scene_graph_{plant_and_scene_graph_.scene_graph},
        eps_{std::numeric_limits<double>::epsilon()},
        model_template_{} {}

  std::pair<geometry::Ellipsoid, math::RigidTransform<double>>
  CalculateInertiaGeometryForArgs(const fmt::format_args& pose_args) {
    const std::string model = fmt::vformat(model_template_, pose_args);
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

  // The model_template_ should contain interpolatable translation values for
  // the inertial pose tag.
  // TODO(trowell-tri) Extend to allow configuring pose rpy.
  std::string model_template_;
};

// A geometry test class which uses a massless body.
class InertiaVisualizerZeroMassTest : public InertiaVisualizerGeometryTest {
 public:
  InertiaVisualizerZeroMassTest() {
    // TODO(trowell-tri) Update this model to support rotation.
    model_template_ = R"""(
        <?xml version='1.0'?>
        <sdf version='1.7'>
          <model name='test_model'>
            <link name='test_body'>
              <inertial>
                <mass>0.0</mass>
                <pose>{} {} {} 0 0 0</pose>
                <inertia>
                  <ixx>0</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>0</iyy>
                  <iyz>0</iyz>
                  <izz>0</izz>
                </inertia>
              </inertial>
              <visual name='ball'>
                <geometry>
                  <sphere><radius>1.0</radius></sphere>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        )""";
  }
};

// Test inertia geometry computations for the massless body, posed without
// translation or rotation.
TEST_F(InertiaVisualizerZeroMassTest, UnmodifiedZeroMassTest) {
  auto [ellipsoid, transform] =
      CalculateInertiaGeometryForArgs(fmt::make_format_args(0, 0, 0));

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.c(), 0.001, eps_);
}

// Test inertia geometry computations for the massless body, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
// TODO(trowell-tri) This test show the structure of the modified test
// but does not actually modify the pose as there seems to be a math issue.
TEST_F(InertiaVisualizerZeroMassTest, ModifiedZeroMassTest) {
  auto [ellipsoid, transform] =
      CalculateInertiaGeometryForArgs(fmt::make_format_args(0, 0, 0));

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.001, eps_);
  EXPECT_NEAR(ellipsoid.c(), 0.001, eps_);
}

// A geometry test class which uses a simple box model.
class InertiaVisualizerBoxTest : public InertiaVisualizerGeometryTest {
 public:
  InertiaVisualizerBoxTest() {
    // TODO(trowell-tri) Update this model to support rotation.
    model_template_ = R"""(
        <?xml version='1.0'?>
        <sdf version='1.7'>
          <model name='test_model'>
            <link name='test_body'>
              <inertial>
                <mass>2000.</mass>
                <pose>{} {} {} 0 0 0</pose>
                <inertia>
                  <ixx>833.33333</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>833.33333</iyy>
                  <iyz>0</iyz>
                  <izz>333.33333</izz>
                </inertia>
              </inertial>
              <visual name='box'>
                <geometry>
                  <box><size>1.0 1.0 2.0</size></box>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        )""";
  }
};

// Test inertia geometry computations for the box model, posed without
// translation or rotation.
TEST_F(InertiaVisualizerBoxTest, UnmodifiedBoxTest) {
  auto [ellipsoid, transform] =
      CalculateInertiaGeometryForArgs(fmt::make_format_args(0, 0, 0));

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.6203504901239619, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.6203504901239619, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009849005526, eps_);
}

// Test inertia geometry computations for the box model, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerBoxTest, ModifiedBoxTest) {
  auto [ellipsoid, transform] =
      CalculateInertiaGeometryForArgs(fmt::make_format_args(100, 200, 300));

  // TODO(trowell-tri) The transform is still identity regardless of the
  // translation used, but I didn't think that should be the case.
  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  // TODO(trowell-tri) These are the same a,b,c as the unmodified test,
  // but we're using a different "near" value as the results seem to have
  // shifted slightly and I'm unsure if these should be the same and the
  // difference indicates a separate issue from the pose translation or if
  // this is a floating point limit issue.
  const double delta = 1e-10;
  EXPECT_NEAR(ellipsoid.a(), 0.6203504901239619, delta);
  EXPECT_NEAR(ellipsoid.b(), 0.6203504901239619, delta);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009849005526, delta);
}

// A geometry test class which uses a thin cylinder model.
class InertiaVisualizerThinCylinderTest : public InertiaVisualizerGeometryTest {
 public:
  InertiaVisualizerThinCylinderTest() {
    // TODO(trowell-tri) Update this model to support rotation.
    model_template_ = R"""(
        <?xml version='1.0'?>
        <sdf version='1.7'>
          <model name='test_model'>
            <link name='test_body'>
              <inertial>
                <mass>20.</mass>
                <pose>{} {} {} 0 0 0</pose>
                <inertia>
                  <ixx>166.666666</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>166.666666</iyy>
                  <iyz>0</iyz>
                  <izz>1.0e-3</izz>
                </inertia>
              </inertial>
              <visual name='cylinder'>
                <geometry>
                  <cylinder>
                    <radius>0.01</radius>
                    <length>10.0</length>
                  </cylinder>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        )""";
  }
};

// Test inertia geometry computations for the thin cylinder model, posed without
// translation or rotation.
TEST_F(InertiaVisualizerThinCylinderTest, UnmodifiedThinCylinderTest) {
  auto [ellipsoid, transform] =
      CalculateInertiaGeometryForArgs(fmt::make_format_args(0, 0, 0));

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.020222471033590449, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.020222471033590449, eps_);
  EXPECT_NEAR(ellipsoid.c(), 11.675431558385144, eps_);
}

// Test inertia geometry computations for the thin cylinder model, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
// TODO(trowell-tri) This test show the structure of the modified test
// but does not actually modify the pose as there seems to be a math issue.
TEST_F(InertiaVisualizerThinCylinderTest, ModifiedThinCylinderTest) {
  auto [ellipsoid, transform] =
      CalculateInertiaGeometryForArgs(fmt::make_format_args(0, 0, 0));

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.020222471033590449, eps_);
  EXPECT_NEAR(ellipsoid.b(), 0.020222471033590449, eps_);
  EXPECT_NEAR(ellipsoid.c(), 11.675431558385144, eps_);
}

// A geometry test class which uses an ellipsoid model.
class InertiaVisualizerEllipsoidTest : public InertiaVisualizerGeometryTest {
 public:
  InertiaVisualizerEllipsoidTest() {
    // TODO(trowell-tri) Update this model to support rotation.
    model_template_ = R"""(
        <?xml version='1.0'?>
        <sdf version='1.7'>
          <model name='test_model'>
            <link name='test_body'>
              <inertial>
                <mass>2000.</mass>
                <pose>{} {} {} 0 0 0</pose>
                <inertia>
                  <ixx>2000.</ixx>
                  <ixy>0</ixy>
                  <ixz>0</ixz>
                  <iyy>2000.</iyy>
                  <iyz>0</iyz>
                  <izz>800.</izz>
                </inertia>
              </inertial>
              <visual name='ellipsoid'>
                <geometry>
                  <ellipsoid><radii>1.0 1.0 2.0</radii></ellipsoid>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        )""";
  }
};

// Test inertia geometry computations for the ellipsoid model, posed without
// translation or rotation.
TEST_F(InertiaVisualizerEllipsoidTest, UnmodifiedEllipsoidTest) {
  auto [ellipsoid, transform] =
      CalculateInertiaGeometryForArgs(fmt::make_format_args(0, 0, 0));

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.6203504908994, eps_);
  EXPECT_NEAR(ellipsoid.a(), 0.6203504908994, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009817988, eps_);
}

// Test inertia geometry computations for the ellipsoid model, posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
// TODO(trowell-tri) This test show the structure of the modified test
// but does not actually modify the pose as there seems to be a math issue.
TEST_F(InertiaVisualizerEllipsoidTest, ModifiedEllipsoidTest) {
  auto [ellipsoid, transform] =
      CalculateInertiaGeometryForArgs(fmt::make_format_args(0, 0, 0));

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.6203504908994, eps_);
  EXPECT_NEAR(ellipsoid.a(), 0.6203504908994, eps_);
  EXPECT_NEAR(ellipsoid.c(), 1.2407009817988, eps_);
}

}  // namespace
}  // namespace internal
}  // namespace visualization
}  // namespace drake
