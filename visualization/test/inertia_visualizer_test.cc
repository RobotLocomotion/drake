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
using drake::multibody::SpatialInertia;
using Eigen::RowVector3d;
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
    parser.SetStrictParsing();
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
  VisualizationConfig config;
  // TODO(trowell-tri) Remove the next line when inertia defaults to published.
  config.publish_inertia = true;
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
  // TODO(trowell-tri) Remove the next line when inertia defaults to published.
  config.publish_inertia = true;
  SetUpDiagramWithConfig(config);

  EXPECT_FALSE(meshcat_->HasPath("/drake/illustration/acrobot"));
  EXPECT_TRUE(meshcat_->HasPath("/drake/inertia/InertiaVisualizer/acrobot"));
}

// This class allows testing the results of inertia geometry calculation.
//
// The CalculateInertiaGeometryFor() method returns the inertia ellipsoid
// and transform for a single-body model when passed a GeometryTestCase.
class InertiaVisualizerGeometryTest
    : public ::testing::TestWithParam<RowVector3d> {
 public:
  InertiaVisualizerGeometryTest()
      : builder_{},
        plant_and_scene_graph_{AddMultibodyPlantSceneGraph(&builder_, 0.001)},
        plant_{plant_and_scene_graph_.plant},
        scene_graph_{plant_and_scene_graph_.scene_graph} {};

  struct GeometryTestCase {
    // The visual_sdf_ should contain the visual element of the test fixture
    // to clarify the geometry being modeled.
    std::string visual_sdf;
    double mass{0};
    double moment_ixx{0};
    double moment_iyy{0};
    double moment_izz{0};
  };

  // Returns the value of CalculateInertiaGeometry for the given input.
  std::pair<geometry::Ellipsoid, math::RigidTransform<double>>
  CalculateInertiaGeometryFor(const GeometryTestCase& input) {
    const Vector3d pose = GetParam().transpose();
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
                  <pose>{} {} {} 0 0 0</pose>
                  <geometry>
                    {}
                  </geometry>
                </visual>
              </link>
            </model>
          </sdf>
          )""";
    const std::string model =
        fmt::format(kModelTemplate, input.mass, pose.x(), pose.y(), pose.z(),
                    input.moment_ixx, input.moment_iyy, input.moment_izz,
                    pose.x(), pose.y(), pose.z(), input.visual_sdf);
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
};

// These tests need to compare results using a somewhat forgiving tolerance
// because they do a lot of math where errors will accumulate. We don't expect
// our code to be only a little bit wrong; any implementation errors should
// cause fairly major differences, well above this tolerance.
constexpr double kTolerance = 1e-10;

// This matches the density used in the inertia visualizer implementation.
constexpr double kNominalDensity = 1000.0;

// Test that an ellipsoid model that uses the same nominal density as the
// visualization ellipsoids has the same radii. This test has translation
// but no rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_P(InertiaVisualizerGeometryTest, EllipsoidTest) {
  const Vector3d pose = GetParam().transpose();
  constexpr double a = 3.0;
  constexpr double b = 2.0;
  constexpr double c = 1.0;
  const auto M = SpatialInertia<double>::SolidEllipsoidWithDensity(
      kNominalDensity, a, b, c);
  const GeometryTestCase test_case{
      .visual_sdf = fmt::format(
          "<ellipsoid><radii>{} {} {}</radii></ellipsoid>", a, b, c),
      .mass = M.get_mass(),
      .moment_ixx = M.CalcRotationalInertia().get_moments()[0],
      .moment_iyy = M.CalcRotationalInertia().get_moments()[1],
      .moment_izz = M.CalcRotationalInertia().get_moments()[2],
  };

  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(test_case);

  EXPECT_TRUE(CompareMatrices(transform.translation(), pose));

  EXPECT_NEAR(ellipsoid.a(), a, kTolerance);
  EXPECT_NEAR(ellipsoid.b(), b, kTolerance);
  EXPECT_NEAR(ellipsoid.c(), c, kTolerance);
}

// Test inertia geometry computations for a massless body posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_P(InertiaVisualizerGeometryTest, ZeroMassTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.visual_sdf = "<sphere><radius>1.0</radius></sphere>"});

  // Pose for zero inertia is ignored.
  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, kTolerance);
  EXPECT_NEAR(ellipsoid.b(), 0.001, kTolerance);
  EXPECT_NEAR(ellipsoid.c(), 0.001, kTolerance);
}

// Test that the inertia visualization ellipsoid for a box that uses the same
// nominal density as those ellipsoids has spatial inertia moments in the same
// ratio as those of the box. This test has translation but no rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_P(InertiaVisualizerGeometryTest, BoxTest) {
  const Vector3d pose = GetParam().transpose();
  constexpr double x = 3.0;
  constexpr double y = 2.0;
  constexpr double z = 1.0;
  const auto M =
      SpatialInertia<double>::SolidBoxWithDensity(kNominalDensity, x, y, z);
  auto box_inertia = M.CalcRotationalInertia();
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.visual_sdf = fmt::format("<box><size>{} {} {}</size></box>", x, y, z),
       .mass = M.get_mass(),
       .moment_ixx = box_inertia.get_moments()[0],
       .moment_iyy = box_inertia.get_moments()[1],
       .moment_izz = box_inertia.get_moments()[2]});

  EXPECT_TRUE(CompareMatrices(transform.translation(), pose));

  const auto ellipsoid_M = SpatialInertia<double>::SolidEllipsoidWithDensity(
      kNominalDensity, ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
  EXPECT_NEAR(ellipsoid_M.get_mass(), M.get_mass(), kTolerance);

  // Check moments of the box and the ellipsoid are proportional to each other
  // and the scale factor isn't too wide.
  auto ellipsoid_inertia = ellipsoid_M.CalcRotationalInertia();
  const Vector3d ratio = ellipsoid_inertia.get_moments().array() /
                         box_inertia.get_moments().array();
  SCOPED_TRACE(
      fmt::format("box_inertia = {}; ellipsoid_inertia = {}; ratio = {}",
                  box_inertia, ellipsoid_inertia, fmt_eigen(ratio)));
  double scale = ratio(0);
  EXPECT_LT(std::max(scale, 1 / scale), 2.0);
  EXPECT_NEAR(ratio(1), scale, kTolerance);
  EXPECT_NEAR(ratio(2), scale, kTolerance);
}

// Test inertia geometry computations for a thin rod model posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_P(InertiaVisualizerGeometryTest, ThinRodTest) {
  const Vector3d pose = GetParam().transpose();
  // TODO(jwnimmer-tri) Our SDFormat parser erroneously rejects rods that are
  // posed too far away.  We need to skip those test cases for now.
  if (pose.maxCoeff() > 5) {
    drake::log()->info("Skipping test case due to SDFormat parser bug");
    return;
  }
  constexpr double length = 5.0;
  const auto M =
      SpatialInertia<double>::ThinRodWithMass(10, length, Vector3d(0, 0, 1));
  const GeometryTestCase test_case{
      .visual_sdf = fmt::format(
          "<cylinder><radius>0.01</radius><length>{}</length></cylinder>",
          length),
      .mass = M.get_mass(),
      .moment_ixx = M.CalcRotationalInertia().get_moments()[0],
      .moment_iyy = M.CalcRotationalInertia().get_moments()[1],
      .moment_izz = M.CalcRotationalInertia().get_moments()[2],
  };

  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(test_case);

  EXPECT_TRUE(CompareMatrices(transform.translation(), pose));

  const auto ellipsoid_M = SpatialInertia<double>::SolidEllipsoidWithDensity(
      kNominalDensity, ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
  EXPECT_NEAR(ellipsoid_M.get_mass(), M.get_mass(), kTolerance);

  // Check that the ellipsoid dimensions are roughly shaped like a rod
  // along the z-axis.
  SCOPED_TRACE(fmt::format("abc = {} {} {}", ellipsoid.a(), ellipsoid.b(),
                           ellipsoid.c()));
  EXPECT_NEAR(ellipsoid.a(), ellipsoid.b(), kTolerance);
  EXPECT_GT(ellipsoid.c(), ellipsoid.a() * 99);
}

INSTANTIATE_TEST_SUITE_P(Poses, InertiaVisualizerGeometryTest,
                         testing::Values(RowVector3d{0, 0, 0},
                                         RowVector3d{10, 20, 30},
                                         RowVector3d{5, 2, 1},
                                         RowVector3d{10, 200, 50}));

}  // namespace
}  // namespace internal
}  // namespace visualization
}  // namespace drake
