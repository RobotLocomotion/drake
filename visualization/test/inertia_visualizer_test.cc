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
// The CalculateInertiaGeometryFor() method returns the inertia ellipsoid
// and transform for a single-body model when passed a GeometryTestCase.
class InertiaVisualizerGeometryTest : public ::testing::Test {
 public:
  InertiaVisualizerGeometryTest()
      : builder_{},
        plant_and_scene_graph_{AddMultibodyPlantSceneGraph(&builder_, 0.001)},
        plant_{plant_and_scene_graph_.plant},
        scene_graph_{plant_and_scene_graph_.scene_graph},
        // The value of near_delta_ is chosen such that the same ellipsoid
        // radii can be used for each posed and unposed test.
        near_delta_{1e-10} {};

  struct GeometryTestCase {
    // TODO(trowell-tri) Update to a Vector6d when rpy is supported.
    Vector3d pose{Vector3d::Zero()};
    double mass{0};
    double moment_ixx{0};
    double moment_iyy{0};
    double moment_izz{0};
    // The visual_sdf_ should contain the visual element of the test fixture
    // to clarify the geometry being modeled.
    std::string visual_sdf;
  };

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

  // A value to pass as abs_error to EXPECT_NEAR() that is more expansive than
  // std::epsilon so we can ignore minor differences between posed and unposed
  // ellipsoid radii which are probably due to floating point limits.
  const double near_delta_;
};

// Test inertia geometry computations for an ellipsoid model posed without
// translation or rotation.
TEST_F(InertiaVisualizerGeometryTest, EllipsoidTest) {
  // Use an ellipsoid with the density of water.
  GeometryTestCase test_case{
      .mass = 25132.7412287,
      .moment_ixx = 65345.12719462,
      .moment_iyy = 50265.4824574,
      .moment_izz = 25132.7412287,
      .visual_sdf = "<ellipsoid><radii>1.0 2.0 3.0</radii></ellipsoid>"};

  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(test_case);

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  // Since our visualization ellipsoid also has the density of water,
  // it should have the same dimensions as the input.
  EXPECT_NEAR(ellipsoid.a(), 1.0, near_delta_);
  EXPECT_NEAR(ellipsoid.b(), 2.0, near_delta_);
  EXPECT_NEAR(ellipsoid.c(), 3.0, near_delta_);

  // Confirm that we get back our input values from a canonically-calculated
  // ellipsoid of the same density.
  auto inertia = SpatialInertia<double>::SolidEllipsoidWithDensity(
      1000, ellipsoid.a(), ellipsoid.b(), ellipsoid.c());

  EXPECT_DOUBLE_EQ(inertia.get_mass(), test_case.mass);

  const Vector3d unit_moments = inertia.get_unit_inertia().get_moments();
  const double moment_delta = 1e-7;
  EXPECT_NEAR(unit_moments(0) * test_case.mass, test_case.moment_ixx,
              moment_delta);
  EXPECT_NEAR(unit_moments(1) * test_case.mass, test_case.moment_iyy,
              moment_delta);
  EXPECT_NEAR(unit_moments(2) * test_case.mass, test_case.moment_izz,
              moment_delta);
}

// Test inertia geometry computations for an ellipsoid model posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerGeometryTest, PosedEllipsoidTest) {
  Vector3d pose{10, 200, 50};
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.pose = pose,
       .mass = 2000,
       .moment_ixx = 5200,
       .moment_iyy = 4000,
       .moment_izz = 2000,
       .visual_sdf = "<ellipsoid><radii>1.0 2.0 3.0</radii></ellipsoid>"});

  EXPECT_TRUE(CompareMatrices(transform.translation(), pose));

  EXPECT_NEAR(ellipsoid.a(), 0.4301270069140498, near_delta_);
  EXPECT_NEAR(ellipsoid.b(), 0.8602540138280997, near_delta_);
  EXPECT_NEAR(ellipsoid.c(), 1.2903810207421496, near_delta_);
}

// Test inertia geometry computations for a massless body posed without
// translation or rotation.
TEST_F(InertiaVisualizerGeometryTest, ZeroMassTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.visual_sdf = "<sphere><radius>1.0</radius></sphere>"});

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, near_delta_);
  EXPECT_NEAR(ellipsoid.b(), 0.001, near_delta_);
  EXPECT_NEAR(ellipsoid.c(), 0.001, near_delta_);
}

// Test inertia geometry computations for a massless body posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerGeometryTest, PosedZeroMassTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.pose = Vector3d{10, 20, 30},
       .visual_sdf = "<sphere><radius>1.0</radius></sphere>"});

  // Pose for zero inertia is ignored.
  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, near_delta_);
  EXPECT_NEAR(ellipsoid.b(), 0.001, near_delta_);
  EXPECT_NEAR(ellipsoid.c(), 0.001, near_delta_);
}

// Test inertia geometry computations for a box model posed without
// translation or rotation.
TEST_F(InertiaVisualizerGeometryTest, BoxTest) {
  // Use a box with the density of water.
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.mass = 6000,
       .moment_ixx = 6500,
       .moment_iyy = 5000,
       .moment_izz = 2500,
       .visual_sdf = "<box><size>1.0 2.0 3.0</size></box>"});

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  // The visualization ellipsoid of the same density is ~24% bigger in each
  // dimension (we have radii).
  const double factor = 1.24070098179;
  EXPECT_NEAR(ellipsoid.a() * 2.0, 1.0 * factor, near_delta_);
  EXPECT_NEAR(ellipsoid.b() * 2.0, 2.0 * factor, near_delta_);
  EXPECT_NEAR(ellipsoid.c() * 2.0, 3.0 * factor, near_delta_);
}

// Test inertia geometry computations for a box model posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerGeometryTest, PosedBoxTest) {
  const Vector3d pose{100, 200, 300};
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.pose = pose,
       .mass = 2000,
       .moment_ixx = 2166.66666,
       .moment_iyy = 1666.66666,
       .moment_izz = 833.333333,
       .visual_sdf = "<box><size>1.0 2.0 3.0</size></box>"});

  EXPECT_TRUE(CompareMatrices(transform.translation(), pose));

  EXPECT_NEAR(ellipsoid.a(), 0.4301270070992436, near_delta_);
  EXPECT_NEAR(ellipsoid.b(), 0.8602540145210817, near_delta_);
  EXPECT_NEAR(ellipsoid.c(), 1.2903810191470948, near_delta_);
}

// Test inertia geometry computations for a thin cylinder model posed without
// translation or rotation.
TEST_F(InertiaVisualizerGeometryTest, ThinCylinderTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.mass = 10,
       .moment_ixx = 20.8333333,
       .moment_iyy = 20.8333333,
       .moment_izz = 1e-3,
       .visual_sdf =
           "<cylinder><radius>0.01</radius><length>5.0</length></cylinder>"});

  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.022699035696719869, near_delta_);
  EXPECT_NEAR(ellipsoid.b(), 0.022699035696719869, near_delta_);
  EXPECT_NEAR(ellipsoid.c(), 4.6333656540865427, near_delta_);
}

// Test inertia geometry computations for a thin cylinder model posed with
// translation but not rotation.
// TODO(trowell-tri) Add rotation to the test when supported.
TEST_F(InertiaVisualizerGeometryTest, PosedThinCylinderTest) {
  const Vector3d pose{5, 2, 1};
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.pose = pose,
       .mass = 10,
       .moment_ixx = 20.8333333,
       .moment_iyy = 20.8333333,
       .moment_izz = 1e-3,
       .visual_sdf =
           "<cylinder><radius>0.01</radius><length>5.0</length></cylinder>"});

  EXPECT_TRUE(CompareMatrices(transform.translation(), pose));

  EXPECT_NEAR(ellipsoid.a(), 0.022699035696719869, near_delta_);
  EXPECT_NEAR(ellipsoid.b(), 0.022699035696719869, near_delta_);
  EXPECT_NEAR(ellipsoid.c(), 4.6333656540865427, near_delta_);
}

}  // namespace
}  // namespace internal
}  // namespace visualization
}  // namespace drake
