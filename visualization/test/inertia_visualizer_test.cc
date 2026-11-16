#include "drake/visualization/inertia_visualizer.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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
using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraphResult;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using Eigen::Vector3d;
using Matrix34d = Eigen::Matrix<double, 3, 4>;
using RowVector6d = Eigen::Matrix<double, 1, 6>;

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

// The visualizer registers one SceneGraph frame per body it draws an ellipsoid
// for, so a body is drawn iff a frame named for it turns up in SceneGraph.
bool IsDrawn(const MultibodyPlant<double>& plant,
             const SceneGraph<double>& scene_graph,
             const multibody::RigidBody<double>& body) {
  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph.model_inspector();
  const std::string name = fmt::format(
      "InertiaVisualizer::{}::{}",
      plant.GetModelInstanceName(body.model_instance()), body.name());
  const std::vector<geometry::FrameId> frame_ids = inspector.GetAllFrameIds();
  return std::any_of(frame_ids.begin(), frame_ids.end(),
                     [&](geometry::FrameId frame_id) {
                       return inspector.GetName(frame_id) == name;
                     });
}

// Anchored bodies (World and any body welded to it, directly or indirectly)
// can't move, so they get no inertia ellipsoid.
GTEST_TEST(InertiaVisualizerTest, AnchoredBodiesAreSkipped) {
  constexpr char kSdf[] = R"""(
  <?xml version="1.0"?>
  <sdf version="1.7">
    <model name="anchored">
      <link name="base"/>
      <link name="tool"/>
      <link name="free"/>
      <joint name="world_base" type="fixed">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <joint name="base_tool" type="fixed">
        <parent>base</parent>
        <child>tool</child>
      </joint>
    </model>
  </sdf>
  )""";
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.01);
  Parser(&plant).AddModelsFromString(kSdf, "sdf");
  plant.Finalize();

  const multibody::RigidBody<double>& base = plant.GetBodyByName("base");
  const multibody::RigidBody<double>& tool = plant.GetBodyByName("tool");
  const multibody::RigidBody<double>& free = plant.GetBodyByName("free");
  // Each of these has a SceneGraph frame, so the visualizer could have drawn
  // it.
  for (const auto* body : {&base, &tool, &free}) {
    ASSERT_TRUE(plant.GetBodyFrameIdIfExists(body->index()).has_value());
  }

  InertiaVisualizer<double>::AddToBuilder(&builder, plant, &scene_graph);

  EXPECT_FALSE(IsDrawn(plant, scene_graph, plant.world_body()));
  EXPECT_FALSE(IsDrawn(plant, scene_graph, base));
  EXPECT_FALSE(IsDrawn(plant, scene_graph, tool));
  EXPECT_TRUE(IsDrawn(plant, scene_graph, free));
}

// When MultibodyPlant models a closed kinematic loop it splits a link into a
// primary link and a "shadow" link, which has no mass properties of its own.
// Only the user's bodies get inertia ellipsoids, and the split link's shows its
// full mass.
GTEST_TEST(InertiaVisualizerTest, ShadowLinkIsSkipped) {
  // A minimal closed loop: two identical links joined to World and to each
  // other.
  constexpr char kLoopSdf[] = R"""(
  <?xml version="1.0"?>
  <sdf version="1.7">
    <model name="loop">
      <link name="upper"/>
      <link name="lower"/>
      <joint name="world_upper" type="revolute">
        <parent>world</parent>
        <child>upper</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="world_lower" type="revolute">
        <parent>world</parent>
        <child>lower</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="upper_lower" type="revolute">
        <parent>upper</parent>
        <child>lower</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
    </model>
  </sdf>
  )""";
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.01);
  plant.SetEnableLoopTopology(true);
  Parser(&plant).AddModelsFromString(kLoopSdf, "sdf");
  plant.Finalize();

  // Breaking the loop added a shadow body, after the user's bodies. It has a
  // SceneGraph frame of its own, so the visualizer could have drawn it.
  ASSERT_EQ(plant.num_loop_constraints(), 1);
  ASSERT_EQ(plant.num_bodies(), plant.num_user_bodies() + 1);
  const multibody::BodyIndex shadow_index(plant.num_bodies() - 1);
  const multibody::RigidBody<double>& shadow = plant.get_body(shadow_index);
  ASSERT_TRUE(shadow.is_ephemeral());
  ASSERT_TRUE(plant.GetBodyFrameIdIfExists(shadow_index).has_value());

  InertiaVisualizer<double>::AddToBuilder(&builder, plant, &scene_graph);

  const multibody::RigidBody<double>& upper = plant.GetBodyByName("upper");
  const multibody::RigidBody<double>& lower = plant.GetBodyByName("lower");
  EXPECT_TRUE(IsDrawn(plant, scene_graph, upper));
  EXPECT_TRUE(IsDrawn(plant, scene_graph, lower));
  EXPECT_FALSE(IsDrawn(plant, scene_graph, shadow));

  // The two links have the same (default) mass properties, so the one that was
  // split gets the same ellipsoid as the one that wasn't, showing its full
  // mass.
  const auto context = plant.CreateDefaultContext();
  const auto [upper_ellipsoid, X_upper] =
      CalculateInertiaGeometry(upper, *context);
  const auto [lower_ellipsoid, X_lower] =
      CalculateInertiaGeometry(lower, *context);
  EXPECT_EQ(upper_ellipsoid.a(), lower_ellipsoid.a());
  EXPECT_EQ(upper_ellipsoid.b(), lower_ellipsoid.b());
  EXPECT_EQ(upper_ellipsoid.c(), lower_ellipsoid.c());
}

// This class allows testing the results of inertia geometry calculation.
//
// The CalculateInertiaGeometryFor() method returns the inertia ellipsoid
// and transform for a single-body model when passed a GeometryTestCase.
//
// The RowVector6d test parameter indicates an xyzrpy pose for the inertia.
class InertiaVisualizerGeometryTest
    : public ::testing::TestWithParam<RowVector6d> {
 public:
  InertiaVisualizerGeometryTest()
      : builder_{},
        plant_and_scene_graph_{AddMultibodyPlantSceneGraph(&builder_, 0.001)},
        plant_{plant_and_scene_graph_.plant},
        scene_graph_{plant_and_scene_graph_.scene_graph} {};

  struct GeometryTestCase {
    // The visual_sdf should contain the visual element of the test fixture
    // to clarify the geometry being modeled.
    std::string visual_sdf;
    double mass{0};
    double moment_ixx{0};
    double moment_iyy{0};
    double moment_izz{0};
  };

  // Returns the value of CalculateInertiaGeometry for the given input.
  std::pair<geometry::Ellipsoid, RigidTransform<double>>
  CalculateInertiaGeometryFor(const GeometryTestCase& input,
                              MultibodyPlant<double>* plant = nullptr) {
    static constexpr char kModelTemplate[] = R"""(
          <?xml version='1.0'?>
          <sdf version='1.7'>
            <model name='test_model'>
              <link name='test_body'>
                <inertial>
                  <mass>{mass}</mass>
                  <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
                  <inertia>
                    <ixx>{ixx}</ixx>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyy>{iyy}</iyy>
                    <iyz>0</iyz>
                    <izz>{izz}</izz>
                  </inertia>
                </inertial>
                <visual name="test_visual">
                  <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
                  <geometry>
                    {visual}
                  </geometry>
                </visual>
              </link>
            </model>
          </sdf>
          )""";
    const std::string model = fmt::format(
        kModelTemplate, fmt::arg("mass", input.mass),
        fmt::arg("ixx", input.moment_ixx), fmt::arg("iyy", input.moment_iyy),
        fmt::arg("izz", input.moment_izz), fmt::arg("visual", input.visual_sdf),
        fmt::arg("x", xyz().x()), fmt::arg("y", xyz().y()),
        fmt::arg("z", xyz().z()), fmt::arg("roll", rpy()(0)),
        fmt::arg("pitch", rpy()(1)), fmt::arg("yaw", rpy()(2)));
    drake::log()->info("Using SDF model: {}", model);

    if (plant == nullptr) {
      plant = &plant_;
    }
    multibody::Parser parser(plant);
    parser.SetStrictParsing();
    parser.AddModelsFromString(model, "sdf");
    plant->Finalize();
    const multibody::RigidBody<double>& body =
        plant->GetBodyByName("test_body");
    return CalculateInertiaGeometry(body, *plant->CreateDefaultContext());
  }

  /* Returns the translation (xyz) vector for this test case. */
  Vector3d xyz() const { return GetParam().transpose().head(3); }

  /* Returns the rotation (rpy) vector for this test case (in radians). */
  Vector3d rpy() const { return GetParam().transpose().tail(3); }

  /* Returns a canonical ellipsoid rpy vector for the given transform's rotation
  matrix, in the sense that any two rotations that look the same on the screen
  when applied to an ellipsoid will be mapped to the same value here. In short,
  we need to handle the 180 degree symmetry of an ellipsoid, and we'll do that
  by flipping all three angles to be nonnegative. N.B. This does _not_ handle
  the symmetry in case two or more axes of the ellipse have the same value,
  i.e., we assume that ellipsoid axes (a, b, c) are all distinct values (we
  assume the ellipsoid is not a sphere or spheroid). */
  static Vector3d GetCanonicalRpy(const RigidTransform<double>& X) {
    const RotationMatrix<double>& R = X.rotation();
    // TODO(jwnimmer-tri) There's probably a more direct way to calculate this.
    for (double flip_r : {0.0, M_PI}) {
      for (double flip_p : {0.0, M_PI}) {
        for (double flip_y : {0.0, M_PI}) {
          const RotationMatrix<double> flips(
              RollPitchYaw<double>(flip_r, flip_p, flip_y));
          const Vector3d result = RollPitchYaw<double>(R * flips).vector();
          if (result.minCoeff() >= 0.0) {
            return result;
          }
        }
      }
    }
    DRAKE_UNREACHABLE();
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
TEST_P(InertiaVisualizerGeometryTest, EllipsoidTest) {
  // We must use distinct values for all of a,b,c not only to achieve sufficient
  // test coverage but also because GetCanonicalRpy doesn't handle certain kinds
  // of symmetries.
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

  EXPECT_TRUE(CompareMatrices(transform.translation(), xyz(), kTolerance));
  EXPECT_TRUE(CompareMatrices(GetCanonicalRpy(transform), rpy(), kTolerance));

  EXPECT_NEAR(ellipsoid.a(), a, kTolerance);
  EXPECT_NEAR(ellipsoid.b(), b, kTolerance);
  EXPECT_NEAR(ellipsoid.c(), c, kTolerance);
}

// Test inertia geometry computations for a massless body posed with
// translation but not rotation.
TEST_P(InertiaVisualizerGeometryTest, ZeroMassTest) {
  auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
      {.visual_sdf = "<sphere><radius>1.0</radius></sphere>"});

  // Pose for zero inertia is ignored.
  EXPECT_EQ(transform.IsExactlyIdentity(), true);

  EXPECT_NEAR(ellipsoid.a(), 0.001, kTolerance);
  EXPECT_NEAR(ellipsoid.b(), 0.001, kTolerance);
  EXPECT_NEAR(ellipsoid.c(), 0.001, kTolerance);
}

// Test inertia geometry computations for a point-mass body. It should always
// put a sphere at the center of mass (as defined by xyz()). The volume of the
// sphere is proportional to the mass (such that 1 kg -> 1 cm radius).
TEST_P(InertiaVisualizerGeometryTest, PointMass) {
  // The volume of a 6.2 cm radius sphere (the size for a 1-kg point mass).
  const double reference_volume = (4.0 / 3.0) * M_PI * 0.062 * 0.062 * 0.062;
  const double reference_mass = 1.0;  // kg
  for (const double& mass : {0.125, 1.0, 8.0}) {
    SCOPED_TRACE(fmt::format("Testing mass = {}", mass));
    // The test harness only allows one call to CalculateInertiaGeometryFor per
    // test (when using its built-in plant). So, we'll construct a new plant
    // for each iteration.
    systems::DiagramBuilder<double> builder;
    auto [plant, _] = AddMultibodyPlantSceneGraph(&builder, 0.001);
    auto [ellipsoid, transform] = CalculateInertiaGeometryFor(
        {.mass = mass, .moment_ixx = 0, .moment_iyy = 0, .moment_izz = 0},
        &plant);

    // Pose is strictly the position of the center of mass (R = I).
    EXPECT_TRUE(transform.IsExactlyEqualTo(RigidTransform<double>(xyz())));

    // The ratio of sphere volume to reference volume should be the same as the
    // ratio of the mass to the reference mass (1 kg).
    const double volume = CalcVolume(ellipsoid);
    EXPECT_DOUBLE_EQ(volume / reference_volume, mass / reference_mass);

    // It's a *sphere*.
    EXPECT_EQ(ellipsoid.a(), ellipsoid.b());
    EXPECT_EQ(ellipsoid.a(), ellipsoid.c());
  }
}

// Test that the inertia visualization ellipsoid for a box that uses the same
// nominal density as those ellipsoids has spatial inertia moments in the same
// ratio as those of the box. This test has translation but no rotation.
TEST_P(InertiaVisualizerGeometryTest, BoxTest) {
  // We must use distinct values for all of x,y,z not only to achieve sufficient
  // test coverage but also because GetCanonicalRpy doesn't handle certain kinds
  // of symmetries.
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

  EXPECT_TRUE(CompareMatrices(transform.translation(), xyz(), kTolerance));
  EXPECT_TRUE(CompareMatrices(GetCanonicalRpy(transform), rpy(), kTolerance));

  const auto ellipsoid_M = SpatialInertia<double>::SolidEllipsoidWithDensity(
      kNominalDensity, ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
  EXPECT_NEAR(ellipsoid_M.get_mass(), M.get_mass(), kTolerance);

  // Check that the moments of the box and the ellipsoid are proportional to
  // each other and the scale factor isn't too wide.
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

// Test inertia geometry computations for a thin rod model.
TEST_P(InertiaVisualizerGeometryTest, ThinRodTest) {
  constexpr double length = 5.0;
  const auto M =
      SpatialInertia<double>::ThinRodWithMass(10, length, Vector3d(1, 0, 0));
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

  EXPECT_TRUE(CompareMatrices(transform.translation(), xyz(), kTolerance));
  // The x-axis thin rod is symmetric in roll; only compare pitch and yaw.
  EXPECT_TRUE(CompareMatrices(GetCanonicalRpy(transform).tail(2), rpy().tail(2),
                              kTolerance));

  const auto ellipsoid_M = SpatialInertia<double>::SolidEllipsoidWithDensity(
      kNominalDensity, ellipsoid.a(), ellipsoid.b(), ellipsoid.c());
  EXPECT_NEAR(ellipsoid_M.get_mass(), M.get_mass(), kTolerance);

  // Check that the ellipsoid dimensions are roughly shaped like a rod
  // along the x-axis.
  SCOPED_TRACE(fmt::format("abc = {} {} {}", ellipsoid.a(), ellipsoid.b(),
                           ellipsoid.c()));
  EXPECT_NEAR(ellipsoid.b(), ellipsoid.c(), kTolerance);
  EXPECT_GT(ellipsoid.a(), ellipsoid.b() * 99);
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(Poses, InertiaVisualizerGeometryTest, testing::Values(
  //                 x,   y,  z,   r,   p,   y
  (RowVector6d() <<  0,   0,  0,   0,   0,   0).finished(),
  (RowVector6d() << 10,  20, 30,   0,   0,   0).finished(),
  (RowVector6d() <<  5,   2,  1,   0,   0,   0).finished(),
  (RowVector6d() << 10, 200, 50,   0,   0,   0).finished(),
  (RowVector6d() <<  0,   0,  0, 0.1,   0,   0).finished(),
  (RowVector6d() <<  0,   0,  0,   0, 0.1,   0).finished(),
  (RowVector6d() <<  0,   0,  0,   0,   0, 0.1).finished(),
  (RowVector6d() <<  0,   0,  0, 0.1, 0.2, 0.3).finished(),
  (RowVector6d() <<  5,   2,  1, 0.5, 0.2, 0.1).finished(),
  (RowVector6d() << 10, 200, 50, 0.5, 0.2, 0.1).finished()
));
// clang-format on

}  // namespace
}  // namespace internal
}  // namespace visualization
}  // namespace drake
