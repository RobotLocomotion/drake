#include "drake/multibody/parsing/detail_mujoco_parser.h"

#include <filesystem>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using ::testing::MatchesRegex;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using drake::internal::DiagnosticPolicy;
using geometry::FrameId;
using geometry::GeometryId;
using geometry::Role;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

const double kInf = std::numeric_limits<double>::infinity();


class MujocoParserTest : public test::DiagnosticPolicyTestBase {
 public:
  MujocoParserTest() {
    plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
  }

  std::optional<ModelInstanceIndex> AddModelFromFile(
      const std::string& file_name,
      const std::string& model_name) {
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       &plant_, &resolver, NoSelect};
    auto result = wrapper_.AddModel(
        {DataSource::kFilename, &file_name}, model_name, {}, w);
    resolver.Resolve(diagnostic_policy_);
    return result;
  }

  std::optional<ModelInstanceIndex> AddModelFromString(
      const std::string& file_contents,
      const std::string& model_name) {
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       &plant_, &resolver, NoSelect};
    auto result = wrapper_.AddModel(
        {DataSource::kContents, &file_contents}, model_name, {}, w);
    resolver.Resolve(diagnostic_policy_);
    return result;
  }

  std::vector<ModelInstanceIndex> AddAllModelsFromFile(
      const std::string& file_name,
      const std::optional<std::string>& parent_model_name) {
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       &plant_, &resolver, NoSelect};
    auto result = wrapper_.AddAllModels(
        {DataSource::kFilename, &file_name}, parent_model_name, w);
    resolver.Resolve(diagnostic_policy_);
    return result;
  }

  std::vector<ModelInstanceIndex> AddAllModelsFromString(
      const std::string& file_contents,
      const std::optional<std::string>& parent_model_name) {
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       &plant_, &resolver, NoSelect};
    auto result = wrapper_.AddAllModels(
        {DataSource::kContents, &file_contents}, parent_model_name, w);
    resolver.Resolve(diagnostic_policy_);
    return result;
  }

  // Mujoco cannot delegate to any other parsers.
  static ParserInterface& NoSelect(
      const drake::internal::DiagnosticPolicy&, const std::string&) {
    DRAKE_UNREACHABLE();
  }

 protected:
  ParsingOptions options_;
  PackageMap package_map_;
  MultibodyPlant<double> plant_{0.1};
  SceneGraph<double> scene_graph_;
  MujocoParserWrapper wrapper_;

  std::string box_obj_{std::filesystem::canonical(FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj"))};
  std::string non_convex_obj_{std::filesystem::canonical(FindResourceOrThrow(
      "drake/geometry/test/non_convex_mesh.obj"))};
  std::string box_urdf_{std::filesystem::canonical(FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/urdfs/box.urdf"))};
};

class DeepMindControlTest : public MujocoParserTest,
                            public testing::WithParamInterface<const char*> {};

TEST_P(DeepMindControlTest, DeepMindControl) {
  // Confirm successful parsing of the MuJoCo models in the DeepMind control
  // suite.
  std::string model{GetParam()};
  const std::string filename = FindResourceOrThrow(
      fmt::format("drake/multibody/parsing/dm_control/suite/{}.xml", model));
  AddModelFromFile(filename, model);

  EXPECT_TRUE(plant_.HasModelInstanceNamed(model));

  // For this test, ignore all warnings.
  warning_records_.clear();
}

const char* dm_control_models[] = {
    "acrobot",  "cartpole",   "cheetah",      "finger",  "fish",
    "hopper",   "humanoid",   "humanoid_CMU", "lqr",     "manipulator",
    "pendulum", "point_mass", "quadruped",    "reacher", "stacker",
    "swimmer",  "walker"};
INSTANTIATE_TEST_SUITE_P(DeepMindControl, DeepMindControlTest,
                         testing::ValuesIn(dm_control_models));

class MujocoMenagerieTest : public MujocoParserTest,
                            public testing::WithParamInterface<const char*> {};

TEST_P(MujocoMenagerieTest, MujocoMenagerie) {
  // Confirm successful parsing of the MuJoCo models in the DeepMind control
  // suite.
  std::string model{GetParam()};
  const std::string filename = FindResourceOrThrow(
      fmt::format("drake/multibody/parsing/mujoco_menagerie/{}.xml", model));
  AddModelFromFile(filename, model);

  EXPECT_TRUE(plant_.HasModelInstanceNamed(model));

  // For this test, ignore all warnings.
  warning_records_.clear();
}

const char* mujoco_menagerie_models[] = {"google_robot/robot",
                                         "kuka_iiwa_14/iiwa14"};
// TODO(russt): Add the remaining models, once they can be parsed correctly, as
// tracked in #20444.

INSTANTIATE_TEST_SUITE_P(MujocoMenagerie, MujocoMenagerieTest,
                         testing::ValuesIn(mujoco_menagerie_models));

// In addition to confirming that the parser can successfully parse the model,
// this test can be used to manually inspect the resulting visualization.
GTEST_TEST(MujocoParserExtraTest, Visualize) {
  systems::DiagramBuilder<double> builder;
  std::shared_ptr<geometry::Meshcat> meshcat =
      geometry::GetTestEnvironmentMeshcat();
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);

  ParsingOptions options;
  PackageMap package_map;
  MujocoParserWrapper wrapper;
  internal::CollisionFilterGroupResolver resolver{&plant};
  internal::DiagnosticPolicy diagnostic_policy;
  ParsingWorkspace w{
      options,
      package_map,
      diagnostic_policy,
      &plant,
      &resolver,
      [](const drake::internal::DiagnosticPolicy&,
         const std::string&) -> drake::multibody::internal::ParserInterface& {
        DRAKE_UNREACHABLE();
      }};
  const std::string model_name = "quadruped";
  const std::string filename = FindResourceOrThrow(fmt::format(
      "drake/multibody/parsing/dm_control/suite/{}.xml", model_name));
  wrapper.AddModel({DataSource::kFilename, &filename}, model_name, {}, w);
  resolver.Resolve(diagnostic_policy);
  plant.Finalize();
  visualization::AddDefaultVisualization(&builder, meshcat);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  diagram->ExecuteInitializationEvents(context.get());
  diagram->ForcedPublish(*context);
  common::MaybePauseForUser();
}

TEST_F(MujocoParserTest, CartPole) {
  const std::string filename = FindResourceOrThrow(
      "drake/multibody/parsing/dm_control/suite/cartpole.xml");
  AddModelFromFile(filename, "cartpole");
  // For this parse, ignore all warnings.
  warning_records_.clear();

  plant_.Finalize();
  // Check the kinematics. Passing this test requires a correct parsing of
  // joint defaults.
  auto context = plant_.CreateDefaultContext();
  const double x = 0.1;
  const double theta = 0.2;
  const double l = 1.0;
  const double z_offset_from_model = 1.0;
  plant_.SetPositions(context.get(), Vector2d(x, theta));
  Vector3d p_WP;
  plant_.CalcPointsPositions(*context, plant_.GetFrameByName("pole_1"),
                             Vector3d{0, 0, l}, plant_.world_frame(),
                             &p_WP);
  EXPECT_TRUE(CompareMatrices(
      p_WP,
      Vector3d{x + l * sin(theta), 0, z_offset_from_model + l * cos(theta)},
      1e-14));
}

TEST_F(MujocoParserTest, Acrobot) {
  const std::string filename = FindResourceOrThrow(
      "drake/multibody/parsing/dm_control/suite/acrobot.xml");
  AddModelFromFile(filename, "acrobot");
  // For this parse, ignore all warnings.
  warning_records_.clear();

  plant_.Finalize();
  // Check the kinematics. Passing this test requires a correct parsing of the
  // joint position being defined in the child body frame, not the parent
  // body frame.
  auto context = plant_.CreateDefaultContext();
  const Vector2d q = {0.1, 0.2};
  const double l1 = 1.0;
  const double l2 = 0.5;
  const double z_offset_from_model = 2.0;
  plant_.SetPositions(context.get(), q);
  Vector3d p_WP;
  plant_.CalcPointsPositions(*context, plant_.GetFrameByName("lower_arm"),
                             Vector3d{0, 0, l2}, plant_.world_frame(),
                             &p_WP);
  EXPECT_TRUE(CompareMatrices(
      p_WP,
      Vector3d{l1 * sin(q[0]) + l2 * sin(q[0] + q[1]), 0,
               z_offset_from_model + l1 * cos(q[0]) + l2 * cos(q[0] + q[1])},
      1e-14));
}

TEST_F(MujocoParserTest, Option) {
  std::string xml = R"""(
<mujoco model="test">
  <option gravity="0 -9.81 0"/>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_TRUE(CompareMatrices(plant_.gravity_field().gravity_vector(),
                              Vector3d{0, -9.81, 0}));
}

TEST_F(MujocoParserTest, EmptyString) {
  EXPECT_TRUE(AddAllModelsFromString("", {}).empty());
  EXPECT_THAT(TakeError(), MatchesRegex(".*XML_ERROR_EMPTY_DOCUMENT.*"));
}

TEST_F(MujocoParserTest, EmptyFile) {
  EXPECT_TRUE(AddAllModelsFromFile("/dev/null", {}).empty());
  EXPECT_THAT(TakeError(), MatchesRegex(".*XML_ERROR_EMPTY_DOCUMENT.*"));
}

TEST_F(MujocoParserTest, NoMujocoRoot) {
  std::string xml = "<stuff/>";
  EXPECT_TRUE(AddAllModelsFromString(xml, {}).empty());
  EXPECT_THAT(TakeError(), MatchesRegex(".*not.*mujoco tag.*"));
}

TEST_F(MujocoParserTest, NoName) {
  std::string xml = "<mujoco/>";
  EXPECT_TRUE(AddAllModelsFromString(xml, {}).empty());
  EXPECT_THAT(TakeError(), MatchesRegex(".*robot must have a name.*"));
}

TEST_F(MujocoParserTest, GeometryTypes) {
  std::string xml = R"""(
<mujoco model="test">
  <default class="default_box">
    <geom type="box" size="0.1 0.2 0.3"/>
    <default class="sub">
      <geom size="0.4 0.5 0.6"/>
    </default>
  </default>
  <worldbody>
    <geom name="default" size="0.1"/>
    <geom name="plane" type="plane"/>
    <geom name="sphere" type="sphere" size="0.1"/>
    <geom name="capsule" type="capsule" size="0.1 2.0"/>
    <geom name="ellipsoid" type="ellipsoid" size="0.1 0.2 0.3"/>
    <geom name="cylinder" type="cylinder" size="0.1 2.0"/>
    <geom name="box" type="box" size="0.1 2.0 3.0"/>
    <geom name="box_from_default" class="default_box"/>
    <geom name="ellipsoid_from_default" type="ellipsoid" class="default_box"/>
    <geom name="box_from_sub" class="sub"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  const SceneGraphInspector<double>& inspector = scene_graph_.model_inspector();

  auto CheckShape = [&inspector](const std::string& geometry_name,
                                 std::string_view shape_type) {
    GeometryId geom_id = inspector.GetGeometryIdByName(
        inspector.world_frame_id(), Role::kProximity, geometry_name);
    EXPECT_EQ(inspector.GetShape(geom_id).type_name(), shape_type);
    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kPerception, geometry_name);
    EXPECT_EQ(inspector.GetShape(geom_id).type_name(), shape_type);
    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kIllustration, geometry_name);
    EXPECT_EQ(inspector.GetShape(geom_id).type_name(), shape_type);
  };

  // TODO(russt): Check the sizes of the shapes.  (It seems that none of our
  // existing parser tests actually check the sizes of of the shapes; the Reify
  // workflow makes it painful.)  Note that we do have some coverage from this
  // by testing for inertias set from the geometry below.

  CheckShape("default", "Sphere");
  CheckShape("plane", "HalfSpace");
  CheckShape("sphere", "Sphere");
  CheckShape("capsule", "Capsule");
  CheckShape("ellipsoid", "Ellipsoid");
  CheckShape("cylinder", "Cylinder");
  CheckShape("box", "Box");
  CheckShape("box_from_default", "Box");
  CheckShape("ellipsoid_from_default", "Ellipsoid");
  CheckShape("box_from_sub", "Box");
}

// Confirm that multiple instances of the same geometry defaults get unique
// names.
TEST_F(MujocoParserTest, UniqueGeometryNames) {
  std::string xml = R"""(
<mujoco model="test">
  <default class="default_box">
    <geom type="box" size="0.1 0.2 0.3"/>
  </default>
  <worldbody>
    <geom class="default_box"/>
    <geom class="default_box"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  const SceneGraphInspector<double>& inspector = scene_graph_.model_inspector();
  EXPECT_NO_THROW(inspector.GetGeometryIdByName(
      inspector.world_frame_id(), Role::kProximity, "geom0"));
  EXPECT_NO_THROW(inspector.GetGeometryIdByName(
      inspector.world_frame_id(), Role::kProximity, "geom1"));
}

TEST_F(MujocoParserTest, UnrecognizedGeometryTypes) {
  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <geom name="garbage" type="garbage"/>
  </worldbody>
</mujoco>
)""";
  AddModelFromString(xml, "test_unrecognized");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Unrecognized.*"));
}

TEST_F(MujocoParserTest, DefaultError) {
  std::string xml = R"""(
<mujoco model="test">
  <default class="default_box">
    <geom type="box" size="0.1 0.2 0.3"/>
    <default/>
  </default>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeError(), MatchesRegex(".*class.*attribute is required.*"));
}

TEST_F(MujocoParserTest, GeometryPose) {
  // By default, angles are in degrees.
  std::string xml = R"""(
<mujoco model="test">
  <compiler eulerseq="XYZ"/>
  <default class="default_pose">
    <geom pos="1 2 3" quat="0 1 0 0"/>
  </default>
  <worldbody>
    <geom name="identity" type="sphere" size="0.1" />
    <geom name="quat" quat="0 1 0 0" pos="1 2 3" type="sphere" size="0.1" />
    <geom name="axisangle" axisangle="4 5 6 30" pos="1 2 3" type="sphere"
          size="0.1" />
    <geom name="euler" euler="30 45 60" pos="1 2 3" type="sphere" size="0.1" />
    <geom name="xyaxes" xyaxes="0 1 0 -1 0 0" pos="1 2 3" type="sphere"
          size="0.1" />
    <geom name="zaxis" zaxis="0 1 0" pos="1 2 3" type="sphere" size="0.1" />
    <geom name="fromto_capsule" fromto="-1 -3 -3 -1 -1 -3"
          quat="0.2 0.4 0.3 .1" pos="1 2 3" type="capsule" size="0.1" />
    <geom name="fromto_cylinder" fromto="-1 -3 -3 -1 -1 -3"
          quat="0.2 0.4 0.3 .1" pos="1 2 3" type="cylinder" size="0.1" />
    <geom name="from_default" type="sphere" size="0.1" class="default_pose" />
  </worldbody>
</mujoco>
)""";

  // Explicitly set radians. Eulerseq is default "xyz".
  std::string radians_xml = R"""(
<mujoco model="test">
  <compiler angle="radian"/>
  <worldbody>
    <geom name="axisangle_rad" axisangle="4 5 6 0.5" pos="1 2 3" type="sphere"
          size="0.1" />
    <geom name="euler_rad" euler="0.5 0.7 1.05" pos="1 2 3" type="sphere"
          size="0.1" />
  </worldbody>
</mujoco>
)""";

  // Explicitly set degrees. Eulerseq is "ZYZ".
  std::string degrees_xml = R"""(
<mujoco model="test">
  <compiler angle="degree" eulerseq="ZYZ"/>
  <worldbody>
    <geom name="axisangle_deg" axisangle="4 5 6 30" pos="1 2 3" type="sphere"
          size="0.1" />
    <geom name="euler_deg" euler="30 45 60" pos="1 2 3" type="sphere"
          size="0.1" />
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  AddModelFromString(radians_xml, "radians_test");
  AddModelFromString(degrees_xml, "degrees_test");

  const SceneGraphInspector<double>& inspector = scene_graph_.model_inspector();

  auto CheckPose = [&inspector](const std::string& geometry_name,
                                const RigidTransformd& X_FG) {
    GeometryId geom_id = inspector.GetGeometryIdByName(
        inspector.world_frame_id(), Role::kProximity, geometry_name);
    EXPECT_TRUE(inspector.GetPoseInFrame(geom_id).IsNearlyEqualTo(X_FG, 1e-14))
        << geometry_name;
    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kPerception, geometry_name);
    EXPECT_TRUE(inspector.GetPoseInFrame(geom_id).IsNearlyEqualTo(X_FG, 1e-14))
        << geometry_name;
    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kIllustration, geometry_name);
    EXPECT_TRUE(inspector.GetPoseInFrame(geom_id).IsNearlyEqualTo(X_FG, 1e-14))
        << geometry_name;
  };

  const Vector3d p{1, 2, 3};
  CheckPose("identity", RigidTransformd());
  CheckPose("quat", RigidTransformd(Eigen::Quaternion<double>{0, 1, 0, 0}, p));
  CheckPose("axisangle",
            RigidTransformd(
                Eigen::AngleAxis<double>(M_PI / 6.0, Vector3d{4, 5, 6}), p));
  CheckPose("euler", RigidTransformd(
                         RollPitchYawd{M_PI / 6.0, M_PI / 4.0, M_PI / 3.0}, p));
  CheckPose("xyaxes",
            RigidTransformd(RotationMatrixd::MakeZRotation(M_PI / 2.0), p));
  CheckPose("zaxis",
            RigidTransformd(RotationMatrixd::MakeXRotation(-M_PI / 2.0), p));
  CheckPose("fromto_capsule",
            RigidTransformd(RotationMatrixd::MakeXRotation(-M_PI / 2.0), -p));
  CheckPose("fromto_cylinder",
            RigidTransformd(RotationMatrixd::MakeXRotation(-M_PI / 2.0), -p));
  CheckPose("from_default",
            RigidTransformd(Eigen::Quaternion<double>{0, 1, 0, 0}, p));

  CheckPose(
      "axisangle_rad",
      RigidTransformd(Eigen::AngleAxis<double>(0.5, Vector3d{4, 5, 6}), p));
  CheckPose("euler_rad",
            RigidTransformd(RotationMatrixd::MakeXRotation(0.5) *
                                RotationMatrixd::MakeYRotation(0.7) *
                                RotationMatrixd::MakeZRotation(1.05),
                            p));
  CheckPose("axisangle_deg",
            RigidTransformd(
                Eigen::AngleAxis<double>(M_PI / 6.0, Vector3d{4, 5, 6}), p));
  CheckPose("euler_deg",
            RigidTransformd(RotationMatrixd::MakeZRotation(M_PI / 3.0) *
                                RotationMatrixd::MakeYRotation(M_PI / 4.0) *
                                RotationMatrixd::MakeZRotation(M_PI / 6.0),
                            p));
}



TEST_F(MujocoParserTest, GeometryPoseErrors) {
  const std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <geom name="two_angles" type="sphere" size="0.1" pos="1 2 3"
          quat="0 1 0 0" euler="30 45 60" />
  </worldbody>
</mujoco>
)""";
  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*has more than one orientation attribute specified.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*has more than one orientation attribute specified.*"));
}

TEST_F(MujocoParserTest, GeometryProperties) {
  std::string xml = R"""(
<mujoco model="test">
  <asset>
    <material name="Orange" rgba="1 0.5 0 1"/>
  </asset>
  <default class="default_rgba">
    <geom rgba="0 1 0 1"/>
  </default>
  <worldbody>
    <geom name="default" type="sphere" size="0.1"/>
    <geom name="default_rgba" type="sphere" size="0.1" class="default_rgba"/>
    <geom name="sphere" type="sphere" size="0.1" friction="0.9 0.0 0.0"
          rgba="1 0 0 1"/>
    <geom name="orange" type="sphere" size="0.1" material="Orange"/>
    <geom name="red" type="sphere" size="0.1" material="Orange" rgba="1 0 0 1"/>
    <geom name="green" type="sphere" size="0.1" material="Orange"
          class="default_rgba"/>
    <geom name="no_collision" contype="0" conaffinity="0" />
    <geom name="no_visual" size="0.1" group="3" />
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");

  const SceneGraphInspector<double>& inspector = scene_graph_.model_inspector();

  auto CheckProperties = [&inspector](const std::string& geometry_name,
                                      double mu, const Vector4d& rgba,
                                      bool has_collision = true,
                                      bool has_visual = true) {
    if (has_collision) {
      GeometryId geom_id = inspector.GetGeometryIdByName(
          inspector.world_frame_id(), Role::kProximity, geometry_name);
      const geometry::ProximityProperties* proximity_prop =
          inspector.GetProximityProperties(geom_id);
      EXPECT_TRUE(proximity_prop);
      EXPECT_TRUE(proximity_prop->HasProperty("material", "coulomb_friction"));
      const auto& friction =
          proximity_prop->GetProperty<CoulombFriction<double>>(
              "material", "coulomb_friction");
      EXPECT_EQ(friction.static_friction(), mu);
      EXPECT_EQ(friction.dynamic_friction(), mu);
    } else {
      DRAKE_EXPECT_THROWS_MESSAGE(
          inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                        Role::kProximity, geometry_name),
          ".*has no geometry.*");
    }

    if (has_visual) {
      GeometryId geom_id = inspector.GetGeometryIdByName(
          inspector.world_frame_id(), Role::kPerception, geometry_name);
      const geometry::PerceptionProperties* perception_prop =
          inspector.GetPerceptionProperties(geom_id);
      EXPECT_TRUE(perception_prop);
      EXPECT_TRUE(perception_prop->HasProperty("phong", "diffuse"));
      EXPECT_TRUE(CompareMatrices(
          perception_prop->GetProperty<Vector4d>("phong", "diffuse"), rgba));
    } else {
      DRAKE_EXPECT_THROWS_MESSAGE(
          inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                        Role::kPerception, geometry_name),
          ".*has no geometry.*");
    }

    if (has_visual) {
      GeometryId geom_id = inspector.GetGeometryIdByName(
          inspector.world_frame_id(), Role::kIllustration, geometry_name);
      const geometry::IllustrationProperties* illustration_prop =
          inspector.GetIllustrationProperties(geom_id);
      EXPECT_TRUE(illustration_prop);
      EXPECT_TRUE(illustration_prop->HasProperty("phong", "diffuse"));
      EXPECT_TRUE(CompareMatrices(
          illustration_prop->GetProperty<Vector4d>("phong", "diffuse"), rgba));
    } else {
      DRAKE_EXPECT_THROWS_MESSAGE(
          inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                        Role::kIllustration, geometry_name),
          ".*has no geometry.*");
    }
  };

  CheckProperties("default", 1.0, Vector4d{.5, .5, .5, 1});
  CheckProperties("default_rgba", 1.0, Vector4d{0, 1, 0, 1});
  CheckProperties("sphere", 0.9, Vector4d{1, 0, 0, 1});
  CheckProperties("orange", 1.0, Vector4d{1, 0.5, 0, 1});
  // If both material and rgba are specified, rgba wins.
  CheckProperties("red", 1.0, Vector4d{1, 0, 0, 1});
  CheckProperties("green", 1.0, Vector4d{0, 1, 0, 1});
  CheckProperties("no_collision", 1.0, Vector4d{.5, .5, .5, 1},
                  false /* has collision */, true /* has visual */);
  CheckProperties("no_visual", 1.0, Vector4d{.5, .5, .5, 1},
                  true /* has collision */, false /* has visual */);

  // Confirm that the default geometry is a zero-radius sphere.
  GeometryId no_collision_id = inspector.GetGeometryIdByName(
      inspector.world_frame_id(), Role::kIllustration, "no_collision");
  const geometry::Sphere* no_collision_shape =
      dynamic_cast<const geometry::Sphere*>(
          &inspector.GetShape(no_collision_id));
  ASSERT_NE(no_collision_shape, nullptr);
  EXPECT_EQ(no_collision_shape->radius(), 0.0);
}

TEST_F(MujocoParserTest, Include) {
  EXPECT_EQ(plant_.num_model_instances(), 2);
  // This scene.xml defines a scene with two pendula, defined using <include>
  // (and a nested <include>).
  AddAllModelsFromFile(
      FindResourceOrThrow(
          "drake/multibody/parsing/test/mujoco_parser_test/scene.xml"),
      {});
  FlushDiagnostics();
  plant_.Finalize();
  EXPECT_EQ(plant_.num_model_instances(), 3);
  EXPECT_EQ(plant_.num_positions(), 2);
  EXPECT_EQ(plant_.num_velocities(), 2);

  // In order for the total mass to be correct, the geom from the nested
  // include inside pendulum.xml must have been processed.
  auto context = plant_.CreateDefaultContext();
  EXPECT_EQ(plant_.CalcTotalMass(*context), 2.0);
}

class BoxMeshTest : public MujocoParserTest {
 public:
  // Load and evaluate a box mesh, specified in various ways by caller-supplied
  // asset and compiler xml text.
  //
  // Note: This method can only be used once per test case, since it hard-codes
  // the geometry name.
  void TestBoxMesh(std::string expected_filename, std::string mesh_asset,
                std::string compiler = "", double expected_scale = 1.0) {
    std::string xml = fmt::format(
        R"""(
<mujoco model="test">
  {}
  <asset>
    {}
  </asset>
  <worldbody>
    <geom name="box_geom" type="mesh" mesh="box"/>
  </worldbody>
</mujoco>
)""",
        compiler, mesh_asset);

    AddModelFromString(xml, "test");

    const SceneGraphInspector<double>& inspector =
        scene_graph_.model_inspector();
    GeometryId geom_id = inspector.GetGeometryIdByName(
        inspector.world_frame_id(), Role::kProximity, "box_geom");
    auto* mesh =
        dynamic_cast<const geometry::Mesh*>(&inspector.GetShape(geom_id));
    EXPECT_NE(mesh, nullptr);
    EXPECT_EQ(mesh->filename(), expected_filename);
    EXPECT_EQ(mesh->scale(), expected_scale);
  }
};

TEST_F(MujocoParserTest, GeometryPropertiesErrors) {
  std::string xml = R"""(
<mujoco model="test">
  <asset>
    <material/>
  </asset>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeError(), MatchesRegex(".*Material.*must have a name.*"));
}

TEST_F(BoxMeshTest, MeshFileDirectNoName) {
  // Absolute path referencing the obj with the default heuristic for `name`.
  std::string mesh_asset = fmt::format(R"""(<mesh file="{}"/>)""", box_obj_);
  TestBoxMesh(box_obj_, mesh_asset);
}

TEST_F(BoxMeshTest, MeshFileDirect) {
  // Absolute path, referencing the obj directly and with a `name` attribute.
  std::string mesh_asset =
      fmt::format(R"""(<mesh name="box" file="{}"/>)""", box_obj_);
  TestBoxMesh(box_obj_, mesh_asset);
}

TEST_F(BoxMeshTest, MeshFileStlOrObj) {
  // Absolute path, referencing an stl with an obj replacement available.
  std::string quad_cube_stl =
      FindResourceOrThrow("drake/geometry/test/quad_cube.stl");
  std::string quad_cube_obj = std::filesystem::canonical(
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj"));
  std::string mesh_asset =
      fmt::format(R"""(<mesh name="box" file="{}"/>)""", quad_cube_stl);
  TestBoxMesh(quad_cube_obj, mesh_asset);
}

TEST_F(BoxMeshTest, MeshFileRelativePath) {
  // Relative path (from string, so path is relative to cwd).
  std::string mesh_asset = R"""(
<mesh name="box" file="multibody/parsing/test/box_package/meshes/box.obj"/>)""";
  TestBoxMesh(box_obj_, mesh_asset);
}

TEST_F(BoxMeshTest, MeshFileAbsolutePathCompiler) {
  // Absolute path in the meshdir compiler attribute.
  std::string mesh_asset =
      R"""(<mesh name="box" file="box.obj"/>)""";
  // Additionally confirm that meshdir takes priority over assetdir.
  std::string compiler =
      fmt::format(R"""(<compiler assetdir="invalid_name" meshdir="{}"/>)""",
                  std::filesystem::path(box_obj_).parent_path().string());
  TestBoxMesh(box_obj_, mesh_asset, compiler);
}

TEST_F(BoxMeshTest, MeshFileAbsolutePathCompilerUsingAssetDir) {
  // Absolute path in the meshdir compiler attribute.
  std::string mesh_asset =
      R"""(<mesh name="box" file="box.obj"/>)""";
  std::string compiler =
      fmt::format(R"""(<compiler assetdir="{}"/>)""",
                  std::filesystem::path(box_obj_).parent_path().string());
  TestBoxMesh(box_obj_, mesh_asset, compiler);
}

TEST_F(BoxMeshTest, MeshFileRelativePathCompiler) {
  // Relative path + meshdir compiler attribute.
  std::string mesh_asset =
      R"""(<mesh name="box" file="box.obj"/>)""";
  std::string compiler =
      R"""(<compiler meshdir="multibody/parsing/test/box_package/meshes"/>)""";
  TestBoxMesh(box_obj_, mesh_asset, compiler);
}

TEST_F(BoxMeshTest, MeshFileScale) {
  // Test the scale attribute.
  std::string mesh_asset =
      fmt::format(R"""(<mesh name="box" file="{}" scale="2 2 2"/>)""", box_obj_);
  TestBoxMesh(box_obj_, mesh_asset, "", 2.0);
}

TEST_F(BoxMeshTest, MeshFileScaleViaDefault) {
  // Test the scale set via mesh defaults. According to the mjcf docs, this is
  // the only supported mesh default.
  std::string mesh_asset =
      fmt::format(R"""(<mesh name="box" file="{}"/>)""", box_obj_);
  std::string defaults = R"""(<default><mesh scale="2 2 2"/></default>)""";
  TestBoxMesh(box_obj_, mesh_asset, defaults, 2.0);
}

TEST_F(MujocoParserTest, MeshFileRelativePathFromFile) {
  const std::string file = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/mjcfs/box.xml");

  AddModelFromFile(file, "test");

  const SceneGraphInspector<double>& inspector =
      scene_graph_.model_inspector();
  GeometryId geom_id = inspector.GetGeometryIdByName(
      inspector.world_frame_id(), Role::kProximity, "box_geom");
  auto* mesh =
      dynamic_cast<const geometry::Mesh*>(&inspector.GetShape(geom_id));

  EXPECT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->filename(), box_obj_);
  EXPECT_EQ(mesh->scale(), 1.0);
}

TEST_F(MujocoParserTest, InertiaFromGeometry) {
  std::string xml = fmt::format(R"""(
<mujoco model="test">
  <default class="main">
    <geom mass="2.53"/>
    <default class="default_box">
      <geom type="box" size="0.1 0.2 0.3"/>
      <default class="sub">
        <geom size="0.4 0.5 0.6"/>
      </default>
    </default>
  </default>
  <asset>
    <mesh name="box_mesh" file="{}"/>
    <mesh name="non_convex_mesh" file="{}"/>
  </asset>
  <worldbody>
    <body name="default">
      <geom name="default" size="0.1"/>
    </body>
    <body name="sphere">
      <inertial mass="524" diaginertia="1 2 3"/>
      <geom name="sphere" type="sphere" size="0.1"/>
    </body>
    <body name="capsule">
      <geom name="capsule" type="capsule" size="0.1 2.0"/>
    </body>
    <body name="ellipsoid">
      <geom name="ellipsoid" type="ellipsoid" size="0.1 0.2 0.3"/>
    </body>
    <body name="cylinder">
      <geom name="cylinder" type="cylinder" size="0.1 2.0"/>
    </body>
    <body name="box">
      <geom name="box" type="box" size="0.1 2.0 3.0"/>
    </body>
    <body name="box_from_default">
      <geom name="box_from_default" class="default_box"/>
    </body>
    <body name="ellipsoid_from_default">
      <geom name="ellipsoid_from_default" type="ellipsoid" class="default_box"/>
    </body>
    <body name="box_from_sub" childclass="sub">
      <geom name="box_from_sub"/>
    </body>
    <body name="two_spheres">
      <geom name="thing1" type="sphere" size="0.1"/>
      <geom name="thing2" type="sphere" size="0.2"/>
    </body>
    <body name="offset_cylinder">
      <geom name="offset_cylinder" type="cylinder" size="0.4 2.0" pos="2 0 0"
            zaxis="1 0 0"/>
    </body>
    <body name="box_from_mesh">
      <geom name="box_from_mesh" type="mesh" mesh="box_mesh" mass="1.0"/>
    </body>
    <body name="non_convex_body">
      <geom name="non_convex" type="mesh" mesh="non_convex_mesh" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
)""",
                                box_obj_, non_convex_obj_);

  AddModelFromString(xml, "test");

  xml = fmt::format(R"""(
<mujoco model="test_auto">
  <compiler inertiafromgeom="auto"/>
  <asset>
    <mesh name="box_mesh" file="{}"/>
    <mesh name="non_convex_mesh" file="{}"/>
  </asset>
  <worldbody>
    <body name="sphere_auto">
      <inertial mass="524" diaginertia="1 2 3"/>
      <geom name="sphere" mass="2.53" type="sphere" size="0.1"/>
    </body>
    <body name="box_w_density">
      <geom name="box_w_density" density="123" type="box" size=".4 .5 .6"/>
    </body>
    <body name="box_default_density">
      <geom name="box_w_density" type="box" size=".4 .5 .6"/>
    </body>
    <body name="box_from_mesh_w_density">
      <geom name="box_from_mesh_w_density" type="mesh" mesh="box_mesh"
            density="1.0"/>
    </body>
    <body name="non_convex_body_w_density">
      <geom name="non_convex_w_density" type="mesh" mesh="non_convex_mesh"
            density="1.0"/>
    </body>
  </worldbody>
</mujoco>
)""",
                    box_obj_, non_convex_obj_);

  AddModelFromString(xml, "test_auto");

  xml = R"""(
<mujoco model="test_true">
  <compiler inertiafromgeom="true"/>
  <default class="main">
    <geom mass="2.53"/>
  </default>
  <worldbody>
    <body name="sphere_true">
      <inertial mass="524" diaginertia="1 2 3"/>
      <geom name="sphere" type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test_true");

  xml = R"""(
<mujoco model="test_false">
  <compiler inertiafromgeom="false"/>
  <default class="main">
    <geom mass="2.53"/>
  </default>
  <worldbody>
    <body name="sphere_false">
      <inertial mass="524" diaginertia="1 2 3"/>
      <geom name="sphere" type="sphere" size="0.1"/>
    </body>
    <body name="full_inertia">
      <inertial mass="1.23" fullinertia="2 2 2 -.75 -.75 -.75"/>
    </body>
    <body name="full_inertia_w_pose">
      <inertial mass="1.23" fullinertia="2 2 2 -.75 -.75 -.75"
               quat="0.2 0.4 0.3 .1" pos="1 2 3"/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test_false");

  plant_.Finalize();

  auto context = plant_.CreateDefaultContext();

  auto check_body = [this, &context](
                        const std::string& body_name,
                        const UnitInertia<double>& unit_M_BBo_B) {
    SCOPED_TRACE(fmt::format("checking body {}", body_name));
    const RigidBody<double>& body = plant_.GetBodyByName(body_name);
    EXPECT_TRUE(CompareMatrices(
        body.CalcSpatialInertiaInBodyFrame(*context).CopyToFullMatrix6(),
        SpatialInertia<double>(2.53, Vector3d::Zero(), unit_M_BBo_B)
            .CopyToFullMatrix6(),
        1e-14));
  };

  auto check_body_spatial = [this, &context](
                                const std::string& body_name,
                                const SpatialInertia<double>& M_BBo_B,
                                double tol = 1e-14) {
    SCOPED_TRACE(fmt::format("checking body {} (spatial)", body_name));
    const RigidBody<double>& body = plant_.GetBodyByName(body_name);
    EXPECT_TRUE(CompareMatrices(
        body.CalcSpatialInertiaInBodyFrame(*context).CopyToFullMatrix6(),
        M_BBo_B.CopyToFullMatrix6(), tol));
  };

  const SpatialInertia<double> inertia_from_inertial_tag =
      SpatialInertia<double>::MakeFromCentralInertia(
          524, Vector3d::Zero(), RotationalInertia<double>(1, 2, 3));

  check_body("default", UnitInertia<double>::SolidSphere(0.1));
  check_body_spatial("sphere", inertia_from_inertial_tag);
  check_body("capsule",
      UnitInertia<double>::SolidCapsule(0.1, 4.0, Vector3d::UnitZ()));
  check_body("ellipsoid", UnitInertia<double>::SolidEllipsoid(0.1, 0.2, 0.3));
  check_body("cylinder",
      UnitInertia<double>::SolidCylinder(0.1, 4.0, Vector3d::UnitZ()));
  check_body("box", UnitInertia<double>::SolidBox(0.2, 4.0, 6.0));
  check_body("box_from_default", UnitInertia<double>::SolidBox(0.2, 0.4, 0.6));
  check_body("ellipsoid_from_default",
             UnitInertia<double>::SolidEllipsoid(0.1, 0.2, 0.3));
  check_body("box_from_sub", UnitInertia<double>::SolidBox(0.8, 1.0, 1.2));
  SpatialInertia<double> M_BBo_B =
      SpatialInertia<double>::SolidSphereWithMass(2.53, 0.1);
  M_BBo_B += SpatialInertia<double>::SolidSphereWithMass(2.53, 0.2);
  check_body_spatial("two_spheres", M_BBo_B);
  // Use the equation for the moment of inertia of a cylinder about one end.
  M_BBo_B = SpatialInertia<double>(
      2.53,                                    // mass
      Vector3d{2.0, 0.0, 0.0},                 // Center of mass
      UnitInertia<double>(0.16 / 2.0,          // 1/2 r²
                          .04 + (16.0 / 3.0),  // 1/4 r^2+ 1/3 L^2)
                          .04 + (16.0 / 3.0)));
  check_body_spatial("offset_cylinder", M_BBo_B);
  check_body_spatial(
      "box_from_mesh",
      SpatialInertia<double>::SolidCubeWithMass(1.0, 2.0),
      1e-13);
  // This unit inertia and center of mass were collected empirically from the
  // results of multibody::CalcSpatialInertia() on the non-convex mesh. The
  // important fact is that it differs from the result obtained by estimating
  // inertia on an oriented bounding box.
  const UnitInertia<double> non_convex_unit_inertia{0.168,  0.168,  0.168,
                                                    -0.034, -0.034, -0.034};
  const Vector3d non_convex_com = Vector3d::Constant(0.2166666666666666);
  check_body_spatial(
      "non_convex_body",
      SpatialInertia<double>{1.0, non_convex_com, non_convex_unit_inertia},
      1e-13);

  check_body_spatial("sphere_auto", inertia_from_inertial_tag);
  check_body("sphere_true", UnitInertia<double>::SolidSphere(0.1));
  check_body_spatial("sphere_false", inertia_from_inertial_tag);

  check_body_spatial(
      "box_w_density",
      SpatialInertia<double>::SolidBoxWithDensity(123, 0.8, 1.0, 1.2));
  check_body_spatial(
      "box_default_density",
      SpatialInertia<double>::SolidBoxWithDensity(1000, 0.8, 1.0, 1.2));
  check_body_spatial(
      "box_from_mesh_w_density",
      SpatialInertia<double>::SolidCubeWithMass(8.0, 2.0),
      1e-12);
  check_body_spatial(
      "non_convex_body_w_density",
      SpatialInertia<double>{0.1, non_convex_com, non_convex_unit_inertia},
      1e-13);

  // A cube rotating about its corner.
  RotationalInertia<double> I_BFo_B(2, 2, 2, -.75, -.75, -.75);
  check_body_spatial("full_inertia",
                     SpatialInertia<double>::MakeFromCentralInertia(
                         1.23, Vector3d::Zero(), I_BFo_B));
  // The rotational component of this inertia is ignored, because the full
  // inertia was specified.
  check_body_spatial("full_inertia_w_pose",
                     SpatialInertia<double>::MakeFromCentralInertia(
                         1.23, Vector3d{1, 2, 3}, I_BFo_B));
}

TEST_F(MujocoParserTest, CompilerErrors) {
  std::string xml = R"""(
<mujoco model="test">
  <compiler coordinate="global" angle="grad" inertiafromgeom="QQQ" eulerseq="abc"/>
  <worldbody>
    <body name="trigger_bad_euler_seq" euler="30 45 60"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*grad.*angle.*ignored.*"));
  EXPECT_THAT(TakeWarning(),
              MatchesRegex(".*inertiafromgeom.*interpret.*ignored.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*coordinate.*not supported.*"));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Illegal value 'abc' for the eulerseq in compiler.*"));
}

TEST_F(MujocoParserTest, CompilerErrorsShortEulerSeq) {
  // Eulerseq is 1 character instead of 3.
  std::string xml = R"""(
<mujoco model="test">
  <compiler eulerseq="a"/>
  <worldbody>
    <body name="trigger_bad_euler_seq" euler="30 45 60"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Illegal value 'a' for the eulerseq in compiler.*"));
}

TEST_F(MujocoParserTest, AssetErrors) {
  std::string xml = fmt::format(R"""(
<mujoco model="test">
  <asset>
    <mesh name="box_mesh" file="{}" scale="1 3 5"/>
    <mesh name="wrong-format" file="{}"/>
    <mesh name="missing-file" file="{}QQQ"/>
    <mesh name="no-file-name"/>
  </asset>
</mujoco>
)""", box_obj_, box_urdf_, box_urdf_);

  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*only supports.*obj format.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*not.*found.*nor.*replacement.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*not specify.*file.*ignored.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*non-uniform scale.*"));
}

TEST_F(MujocoParserTest, AssetDirErrors) {
  std::string xml = R"""(
<mujoco model="test">
  <compiler assetdir="invalid_dir"/>
  <asset>
    <mesh name="box_mesh" file="box.obj" />
  </asset>
  <worldbody>
    <geom name="box_geom" type="mesh" mesh="box_mesh"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeWarning(),
              MatchesRegex(".*mesh asset.*could not be found.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*specified unknown mesh.*"));
}

TEST_F(MujocoParserTest, BodyError) {
  std::string xml = R"""(
<mujoco model="test">
  <compiler inertiafromgeom="false"/>
  <worldbody>
    <body name="sphere"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeError(), MatchesRegex(".*must specify an inertia.*"));
}

TEST_F(MujocoParserTest, Joint) {
  std::string xml = R"""(
<mujoco model="test">
  <compiler eulerseq="XYZ"/>
  <default>
    <geom type="sphere" size="1"/>
    <default class="default_joint">
      <joint type="hinge" damping="0.24" pos="-.1 -.2 -.3"
             axis="1 0 0" limited="true" range="-30 60" />
    </default>
  </default>
  <worldbody>
    <body name="freejoint" pos="1 2 3" euler="30 45 60">
      <freejoint name="xfreejoint"/>  <!-- ignored -->
    </body>
    <body name="free" pos="1 2 3" euler="30 45 60">
      <joint type="free" name="xfree"/>  <!-- ignored -->
    </body>
    <body name="ball" pos="1 2 3" euler="30 45 60">
      <joint type="ball" name="ball" damping="0.1" pos=".1 .2 .3"/>
    </body>
    <body name="slide" pos="1 2 3" euler="30 45 60">
      <joint type="slide" name="slide" damping="0.2" pos=".1 .2 .3"
             axis="1 0 0" limited="true" range="-2 1.5"/>
    </body>
    <body name="hinge" pos="1 2 3" euler="30 45 60">
      <joint type="hinge" name="hinge" damping="0.3" pos=".1 .2 .3"
             axis="0 1 0" limited="true" range="-30 60"/>
    </body>
    <body name="hinge_w_joint_defaults" pos="1 2 3" euler="30 45 60">
      <joint type="hinge" name="hinge_w_joint_defaults" class="default_joint" />
    </body>
    <body name="default" pos="1 2 3" euler="30 45 60">
      <!-- without the limited=true tag -->
      <joint name="default" damping="0.4" range="-20 15"/>
    </body>
    <body name="weld" pos="1 2 3" euler="30 45 60"/>
    <body name="two_hinges" pos="1 2 3" euler="30 45 60">
      <joint type="hinge" name="hinge1" damping="0.5" pos=".1 .2 .3"
             axis="1 0 0"/>
      <joint type="hinge" name="hinge2" damping="0.6" pos=".1 .2 .3"
             axis="0 1 0"/>
    </body>
    <body name="default_joint">
      <!-- provides code coverage for defaults logic. -->
      <joint/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  plant_.Finalize();

  auto context = plant_.CreateDefaultContext();

  RigidTransformd X_WB(RollPitchYawd{M_PI / 6.0, M_PI / 4.0, M_PI / 3.0},
                       Vector3d{1.0, 2.0, 3.0});
  Vector3d pos{.1, .2, .3};

  // Note: for free bodies Drake ignores the given Mujoco joint name and makes
  // its own floating joint named like the body.
  const RigidBody<double>& freejoint_body = plant_.GetBodyByName("freejoint");
  EXPECT_FALSE(plant_.HasJointNamed("xfreejoint"));
  EXPECT_TRUE(plant_.HasJointNamed("freejoint"));
  EXPECT_TRUE(freejoint_body.is_floating());
  EXPECT_TRUE(plant_.GetFreeBodyPose(*context, freejoint_body)
                  .IsNearlyEqualTo(X_WB, 1e-14));

  const RigidBody<double>& free_body = plant_.GetBodyByName("free");
  EXPECT_FALSE(plant_.HasJointNamed("xfree"));
  EXPECT_TRUE(plant_.HasJointNamed("free"));
  EXPECT_TRUE(free_body.is_floating());
  EXPECT_TRUE(
      plant_.GetFreeBodyPose(*context, free_body).IsNearlyEqualTo(X_WB, 1e-14));

  const BallRpyJoint<double>& ball_joint =
      plant_.GetJointByName<BallRpyJoint>("ball");
  EXPECT_EQ(ball_joint.default_damping(), 0.1);
  EXPECT_TRUE(ball_joint.frame_on_child()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  EXPECT_TRUE(
      plant_.GetBodyByName("ball").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));

  const PrismaticJoint<double>& slide_joint =
      plant_.GetJointByName<PrismaticJoint>("slide");
  EXPECT_EQ(slide_joint.default_damping(), 0.2);
  EXPECT_TRUE(slide_joint.frame_on_child()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  EXPECT_TRUE(
      CompareMatrices(slide_joint.translation_axis(), Vector3d{1, 0, 0}));
  EXPECT_TRUE(
      plant_.GetBodyByName("slide").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));
  EXPECT_TRUE(CompareMatrices(
      plant_.GetJointByName("slide").position_lower_limits(), Vector1d{-2.0}));
  EXPECT_TRUE(CompareMatrices(
      plant_.GetJointByName("slide").position_upper_limits(), Vector1d{1.5}));

  const RevoluteJoint<double>& hinge_joint =
      plant_.GetJointByName<RevoluteJoint>("hinge");
  EXPECT_EQ(hinge_joint.default_damping(), 0.3);
  EXPECT_TRUE(hinge_joint.frame_on_child()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  EXPECT_TRUE(CompareMatrices(hinge_joint.revolute_axis(), Vector3d{0, 1, 0}));
  EXPECT_TRUE(
      plant_.GetBodyByName("hinge").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));
  EXPECT_TRUE(
      CompareMatrices(plant_.GetJointByName("hinge").position_lower_limits(),
                      Vector1d{-M_PI / 6.0}, 1e-14));
  EXPECT_TRUE(
      CompareMatrices(plant_.GetJointByName("hinge").position_upper_limits(),
                      Vector1d{M_PI / 3.0}, 1e-14));

  const RevoluteJoint<double>& hinge_w_joint_defaults_joint =
      plant_.GetJointByName<RevoluteJoint>("hinge_w_joint_defaults");
  EXPECT_EQ(hinge_w_joint_defaults_joint.default_damping(), 0.24);
  EXPECT_TRUE(
      hinge_w_joint_defaults_joint.frame_on_child()
          .CalcPoseInBodyFrame(*context)
          .IsNearlyEqualTo(RigidTransformd(Vector3d{-0.1, -0.2, -0.3}), 1e-14));
  EXPECT_TRUE(CompareMatrices(hinge_w_joint_defaults_joint.revolute_axis(),
                              Vector3d{1, 0, 0}));
  EXPECT_TRUE(plant_.GetBodyByName("hinge_w_joint_defaults")
                  .EvalPoseInWorld(*context)
                  .IsNearlyEqualTo(X_WB, 1e-14));
  EXPECT_TRUE(CompareMatrices(
      plant_.GetJointByName("hinge_w_joint_defaults").position_lower_limits(),
      Vector1d{-M_PI / 6.0}, 1e-14));
  EXPECT_TRUE(CompareMatrices(
      plant_.GetJointByName("hinge_w_joint_defaults").position_upper_limits(),
      Vector1d{M_PI / 3.0}, 1e-14));

  const RevoluteJoint<double>& default_joint =
      plant_.GetJointByName<RevoluteJoint>("default");
  EXPECT_EQ(default_joint.default_damping(), 0.4);
  EXPECT_TRUE(default_joint.frame_on_child()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyIdentity(1e-14));
  EXPECT_TRUE(
      CompareMatrices(default_joint.revolute_axis(), Vector3d{0, 0, 1}));
  EXPECT_TRUE(
      plant_.GetBodyByName("default").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));
  EXPECT_TRUE(
      CompareMatrices(plant_.GetJointByName("default").position_lower_limits(),
                      Vector1d{-M_PI / 9.0}));
  EXPECT_TRUE(
      CompareMatrices(plant_.GetJointByName("default").position_upper_limits(),
                      Vector1d{M_PI / 12.0}));

  const RevoluteJoint<double>& hinge1_joint =
      plant_.GetJointByName<RevoluteJoint>("hinge1");
  EXPECT_EQ(hinge1_joint.default_damping(), 0.5);
  EXPECT_TRUE(CompareMatrices(hinge1_joint.revolute_axis(), Vector3d{1, 0, 0}));
  EXPECT_TRUE(hinge1_joint.frame_on_parent()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  const RevoluteJoint<double>& hinge2_joint =
      plant_.GetJointByName<RevoluteJoint>("hinge2");
  EXPECT_EQ(hinge2_joint.default_damping(), 0.6);
  EXPECT_TRUE(CompareMatrices(hinge2_joint.revolute_axis(), Vector3d{0, 1, 0}));
  EXPECT_TRUE(hinge2_joint.frame_on_child()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  EXPECT_TRUE(plant_.GetBodyByName("two_hinges")
                  .EvalPoseInWorld(*context)
                  .IsNearlyEqualTo(X_WB, 1e-14));

  EXPECT_TRUE(
      plant_.GetBodyByName("weld").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));
  const WeldJoint<double>& weld_joint =
      plant_.GetJointByName<WeldJoint>("world_welds_to_weld");
  EXPECT_TRUE(weld_joint.X_FM().IsNearlyEqualTo(X_WB, 1e-14));
}

TEST_F(MujocoParserTest, JointErrors) {
  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <body name="free" pos="1 2 3" euler="30 45 60">
      <joint type="free" name="free" damping="10"/>
      <joint type="hinge"/>
    </body>
    <body>
      <joint type="ball" limited="true" range="-1 1"/>
    </body>
    <body>
      <joint type="impossible"/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*Damping.*not supported for free.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*range.*unsupported.*ignored.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*a free joint is defined.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*Unknown joint type.*"));
}

std::string MakeAutoLimitsXML(
  bool auto_limits, const std::string& motor_limit_prefix) {
    return fmt::format(R"""(
  <mujoco model="test">
    <compiler autolimits="{0}" angle="radian"/>
    <worldbody>
      <body>
        <joint type="hinge" name="hinge0" limited="true"/>
        <joint type="hinge" name="hinge1" range="-1 1"/>
        <joint type="hinge" name="hinge2" range="-2 2" limited="auto"/>
        <joint type="hinge" name="hinge3" range="-3 3" limited="true"/>
        <joint type="hinge" name="hinge4" range="-4 4" limited="false"/>
        <joint type="hinge" name="hinge6" range="-5 5" limited="bad"/>
      </body>
    </worldbody>
    <actuator>
      <!-- Drake does not allow effort limits to be zero. -->
      <!-- <motor joint="hinge0" {1}limited="true"/> -->
      <motor joint="hinge1" {1}range="-1 1"/>
      <motor joint="hinge2" {1}range="-2 2" {1}limited="auto"/>
      <motor joint="hinge3" {1}range="-3 3" {1}limited="true"/>
      <motor joint="hinge4" {1}range="-4 4" {1}limited="false"/>
      <motor joint="hinge6" {1}range="-5 5" {1}limited="bad"/>
    </actuator>
  </mujoco>
  )""", auto_limits, motor_limit_prefix);
}

TEST_F(MujocoParserTest, AutoLimitsTrue) {
  const std::string kXml = MakeAutoLimitsXML(true, "ctrl");
  AddModelFromString(kXml, "test");
  plant_.Finalize();
  EXPECT_EQ(plant_.num_positions(), 6);
  VectorXd expected_limits(6);
  expected_limits << 0.0, 1.0, 2.0, 3.0, kInf, kInf;
  EXPECT_TRUE(
      CompareMatrices(plant_.GetPositionLowerLimits(), -expected_limits));
  EXPECT_TRUE(
      CompareMatrices(plant_.GetPositionUpperLimits(), expected_limits));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The 'limited' attribute must be one of.*"));
  EXPECT_TRUE(CompareMatrices(
    plant_.GetEffortLowerLimits(), -expected_limits.tail<5>()));
  EXPECT_TRUE(CompareMatrices(
    plant_.GetEffortUpperLimits(), expected_limits.tail<5>()));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The 'ctrllimited' attribute must be one of.*"));
}

TEST_F(MujocoParserTest, AutoLimitsFalse) {
  const std::string kXml = MakeAutoLimitsXML(false, "force");
  AddModelFromString(kXml, "test");
  plant_.Finalize();
  EXPECT_EQ(plant_.num_positions(), 6);
  VectorXd expected_limits(6);
  expected_limits << 0.0, kInf, kInf, 3.0, kInf, kInf;
  EXPECT_TRUE(
      CompareMatrices(plant_.GetPositionLowerLimits(), -expected_limits));
  EXPECT_TRUE(
      CompareMatrices(plant_.GetPositionUpperLimits(), expected_limits));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The 'range' attribute was specified.*but "
                           "'autolimits' is disabled."));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The 'range' attribute was specified.*but "
                           "'autolimits' is disabled."));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The 'limited' attribute must be one of.*"));
  EXPECT_TRUE(CompareMatrices(
    plant_.GetEffortLowerLimits(), -expected_limits.tail<5>()));
  EXPECT_TRUE(CompareMatrices(
    plant_.GetEffortUpperLimits(), expected_limits.tail<5>()));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The 'forcerange' attribute was specified.*but "
                           "'autolimits' is disabled."));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The 'forcerange' attribute was specified.*but "
                           "'autolimits' is disabled."));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The 'forcelimited' attribute must be one of.*"));
}

TEST_F(MujocoParserTest, InertialErrors) {
  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <body>
      <inertial/>
    </body>
    <body>
      <inertial mass="10"/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*inertial.*must include.*mass.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*inertial.*must include.*diaginertia or fullinertia.*"));
}

TEST_F(MujocoParserTest, GeomAutoName) {
  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <body>
      <geom type="sphere" size="1"/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  auto& inspector = scene_graph_.model_inspector();
  auto geom_ids = inspector.GetAllGeometryIds();
  EXPECT_EQ(geom_ids.size(), 2);
  for (auto geom_id : geom_ids) {
    EXPECT_EQ(inspector.GetName(geom_id), fmt::format("test::geom0"));
  }
}

TEST_F(MujocoParserTest, GeomErrors) {
  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <body>
      <geom/>
      <geom type="capsule"/>
      <geom type="capsule" fromto="1 2 3 4 5 6"/>
      <geom type="ellipsoid"/>
      <geom type="ellipsoid" fromto="1 2 3 4 5 6"/>
      <geom type="cylinder"/>
      <geom type="cylinder" fromto="1 2 3 4 5 6"/>
      <geom type="box"/>
      <geom type="box" fromto="1 2 3 4 5 6"/>
      <geom type="mesh"/>
      <geom type="mesh" mesh="nonsense"/>
      <geom type="hfield"/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromString(xml, "test");

  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*zero-radius spheres.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*fromto.*ellipsoid.*unsupported.*ignored.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*fromto.*box.*unsupported.*ignored.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*unknown mesh.*ignored.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*hfield.*unsupported.*ignored.*"));

  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*size.*capsule.*must have.*two elements.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*size.*capsule.*fromto.*must have.*one element.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*size.*ellipsoid.*must have.*three elements.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*size.*cylinder.*must have.*two elements.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*size.*cylinder.*fromto.*must have.*one element.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*size.*box.*must have.*three elements.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(
      ".*mesh.*did not set the mesh attribute.*"));
}

TEST_F(MujocoParserTest, Motor) {
  std::string xml = R"""(
<mujoco model="test">
  <default>
    <geom type="sphere" size="1"/>
  </default>
  <worldbody>
    <body>
      <joint type="hinge" name="hinge0" axis="0 1 0"/>
    </body>
    <body>
      <joint type="hinge" name="hinge1" axis="0 1 0"/>
    </body>
    <body>
      <joint type="hinge" name="hinge2" axis="0 1 0"/>
    </body>
    <body>
      <joint type="hinge" name="hinge3" axis="0 1 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge0"/>
    <!-- intentionally asymmetric effort limits to cover the warning code -->
    <motor name="motor1" joint="hinge1" ctrllimited="true" ctrlrange="-1 2"/>
    <motor name="motor2" joint="hinge2" ctrllimited="true" ctrlrange="-3 2"
           forcelimited="true" forcerange="-.5 .4"/>
    <!-- malformed limits will be ignored -->
    <motor name="motor3" joint="hinge3" ctrllimited="true" ctrlrange="2 1"
           forcelimited="true" forcerange="2 1"/>
  </actuator>
</mujoco>
)""";

  AddModelFromString(xml, "test");
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*motor1.*ctrlrange.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*motor2.*ctrlrange.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*motor2.*forcerange.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*motor3.*ctrlrange.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*motor3.*forcerange.*"));

  plant_.Finalize();

  EXPECT_EQ(plant_.get_actuation_input_port().size(), 4);

  const JointActuator<double>& motor0 =
      plant_.get_joint_actuator(JointActuatorIndex(0));
  EXPECT_EQ(motor0.name(), "motor0");
  EXPECT_EQ(motor0.joint().name(), "hinge0");
  EXPECT_EQ(motor0.effort_limit(), std::numeric_limits<double>::infinity());

  const JointActuator<double>& motor1 =
      plant_.get_joint_actuator(JointActuatorIndex(1));
  EXPECT_EQ(motor1.name(), "motor1");
  EXPECT_EQ(motor1.joint().name(), "hinge1");
  EXPECT_EQ(motor1.effort_limit(), 2);

  const JointActuator<double>& motor2 =
      plant_.get_joint_actuator(JointActuatorIndex(2));
  EXPECT_EQ(motor2.name(), "motor2");
  EXPECT_EQ(motor2.joint().name(), "hinge2");
  EXPECT_EQ(motor2.effort_limit(), .5);

  const JointActuator<double>& motor3 =
      plant_.get_joint_actuator(JointActuatorIndex(3));
  EXPECT_EQ(motor3.name(), "motor3");
  EXPECT_EQ(motor3.joint().name(), "hinge3");
  EXPECT_EQ(motor3.effort_limit(), std::numeric_limits<double>::infinity());
}

class ContactTest : public MujocoParserTest,
                    public testing::WithParamInterface<std::tuple<bool, bool>> {
};

TEST_P(ContactTest, Contact) {
  auto [include_contact, adjacent_bodies_collision_filters] = GetParam();

  static constexpr char xml_base[] = R"""(
  <mujoco model="test">
    <default>
      <geom type="sphere" size="1"/>
    </default>
    <worldbody>
      <body name="base">
        <geom name="base_geom"/>
        <body name="body1">
          <geom name="body1_geom"/>
          <joint type="hinge"/>
        </body>
        <body name="body2">
          <geom name="body2_geom"/>
          <joint type="hinge"/>
        </body>
      </body>
    </worldbody>
    {}
  </mujoco>
  )""";
  static constexpr char contact_node[] = R"""(
    <contact>
      <pair name="pair1" geom1="base_geom" geom2="body1_geom"/>
      <exclude name="exclude1" body1="body1" body2="body2" />
    </contact>
  )""";

  std::string xml = fmt::format(xml_base, include_contact ? contact_node : "");
  plant_.set_adjacent_bodies_collision_filters(
      adjacent_bodies_collision_filters);
  AddModelFromString(xml, "test");
  plant_.Finalize();

  const SceneGraphInspector<double>& inspector = scene_graph_.model_inspector();
  GeometryId base_geom = inspector.GetGeometries(
      plant_.GetBodyFrameIdOrThrow(plant_.GetBodyByName("base").index()),
      geometry::Role::kProximity)[0];
  GeometryId body1_geom = inspector.GetGeometries(
      plant_.GetBodyFrameIdOrThrow(plant_.GetBodyByName("body1").index()),
      geometry::Role::kProximity)[0];
  GeometryId body2_geom = inspector.GetGeometries(
      plant_.GetBodyFrameIdOrThrow(plant_.GetBodyByName("body2").index()),
      geometry::Role::kProximity)[0];
  if (include_contact) {
    if (adjacent_bodies_collision_filters) {
      // In this case, the parser will have emitted a warning, and the
      // collision filter gets overwritten during Finalize().
      EXPECT_THAT(TakeWarning(), MatchesRegex(".*Finalize.*will overwrite.*"));
      EXPECT_TRUE(inspector.CollisionFiltered(base_geom, body1_geom));
    } else {
      // No warning is emitted; the parsed filter is consistent with the
      // defaults.
      EXPECT_FALSE(inspector.CollisionFiltered(base_geom, body1_geom));
    }
    EXPECT_TRUE(inspector.CollisionFiltered(body1_geom, body2_geom));
  } else {
    if (adjacent_bodies_collision_filters) {
      EXPECT_TRUE(inspector.CollisionFiltered(base_geom, body1_geom));
    } else {
      EXPECT_FALSE(inspector.CollisionFiltered(base_geom, body1_geom));
    }
    EXPECT_FALSE(inspector.CollisionFiltered(body1_geom, body2_geom));
  }
}

INSTANTIATE_TEST_SUITE_P(
    ContactTests, ContactTest,
    testing::Combine(testing::Bool(), testing::Bool()));

TEST_F(MujocoParserTest, ContactWarnings) {
  std::string xml = R"""(
<mujoco model="test">
  <default>
    <geom type="sphere" size="1"/>
  </default>
  <worldbody>
    <body name="base">
      <geom name="base_geom"/>
      <body name="body1">
        <geom name="body1_geom"/>
        <joint type="hinge"/>
      </body>
      <body name="body2">
        <geom name="body2_geom"/>
        <joint type="hinge"/>
      </body>
    </body>
  </worldbody>
  <contact>
    <pair name="no-geom1" geom2="body1_geom"/>
    <pair name="no-geom2" geom1="body1_geom"/>
    <pair name="unknown-geom1" geom1="QQQ" geom2="body1_geom"/>
    <pair name="unknown-geom2" geom1="body1_geom" geom2="QQQ"/>
    <exclude name="no-body1" body2="body2" />
    <exclude name="no-body2" body1="body1" />
  </contact>
</mujoco>
)""";

  AddModelFromString(xml, "test");

  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*pair.*not have.*geom1.*geom2.*ignored.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*pair.*not have.*geom1.*geom2.*ignored.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*pair.*unknown geom1.*ignored.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*pair.*unknown geom2.*ignored.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*exclude.*not have.*body1.*body2.*ignored.*"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(
      ".*exclude.*not have.*body1.*body2.*ignored.*"));
}

// TODO(rpoyner-tri) consider how to convert these to diagnostics.
TEST_F(MujocoParserTest, ContactThrows) {
  static constexpr char xml_base[] = R"""(
<mujoco model="test">
  <worldbody>
    <body name="base">
      <body name="body1">
        <joint type="hinge"/>
      </body>
      <body name="body2">
        <joint type="hinge"/>
      </body>
    </body>
  </worldbody>
  <contact>
    <exclude name="unknown-body" body1="{}" body2="{}" />
  </contact>
</mujoco>
)""";

  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromString(fmt::format(xml_base, "QQQ", "body2"), "bad_body1"),
      ".*no RigidBody named 'QQQ' anywhere.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromString(fmt::format(xml_base, "body1", "QQQ"), "bad_body2"),
      ".*no RigidBody named 'QQQ' anywhere.*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
