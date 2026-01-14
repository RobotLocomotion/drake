#include "drake/multibody/parsing/detail_sdf_parser.h"

#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sdf/parser.hh>

#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/scope_exit.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsing/detail_mujoco_parser.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/curvilinear_joint.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/screw_joint.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using ::testing::MatchesRegex;

using drake::internal::DiagnosticDetail;
using drake::internal::DiagnosticPolicy;
using drake::trajectories::PiecewiseConstantCurvatureTrajectory;
using drake::trajectories::Trajectory;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RollPitchYawd;
using systems::Context;

using CollisionPair = SortedPair<std::string>;

const double kEps = std::numeric_limits<double>::epsilon();

// Fixture to add test coverage for the SDF parser. Some features such as mimic
// joints and ball constraints are only supported in discrete mode when using
// the SAP solver. For testing such features, we set a model approximation that
// uses the SAP solver. More specifically, we call
// set_discrete_contact_approximation(DiscreteContactApproximation::kSap) on the
// MultibodyPlant used for testing before parsing.
class SdfParserTest : public test::DiagnosticPolicyTestBase {
 public:
  SdfParserTest() { RecordErrors(); }

  void AddSceneGraph() { plant_.RegisterAsSourceForSceneGraph(&scene_graph_); }

  static ParserInterface& TestingSelect(const DiagnosticPolicy&,
                                        const std::string& filename) {
    // TODO(rpoyner-tri): add more formats here, as tests use them.
    static never_destroyed<UrdfParserWrapper> urdf;
    static never_destroyed<MujocoParserWrapper> mujoco;
    if (EndsWithCaseInsensitive(filename, ".urdf")) {
      return urdf.access();
    }
    if (EndsWithCaseInsensitive(filename, ".xml")) {
      return mujoco.access();
    }
    throw std::runtime_error(fmt::format(
        "Unsupported file format in unittest for file ({})", filename));
  }

  ModelInstanceIndex AddModelFromSdfFile(
      const std::string& file_name, const std::string& model_name,
      const std::optional<std::string>& parent_model_name = {}) {
    const DataSource data_source{DataSource::kFilename, &file_name};
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_, nullptr,
                       &plant_,  &resolver,    TestingSelect};
    std::optional<ModelInstanceIndex> result =
        AddModelFromSdf(data_source, model_name, parent_model_name, w);
    EXPECT_TRUE(result.has_value());
    last_parsed_groups_ = ConvertInstancedNamesToStrings(
        resolver.Resolve(diagnostic_policy_), plant_);
    return result.value_or(ModelInstanceIndex{});
  }

  std::vector<ModelInstanceIndex> AddModelsFromSdfFile(
      const std::string& file_name,
      const std::optional<std::string>& parent_model_name = {}) {
    const DataSource data_source{DataSource::kFilename, &file_name};
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_, nullptr,
                       &plant_,  &resolver,    TestingSelect};
    auto result = AddModelsFromSdf(data_source, parent_model_name, w);
    last_parsed_groups_ = ConvertInstancedNamesToStrings(
        resolver.Resolve(diagnostic_policy_), plant_);
    return result;
  }

  std::vector<ModelInstanceIndex> AddModelsFromSdfString(
      const std::string& file_contents,
      const std::optional<std::string>& parent_model_name = {}) {
    const DataSource data_source{DataSource::kContents, &file_contents};
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_, nullptr,
                       &plant_,  &resolver,    TestingSelect};
    auto result = AddModelsFromSdf(data_source, parent_model_name, w);
    last_parsed_groups_ = ConvertInstancedNamesToStrings(
        resolver.Resolve(diagnostic_policy_), plant_);
    return result;
  }

  void ParseTestString(
      const std::string& inner,
      const std::optional<std::string>& sdf_version = {},
      const std::optional<std::string>& parent_model_name = {}) {
    SCOPED_TRACE(inner);
    FlushDiagnostics();
    const std::string file_contents = "<sdf version='" +
                                      sdf_version.value_or("1.6") + "'>" +
                                      inner + "\n</sdf>\n";
    AddModelsFromSdfString(file_contents, parent_model_name);
  }

  void VerifyCollisionFilters(const std::vector<GeometryId>& ids,
                              const std::set<CollisionPair>& expected_filters) {
    const int num_links = ids.size();
    const auto& inspector = scene_graph_.model_inspector();
    for (int m = 0; m < num_links; ++m) {
      const std::string& m_name = inspector.GetName(ids[m]);
      for (int n = m + 1; n < num_links; ++n) {
        const std::string& n_name = inspector.GetName(ids[n]);
        SCOPED_TRACE(fmt::format("{}[{}] vs {}[{}]", m_name, m, n_name, n));
        CollisionPair names{m_name, n_name};
        auto contains = [&expected_filters](const CollisionPair& key) {
          return expected_filters.contains(key);
        };
        EXPECT_EQ(inspector.CollisionFiltered(ids[m], ids[n]), contains(names));
      }
    }
  }

 protected:
  ParsingOptions options_;
  PackageMap package_map_;
  DiagnosticPolicy diagnostic_;
  MultibodyPlant<double> plant_{0.01};
  SceneGraph<double> scene_graph_;
  CollisionFilterGroupsImpl<std::string> last_parsed_groups_;
};

const Frame<double>& GetModelFrameByName(const MultibodyPlant<double>& plant,
                                         const std::string& name) {
  const auto model_instance = plant.GetModelInstanceByName(name);
  return plant.GetFrameByName("__model__", model_instance);
}

// Verifies that the SDF loader can leverage a specified package map.
TEST_F(SdfParserTest, PackageMapSpecified) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  ASSERT_EQ(plant_.num_model_instances(), 2);

  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/sdfs/box.sdf");
  std::filesystem::path package_path = full_sdf_filename;
  package_path = package_path.parent_path();
  package_path = package_path.parent_path();

  // Construct the PackageMap.
  package_map_.PopulateFromFolder(package_path.string());

  // Read in the SDF file.
  AddModelFromSdfFile(full_sdf_filename, "");
  plant_.Finalize();

  // Verify the number of model instances.
  EXPECT_EQ(plant_.num_model_instances(), 3);
}

// Acceptance test that libsdformat can upgrade very old files.  This ensures
// the upgrade machinery keeps working (in particular our re-implementation of
// the embedSdf.rb tool within tools/workspace/sdformat).
TEST_F(SdfParserTest, VeryOldVersion) {
  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/very_old_version.sdf");

  EXPECT_EQ(plant_.num_model_instances(), 2);
  AddModelFromSdfFile(full_sdf_filename, "");
  plant_.Finalize();
  EXPECT_EQ(plant_.num_model_instances(), 3);
}

// Verifies model instances are correctly created in the plant.
TEST_F(SdfParserTest, ModelInstanceTest) {
  // We start with the world and default model instances (model_instance.h
  // explains why there are two).
  ASSERT_EQ(plant_.num_model_instances(), 2);

  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "links_with_visuals_and_collisions.sdf");

  ModelInstanceIndex instance1 = AddModelFromSdfFile(full_name, "instance1");

  // Check that a duplicate model names are not allowed.
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromSdfFile(full_name, "instance1"),
      "This model already contains a model instance named 'instance1'. "
      "Model instance names must be unique within a given model.");

  // Load two acrobots to check per-model-instance items.
  const std::string acrobot_sdf_name =
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf");
  ModelInstanceIndex acrobot1 = AddModelFromSdfFile(acrobot_sdf_name, "");

  // Loading the model again without specifying a different model name should
  // throw.
  EXPECT_THROW(AddModelFromSdfFile(acrobot_sdf_name, ""), std::logic_error);

  // Avoid name collisions with model renaming.
  ModelInstanceIndex acrobot2 =
      AddModelFromSdfFile(acrobot_sdf_name, "acrobot2");

  // Avoid name collisions with parent model names; both entry points.
  ModelInstanceIndex acrobot3 = AddModelFromSdfFile(acrobot_sdf_name, "", "3");
  ModelInstanceIndex acrobot3rename =
      AddModelFromSdfFile(acrobot_sdf_name, "new_model_name", "3");
  ModelInstanceIndex acrobot4 =
      AddModelsFromSdfFile(acrobot_sdf_name, "4").at(0);

  // We are done adding models.
  plant_.Finalize();

  ASSERT_EQ(plant_.num_model_instances(), 8);
  EXPECT_EQ(plant_.GetModelInstanceByName("instance1"), instance1);
  EXPECT_EQ(plant_.GetModelInstanceByName("acrobot"), acrobot1);
  EXPECT_EQ(plant_.GetModelInstanceByName("acrobot2"), acrobot2);
  EXPECT_EQ(plant_.GetModelInstanceByName("3::acrobot"), acrobot3);
  EXPECT_EQ(plant_.GetModelInstanceByName("3::new_model_name"), acrobot3rename);
  EXPECT_EQ(plant_.GetModelInstanceByName("4::acrobot"), acrobot4);

  // Check a couple links from the first model without specifying the model
  // instance.
  EXPECT_TRUE(plant_.HasBodyNamed("link3"));
  EXPECT_FALSE(plant_.HasBodyNamed("link_which_doesnt_exist"));

  // Links which appear in multiple model instances throw if the instance
  // isn't specified.
  DRAKE_EXPECT_THROWS_MESSAGE(plant_.HasBodyNamed("Link1"),
                              ".*Body.*Link1.*multiple model instances.*");

  EXPECT_FALSE(plant_.HasBodyNamed("Link1", instance1));
  EXPECT_TRUE(plant_.HasBodyNamed("Link1", acrobot1));
  EXPECT_TRUE(plant_.HasBodyNamed("Link1", acrobot2));

  const RigidBody<double>& acrobot1_link1 =
      plant_.GetBodyByName("Link1", acrobot1);
  const RigidBody<double>& acrobot2_link1 =
      plant_.GetBodyByName("Link1", acrobot2);
  EXPECT_NE(acrobot1_link1.index(), acrobot2_link1.index());
  EXPECT_EQ(acrobot1_link1.model_instance(), acrobot1);
  EXPECT_EQ(acrobot2_link1.model_instance(), acrobot2);

  DRAKE_EXPECT_THROWS_MESSAGE(plant_.GetBodyByName("Link1"),
                              ".*Body.*Link1.*multiple model instances.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.HasJointNamed("ShoulderJoint"),
      ".*Joint.*ShoulderJoint.*multiple model instances.*");
  EXPECT_FALSE(plant_.HasJointNamed("ShoulderJoint", instance1));
  EXPECT_TRUE(plant_.HasJointNamed("ShoulderJoint", acrobot1));
  EXPECT_TRUE(plant_.HasJointNamed("ShoulderJoint", acrobot2));

  const Joint<double>& acrobot1_joint =
      plant_.GetJointByName("ShoulderJoint", acrobot1);
  const Joint<double>& acrobot2_joint =
      plant_.GetJointByName("ShoulderJoint", acrobot2);
  EXPECT_NE(acrobot1_joint.index(), acrobot2_joint.index());
  EXPECT_EQ(acrobot1_joint.model_instance(), acrobot1);
  EXPECT_EQ(acrobot2_joint.model_instance(), acrobot2);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.GetJointByName("ShoulderJoint"),
      ".*Joint.*ShoulderJoint.*multiple model instances.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.HasJointActuatorNamed("ElbowJoint"),
      ".*JointActuator.*ElbowJoint.*multiple model instances.*");

  const JointActuator<double>& acrobot1_actuator =
      plant_.GetJointActuatorByName("ElbowJoint", acrobot1);
  const JointActuator<double>& acrobot2_actuator =
      plant_.GetJointActuatorByName("ElbowJoint", acrobot2);
  EXPECT_NE(acrobot1_actuator.index(), acrobot2_actuator.index());

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.GetJointActuatorByName("ElbowJoint"),
      ".*JointActuator.*ElbowJoint.*multiple model instances.*");

  const Frame<double>& acrobot1_link1_frame =
      plant_.GetFrameByName("Link1", acrobot1);
  const Frame<double>& acrobot2_link1_frame =
      plant_.GetFrameByName("Link1", acrobot2);
  EXPECT_NE(acrobot1_link1_frame.index(), acrobot2_link1_frame.index());

  DRAKE_EXPECT_THROWS_MESSAGE(plant_.GetFrameByName("Link1"),
                              ".*Frame.*Link1.*multiple model instances.*");

  // Check model scope frames.
  auto context = plant_.CreateDefaultContext();
  auto check_frame = [this, instance1, &context](
                         std::string parent_name, std::string name,
                         const RigidTransformd& X_PF_expected) {
    const Frame<double>& frame = plant_.GetFrameByName(name, instance1);
    const Frame<double>& parent_frame =
        plant_.GetFrameByName(parent_name, instance1);
    const RigidTransformd X_PF =
        plant_.CalcRelativeTransform(*context, parent_frame, frame);
    EXPECT_TRUE(CompareMatrices(X_PF_expected.GetAsMatrix4(),
                                X_PF.GetAsMatrix4(), kEps))
        << name;
  };

  const RigidTransformd X_L1F1(RollPitchYawd(0.4, 0.5, 0.6),
                               Vector3d(0.1, 0.2, 0.3));
  check_frame("link1", "model_scope_link1_frame", X_L1F1);
  const RigidTransformd X_F1F2(Vector3d(0.1, 0.0, 0.0));
  check_frame("model_scope_link1_frame", "model_scope_link1_frame_child",
              X_F1F2);
  const RigidTransformd X_MF3(Vector3d(0.7, 0.8, 0.9));
  check_frame("__model__", "model_scope_model_frame_implicit", X_MF3);
}

TEST_F(SdfParserTest, ParentModelNameWithString) {
  std::string model = "<model name='something'><link name='link'/></model>";
  ParseTestString(model);
  // Add a parent model name to avoid name collisions.
  ParseTestString(model, {}, "2");
  plant_.Finalize();
  ASSERT_EQ(plant_.num_model_instances(), 4);
  EXPECT_NO_THROW(plant_.GetModelInstanceByName("something"));
  EXPECT_NO_THROW(plant_.GetModelInstanceByName("2::something"));
}

TEST_F(SdfParserTest, ParentModelNameWithStringWorld) {
  std::string model = R"""(
<world name='here'>
  <model name='a'><link name='link'/></model>
  <model name='b'><link name='link'/></model>
</world>
)""";
  ParseTestString(model);
  // Add a parent model name to avoid name collisions.
  ParseTestString(model, {}, "2");
  plant_.Finalize();
  ASSERT_EQ(plant_.num_model_instances(), 6);
  EXPECT_NO_THROW(plant_.GetModelInstanceByName("a"));
  EXPECT_NO_THROW(plant_.GetModelInstanceByName("b"));
  EXPECT_NO_THROW(plant_.GetModelInstanceByName("2::a"));
  EXPECT_NO_THROW(plant_.GetModelInstanceByName("2::b"));
}

TEST_F(SdfParserTest, EntireInertialTagOmitted) {
  AddSceneGraph();
  // Test that parsing a link with no inertial tag yields the expected result
  // (mass = 1, ixx = iyy = izz = 1, ixy = ixz = iyz = 0).
  // TODO(avalenzu): Re-visit this if the SDF spec changes to allow for more
  // parsimonious specification of massless links. See #13903 for more details.
  ParseTestString(R"""(
<model name='entire_inertial_tag_omitted'>
  <link name='entire_inertial_tag_omitted'/>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
      &plant_.GetBodyByName("entire_inertial_tag_omitted"));
  EXPECT_EQ(body->default_mass(), 1.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isOnes());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

TEST_F(SdfParserTest, InertiaTagOmitted) {
  AddSceneGraph();
  // Test that parsing a link with no inertia tag yields the expected result
  // (mass as specified, ixx = iyy = izz = 1, ixy = ixz = iyz = 0).
  // TODO(avalenzu): Re-visit this if the SDF spec changes to allow for more
  // parsimonious specification of massless links. See #13903 for more details.
  ParseTestString(R"""(
<model name='inertia_tag_omitted'>
  <link name='inertia_tag_omitted'>
    <inertial>
      <mass>2</mass>
    </inertial>
  </link>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
      &plant_.GetBodyByName("inertia_tag_omitted"));
  EXPECT_EQ(body->default_mass(), 2.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isOnes());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

TEST_F(SdfParserTest, MassTagOmitted) {
  AddSceneGraph();
  // Test that parsing a link with no mass tag yields the expected result
  // (mass = 1, inertia as specified).
  // TODO(avalenzu): Re-visit this if the SDF spec changes to allow for more
  // parsimonious specification of massless links. See #13903 for more details.
  ParseTestString(R"""(
<model name='mass_tag_omitted'>
  <link name='mass_tag_omitted'>
    <inertial>
      <inertia>
        <ixx>1</ixx>
        <ixy>0.1</ixy>
        <ixz>0.1</ixz>
        <iyy>1</iyy>
        <iyz>0.1</iyz>
        <izz>1</izz>
      </inertia>
    </inertial>
  </link>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
      &plant_.GetBodyByName("mass_tag_omitted"));
  EXPECT_EQ(body->default_mass(), 1.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isOnes());
  EXPECT_EQ(body->default_rotational_inertia().get_products(),
            Vector3d::Constant(0.1));
}

TEST_F(SdfParserTest, MasslessBody) {
  AddSceneGraph();
  // Test that massless bodies can be parsed.
  ParseTestString(R"""(
<model name='has_massless_link'>
  <link name='massless_link'>
    <inertial>
      <mass>0</mass>
      <inertia>
        <ixx>0</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0</iyy>
        <iyz>0</iyz>
        <izz>0</izz>
      </inertia>
    </inertial>
  </link>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
      &plant_.GetBodyByName("massless_link"));
  EXPECT_EQ(body->default_mass(), 0.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

TEST_F(SdfParserTest, PointMass) {
  AddSceneGraph();
  // Test that point masses don't get sent through the massless body branch.
  ParseTestString(R"""(
<model name='point_mass'>
  <link name='point_mass'>
    <inertial>
      <mass>1</mass>
      <inertia>
        <ixx>0</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0</iyy>
        <iyz>0</iyz>
        <izz>0</izz>
      </inertia>
    </inertial>
  </link>
</model>)""");
  const RigidBody<double>* body = dynamic_cast<const RigidBody<double>*>(
      &plant_.GetBodyByName("point_mass"));
  EXPECT_EQ(body->default_mass(), 1.);
  EXPECT_TRUE(body->default_rotational_inertia().get_moments().isZero());
  EXPECT_TRUE(body->default_rotational_inertia().get_products().isZero());
}

TEST_F(SdfParserTest, ZeroMassNonZeroInertia) {
  // Test that attempt to parse links with zero mass and non-zero inertia fails.
  AddSceneGraph();
  const std::string expected_message = ".*condition 'mass > 0' failed.";
  ParseTestString(R"""(
<model name='bad'>
  <link name='bad'>
    <inertial>
      <mass>0</mass>
      <inertia>
        <ixx>1</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>1</iyy>
        <iyz>0</iyz>
        <izz>1</izz>
      </inertia>
    </inertial>
  </link>
</model>)""");
  EXPECT_THAT(TakeWarning(), MatchesRegex(expected_message));
}

TEST_F(SdfParserTest, FloatingBodyPose) {
  // Test that floating bodies (links) still have their poses preserved.
  AddSceneGraph();
  ParseTestString(R"""(
<model name='good'>
  <link name='a'>
    <pose>1 2 3  0.1 0.2 0.3</pose>
  </link>
  <link name='b'>
    <pose>4 5 6  0.4 0.5 0.6</pose>
  </link>
</model>)""");
  plant_.Finalize();
  EXPECT_GT(plant_.num_positions(), 0);
  auto context = plant_.CreateDefaultContext();
  const RigidTransformd X_WA_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                      Vector3d(1, 2, 3));
  const RigidTransformd X_WA =
      plant_.GetFrameByName("a").CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WA_expected.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
  const RigidTransformd X_WB_expected(RollPitchYawd(0.4, 0.5, 0.6),
                                      Vector3d(4, 5, 6));
  const RigidTransformd X_WB =
      plant_.GetFrameByName("b").CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WB_expected.GetAsMatrix4(), X_WB.GetAsMatrix4(), kEps));
}

TEST_F(SdfParserTest, StaticModelSupported1) {
  // Test that static models are partially supported.
  ParseTestString(R"""(
  <model name='good'>
    <static>true</static>
    <link name='a'>
      <pose>1 2 3  0.1 0.2 0.3</pose>
    </link>
    <link name='b'>
      <pose>4 5 6  0.4 0.5 0.6</pose>
    </link>
  </model>)""");
  plant_.Finalize();
  EXPECT_EQ(plant_.num_positions(), 0);
  auto context = plant_.CreateDefaultContext();
  const RigidTransformd X_WA_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                      Vector3d(1, 2, 3));
  const RigidTransformd X_WA =
      plant_.GetFrameByName("a").CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WA_expected.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
  const RigidTransformd X_WB_expected(RollPitchYawd(0.4, 0.5, 0.6),
                                      Vector3d(4, 5, 6));
  const RigidTransformd X_WB =
      plant_.GetFrameByName("b").CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WB_expected.GetAsMatrix4(), X_WB.GetAsMatrix4(), kEps));
}

TEST_F(SdfParserTest, StaticModelSupported2) {
  // Verify that static models don't need to have a canonical link.
  AddSceneGraph();
  ParseTestString(R"""(
  <model name='a'>
    <pose>1 2 3  0.1 0.2 0.3</pose>
    <static>true</static>
  </model>)""",
                  "1.8");
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();
  const RigidTransformd X_WA_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                      Vector3d(1, 2, 3));

  const auto& frame_A = GetModelFrameByName(plant_, "a");
  const RigidTransformd X_WA = frame_A.CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WA_expected.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
  EXPECT_EQ(frame_A.body().index(), plant_.world_body().index());
}

TEST_F(SdfParserTest, StaticModelSupported3) {
  // Verify that models that contain static models don't need a link.
  AddSceneGraph();
  ParseTestString(R"""(
  <model name='a'>
    <pose>1 2 3  0.0 0.0 0.3</pose>
    <model name='b'>
      <pose>0 0 0  0.1 0.2 0.0</pose>
      <static>true</static>
    </model>
  </model>)""",
                  "1.8");
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();
  const RigidTransformd X_WA_expected(RollPitchYawd(0.0, 0.0, 0.3),
                                      Vector3d(1, 2, 3));

  const auto& frame_A = GetModelFrameByName(plant_, "a");
  const RigidTransformd X_WA = frame_A.CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WA_expected.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
  EXPECT_EQ(frame_A.body().index(), plant_.world_body().index());

  const RigidTransformd X_WB_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                      Vector3d(1, 2, 3));

  const auto& frame_B = GetModelFrameByName(plant_, "a::b");
  const RigidTransformd X_WB = frame_B.CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WB_expected.GetAsMatrix4(), X_WB.GetAsMatrix4(), kEps));
  EXPECT_EQ(frame_B.body().index(), plant_.world_body().index());
}

TEST_F(SdfParserTest, StaticFrameOnlyModelsSupported) {
  AddSceneGraph();
  // Verify that static models can contain just frames.
  ParseTestString(R"""(
  <model name='a'>
    <static>true</static>
    <pose>1 0 0  0 0 0</pose>
    <frame name='b'>
      <pose>0 2 0 0 0 0</pose>
    </frame>
    <frame name='c' attached_to='b'>
      <pose>0 0 3 0 0 0</pose>
    </frame>
    <frame name='d'>
      <pose relative_to='c'>0 0 0 0 0 0.3</pose>
    </frame>
  </model>)""",
                  "1.8");
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  auto test_frame = [&](const std::string& frame_name,
                        const RigidTransformd& X_WF_expected) {
    const auto& frame = plant_.GetFrameByName(frame_name);
    const RigidTransformd X_WF = frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WF_expected.GetAsMatrix4(),
                                X_WF.GetAsMatrix4(), kEps));
    EXPECT_EQ(frame.body().index(), plant_.world_body().index());
  };

  test_frame("__model__", {RollPitchYawd(0.0, 0.0, 0.0), Vector3d(1, 0, 0)});
  test_frame("b", {RollPitchYawd(0.0, 0.0, 0.0), Vector3d(1, 2, 0)});
  test_frame("c", {RollPitchYawd(0.0, 0.0, 0.0), Vector3d(1, 2, 3)});
  test_frame("d", {RollPitchYawd(0.0, 0.0, 0.3), Vector3d(1, 2, 3)});
}

TEST_F(SdfParserTest, StaticModelWithJoints) {
  AddSceneGraph();
  // Specifying redundant welds in the model yields no errors, either to the
  // world or between links.
  DRAKE_EXPECT_NO_THROW(ParseTestString(R"""(
<model name='good'>
  <static>true</static>
  <link name='a'/>
  <joint name='a_weld' type='fixed'>
    <parent>world</parent>
    <child>a</child>
  </joint>
  <link name='b'/>
  <joint name='b_weld' type='fixed'>
    <parent>a</parent>
    <child>b</child>
  </joint>
</model>)"""));

  // Attempting to weld should fail fast, either during the welding or when
  // finalizing (due to loop).
  ParseTestString(R"""(
<model name='bad'>
  <static>true</static>
  <link name='a'/>
</model>
)""");
  auto weld_and_finalize = [this]() {
    ModelInstanceIndex bad = plant_.GetModelInstanceByName("bad");
    plant_.WeldFrames(plant_.world_frame(), plant_.GetFrameByName("a", bad));
    plant_.Finalize();
  };
  // The message contains the elaborate joint name inserted by the parser.
  DRAKE_EXPECT_THROWS_MESSAGE(weld_and_finalize(), ".*sdformat_model_static.*");

  // Drake does not support "frozen" joints (#12227).
  ParseTestString(R"""(
<model name='mixed_emotions'>
  <static>true</static>
  <link name='a'/>
  <link name='b'/>
  <joint name='my_hinge' type='revolute'>
    <parent>a</parent>
    <child>b</child>
    <axis>
      <xyz>0 0 1</xyz>
    </axis>
  </joint>
</model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Only fixed joints are permitted in static models."));
}

// Revolute joints should have an axis and 1-dof joints should have no axis2.
TEST_F(SdfParserTest, JointWithNoAxisError) {
  ParseTestString(R"""(
<model name='joint_no_axis'>
  <link name='a'/>
  <link name='b'/>
  <joint name='no_axis' type='revolute'>
    <parent>a</parent>
    <child>b</child>
    <axis2>
      <xyz>0 0 1</xyz>
    </axis2>
  </joint>
</model>)""");
  // Two errors are recorded because of the error diagnostic policy
  // not set to throw. The first error comes from ExtractJointAxis
  // that has no breaking behavior while the second one comes from
  // ParseJointLimits which mimics the throw behavior.
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*An axis must be specified for joint 'no_axis'.*"));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*An axis must be specified for joint 'no_axis'.*"));
}

// Ball joints should not have an axis2.
TEST_F(SdfParserTest, BallJointWithAxis2Error) {
  ParseTestString(R"""(
<model name='joint_no_axis'>
  <link name='a'/>
  <link name='b'/>
  <joint name='should_not_have_axis' type='ball'>
    <parent>a</parent>
    <child>b</child>
    <axis>
      <xyz>0 0 1</xyz>
    </axis>
    <axis2>
      <xyz>0 0 1</xyz>
    </axis2>
  </joint>
</model>)""");
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(".*A ball joint axis will be ignored. Only the dynamic"
                   " parameters and limits will be considered.*"));
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(
          R"(.*Actuation \(via non-zero effort limits\) for ball joint )"
          R"('should_not_have_axis' is not implemented yet and will be )"
          R"(ignored.*)"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*An axis2 may not be specified for "
                                          "ball joint 'should_not_have_axis' "
                                          "and will be ignored.*"));
}

// Joint axis upper limit should be lower than upper limit.
TEST_F(SdfParserTest, JointAxisLimitsError) {
  ParseTestString(R"""(
<model name='joint_no_axis'>
  <link name='a'/>
  <link name='b'/>
  <joint name='no_axis' type='prismatic'>
    <parent>a</parent>
    <child>b</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <lower>2</lower>
        <upper>1</upper>
      </limit>
    </axis>
  </joint>
</model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(R"(.*The lower limit must be lower \(or equal\) than the)"
                   R"( upper limit for joint 'no_axis'.*)"));
}

// Joint axis drake:acceleration should be non negative.
TEST_F(SdfParserTest, JointAxisDrakeAccelerationError) {
  ParseTestString(R"""(
<model name='joint_no_axis'>
  <link name='a'/>
  <link name='b'/>
  <joint name='no_axis' type='prismatic'>
    <parent>a</parent>
    <child>b</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <drake:acceleration>-1</drake:acceleration>
      </limit>
    </axis>
  </joint>
</model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Acceleration limit is negative for joint 'no_axis'."
                   " Aceleration limit must be a non-negative number.*"));
}

// Check that prismatic joints must have an axis.
TEST_F(SdfParserTest, PrismaticJointWithNoAxisError) {
  ParseTestString(R"""(
<model name='joint_no_axis'>
  <link name='a'/>
  <link name='b'/>
  <joint name='no_axis' type='prismatic'>
    <parent>a</parent>
    <child>b</child>
  </joint>
</model>)""");
  // Two errors are recorded because of the error diagnostic policy
  // not set to throw. The first error comes from ExtractJointAxis
  // that has no breaking behavior while the second one comes from
  // ParseJointLimits which mimics the throw behavior.
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*An axis must be specified for joint 'no_axis'.*"));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*An axis must be specified for joint 'no_axis'.*"));
}

// Make sure world joints are fixed.
TEST_F(SdfParserTest, WorldJointNotFixedError) {
  ParseTestString(R"""(
<world name='uno'>
  <joint name='no_axis' type='prismatic'>
    <parent>a</parent>
    <child>b</child>
    <axis>
      <xyz>0 0 1</xyz>
    </axis>
  </joint>
  <model name='a'>
  <link name='l'/>
  </model>
  <model name='b'>
  <link name='h'/>
  </model>
</world>
)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Only fixed joints are permitted in world joints.*"));
}

// drake:joint should have a type.
TEST_F(SdfParserTest, DrakeJointNoTypeError) {
  ParseTestString(R"""(
<model name='good'>
  <link name='a'/>
  <drake:joint>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*<drake:joint>: Unable to find the 'type' attribute.*"));
}

// drake:joint should have a name.
TEST_F(SdfParserTest, DrakeJointNoNameError) {
  ParseTestString(R"""(
<model name='good'>
  <link name='a'/>
  <drake:joint type='nonetype'>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*<drake:joint>: Unable to find the 'name' attribute.*"));
}

// drake:joint does not support pose tags.
TEST_F(SdfParserTest, DrakeJointPoseError) {
  ParseTestString(R"""(
<model name='good'>
  <link name='a'/>
  <drake:joint type='nonetype' name='joint_name'>
    <pose></pose>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          ".*<drake:joint> does not yet support the <pose> child tag.*"));
}

// Verify that drake:joint yields an error for unrecognized types.
TEST_F(SdfParserTest, DrakeJointUnrecognizedTypeError) {
  ParseTestString(R"""(
<model name='good'>
  <link name='a'/>
  <link name='b'/>
  <drake:joint type='nonetype' name='joint_name'>
    <drake:parent>a</drake:parent>
    <drake:child>b</drake:child>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*<drake:joint> 'joint_name' has unrecognized value for"
                   " 'type' attribute: nonetype.*"));
}

// drake:joint/drake:{parent,child} can refer to nested models.
TEST_F(SdfParserTest, DrakeJointNestedParentChild) {
  ParseTestString(R"""(
<model name='good'>
  <model name='nested'>
    <link name='a'/>
    <link name='b'/>
  </model>
  <drake:joint type='planar' name='joint_name'>
    <drake:parent>nested::a</drake:parent>
    <drake:child>nested::b</drake:child>
    <drake:damping>0.1 0.1 0.1</drake:damping>
  </drake:joint>
</model>
)""");
}

// drake:joint/drake:parent yields an error on bad nested model name.
TEST_F(SdfParserTest, DrakeJointNestedParentBad) {
  ParseTestString(R"""(
<model name='good'>
  <model name='nested'>
    <link name='a'/>
    <link name='b'/>
  </model>
  <drake:joint type='planar' name='joint_name'>
    <drake:parent>nesQQQted::a</drake:parent>
    <drake:child>nested::b</drake:child>
    <drake:damping>0.1 0.1 0.1</drake:damping>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          ".*<drake:joint>: Model instance name 'good::nesQQQted' .*implied by"
          " frame name 'nesQQQted::a' in <drake:parent> within model instance"
          " 'good'.* does not exist.*"));
}

// drake:joint/drake:child yields an error on bad nested model name.
TEST_F(SdfParserTest, DrakeJointNestedChildBad) {
  ParseTestString(R"""(
<model name='good'>
  <model name='nested'>
    <link name='a'/>
    <link name='b'/>
  </model>
  <drake:joint type='planar' name='joint_name'>
    <drake:parent>nested::a</drake:parent>
    <drake:child>nesQQQted::b</drake:child>
    <drake:damping>0.1 0.1 0.1</drake:damping>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          ".*<drake:joint>: Model instance name 'good::nesQQQted' .*implied by"
          " frame name 'nesQQQted::b' in <drake:child> within model instance"
          " 'good'.* does not exist.*"));
}

TEST_F(SdfParserTest, MimicSuccessfulParsing) {
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  ParseTestString(R"""(
    <model name='a'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='joint_AB' type='revolute'>
        <parent>A</parent>
        <child>B</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint name='joint_AC' type='revolute'>
        <parent>A</parent>
        <child>C</child>
        <drake:mimic joint='joint_AB' multiplier='1' offset='0.5' />
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
    </model>)""");
  // Revolute joint with mimic
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName("joint_AC"));
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName("joint_AB"));
  const Joint<double>& joint_with_mimic = plant_.GetJointByName("joint_AC");
  const Joint<double>& joint_to_mimic = plant_.GetJointByName("joint_AB");

  EXPECT_EQ(plant_.num_constraints(), 1);
  EXPECT_EQ(plant_.num_coupler_constraints(), 1);
  const internal::CouplerConstraintSpec& spec =
      plant_.get_coupler_constraint_specs().begin()->second;
  EXPECT_EQ(spec.joint0_index, joint_with_mimic.index());
  EXPECT_EQ(spec.joint1_index, joint_to_mimic.index());
  // These must be kept in sync with the values in joint_parsing_test.urdf.
  EXPECT_EQ(spec.gear_ratio, 1);
  EXPECT_EQ(spec.offset, 0.5);
}

TEST_F(SdfParserTest, MimicSuccessfulParsingForwardReference) {
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  ParseTestString(R"""(
    <model name='a'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='joint_AB' type='revolute'>
        <parent>A</parent>
        <child>B</child>
        <drake:mimic joint='joint_AC' multiplier='1' offset='0.5' />
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint name='joint_AC' type='revolute'>
        <parent>A</parent>
        <child>C</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
    </model>)""");
  // Revolute joint with mimic
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName("joint_AB"));
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName("joint_AC"));
  const Joint<double>& joint_with_mimic = plant_.GetJointByName("joint_AB");
  const Joint<double>& joint_to_mimic = plant_.GetJointByName("joint_AC");

  EXPECT_EQ(plant_.num_constraints(), 1);
  EXPECT_EQ(plant_.num_coupler_constraints(), 1);
  const internal::CouplerConstraintSpec& spec =
      plant_.get_coupler_constraint_specs().begin()->second;
  EXPECT_EQ(spec.joint0_index, joint_with_mimic.index());
  EXPECT_EQ(spec.joint1_index, joint_to_mimic.index());
  // These must be kept in sync with the values in joint_parsing_test.urdf.
  EXPECT_EQ(spec.gear_ratio, 1);
  EXPECT_EQ(spec.offset, 0.5);
}

TEST_F(SdfParserTest, MimicNoSap) {
  plant_.set_discrete_contact_approximation(
      DiscreteContactApproximation::kTamsi);
  ParseTestString(R"""(
    <model name='a'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='joint_AB' type='revolute'>
        <parent>A</parent>
        <child>B</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint name='joint_AC' type='revolute'>
        <parent>A</parent>
        <child>C</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <drake:mimic joint='joint_AB' multiplier='1' offset='0.5' />
      </joint>
    </model>)""");

  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(
          ".*Mimic elements are currently only supported by MultibodyPlant "
          "with a discrete time step and using DiscreteContactSolver::kSap."));
}

TEST_F(SdfParserTest, MimicNoJoint) {
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  ParseTestString(R"""(
    <model name='a'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='joint_AB' type='revolute'>
        <parent>A</parent>
        <child>B</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint name='joint_AC' type='revolute'>
        <parent>A</parent>
        <child>C</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <drake:mimic multiplier='1' offset='0.5' />
      </joint>
    </model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Joint 'joint_AC' drake:mimic element is missing the "
                   "required 'joint' attribute."));
}

TEST_F(SdfParserTest, MimicBadJoint) {
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  ParseTestString(R"""(
    <model name='a'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='joint_AB' type='revolute'>
        <parent>A</parent>
        <child>B</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint name='joint_AC' type='revolute'>
        <parent>A</parent>
        <child>C</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <drake:mimic joint='nonexistent' multiplier='1' offset='0.5' />
      </joint>
    </model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Joint 'joint_AC' drake:mimic element specifies joint "
                   "'nonexistent' which does not exist."));
}

TEST_F(SdfParserTest, MimicSameJoint) {
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  ParseTestString(R"""(
    <model name='a'>
      <link name='A'/>
      <link name='B'/>
      <joint name='joint_AB' type='revolute'>
        <parent>A</parent>
        <child>B</child>
        <drake:mimic joint='joint_AB' multiplier='1' offset='0.5' />
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
    </model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Joint 'joint_AB' drake:mimic element specifies joint "
                   "'joint_AB'. Joints cannot mimic themselves."));
}

TEST_F(SdfParserTest, MimicNoMultiplier) {
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  ParseTestString(R"""(
    <model name='a'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='joint_AB' type='revolute'>
        <parent>A</parent>
        <child>B</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint name='joint_AC' type='revolute'>
        <parent>A</parent>
        <child>C</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <drake:mimic joint='joint_AB' offset='0.5' />
      </joint>
    </model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Joint 'joint_AC' drake:mimic element is missing the "
                   "required 'multiplier' attribute."));
}

TEST_F(SdfParserTest, MimicNoOffset) {
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  ParseTestString(R"""(
    <model name='a'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='joint_AB' type='revolute'>
        <parent>A</parent>
        <child>B</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint name='joint_AC' type='revolute'>
        <parent>A</parent>
        <child>C</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <drake:mimic joint='joint_AB' multiplier='1' />
      </joint>
    </model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Joint 'joint_AC' drake:mimic element is missing the "
                   "required 'offset' attribute."));
}

TEST_F(SdfParserTest, MimicOnlyOneDOFJoint) {
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);
  ParseTestString(R"""(
    <model name='a'>
      <link name='A'/>
      <link name='B'/>
      <link name='C'/>
      <joint name='joint_AB' type='fixed'>
        <parent>A</parent>
        <child>B</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint name='joint_AC' type='fixed'>
        <parent>A</parent>
        <child>C</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <drake:mimic joint='joint_AB' multiplier='1' offset='0.5' />
      </joint>
    </model>)""");
  EXPECT_THAT(TakeWarning(),
              MatchesRegex(".*Drake only supports the drake:mimic element for "
                           "single-dof joints.*"));
}

// Verify error when no model is found.
TEST_F(SdfParserTest, AddModelFromSdfNoModelError) {
  const std::string sdf_string = R"""(
<sdf version='1.6'>
  <world name='empty_world'>
  </world>
</sdf>
)""";

  const DataSource data_source{DataSource::kContents, &sdf_string};
  internal::CollisionFilterGroupResolver resolver{&plant_};
  ParsingWorkspace w{options_, package_map_, diagnostic_policy_, nullptr,
                     &plant_,  &resolver,    TestingSelect};
  std::optional<ModelInstanceIndex> result =
      AddModelFromSdf(data_source, "", "", w);
  resolver.Resolve(diagnostic_policy_);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*File must have a single <model> element.*"));
  EXPECT_FALSE(result.has_value());
}

// Verify error thrown when more than one world are present.
TEST_F(SdfParserTest, MoreThanOneWorldOrModelError) {
  ParseTestString(R"""(
<world name='uno'>
</world>
<world name='dos'>
</world>
)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(
                  ".*File must have exactly one <model> or exactly one <world>,"
                  " but instead has 0 models and 2 worlds.*"));
}

// Verify that our SDF parser throws an exception when a user specifies a joint
// with negative damping.
TEST_F(SdfParserTest, ThrowsWhenJointDampingIsNegative) {
  const std::string sdf_file_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "negative_damping_joint.sdf");
  AddModelFromSdfFile(sdf_file_path, "");
  EXPECT_THAT(TakeError(), MatchesRegex(".*damping is negative.*"));
}

TEST_F(SdfParserTest, IncludeTags) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "include_models.sdf");

  // We start with the world and default model instances.
  ASSERT_EQ(plant_.num_model_instances(), 2);
  ASSERT_EQ(plant_.num_bodies(), 1);
  ASSERT_EQ(plant_.num_joints(), 0);

  package_map_.PopulateFromFolder(
      std::filesystem::path(full_name).parent_path());
  AddModelsFromSdfFile(full_name);
  plant_.Finalize();

  // We should have loaded 5 more models.
  EXPECT_EQ(plant_.num_model_instances(), 7);
  // The models should have added 8 more bodies.
  EXPECT_EQ(plant_.num_bodies(), 9);
  // The models should have added 8 more joints.
  EXPECT_EQ(plant_.num_joints(), 8);

  // There should be a model instance with the name "robot1".
  ASSERT_TRUE(plant_.HasModelInstanceNamed("robot1"));
  ModelInstanceIndex robot1_model = plant_.GetModelInstanceByName("robot1");
  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant_.HasBodyNamed("base_link", robot1_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant_.HasBodyNamed("moving_link", robot1_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant_.HasJointNamed("slider", robot1_model));

  // There should be a model instance with the name "robot2".
  ASSERT_TRUE(plant_.HasModelInstanceNamed("robot2"));
  ModelInstanceIndex robot2_model = plant_.GetModelInstanceByName("robot2");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant_.HasBodyNamed("base_link", robot2_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant_.HasBodyNamed("moving_link", robot2_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant_.HasJointNamed("slider", robot2_model));

  // There should be a model instance with the name "weld_robots".
  EXPECT_TRUE(plant_.HasModelInstanceNamed("weld_models"));

  ASSERT_TRUE(plant_.HasModelInstanceNamed("weld_models::robot1"));
  ModelInstanceIndex weld_model_robot1_model =
      plant_.GetModelInstanceByName("weld_models::robot1");

  ASSERT_TRUE(plant_.HasModelInstanceNamed("weld_models::robot2"));
  ModelInstanceIndex weld_model_robot2_model =
      plant_.GetModelInstanceByName("weld_models::robot2");

  // There should be all the bodies and joints contained in "simple_robot1"
  // which is inside "weld_models"
  EXPECT_TRUE(plant_.HasBodyNamed("base_link", weld_model_robot1_model));
  EXPECT_TRUE(plant_.HasBodyNamed("moving_link", weld_model_robot1_model));
  EXPECT_TRUE(plant_.HasJointNamed("slider", weld_model_robot1_model));
  // There should be all the bodies and joints contained in "simple_robot2"
  // which is inside "weld_models"
  EXPECT_TRUE(plant_.HasBodyNamed("base_link", weld_model_robot2_model));
  EXPECT_TRUE(plant_.HasBodyNamed("moving_link", weld_model_robot2_model));
  EXPECT_TRUE(plant_.HasJointNamed("slider", weld_model_robot2_model));
  // There should be a joint named "weld_robots". By convention, the joint
  // will have the same model instance as the child frame.
  EXPECT_TRUE(plant_.HasJointNamed("weld_robots", weld_model_robot2_model));
}

TEST_F(SdfParserTest, TestSceneGraph) {
  AddSceneGraph();
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "links_with_visuals_and_collisions.sdf");
  // Test that having a scene graph results in visual geometries.
  AddModelsFromSdfFile(full_name);
  plant_.Finalize();
  EXPECT_NE(plant_.num_visual_geometries(), 0);
}

// Verifies that the SDF loader can leverage a specified package map.
TEST_F(SdfParserTest, JointParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "joint_parsing_test.sdf");

  // Read in the SDF file.
  const std::vector<ModelInstanceIndex> instances =
      AddModelsFromSdfFile(full_name);
  const ModelInstanceIndex instance1 = instances.front();
  plant_.Finalize();

  // Revolute joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<RevoluteJoint>("revolute_joint", instance1));
  const RevoluteJoint<double>& revolute_joint =
      plant_.GetJointByName<RevoluteJoint>("revolute_joint", instance1);
  EXPECT_EQ(revolute_joint.name(), "revolute_joint");
  EXPECT_EQ(revolute_joint.parent_body().name(), "link1");
  EXPECT_EQ(revolute_joint.child_body().name(), "link2");
  EXPECT_EQ(revolute_joint.revolute_axis(), Vector3d::UnitZ());
  EXPECT_EQ(revolute_joint.default_damping(), 0.2);
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_lower_limits(), Vector1d(-1)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.position_upper_limits(), Vector1d(2)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_lower_limits(), Vector1d(-100)));
  EXPECT_TRUE(
      CompareMatrices(revolute_joint.velocity_upper_limits(), Vector1d(100)));
  EXPECT_TRUE(CompareMatrices(revolute_joint.acceleration_lower_limits(),
                              Vector1d(-200)));
  EXPECT_TRUE(CompareMatrices(revolute_joint.acceleration_upper_limits(),
                              Vector1d(200)));

  // Prismatic joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<PrismaticJoint>("prismatic_joint", instance1));
  const PrismaticJoint<double>& prismatic_joint =
      plant_.GetJointByName<PrismaticJoint>("prismatic_joint", instance1);
  EXPECT_EQ(prismatic_joint.name(), "prismatic_joint");
  EXPECT_EQ(prismatic_joint.parent_body().name(), "link2");
  EXPECT_EQ(prismatic_joint.child_body().name(), "link3");
  EXPECT_EQ(prismatic_joint.translation_axis(), Vector3d::UnitZ());
  EXPECT_EQ(prismatic_joint.default_damping(), 0.3);
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_lower_limits(), Vector1d(-2)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.position_upper_limits(), Vector1d(1)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_lower_limits(), Vector1d(-5)));
  EXPECT_TRUE(
      CompareMatrices(prismatic_joint.velocity_upper_limits(), Vector1d(5)));
  EXPECT_TRUE(CompareMatrices(prismatic_joint.acceleration_lower_limits(),
                              Vector1d(-10)));
  EXPECT_TRUE(CompareMatrices(prismatic_joint.acceleration_upper_limits(),
                              Vector1d(10)));

  // Limitless revolute joint
  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName<RevoluteJoint>(
      "revolute_joint_no_limits", instance1));
  const RevoluteJoint<double>& no_limit_joint =
      plant_.GetJointByName<RevoluteJoint>("revolute_joint_no_limits",
                                           instance1);
  EXPECT_EQ(no_limit_joint.name(), "revolute_joint_no_limits");
  EXPECT_EQ(no_limit_joint.parent_body().name(), "link3");
  EXPECT_EQ(no_limit_joint.child_body().name(), "link4");
  EXPECT_EQ(no_limit_joint.revolute_axis(), Vector3d::UnitZ());
  const Vector1d inf(std::numeric_limits<double>::infinity());
  const Vector1d neg_inf(-std::numeric_limits<double>::infinity());
  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.position_upper_limits(), inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.velocity_upper_limits(), inf));
  EXPECT_TRUE(
      CompareMatrices(no_limit_joint.acceleration_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(no_limit_joint.acceleration_upper_limits(), inf));

  // Ball joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<BallRpyJoint>("ball_joint", instance1));
  const BallRpyJoint<double>& ball_joint =
      plant_.GetJointByName<BallRpyJoint>("ball_joint", instance1);
  EXPECT_EQ(ball_joint.name(), "ball_joint");
  EXPECT_EQ(ball_joint.parent_body().name(), "link4");
  EXPECT_EQ(ball_joint.child_body().name(), "link5");
  EXPECT_EQ(ball_joint.default_damping(), 0.1);
  const Vector3d inf3(std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity());
  const Vector3d neg_inf3(-std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(CompareMatrices(ball_joint.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(ball_joint.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(ball_joint.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(ball_joint.velocity_upper_limits(), inf3));
  // Ball joints with axis produce a waring indicating it only some params
  // of it are used.
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(".*A ball joint axis will be ignored. Only the dynamic"
                   " parameters and limits will be considered.*"));
  FlushDiagnostics();

  // Universal joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<UniversalJoint>("universal_joint", instance1));
  const UniversalJoint<double>& universal_joint =
      plant_.GetJointByName<UniversalJoint>("universal_joint", instance1);
  EXPECT_EQ(universal_joint.name(), "universal_joint");
  EXPECT_EQ(universal_joint.parent_body().name(), "link5");
  EXPECT_EQ(universal_joint.child_body().name(), "link6");
  EXPECT_EQ(universal_joint.default_damping(), 0.1);
  const Vector2d inf2(std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity());
  const Vector2d neg_inf2(-std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity());
  EXPECT_TRUE(
      CompareMatrices(universal_joint.position_lower_limits(), neg_inf2));
  EXPECT_TRUE(CompareMatrices(universal_joint.position_upper_limits(), inf2));
  EXPECT_TRUE(
      CompareMatrices(universal_joint.velocity_lower_limits(), neg_inf2));
  EXPECT_TRUE(CompareMatrices(universal_joint.velocity_upper_limits(), inf2));
  // axis = (0, 0, 1) and axis2 = (0, 1, 0) in the model frame (aka the world
  // frame in this case). So Ix, Iy, Iz are (0, 0, 1), (0, 1, 0), and (-1, 0, 0)
  // respectively when expressed in the world frame. Since M = F = I and C = P =
  // W, We have R_PF = R_CM = R_WI, and R_WI has column vectors Ix_W, Iy_W, and
  // Iz_W.
  Matrix3<double> R_WI;
  // clang-format off
  R_WI << 0, 0, -1,
          0, 1, 0,
          1, 0, 0;
  // clang-format on
  // The intermediate frame I and the joint frame J are off by a rotation and
  // p_WI = p_WJ.
  const Vector3d p_WI(
      1.0, 2.0, 3.0); /* To be kept in sync with the testing SDFormat file. */
  const RigidTransform<double> X_WI =
      RigidTransform<double>(math::RotationMatrix<double>(R_WI), p_WI);
  // In the SDFormat file, the frames on all links are coincident with the
  // world frame. So p_PJ = p_CJ = p_WJ = p_WI. Therefore, X_PJ = X_CJ = X_WJ.
  EXPECT_TRUE(universal_joint.frame_on_parent()
                  .GetFixedPoseInBodyFrame()
                  .IsExactlyEqualTo(X_WI));
  EXPECT_TRUE(universal_joint.frame_on_child()
                  .GetFixedPoseInBodyFrame()
                  .IsExactlyEqualTo(X_WI));

  // Planar joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<PlanarJoint>("planar_joint", instance1));
  const PlanarJoint<double>& planar_joint =
      plant_.GetJointByName<PlanarJoint>("planar_joint", instance1);
  EXPECT_EQ(planar_joint.name(), "planar_joint");
  EXPECT_EQ(planar_joint.parent_body().name(), "link6");
  EXPECT_EQ(planar_joint.parent_body().model_instance(), instance1);
  EXPECT_EQ(planar_joint.child_body().name(), "link7");
  EXPECT_EQ(planar_joint.child_body().model_instance(), instance1);
  EXPECT_TRUE(
      CompareMatrices(planar_joint.default_damping(), Vector3d::Constant(0.1)));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint.velocity_upper_limits(), inf3));

  const ModelInstanceIndex instance2 = instances.back();
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<PlanarJoint>("planar_joint", instance2));
  const PlanarJoint<double>& planar_joint2 =
      plant_.GetJointByName<PlanarJoint>("planar_joint", instance2);
  EXPECT_EQ(planar_joint2.name(), "planar_joint");
  EXPECT_EQ(planar_joint2.parent_body().name(), "link6");
  EXPECT_EQ(planar_joint2.parent_body().model_instance(), instance2);
  EXPECT_EQ(planar_joint2.child_body().name(), "link7");
  EXPECT_EQ(planar_joint2.child_body().model_instance(), instance2);
  EXPECT_TRUE(CompareMatrices(planar_joint2.default_damping(),
                              Vector3d::Constant(0.2)));
  EXPECT_TRUE(CompareMatrices(planar_joint2.position_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint2.position_upper_limits(), inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint2.velocity_lower_limits(), neg_inf3));
  EXPECT_TRUE(CompareMatrices(planar_joint2.velocity_upper_limits(), inf3));

  // Continuous joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<RevoluteJoint>("continuous_joint", instance1));
  const RevoluteJoint<double>& continuous_joint =
      plant_.GetJointByName<RevoluteJoint>("continuous_joint", instance1);
  EXPECT_EQ(continuous_joint.name(), "continuous_joint");
  EXPECT_EQ(continuous_joint.parent_body().name(), "link7");
  EXPECT_EQ(continuous_joint.child_body().name(), "link8");
  EXPECT_EQ(continuous_joint.revolute_axis(), Vector3d::UnitZ());
  EXPECT_TRUE(
      CompareMatrices(continuous_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(continuous_joint.position_upper_limits(), inf));
  EXPECT_TRUE(
      CompareMatrices(continuous_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(continuous_joint.velocity_upper_limits(), inf));
  EXPECT_TRUE(
      CompareMatrices(continuous_joint.acceleration_lower_limits(), neg_inf));
  EXPECT_TRUE(
      CompareMatrices(continuous_joint.acceleration_upper_limits(), inf));

  // Screw joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<ScrewJoint>("screw_joint", instance1));
  const ScrewJoint<double>& screw_joint =
      plant_.GetJointByName<ScrewJoint>("screw_joint", instance1);
  EXPECT_EQ(screw_joint.name(), "screw_joint");
  EXPECT_EQ(screw_joint.parent_body().name(), "link8");
  EXPECT_EQ(screw_joint.child_body().name(), "link9");
  EXPECT_EQ(screw_joint.screw_axis(), Vector3d::UnitX());
  EXPECT_EQ(screw_joint.screw_pitch(), 0.04);
  EXPECT_EQ(screw_joint.default_damping(), 0.1);
  EXPECT_TRUE(CompareMatrices(screw_joint.position_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(screw_joint.position_upper_limits(), inf));
  EXPECT_TRUE(CompareMatrices(screw_joint.velocity_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(screw_joint.velocity_upper_limits(), inf));
  EXPECT_TRUE(
      CompareMatrices(screw_joint.acceleration_lower_limits(), neg_inf));
  EXPECT_TRUE(CompareMatrices(screw_joint.acceleration_upper_limits(), inf));

  // periodic curvilinear joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<CurvilinearJoint>("curvilinear_joint_periodic"));
  const CurvilinearJoint<double>& curvilinear_joint =
      plant_.GetJointByName<CurvilinearJoint>("curvilinear_joint_periodic");
  EXPECT_EQ(curvilinear_joint.name(), "curvilinear_joint_periodic");
  EXPECT_EQ(curvilinear_joint.parent_body().name(), "link9");
  EXPECT_EQ(curvilinear_joint.child_body().name(), "link10");
  EXPECT_EQ(curvilinear_joint.default_damping(), 0.1);
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint.position_lower_limits(),
                      Vector1d(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint.position_upper_limits(),
                      Vector1d(std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint.velocity_lower_limits(),
                      Vector1d(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint.velocity_upper_limits(),
                      Vector1d(std::numeric_limits<double>::infinity())));
  const PiecewiseConstantCurvatureTrajectory<double> joint_curve =
      curvilinear_joint.get_trajectory();
  EXPECT_EQ(joint_curve.is_periodic(), true);
  std::vector<double> breaks_expected{0., 1., 1. + M_PI, 1. + 2 * M_PI};
  std::vector<double> turning_rates_expected{0., 0.5, -0.5};

  PiecewiseConstantCurvatureTrajectory<double> joint_curve_expected{
      breaks_expected,   turning_rates_expected, Vector3d::UnitZ(),
      Vector3d::UnitX(), Vector3d::Zero(),       true};
  const double curve_length = joint_curve.length();
  const double kEpsilon = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(curve_length, joint_curve_expected.length(), kEpsilon);
  EXPECT_TRUE(joint_curve.CalcPose(0.).IsNearlyEqualTo(
      joint_curve_expected.CalcPose(0.), kEpsilon));
  EXPECT_TRUE(joint_curve.CalcPose(curve_length - kEpsilon)
                  .IsNearlyEqualTo(
                      joint_curve_expected.CalcPose(curve_length - kEpsilon),
                      kEpsilon));
  EXPECT_TRUE(
      joint_curve.CalcPose(curve_length * 1.5)
          .IsNearlyEqualTo(joint_curve_expected.CalcPose(curve_length * 1.5),
                           kEpsilon));

  // non-periodic curvilinear planar joint
  DRAKE_EXPECT_NO_THROW(
      plant_.GetJointByName<CurvilinearJoint>("curvilinear_joint_aperiodic"));
  const CurvilinearJoint<double>& curvilinear_joint2 =
      plant_.GetJointByName<CurvilinearJoint>("curvilinear_joint_aperiodic");
  const PiecewiseConstantCurvatureTrajectory<double> joint2_curve =
      curvilinear_joint2.get_trajectory();
  EXPECT_EQ(curvilinear_joint2.name(), "curvilinear_joint_aperiodic");
  EXPECT_EQ(curvilinear_joint2.parent_body().name(), "link10");
  EXPECT_EQ(curvilinear_joint2.child_body().name(), "link11");
  EXPECT_EQ(curvilinear_joint2.default_damping(), 0.1);
  EXPECT_TRUE(CompareMatrices(curvilinear_joint2.position_lower_limits(),
                              Vector1d(0.)));
  EXPECT_TRUE(CompareMatrices(curvilinear_joint2.position_upper_limits(),
                              Vector1d(joint2_curve.length())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint2.velocity_lower_limits(),
                      Vector1d(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(
      CompareMatrices(curvilinear_joint2.velocity_upper_limits(),
                      Vector1d(std::numeric_limits<double>::infinity())));
  PiecewiseConstantCurvatureTrajectory<double> joint2_curve_expected{
      breaks_expected,   turning_rates_expected, Vector3d::UnitZ(),
      Vector3d::UnitX(), Vector3d::Zero(),       false};
  EXPECT_EQ(joint2_curve.is_periodic(), false);
  const double curve2_length = joint2_curve.length();
  EXPECT_NEAR(curve2_length, joint2_curve_expected.length(), kEpsilon);
  EXPECT_TRUE(joint2_curve.CalcPose(0.).IsNearlyEqualTo(
      joint2_curve_expected.CalcPose(0.), kEpsilon));
  EXPECT_TRUE(joint2_curve.CalcPose(curve_length - kEpsilon)
                  .IsNearlyEqualTo(
                      joint2_curve_expected.CalcPose(curve_length - kEpsilon),
                      kEpsilon));
  EXPECT_TRUE(
      joint2_curve.CalcPose(curve_length * 1.5)
          .IsNearlyEqualTo(joint2_curve_expected.CalcPose(curve_length * 1.5),
                           kEpsilon));
}

// Tests the error handling for an unsupported joint type (when actuated).
TEST_F(SdfParserTest, ActuatedUniversalJointParsingTest) {
  ParseTestString(R"""(
<model name="molly">
  <link name="larry" />
  <joint name="jerry" type="universal">
    <parent>world</parent>
    <child>larry</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <effort>100</effort>
      </limit>
    </axis>
    <axis2>
      <xyz>0 1 0</xyz>
      <limit>
        <effort>100</effort>
      </limit>
    </axis2>
  </joint>
</model>)""");
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(".*effort limits.*universal joint.*not implemented.*"));
}

// Tests the error handling when axis2 isn't specified for universal joints.
TEST_F(SdfParserTest, UniversalJointAxisParsingTest) {
  ParseTestString(R"""(
<model name="molly">
  <link name="larry" />
  <joint name="jerry" type="universal">
    <parent>world</parent>
    <child>larry</child>
    <axis>
      <xyz> 0 0 1</xyz>
    </axis>
  </joint>
</model>)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Both axis and axis2 must be specified.*jerry.*"));
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(".*effort limits.*universal joint.*not implemented.*"));
}

// Tests the error handling for an non-orthogonal axis and axis2 in universal
// joints.
TEST_F(SdfParserTest, UniversalJointNonOrthogonalAxisParsingTest) {
  ParseTestString(R"""(
<model name="molly">
  <link name="larry" />
  <joint name="jerry" type="universal">
    <parent>world</parent>
    <child>larry</child>
    <axis>
      <xyz>0 0 1</xyz>
    </axis>
    <axis2>
      <xyz>0 0 1</xyz>
    </axis2>
  </joint>
</model>)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*axis and axis2 must be orthogonal.*jerry.*"));
}

// Tests the error handling for axis and axis2 with incompatible damping
// coefficients in universal joints.
TEST_F(SdfParserTest, UniversalJointDampingCoeffParsingTest) {
  ParseTestString(R"""(
<model name="molly">
  <link name="larry" />
  <joint name="jerry" type="universal">
    <parent>world</parent>
    <child>larry</child>
    <axis>
      <limit>
        <effort>0</effort>
      </limit>
      <dynamics>
        <damping>0.1</damping>
      </dynamics>
      <xyz>0 0 1</xyz>
    </axis>
    <axis2>
      <limit>
        <effort>0</effort>
      </limit>
      <dynamics>
        <damping>0.2</damping>
      </dynamics>
      <xyz>0 1 0</xyz>
    </axis2>
  </joint>
</model>)""");
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(
          ".*damping must be equal.*jerry.*damping coefficient.*0.1.*is "
          "used.*0.2.*is ignored.*should be explicitly defined as 0.1 to "
          "match.*"));
}

// Tests the error handling for an unsupported joint type (when actuated).
TEST_F(SdfParserTest, ActuatedBallJointParsingTest) {
  ParseTestString(R"""(
<model name="molly">
  <link name="larry" />
  <joint name="jerry" type="ball">
    <parent>world</parent>
    <child>larry</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <effort>100</effort>
      </limit>
    </axis>
  </joint>
</model>)""");
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(".*A ball joint axis will be ignored. Only the dynamic"
                   " parameters and limits will be considered.*"));
  EXPECT_THAT(TakeWarning(),
              MatchesRegex(".*effort limits.*ball joint.*not implemented.*"));
}

// Tests the error handling for an unsupported joint type.
TEST_F(SdfParserTest, GearboxJointParsingTest) {
  ParseTestString(R"""(
<model name="molly">
  <link name="larry" />
  <joint name="jerry" type="gearbox">
    <parent>world</parent>
    <child>larry</child>
  </joint>
</model>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*gearbox.*not supported.*jerry.*"));
}

// Tests the error handling for an unsupported joint type.
TEST_F(SdfParserTest, Revolute2JointParsingTest) {
  ParseTestString(R"""(
<model name="molly">
  <link name="larry" />
  <joint name="jerry" type="revolute2">
    <parent>world</parent>
    <child>larry</child>
  </joint>
</model>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*revolute2.*not supported.*jerry.*"));
}

// Tests the error handling for a misspelled joint type.
TEST_F(SdfParserTest, MisspelledJointParsingTest) {
  ParseTestString(R"""(
<model name="molly">
  <link name="larry" />
  <joint name="jerry" type="revoluteQQQ">
    <parent>world</parent>
    <child>larry</child>
  </joint>
</model>)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*revoluteqqq is invalid.*"));
}

// Verifies that the SDF parser parses the joint actuator limit correctly.
TEST_F(SdfParserTest, JointActuatorParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "joint_actuator_parsing_test.sdf");

  // Read in the SDF file.
  AddModelFromSdfFile(full_name, "");
  plant_.Finalize();

  // In SDF, effort limits are specified in <joint><axis><limit><effort>,
  // which is the reason we read the joint actuator using the joint name.
  // Test the joint actuator with a positive effort limit.
  const auto& limited_joint_actuator =
      plant_.GetJointActuatorByName("revolute_joint_positive_limit");
  EXPECT_EQ(limited_joint_actuator.effort_limit(), 100);

  // Test the joint actuator with the effort limit set to negative value,
  // which will be treated as no limit per the SDF standard.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const auto& no_limit_joint_actuator =
      plant_.GetJointActuatorByName("revolute_joint_no_limit");
  EXPECT_TRUE(no_limit_joint_actuator.effort_limit() == kInf);

  // Test the joint actuator with the effort limit set to 0, which means no
  // actuation.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.GetJointActuatorByName("prismatic_joint_zero_limit"),
      ".*There is no JointActuator named.*");

  Vector2d effort_limits{100, kInf};
  EXPECT_TRUE(CompareMatrices(plant_.GetEffortLowerLimits(), -effort_limits));
  EXPECT_TRUE(CompareMatrices(plant_.GetEffortUpperLimits(), effort_limits));
}

// Verifies that the SDF parser parses the prismatic spring parameters
// correctly.
TEST_F(SdfParserTest, PrismaticSpringParsingTest) {
  ParseTestString(R"""(
  <model name='model_with_prismatic_spring'>
    <link name='a'/>
    <joint name='a_prismatic' type='prismatic'>
      <parent>world</parent>
      <child>a</child>
      <axis>
          <dynamics>
            <spring_reference>1.5</spring_reference>
            <spring_stiffness>5.0</spring_stiffness>
          </dynamics>
      </axis>
    </joint>
  </model>)""");

  // Force Element indexed 1 because Gravity is indexed 0.
  const auto& spring =
      plant_.GetForceElement<PrismaticSpring>(ForceElementIndex(1));

  EXPECT_EQ(spring.joint().name(), "a_prismatic");
  EXPECT_EQ(spring.stiffness(), 5.0);
  EXPECT_EQ(spring.nominal_position(), 1.5);
}

// Verifies that the SDF parser handles bad inputs (negative stiffness)
// correctly
TEST_F(SdfParserTest, NegativeStiffnessPrismaticSpringParsingTest) {
  ParseTestString(R"""(
  <model name='model_with_prismatic_spring'>
    <link name='a'/>
    <joint name='a_prismatic' type='prismatic'>
      <parent>world</parent>
      <child>a</child>
      <axis>
          <dynamics>
            <spring_reference>1.5</spring_reference>
            <spring_stiffness>-5.0</spring_stiffness>
          </dynamics>
      </axis>
    </joint>
  </model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          ".*The stiffness specified for joint '.*' must be non-negative."));
}

// Verifies that the SDF parser parses the revolute spring parameters correctly.
TEST_F(SdfParserTest, RevoluteSpringParsingTest) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "revolute_spring_parsing_test.sdf");

  // Reads in the SDF file.
  AddModelFromSdfFile(full_name, "");
  plant_.Finalize();

  // Plant should have a UniformGravityFieldElement by default.
  // Our test contains two joints that have nonzero stiffness
  // and two joints that have zero stiffness. We only add a
  // spring for nonzero stiffness, so only two spring forces
  // should have been added.
  constexpr int kNumSpringForces = 2;
  DRAKE_DEMAND(plant_.num_force_elements() == kNumSpringForces + 1);

  // In these two tests, we verify that the generalized forces are
  // correct for both springs. The first spring has a nonzero reference
  // of 1.0 radians so should have nonzero torque. The second spring
  // has a zero reference, so it should have no applied torque.
  MultibodyForces<double> forces(plant_);
  auto context = plant_.CreateDefaultContext();
  constexpr int kGeneralizedForcesSize = 10;
  Matrix2X<double> expected_generalized_forces(kNumSpringForces,
                                               kGeneralizedForcesSize);
  expected_generalized_forces << 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0;
  for (int i = 0; i < kNumSpringForces; ++i) {
    // The ForceElement at index zero is gravity, so we skip that index.
    const ForceElementIndex force_index(i + 1);
    const auto& nonzero_reference = plant_.GetForceElement(force_index);
    forces.SetZero();
    nonzero_reference.CalcAndAddForceContribution(
        *context, plant_.EvalPositionKinematics(*context),
        plant_.EvalVelocityKinematics(*context), &forces);

    const VectorX<double>& generalized_forces = forces.generalized_forces();
    EXPECT_TRUE(CompareMatrices(generalized_forces,
                                expected_generalized_forces.row(i).transpose(),
                                kEps, MatrixCompareType::relative));
  }
}

TEST_F(SdfParserTest, CurvilinearJointErrors0) {
  ParseTestString(R"""(
<model name='test'>
  <link name='a'/>
  <link name='b'/>
  <drake:joint name="curvilinear_joint" type="curvilinear">
    <drake:parent>a</drake:parent>
    <drake:child>b</drake:child>
    <drake:is_periodic>TRUE</drake:is_periodic>
    <drake:curves>
      <drake:line_segment>
        <drake:length>1.0</drake:length>
      </drake:line_segment>
    </drake:curves>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*drake:is_periodic.*non-boolean.*"));
}

TEST_F(SdfParserTest, CurvilinearJointErrors1) {
  ParseTestString(R"""(
<model name='test'>
  <link name='a'/>
  <link name='b'/>
  <drake:joint name="curvilinear_joint" type="curvilinear">
    <drake:parent>a</drake:parent>
    <drake:child>b</drake:child>
</drake:joint>
</model>
)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*drake:curves.*child.*"));
}

TEST_F(SdfParserTest, CurvilinearJointErrors2) {
  ParseTestString(R"""(
<model name='test'>
  <link name='a'/>
  <link name='b'/>
  <drake:joint name="curvilinear_joint" type="curvilinear">
    <drake:parent>a</drake:parent>
    <drake:child>b</drake:child>
    <drake:curves>
      <drake:line_segment>
      </drake:line_segment>
    </drake:curves>
</drake:joint>
</model>
)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*drake:length.*child.*"));
}

TEST_F(SdfParserTest, CurvilinearJointErrors3) {
  ParseTestString(R"""(
<model name='test'>
  <link name='a'/>
  <link name='b'/>
  <drake:joint name="curvilinear_joint_aperiodic" type="curvilinear">
    <drake:parent>a</drake:parent>
    <drake:child>b</drake:child>
    <drake:curves>
      <drake:line_segment>
      </drake:line_segment>
    </drake:curves>
</drake:joint>
</model>
)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*drake:length.*child.*"));
}

TEST_F(SdfParserTest, CurvilinearJointErrors4) {
  ParseTestString(R"""(
<model name='test'>
  <link name='a'/>
  <link name='b'/>
  <drake:joint name="curvilinear_joint" type="curvilinear">
    <drake:parent>a</drake:parent>
    <drake:child>b</drake:child>
    <drake:damping>-1</drake:damping>
    <drake:curves>
      <drake:line_segment>
        <drake:length>1.0</drake:length>
      </drake:line_segment>
    </drake:curves>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*negative.*damping.*"));
}

TEST_F(SdfParserTest, CurvilinearJointErrors5) {
  ParseTestString(R"""(
<model name='test'>
  <link name='a'/>
  <link name='b'/>
  <drake:joint name="curvilinear_joint" type="curvilinear">
    <drake:parent>a</drake:parent>
    <drake:child>b</drake:child>
    <drake:curves>
      <drake:circular_arc>
        <drake:radius>-2.0</drake:radius>
        <drake:angle>1.57</drake:angle>
      </drake:circular_arc>
    </drake:curves>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*negative.*radius.*"));
}

TEST_F(SdfParserTest, CurvilinearJointErrors6) {
  ParseTestString(R"""(
<model name='test'>
  <link name='a'/>
  <link name='b'/>
  <drake:joint name="curvilinear_joint" type="curvilinear">
    <drake:parent>a</drake:parent>
    <drake:child>b</drake:child>
    <drake:curves>
      <drake:circular_arc>
        <drake:radius>2.0</drake:radius>
        <drake:angle>1.57</drake:angle>
      </drake:circular_arc>
      <drake:cirQQQcular_arc/>
    </drake:curves>
  </drake:joint>
</model>
)""");
  EXPECT_THAT(TakeError(), MatchesRegex(".*invalid.*cirQQQcular_arc.*"));
}

TEST_F(SdfParserTest, TestSupportedFrames1) {
  AddSceneGraph();
  // Test `//link/pose[@relative_to]`.
  ParseTestString(R"""(
<model name='good'>
  <frame name='my_frame'/>
  <link name='my_link'>
    <pose relative_to='my_frame'/>
  </link>
</model>
)""");
}

TEST_F(SdfParserTest, TestSupportedFrames2) {
  AddSceneGraph();
  // Test `//link/visual/pose[@relative_to]`.
  ParseTestString(R"""(
<model name='good'>
  <frame name='my_frame'/>
  <link name='my_link'>
    <visual name='my_visual'>
      <pose relative_to='my_frame'/>
    </visual>
  </link>
</model>
)""");
}

TEST_F(SdfParserTest, TestSupportedFrames3) {
  AddSceneGraph();
  // Test `//link/collision/pose[@relative_to]`.
  ParseTestString(R"""(
<model name='good'>
  <frame name='my_frame'/>
  <link name='my_link'>
    <collision name='my_collision'>
      <pose relative_to='my_frame'/>
    </collision>
  </link>
</model>
)""");
}

TEST_F(SdfParserTest, TestSupportedFrames4) {
  AddSceneGraph();
  // Test `//joint/pose[@relative_to]`.
  ParseTestString(R"""(
<model name='good'>
  <link name='a'/>
  <frame name='my_frame'/>
  <joint name='b' type='fixed'>"
    <pose relative_to='my_frame'/>"
    <parent>world</parent>
    <child>a</child>"
  </joint>
</model>)""");
}

TEST_F(SdfParserTest, TestUnsupportedFrames) {
  // Model frames cannot attach to / nor be relative to the world frame.
  ParseTestString(R"""(
<model name='bad'>
  <link name='dont_crash_plz'/>  <!-- Need at least one link -->
  <frame name='model_scope_world_frame' attached_to='world'>
    <pose>0 0 0 0 0 0</pose>
  </frame>
</model>
)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          R"(.*(attached_to|relative_to) name\[world\] specified by frame )"
          R"(with name\[.*\] does not match a nested model, link, joint, or )"
          R"(frame name in model with name\[bad\].*)"));
  // Ignore additional errors for this test.
  EXPECT_EQ(NumErrors(), 5);
  ClearDiagnostics();

  ParseTestString(R"""(
<model name='bad'>
  <link name='dont_crash_plz'/>  <!-- Need at least one link -->
  <frame name='model_scope_world_relative_frame'>
    <pose relative_to='world'>0 0 0 0 0 0</pose>
  </frame>
</model>
)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          R"(.*(attached_to|relative_to) name\[world\] specified by frame )"
          R"(with name\[.*\] does not match a nested model, link, joint, or )"
          R"(frame name in model with name\[bad\].*)"));
  // Ignore additional errors for this test.
  EXPECT_EQ(NumErrors(), 2);
  ClearDiagnostics();

  for (std::string bad_name : {"world", "__model__", "__anything__"}) {
    SCOPED_TRACE(bad_name);
    ParseTestString(fmt::format(R"""(
<model name='bad'>
  <link name='dont_crash_plz'/>  <!-- Need at least one link -->
  <frame name='{}'/>  <!-- Invalid name -->
</model>
)""",
                                bad_name));
    EXPECT_THAT(
        TakeError(),
        MatchesRegex(R"(.*The supplied frame name \[.*\] is reserved..*)"));
    // Ignore additional errors for this test. The number ignored varies.
    ClearDiagnostics();
  }

  ParseTestString(R"""(
<model name='bad'>
  <pose relative_to='invalid_usage'/>
  <link name='dont_crash_plz'/>  <!-- Need at least one frame -->
</model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(R"(.*Attribute //pose\[@relative_to\] of top level model )"
                   R"(must be left empty.*)"));
  // Ignore additional errors for this test.
  EXPECT_EQ(NumErrors(), 2);
  ClearDiagnostics();

  ParseTestString(R"""(
<model name='bad'>
  <frame name='my_frame'/>
  <link name='a'>
    <inertial><pose relative_to='my_frame'/></inertial>
  </link>
</model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(R"(.*XML Attribute\[relative_to\] in element\[pose\] not )"
                   R"(defined in SDF.*)"));
}

// Tests Drake's usage of sdf::EnforcementPolicy.
TEST_F(SdfParserTest, TestSdformatParserPolicies) {
  AddSceneGraph();
  ParseTestString(R"""(
<model name='model_with_bad_attribute' bad_attribute="junk">
  <link name='a'/>
</model>
)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(
                  R"(.*XML Attribute\[bad_attribute\] in element\[model\] not )"
                  R"(defined in SDF.*)"));
  FlushDiagnostics();

  ParseTestString(R"""(
<model name='model_with_too_many_top_level_elements'>
  <link name='a'/>
</model>
<model name='two_models_too_many'>
  <link name='b'/>
</model>
)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Root object can only contain one model.*"));
  FlushDiagnostics();

  // TODO(#15018): This throws a warning, make this an error.
  ParseTestString(R"""(
<model name='model_with_bad_element'>
  <link name='a'/>
  <bad_element/>
</model>
)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(R"(.*XML Element\[bad_element\], child of)"
                           R"( element\[model\], not defined in SDF.*)"));
  FlushDiagnostics();

  ParseTestString(R"""(
<model name='model_with_initial_position'>
  <link name='l1'/>
  <link name='l2'/>
  <joint name='b' type="revolute">
    <child>l1</child>
    <parent>l2</parent>
    <axis>
      <initial_position>0</initial_position>
    </axis>
  </joint>
</model>)""",
                  "1.9");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(R"(.*XML Element\[initial_position\], child of element)"
                   R"(\[axis\], not defined in SDF.*)"));
  FlushDiagnostics();

  ParseTestString(R"""(
<model name='deprecation_test'>
  <link name='l1'/>
</model>
<_drake_deprecation_unit_test_element/>
)""",
                  "1.9");
  EXPECT_THAT(TakeWarning(),
              testing::MatchesRegex(
                  ".*drake_deprecation_unit_test_element.*is deprecated.*"));
  FlushDiagnostics();
}

// Reports if the frame with the given id has a geometry with the given role
// whose name is the same as what ShapeType{}.type_name() would produce.
template <typename ShapeType>
::testing::AssertionResult FrameHasShape(geometry::FrameId frame_id,
                                         geometry::Role role,
                                         const SceneGraph<double>& scene_graph,
                                         const ShapeType& shape) {
  const auto& inspector = scene_graph.model_inspector();
  const std::string name{shape.type_name()};
  try {
    // Note: MBP prepends the model index to the geometry name; in this case
    // that model instance  name is "test_robot".
    const geometry::GeometryId geometry_id =
        inspector.GetGeometryIdByName(frame_id, role, "test_robot::" + name);
    const std::string_view shape_type =
        inspector.GetShape(geometry_id).type_name();
    if (shape_type != name) {
      return ::testing::AssertionFailure() << fmt::format(
                 "Geometry with role {} has wrong shape type.\n"
                 "  Expected: {}\n"
                 "  Found: {}",
                 role, name, shape_type);
    }
  } catch (const std::exception& e) {
    return ::testing::AssertionFailure() << fmt::format(
               "Frame {} does not have a geometry with role {} and name {}.\n"
               "  Exception message: {}",
               frame_id, role, name, e.what());
  }
  return ::testing::AssertionSuccess();
}

class GeometrySdfParserTest : public SdfParserTest {
 public:
  GeometrySdfParserTest() { AddSceneGraph(); }

 protected:
  // Confirms that all supported geometries in an SDF file are registered. The
  // *details* of the geometries are ignored -- we assume that that
  // functionality is tested in detail_sdf_geometry_test.cc. This merely makes
  // sure that *that* functionality is exercised appropriately.
  void TestForParsedGeometry(const char* sdf_name, geometry::Role role) {
    const std::string full_name = FindResourceOrThrow(sdf_name);
    AddModelsFromSdfFile(full_name);
    plant_.Finalize();

    const auto frame_id =
        plant_.GetBodyFrameIdOrThrow(plant_.GetBodyByName("link1").index());

    const std::string mesh_uri = "drake/multibody/parsing/test/tri_cube.obj";

    // Note: the parameters for the various example shapes do not matter to this
    // test.
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Box{0.1, 0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Capsule{0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Convex{mesh_uri, 1.0}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Cylinder{0.1, 0.1}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Ellipsoid{0.1, 0.1, 0.1}));
    EXPECT_TRUE(
        FrameHasShape(frame_id, role, scene_graph_, geometry::HalfSpace{}));
    EXPECT_TRUE(FrameHasShape(frame_id, role, scene_graph_,
                              geometry::Mesh{mesh_uri, 1.0}));
    EXPECT_TRUE(
        FrameHasShape(frame_id, role, scene_graph_, geometry::Sphere{0.1}));
  }
};

TEST_F(GeometrySdfParserTest, CollisionGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "all_geometries_as_collision.sdf",
      geometry::Role::kProximity);
}

TEST_F(GeometrySdfParserTest, VisualGeometryParsing) {
  TestForParsedGeometry(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "all_geometries_as_visual.sdf",
      geometry::Role::kPerception);
}

TEST_F(SdfParserTest, BallConstraint) {
  AddSceneGraph();

  // TODO(joemasterjohn): Currently ball constraints are only supported in SAP.
  // Add coverage for other solvers and continuous mode when available.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);

  // Test successful parsing.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:ball_constraint>
          <drake:ball_constraint_body_A>A</drake:ball_constraint_body_A>
          <drake:ball_constraint_body_B>B</drake:ball_constraint_body_B>
          <drake:ball_constraint_p_AP>1 2 3</drake:ball_constraint_p_AP>
          <drake:ball_constraint_p_BQ>4 5 6</drake:ball_constraint_p_BQ>
        </drake:ball_constraint>
      </model>
    </world>)""");

  EXPECT_EQ(plant_.num_constraints(), 1);
  EXPECT_EQ(plant_.num_ball_constraints(), 1);

  const std::map<MultibodyConstraintId, BallConstraintSpec>& ball_constraints =
      plant_.get_ball_constraint_specs();

  ASSERT_EQ(ssize(ball_constraints), 1);

  const MultibodyConstraintId id = ball_constraints.begin()->first;
  const BallConstraintSpec& ball_constraint = ball_constraints.begin()->second;

  EXPECT_TRUE(ball_constraint.IsValid());
  EXPECT_EQ(ball_constraint.body_A, plant_.GetBodyByName("A").index());
  EXPECT_EQ(ball_constraint.body_B, plant_.GetBodyByName("B").index());
  EXPECT_EQ(ball_constraint.p_AP, Vector3d(1, 2, 3));
  EXPECT_EQ(ball_constraint.p_BQ, Vector3d(4, 5, 6));
  EXPECT_EQ(ball_constraint.id, id);
}

// Test missing body
TEST_F(SdfParserTest, BallConstraintMissingBody) {
  AddSceneGraph();

  // TODO(joemasterjohn): Currently ball constraints are only supported in SAP.
  // Add coverage for other solvers and continuous mode when available.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);

  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:ball_constraint>
          <drake:ball_constraint_body_A>A</drake:ball_constraint_body_A>
          <!-- Omit body B -->
          <drake:ball_constraint_p_AP>1 2 3</drake:ball_constraint_p_AP>
          <drake:ball_constraint_p_BQ>4 5 6</drake:ball_constraint_p_BQ>
        </drake:ball_constraint>
      </model>
    </world>)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*<drake:ball_constraint>: Unable to find the "
                           "<drake:ball_constraint_body_B> child tag."));
}

// Test missing point
TEST_F(SdfParserTest, BallConstraintMissingPoint) {
  AddSceneGraph();

  // TODO(joemasterjohn): Currently ball constraints are only supported in SAP.
  // Add coverage for other solvers and continuous mode when available.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);

  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:ball_constraint>
          <drake:ball_constraint_body_A>A</drake:ball_constraint_body_A>
          <drake:ball_constraint_body_B>B</drake:ball_constraint_body_B>
          <!-- Omit p_AP -->
          <drake:ball_constraint_p_BQ>4 5 6</drake:ball_constraint_p_BQ>
        </drake:ball_constraint>
      </model>
    </world>)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*<drake:ball_constraint>: Unable to find the "
                           "<drake:ball_constraint_p_AP> child tag."));
}

// Test non-existent body
TEST_F(SdfParserTest, BallConstraintNonExistentBody) {
  AddSceneGraph();

  // TODO(joemasterjohn): Currently ball constraints are only supported in SAP.
  // Add coverage for other solvers and continuous mode when available.
  plant_.set_discrete_contact_approximation(DiscreteContactApproximation::kSap);

  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:ball_constraint>
          <drake:ball_constraint_body_A>A</drake:ball_constraint_body_A>
          <!-- Body doesn't exist in the model -->
          <drake:ball_constraint_body_B>C</drake:ball_constraint_body_B>
          <drake:ball_constraint_p_AP>1 2 3</drake:ball_constraint_p_AP>
          <drake:ball_constraint_p_BQ>4 5 6</drake:ball_constraint_p_BQ>
        </drake:ball_constraint>
      </model>
    </world>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          ".*<drake:ball_constraint>: Body 'C' specified for "
          "<drake:ball_constraint_body_B> does not exist in the model."));
}

TEST_F(SdfParserTest, TendonConstraint) {
  AddSceneGraph();

  // Test successful parsing.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <link name='C'/>
        <joint name='revolute_AB' type='revolute'>
          <child>A</child>
          <parent>B</parent>
          <axis>
            <xyz>0 0 1</xyz>
          </axis>
        </joint>
        <joint name='prismatic_BC' type='prismatic'>
          <child>B</child>
          <parent>C</parent>
          <axis>
            <xyz>0 0 1</xyz>
          </axis>
        </joint>
        <drake:tendon_constraint>
          <drake:tendon_constraint_joint name='revolute_AB' a='10'/>
          <drake:tendon_constraint_joint name='prismatic_BC' a='20'/>
          <drake:tendon_constraint_offset>0.5</drake:tendon_constraint_offset>
          <drake:tendon_constraint_lower_limit>-1.0</drake:tendon_constraint_lower_limit>
          <drake:tendon_constraint_upper_limit>1.0</drake:tendon_constraint_upper_limit>
          <drake:tendon_constraint_stiffness>0.1</drake:tendon_constraint_stiffness>
          <drake:tendon_constraint_damping>0.01</drake:tendon_constraint_damping>
        </drake:tendon_constraint>
      </model>
    </world>)""");

  EXPECT_EQ(plant_.num_constraints(), 1);
  EXPECT_EQ(plant_.num_tendon_constraints(), 1);

  const std::map<MultibodyConstraintId, TendonConstraintSpec>&
      tendon_constraints = plant_.get_tendon_constraint_specs();

  ASSERT_EQ(ssize(tendon_constraints), 1);

  const MultibodyConstraintId id = tendon_constraints.begin()->first;
  const TendonConstraintSpec& tendon_constraint =
      tendon_constraints.begin()->second;

  ASSERT_EQ(ssize(tendon_constraint.joints), 2);
  ASSERT_EQ(ssize(tendon_constraint.a), 2);

  EXPECT_EQ(tendon_constraint.joints[0],
            plant_.GetJointByName("revolute_AB").index());
  EXPECT_EQ(tendon_constraint.a[0], 10.0);
  EXPECT_EQ(tendon_constraint.joints[1],
            plant_.GetJointByName("prismatic_BC").index());
  EXPECT_EQ(tendon_constraint.a[1], 20.0);

  EXPECT_EQ(tendon_constraint.offset, 0.5);
  EXPECT_EQ(tendon_constraint.lower_limit, -1.0);
  EXPECT_EQ(tendon_constraint.upper_limit, 1.0);
  EXPECT_EQ(tendon_constraint.stiffness, 0.1);
  EXPECT_EQ(tendon_constraint.damping, 0.01);
  EXPECT_EQ(tendon_constraint.id, id);
}

TEST_F(SdfParserTest, TendonConstraintNonExistentJoint) {
  AddSceneGraph();

  // Test successful parsing.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <joint name='revolute_AB' type='revolute'>
          <child>A</child>
          <parent>B</parent>
          <axis>
            <xyz>0 0 1</xyz>
          </axis>
        </joint>
        <drake:tendon_constraint>
          <drake:tendon_constraint_joint name='revolute_AB' a='10'/>
          <!-- Joint does not exist in the model -->
          <drake:tendon_constraint_joint name='does_not_exist' a='20'/>
          <drake:tendon_constraint_offset>0.5</drake:tendon_constraint_offset>
          <drake:tendon_constraint_lower_limit>-1.0</drake:tendon_constraint_lower_limit>
          <drake:tendon_constraint_upper_limit>1.0</drake:tendon_constraint_upper_limit>
          <drake:tendon_constraint_stiffness>0.1</drake:tendon_constraint_stiffness>
          <drake:tendon_constraint_damping>0.01</drake:tendon_constraint_damping>
        </drake:tendon_constraint>
      </model>
    </world>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          ".*<drake:tendon_constraint>: Joint 'does_not_exist' specified for "
          "<drake:tendon_constraint_joint> does not exist in the model."));
}

TEST_F(SdfParserTest, TendonConstraintMissingJointName) {
  AddSceneGraph();

  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <joint name='revolute_AB' type='revolute'>
          <child>A</child>
          <parent>B</parent>
          <axis>
            <xyz>0 0 1</xyz>
          </axis>
        </joint>
        <drake:tendon_constraint>
          <!-- no "name" attribute -->
          <drake:tendon_constraint_joint a='10'/>
          <drake:tendon_constraint_joint name='revolute_AB' a='20'/>
          <drake:tendon_constraint_offset>0.5</drake:tendon_constraint_offset>
          <drake:tendon_constraint_lower_limit>-1.0</drake:tendon_constraint_lower_limit>
          <drake:tendon_constraint_upper_limit>1.0</drake:tendon_constraint_upper_limit>
          <drake:tendon_constraint_stiffness>0.1</drake:tendon_constraint_stiffness>
          <drake:tendon_constraint_damping>0.01</drake:tendon_constraint_damping>
        </drake:tendon_constraint>
      </model>
    </world>)""");

  // Two errors are produced because 1) the name attribute fails to parse and
  // defaults to empty-string, and 2) the empty-string joint does not exist in
  // the model.
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The tag <drake:tendon_constraint_joint> is "
                           "missing the required attribute \"name\""));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(
          ".*<drake:tendon_constraint>: Joint '' specified for "
          "<drake:tendon_constraint_joint> does not exist in the model."));
}

TEST_F(SdfParserTest, TendonConstraintMissingJointCoeff) {
  AddSceneGraph();

  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <link name='C'/>
        <joint name='revolute_AB' type='revolute'>
          <child>A</child>
          <parent>B</parent>
          <axis>
            <xyz>0 0 1</xyz>
          </axis>
        </joint>
        <joint name='prismatic_BC' type='prismatic'>
          <child>B</child>
          <parent>C</parent>
          <axis>
            <xyz>0 0 1</xyz>
          </axis>
        </joint>
        <drake:tendon_constraint>
          <!-- no "a" attribute -->
          <drake:tendon_constraint_joint name='revolute_AB'/>
          <drake:tendon_constraint_joint name='prismatic_BC' a='20'/>
          <drake:tendon_constraint_offset>0.5</drake:tendon_constraint_offset>
          <drake:tendon_constraint_lower_limit>-1.0</drake:tendon_constraint_lower_limit>
          <drake:tendon_constraint_upper_limit>1.0</drake:tendon_constraint_upper_limit>
          <drake:tendon_constraint_stiffness>0.1</drake:tendon_constraint_stiffness>
          <drake:tendon_constraint_damping>0.01</drake:tendon_constraint_damping>
        </drake:tendon_constraint>
      </model>
    </world>)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The tag <drake:tendon_constraint_joint> is "
                           "missing the required attribute \"a\""));
}

TEST_F(SdfParserTest, LinearSpringDamperParsingGood) {
  AddSceneGraph();
  // Test successful parsing.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A>A</drake:linear_spring_damper_body_A>
          <drake:linear_spring_damper_p_AP>1 2 3</drake:linear_spring_damper_p_AP>
          <drake:linear_spring_damper_body_B>B</drake:linear_spring_damper_body_B>
          <drake:linear_spring_damper_p_BQ>4 5 6</drake:linear_spring_damper_p_BQ>
          <drake:linear_spring_damper_free_length>7.0</drake:linear_spring_damper_free_length>
          <drake:linear_spring_damper_stiffness>8.0</drake:linear_spring_damper_stiffness>
          <drake:linear_spring_damper_damping>9.0</drake:linear_spring_damper_damping>
        </drake:linear_spring_damper>
      </model>
    </world>)""");

  // MBP will always create a UniformGravityField, so the only other
  // ForceElements should be the LinearSpringDamper elements parsed.
  EXPECT_EQ(plant_.num_force_elements(), 2);

  const LinearSpringDamper<double>& linear_spring_damper =
      plant_.GetForceElement<LinearSpringDamper>(ForceElementIndex(1));

  EXPECT_STREQ(linear_spring_damper.bodyA().name().c_str(), "A");
  EXPECT_STREQ(linear_spring_damper.bodyB().name().c_str(), "B");
  EXPECT_EQ(linear_spring_damper.bodyA().model_instance(),
            linear_spring_damper.model_instance());
  EXPECT_EQ(linear_spring_damper.p_AP(), Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(linear_spring_damper.p_BQ(), Eigen::Vector3d(4, 5, 6));
  EXPECT_EQ(linear_spring_damper.free_length(), 7.0);
  EXPECT_EQ(linear_spring_damper.stiffness(), 8.0);
  EXPECT_EQ(linear_spring_damper.damping(), 9.0);
}

TEST_F(SdfParserTest, LinearSpringDamperParsingUnsupportedChildTag) {
  AddSceneGraph();
  // Test with an extra tag which is not supported.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <INVALID_TAG>0.0</INVALID_TAG>
        </drake:linear_spring_damper>
      </model>
    </world>)""");

  // We've omitted the required tags. We'll get one error for the first tag it
  // attempted to read (it skips the others). But the invalid tag will still
  // generate a warning.
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(".*Unable to find the .* child tag."));

  EXPECT_THAT(TakeWarning(), ::testing::MatchesRegex(
                                 ".*Ignoring unsupported SDFormat element in "
                                 "drake:linear_spring_damper: INVALID_TAG"));
}

TEST_F(SdfParserTest, LinearSpringDamperParsingNoBodyA) {
  AddSceneGraph();
  // Test missing body tag.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_p_AP>1 2 3</drake:linear_spring_damper_p_AP>
          <drake:linear_spring_damper_body_B>C</drake:linear_spring_damper_body_B>
          <drake:linear_spring_damper_p_BQ>4 5 6</drake:linear_spring_damper_p_BQ>
          <drake:linear_spring_damper_free_length>7.0</drake:linear_spring_damper_free_length>
          <drake:linear_spring_damper_stiffness>8.0</drake:linear_spring_damper_stiffness>
          <drake:linear_spring_damper_damping>9.0</drake:linear_spring_damper_damping>
        </drake:linear_spring_damper>
      </model>
    </world>)""");

  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*<drake:linear_spring_damper>: Unable to find the "
                  "<drake:linear_spring_damper_body_A> child tag."));
}

TEST_F(SdfParserTest, LinearSpringDamperParsingNoBodyB) {
  AddSceneGraph();
  // Test missing body tag.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A>A</drake:linear_spring_damper_body_A>
          <drake:linear_spring_damper_p_AP>1 2 3</drake:linear_spring_damper_p_AP>
          <drake:linear_spring_damper_p_BQ>4 5 6</drake:linear_spring_damper_p_BQ>
          <drake:linear_spring_damper_free_length>7.0</drake:linear_spring_damper_free_length>
          <drake:linear_spring_damper_stiffness>8.0</drake:linear_spring_damper_stiffness>
          <drake:linear_spring_damper_damping>9.0</drake:linear_spring_damper_damping>
        </drake:linear_spring_damper>
      </model>
    </world>)""");

  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*<drake:linear_spring_damper>: Unable to find the "
                  "<drake:linear_spring_damper_body_B> child tag."));
}

TEST_F(SdfParserTest, LinearSpringDamperParsingNonExistentBody) {
  AddSceneGraph();
  // Test non-existent body tag.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A>A</drake:linear_spring_damper_body_A>
          <drake:linear_spring_damper_p_AP>1 2 3</drake:linear_spring_damper_p_AP>
          <drake:linear_spring_damper_body_B>C</drake:linear_spring_damper_body_B>
          <drake:linear_spring_damper_p_BQ>4 5 6</drake:linear_spring_damper_p_BQ>
          <drake:linear_spring_damper_free_length>7.0</drake:linear_spring_damper_free_length>
          <drake:linear_spring_damper_stiffness>8.0</drake:linear_spring_damper_stiffness>
          <drake:linear_spring_damper_damping>9.0</drake:linear_spring_damper_damping>
        </drake:linear_spring_damper>
      </model>
    </world>)""");

  EXPECT_THAT(
      TakeError(),
      ::testing::MatchesRegex(
          ".*<drake:linear_spring_damper>: Body 'C' specified for "
          "<drake:linear_spring_damper_body_B> does not exist in the model."));
}

TEST_F(SdfParserTest, LinearSpringDamperParsingInvalidFreeLength) {
  AddSceneGraph();
  // Test zero free length.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A>A</drake:linear_spring_damper_body_A>
          <drake:linear_spring_damper_p_AP>1 2 3</drake:linear_spring_damper_p_AP>
          <drake:linear_spring_damper_body_B>B</drake:linear_spring_damper_body_B>
          <drake:linear_spring_damper_p_BQ>4 5 6</drake:linear_spring_damper_p_BQ>
          <drake:linear_spring_damper_free_length>0</drake:linear_spring_damper_free_length>
          <drake:linear_spring_damper_stiffness>8.0</drake:linear_spring_damper_stiffness>
          <drake:linear_spring_damper_damping>9.0</drake:linear_spring_damper_damping>
        </drake:linear_spring_damper>
      </model>
    </world>)""");

  EXPECT_THAT(
      TakeError(),
      ::testing::MatchesRegex(
          ".*<drake:linear_spring_damper>: The "
          "<drake:linear_spring_damper_free_length> child tag must be strictly "
          "positive."));
}

TEST_F(SdfParserTest, LinearSpringDamperParsingInvalidStiffness) {
  AddSceneGraph();
  // Test negative stiffness.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A>A</drake:linear_spring_damper_body_A>
          <drake:linear_spring_damper_p_AP>1 2 3</drake:linear_spring_damper_p_AP>
          <drake:linear_spring_damper_body_B>B</drake:linear_spring_damper_body_B>
          <drake:linear_spring_damper_p_BQ>4 5 6</drake:linear_spring_damper_p_BQ>
          <drake:linear_spring_damper_free_length>7.0</drake:linear_spring_damper_free_length>
          <drake:linear_spring_damper_stiffness>-8.0</drake:linear_spring_damper_stiffness>
          <drake:linear_spring_damper_damping>9.0</drake:linear_spring_damper_damping>
        </drake:linear_spring_damper>
      </model>
    </world>)""");

  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*<drake:linear_spring_damper>: The "
                  "<drake:linear_spring_damper_stiffness> child tag must be "
                  "non-negative."));
}

TEST_F(SdfParserTest, LinearSpringDamperParsingInvalidDamping) {
  AddSceneGraph();
  // Test negative damping.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A>A</drake:linear_spring_damper_body_A>
          <drake:linear_spring_damper_p_AP>1 2 3</drake:linear_spring_damper_p_AP>
          <drake:linear_spring_damper_body_B>B</drake:linear_spring_damper_body_B>
          <drake:linear_spring_damper_p_BQ>4 5 6</drake:linear_spring_damper_p_BQ>
          <drake:linear_spring_damper_free_length>7.0</drake:linear_spring_damper_free_length>
          <drake:linear_spring_damper_stiffness>8.0</drake:linear_spring_damper_stiffness>
          <drake:linear_spring_damper_damping>-9.0</drake:linear_spring_damper_damping>
        </drake:linear_spring_damper>
      </model>
    </world>)""");

  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*<drake:linear_spring_damper>: The "
                  "<drake:linear_spring_damper_damping> child tag must be "
                  "non-negative."));
}

TEST_F(SdfParserTest, LinearSpringDamperParsingMissingValue) {
  AddSceneGraph();
  // Test missing free length.
  ParseTestString(R"""(
    <world name='World'>
      <model name='Model'>
        <link name='A'/>
        <link name='B'/>
        <drake:linear_spring_damper>
          <drake:linear_spring_damper_body_A>A</drake:linear_spring_damper_body_A>
          <drake:linear_spring_damper_p_AP>1 2 3</drake:linear_spring_damper_p_AP>
          <drake:linear_spring_damper_body_B>B</drake:linear_spring_damper_body_B>
          <drake:linear_spring_damper_p_BQ>4 5 6</drake:linear_spring_damper_p_BQ>
          <drake:linear_spring_damper_stiffness>8.0</drake:linear_spring_damper_stiffness>
          <drake:linear_spring_damper_damping>9.0</drake:linear_spring_damper_damping>
        </drake:linear_spring_damper>
      </model>
    </world>)""");

  EXPECT_THAT(
      TakeError(),
      ::testing::MatchesRegex(
          ".*<drake:linear_spring_damper>: Unable "
          "to find the <drake:linear_spring_damper_free_length> child tag."));
}

TEST_F(SdfParserTest, BushingParsingGood) {
  AddSceneGraph();
  // Test successful parsing.  Add two copies of the model to make sure the
  // bushings are associated with the proper model instance.
  ParseTestString(R"""(
    <world name='BushingWorld'>
      <model name='BushingModel'>
        <link name='A'/>
        <link name='C'/>
        <frame name='frameA' attached_to='A'/>
        <frame name='frameC' attached_to='C'/>
        <drake:linear_bushing_rpy>
          <drake:bushing_frameA>frameA</drake:bushing_frameA>
          <drake:bushing_frameC>frameC</drake:bushing_frameC>
          <drake:bushing_torque_stiffness>1 2 3</drake:bushing_torque_stiffness>
          <drake:bushing_torque_damping>4 5 6</drake:bushing_torque_damping>
          <drake:bushing_force_stiffness>7 8 9</drake:bushing_force_stiffness>
          <drake:bushing_force_damping>10 11 12</drake:bushing_force_damping>
        </drake:linear_bushing_rpy>
      </model>
      <model name='BushingModel2'>
        <link name='A'/>
        <link name='C'/>
        <frame name='frameA' attached_to='A'/>
        <frame name='frameC' attached_to='C'/>
        <drake:linear_bushing_rpy>
          <drake:bushing_frameA>frameA</drake:bushing_frameA>
          <drake:bushing_frameC>frameC</drake:bushing_frameC>
          <drake:bushing_torque_stiffness>11 12 13</drake:bushing_torque_stiffness>
          <drake:bushing_torque_damping>14 15 16</drake:bushing_torque_damping>
          <drake:bushing_force_stiffness>17 18 19</drake:bushing_force_stiffness>
          <drake:bushing_force_damping>20 21 22</drake:bushing_force_damping>
        </drake:linear_bushing_rpy>
      </model>
    </world>)""");

  // MBP will always create a UniformGravityField, so the only other
  // ForceElements should be the LinearBushingRollPitchYaw elements parsed.
  EXPECT_EQ(plant_.num_force_elements(), 3);

  const LinearBushingRollPitchYaw<double>& bushing =
      plant_.GetForceElement<LinearBushingRollPitchYaw>(ForceElementIndex(1));

  EXPECT_STREQ(bushing.frameA().name().c_str(), "frameA");
  EXPECT_STREQ(bushing.frameC().name().c_str(), "frameC");
  EXPECT_EQ(bushing.frameA().model_instance(), bushing.model_instance());
  EXPECT_EQ(bushing.torque_stiffness_constants(), Eigen::Vector3d(1, 2, 3));
  EXPECT_EQ(bushing.torque_damping_constants(), Eigen::Vector3d(4, 5, 6));
  EXPECT_EQ(bushing.force_stiffness_constants(), Eigen::Vector3d(7, 8, 9));
  EXPECT_EQ(bushing.force_damping_constants(), Eigen::Vector3d(10, 11, 12));

  const LinearBushingRollPitchYaw<double>& bushing2 =
      plant_.GetForceElement<LinearBushingRollPitchYaw>(ForceElementIndex(2));

  EXPECT_STREQ(bushing2.frameA().name().c_str(), "frameA");
  EXPECT_STREQ(bushing2.frameC().name().c_str(), "frameC");
  EXPECT_EQ(bushing2.frameA().model_instance(), bushing2.model_instance());
  EXPECT_NE(bushing.model_instance(), bushing2.model_instance());
  EXPECT_EQ(bushing2.torque_stiffness_constants(), Eigen::Vector3d(11, 12, 13));
  EXPECT_EQ(bushing2.torque_damping_constants(), Eigen::Vector3d(14, 15, 16));
  EXPECT_EQ(bushing2.force_stiffness_constants(), Eigen::Vector3d(17, 18, 19));
  EXPECT_EQ(bushing2.force_damping_constants(), Eigen::Vector3d(20, 21, 22));
}

TEST_F(SdfParserTest, BushingParsingBad1) {
  AddSceneGraph();
  // Test missing frame tag
  ParseTestString(R"""(
    <model name='BushingModel'>
      <link name='A'/>
      <link name='C'/>
      <frame name='frameA' attached_to='A'/>
      <frame name='frameC' attached_to='C'/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA>frameA</drake:bushing_frameA>
        <!-- missing the drake:bushing_frameC tag -->
        <drake:bushing_torque_stiffness>1 2 3</drake:bushing_torque_stiffness>
        <drake:bushing_torque_damping>4 5 6</drake:bushing_torque_damping>
        <drake:bushing_force_stiffness>7 8 9</drake:bushing_force_stiffness>
        <drake:bushing_force_damping>10 11 12</drake:bushing_force_damping>
      </drake:linear_bushing_rpy>
    </model>)""");
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*<drake:linear_bushing_rpy>: Unable to find the "
                           "<drake:bushing_frameC> child tag."));
}

TEST_F(SdfParserTest, BushingParsingBad2) {
  AddSceneGraph();
  // Test non-existent frame
  ParseTestString(R"""(
    <model name='BushingModel'>
      <link name='A'/>
      <link name='C'/>
      <frame name='frameA' attached_to='A'/>
      <frame name='frameC' attached_to='C'/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA>frameA</drake:bushing_frameA>
        <drake:bushing_frameC>frameZ</drake:bushing_frameC>
        <!-- frameZ does not exist in the model -->
        <drake:bushing_torque_stiffness>1 2 3</drake:bushing_torque_stiffness>
        <drake:bushing_torque_damping>4 5 6</drake:bushing_torque_damping>
        <drake:bushing_force_stiffness>7 8 9</drake:bushing_force_stiffness>
        <drake:bushing_force_damping>10 11 12</drake:bushing_force_damping>
      </drake:linear_bushing_rpy>
    </model>)""");
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*<drake:linear_bushing_rpy>: Frame 'frameZ' specified for "
                   "<drake:bushing_frameC> does not exist in "
                   "the model."));
}

TEST_F(SdfParserTest, BushingParsingBad3) {
  AddSceneGraph();
  ThrowErrors();
  // Test missing constants tag
  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseTestString(R"""(
    <model name='BushingModel'>
      <link name='A'/>
      <link name='C'/>
      <frame name='frameA' attached_to='A'/>
      <frame name='frameC' attached_to='C'/>
      <drake:linear_bushing_rpy>
        <drake:bushing_frameA>frameA</drake:bushing_frameA>
        <drake:bushing_frameC>frameC</drake:bushing_frameC>
        <drake:bushing_torque_stiffness>1 2 3</drake:bushing_torque_stiffness>
        <!-- missing the drake:bushing_torque_damping tag -->
        <drake:bushing_force_stiffness>7 8 9</drake:bushing_force_stiffness>
        <drake:bushing_force_damping>10 11 12</drake:bushing_force_damping>
      </drake:linear_bushing_rpy>
    </model>)"""),
      ".*<drake:linear_bushing_rpy>: Unable to find the "
      "<drake:bushing_torque_damping> child tag.");
}

TEST_F(SdfParserTest, ReflectedInertiaParametersParsing) {
  AddSceneGraph();
  // Common SDF string with format options for the two custom tags.
  constexpr const char* test_string = R"""(
    <model name='ReflectedInertiaModel_{2}'>
      <link name='A'/>
      <link name='B'/>
      <joint name='revolute_AB' type='revolute'>
        <child>A</child>
        <parent>B</parent>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <effort>-1</effort>
          </limit>
        </axis>
        {0}
        {1}
      </joint>
    </model>)""";

  // Test successful parsing of both parameters.
  {
    ParseTestString(fmt::format(
        test_string, "<drake:rotor_inertia>1.5</drake:rotor_inertia>",
        "<drake:gear_ratio>300.0</drake:gear_ratio>", "specify_both"));

    const ModelInstanceIndex model =
        plant_.GetModelInstanceByName("ReflectedInertiaModel_specify_both");
    const JointActuator<double>& actuator =
        plant_.GetJointActuatorByName("revolute_AB", model);

    EXPECT_EQ(actuator.default_rotor_inertia(), 1.5);
    EXPECT_EQ(actuator.default_gear_ratio(), 300.0);
  }

  // Test successful parsing of rotor_inertia and default value for
  // gear_ratio.
  {
    ParseTestString(fmt::format(
        test_string, "<drake:rotor_inertia>1.5</drake:rotor_inertia>", "",
        "default_gear"));

    const ModelInstanceIndex model =
        plant_.GetModelInstanceByName("ReflectedInertiaModel_default_gear");
    const JointActuator<double>& actuator =
        plant_.GetJointActuatorByName("revolute_AB", model);

    EXPECT_EQ(actuator.default_rotor_inertia(), 1.5);
    EXPECT_EQ(actuator.default_gear_ratio(), 1.0);
  }

  // Test successful parsing of gear_ratio and default value for
  // rotor_inertia.
  {
    ParseTestString(fmt::format(test_string, "",
                                "<drake:gear_ratio>300.0</drake:gear_ratio>",
                                "default_rotor"));

    const ModelInstanceIndex model =
        plant_.GetModelInstanceByName("ReflectedInertiaModel_default_rotor");
    const JointActuator<double>& actuator =
        plant_.GetJointActuatorByName("revolute_AB", model);

    EXPECT_EQ(actuator.default_rotor_inertia(), 0.0);
    EXPECT_EQ(actuator.default_gear_ratio(), 300.0);
  }
}

TEST_F(SdfParserTest, ControllerGainsParsing) {
  AddSceneGraph();
  // Common SDF string with format options for the custom tag.
  constexpr const char* test_string = R"""(
    <model name='ControllerGainsModel_{1}'>
      <link name='A'/>
      <link name='B'/>
      <joint name='revolute_AB' type='revolute'>
        <child>A</child>
        <parent>B</parent>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <effort>-1</effort>
          </limit>
        </axis>
        {0}
      </joint>
    </model>)""";

  // Test successful parsing of both attributes.
  {
    ParseTestString(fmt::format(
        test_string, "<drake:controller_gains p='10000.0' d='100.0' />",
        "specify_both"));

    const ModelInstanceIndex model =
        plant_.GetModelInstanceByName("ControllerGainsModel_specify_both");
    const JointActuator<double>& actuator =
        plant_.GetJointActuatorByName("revolute_AB", model);

    EXPECT_EQ(actuator.get_controller_gains().p, 10000.0);
    EXPECT_EQ(actuator.get_controller_gains().d, 100.0);
  }
  // Test missing 'p' attribute.
  {
    const std::string expected_message = ".*Unable to find the 'p' attribute.*";
    ParseTestString(fmt::format(
        test_string, "<drake:controller_gains d='100.0' />", "missing_p"));
    EXPECT_THAT(TakeError(), MatchesRegex(expected_message));
  }
  // Test missing 'd' attribute.
  {
    const std::string expected_message = ".*Unable to find the 'd' attribute.*";
    ParseTestString(fmt::format(
        test_string, "<drake:controller_gains p='10000.0'/>", "missing_d"));
    EXPECT_THAT(TakeError(), MatchesRegex(expected_message));
  }
}

// Verifies that the SDFormat loader can add directly nested models to a
// multibody plant. For reference, the files test/integration/model_dom.cc and
// test/integration/nested_model.cc in the libsdformat source code (tag
// sdformat11_11.0.0) contain tests that show more detailed behavior of
// SDFormat's nested model support.
TEST_F(SdfParserTest, LoadDirectlyNestedModelsInWorld) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "world_with_directly_nested_models.sdf");

  // We start with the world and default model instances.
  ASSERT_EQ(plant_.num_model_instances(), 2);
  ASSERT_EQ(plant_.num_bodies(), 1);
  ASSERT_EQ(plant_.num_joints(), 0);

  AddModelsFromSdfFile(full_name);
  plant_.Finalize();

  // We should have loaded 3 more models.
  EXPECT_EQ(plant_.num_model_instances(), 5);
  // The models should have added 4 more bodies.
  EXPECT_EQ(plant_.num_bodies(), 5);
  // The models should have added 4 more joints.
  EXPECT_EQ(plant_.num_joints(), 4);

  // There should be a model instance with the name "parent_model".
  ASSERT_TRUE(plant_.HasModelInstanceNamed("parent_model"));

  // There should be a model instance with the name "parent_model::robot1".
  // This is the model "robot1" nested inside "parent_model"
  ASSERT_TRUE(plant_.HasModelInstanceNamed("parent_model::robot1"));
  ModelInstanceIndex robot1_model =
      plant_.GetModelInstanceByName("parent_model::robot1");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant_.HasBodyNamed("base_link", robot1_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant_.HasBodyNamed("moving_link", robot1_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant_.HasJointNamed("slider", robot1_model));

  // There should be a model instance with the name "parent_model::robot2".
  // This is the model "robot2" nested inside "parent_model"
  ASSERT_TRUE(plant_.HasModelInstanceNamed("parent_model::robot2"));
  ModelInstanceIndex robot2_model =
      plant_.GetModelInstanceByName("parent_model::robot2");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant_.HasBodyNamed("base_link", robot2_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant_.HasBodyNamed("moving_link", robot2_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant_.HasJointNamed("slider", robot2_model));

  // There should be a joint named "weld_robots". By convention, the joint
  // will have the same model instance as the child frame.
  EXPECT_TRUE(plant_.HasJointNamed("weld_robots", robot2_model));
}

// Same test as LoadDirectlyNestedModelsInWorld, but where a model file contains
// directly nested models.
TEST_F(SdfParserTest, LoadDirectlyNestedModelsInModel) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "model_with_directly_nested_models.sdf");

  // We start with the world and default model instances.
  ASSERT_EQ(plant_.num_model_instances(), 2);
  ASSERT_EQ(plant_.num_bodies(), 1);
  ASSERT_EQ(plant_.num_joints(), 0);

  AddModelsFromSdfFile(full_name);
  plant_.Finalize();

  // We should have loaded 4 more models.
  EXPECT_EQ(plant_.num_model_instances(), 6);
  // The models should have added 4 more bodies.
  EXPECT_EQ(plant_.num_bodies(), 5);
  // The models should have added 4 more joints.
  EXPECT_EQ(plant_.num_joints(), 4);

  // There should be a model instance with the name "grand_parent_model" (top
  // level model).
  ASSERT_TRUE(plant_.HasModelInstanceNamed("grand_parent_model"));

  // There should be a model instance with the name
  // "grand_parent_model::parent_model". This is the model "parent_model"
  // nested inside "grand_parent_model"
  ASSERT_TRUE(plant_.HasModelInstanceNamed("grand_parent_model::parent_model"));

  // There should be a model instance with the name
  // "grand_parent_model::parent_model::robot1". This is the model "robot1"
  // nested inside "parent_model" which itself is nested inside
  // grand_parent_model
  ASSERT_TRUE(
      plant_.HasModelInstanceNamed("grand_parent_model::parent_model::robot1"));
  ModelInstanceIndex robot1_model =
      plant_.GetModelInstanceByName("grand_parent_model::parent_model::robot1");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant_.HasBodyNamed("base_link", robot1_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant_.HasBodyNamed("moving_link", robot1_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant_.HasJointNamed("slider", robot1_model));

  // There should be a model instance with the name
  // "grand_parent_model::parent_model::robot2". This is the model "robot2"
  // nested inside "parent_model" which itself is nested inside
  // grand_parent_model
  ASSERT_TRUE(
      plant_.HasModelInstanceNamed("grand_parent_model::parent_model::robot2"));
  ModelInstanceIndex robot2_model =
      plant_.GetModelInstanceByName("grand_parent_model::parent_model::robot2");

  // There should be a body with the name "base_link".
  EXPECT_TRUE(plant_.HasBodyNamed("base_link", robot2_model));
  // There should be another body with the name "moving_link".
  EXPECT_TRUE(plant_.HasBodyNamed("moving_link", robot2_model));
  // There should be joint with the name "slider".
  EXPECT_TRUE(plant_.HasJointNamed("slider", robot2_model));

  // There should be a joint named "weld_robots". By convention, the joint
  // will have the same model instance as the child frame.
  EXPECT_TRUE(plant_.HasJointNamed("weld_robots", robot2_model));
}

// Example model taken from
// http://sdformat.org/tutorials?tut=composition_proposal&cat=pose_semantics_docs&#1-4-4-placement-frame-model-placement_frame-and-include-placement_frame
TEST_F(SdfParserTest, ModelPlacementFrame) {
  AddSceneGraph();
  const std::string model_string = R"""(
<model name='table'> <!-- T -->
  <pose>0 10 0  0 0 0</pose>
  <link name='table_top'> <!-- S -->
    <pose>0 0 1  0 0 0</pose>
  </link>

  <model name='mug' placement_frame='base'> <!-- M -->
    <pose relative_to='table_top'/>
    <link name='handle'> <!-- H -->
      <pose>0.1 0 0  0 0 0</pose>
    </link>
    <link name='base'> <!-- B -->
      <pose>0 0 -0.1  0 0 0</pose>
    </link>
  </model>

</model>)""";
  ParseTestString(model_string, "1.8");
  plant_.Finalize();
  EXPECT_GT(plant_.num_positions(), 0);
  auto context = plant_.CreateDefaultContext();

  ASSERT_TRUE(plant_.HasModelInstanceNamed("table::mug"));
  ModelInstanceIndex model_m = plant_.GetModelInstanceByName("table::mug");

  ASSERT_TRUE(plant_.HasFrameNamed("__model__", model_m));
  const Frame<double>& frame_M = plant_.GetFrameByName("__model__", model_m);

  ASSERT_TRUE(plant_.HasFrameNamed("table_top"));
  const Frame<double>& frame_S = plant_.GetFrameByName("table_top");

  ASSERT_TRUE(plant_.HasFrameNamed("base", model_m));
  const Frame<double>& frame_B = plant_.GetFrameByName("base", model_m);

  ASSERT_TRUE(plant_.HasFrameNamed("handle", model_m));
  const Frame<double>& frame_H = plant_.GetFrameByName("handle", model_m);

  // X_SM = X_SB * X_MB^-1.
  const RigidTransformd X_SM_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.0, 0.0, 0.1));

  const RigidTransformd X_SB_expected = RigidTransformd::Identity();
  // X_SH = X_SB * X_HB^-1.
  //      = X_SB * (X_MH^-1 * X_MB)^-1.
  const RigidTransformd X_SH_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.1, 0.0, 0.1));

  const RigidTransformd X_SM = frame_M.CalcPose(*context, frame_S);
  const RigidTransformd X_SH = frame_H.CalcPose(*context, frame_S);
  const RigidTransformd X_SB = frame_B.CalcPose(*context, frame_S);

  EXPECT_TRUE(
      CompareMatrices(X_SM_expected.GetAsMatrix4(), X_SM.GetAsMatrix4(), kEps));
  EXPECT_TRUE(
      CompareMatrices(X_SB_expected.GetAsMatrix4(), X_SB.GetAsMatrix4(), kEps));
  EXPECT_TRUE(
      CompareMatrices(X_SH_expected.GetAsMatrix4(), X_SH.GetAsMatrix4(), kEps));

  // X_WM = X_WT * X_TM
  // X_TM = X_TS * X_MS^-1
  // X_MS = X_MB * X_SB^-1
  // The model frame M is 0.1m in the +z axis from frame B, but we know from the
  // use of placement_frame that frame B and frame S are coincident. So X_WM is
  // 0.1m in the +z axis from frame S.
  const RigidTransformd X_WM_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.0, 10.0, 1.1));

  const RigidTransformd X_WB_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.0, 10.0, 1.0));

  const RigidTransformd X_WH_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.1, 10.0, 1.1));

  const RigidTransformd X_WM = frame_M.CalcPoseInWorld(*context);
  const RigidTransformd X_WH = frame_H.CalcPoseInWorld(*context);
  const RigidTransformd X_WB = frame_B.CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WM_expected.GetAsMatrix4(), X_WM.GetAsMatrix4(), kEps));
  EXPECT_TRUE(
      CompareMatrices(X_WB_expected.GetAsMatrix4(), X_WB.GetAsMatrix4(), kEps));
  EXPECT_TRUE(
      CompareMatrices(X_WH_expected.GetAsMatrix4(), X_WH.GetAsMatrix4(), kEps));
}

// Verify that poses can be given relative to deeply nested frames.
TEST_F(SdfParserTest, PoseRelativeToMultiLevelNestedFrame) {
  AddSceneGraph();
  const std::string model_string = R"""(
<model name='a'>
  <pose>0.1 0 0  0 0 0</pose>
  <model name='b'>
    <pose>0 0.2 0.0  0 0 0</pose>
    <model name='c'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='d'>
        <pose>0 0.0 0.0  0 0.5 0.6</pose>
      </link>
    </model>
  </model>
  <link name='e'>
    <pose relative_to="b::c::d">0 0 0  0.4 0 0.0</pose>
  </link>
</model>)""";
  ParseTestString(model_string, "1.8");
  plant_.Finalize();
  EXPECT_GT(plant_.num_positions(), 0);
  auto context = plant_.CreateDefaultContext();

  const RigidTransformd X_WE_expected(RollPitchYawd(0.4, 0.5, 0.6),
                                      Vector3d(0.1, 0.2, 0.3));

  const RigidTransformd X_WE =
      plant_.GetFrameByName("e").CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WE_expected.GetAsMatrix4(), X_WE.GetAsMatrix4(), kEps));
}

// Verify that joint axis can be expressed in deeply nested frames.
TEST_F(SdfParserTest, AxisXyzExperssedInMultiLevelNestedFrame) {
  AddSceneGraph();
  const std::string model_string = fmt::format(R"""(
<model name='a'>
  <pose>0.1 0 0  0 0 0</pose>
  <model name='b'>
    <pose>0 0.2 0.0  0 0 0</pose>
    <model name='c'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='d'>
        <pose>0 0.0 0.0  0 {} {}</pose>
      </link>
    </model>
  </model>
  <link name='e'/>
  <link name='f'/>
  <joint name="j" type="revolute">
    <parent>e</parent>
    <child>f</child>
    <axis>
      <xyz expressed_in="b::c::d">1 0 0</xyz>
    </axis>
  </joint>
</model>)""",
                                               M_PI_2, M_PI_2);
  ParseTestString(model_string, "1.8");
  plant_.Finalize();
  EXPECT_GT(plant_.num_positions(), 0);
  auto context = plant_.CreateDefaultContext();

  const RollPitchYawd R_WD(0.0, M_PI_2, M_PI_2);

  const Vector3d xyz_D(1, 0, 0);

  const Vector3d xyz_W_expected = R_WD.ToRotationMatrix() * xyz_D;

  DRAKE_EXPECT_NO_THROW(plant_.GetJointByName<RevoluteJoint>("j"));
  const RevoluteJoint<double>& joint_j =
      plant_.GetJointByName<RevoluteJoint>("j");
  EXPECT_TRUE(CompareMatrices(xyz_W_expected, joint_j.revolute_axis(), kEps));
}

// Verify frames can be attached to nested links or models
TEST_F(SdfParserTest, FrameAttachedToMultiLevelNestedFrame) {
  AddSceneGraph();
  const std::string model_string = R"""(
<model name='a'>
  <pose>0.1 0 0  0 0 0</pose>
  <model name='b'>
    <pose>0 0.2 0.0  0 0 0</pose>
    <model name='c'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='d'>
        <pose>0 0.0 0.0  0 0.5 0.6</pose>
      </link>
    </model>
  </model>
  <frame name='e' attached_to='b::c::d'> <!-- Frame attached to a link -->
    <pose>0 0 0  0.4 0 0.0</pose>
  </frame>
  <frame name='f' attached_to='b::c'> <!-- Frame attached to a model -->
    <pose>0 0 0  0.4 0.5 0.6</pose>
  </frame>
</model>)""";
  ParseTestString(model_string, "1.8");
  plant_.Finalize();
  EXPECT_GT(plant_.num_positions(), 0);
  auto context = plant_.CreateDefaultContext();

  const RigidTransformd X_WE_expected(RollPitchYawd(0.4, 0.5, 0.6),
                                      Vector3d(0.1, 0.2, 0.3));
  const RigidTransformd X_WF_expected(RollPitchYawd(0.4, 0.5, 0.6),
                                      Vector3d(0.1, 0.2, 0.3));

  const auto& frame_E = plant_.GetFrameByName("e");
  const RigidTransformd X_WE = frame_E.CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WE_expected.GetAsMatrix4(), X_WE.GetAsMatrix4(), kEps));

  const auto& frame_F = plant_.GetFrameByName("f");
  const RigidTransformd X_WF = frame_F.CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WF_expected.GetAsMatrix4(), X_WF.GetAsMatrix4(), kEps));

  // Also check that the frame is attached to the right body
  ModelInstanceIndex model_c_instance =
      plant_.GetModelInstanceByName("a::b::c");
  EXPECT_EQ(frame_E.body().index(),
            plant_.GetBodyByName("d", model_c_instance).index());

  EXPECT_EQ(frame_F.body().index(),
            plant_.GetBodyByName("d", model_c_instance).index());
}

// Verify frames and links can have the same local name without violating name
// uniqueness requirements
TEST_F(SdfParserTest, RepeatedLinkName) {
  AddSceneGraph();
  const std::string model_string = R"""(
<world name='a'>
  <model name='b1'>
    <link name='c'/>
    <frame name='d'/>
  </model>
  <model name='b2'>
    <link name='c'/>
    <frame name='d'/>
  </model>
</world>)""";
  DRAKE_ASSERT_NO_THROW(ParseTestString(model_string, "1.8"));
}

// Verify frames can be attached to models in a SDFormat world
TEST_F(SdfParserTest, FrameAttachedToModelFrameInWorld) {
  AddSceneGraph();
  const std::string model_string = R"""(
<world name='a'>
  <model name='b'>
    <pose>0.1 0.2 0.0  0 0 0</pose>
    <model name='c'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='d'/>
    </model>
  </model>
  <frame name='e' attached_to='b'>
    <pose>0 0 0.3  0.0 0.0 0.0</pose>
  </frame>
  <frame name='f' attached_to='b::c'>
    <pose>0 0 0  0.0 0.0 0.6</pose>
  </frame>
</world>)""";
  ParseTestString(model_string, "1.8");

  plant_.Finalize();
  EXPECT_GT(plant_.num_positions(), 0);
  auto context = plant_.CreateDefaultContext();

  const RigidTransformd X_WE_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                      Vector3d(0.1, 0.2, 0.3));
  const RigidTransformd X_WF_expected(RollPitchYawd(0.0, 0.0, 0.6),
                                      Vector3d(0.1, 0.2, 0.3));

  const auto& frame_E = plant_.GetFrameByName("e");
  const RigidTransformd X_WE = frame_E.CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WE_expected.GetAsMatrix4(), X_WE.GetAsMatrix4(), kEps));

  const auto& frame_F = plant_.GetFrameByName("f");
  const RigidTransformd X_WF = frame_F.CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WF_expected.GetAsMatrix4(), X_WF.GetAsMatrix4(), kEps));

  // Also check that the frame is attached to the right body
  EXPECT_EQ(frame_E.body().index(), plant_.GetBodyByName("d").index());

  EXPECT_EQ(frame_F.body().index(), plant_.GetBodyByName("d").index());
}

// Verify frames can be attached to joint frames
TEST_F(SdfParserTest, FrameAttachedToJointFrame) {
  AddSceneGraph();
  const std::string model_string = R"""(
<world name='default'>
  <model name='parent_model'>
    <pose>0.1 0.2 0.0  0 0 0</pose>
    <link name='L1'/>
    <link name='L2'/>
    <model name='M1'>
      <pose>0 0.0 0.3  0 0 0</pose>
      <link name='L3'/>
    </model>
    <!-- SDFormat has implicit frames for joints J1 and J2, but Drake does not
    -->
    <joint name='J1' type='fixed'>
      <parent>L1</parent>
      <child>L2</child>
    </joint>
    <joint name='J2' type='fixed'>
      <parent>L1</parent>
      <child>M1::L3</child>
    </joint>
    <frame name='F1' attached_to='J1'/>
    <frame name='F2' attached_to='J2'/>
  </model>
</world>)""";
  ParseTestString(model_string, "1.8");

  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  const RigidTransformd X_WF1_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                       Vector3d(0.1, 0.2, 0.0));

  const RigidTransformd X_WF2_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                       Vector3d(0.1, 0.2, 0.3));

  const auto& frame_F1 = plant_.GetFrameByName("F1");
  const auto& frame_F2 = plant_.GetFrameByName("F2");
  const RigidTransformd X_WF1 = frame_F1.CalcPoseInWorld(*context);
  const RigidTransformd X_WF2 = frame_F2.CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(X_WF1_expected.GetAsMatrix4(),
                              X_WF1.GetAsMatrix4(), kEps));
  EXPECT_TRUE(CompareMatrices(X_WF2_expected.GetAsMatrix4(),
                              X_WF2.GetAsMatrix4(), kEps));

  // Also check that the frame is attached to the right body
  EXPECT_EQ(frame_F1.body().index(), plant_.GetBodyByName("L2").index());
  EXPECT_EQ(frame_F2.body().index(), plant_.GetBodyByName("L3").index());
}

TEST_F(SdfParserTest, SupportNonDefaultCanonicalLink) {
  AddSceneGraph();
  // Verify that non-default canonical links are handled properly. Here we have
  // three different types of references used for the canonical link:
  // * c::e - Nested link
  // * f - Link that is not the first link in the model
  const std::string model_string = R"""(
  <model name='a' canonical_link='c::e'>
    <link name='b'/>
    <model name='c' canonical_link='f'>
      <link name='d'/>
      <link name='e'/>
      <link name='f'/>
    </model>
  </model>)""";
  ParseTestString(model_string, "1.8");

  plant_.Finalize();

  EXPECT_EQ(GetModelFrameByName(plant_, "a").body().index(),
            plant_.GetBodyByName("e").index());

  EXPECT_EQ(GetModelFrameByName(plant_, "a::c").body().index(),
            plant_.GetBodyByName("f").index());
}

// Verify that frames can be used for //joint/parent and //joint/child
TEST_F(SdfParserTest, FramesAsJointParentOrChild) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "frames_as_joint_parent_or_child.sdf");

  AddModelsFromSdfFile(full_name);
  ASSERT_TRUE(plant_.HasModelInstanceNamed("parent_model"));

  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  const RigidTransformd X_CJc_expected = RigidTransformd::Identity();
  const RigidTransformd X_PJp_expected(RollPitchYawd(0, 0, 0),
                                       Vector3d(3, 3, 3));

  // Frames attached to links in the same model
  {
    const auto& joint = plant_.GetJointByName("J1");
    const auto& frame_P = plant_.GetFrameByName("L1_offset");
    const auto& frame_C = plant_.GetFrameByName("L2_offset");

    const RigidTransformd X_PJp =
        joint.frame_on_parent().CalcPose(*context, frame_P);
    EXPECT_TRUE(CompareMatrices(X_PJp_expected.GetAsMatrix4(),
                                X_PJp.GetAsMatrix4(), kEps));

    const RigidTransformd X_CJc =
        joint.frame_on_child().CalcPose(*context, frame_C);
    EXPECT_TRUE(CompareMatrices(X_CJc_expected.GetAsMatrix4(),
                                X_CJc.GetAsMatrix4(), kEps));
  }
  // Frames attached to links in the other (nested) models
  {
    const auto& joint = plant_.GetJointByName("J2");

    const auto& frame_P = plant_.GetFrameByName("M1_base_link_offset");
    const auto& frame_C = plant_.GetFrameByName("M2_base_link_offset");

    const RigidTransformd X_PJp =
        joint.frame_on_parent().CalcPose(*context, frame_P);
    EXPECT_TRUE(CompareMatrices(X_PJp_expected.GetAsMatrix4(),
                                X_PJp.GetAsMatrix4(), kEps));

    const RigidTransformd X_CJc =
        joint.frame_on_child().CalcPose(*context, frame_C);
    EXPECT_TRUE(CompareMatrices(X_CJc_expected.GetAsMatrix4(),
                                X_CJc.GetAsMatrix4(), kEps));
  }
}

// Verifies that URDF files can be loaded into Drake via libsdformat's Interface
// API which bypasses the URDF->SDFormat conversion. This also verifies that
// SDFormat files can be forced to be loaded via the Interface API by changing
// their file extension and registering the appropriate custom parser.
TEST_F(SdfParserTest, InterfaceApi) {
  AddSceneGraph();
  const std::string sdf_file_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/interface_api_test/"
      "top.sdf");
  package_map_.AddPackageXml(FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/interface_api_test/"
      "package.xml"));

  DRAKE_ASSERT_NO_THROW(AddModelFromSdfFile(sdf_file_path, ""));

  // Test collision filtering across models. Done before Finalize() to exclude
  // default filters.
  {
    const auto& inspector = scene_graph_.model_inspector();
    static constexpr int kNumLinks = 4;
    // Verify the number we expect and that they are all in proximity role.
    ASSERT_EQ(kNumLinks, inspector.num_geometries());
    ASSERT_EQ(kNumLinks,
              inspector.NumGeometriesWithRole(geometry::Role::kProximity));
    std::vector<GeometryId> ids = inspector.GetAllGeometryIds();
    ASSERT_EQ(ids.size(), kNumLinks);

    // Make sure the plant is not finalized such that the Finalize() default
    // filtering has not taken into effect yet. This guarantees that the
    // collision filtering is applied due to the collision filter group parsing.
    ASSERT_FALSE(plant_.is_finalized());

    // Verify filtering among all links.
    std::set<CollisionPair> expected_filters = {
        {"top::arm::L1", "top::gripper::gripper_link"},
        {"top::arm::L1", "top::torso"},
        {"top::gripper::gripper_link", "top::torso"},
    };
    VerifyCollisionFilters(ids, expected_filters);

    // Verify parser-level collision filter reporting.
    CollisionFilterGroupsImpl<std::string> expected_report;
    expected_report.AddGroup(
        "top::g1",
        {"top::arm::L1", "top::gripper::gripper_link", "top::torso"});
    expected_report.AddExclusionPair({"top::g1", "top::g1"});
    EXPECT_EQ(last_parsed_groups_, expected_report);
  }

  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  {
    // Frame A represents the model frame of model top::arm
    const RigidTransformd X_WA_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                        Vector3d(1, 0, 0));
    const auto arm_model_instance = plant_.GetModelInstanceByName("top::arm");
    const auto& arm_model_frame =
        plant_.GetFrameByName("__model__", arm_model_instance);
    const RigidTransformd X_WA = arm_model_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WA_expected.GetAsMatrix4(),
                                X_WA.GetAsMatrix4(), kEps));
    const auto& arm_L1 = plant_.GetFrameByName("L1", arm_model_instance);
    const RigidTransformd X_WL1 = arm_L1.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WA_expected.GetAsMatrix4(),
                                X_WL1.GetAsMatrix4(), kEps));
  }

  // Test placement_frame using a table and a mug flipped upside down
  {
    // Frame T represents the frame top::table_and_mug::mug::top
    const RigidTransformd X_WT_expected(RollPitchYawd(M_PI_2, 0.0, 0.0),
                                        Vector3d(3, 0, 0.5));
    const auto mug_model_instance = plant_.GetModelInstanceByName("top::mug");
    const auto& mug_top_frame =
        plant_.GetFrameByName("top", mug_model_instance);
    const RigidTransformd X_WT = mug_top_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WT_expected.GetAsMatrix4(),
                                X_WT.GetAsMatrix4(), kEps));
  }
}

// Verifies that parser diagnostics from URDF files included by SDFormat files
// make it through the entire call stack.
TEST_F(SdfParserTest, ErrorsFromIncludedUrdf) {
  ParseTestString(R"""(
<model name="top">
  <link name="torso"/>
  <include>
    <pose relative_to="torso">1 0 0 0 0 0</pose>
    <uri>package://drake/multibody/parsing/test/sdf_parser_test/bad.urdf</uri>
    <name>arm</name>
 </include>
</model>)""",
                  "1.8");
  EXPECT_THAT(TakeError(), MatchesRegex(".*bad.urdf.*XML_ERROR.*"));
}

// TODO(SeanCurtis-TRI) The logic testing for collision filter group parsing
// belongs in detail_common_test.cc. Urdf and Sdf parsing just need enough
// testing to indicate that the method is being invoked correctly.
TEST_F(SdfParserTest, CollisionFilterGroupParsingTest) {
  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
      "sdf_parser_test/collision_filter_group_parsing_test/test.sdf");
  AddSceneGraph();

  // Read in the SDF file.
  package_map_.PopulateFromFolder(
      std::filesystem::path(full_sdf_filename).parent_path());
  AddModelFromSdfFile(full_sdf_filename, "");

  // Get geometry ids for all the bodies.
  const auto& inspector = scene_graph_.model_inspector();
  static constexpr int kNumRobots = 2;
  static constexpr int kNumLinksPerRobot = 6;
  static constexpr int kNumLinks = kNumLinksPerRobot * kNumRobots;
  // Verify the number we expect and that they are all in proximity role.
  ASSERT_EQ(kNumLinks, inspector.num_geometries());
  ASSERT_EQ(kNumLinks,
            inspector.NumGeometriesWithRole(geometry::Role::kProximity));
  std::vector<GeometryId> ids = inspector.GetAllGeometryIds();
  ASSERT_EQ(ids.size(), kNumLinks);

  // Make sure the plant is not finalized such that the Finalize() default
  // filtering has not taken into effect yet. This guarantees that the
  // collision filtering is applied due to the collision filter group parsing.
  ASSERT_FALSE(plant_.is_finalized());

  // Verify filtering among all links.
  std::set<CollisionPair> expected_filters = {
      // Filtered within robot1.
      {"test::robot1::link1_sphere", "test::robot1::link3_sphere"},
      {"test::robot1::link1_sphere", "test::robot1::link4_sphere"},
      {"test::robot1::link2_sphere", "test::robot1::link3_sphere"},
      {"test::robot1::link2_sphere", "test::robot1::link5_sphere"},
      {"test::robot1::link2_sphere", "test::robot1::link6_sphere"},
      {"test::robot1::link3_sphere", "test::robot1::link4_sphere"},
      {"test::robot1::link3_sphere", "test::robot1::link5_sphere"},
      {"test::robot1::link3_sphere", "test::robot1::link6_sphere"},
      {"test::robot1::link5_sphere", "test::robot1::link6_sphere"},

      // Filtered across both robots.
      {"test::robot1::link3_sphere", "test::robot2::link3_sphere"},
      {"test::robot1::link6_sphere", "test::robot2::link6_sphere"},

      // Filtered within robot2.
      {"test::robot2::link1_sphere", "test::robot2::link3_sphere"},
      {"test::robot2::link1_sphere", "test::robot2::link4_sphere"},
      {"test::robot2::link2_sphere", "test::robot2::link3_sphere"},
      {"test::robot2::link2_sphere", "test::robot2::link5_sphere"},
      {"test::robot2::link2_sphere", "test::robot2::link6_sphere"},
      {"test::robot2::link3_sphere", "test::robot2::link4_sphere"},
      {"test::robot2::link3_sphere", "test::robot2::link5_sphere"},
      {"test::robot2::link3_sphere", "test::robot2::link6_sphere"},
      {"test::robot2::link5_sphere", "test::robot2::link6_sphere"},

      // Filtered by group_of_groups.
      {"test::robot1::link2_sphere", "test::robot2::link3_sphere"},
  };
  VerifyCollisionFilters(ids, expected_filters);

  // Verify parser-level collision filter reporting.
  CollisionFilterGroupsImpl<std::string> expected_report;
  expected_report.AddGroup("test::group_3s", {"test::robot1::link3"});
  expected_report.AddGroup("test::group_6s",
                           {"test::robot1::link6", "test::robot2::link6"});
  expected_report.AddGroup("test::group_of_groups",
                           {"test::robot1::link2", "test::robot2::link3"});

  expected_report.AddGroup("test::robot1::group_link14",
                           {"test::robot1::link1", "test::robot1::link4"});
  expected_report.AddGroup("test::robot1::group_link2",
                           {"test::robot1::link2"});
  expected_report.AddGroup("test::robot1::group_link3",
                           {"test::robot1::link3"});
  expected_report.AddGroup("test::robot1::group_link56",
                           {"test::robot1::link5", "test::robot1::link6"});

  expected_report.AddGroup("test::robot2::group_link14",
                           {"test::robot2::link1", "test::robot2::link4"});
  expected_report.AddGroup("test::robot2::group_link2",
                           {"test::robot2::link2"});
  expected_report.AddGroup("test::robot2::group_link3",
                           {"test::robot2::link3"});
  expected_report.AddGroup("test::robot2::group_link56",
                           {"test::robot2::link5", "test::robot2::link6"});

  expected_report.AddExclusionPair(
      {"test::group_3s", "test::robot2::group_link3"});
  expected_report.AddExclusionPair({"test::group_6s", "test::group_6s"});
  expected_report.AddExclusionPair(
      {"test::group_of_groups", "test::group_of_groups"});
  expected_report.AddExclusionPair(
      {"test::robot1::group_link14", "test::robot1::group_link14"});
  expected_report.AddExclusionPair(
      {"test::robot1::group_link14", "test::robot1::group_link3"});
  expected_report.AddExclusionPair(
      {"test::robot1::group_link2", "test::robot1::group_link3"});
  expected_report.AddExclusionPair(
      {"test::robot1::group_link2", "test::robot1::group_link56"});
  expected_report.AddExclusionPair(
      {"test::robot1::group_link3", "test::robot1::group_link56"});
  expected_report.AddExclusionPair(
      {"test::robot1::group_link56", "test::robot1::group_link56"});
  expected_report.AddExclusionPair(
      {"test::robot2::group_link14", "test::robot2::group_link14"});
  expected_report.AddExclusionPair(
      {"test::robot2::group_link14", "test::robot2::group_link3"});
  expected_report.AddExclusionPair(
      {"test::robot2::group_link2", "test::robot2::group_link3"});
  expected_report.AddExclusionPair(
      {"test::robot2::group_link2", "test::robot2::group_link56"});
  expected_report.AddExclusionPair(
      {"test::robot2::group_link3", "test::robot2::group_link56"});
  expected_report.AddExclusionPair(
      {"test::robot2::group_link56", "test::robot2::group_link56"});

  EXPECT_EQ(last_parsed_groups_, expected_report);

  // Make sure we can add the model a second time.
  AddModelFromSdfFile(full_sdf_filename, "model2");
}

TEST_F(SdfParserTest, CollisionFilterGroupParsingErrorsTest) {
  AddSceneGraph();
  DRAKE_EXPECT_NO_THROW(ParseTestString(R"""(
<model name='error1'>
  <link name='a'/>
  <drake:collision_filter_group/>
</model>)"""));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*The tag <drake:collision_filter_group> is "
                           "missing the required attribute \"name\".*"));
  FlushDiagnostics();

  // Testing several errors set to keep record instead of throwing
  DRAKE_EXPECT_NO_THROW(ParseTestString(R"""(
<model name='error2'>
  <link name='a'/>
  <drake:collision_filter_group name="group_a">
    <drake:member></drake:member>
    <drake:member_group></drake:member_group>
  </drake:collision_filter_group>
</model>)"""));
  EXPECT_THAT(TakeError(), MatchesRegex(".*The tag <drake:member> is missing"
                                        " a required string value.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*The tag <drake:member_group> is"
                                        " missing a required string value.*"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'error2::group_a'.*no members"));
  FlushDiagnostics();

  DRAKE_EXPECT_NO_THROW(ParseTestString(R"""(
<model name='error3'>
  <link name='a'/>
  <drake:collision_filter_group name="group_a">
    <drake:ignored_collision_filter_group>
    </drake:ignored_collision_filter_group>
  </drake:collision_filter_group>
</model>)"""));
  EXPECT_THAT(TakeError(), MatchesRegex(".*'error3::group_a'.*no members"));
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*The tag <drake:ignored_collision_filter_group> is missing"
                   " a required string value.*"));
  FlushDiagnostics();
}

TEST_F(SdfParserTest, PoseWithRotationInDegreesOrQuaternions) {
  AddSceneGraph();
  const std::string model_string = R"""(
  <model name='test_model'>
    <link name='E1'>
      <pose degrees='true'>1 2 3   15 30 45</pose>
    </link>
    <link name='E2'>
      <pose rotation_format="euler_rpy" degrees='true'>1 2 3   15 30 45</pose>
    </link>
    <link name='Q'>
      <pose rotation_format='quat_xyzw'>
        1 2 3
        0.3826834323650898 0 0 0.9238795325112867
      </pose>
    </link>
  </model>)""";
  ParseTestString(model_string, "1.9");
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  constexpr double kDegToRad = M_PI / 180.0;

  const RigidTransformd X_WE_expected(
      RollPitchYawd(kDegToRad * 15, kDegToRad * 30, kDegToRad * 45),
      Vector3d(1, 2, 3));

  const RigidTransformd X_WE1 =
      plant_.GetFrameByName("E1").CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(X_WE_expected.GetAsMatrix4(),
                              X_WE1.GetAsMatrix4(), kEps));

  const RigidTransformd X_WE2 =
      plant_.GetFrameByName("E2").CalcPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(X_WE_expected.GetAsMatrix4(),
                              X_WE2.GetAsMatrix4(), kEps));

  const RigidTransformd X_WQ_expected(RollPitchYawd(M_PI_4, 0, 0),
                                      Vector3d(1, 2, 3));

  const RigidTransformd X_WQ =
      plant_.GetFrameByName("Q").CalcPoseInWorld(*context);
  EXPECT_TRUE(
      CompareMatrices(X_WQ_expected.GetAsMatrix4(), X_WQ.GetAsMatrix4(), kEps));
}

TEST_F(SdfParserTest, MergeInclude) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "merge_include_models.sdf");

  // We start with the world and default model instances.
  ASSERT_EQ(plant_.num_model_instances(), 2);
  ASSERT_EQ(plant_.num_bodies(), 1);
  ASSERT_EQ(plant_.num_joints(), 0);

  package_map_.PopulateFromFolder(
      std::filesystem::path(full_name).parent_path());
  AddModelsFromSdfFile(full_name);
  plant_.Finalize();

  // We should have loaded *only* 1 more model.
  EXPECT_EQ(plant_.num_model_instances(), 3);
  EXPECT_EQ(plant_.num_bodies(), 4);
  EXPECT_EQ(plant_.num_joints(), 3);

  ASSERT_TRUE(plant_.HasModelInstanceNamed("robot1_with_tool"));
  ModelInstanceIndex robot1_model =
      plant_.GetModelInstanceByName("robot1_with_tool");

  // The bodies and joints from "simple_robot1" should be merged into
  // "robot1_with_tool" making them direct children of the "robot1_with_tool"
  // model instance.
  EXPECT_TRUE(plant_.HasBodyNamed("base_link", robot1_model));
  EXPECT_TRUE(plant_.HasBodyNamed("moving_link", robot1_model));
  EXPECT_TRUE(plant_.HasJointNamed("slider", robot1_model));

  // The bodies and joints directly specified in "robot1_with_tool" should be at
  // the same level of hierarchy as those merged from "simple_robot1"
  EXPECT_TRUE(plant_.HasBodyNamed("tool", robot1_model));
  EXPECT_TRUE(plant_.HasJointNamed("tool_joint", robot1_model));
}

TEST_F(SdfParserTest, UnsupportedElements) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "unsupported_elements.sdf");

  AddSceneGraph();
  AddModelsFromSdfFile(full_name);

  EXPECT_THAT(TakeError(), MatchesRegex(".*drake:proximity_properties"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*self_collide"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*cast_shadows"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*transparency"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*lighting"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*script"));
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*shader"));
  EXPECT_THAT(TakeError(), MatchesRegex(".*drake:QQQ_dynamic"));
}

TEST_F(SdfParserTest, WorldJoint) {
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/"
      "world_joint_test.sdf");
  AddSceneGraph();
  AddModelsFromSdfFile(full_name);
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  ASSERT_TRUE(plant_.HasModelInstanceNamed("parent_model"));
  ASSERT_TRUE(plant_.HasModelInstanceNamed("child_model"));
  ASSERT_TRUE(plant_.HasFrameNamed("child_frame"));
  ASSERT_TRUE(plant_.HasJointNamed("J1"));
  ASSERT_TRUE(plant_.HasJointNamed("J2"));

  EXPECT_EQ(plant_.num_joints(), 2);
  EXPECT_EQ(plant_.num_model_instances(), 4);

  ModelInstanceIndex parent_instance =
      plant_.GetModelInstanceByName("parent_model");
  ModelInstanceIndex child_instance =
      plant_.GetModelInstanceByName("child_model");

  EXPECT_TRUE(plant_.HasBodyNamed("L_P", parent_instance));
  EXPECT_TRUE(plant_.HasBodyNamed("L_C", child_instance));

  const RigidBody<double>& parent_link =
      plant_.GetBodyByName("L_P", parent_instance);
  const RigidBody<double>& child_link =
      plant_.GetBodyByName("L_C", child_instance);
  EXPECT_NE(parent_link.index(), child_link.index());
  EXPECT_EQ(parent_link.model_instance(), parent_instance);
  EXPECT_EQ(child_link.model_instance(), child_instance);

  const Joint<double>& joint = plant_.GetJointByName<Joint>("J1");
  EXPECT_EQ(joint.name(), "J1");
  EXPECT_EQ(joint.parent_body().name(), "L_P");
  EXPECT_EQ(joint.child_body().name(), "L_C");

  // check relative pose between frames and joint
  const RigidTransformd X_CJc_expected(RollPitchYawd(0, 0, 0),
                                       Vector3d(1, 1, 1));
  const RigidTransformd X_PJp_expected(RollPitchYawd(0, 0, 0),
                                       Vector3d(4, 4, 4));

  const auto& frame_P = plant_.GetFrameByName("parent_frame");
  const auto& frame_C = plant_.GetFrameByName("child_frame");

  const RigidTransformd X_PJp =
      joint.frame_on_parent().CalcPose(*context, frame_P);
  EXPECT_TRUE(CompareMatrices(X_PJp_expected.GetAsMatrix4(),
                              X_PJp.GetAsMatrix4(), kEps));

  const RigidTransformd X_CJc =
      joint.frame_on_child().CalcPose(*context, frame_C);
  EXPECT_TRUE(CompareMatrices(X_CJc_expected.GetAsMatrix4(),
                              X_CJc.GetAsMatrix4(), kEps));
}

// Tests the error handling for an unsupported visual geometry.
TEST_F(SdfParserTest, TestUnsupportedVisualGeometry) {
  AddSceneGraph();
  ParseTestString(R"""(
  <model name="heightmap_model">
    <link name="a">
      <visual name="b">
        <geometry>
          <heightmap/>
        </geometry>
      </visual>
    </link>
  </model>)""");
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(
          ".*Ignoring unsupported SDFormat element in geometry: heightmap.*"));
  FlushDiagnostics();

  ParseTestString(R"""(
  <model name="polyline_model">
    <link name="a">
      <visual name="b">
        <geometry>
          <polyline/>
        </geometry>
      </visual>
    </link>
  </model>)""");
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(
          ".*Ignoring unsupported SDFormat element in geometry: polyline.*"));
}

// Tests the error handling for an unsupported collision geometry.
TEST_F(SdfParserTest, TestUnsupportedCollisionGeometry) {
  AddSceneGraph();
  ParseTestString(R"""(
  <model name="heightmap_model">
    <link name="a">
      <collision name="b">
        <geometry>
          <heightmap/>
        </geometry>
      </collision>
    </link>
  </model>)""");
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(
          ".*Ignoring unsupported SDFormat element in geometry: heightmap.*"));
  FlushDiagnostics();

  ParseTestString(R"""(
  <model name="polyline_model">
    <link name="a">
      <collision name="b">
        <geometry>
          <polyline/>
        </geometry>
      </collision>
    </link>
  </model>)""");
  EXPECT_THAT(
      TakeWarning(),
      MatchesRegex(
          ".*Ignoring unsupported SDFormat element in geometry: polyline.*"));
}

// Regression test for #18878.
TEST_F(SdfParserTest, TestSingleModelInWorld) {
  const std::string full_sdf_filename = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/table_in_world.sdf");

  // Read in the SDF file.
  AddModelFromSdfFile(full_sdf_filename, "");
  plant_.Finalize();

  // Verify the number of model instances.
  EXPECT_EQ(plant_.num_model_instances(), 3);
}

// Test case discussed during solution of #18878. This error is caught by
// sdformat library processing.
TEST_F(SdfParserTest, TestSingleModelEnforcement) {
  const std::string multi_models = R"""(
  <sdf version='1.9'>
  <model name='a'><link name='a'/></model>
  <model name='b'><link name='b'/></model>
  </sdf>
)""";

  const DataSource data_source{DataSource::kContents, &multi_models};
  internal::CollisionFilterGroupResolver resolver{&plant_};
  ParsingWorkspace w{options_, package_map_, diagnostic_policy_, nullptr,
                     &plant_,  &resolver,    TestingSelect};
  std::optional<ModelInstanceIndex> result =
      AddModelFromSdf(data_source, "", {}, w);
  resolver.Resolve(diagnostic_policy_);
  EXPECT_FALSE(result.has_value());

  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Root object can only contain one model.*"));
}

// Verify merge-include works with Interface API.
TEST_F(SdfParserTest, BasicMergeIncludeInterfaceApi) {
  AddSceneGraph();
  const std::string full_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/interface_api_test/"
      "arm_with_gripper_merge_include.sdf");

  // We start with the world and default model instances.
  ASSERT_EQ(plant_.num_model_instances(), 2);
  ASSERT_EQ(plant_.num_bodies(), 1);
  ASSERT_EQ(plant_.num_joints(), 0);

  package_map_.PopulateFromFolder(
      std::filesystem::path(full_name).parent_path());
  AddModelsFromSdfFile(full_name);
  plant_.Finalize();

  // We should have loaded *only* 1 more model.
  EXPECT_EQ(plant_.num_model_instances(), 3);
  EXPECT_EQ(plant_.num_bodies(), 4);
  EXPECT_EQ(plant_.num_joints(), 3);

  const char* model_name = "arm_with_gripper";
  ASSERT_TRUE(plant_.HasModelInstanceNamed(model_name));
  ModelInstanceIndex arm_model = plant_.GetModelInstanceByName(model_name);

  // Check that the links and joints from arm.urdf are included in the model
  // without introducing a nested scope, thus exercising the merge-include
  // behavior through SDFormat's Interface API.
  EXPECT_TRUE(plant_.HasBodyNamed("L1", arm_model));
  EXPECT_TRUE(plant_.HasBodyNamed("L2", arm_model));
  EXPECT_TRUE(plant_.HasJointNamed("J1", arm_model));

  // Check gripper.sdf is included in the model without introducing a nested
  // scope showing that both urdf and sdf files can be merge included into a
  // single containing model.
  EXPECT_TRUE(plant_.HasBodyNamed("gripper_link", arm_model));
}

void TestMergeIncludeWithInterfaceApi(const MultibodyPlant<double>& plant,
                                      const SceneGraph<double>& scene_graph,
                                      const std::string model_prefix) {
  SCOPED_TRACE(model_prefix);
  auto context = plant.CreateDefaultContext();
  EXPECT_FALSE(
      plant.HasModelInstanceNamed(sdf::JoinName(model_prefix, "arm::gripper")));
  EXPECT_FALSE(plant.HasModelInstanceNamed(
      sdf::JoinName(model_prefix, "arm_sdf::test_arm_sdf_name")));
  EXPECT_FALSE(plant.HasModelInstanceNamed(
      sdf::JoinName(model_prefix, "arm_urdf::test_arm_urdf_name")));

  EXPECT_FALSE(plant.HasModelInstanceNamed(
      sdf::JoinName(model_prefix, "arm_sdf_name_override::test_arm_sdf_name")));
  EXPECT_FALSE(plant.HasModelInstanceNamed(sdf::JoinName(
      model_prefix, "arm_urdf_name_override::test_arm_urdf_name")));

  ASSERT_TRUE(
      plant.HasModelInstanceNamed(sdf::JoinName(model_prefix, "arm_urdf")));
  const auto arm_urdf_model_instance =
      plant.GetModelInstanceByName(sdf::JoinName(model_prefix, "arm_urdf"));
  EXPECT_TRUE(plant.HasFrameNamed(sdf::computeMergedModelProxyFrameName("arm"),
                                  arm_urdf_model_instance));

  const auto arm_urdf_name_override_model_instance =
      plant.GetModelInstanceByName(
          sdf::JoinName(model_prefix, "arm_urdf_name_override"));

  EXPECT_TRUE(plant.HasFrameNamed(
      sdf::computeMergedModelProxyFrameName("test_arm_urdf_name"),
      arm_urdf_name_override_model_instance));

  ASSERT_TRUE(
      plant.HasModelInstanceNamed(sdf::JoinName(model_prefix, "arm_sdf")));
  const auto arm_sdf_model_instance =
      plant.GetModelInstanceByName(sdf::JoinName(model_prefix, "arm_sdf"));

  ASSERT_TRUE(
      plant.HasModelInstanceNamed(sdf::JoinName(model_prefix, "arm_mjcf")));
  const auto arm_mjcf_model_instance =
      plant.GetModelInstanceByName(sdf::JoinName(model_prefix, "arm_mjcf"));

  // Pose of torso link
  const RigidTransformd X_WT(RollPitchYawd(0, 0, 0), Vector3d(0, 0, 1));

  {
    // Frame G represents the frame of model top::arm_sdf::gripper_frame
    const RigidTransformd X_WG_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                        Vector3d(1, 2, 4));
    const auto& grasp_frame =
        plant.GetFrameByName("grasp_frame", arm_sdf_model_instance);

    const RigidTransformd X_WG = grasp_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WG_expected.GetAsMatrix4(),
                                X_WG.GetAsMatrix4(), kEps));
  }
  {
    // Frame A represents the model frame of model top::arm_sdf
    const auto& arm_sdf_model_frame =
        plant.GetFrameByName("__model__", arm_sdf_model_instance);
    const RigidTransformd X_WA = arm_sdf_model_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(
        CompareMatrices(X_WT.GetAsMatrix4(), X_WA.GetAsMatrix4(), kEps));
    const RigidTransformd X_WL1_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                         Vector3d(1, 0, 1));
    const auto& arm_L1 = plant.GetFrameByName("L1", arm_sdf_model_instance);
    const RigidTransformd X_WL1 = arm_L1.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WL1_expected.GetAsMatrix4(),
                                X_WL1.GetAsMatrix4(), kEps));
  }

  {
    // Frame E represents the model frame of model top::arm_urdf
    const auto& arm_urdf_model_frame =
        plant.GetFrameByName("__model__", arm_urdf_model_instance);
    const RigidTransformd X_WE = arm_urdf_model_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(
        CompareMatrices(X_WT.GetAsMatrix4(), X_WE.GetAsMatrix4(), kEps));

    const RigidTransformd X_WL2_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                         Vector3d(1, 4, 4));
    const auto& arm_L2 = plant.GetFrameByName("L2", arm_urdf_model_instance);
    const RigidTransformd X_WL2 = arm_L2.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WL2_expected.GetAsMatrix4(),
                                X_WL2.GetAsMatrix4(), kEps));
  }
  {
    // Frame F represents the model frame of model top::arm_sdf::flange
    const RigidTransformd X_WF_expected(RollPitchYawd(0.0, 0.0, 0.0),
                                        Vector3d(1, 2, 2));
    const auto flange_model_instance = plant.GetModelInstanceByName(
        sdf::JoinName(model_prefix, "arm_sdf::flange"));
    const auto& flange_model_frame =
        plant.GetFrameByName("__model__", flange_model_instance);
    const RigidTransformd X_WF = flange_model_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WF_expected.GetAsMatrix4(),
                                X_WF.GetAsMatrix4(), kEps));

    // Frame M represents the frame of model top::arm::flange::gripper_mount
    const RigidTransformd X_WM_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                        Vector3d(1, 2, 4));
    const auto& gripper_mount_frame =
        plant.GetFrameByName("gripper_mount", flange_model_instance);
    const RigidTransformd X_WM = gripper_mount_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WM_expected.GetAsMatrix4(),
                                X_WM.GetAsMatrix4(), kEps));
  }
  {
    // Frame G represents the model frame of model top::arm_mjcf
    const auto& arm_mjcf_model_frame =
        plant.GetFrameByName("__model__", arm_mjcf_model_instance);
    const RigidTransformd X_WG = arm_mjcf_model_frame.CalcPoseInWorld(*context);
    EXPECT_TRUE(
        CompareMatrices(X_WT.GetAsMatrix4(), X_WG.GetAsMatrix4(), kEps));

    const RigidTransformd X_WL2_expected(RollPitchYawd(0.1, 0.2, 0.3),
                                         Vector3d(11, 4, 4));
    const auto& arm_L2 = plant.GetFrameByName("L2", arm_mjcf_model_instance);
    const RigidTransformd X_WL2 = arm_L2.CalcPoseInWorld(*context);
    EXPECT_TRUE(CompareMatrices(X_WL2_expected.GetAsMatrix4(),
                                X_WL2.GetAsMatrix4(), kEps));
  }
}

TEST_F(SdfParserTest, MergeIncludeInterfaceApi1) {
  AddSceneGraph();
  package_map_.AddPackageXml(FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/interface_api_test/"
      "package.xml"));
  const std::string sdf_file_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/interface_api_test/"
      "top_merge_include.sdf");

  AddModelFromSdfFile(sdf_file_path, "");

  plant_.Finalize();
  SCOPED_TRACE("MergeIncludeInterfaceApi1");
  TestMergeIncludeWithInterfaceApi(plant_, scene_graph_, "top");
}

TEST_F(SdfParserTest, MergeIncludeInterfaceApi2) {
  AddSceneGraph();
  package_map_.AddPackageXml(FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/interface_api_test/"
      "package.xml"));
  // Use AddModelsFromSdfFile (note the plural Models)
  const std::string sdf_file_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/interface_api_test/"
      "top_merge_include_world.sdf");

  AddModelsFromSdfFile(sdf_file_path);

  plant_.Finalize();
  SCOPED_TRACE("MergeIncludeInterfaceApi2");
  TestMergeIncludeWithInterfaceApi(plant_, scene_graph_, "top");
  TestMergeIncludeWithInterfaceApi(plant_, scene_graph_, "another_top");
}

TEST_F(SdfParserTest, MergeIncludeIntoWorld) {
  AddSceneGraph();
  package_map_.AddPackageXml(FindResourceOrThrow(
      "drake/multibody/parsing/test/sdf_parser_test/interface_api_test/"
      "package.xml"));
  const std::string sdf_string = R"""(
<sdf version='1.10'>
  <world name='merge_into_world'>
    <include merge="true">
      <uri>
        package://interface_api_test/top_merge_include_in_nested_model.sdf
      </uri>
    </include>
  </world>
</sdf>
)""";

  AddModelsFromSdfString(sdf_string);
  plant_.Finalize();
  SCOPED_TRACE("MergeIncludeIntoWorld");
  TestMergeIncludeWithInterfaceApi(plant_, scene_graph_, "top");
  TestMergeIncludeWithInterfaceApi(plant_, scene_graph_, "another_top");
}

TEST_F(SdfParserTest, VisualRoleConfiguration) {
  AddSceneGraph();
  // Test that point masses don't get sent through the massless body branch.
  ParseTestString(R"""(
  <model name="visual_model">
    <link name="unused_default_geometry">
      <visual name="general_visual">
        <pose>-1 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <!-- Both roles disabled for <visual> generates a warning. -->
        <drake:perception_properties enabled="false"/>
        <drake:illustration_properties enabled="false"/>
      </visual>

      <drake:visual name="non-existent">
        <drake:geometry>
          <drake:sphere>
            <drake:radius>10</drake:radius>
          </drake:sphere>
        </drake:geometry>
        <!-- Both roles diabled for <drake:visual> does *not* generate a
             warning. -->
        <drake:perception_properties enabled="false"/>
        <drake:illustration_properties enabled="false"/>
      </drake:visual>

      <drake:visual name="illustration">
        <drake:pose>1 0 0 0 0 0</drake:pose>
        <drake:geometry>
          <drake:cylinder>
            <drake:radius>0.8</drake:radius>
            <drake:length>0.02</drake:length>
          </drake:cylinder>
        </drake:geometry>
        <!-- perception disabled; illustration only. -->
        <drake:perception_properties enabled="false"/>
      </drake:visual>

      <drake:visual name="perception">
        <drake:pose>0 0 0 0 0 0</drake:pose>
        <drake:geometry>
          <drake:sphere>
            <drake:radius>2</drake:radius>
          </drake:sphere>
        </drake:geometry>
        <!-- illustration disabled; perception only. -->
        <drake:illustration_properties enabled="false"/>
      </drake:visual>

      <drake:visual name="bad_nesting">
        <!-- Non drake: child element should emit diagnostic error. -->
        <pose>0 0 0 0 0 0</pose>
      </drake:visual>

    </link>
  </model>)""");

  struct ExpectedGeometry {
    std::string name;
    std::string shape_string;
    bool operator<(const ExpectedGeometry& other) const {
      if (name < other.name) return true;
      if (name > other.name) return false;
      return shape_string < other.shape_string;
    }
  };

  const std::set<ExpectedGeometry> expected_geometries{
      {"visual_model::illustration", "Cylinder(radius=0.8, length=0.02)"},
      {"visual_model::perception", "Sphere(radius=2)"}};

  const auto& inspector = scene_graph_.model_inspector();
  EXPECT_EQ(inspector.num_geometries(), expected_geometries.size());
  for (const auto id : inspector.GetAllGeometryIds()) {
    EXPECT_TRUE(expected_geometries.contains(ExpectedGeometry{
        inspector.GetName(id), inspector.GetShape(id).to_string()}));
  }

  EXPECT_THAT(TakeWarning(),
              ::testing::HasSubstr("<visual name=\"general_visual\"> tag had "
                                   "all visual roles turned off"));

  EXPECT_THAT(TakeError(),
              ::testing::HasSubstr("under a drake-namespaced tag must likewise "
                                   "be drake-namespaced"));

  // Note: we haven't done anything explicit to test the <drake:pose>
  // conversion. Given that the other elements of the <drake:visualizer> tree
  // have processed, we assume the <drake:pose> has processed as well. The only
  // concern we'd have if the "relative-to" plumbing weren't set up correctly.
  // However, in that case, this test would fail because each of the
  // <drake:visual> elements with a <drake:pose> tag would generate an error
  // in the diagnostics:
  //    "SemanticPose has invalid pointer to PoseRelativeToGraph."
  // If those errors were emitted, this test would fail on completion.
  EXPECT_THAT(NumErrors(), 0);
}

// Happy‑path: a minimal deformable model containing a single link with a mesh
// collision element should parse and create exactly one deformable body.
TEST_F(SdfParserTest, ParseMinimalDeformableModel) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='deformable'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
    </link>
  </model>)";

  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 0);
  plant_.Finalize();
  EXPECT_EQ(plant_.deformable_model().num_bodies(), 1);
  // Without specifying any proximity properties, the default values are used.
  GeometryId g_id =
      plant_.deformable_model().GetBodyByName("body").geometry_id();
  const auto* proximity_props_ptr =
      scene_graph_.model_inspector().GetProximityProperties(g_id);
  ASSERT_NE(proximity_props_ptr, nullptr);
  EXPECT_EQ(
      proximity_props_ptr
          ->GetProperty<CoulombFriction<double>>(
              geometry::internal::kMaterialGroup, geometry::internal::kFriction)
          .dynamic_friction(),
      1.0);
}

// Happy‑path: a full-fledged deformable model containing a single link with a
// mesh collision element should parse and create exactly one deformable body.
TEST_F(SdfParserTest, ParseFullFeatureDeformableModel) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='deformable'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
        <drake:proximity_properties>
          <drake:mu_dynamic>0.5</drake:mu_dynamic>
          <drake:hunt_crossley_dissipation>0.6</drake:hunt_crossley_dissipation>
          <drake:relaxation_time>0.7</drake:relaxation_time>
        </drake:proximity_properties>
      </collision>
      <visual name='visual'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/tri_cube.obj</uri></mesh>
        </geometry>
      </visual>
      <drake:deformable_properties>
        <drake:youngs_modulus>100.0</drake:youngs_modulus>
        <drake:poissons_ratio>0.3</drake:poissons_ratio>
        <drake:mass_damping>0.01</drake:mass_damping>
        <drake:stiffness_damping>0.01</drake:stiffness_damping>
        <drake:mass_density>800.0</drake:mass_density>
        <drake:material_model>corotated</drake:material_model>
      </drake:deformable_properties>
    </link>
  </model>)";

  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 0);
  EXPECT_THAT(NumWarnings(), 0);
  plant_.Finalize();
  EXPECT_EQ(plant_.deformable_model().num_bodies(), 1);
  const DeformableBodyId body_id =
      plant_.deformable_model().GetBodyByName("body").body_id();
  auto geometry_id = plant_.deformable_model().GetGeometryId(body_id);
  const geometry::ProximityProperties* props =
      scene_graph_.model_inspector().GetProximityProperties(geometry_id);
  ASSERT_NE(props, nullptr);
  EXPECT_EQ(
      props
          ->GetProperty<CoulombFriction<double>>(
              geometry::internal::kMaterialGroup, geometry::internal::kFriction)
          .dynamic_friction(),
      0.5);
  EXPECT_EQ(props->GetProperty<double>(geometry::internal::kMaterialGroup,
                                       geometry::internal::kHcDissipation),
            0.6);
  EXPECT_EQ(props->GetProperty<double>(geometry::internal::kMaterialGroup,
                                       geometry::internal::kRelaxationTime),
            0.7);
}

TEST_F(SdfParserTest, MultipleDeformableBodies) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='deformable'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
    </link>
    <link name='body2'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
    </link>
  </model>)";

  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 0);
  plant_.Finalize();
  EXPECT_EQ(plant_.deformable_model().num_bodies(), 2);
}

// Deformable bodies can be included in collision filter groups.
TEST_F(SdfParserTest, DeformableBodiesWithFilteredCollision) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='deformable'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
    </link>
    <link name='body2'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
    </link>
    <drake:collision_filter_group name="group_a">
      <drake:member>body</drake:member>
      <drake:member>body2</drake:member>
      <drake:ignored_collision_filter_group>group_a</drake:ignored_collision_filter_group>
    </drake:collision_filter_group>
  </model>)";

  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 0);
  plant_.Finalize();
  EXPECT_EQ(plant_.deformable_model().num_bodies(), 2);

  // Test that collisions are filtered between the deformable bodies.
  const auto& inspector = scene_graph_.model_inspector();
  std::vector<GeometryId> ids = inspector.GetAllGeometryIds();
  ASSERT_EQ(ids.size(), 2);

  std::set<CollisionPair> expected_filters = {
      {"deformable::body", "deformable::body2"},
  };
  VerifyCollisionFilters(ids, expected_filters);
}

// Specifying both deformable and rigid bodies in the same model is fine.
TEST_F(SdfParserTest, MixingModelAndDeformableModel) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='mixed'>
    <link name='deformable'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
    </link>
    <link name='rigid'>
    </link>
  </model>)";
  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 0);
  EXPECT_EQ(plant_.deformable_model().num_bodies(), 1);
  // World body + the parsed rigid body.
  EXPECT_EQ(plant_.num_bodies(), 2);
}

// Parsing deformable model requires a SceneGraph. Emits a warning if
// SceneGraph is not added.
TEST_F(SdfParserTest, DeformableModelNoSceneGraph) {
  const std::string sdf = R"(
  <model name='deformable'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
    </link>
  </model>)";
  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 0);
  EXPECT_THAT(NumWarnings(), 1);
  EXPECT_THAT(TakeWarning(),
              MatchesRegex(".*deformable.*without.*geometry source.*"));
}

TEST_F(SdfParserTest, IllegalDeformablePropertiesParsing) {
  AddSceneGraph();

  constexpr const char* sdf_template = R"""(
<model name='deformable_with_illegal_{suffix}'>
  <link name='body_{suffix}'>
    <collision name='collision'>
      <geometry>
        <mesh>
          <uri>package://drake/multibody/parsing/test/single_tet.vtk</uri>
        </mesh>
      </geometry>
    </collision>
    <drake:deformable_properties>
      {snippet}
    </drake:deformable_properties>
  </link>
</model>)""";

  struct TestCase {
    std::string snippet;  // the single bad-property XML
    std::string suffix;   // used in the model name
    std::string regex;    // expected error regex
  };

  const std::vector<TestCase> cases = {
      {"<drake:youngs_modulus>0.0</drake:youngs_modulus>", "youngs_modulus",
       ".*Young's modulus.*"},
      {"<drake:poissons_ratio>1.5</drake:poissons_ratio>", "poissons_ratio",
       ".*Poisson's ratio.*"},
      {"<drake:mass_damping>-1.0</drake:mass_damping>", "mass_damping",
       ".*Mass damping.*"},
      {"<drake:stiffness_damping>-1.0</drake:stiffness_damping>",
       "stiffness_damping", ".*Stiffness damping.*"},
      {"<drake:mass_density>-1.0</drake:mass_density>", "mass_density",
       ".*Mass density.*"},
      {"<drake:material_model>not_a_material_model</drake:material_model>",
       "material_model", ".*material_model.*not_a_material_model.*"},
  };

  for (auto const& c : cases) {
    SCOPED_TRACE(c.suffix);
    const std::string sdf =
        fmt::format(sdf_template, fmt::arg("snippet", c.snippet),
                    fmt::arg("suffix", c.suffix));

    ParseTestString(sdf);
    EXPECT_EQ(NumErrors(), 1);
    EXPECT_THAT(TakeError(), MatchesRegex(c.regex));
  }
}

TEST_F(SdfParserTest, IllegalProximityPropertyParsingForDeformable) {
  AddSceneGraph();

  constexpr const char* sdf_template = R"""(
<model name='deformable_with_illegal_{suffix}'>
  <link name='body_{suffix}'>
    <collision name='collision'>
      <geometry>
        <mesh>
          <uri>package://drake/multibody/parsing/test/single_tet.vtk</uri>
        </mesh>
      </geometry>
      <drake:proximity_properties>
        {snippet}
      </drake:proximity_properties>
    </collision>
    <drake:deformable_properties/>
  </link>
</model>)""";

  struct TestCase {
    std::string snippet;  // the single bad-property XML
    std::string suffix;   // used in the model name
    std::string regex;    // expected error regex
  };

  const std::vector<TestCase> cases = {
      {"<drake:mu_dynamic>-1.0</drake:mu_dynamic>", "mu_dynamic",
       ".*mu_dynamic.*"},
      {"<drake:hunt_crossley_dissipation>-0.5</"
       "drake:hunt_crossley_dissipation>",
       "hunt_crossley_dissipation", ".*hunt_crossley_dissipation.*"},
      {"<drake:relaxation_time>-1.0</drake:relaxation_time>", "relaxation_time",
       ".*relaxation_time.*"},
  };

  for (auto const& c : cases) {
    SCOPED_TRACE(c.suffix);
    const std::string sdf =
        fmt::format(sdf_template, fmt::arg("snippet", c.snippet),
                    fmt::arg("suffix", c.suffix));

    ParseTestString(sdf);
    EXPECT_EQ(NumErrors(), 1);
    EXPECT_THAT(TakeError(), MatchesRegex(c.regex));
  }
}

TEST_F(SdfParserTest, DeformableMaterialModels) {
  AddSceneGraph();

  constexpr const char* sdf_template = R"""(
<model name='deformable_with_{model}'>
  <link name='body_{model}'>
    <collision name='collision'>
      <geometry>
        <mesh>
          <uri>package://drake/multibody/parsing/test/single_tet.vtk</uri>
        </mesh>
      </geometry>
    </collision>
    <drake:deformable_properties>
      <drake:material_model>
        {model}
      </drake:material_model>
    </drake:deformable_properties>
  </link>
</model>)""";

  const std::vector<std::string> cases = {
      "corotated",
      "neohookean",
      "linear_corotated",
      "linear",
  };

  for (auto const& c : cases) {
    SCOPED_TRACE(c);
    const std::string sdf = fmt::format(sdf_template, fmt::arg("model", c));
    ParseTestString(sdf);
    EXPECT_EQ(NumErrors(), 0);
  }
  plant_.Finalize();
  EXPECT_EQ(plant_.deformable_model().num_bodies(), cases.size());
}

TEST_F(SdfParserTest, DeformableWithMoreThanOneCollision) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='deformable'>
    <link name='body'>
      <collision name='collision1'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <collision name='collision2'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
    </link>
  </model>)";

  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 1);
  EXPECT_THAT(TakeError(), MatchesRegex(".*exactly one <collision>.*"));
}

TEST_F(SdfParserTest, DeformableWithMoreThanOneVisual) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='deformable'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <visual name='visual1'>
      </visual>
      <visual name='visual2'>
      </visual>
      <drake:deformable_properties/>
    </link>
  </model>)";

  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 1);
  EXPECT_THAT(TakeError(), MatchesRegex(".*at most one <visual>.*"));
}

TEST_F(SdfParserTest, ComposedPoseForDeformable) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='deformable'>
    <pose>3 0 0 0 0 0</pose>
    <link name='body'>
      <pose>4 0 0 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
    </link>
  </model>)";
  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 0);
  plant_.Finalize();
  EXPECT_EQ(plant_.deformable_model().num_bodies(), 1);
  const DeformableBodyId body_id =
      plant_.deformable_model().GetBodyByName("body").body_id();
  const VectorXd q_WB =
      plant_.deformable_model().GetReferencePositions(body_id);
  VectorXd q_WB_expected(12);
  // clang-format off
  q_WB_expected << -3, -10, -10,
                    17, 0,  0,
                    7,  10, 0,
                    7,  0,  10;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(q_WB, q_WB_expected,
                              4.0 * std::numeric_limits<double>::epsilon()));
}

/* Specifying a non-empty visual geometry when perception properties are turned
 off is a warning. */
TEST_F(SdfParserTest, DisabledPerceptionWithVisualMesh) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='deformable'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/tri_cube.obj</uri></mesh>
        </geometry>
        <drake:perception_properties enabled="false"/>
      </visual>
      <drake:deformable_properties/>
    </link>
  </model>)";

  ParseTestString(sdf);
  EXPECT_THAT(NumWarnings(), 1);
  EXPECT_THAT(TakeWarning(), MatchesRegex(".*non-empty.*geometry.*ignored.*"));
}

TEST_F(SdfParserTest, DeformableWallBoundaryConditionsBodyOutsideHalfSpace) {
  AddSceneGraph();
  const std::string sdf_outside = R"(
  <model name='deformable'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
      <drake:wall_boundary_condition>
        <drake:point_on_plane>0.0 0.0 -10.0</drake:point_on_plane>
        <drake:outward_normal>0.0 0.0 1.0</drake:outward_normal>
      </drake:wall_boundary_condition>
    </link>
  </model>)";

  ParseTestString(sdf_outside);
  EXPECT_THAT(NumErrors(), 0);
  EXPECT_THAT(NumWarnings(), 0);
  plant_.Finalize();

  EXPECT_EQ(plant_.deformable_model().num_bodies(), 1);

  // Verify that the deformable body was created.
  DeformableBodyId body_id_outside =
      plant_.deformable_model().GetBodyByName("body").body_id();
  EXPECT_TRUE(body_id_outside.is_valid());

  // Body is outside the half space, so no boundary conditions should be added.
  const auto& fem_model_outside =
      plant_.deformable_model().GetFemModel(body_id_outside);
  const auto& dirichlet_boundary_cond_outside =
      fem_model_outside.dirichlet_boundary_condition();
  EXPECT_TRUE(
      dirichlet_boundary_cond_outside.index_to_boundary_state().empty());
}

TEST_F(SdfParserTest, DeformableWallBoundaryConditionsBodyInHalfSpace) {
  AddSceneGraph();

  // Test with body in half space region (constraints should be added).
  // Plane at (0, 0, 10) with normal (0, 0, 1) creates a half space z <= 10,
  // and the body at origin is below z = 10, so it's inside the half space.
  const std::string sdf_inside = R"(
  <model name='deformable_in_halfspace'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
      <drake:wall_boundary_condition>
        <drake:point_on_plane>0.0 0.0 10.0</drake:point_on_plane>
        <drake:outward_normal>0.0 0.0 1.0</drake:outward_normal>
      </drake:wall_boundary_condition>
    </link>
  </model>)";

  ParseTestString(sdf_inside);
  EXPECT_THAT(NumErrors(), 0);
  EXPECT_THAT(NumWarnings(), 0);
  plant_.Finalize();

  EXPECT_EQ(plant_.deformable_model().num_bodies(), 1);

  // Verify that the deformable body was created.
  const DeformableBodyId body_id_inside =
      plant_.deformable_model().GetBodyByName("body").body_id();
  EXPECT_TRUE(body_id_inside.is_valid());

  // Body is in the half space, so boundary conditions should be added.
  const auto& fem_model_inside =
      plant_.deformable_model().GetFemModel(body_id_inside);
  const auto& dirichlet_boundary_cond_inside =
      fem_model_inside.dirichlet_boundary_condition();
  EXPECT_FALSE(
      dirichlet_boundary_cond_inside.index_to_boundary_state().empty());
}

TEST_F(SdfParserTest, DeformableWallMultipleBoundaryConditions) {
  AddSceneGraph();

  const std::string sdf = R"(
  <model name='deformable_with_wall_bc'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
      <drake:wall_boundary_condition>
        <drake:point_on_plane>0.0 0.0 10.0</drake:point_on_plane>
        <drake:outward_normal>0.0 0.0 1.0</drake:outward_normal>
      </drake:wall_boundary_condition>
      <drake:wall_boundary_condition>
        <drake:point_on_plane>0.0 0.0 -10.0</drake:point_on_plane>
        <drake:outward_normal>0.0 0.0 1.0</drake:outward_normal>
      </drake:wall_boundary_condition>
    </link>
  </model>)";

  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 0);
  EXPECT_THAT(NumWarnings(), 0);
  plant_.Finalize();

  EXPECT_EQ(plant_.deformable_model().num_bodies(), 1);

  // Verify that the deformable body was created.
  const DeformableBodyId body_id =
      plant_.deformable_model().GetBodyByName("body").body_id();
  EXPECT_TRUE(body_id.is_valid());

  // Body is in the half space provided by first boundary condition,
  // so boundary conditions should be added.
  const auto& fem_model = plant_.deformable_model().GetFemModel(body_id);
  const auto& dirichlet_bc = fem_model.dirichlet_boundary_condition();
  EXPECT_FALSE(dirichlet_bc.index_to_boundary_state().empty());
}

TEST_F(SdfParserTest, DeformableWallBoundaryConditionMissingChildTag) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='deformable_bad_wall_bc'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties/>
      <drake:wall_boundary_condition>
        <drake:point_on_plane>1.0 2.0 3.0</drake:point_on_plane>
        <!-- Missing drake:outward_normal -->
      </drake:wall_boundary_condition>
    </link>
  </model>)";

  ParseTestString(sdf);
  EXPECT_THAT(NumErrors(), 2);
  EXPECT_THAT(
      TakeError(),
      MatchesRegex(".*Unable to find the <drake:outward_normal> child tag.*"));
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Outward normal vector cannot be zero.*"));
}

TEST_F(SdfParserTest, WallBoundaryConditionOnRigidLink) {
  AddSceneGraph();
  const std::string sdf = R"(
  <model name='rigid_with_wall_bc'>
    <link name='body'>
      <drake:wall_boundary_condition>
        <drake:point_on_plane>0.0 0.0 0.0</drake:point_on_plane>
        <drake:outward_normal>0.0 0.0 1.0</drake:outward_normal>
      </drake:wall_boundary_condition>
    </link>
  </model>)";

  ParseTestString(sdf);
  // Wall boundary conditions on rigid links should produce an error
  EXPECT_THAT(NumErrors(), 1);
  EXPECT_THAT(TakeError(),
              MatchesRegex(".*Unsupported SDFormat element in link: "
                           "drake:wall_boundary_condition.*"));
}

/* Requesting auto calculation of link inertia. This is a regression test
 against SDFormat's implementation. It should handle its primitive types,
 accounting for pose values, and multiple geometries so they all combine into a
 single spatial inertia. The purpose of this test is to exercise enough of the
 functionality that Drake would rely on, that we are confident its does the
 right thing. */
TEST_F(SdfParserTest, AutoInertiaForPrimitive) {
  AddSceneGraph();
  ParseTestString(R"""(
<model name='inertia_from_box'>
  <link name='body'>
    <inertial auto="true">
      <density>25</density>
    </inertial>
    <collision name="collision1">
      <pose>1 2 3 0 0 0</pose>
      <geometry>
        <box>
          <size>0.2 0.3 0.4</size>
        </box>
      </geometry>
    </collision>
    <collision name="collision2">
      <pose>10 20 30 0 0 0</pose>
      <geometry>
        <sphere>
          <radius>0.35</radius>
        </sphere>
      </geometry>
    </collision>
  </link>
</model>)""");

  const double kDensity = 25;
  // Box inertia, shifted to body origin.
  auto M_BBo_B_expected =
      SpatialInertia<double>::SolidBoxWithDensity(kDensity, /* lx= */ 0.2,
                                                  /* ly= */ 0.3, /* lz= */ 0.4)
          .Shift({-1, -2, -3});
  // Add in sphere inertia, shifted to body origin.
  M_BBo_B_expected += SpatialInertia<double>::SolidSphereWithDensity(
                          kDensity, /* radius= */ 0.35)
                          .Shift({-10, -20, -30});

  const RigidBody<double>* body =
      dynamic_cast<const RigidBody<double>*>(&plant_.GetBodyByName("body"));

  const SpatialInertia<double>& M_BBo_B = body->default_spatial_inertia();
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
                              M_BBo_B_expected.CopyToFullMatrix6(), 1e-12));
}

/* In contrast with AutoInertiaForPrimitive, this test checks the behavior for
 mesh geometries which Drake has explicitly implemented.

 In this case, we expect to compute spatial inertia from a valid mesh (a cube,
 two units on a side).

 Like with AutoInertiaForPrimitive, we're including multiple collision
 geometries with disparate poses to make sure they're combined as expected. */
TEST_F(SdfParserTest, AutoInertiaForMesh) {
  AddSceneGraph();
  ParseTestString(R"""(
<model name='inertia_from_box'>
  <link name='body'>
    <inertial auto="true">
      <density>25</density>
    </inertial>
    <collision name="collision1">
      <pose>1 2 3 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>package://drake/multibody/parsing/test/tri_cube.obj</uri>
          <scale>0.1 0.15 0.2</scale>
        </mesh>
      </geometry>
    </collision>
    <collision name="collision2">
      <pose>10 20 30 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>package://drake/multibody/parsing/test/tri_cube.obj</uri>
          <scale>0.3 0.05 0.2</scale>
        </mesh>
      </geometry>
    </collision>
  </link>
</model>)""");
  // The box measures are twice the scale factors, because tri_cube.obj has
  // length 2.
  auto make_box_inertia = [](const Vector3d& half_size, const Vector3d& p_BoC,
                             double density) {
    const Vector3d size = 2.0 * half_size;
    return SpatialInertia<double>::SolidBoxWithDensity(density, size.x(),
                                                       size.y(), size.z())
        .Shift(-p_BoC);
  };

  constexpr double kDensity = 25.0;
  SpatialInertia<double> M_BBo_B_expected =
      make_box_inertia({0.1, 0.15, 0.2}, {1, 2, 3}, kDensity);
  M_BBo_B_expected +=
      make_box_inertia({0.3, 0.05, 0.2}, {10, 20, 30}, kDensity);

  const RigidBody<double>* body =
      dynamic_cast<const RigidBody<double>*>(&plant_.GetBodyByName("body"));

  const SpatialInertia<double>& M_BBo_B = body->default_spatial_inertia();
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
                              M_BBo_B_expected.CopyToFullMatrix6(), 1e-14));
}

/* Identical to AutoInertiaForMesh, but this time we specify the mass. The total
 mass of the spatial inertia should reflect the explicitly declared mass. */
TEST_F(SdfParserTest, AutoInertiaForMeshFromMass) {
  AddSceneGraph();
  ParseTestString(R"""(
<model name='inertia_from_box'>
  <link name='body'>
    <inertial auto="true">
      <mass>25</mass>
    </inertial>
    <collision name="collision1">
      <pose>1 2 3 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>package://drake/multibody/parsing/test/tri_cube.obj</uri>
          <scale>0.1 0.15 0.2</scale>
        </mesh>
      </geometry>
    </collision>
    <collision name="collision2">
      <pose>10 20 30 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>package://drake/multibody/parsing/test/tri_cube.obj</uri>
          <scale>0.3 0.05 0.2</scale>
        </mesh>
      </geometry>
    </collision>
  </link>
</model>)""");
  // The box measures are twice the scale factors, because tri_cube.obj has
  // length 2.
  auto make_box_inertia = [](const Vector3d& half_size, const Vector3d& p_BoC) {
    const Vector3d size = 2.0 * half_size;
    const double kDensity = 1.0;
    return SpatialInertia<double>::SolidBoxWithDensity(kDensity, size.x(),
                                                       size.y(), size.z())
        .Shift(-p_BoC);
  };

  SpatialInertia<double> M_BBo_B_unit =
      make_box_inertia({0.1, 0.15, 0.2}, {1, 2, 3});
  M_BBo_B_unit += make_box_inertia({0.3, 0.05, 0.2}, {10, 20, 30});
  SpatialInertia<double> M_BBo_B_expected(
      /* mass= */ 25, M_BBo_B_unit.get_com(), M_BBo_B_unit.get_unit_inertia());

  const RigidBody<double>* body =
      dynamic_cast<const RigidBody<double>*>(&plant_.GetBodyByName("body"));

  const SpatialInertia<double>& M_BBo_B = body->default_spatial_inertia();
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
                              M_BBo_B_expected.CopyToFullMatrix6(), 1e-14));
}

TEST_F(SdfParserTest, AutoInertiaForMeshBadData) {
  AddSceneGraph();
  ParseTestString(R"""(
<model name='inertia_from_box'>
  <link name='missing_uri'>
    <inertial auto="true"/>      <!-- default density is not a problem. -->
    <collision name="collision1">
      <geometry>
        <mesh/>                   <!-- missing uri -->
      </geometry>
    </collision>
  </link>

  <link name='bad_density'>
    <inertial auto="true">
      <density>-1</density>       <!-- bad density -->
    </inertial>
    <collision name="collision1">
      <geometry>
        <mesh>
          <uri>package://drake/multibody/parsing/test/tri_cube.obj</uri>
        </mesh>
      </geometry>
    </collision>
  </link>
</model>)""");
  ASSERT_EQ(this->NumErrors(), 2);
  EXPECT_THAT(TakeError(), MatchesRegex(".*non-physical density value: -1.*"));
  // This gets dispatched in detail_sdf_geometry.cc, MakeShapeFromSdfGeometry().
  EXPECT_THAT(TakeError(), MatchesRegex(".*missing the required 'uri'.*"));
}

/* Similar to AutoInertiaForMesh, but in this case, we provide a Mesh that will
 produce an invalid spatial inertia (its faces are reversed). So, it must
 resort to its convex hull. The mesh is comprised of two faces with normals that
 point toward each other and can't produce a meaningful inertia. Instead, we'll
 use the mesh's convex hull (a cube, 2 meters on a side). We can *tell* that
 we've used the fallback because we check for the corresponding warning output
 to the diagnostic policy. */
TEST_F(SdfParserTest, AutoInertiaForMeshConvexFallback) {
  AddSceneGraph();
  ParseTestString(R"""(
<model name='inertia_from_box'>
  <link name='body'>
    <inertial auto="true">
      <density>25</density>
    </inertial>
    <collision name="collision">
      <pose>1 2 3 0 0 0</pose>
      <geometry>
        <mesh>
          <uri>package://drake/geometry/test/two_inverted_faces.obj</uri>
          <scale>0.1 0.15 0.2</scale>
        </mesh>
      </geometry>
    </collision>
  </link>
</model>)""");
  // The box measures are twice the scale factors, because tri_cube.obj has
  // length 2.
  const double lx = 0.2;
  const double ly = 0.3;
  const double lz = 0.4;
  const double density = 25;
  const auto M_BBo_B_expected =
      SpatialInertia<double>::SolidBoxWithDensity(density, lx, ly, lz)
          .Shift(-Vector3d(1, 2, 3));

  const RigidBody<double>* body =
      dynamic_cast<const RigidBody<double>*>(&plant_.GetBodyByName("body"));

  const SpatialInertia<double>& M_BBo_B = body->default_spatial_inertia();
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
                              M_BBo_B_expected.CopyToFullMatrix6(), 1e-14));
  ASSERT_EQ(this->NumWarnings(), 1);
  EXPECT_THAT(TakeWarning(),
              MatchesRegex(".*calculated volume of a triangle surface mesh.*"));
}

// Drake has introduced two custom geometry types to be parsed in sdf: ellipsoid
// and capsule. These should be removed (as per issue 14837). In the meantime,
// using those shapes and asking for auto-inertia results in a parse failure.
TEST_F(SdfParserTest, AutoInertiaForDrakeShapes) {
  AddSceneGraph();
  ParseTestString(R"""(
<model name='inertia_from_box'>
  <link name='body'>
    <inertial auto="true">
      <density>25</density>
    </inertial>
    <collision name="ellipsoid">
      <geometry>
        <drake:ellipsoid>
          <a>0.5</a>
          <b>0.6</b>
          <c>0.7</c>
        </drake:ellipsoid>
      </geometry>
    </collision>
    <collision name="capsule">
      <geometry>
        <drake:capsule>
          <radius>0.5</radius>
          <length>1.2</length>
        </drake:capsule>
      </geometry>
    </collision>
  </link>
</model>)""");

  // Nothing was loaded; we still only have the world body.
  EXPECT_EQ(plant_.num_bodies(), 1);

  // SDFormat's message about unsupported geometry for auto inertia.
  const std::string_view unsupported_geo_message =
      ".*Automatic inertia calculations are not supported for the given "
      "Geometry type.*";
  ASSERT_EQ(NumErrors(), 2);
  EXPECT_THAT(TakeError(), MatchesRegex(unsupported_geo_message));
  EXPECT_THAT(TakeError(), MatchesRegex(unsupported_geo_message));

  // Failure to compute inertia for the unknown shapes also leads to complaints
  // about missing inertia.
  const std::string_view missing_inertia_message =
      ".*Inertia Calculated for collision: (ellipsoid|capsule) is invalid.*";
  ASSERT_EQ(NumWarnings(), 2);
  EXPECT_THAT(TakeWarning(), MatchesRegex(missing_inertia_message));
  EXPECT_THAT(TakeWarning(), MatchesRegex(missing_inertia_message));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
