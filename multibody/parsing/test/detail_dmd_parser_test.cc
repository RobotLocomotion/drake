#include "drake/multibody/parsing/detail_dmd_parser.h"

#include <string>
#include <tuple>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;
using Eigen::Vector3d;
using geometry::SceneGraph;
using math::RigidTransformd;
using parsing::ModelDirectives;
using parsing::ModelInstanceInfo;
using schema::Transform;
using yaml::LoadYamlString;

/* TODO(SeanCurtis-TRI): This file was instantiated with the addition of a new
feature. However, we're missing test coverage of the old features. For example,
confirming that a single model could be added multiple times and posed each
time using parse-workspace-dependent namespace fails. We need to flesh out tests
for the rest of the implementations in detail_dmd_parser.h */

class DmdParserTest : public test::DiagnosticPolicyTestBase {
 public:
  DmdParserTest() { RecordErrors(); }

  static ParserInterface& TestingSelect(const DiagnosticPolicy&,
                                        const std::string& filename) {
    // The test currently only uses cases with .sdf files.
    static never_destroyed<SdfParserWrapper> sdf;
    if (EndsWithCaseInsensitive(filename, ".sdf")) {
      return sdf.access();
    }
    throw std::runtime_error("This test should only be using sdf files.");
  }

  std::vector<ModelInstanceInfo> ParseModelDirectives(
      const ModelDirectives& directives) {
    internal::CollisionFilterGroupResolver resolver{&plant_};
    const ParsingWorkspace w{options_,     package_map_, diagnostic_policy_,
                             nullptr,      &plant_,      &resolver,
                             TestingSelect};
    auto result =
        multibody::internal::ParseModelDirectives(directives, std::nullopt, w);
    resolver.Resolve(diagnostic_policy_);
    return result;
  }

  void AddSceneGraph() { plant_.RegisterAsSourceForSceneGraph(&scene_graph_); }

 protected:
  ParsingOptions options_;
  PackageMap package_map_;
  MultibodyPlant<double> plant_{0.01};
  SceneGraph<double> scene_graph_;
};

/* Make an SDFormat file containing a "ball" body (with an attached frame) and
 a dummy body (with an attached frame). */
std::filesystem::path MakeSphereSdf(const Vector3d& p_BF) {
  std::filesystem::path dir = temp_directory();
  std::filesystem::path sdf_path = dir / "sphere.sdf";
  std::ofstream f(sdf_path);
  DRAKE_DEMAND(f.is_open());
  f << fmt::format(R"""(
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="sphere">
    <link name="ball">
      <inertial>
        <mass>0.03</mass>
        <inertia>
          <ixx>1.0e-5</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1.0e-5</iyy>
          <iyz>0</iyz>
          <izz>1.0e-5</izz>
        </inertia>
      </inertial>
    </link>
    <frame name="base_offset" attached_to="ball">
      <pose>{x} {y} {z} 0 0 0</pose>
    </frame>
    <link name="dummy"/>
    <frame name="dummy_offset" attached_to="dummy">
      <pose>{x} {y} {z} 0 0 0</pose>
    </frame>
  </model>
</sdf>
  )""",
                   fmt::arg("x", p_BF.x()), fmt::arg("y", p_BF.y()),
                   fmt::arg("z", p_BF.z()));
  return sdf_path;
}

/* Make an SDFormat file containing a deformable body */
std::filesystem::path MakeDeformableSdf() {
  std::filesystem::path dir = temp_directory();
  std::filesystem::path sdf_path = dir / "deformable.sdf";
  std::ofstream f(sdf_path);
  DRAKE_DEMAND(f.is_open());
  f << R"""(
<?xml version="1.0"?>
<sdf version="1.7">
  <model name='deformable'>
    <link name='body'>
      <collision name='collision'>
        <geometry>
          <mesh><uri>package://drake/multibody/parsing/test/single_tet.vtk</uri></mesh>
        </geometry>
      </collision>
      <drake:deformable_properties></drake:deformable_properties>
    </link>
  </model>
</sdf>
  )""";
  return sdf_path;
}

/* This test explores the dmd parser's logic for handling specifying a pose when
adding a model. To that end, we'll configure a scene with various models and
frames, exercising the options for posing free bodies.

We'll test both the basic plumbing (by confirming the values declared are echoed
back by the MbP API) and well as the final effect (by evaluating world poses of
free bodies from the default context).

One piece of logic is conspicuously absent from this test. When specifying the
pose of a free body relative to a non-world frame, the joint injected is named
for the body. If that model instance already has a joint with that name, the
name gets pre-fixed '_' to make it unique. It is *highly* unlikely that an
input model would have a body named "foo" *and* a joint named "foo" and *not*
have that joint already driving that body. So, we don't worry about testing
for now. */
TEST_F(DmdParserTest, DefaultFreeBodyPose) {
  /* This uses the same transform (an offset of [1, 2, 3] in all cases. This
   simplifies predicting the expected outcome by counting how many such
   transforms lie between the body in question and the world and simply
   multiplying that displacement vector by that count.

   The sphere used below has a frame attached to it at the offset. When we
   define the pose of the body *with that offset*, it effectively moves the
   sphere in the opposite direction, cancelling out an offset instead of
   adding to it (see pose_frame_to_body_6 and pose_frame_to_frame_7). */
  const Vector3d p_PC(1, 2, 3);
  const char offset_str[] = "[1.0, 2.0, 3.0]";

  const std::string sphere_sdf = "file://" + MakeSphereSdf(p_PC).string();
  /* The models are named suggesting what is being tested and includes the
   expected model instance index for reference below. Model instance indices
   0 & 1 are reserved, so the added models start at 2 and count up. */
  const ModelDirectives directives =
      LoadYamlString<ModelDirectives>(fmt::format(
                                          R"""(
directives:
- add_model:
    name: pose_body_to_world_implicitly_2
    file: {sphere_sdf}
    default_free_body_pose:
        ball:
            translation: {offset_str}
- add_model:
    name: pose_body_to_world_explicitly_3
    file: {sphere_sdf}
    default_free_body_pose:
        ball:
            base_frame: world
            translation: {offset_str}
- add_model:
    name: pose_body_to_body_4
    file: {sphere_sdf}
    default_free_body_pose:
        ball:
            base_frame: pose_body_to_world_explicitly_3::ball
            translation: {offset_str}
- add_model:
    name: pose_body_to_frame_5
    file: {sphere_sdf}
    default_free_body_pose:
        ball:
            base_frame: pose_body_to_world_implicitly_2::base_offset
            translation: {offset_str}
- add_model:
    name: pose_frame_to_body_6
    file: {sphere_sdf}
    default_free_body_pose:
        base_offset:
            base_frame: pose_body_to_world_implicitly_2::ball
            translation: {offset_str}
- add_model:
    name: pose_frame_to_frame_7
    file: {sphere_sdf}
    default_free_body_pose:
        base_offset:
            base_frame: pose_body_to_world_implicitly_2::base_offset
            translation: {offset_str}
- add_model:
    name: pose_body_to_in_scope_frame_8
    file: {sphere_sdf}
    default_free_body_pose:
        ball:
            base_frame: pose_body_to_in_scope_frame_8::dummy_offset
            translation: {offset_str}
- add_model:
    name: pose_frame_to_world_9
    file: {sphere_sdf}
    default_free_body_pose:
        base_offset:
            translation: {offset_str}
- add_model:
    name: pose_canonical_body_to_world_10
    file: package://drake/multibody/parsing/test/process_model_directives_test/simple_model.sdf
    default_free_body_pose:
        "":
            translation: {offset_str}
)""",
                                          fmt::arg("offset_str", offset_str),
                                          fmt::arg("sphere_sdf", sphere_sdf)),
                                      {}, ModelDirectives());

  ParseModelDirectives(directives);
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  /* To check the correctness of the parse, we'll check which pair of frames
   every joint connects, and the world pose of every frame. */

  // List of (joint, parent_frame, child_frame) tuples, in joint index order.
  using JointDetail = std::tuple<std::string, std::string, std::string>;
  std::vector<JointDetail> joints_actual;
  for (JointIndex index : plant_.GetJointIndices()) {
    const Joint<double>& joint = plant_.get_joint(index);
    joints_actual.emplace_back(
        ScopedName(plant_.GetModelInstanceName(joint.model_instance()),
                   joint.name())
            .to_string(),
        joint.frame_on_parent().scoped_name().to_string(),
        joint.frame_on_child().scoped_name().to_string());
  }
  // clang-format off
  std::vector<JointDetail> joints_expected{
      // The default_free_body_pose for #4 added a joint.
      {"pose_body_to_body_4::ball",
       "pose_body_to_world_explicitly_3::ball",
       "pose_body_to_body_4::ball"},
      // The default_free_body_pose for #5 added a joint.
      {"pose_body_to_frame_5::ball",
       "pose_body_to_world_implicitly_2::base_offset",
       "pose_body_to_frame_5::ball"},
      // The default_free_body_pose for #6 added a joint.
      {"pose_frame_to_body_6::ball",
       "pose_body_to_world_implicitly_2::ball",
       "pose_frame_to_body_6::base_offset"},
      // The default_free_body_pose for #7 added a joint.
      {"pose_frame_to_frame_7::ball",
       "pose_body_to_world_implicitly_2::base_offset",
       "pose_frame_to_frame_7::base_offset"},
      // The default_free_body_pose for #8 added a joint.
      {"pose_body_to_in_scope_frame_8::ball",
       "pose_body_to_in_scope_frame_8::dummy_offset",
       "pose_body_to_in_scope_frame_8::ball"},
      // The default_free_body_pose for #9 added a joint.
      {"pose_frame_to_world_9::ball",
       "WorldModelInstance::world",
       "pose_frame_to_world_9::base_offset"},
      // All other joints were added during Finalize. The exact order here is
      // not important, rather it's just hard-coded for simplicity. If Finalize
      // ever changes its mind about how the below entries get ordered, it's
      // fine to adjust this list.
      {"pose_body_to_world_implicitly_2::ball", "WorldModelInstance::world",
       "pose_body_to_world_implicitly_2::ball"},
      {"pose_body_to_world_explicitly_3::ball", "WorldModelInstance::world",
       "pose_body_to_world_explicitly_3::ball"},
      {"pose_body_to_in_scope_frame_8::dummy", "WorldModelInstance::world",
       "pose_body_to_in_scope_frame_8::dummy"},
      {"pose_body_to_world_implicitly_2::dummy", "WorldModelInstance::world",
       "pose_body_to_world_implicitly_2::dummy"},
      {"pose_body_to_world_explicitly_3::dummy", "WorldModelInstance::world",
       "pose_body_to_world_explicitly_3::dummy"},
      {"pose_body_to_body_4::dummy", "WorldModelInstance::world",
       "pose_body_to_body_4::dummy"},
      {"pose_body_to_frame_5::dummy", "WorldModelInstance::world",
       "pose_body_to_frame_5::dummy"},
      {"pose_frame_to_body_6::dummy", "WorldModelInstance::world",
       "pose_frame_to_body_6::dummy"},
      {"pose_frame_to_frame_7::dummy", "WorldModelInstance::world",
       "pose_frame_to_frame_7::dummy"},
      {"pose_frame_to_world_9::dummy", "WorldModelInstance::world",
       "pose_frame_to_world_9::dummy"},
      {"pose_canonical_body_to_world_10::base", "WorldModelInstance::world",
       "pose_canonical_body_to_world_10::base"},
  };
  // clang-format on
  EXPECT_THAT(joints_actual, testing::ContainerEq(joints_expected));

  // List of (frame, X_WF) in frame index order.
  using FrameDetail = std::tuple<std::string, Eigen::RowVector3d>;
  std::vector<FrameDetail> poses_actual;
  for (FrameIndex i{0}; i < plant_.num_frames(); ++i) {
    const Frame<double>& frame = plant_.get_frame(i);
    poses_actual.emplace_back(frame.scoped_name().to_string(),
                              frame.CalcPoseInWorld(*context).translation());
  }
  // clang-format off
  const std::vector<FrameDetail> poses_expected{
      {"WorldModelInstance::world",                     p_PC * 0},
      {"pose_body_to_world_implicitly_2::ball",         p_PC * 1},
      {"pose_body_to_world_implicitly_2::dummy",        p_PC * 0},
      {"pose_body_to_world_implicitly_2::__model__",    p_PC * 1},
      {"pose_body_to_world_implicitly_2::base_offset",  p_PC * 2},
      {"pose_body_to_world_implicitly_2::dummy_offset", p_PC * 1},
      {"pose_body_to_world_explicitly_3::ball",         p_PC * 1},
      {"pose_body_to_world_explicitly_3::dummy",        p_PC * 0},
      {"pose_body_to_world_explicitly_3::__model__",    p_PC * 1},
      {"pose_body_to_world_explicitly_3::base_offset",  p_PC * 2},
      {"pose_body_to_world_explicitly_3::dummy_offset", p_PC * 1},
      {"pose_body_to_body_4::ball",                     p_PC * 2},
      {"pose_body_to_body_4::dummy",                    p_PC * 0},
      {"pose_body_to_body_4::__model__",                p_PC * 2},
      {"pose_body_to_body_4::base_offset",              p_PC * 3},
      {"pose_body_to_body_4::dummy_offset",             p_PC * 1},
      {"pose_body_to_frame_5::ball",                    p_PC * 3},
      {"pose_body_to_frame_5::dummy",                   p_PC * 0},
      {"pose_body_to_frame_5::__model__",               p_PC * 3},
      {"pose_body_to_frame_5::base_offset",             p_PC * 4},
      {"pose_body_to_frame_5::dummy_offset",            p_PC * 1},
      {"pose_frame_to_body_6::ball",                    p_PC * 1},
      {"pose_frame_to_body_6::dummy",                   p_PC * 0},
      {"pose_frame_to_body_6::__model__",               p_PC * 1},
      {"pose_frame_to_body_6::base_offset",             p_PC * 2},
      {"pose_frame_to_body_6::dummy_offset",            p_PC * 1},
      {"pose_frame_to_frame_7::ball",                   p_PC * 2},
      {"pose_frame_to_frame_7::dummy",                  p_PC * 0},
      {"pose_frame_to_frame_7::__model__",              p_PC * 2},
      {"pose_frame_to_frame_7::base_offset",            p_PC * 3},
      {"pose_frame_to_frame_7::dummy_offset",           p_PC * 1},
      {"pose_body_to_in_scope_frame_8::ball",           p_PC * 2},
      {"pose_body_to_in_scope_frame_8::dummy",          p_PC * 0},
      {"pose_body_to_in_scope_frame_8::__model__",      p_PC * 2},
      {"pose_body_to_in_scope_frame_8::base_offset",    p_PC * 3},
      {"pose_body_to_in_scope_frame_8::dummy_offset",   p_PC * 1},
      {"pose_frame_to_world_9::ball",                   p_PC * 0},
      {"pose_frame_to_world_9::dummy",                  p_PC * 0},
      {"pose_frame_to_world_9::__model__",              p_PC * 0},
      {"pose_frame_to_world_9::base_offset",            p_PC * 1},
      {"pose_frame_to_world_9::dummy_offset",           p_PC * 1},
      {"pose_canonical_body_to_world_10::base",         p_PC * 1},
      {"pose_canonical_body_to_world_10::__model__",    p_PC * 1},
      {"pose_canonical_body_to_world_10::frame",        p_PC * 2},
  };
  // clang-format on
  EXPECT_THAT(poses_actual, testing::ContainerEq(poses_expected));
}

/* When a joint already exists between two bodies, adding a "default free body
pose" between them should throw. */
TEST_F(DmdParserTest, DefaultFreeBodyPoseAddsDuplicateJoints) {
  /* This is a stripped-down version of DefaultFreeBodyPose, giving us just
  enough model to attempt to introduce a redundant joint between bodies. */
  const Vector3d p_PC(1, 2, 3);
  const char offset_str[] = "[1.0, 2.0, 3.0]";

  const std::string sphere_sdf = "file://" + MakeSphereSdf(p_PC).string();
  const ModelDirectives directives =
      LoadYamlString<ModelDirectives>(fmt::format(
                                          R"""(
directives:
- add_model:
    name: dummy
    file: {sphere_sdf}
    default_free_body_pose:
        ball:
            base_frame: dummy::dummy
            translation: {offset_str}
        dummy:
            base_frame: dummy::ball
            translation: {offset_str}
)""",
                                          fmt::arg("offset_str", offset_str),
                                          fmt::arg("sphere_sdf", sphere_sdf)),
                                      {}, ModelDirectives());

  DRAKE_EXPECT_THROWS_MESSAGE(
      ParseModelDirectives(directives),
      ".*already has.*joint.*connecting.*dummy.*ball.*");
}

/* When a model contains multiple bodies, it is an error not to specify which
body to posture. */
TEST_F(DmdParserTest, DefaultFreeBodyPoseMultipleBodies) {
  const std::string sphere_sdf =
      "file://" + MakeSphereSdf(Vector3d::Zero()).string();
  const ModelDirectives directives =
      LoadYamlString<ModelDirectives>(fmt::format(
                                          R"""(
directives:
- add_model:
    name: two_bodies
    file: {sphere_sdf}
    default_free_body_pose:
      "":
        translation: [1, 2, 3]
)""",
                                          fmt::arg("sphere_sdf", sphere_sdf)),
                                      {}, ModelDirectives());

  ParseModelDirectives(directives);
  EXPECT_THAT(TakeError(),
              testing::MatchesRegex(
                  ".*two_bodies.*default_free_body_pose.*2.*bodies.*"));
}

/* Test adding a rigid body and a deformable body into the same scene. */
TEST_F(DmdParserTest, AddDeformableModel) {
  AddSceneGraph();
  const std::string sphere_sdf =
      "file://" + MakeSphereSdf(Vector3d::Zero()).string();
  const std::string deformable_sdf = "file://" + MakeDeformableSdf().string();
  const ModelDirectives directives = LoadYamlString<ModelDirectives>(
      fmt::format(
          R"""(
directives:
- add_model:
    name: rigid
    file: {sphere_sdf}
- add_model:
    name: deformable
    file: {deformable_sdf}
)""",
          fmt::arg("sphere_sdf", sphere_sdf),
          fmt::arg("deformable_sdf", deformable_sdf)),
      {}, ModelDirectives());

  ParseModelDirectives(directives);
  plant_.Finalize();
  // Two added rigid bodies plus the world body.
  EXPECT_EQ(plant_.num_bodies(), 3);
  EXPECT_EQ(plant_.deformable_model().num_bodies(), 1);
}

/* default_free_body_pose can be used to pose a deformable body in the world
 frame. */
TEST_F(DmdParserTest, FreeBodyPoseDeformableWorldFrame) {
  AddSceneGraph();
  const std::string deformable_sdf = "file://" + MakeDeformableSdf().string();
  const ModelDirectives directives = LoadYamlString<ModelDirectives>(
      fmt::format(
          R"""(
directives:
- add_model:
    name: deformable
    file: {deformable_sdf}
    default_free_body_pose:
        body:
            translation: [1, 2, 3]
)""",
          fmt::arg("deformable_sdf", deformable_sdf)),
      {}, ModelDirectives());
  ParseModelDirectives(directives);
  plant_.Finalize();
  EXPECT_EQ(plant_.deformable_model().num_bodies(), 1);
  EXPECT_TRUE(plant_.deformable_model()
                  .GetBodyByName("body")
                  .get_default_pose()
                  .IsExactlyEqualTo(RigidTransformd(Vector3d(1, 2, 3))));
}

/* default_free_body_pose should not be used to pose a deformable body in a
 non-world frame. */
TEST_F(DmdParserTest, FreeBodyPoseDeformableNonWorldFrame) {
  AddSceneGraph();
  const std::string sphere_sdf =
      "file://" + MakeSphereSdf(Vector3d::Zero()).string();
  const std::string deformable_sdf = "file://" + MakeDeformableSdf().string();
  const ModelDirectives directives = LoadYamlString<ModelDirectives>(
      fmt::format(
          R"""(
directives:
- add_model:
       name: rigid
       file: {sphere_sdf}
- add_model:
    name: deformable
    file: {deformable_sdf}
    default_free_body_pose:
        body:
            base_frame: rigid::ball
            translation: [1, 2, 3]
)""",
          fmt::arg("sphere_sdf", sphere_sdf),
          fmt::arg("deformable_sdf", deformable_sdf)),
      {}, ModelDirectives());

  /* Deformable bodies can only have their poses set relative to the world
   frame. */
  ParseModelDirectives(directives);
  EXPECT_THAT(
      TakeError(),
      testing::MatchesRegex(
          ".*Default pose for deformable body 'body' can only be specified "
          "relative to the world frame.*"));
}

/* default_joint_positions should not be used to pose a deformable body. */
TEST_F(DmdParserTest, DefaultJointPositions) {
  AddSceneGraph();
  const std::string deformable_sdf = "file://" + MakeDeformableSdf().string();
  const ModelDirectives directives = LoadYamlString<ModelDirectives>(
      fmt::format(
          R"""(
directives:
- add_model:
    name: deformable
    file: {deformable_sdf}
    default_joint_positions:
      wrong: [0.0]
)""",
          fmt::arg("deformable_sdf", deformable_sdf)),
      {}, ModelDirectives());
  /* Deformables aren't associated with any joint */
  DRAKE_EXPECT_THROWS_MESSAGE(ParseModelDirectives(directives), ".*no Joint.*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
