#include "drake/multibody/parsing/detail_dmd_parser.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;
using Eigen::Vector3d;
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
  DmdParserTest() {
    RecordErrors();
  }

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
    const ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                             &plant_,  &resolver,    TestingSelect};
    auto result =
        multibody::internal::ParseModelDirectives(directives, std::nullopt, w);
    resolver.Resolve(diagnostic_policy_);
    return result;
  }

 protected:
  ParsingOptions options_;
  PackageMap package_map_;
  DiagnosticPolicy diagnostic_;
  MultibodyPlant<double> plant_{0.01};
};

/* Make an SDF containing a "ball" body (with an attached frame) and a dummy
 body (with an attached frame). */
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
)""",
                                          fmt::arg("offset_str", offset_str),
                                          fmt::arg("sphere_sdf", sphere_sdf)),
                                      {}, ModelDirectives());

  std::vector<ModelInstanceInfo> results = ParseModelDirectives(directives);
  ASSERT_EQ(results.size(), 7);

  /* Instantiate the floating joint between ball and world and set all the
   default poses. */
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  /* We'll grab all the relevant bodies (the balls we posed and their parent
   bodies), enumerating them based on the expected model instance index. */
  auto get_body = [this, &results](
                      int expected_index,
                      std::string_view body_name =
                          "ball") -> const RigidBody<double>& {
    const ModelInstanceIndex index(expected_index);
    const int result_index = expected_index - 2;
    EXPECT_EQ(results[result_index].model_instance, index);
    return this->plant_.GetBodyByName(body_name, index);
  };
  const RigidBody<double>& ball_2 = get_body(2);
  const RigidBody<double>& ball_3 = get_body(3);
  const RigidBody<double>& ball_4 = get_body(4);
  const RigidBody<double>& ball_5 = get_body(5);
  const RigidBody<double>& ball_6 = get_body(6);
  const RigidBody<double>& ball_7 = get_body(7);
  const RigidBody<double>& ball_8 = get_body(8);
  // Ball 8 is ultimately a child of dummy 8, so we'll grab it for later.
  const RigidBody<double>& dummy_8 = get_body(8, "dummy");
  const RigidBody<double>& world = plant_.world_body();

  /* Now we'll look at each of the spheres in turn, and confirm that they are
   posed as we expect. */
  struct BallExpectations {
    const RigidBody<double>* body{};
    const RigidBody<double>* parent{};
    int num_offset{1};
    bool body_floats{false};
  };
  std::vector<BallExpectations> balls{
      {.body = &ball_2, .parent = &world, .body_floats = true},
      {.body = &ball_3, .parent = &world, .body_floats = true},
      {.body = &ball_4, .parent = &ball_3, .num_offset = 2},
      {.body = &ball_5, .parent = &ball_2, .num_offset = 3},
      /* Ball 6 is frame-to-body, this undoes the effect of an offset. */
      {.body = &ball_6, .parent = &ball_2, .num_offset = 1},
      /* Ball 7 is frame-to-frame, undoing the effect of an offset. */
      {.body = &ball_7, .parent = &ball_2, .num_offset = 2},
      {.body = &ball_8, .parent = &dummy_8, .num_offset = 2}};

  for (const auto& ball : balls) {
    SCOPED_TRACE(fmt::format(
        "Ball {}", plant_.GetModelInstanceName(ball.body->model_instance())));
    /* Because it's not floating, its default pose must come from the joint. */
    EXPECT_EQ(ball.body->is_floating(), ball.body_floats);
    if (ball.body_floats) {
      /* For floating bodies, the default pose can be reported by the plant. */
      RigidTransformd X_PC_plant_default =
          plant_.GetDefaultFreeBodyPose(*ball.body);
      EXPECT_TRUE(CompareMatrices(
          X_PC_plant_default.GetAsMatrix34(),
          RigidTransformd(ball.num_offset * p_PC).GetAsMatrix34()));
    }
    const std::string joint_name = ball.body->name();
    ASSERT_TRUE(plant_.HasJointNamed(joint_name, ball.body->model_instance()));
    const auto& joint =
        plant_.GetJointByName(joint_name, ball.body->model_instance());

    /* The default pose can *always* be checked from the joint. */
    EXPECT_TRUE(CompareMatrices(joint.GetDefaultPose().GetAsMatrix34(),
                                RigidTransformd(p_PC).GetAsMatrix34()));
    EXPECT_EQ(&joint.parent_body(), ball.parent);
    EXPECT_EQ(&joint.child_body(), ball.body);

    EXPECT_TRUE(CompareMatrices(
        ball.body->EvalPoseInWorld(*context).GetAsMatrix34(),
        RigidTransformd(ball.num_offset * p_PC).GetAsMatrix34()));
  }
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
      ".*already has a joint .* connecting 'dummy' to 'ball'.*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
