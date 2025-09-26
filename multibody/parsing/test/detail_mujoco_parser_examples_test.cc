#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/multibody/parsing/detail_mujoco_parser.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using ::testing::MatchesRegex;

class MujocoParserExamplesTest : public test::DiagnosticPolicyTestBase {
 public:
  MujocoParserExamplesTest() {
    plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
  }

  std::optional<ModelInstanceIndex> AddModelFromFile(
      const std::string& file_name, const std::string& model_name) {
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       nullptr,  &plant_,      &resolver,
                       NoSelect};
    auto result = wrapper_.AddModel({DataSource::kFilename, &file_name},
                                    model_name, {}, w);
    resolver.Resolve(diagnostic_policy_);
    return result;
  }

  // Mujoco cannot delegate to any other parsers.
  static ParserInterface& NoSelect(const drake::internal::DiagnosticPolicy&,
                                   const std::string&) {
    DRAKE_UNREACHABLE();
  }

 protected:
  ParsingOptions options_;
  PackageMap package_map_;
  MultibodyPlant<double> plant_{0.1};
  geometry::SceneGraph<double> scene_graph_;
  MujocoParserWrapper wrapper_;
};

// Given a name for a TEST_SUITE_P test case, returns a safe version of the
// string (i.e., with only alphanumeric characters). Google Test case names
// must not contain any other kinds of characters.
std::string MakeSafeTestCaseName(std::string_view name) {
  std::string result{name};
  std::replace_if(
      result.begin(), result.end(),
      [](char c) {
        return !std::isalnum(c);
      },
      '_');
  return result;
}

class DeepMindControlTest : public MujocoParserExamplesTest,
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
                         testing::ValuesIn(dm_control_models),
                         ([](const auto& test_info) {
                           // This lambda provides a nice human-readable test
                           // case name while running the test, or in case the
                           // test case fails.
                           const auto& model = test_info.param;
                           return MakeSafeTestCaseName(model);
                         }));

constexpr std::string_view kItWorks{""};
constexpr std::string_view kTooSlow =  // #22412
    "skip me";
namespace KnownErrors {
constexpr std::string_view kSizeFromMesh =  // #22372
    ".*size of the shape from the mesh.*";
constexpr std::string_view kStlMesh =  // #19408
    ".*[.][Ss][Tt][Ll].*";
}  // namespace KnownErrors

constexpr std::string_view DebugIsTooSlow(std::string_view non_debug_result) {
  if constexpr (kDrakeAssertIsArmed) {
    return kTooSlow;
  } else {
    return non_debug_result;
  }
}

class MujocoMenagerieTest : public MujocoParserExamplesTest,
                            public testing::WithParamInterface<
                                std::pair<const char*, std::string_view>> {};

TEST_P(MujocoMenagerieTest, MujocoMenagerie) {
  // Confirm successful parsing of the MuJoCo models in the DeepMind control
  // suite.
  auto [model, error_regex] = GetParam();
  if (error_regex == kTooSlow) {
    GTEST_SKIP_("Skipping this test case.");
  }
  const RlocationOrError rlocation =
      FindRunfile(fmt::format("mujoco_menagerie_internal/{}.xml", model));
  ASSERT_EQ(rlocation.error, "");
  AddModelFromFile(rlocation.abspath, model);

  EXPECT_TRUE(plant_.HasModelInstanceNamed(model));

  // For this test, ignore all warnings.
  warning_records_.clear();

  if (!error_regex.empty()) {
    EXPECT_THAT(TakeError(), MatchesRegex(error_regex));
    // For now, we'll just capture the *first* error.
    error_records_.clear();
  }
}

// TODO(russt): Some of the tests are redundant (e.g. the scene models load the
// main robot models.)
const std::pair<const char*, std::string_view> mujoco_menagerie_models[] = {
    {"aloha/aloha", KnownErrors::kStlMesh},
    {"aloha/scene", KnownErrors::kStlMesh},
    {"anybotics_anymal_b/anymal_b", kItWorks},
    {"anybotics_anymal_b/scene", kItWorks},
    {"anybotics_anymal_c/anymal_c", kItWorks},
    {"anybotics_anymal_c/anymal_c_mjx", kItWorks},
    {"anybotics_anymal_c/scene", kItWorks},
    {"anybotics_anymal_c/scene_mjx", kItWorks},
    {"berkeley_humanoid/berkeley_humanoid", KnownErrors::kStlMesh},
    {"berkeley_humanoid/scene", KnownErrors::kStlMesh},
    {"bitcraze_crazyflie_2/cf2", kItWorks},
    {"bitcraze_crazyflie_2/scene", kItWorks},
    {"boston_dynamics_spot/scene", kItWorks},
    {"boston_dynamics_spot/scene_arm", kItWorks},
    {"boston_dynamics_spot/spot", kItWorks},
    {"boston_dynamics_spot/spot_arm", kItWorks},
    {"flybody/fruitfly", DebugIsTooSlow(KnownErrors::kSizeFromMesh)},
    {"flybody/scene", DebugIsTooSlow(KnownErrors::kSizeFromMesh)},
    {"franka_emika_panda/hand", KnownErrors::kStlMesh},
    {"franka_emika_panda/mjx_panda", KnownErrors::kStlMesh},
    {"franka_emika_panda/mjx_scene", KnownErrors::kStlMesh},
    {"franka_emika_panda/mjx_single_cube", KnownErrors::kStlMesh},
    {"franka_emika_panda/panda", DebugIsTooSlow(KnownErrors::kStlMesh)},
    {"franka_emika_panda/panda_nohand", DebugIsTooSlow(KnownErrors::kStlMesh)},
    {"franka_emika_panda/scene", DebugIsTooSlow(KnownErrors::kStlMesh)},
    {"franka_fr3/fr3", DebugIsTooSlow(KnownErrors::kStlMesh)},
    {"franka_fr3/scene", DebugIsTooSlow(KnownErrors::kStlMesh)},
    {"google_barkour_v0/barkour_v0", KnownErrors::kStlMesh},
    {"google_barkour_v0/barkour_v0_mjx", KnownErrors::kStlMesh},
    {"google_barkour_v0/scene", KnownErrors::kStlMesh},
    {"google_barkour_v0/scene_barkour", KnownErrors::kStlMesh},
    {"google_barkour_v0/scene_mjx", KnownErrors::kStlMesh},
    {"google_barkour_vb/barkour_vb", KnownErrors::kStlMesh},
    {"google_barkour_vb/barkour_vb_mjx", KnownErrors::kStlMesh},
    {"google_barkour_vb/scene", KnownErrors::kStlMesh},
    {"google_barkour_vb/scene_hfield_mjx", KnownErrors::kStlMesh},
    {"google_barkour_vb/scene_mjx", KnownErrors::kStlMesh},
    {"google_robot/robot", KnownErrors::kStlMesh},
    {"google_robot/scene", KnownErrors::kStlMesh},
    {"hello_robot_stretch/scene", DebugIsTooSlow(KnownErrors::kStlMesh)},
    {"hello_robot_stretch/stretch", DebugIsTooSlow(KnownErrors::kStlMesh)},
    {"hello_robot_stretch_3/scene", DebugIsTooSlow(KnownErrors::kStlMesh)},
    {"hello_robot_stretch_3/stretch", DebugIsTooSlow(KnownErrors::kStlMesh)},
    {"kinova_gen3/gen3", KnownErrors::kStlMesh},
    {"kinova_gen3/scene", KnownErrors::kStlMesh},
    {"kuka_iiwa_14/iiwa14", kItWorks},
    {"kuka_iiwa_14/scene", kItWorks},
    {"leap_hand/left_hand", kItWorks},
    {"leap_hand/right_hand", kItWorks},
    {"leap_hand/scene_left", kItWorks},
    {"leap_hand/scene_right", kItWorks},
    {"pal_talos/scene_motor", KnownErrors::kStlMesh},
    {"pal_talos/scene_position", KnownErrors::kStlMesh},
    {"pal_talos/talos", KnownErrors::kStlMesh},
    {"pal_talos/talos_motor", KnownErrors::kStlMesh},
    {"pal_talos/talos_position", KnownErrors::kStlMesh},
    {"pal_tiago/scene_motor", KnownErrors::kStlMesh},
    {"pal_tiago/scene_position", KnownErrors::kStlMesh},
    {"pal_tiago/scene_velocity", KnownErrors::kStlMesh},
    {"pal_tiago/tiago", KnownErrors::kStlMesh},
    {"pal_tiago/tiago_motor", KnownErrors::kStlMesh},
    {"pal_tiago/tiago_position", KnownErrors::kStlMesh},
    {"pal_tiago/tiago_velocity", KnownErrors::kStlMesh},
    {"pal_tiago_dual/scene_motor", KnownErrors::kStlMesh},
    {"pal_tiago_dual/scene_position", KnownErrors::kStlMesh},
    {"pal_tiago_dual/scene_velocity", KnownErrors::kStlMesh},
    {"pal_tiago_dual/tiago_dual", KnownErrors::kStlMesh},
    {"pal_tiago_dual/tiago_dual_motor", KnownErrors::kStlMesh},
    {"pal_tiago_dual/tiago_dual_position", KnownErrors::kStlMesh},
    {"pal_tiago_dual/tiago_dual_velocity", KnownErrors::kStlMesh},
    {"realsense_d435i/d435i", KnownErrors::kSizeFromMesh},
    {"rethink_robotics_sawyer/scene", kItWorks},
    {"rethink_robotics_sawyer/sawyer", kItWorks},
    {"robotiq_2f85/2f85", KnownErrors::kStlMesh},
    {"robotiq_2f85/scene", KnownErrors::kStlMesh},
    {"robotiq_2f85_v4/2f85", KnownErrors::kStlMesh},
    {"robotiq_2f85_v4/scene", KnownErrors::kStlMesh},
    {"robotis_op3/op3", KnownErrors::kStlMesh},
    {"robotis_op3/scene", KnownErrors::kStlMesh},
    {"shadow_dexee/scene", KnownErrors::kStlMesh},
    {"shadow_dexee/shadow_dexee", KnownErrors::kStlMesh},
    {"shadow_hand/keyframes", kItWorks},
    {"shadow_hand/right_hand", kItWorks},
    {"shadow_hand/scene_right", kItWorks},
    {"skydio_x2/scene", kItWorks},
    {"skydio_x2/x2", kItWorks},
    {"trossen_vx300s/scene", KnownErrors::kStlMesh},
    {"trossen_vx300s/vx300s", KnownErrors::kStlMesh},
    {"trossen_wx250s/scene", KnownErrors::kStlMesh},
    {"trossen_wx250s/wx250s", KnownErrors::kStlMesh},
    {"trs_so_arm100/scene", KnownErrors::kStlMesh},
    {"trs_so_arm100/so_arm100", KnownErrors::kStlMesh},
    {"ufactory_lite6/lite6", KnownErrors::kStlMesh},
    {"ufactory_lite6/lite6_gripper_narrow", KnownErrors::kStlMesh},
    {"ufactory_lite6/lite6_gripper_wide", KnownErrors::kStlMesh},
    {"ufactory_lite6/scene", KnownErrors::kStlMesh},
    {"ufactory_xarm7/hand", KnownErrors::kStlMesh},
    {"ufactory_xarm7/scene", KnownErrors::kStlMesh},
    {"ufactory_xarm7/xarm7", KnownErrors::kStlMesh},
    {"ufactory_xarm7/xarm7_nohand", KnownErrors::kStlMesh},
    {"unitree_a1/a1", kItWorks},
    {"unitree_a1/scene", kItWorks},
    {"unitree_g1/g1", KnownErrors::kStlMesh},
    {"unitree_g1/g1_with_hands", KnownErrors::kStlMesh},
    {"unitree_g1/scene", KnownErrors::kStlMesh},
    {"unitree_g1/scene_with_hands", KnownErrors::kStlMesh},
    {"unitree_go1/go1", KnownErrors::kStlMesh},
    {"unitree_go1/scene", KnownErrors::kStlMesh},
    {"unitree_go2/go2", kItWorks},
    {"unitree_go2/go2_mjx", kItWorks},
    {"unitree_go2/scene", kItWorks},
    {"unitree_go2/scene_mjx", kItWorks},
    {"unitree_h1/h1", KnownErrors::kStlMesh},
    {"unitree_h1/scene", KnownErrors::kStlMesh},
    {"unitree_z1/scene", KnownErrors::kStlMesh},
    {"unitree_z1/z1", KnownErrors::kStlMesh},
    {"unitree_z1/z1_gripper", KnownErrors::kStlMesh},
    {"universal_robots_ur10e/scene", kItWorks},
    {"universal_robots_ur10e/ur10e", kItWorks},
    {"universal_robots_ur5e/scene", kItWorks},
    {"universal_robots_ur5e/ur5e", kItWorks},
    {"wonik_allegro/left_hand", KnownErrors::kStlMesh},
    {"wonik_allegro/right_hand", KnownErrors::kStlMesh},
    {"wonik_allegro/scene_left", KnownErrors::kStlMesh},
    {"wonik_allegro/scene_right", KnownErrors::kStlMesh},
};

INSTANTIATE_TEST_SUITE_P(MujocoMenagerie, MujocoMenagerieTest,
                         testing::ValuesIn(mujoco_menagerie_models),
                         ([](const auto& test_info) {
                           // This lambda provides a nice human-readable test
                           // case name while running the test, or in case the
                           // test case fails.
                           const auto& [model, error_regex] = test_info.param;
                           return MakeSafeTestCaseName(model);
                         }));

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
