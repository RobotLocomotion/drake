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

  // Mujoco cannot delegate to any other parsers.
  static ParserInterface& NoSelect(
      const drake::internal::DiagnosticPolicy&, const std::string&) {
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
constexpr std::string_view kSkipMe{"skip me"};
namespace KnownErrors {
constexpr std::string_view kNonUniformScale{".*non-uniform scale.*"};  // #22046
constexpr std::string_view kMoreThanOneOrientation{
    ".*more than one orientation.*"};
constexpr std::string_view kCapsuleSize{".*size attribute for capsule geom.*"};
}  // namespace KnownErrors

class MujocoMenagerieTest : public MujocoParserExamplesTest,
                            public testing::WithParamInterface<
                                std::pair<const char*, std::string_view>> {};

TEST_P(MujocoMenagerieTest, MujocoMenagerie) {
  // Confirm successful parsing of the MuJoCo models in the DeepMind control
  // suite.
  auto [model, error_regex] = GetParam();
  if (error_regex == kSkipMe) {
    GTEST_SKIP_("Skipping this test case.");
  }
  const RlocationOrError rlocation = FindRunfile(
      fmt::format("mujoco_menagerie_internal/{}.xml", model));
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

// TODO(russt): Add logic to check for warnings, too. Some are
// acceptable/expected, but warnings like the stl2obj message make the model
// unusable.

const std::pair<const char*, std::string_view> mujoco_menagerie_models[] = {
    {"agility_cassie/cassie", KnownErrors::kNonUniformScale},
    {"agility_cassie/scene", KnownErrors::kNonUniformScale},
    {"aloha/aloha", kItWorks},
    {"aloha/scene", kItWorks},
    {"anybotics_anymal_b/anymal_b", KnownErrors::kMoreThanOneOrientation},
    {"anybotics_anymal_b/scene", KnownErrors::kMoreThanOneOrientation},
    {"anybotics_anymal_c/anymal_c", kItWorks},
    {"anybotics_anymal_c/anymal_c_mjx", kItWorks},
    {"anybotics_anymal_c/scene", kItWorks},
    {"anybotics_anymal_c/scene_mjx", kItWorks},
    {"berkeley_humanoid/berkeley_humanoid", kItWorks},
    {"berkeley_humanoid/scene", kItWorks},
    {"bitcraze_crazyflie_2/cf2", kItWorks},
    {"bitcraze_crazyflie_2/scene", kItWorks},
    {"boston_dynamics_spot/scene", kItWorks},
    {"boston_dynamics_spot/scene_arm", kItWorks},
    {"boston_dynamics_spot/spot", kItWorks},
    {"boston_dynamics_spot/spot_arm", kItWorks},
    {"flybody/fruitfly", kSkipMe},  // works, but too slow in debug mode.
    {"flybody/scene", kSkipMe},     // works, but too slow in debug mode.
    {"franka_emika_panda/hand", kItWorks},
    {"franka_emika_panda/mjx_panda", kItWorks},
    {"franka_emika_panda/mjx_scene", kItWorks},
    {"franka_emika_panda/mjx_single_cube", kItWorks},
    {"franka_emika_panda/panda",
     kSkipMe},  // works, but too slow in debug mode.
    {"franka_emika_panda/panda_nohand",
     kSkipMe},  // works, but too slow in debug mode.
    {"franka_emika_panda/scene",
     kSkipMe},                      // works, but too slow in debug mode.
    {"franka_fr3/fr3", kSkipMe},    // works, but too slow in debug mode.
    {"franka_fr3/scene", kSkipMe},  // works, but too slow in debug mode.
    {"google_barkour_v0/barkour_v0", kItWorks},
    {"google_barkour_v0/barkour_v0_mjx", kItWorks},
    {"google_barkour_v0/scene", kItWorks},
    {"google_barkour_v0/scene_barkour", kItWorks},
    {"google_barkour_v0/scene_mjx", kItWorks},
    {"google_barkour_vb/barkour_vb", kItWorks},
    {"google_barkour_vb/barkour_vb_mjx", kItWorks},
    {"google_barkour_vb/scene", kItWorks},
    {"google_barkour_vb/scene_hfield_mjx", kItWorks},
    {"google_barkour_vb/scene_mjx", kItWorks},
    {"google_robot/robot", kItWorks},
    {"google_robot/scene", kItWorks},
    /* The hello_robot_stretch and hello_robot_stretch_3 models currently throw
       in RotationalInertia<T>::ThrowNotPhysicallyValid(), but only in Debug
       mode. This is possibly due to the fact that the stl geometries are not
       being parsed, so the proper inertias are not being computed. They _also_
       fail with KnownErrors::kNonUniformScale in release mode. */
    {"hello_robot_stretch/scene", kSkipMe},
    {"hello_robot_stretch/stretch", kSkipMe},
    {"hello_robot_stretch_3/scene", kSkipMe},
    {"hello_robot_stretch_3/stretch", kSkipMe},
    {"kinova_gen3/gen3", kItWorks},
    {"kinova_gen3/scene", kItWorks},
    {"kuka_iiwa_14/iiwa14", kItWorks},
    {"kuka_iiwa_14/scene", kItWorks},
    {"leap_hand/left_hand", kItWorks},
    {"leap_hand/right_hand", kItWorks},
    {"leap_hand/scene_left", kItWorks},
    {"leap_hand/scene_right", kItWorks},
    {"pal_talos/scene_motor", KnownErrors::kNonUniformScale},
    {"pal_talos/scene_position", KnownErrors::kNonUniformScale},
    {"pal_talos/talos", KnownErrors::kNonUniformScale},
    {"pal_talos/talos_motor", KnownErrors::kNonUniformScale},
    {"pal_talos/talos_position", KnownErrors::kNonUniformScale},
    {"pal_tiago/scene_motor", KnownErrors::kNonUniformScale},
    {"pal_tiago/scene_position", KnownErrors::kNonUniformScale},
    {"pal_tiago/scene_velocity", KnownErrors::kNonUniformScale},
    {"pal_tiago/tiago", KnownErrors::kNonUniformScale},
    {"pal_tiago/tiago_motor", KnownErrors::kNonUniformScale},
    {"pal_tiago/tiago_position", KnownErrors::kNonUniformScale},
    {"pal_tiago/tiago_velocity", KnownErrors::kNonUniformScale},
    {"pal_tiago_dual/scene_motor", KnownErrors::kNonUniformScale},
    {"pal_tiago_dual/scene_position", KnownErrors::kNonUniformScale},
    {"pal_tiago_dual/scene_velocity", KnownErrors::kNonUniformScale},
    {"pal_tiago_dual/tiago_dual", KnownErrors::kNonUniformScale},
    {"pal_tiago_dual/tiago_dual_motor", KnownErrors::kNonUniformScale},
    {"pal_tiago_dual/tiago_dual_position", KnownErrors::kNonUniformScale},
    {"pal_tiago_dual/tiago_dual_velocity", KnownErrors::kNonUniformScale},
    {"realsense_d435i/d435i", KnownErrors::kCapsuleSize},
    {"rethink_robotics_sawyer/scene", kItWorks},
    {"rethink_robotics_sawyer/sawyer", kItWorks},
    {"robotiq_2f85/2f85", kItWorks},
    {"robotiq_2f85/scene", kItWorks},
    {"robotiq_2f85_v4/2f85", kItWorks},
    {"robotiq_2f85_v4/scene", kItWorks},
    {"robotis_op3/op3", kItWorks},
    {"robotis_op3/scene", kItWorks},
    {"shadow_dexee/scene", kItWorks},
    {"shadow_dexee/shadow_dexee", kItWorks},
    {"shadow_hand/keyframes", kItWorks},
    {"shadow_hand/left_hand", KnownErrors::kNonUniformScale},
    {"shadow_hand/right_hand", kItWorks},
    {"shadow_hand/scene_left", KnownErrors::kNonUniformScale},
    {"shadow_hand/scene_right", kItWorks},
    {"skydio_x2/scene", kItWorks},
    {"skydio_x2/x2", kItWorks},
    {"trossen_vx300s/scene", kItWorks},
    {"trossen_vx300s/vx300s", kItWorks},
    {"trossen_wx250s/scene", kItWorks},
    {"trossen_wx250s/wx250s", kItWorks},
    {"trs_so_arm100/scene", kItWorks},
    {"trs_so_arm100/so_arm100", kItWorks},
    {"ufactory_lite6/lite6", kItWorks},
    {"ufactory_lite6/lite6_gripper_narrow", kItWorks},
    {"ufactory_lite6/lite6_gripper_wide", kItWorks},
    {"ufactory_lite6/scene", kItWorks},
    {"ufactory_xarm7/hand", kItWorks},
    {"ufactory_xarm7/scene", kItWorks},
    {"ufactory_xarm7/xarm7", kItWorks},
    {"ufactory_xarm7/xarm7_nohand", kItWorks},
    {"unitree_a1/a1", kItWorks},
    {"unitree_a1/scene", kItWorks},
    {"unitree_g1/g1", kItWorks},
    {"unitree_g1/g1_with_hands", kItWorks},
    {"unitree_g1/scene", kItWorks},
    {"unitree_g1/scene_with_hands", kItWorks},
    {"unitree_go1/go1", kItWorks},
    {"unitree_go1/scene", kItWorks},
    {"unitree_go2/go2", kItWorks},
    {"unitree_go2/go2_mjx", kItWorks},
    {"unitree_go2/scene", kItWorks},
    {"unitree_go2/scene_mjx", kItWorks},
    {"unitree_h1/h1", kItWorks},
    {"unitree_h1/scene", kItWorks},
    {"unitree_z1/scene", kItWorks},
    {"unitree_z1/z1", kItWorks},
    {"unitree_z1/z1_gripper", kItWorks},
    {"universal_robots_ur10e/scene", kItWorks},
    {"universal_robots_ur10e/ur10e", kItWorks},
    {"universal_robots_ur5e/scene", kItWorks},
    {"universal_robots_ur5e/ur5e", kItWorks},
    {"wonik_allegro/left_hand", kItWorks},
    {"wonik_allegro/right_hand", kItWorks},
    {"wonik_allegro/scene_left", kItWorks},
    {"wonik_allegro/scene_right", kItWorks},
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
