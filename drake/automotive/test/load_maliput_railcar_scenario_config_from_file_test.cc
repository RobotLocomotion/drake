#include "drake/automotive/load_maliput_railcar_scenario_config_from_file.h"

#include <string>

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"

namespace drake {
namespace automotive {
namespace {

// Tests that a scenario config file instantiating a Protocol Buffer message
// defined by maliput_railcar_scenario_config.proto can be parsed.
GTEST_TEST(LoadMaliputRailcarScenarioConfigFromFileTest, BasicTest) {
  const std::string config_file_name = drake::GetDrakePath() +
      "/automotive/test/typical.maliput_railcar_scenario_config";
  MaliputRailcarScenarioConfig scenario_config;
  EXPECT_NO_THROW(LoadMaliputRailcarScenarioConfigFromFile(config_file_name,
    &scenario_config));
  ASSERT_EQ(scenario_config.maliput_railcar_config_size(), 2);

  {
    const MaliputRailcarConfig& config =
        scenario_config.maliput_railcar_config(0);
    EXPECT_EQ(config.car_name(), "prius_prime");
    EXPECT_EQ(config.initial_lane_name(), "fast_lane");
    EXPECT_EQ(config.initial_with_s(), true);
    EXPECT_EQ(config.initial_s(), 1.2843);
    EXPECT_EQ(config.initial_speed(), 4.927);
    EXPECT_EQ(config.r(), 0.2);
    EXPECT_EQ(config.h(), 0.15);
    EXPECT_EQ(config.max_speed(), 13.5);
    EXPECT_EQ(config.velocity_limit_kp(), 20.2);
  }

  {
    const MaliputRailcarConfig& config =
        scenario_config.maliput_railcar_config(1);
    EXPECT_EQ(config.car_name(), "ls460h");
    EXPECT_EQ(config.initial_lane_name(), "luxury_lane");
    EXPECT_EQ(config.initial_with_s(), false);
    EXPECT_EQ(config.initial_s(), 2.5);
    EXPECT_EQ(config.initial_speed(), 7.8);
    EXPECT_EQ(config.r(), -0.6);
    EXPECT_EQ(config.h(), 0.2);
    EXPECT_EQ(config.max_speed(), 71.5);
    EXPECT_EQ(config.velocity_limit_kp(), 45.3);
  }

  {
    ASSERT_EQ(scenario_config.idm_controlled_maliput_railcar_config_size(), 1);
    const IdmControlledMaliputRailcarConfig& idm_railcar_config =
        scenario_config.idm_controlled_maliput_railcar_config(0);
    EXPECT_TRUE(idm_railcar_config.has_maliput_railcar_config());
    EXPECT_TRUE(idm_railcar_config.has_idm_controller_config());

    const MaliputRailcarConfig& railcar_config =
        idm_railcar_config.maliput_railcar_config();
    EXPECT_EQ(railcar_config.car_name(), "controlled_camry");
    EXPECT_EQ(railcar_config.initial_lane_name(), "comfort_lane");
    EXPECT_EQ(railcar_config.initial_with_s(), true);
    EXPECT_EQ(railcar_config.initial_s(), 6.7);
    EXPECT_EQ(railcar_config.initial_speed(), 8.3);
    EXPECT_EQ(railcar_config.r(), 0);
    EXPECT_EQ(railcar_config.h(), 0);
    EXPECT_EQ(railcar_config.max_speed(), 22.5);
    EXPECT_EQ(railcar_config.velocity_limit_kp(), 30.5);

    const IdmControllerConfig& idm_config =
        idm_railcar_config.idm_controller_config();
    EXPECT_EQ(idm_config.v_ref(), 10.1);
    EXPECT_EQ(idm_config.a(), 25.5);
    EXPECT_EQ(idm_config.b(), 30.2);
    EXPECT_EQ(idm_config.s_0(), 5.5);
    EXPECT_EQ(idm_config.time_headway(), 5.2);
    EXPECT_EQ(idm_config.delta(), 6.1);
    EXPECT_EQ(idm_config.bloat_diameter(), 7.5);
    EXPECT_EQ(idm_config.distance_lower_limit(), 8.1);
  }
}

}  // namespace
}  // namespace automotive
}  // namespace drake
