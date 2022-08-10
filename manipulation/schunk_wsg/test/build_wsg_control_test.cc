#include "sim/common/build_wsg_control.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::math::RigidTransformd;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

namespace anzu {
namespace sim {
namespace {

constexpr double kTolerance = 1e-3;

class BuildWsgControlTest : public ::testing::Test {
 public:
  BuildWsgControlTest() = default;

 protected:
  void SetUp() {
    sim_plant_ = builder_.AddSystem<MultibodyPlant<double>>(0.001);
    Parser parser{sim_plant_};
    const std::string wsg_file = drake::FindResourceOrThrow(
        "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");
    wsg_instance_ = parser.AddModelFromFile(wsg_file, "wsg_instance");

    // Weld the gripper to the world frame.
    sim_plant_->WeldFrames(sim_plant_->world_frame(),
                           sim_plant_->GetFrameByName("body", wsg_instance_),
                           RigidTransformd::Identity());
    sim_plant_->Finalize();
  }

  drake::systems::DiagramBuilder<double> builder_;
  MultibodyPlant<double>* sim_plant_{nullptr};
  drake::lcm::DrakeLcm lcm_;
  ModelInstanceIndex wsg_instance_;
};

TEST_F(BuildWsgControlTest, BuildWsgControl) {
  BuildWsgControl(*sim_plant_, wsg_instance_, &lcm_, &builder_);
  const auto diagram = builder_.Build();
  drake::systems::Simulator<double> simulator(*diagram);

  drake::lcm::Subscriber<drake::lcmt_schunk_wsg_status> sub{
      &lcm_, "SCHUNK_WSG_STATUS"};

  // Publish commands and check whether the wsg gripper moves to the correct
  // positions.
  drake::lcmt_schunk_wsg_command command{};
  command.force = 40.0;
  command.target_position_mm = 100.0;
  Publish(&lcm_, "SCHUNK_WSG_COMMAND", command);
  lcm_.HandleSubscriptions(0);
  simulator.AdvanceTo(1.0);
  lcm_.HandleSubscriptions(0);
  EXPECT_NEAR(sub.message().actual_position_mm, 100.0, kTolerance);

  command.target_position_mm = 0.0;
  Publish(&lcm_, "SCHUNK_WSG_COMMAND", command);
  lcm_.HandleSubscriptions(0);
  simulator.AdvanceTo(2.0);
  lcm_.HandleSubscriptions(0);
  EXPECT_NEAR(sub.message().actual_position_mm, 0.0, kTolerance);
}

}  // namespace
}  // namespace sim
}  // namespace anzu
