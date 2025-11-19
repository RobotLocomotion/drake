#include "drake/manipulation/schunk_wsg/build_schunk_wsg_control.h"

#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

using math::RigidTransformd;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;

constexpr double kTolerance = 1e-3;

class BuildSchunkWsgControlTest : public ::testing::Test {
 public:
  BuildSchunkWsgControlTest() = default;

 protected:
  void SetUp() {
    sim_plant_ = builder_.AddSystem<MultibodyPlant<double>>(0.001);
    Parser parser{sim_plant_};
    const std::string wsg_url =
        "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf";
    wsg_instance_ = parser.AddModelsFromUrl(wsg_url).at(0);

    // Weld the gripper to the world frame.
    sim_plant_->WeldFrames(sim_plant_->world_frame(),
                           sim_plant_->GetFrameByName("body", wsg_instance_),
                           RigidTransformd::Identity());
    sim_plant_->Finalize();
  }

  systems::DiagramBuilder<double> builder_;
  MultibodyPlant<double>* sim_plant_{nullptr};
  lcm::DrakeLcm lcm_;
  ModelInstanceIndex wsg_instance_;
};

TEST_F(BuildSchunkWsgControlTest, BuildSchunkWsgControl) {
  BuildSchunkWsgControl(*sim_plant_, wsg_instance_, &lcm_, &builder_);
  const auto diagram = builder_.Build();
  systems::Simulator<double> simulator(*diagram);

  lcm::Subscriber<lcmt_schunk_wsg_status> sub{&lcm_, "SCHUNK_WSG_STATUS"};

  // Publish commands and check whether the wsg gripper moves to the correct
  // positions.
  lcmt_schunk_wsg_command command{};
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

TEST_F(BuildSchunkWsgControlTest, CommandToCloseAtBeginning) {
  // The schunk gripper is initially open is commanded to close.
  BuildSchunkWsgControl(*sim_plant_, wsg_instance_, &lcm_, &builder_);
  const auto diagram = builder_.Build();
  systems::Simulator<double> simulator(*diagram);

  systems::Context<double>& simulator_context = simulator.get_mutable_context();
  auto& plant_context =
      sim_plant_->GetMyMutableContextFromRoot(&simulator_context);
  sim_plant_->SetPositions(&plant_context, wsg_instance_,
                           Eigen::Vector2d(-0.07, 0));

  lcm::Subscriber<lcmt_schunk_wsg_status> sub{&lcm_, "SCHUNK_WSG_STATUS"};

  // Publish commands and check whether the wsg gripper moves to the correct
  // positions.
  lcmt_schunk_wsg_command command{};
  command.force = 40.0;
  command.target_position_mm = 0.0;
  Publish(&lcm_, "SCHUNK_WSG_COMMAND", command);
  lcm_.HandleSubscriptions(0);
  simulator.AdvanceTo(1.0);
  lcm_.HandleSubscriptions(0);
  EXPECT_NEAR(sub.message().actual_position_mm, command.target_position_mm,
              kTolerance);
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
