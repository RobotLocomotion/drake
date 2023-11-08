#include "drake/manipulation/schunk_wsg/build_schunk_wsg_control.h"

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

namespace drake {
namespace manipulation {
namespace schunk_wsg {
namespace {

using math::RigidTransformd;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;

struct BuildSchunkWsgControlTestConfig {
  double tolerance;
  std::string gripper_model_path;
};

std::ostream& operator<<(std::ostream& out,
                         const BuildSchunkWsgControlTestConfig& c) {
  out << "\nBuildSchunkWsgControlTestConfig:";
  out << "\n  tolerance: " << c.tolerance;
  out << "\n  gripper_model_path: " << c.gripper_model_path;
  return out;
}

class BuildSchunkWsgControlTest
    : public ::testing::TestWithParam<BuildSchunkWsgControlTestConfig> {
 public:
  BuildSchunkWsgControlTest() = default;

 protected:
  void SetUp() {
    auto config = GetParam();

    sim_plant_ = builder_.AddSystem<MultibodyPlant<double>>(0.001);
    sim_plant_->set_discrete_contact_solver(
        drake::multibody::DiscreteContactSolver::kSap);
    Parser parser{sim_plant_};
    const std::string wsg_file = FindResourceOrThrow(config.gripper_model_path);
    wsg_instance_ = parser.AddModels(wsg_file).at(0);

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

TEST_P(BuildSchunkWsgControlTest, BuildSchunkWsgControl) {
  auto config = GetParam();

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
  EXPECT_NEAR(sub.message().actual_position_mm, 100.0, config.tolerance);

  command.target_position_mm = 0.0;
  Publish(&lcm_, "SCHUNK_WSG_COMMAND", command);
  lcm_.HandleSubscriptions(0);
  simulator.AdvanceTo(2.0);
  lcm_.HandleSubscriptions(0);
  EXPECT_NEAR(sub.message().actual_position_mm, 0.0, config.tolerance);
}

std::vector<BuildSchunkWsgControlTestConfig> MakeTestCases() {
  return std::vector<BuildSchunkWsgControlTestConfig>{
      {.tolerance = 1e-3,
       .gripper_model_path = "drake/manipulation/models/wsg_50_description/"
                             "sdf/schunk_wsg_50.sdf"},
      {.tolerance = 1e-3,
       .gripper_model_path = "drake/manipulation/models/wsg_50_description/"
                             "sdf/schunk_wsg_50_with_mimic_and_tip.sdf"}};
}

INSTANTIATE_TEST_SUITE_P(BuildSchunkWsgControl, BuildSchunkWsgControlTest,
                         testing::ValuesIn(MakeTestCases()));

}  // namespace
}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
