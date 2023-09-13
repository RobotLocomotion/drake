#include "drake/manipulation/kuka_iiwa/sim_iiwa_driver.h"

#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace internal {
namespace {

using multibody::MultibodyPlant;
using multibody::Parser;
using systems::InputPortIndex;
using systems::OutputPortIndex;
using systems::System;

class SimIiwaDriverTest : public ::testing::Test {
 public:
  SimIiwaDriverTest() = default;

 protected:
  void SetUp() {
    sim_plant_ = std::make_unique<MultibodyPlant<double>>(0.001);
    Parser parser{sim_plant_.get()};
    parser.AddModelsFromUrl(
        "package://drake/manipulation/models/iiwa_description/iiwa7/"
        "iiwa7_no_collision.sdf");
    sim_plant_->WeldFrames(sim_plant_->world_frame(),
                           sim_plant_->GetFrameByName("iiwa_link_0"));
    sim_plant_->Finalize();
    controller_plant_ = System<double>::Clone(*sim_plant_);
  }

  std::unique_ptr<MultibodyPlant<double>> sim_plant_;
  std::unique_ptr<MultibodyPlant<double>> controller_plant_;
  double ext_joint_filter_tau_{0.1};
};

TEST_F(SimIiwaDriverTest, SanityCheck) {
  for (const auto& mode :
       {IiwaControlMode::kPositionOnly, IiwaControlMode::kTorqueOnly,
        IiwaControlMode::kPositionAndTorque}) {
    SCOPED_TRACE(fmt::format("mode = {}", static_cast<int>(mode)));
    const SimIiwaDriver<double> dut(mode, controller_plant_.get(),
                                    ext_joint_filter_tau_, {});

    // Check input port names.
    std::vector<std::string> inputs;
    for (InputPortIndex i{0}; i < dut.num_input_ports(); ++i) {
      inputs.push_back(dut.get_input_port(i).get_name());
    }
    std::vector<std::string> expected_inputs{"state",
                                             "generalized_contact_forces"};
    if (position_enabled(mode)) {
      expected_inputs.push_back("position");
    }
    if (torque_enabled(mode)) {
      expected_inputs.push_back("torque");
    }
    EXPECT_THAT(inputs, testing::ElementsAreArray(expected_inputs));

    // Check output port names.
    std::vector<std::string> outputs;
    for (OutputPortIndex i{0}; i < dut.num_output_ports(); ++i) {
      outputs.push_back(dut.get_output_port(i).get_name());
    }
    std::vector<std::string> expected_outputs{
        "actuation",          "position_commanded", "position_measured",
        "velocity_estimated", "state_estimated",    "torque_commanded",
        "torque_measured",    "torque_external"};
    EXPECT_THAT(outputs, testing::ElementsAreArray(expected_outputs));

    // Check scalar conversion.
    EXPECT_TRUE(systems::is_autodiffxd_convertible(dut));
    EXPECT_TRUE(systems::is_symbolic_convertible(dut));
  }
}

}  // namespace
}  // namespace internal
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
