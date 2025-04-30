#include "drake/manipulation/kuka_iiwa/sim_iiwa_driver.h"

#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace {

// Developer-only configuration.
constexpr bool kVerbose{false};

using Eigen::VectorXd;
using multibody::MultibodyPlant;
using multibody::Parser;
using systems::BasicVector;
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
    iiwa_instance_ = parser.AddModelsFromUrl(
        "package://drake_models/iiwa_description/sdf/"
        "iiwa7_no_collision.sdf")[0];
    sim_plant_->WeldFrames(sim_plant_->world_frame(),
                           sim_plant_->GetFrameByName("iiwa_link_0"));
    sim_plant_->Finalize();
    controller_plant_ = System<double>::Clone(*sim_plant_);
  }

  void TestSimIiwaDriverPorts(IiwaControlMode mode, const System<double>& dut) {
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

    if (position_enabled(mode)) {
      expected_outputs.push_back("velocity_commanded");
    }
    EXPECT_THAT(outputs, testing::UnorderedElementsAreArray(expected_outputs));

    // Check at least some data flow.
    auto simulator{systems::Simulator(dut)};
    auto& context = simulator.get_mutable_context();
    context.FixInputPort(dut.GetInputPort("state").get_index(),
                         Value<BasicVector<double>>(VectorXd::Zero(14)));
    context.FixInputPort(
        dut.GetInputPort("generalized_contact_forces").get_index(),
        Value<BasicVector<double>>(VectorXd::Zero(7)));
    if (position_enabled(mode)) {
      context.FixInputPort(
          dut.GetInputPort("position").get_index(),
          Value<BasicVector<double>>(VectorXd::Constant(7, 0.01)));
    }
    if (torque_enabled(mode)) {
      context.FixInputPort(dut.GetInputPort("torque").get_index(),
                           Value<BasicVector<double>>(VectorXd::Zero(7)));
    }
    simulator.AdvanceTo(0.1);
    if (position_enabled(mode)) {
      // Check position_commanded and velocity_commanded are distinct, and
      // wired correctly.
      auto& position_commanded_port = dut.GetOutputPort("position_commanded");
      EXPECT_EQ(position_commanded_port.Eval(simulator.get_context()),
                VectorXd::Constant(7, 0.01));
      auto& velocity_commanded_port = dut.GetOutputPort("velocity_commanded");
      EXPECT_EQ(velocity_commanded_port.Eval(simulator.get_context()),
                VectorXd::Zero(7));
    } else {
      // Port exists, always 0.
      auto& position_commanded_port = dut.GetOutputPort("position_commanded");
      EXPECT_EQ(position_commanded_port.Eval(simulator.get_context()),
                VectorXd::Zero(7));
    }

    if (kVerbose) {
      fmt::print("\n===BEGIN GRAPHVIZ===\n\n{}\n===END GRAPHVIZ===\n\n",
                 dut.GetGraphvizString());
    }
  }

  std::unique_ptr<MultibodyPlant<double>> sim_plant_;
  std::unique_ptr<MultibodyPlant<double>> controller_plant_;
  multibody::ModelInstanceIndex iiwa_instance_;
  const VectorXd desired_iiwa_kp_gains_ =
      (VectorXd(7) << 100, 100, 100, 100, 100, 100, 100).finished();
  IiwaDriver driver_config_{.desired_kp_gains = desired_iiwa_kp_gains_};
};

TEST_F(SimIiwaDriverTest, SanityCheck) {
  for (const auto& mode :
       {IiwaControlMode::kPositionOnly, IiwaControlMode::kTorqueOnly,
        IiwaControlMode::kPositionAndTorque}) {
    driver_config_.control_mode = FormatIiwaControlMode(mode);
    SCOPED_TRACE(fmt::format("mode = {}", driver_config_.control_mode));
    const SimIiwaDriver<double> dut(driver_config_, controller_plant_.get());

    TestSimIiwaDriverPorts(mode, dut);

    // Check scalar conversion.
    EXPECT_TRUE(systems::is_autodiffxd_convertible(dut));
    EXPECT_TRUE(systems::is_symbolic_convertible(dut));
  }
}

TEST_F(SimIiwaDriverTest, AddToBuilder) {
  for (const auto& mode :
       {IiwaControlMode::kPositionOnly, IiwaControlMode::kTorqueOnly,
        IiwaControlMode::kPositionAndTorque}) {
    driver_config_.control_mode = FormatIiwaControlMode(mode);
    SCOPED_TRACE(fmt::format("mode = {}", driver_config_.control_mode));
    systems::DiagramBuilder<double> builder;
    auto* sim_plant = builder.AddSystem(System<double>::Clone(*sim_plant_));
    const System<double>* const dut = &SimIiwaDriver<double>::AddToBuilder(
        &builder, *sim_plant, iiwa_instance_, driver_config_,
        *controller_plant_);

    TestSimIiwaDriverPorts(mode, *dut);
  }
}

}  // namespace
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
