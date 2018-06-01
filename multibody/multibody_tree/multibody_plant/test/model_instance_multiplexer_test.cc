#include "drake/multibody/multibody_tree/multibody_plant/model_instance_multiplexer.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

namespace drake {
namespace multibody {
namespace multibody_plant {
namespace {

GTEST_TEST(ModelInstanceMultiplexer, ModelInstanceMultiplexerTest) {
  MultibodyPlant<double> plant;

  int instance1_idx = parsing::AddModelFromSdfFile(
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf"),
      &plant);
  int instance2_idx = parsing::AddModelFromSdfFile(
      FindResourceOrThrow("drake/multibody/multibody_tree/multibody_plant/"
                          "test/split_pendulum.sdf"),
      &plant);

  plant.Finalize();

  ModelInstanceMultiplexer<double> dut(plant);
  std::unique_ptr<systems::Context<double>> context =
      dut.CreateDefaultContext();

  context->FixInputPort(
      dut.get_actuation_input_port(instance1_idx).get_index(), {1});
  context->FixInputPort(
      dut.get_actuation_input_port(instance2_idx).get_index(), {2});

  const systems::OutputPort<double>& actuation_output_port =
      dut.get_actuation_output_port();
  std::unique_ptr<systems::AbstractValue> actuation_output =
      actuation_output_port.Allocate();
  actuation_output_port.Calc(*context, actuation_output.get());
  const systems::BasicVector<double>& actuation_vec =
      actuation_output->GetValueOrThrow<systems::BasicVector<double>>();
  EXPECT_EQ(actuation_vec.GetAtIndex(0), 1);
  EXPECT_EQ(actuation_vec.GetAtIndex(1), 2);

  context->FixInputPort(dut.get_state_input_port().get_index(),
                        {11, 12, 13, 14, 15, 16});

  const systems::OutputPort<double>& instance1_output_port =
      dut.get_state_output_port(instance1_idx);
  std::unique_ptr<systems::AbstractValue> instance1_output =
      instance1_output_port.Allocate();
  instance1_output_port.Calc(*context, instance1_output.get());
  const systems::BasicVector<double>& instance1_vec =
      instance1_output->GetValueOrThrow<systems::BasicVector<double>>();
  EXPECT_EQ(instance1_vec.GetAtIndex(0), 11);
  EXPECT_EQ(instance1_vec.GetAtIndex(1), 13);
  EXPECT_EQ(instance1_vec.GetAtIndex(2), 14);
  EXPECT_EQ(instance1_vec.GetAtIndex(3), 16);

  const systems::OutputPort<double>& instance2_output_port =
      dut.get_state_output_port(instance2_idx);
  std::unique_ptr<systems::AbstractValue> instance2_output =
      instance2_output_port.Allocate();
  instance2_output_port.Calc(*context, instance2_output.get());
  const systems::BasicVector<double>& instance2_vec =
      instance2_output->GetValueOrThrow<systems::BasicVector<double>>();
  EXPECT_EQ(instance2_vec.GetAtIndex(0), 12);
  EXPECT_EQ(instance2_vec.GetAtIndex(1), 15);
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
