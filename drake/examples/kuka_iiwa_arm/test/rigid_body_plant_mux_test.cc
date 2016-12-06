#include "drake/examples/kuka_iiwa_arm/rigid_body_plant_mux.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using drake::parsers::urdf::AddModelInstanceFromUrdfFile;

GTEST_TEST(RigidBodyPlantMuxTest, SimpleMuxTest) {
  // Create a tree with 4 actuators and 4 joints.
  RigidBodyTree<double> tree;
  AddModelInstanceFromUrdfFile(
      drake::GetDrakePath() +
      "/multibody/test/rigid_body_tree/four_dof_robot.urdf",
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      &tree);

  // Create our mux under test, and make two inputs and two outputs.
  RigidBodyPlantMux<double> dut(tree);
  auto input_one = dut.AddInput({"joint1", "joint2"});
  auto input_two = dut.AddInput({"joint3", "joint4"});
  auto output_one = dut.AddOutput({"joint1", "joint2"},
                                  {"joint1dot", "joint2dot"});
  auto output_two = dut.AddOutput({"joint3", "joint4"},
                                  {"joint3dot", "joint4dot"});

  // Build up a diagram with a multiplexer and a couple of gain blocks
  // to simulate a plant with 4 inputs and 8 outputs.
  systems::DiagramBuilder<double> plant_builder;
  auto pass = plant_builder.AddSystem<systems::PassThrough<double>>(4);
  auto position_gain = plant_builder.AddSystem<systems::Gain<double>>(10, 4);
  auto velocity_gain = plant_builder.AddSystem<systems::Gain<double>>(100, 4);
  auto plant_mux = plant_builder.AddSystem<systems::Multiplexer<double>>(
      std::vector<int>({4, 4}));

  plant_builder.Connect(*pass, *position_gain);
  plant_builder.Connect(*pass, *velocity_gain);
  plant_builder.Connect(position_gain->get_output_port(),
                        plant_mux->get_input_port(0));
  plant_builder.Connect(velocity_gain->get_output_port(),
                        plant_mux->get_input_port(1));
  plant_builder.ExportInput(pass->get_input_port(0));
  plant_builder.ExportOutput(plant_mux->get_output_port(0));

  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem(plant_builder.Build());
  dut.ConnectPlant(*plant, &builder);
  builder.ExportInput(input_one);
  builder.ExportInput(input_two);
  builder.ExportOutput(output_one);
  builder.ExportOutput(output_two);
  auto sys = builder.Build();

  auto context = sys->CreateDefaultContext();
  auto output = sys->AllocateOutput(*context);
  context->FixInputPort(0, Eigen::Vector2d(1, 2));
  context->FixInputPort(1, Eigen::Vector2d(3, 4));
  sys->EvalOutput(*context, output.get());

  auto output_one_vec = output->get_vector_data(0);
  auto output_two_vec = output->get_vector_data(1);
  ASSERT_EQ(output_one_vec->size(), 4);
  ASSERT_EQ(output_two_vec->size(), 4);

  Eigen::VectorXd expected_output_one(4);
  expected_output_one << 10, 20, 100, 200;

  Eigen::VectorXd expected_output_two(4);
  expected_output_two << 30, 40, 300, 400;

  EXPECT_TRUE(CompareMatrices(output_one_vec->get_value(), expected_output_one,
                              1e-10));
  EXPECT_TRUE(CompareMatrices(output_two_vec->get_value(), expected_output_two,
                              1e-10));
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
