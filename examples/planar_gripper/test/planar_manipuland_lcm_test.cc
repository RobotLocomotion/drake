#include "drake/examples/planar_gripper/planar_manipuland_lcm.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace planar_gripper {
GTEST_TEST(PlanarManipulandLcmTest, PlanarManipulandStatusPassthroughTest) {
  systems::DiagramBuilder<double> builder;
  auto status_encoder = builder.AddSystem<PlanarManipulandStatusEncoder>();
  auto status_decoder = builder.AddSystem<PlanarManipulandStatusDecoder>();
  builder.Connect(status_decoder->get_output_port(0),
                  status_encoder->get_input_port(0));
  builder.ExportInput(status_decoder->get_input_port(0));
  builder.ExportOutput(status_encoder->get_output_port(0));
  auto diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();
  std::unique_ptr<systems::SystemOutput<double>> output =
      diagram->AllocateOutput();

  lcmt_planar_manipuland_status status{};
  status.position[0] = 0.1;
  status.position[1] = 0.2;
  status.theta = 0.3;
  status.velocity[0] = 0.4;
  status.velocity[1] = 0.5;
  status.thetadot = 0.6;

  diagram->get_input_port(0).FixValue(context.get(), status);

  std::unique_ptr<systems::DiscreteValues<double>> update =
      diagram->AllocateDiscreteVariables();
  update->SetFrom(context->get_mutable_discrete_state());
  diagram->CalcDiscreteVariableUpdates(*context, update.get());
  context->get_mutable_discrete_state().SetFrom(*update);
  diagram->CalcOutput(*context, output.get());

  lcmt_planar_manipuland_status status_out =
      output->get_data(0)->get_value<lcmt_planar_manipuland_status>();

  EXPECT_DOUBLE_EQ(status.position[0], status_out.position[0]);
  EXPECT_DOUBLE_EQ(status.position[1], status_out.position[1]);
  EXPECT_DOUBLE_EQ(status.theta, status_out.theta);
  EXPECT_DOUBLE_EQ(status.velocity[0], status_out.velocity[0]);
  EXPECT_DOUBLE_EQ(status.velocity[1], status_out.velocity[1]);
  EXPECT_DOUBLE_EQ(status.thetadot, status_out.thetadot);
}
}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
