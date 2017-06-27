/// @file
///
/// This test sets up a simple passive dynamics simulation of the Fetch mobile
/// robot, i.e., all joint torques are set to zero.

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/Fetch/fetch_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using systems::Context;
using systems::ContinuousState;
using systems::RigidBodyPlant;
using systems::VectorBase;

namespace examples {
namespace Fetch {

GTEST_TEST(FetchSimTest, PassiveTest) {
  drake::lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double> *plant = nullptr;
  const std::string kModelPath =
      drake::GetDrakePath() +
      "/examples/Fetch/fetch_description/robots/fetch.urdf";
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::multibody::AddFlatTerrainToWorld(tree.get());
    CreateTreeFromFloatingModelAtPose(kModelPath, tree.get());

    const double contact_stiffness = 2000;
    const double contact_dissipation = 2;

    plant = builder.AddSystem<RigidBodyPlant<double>>(std::move(tree));
    plant->set_name("plant");
    plant->set_normal_contact_parameters(contact_stiffness,
                                         contact_dissipation);
  }

  // Verifies the tree.
  const RigidBodyTree<double> &tree = plant->get_rigid_body_tree();
  VerifyFetchTree(tree);

  // Creates and adds LCM publisher for visualization.
  auto visualizer = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);

  // Feeds in constant command inputs of zero.
  VectorX<double> zero_values = VectorX<double>::Zero(plant->get_input_size());
  auto zero_source =
      builder.AddSystem<systems::ConstantVectorSource<double>>(zero_values);
  zero_source->set_name("zero_source");
  builder.Connect(zero_source->get_output_port(), plant->get_input_port(0));

  // Connects the visualizer and builds the diagram.
  builder.Connect(plant->get_output_port(0), visualizer->get_input_port(0));
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  Context<double> *fetch_context = diagram->GetMutableSubsystemContext(
      simulator.get_mutable_context(), plant);

  // Sets torso lift initial conditions.
  // See the @file docblock in fetch_common.h for joint index descriptions.
  VectorBase<double> *x0 = fetch_context->get_mutable_continuous_state_vector();
  x0->SetAtIndex(kTorsoLiftJointIdx, 0.1);

  simulator.Initialize();

  // Simulate for the desired duration.
  simulator.set_target_realtime_rate(1);
  simulator.StepTo(0.1);

  // Ensures the simulation was successful.
  const Context<double> &context = simulator.get_context();
  const ContinuousState<double> *state = context.get_continuous_state();
  const VectorBase<double> &position_vector = state->get_generalized_position();
  const VectorBase<double> &velocity_vector = state->get_generalized_velocity();

  const int num_q = position_vector.size();
  const int num_v = velocity_vector.size();

  // Ensures the sizes of the position and velocity vectors are correct.
  EXPECT_EQ(num_q, plant->get_num_positions());
  EXPECT_EQ(num_v, plant->get_num_velocities());
  EXPECT_EQ(num_q, num_v + 1);

  for (int i = 0; i < kNumDofs; i++) {
    EXPECT_FALSE(std::isnan(position_vector.GetAtIndex(i)));
    EXPECT_FALSE(std::isinf(position_vector.GetAtIndex(i)));
  }
}

}  // namespace Fetch
}  // namespace examples
}  // namespace drake
