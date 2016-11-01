//
// Created by foehnph on 01.11.16.
//

#include <drake/systems/plants/RigidBodySystem.h>
#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace examples {
namespace quadrotor {
namespace {

int do_main(int argc, char* argv[]) {
  lcm::DrakeLcm lcm;
  RigidBodyTree tree(GetDrakePath() + "/examples/Quadrotor/plant.urdf",
                     systems::plants::joints::kRollPitchYaw);

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(4);
  systems::DiagramBuilder<RigidBodySystem> builder;
  auto source = builder.AddSystem<systems::ConstantVectorSource>(u0);
  auto plant = builder.AddSystem<systems::RigidBodyPlant>(tree);
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);
  builder.Connect(source->get_output_port(),plant->get_input_ports());
  builder.Connect(plant->get_output_ports(), publisher->get_input_ports());

  auto diagram = builder.Build();
  systems::Simulator<RigidBodySystem> simulator(*diagram);
  systems::Context<RigidBodySystem>* quad_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), plant);
  plant->SetZeroConfiguration(quad_context);

  simulator.Initialize();
  simulator.StepTo(10);
  return 0;
}

}
}
}
}

int main(int argc, char* argv[]) {
  return drake::examples::quadrotor::do_main(argc, argv);
}