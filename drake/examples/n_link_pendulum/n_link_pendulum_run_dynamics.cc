#include "drake/examples/n_link_pendulum/n_link_pendulum_plant.h"
#include "drake/geometry/geometry_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace examples {
namespace n_link_pendulum {
namespace {

using geometry::GeometrySystem;
using geometry::SourceId;
using lcm::DrakeLcm;
using systems::DiagramContext;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using systems::ImplicitEulerIntegrator;
using systems::rendering::PoseBundleToDrawMessage;
using systems::RungeKutta2Integrator;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto geometry_system = builder.AddSystem<GeometrySystem<double>>();
  geometry_system->set_name("geometry_system");

  const double mass = 0.5;      // [Kgr], about a pound.
  const double length = 0.7;    // [m]
  const double radius = 0.015;  // [m]
  const double num_links = 20;
  const double time_step = 0.01;
  const std::string integrator_type = "ImplicitEuler";
  //const std::string integrator_type = "RungeKutta3";
  
  auto pendulum = builder.AddSystem<NLinkPendulumPlant>(
      mass, length, radius, num_links, geometry_system);
  pendulum->set_name("NLinkPendulum");

  DrakeLcm lcm;
  PoseBundleToDrawMessage* converter =
      builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem* publisher =
      builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher->set_publish_period(1 / 60.0);

  builder.Connect(
      pendulum->get_geometry_id_output_port(),
      geometry_system->get_source_frame_id_port(pendulum->source_id()));
  builder.Connect(
      pendulum->get_geometry_pose_output_port(),
      geometry_system->get_source_pose_port(pendulum->source_id()));

  builder.Connect(geometry_system->get_pose_bundle_output_port(),
                  converter->get_input_port(0));
  builder.Connect(*converter, *publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(*geometry_system);

  auto diagram = builder.Build();

  auto diagram_context = diagram->CreateDefaultContext();
  systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(
          *pendulum, diagram_context.get());
  pendulum->SetStraightAtAnAngle(&pendulum_context, M_PI / 3.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

//  SemiExplicitEulerIntegrator<double>* integrator =
  //    simulator.reset_integrator<SemiExplicitEulerIntegrator<double>>(
    //  *diagram, time_step, simulator.get_mutable_context());

  //RungeKutta2Integrator<double>* integrator =
    //      simulator.reset_integrator<RungeKutta2Integrator<double>>(
      //    *diagram, time_step, simulator.get_mutable_context());

  systems::IntegratorBase<double>* integrator;
  if (integrator_type == "ImplicitEuler") {
    integrator =
        simulator.reset_integrator<ImplicitEulerIntegrator<double>>(
            *diagram, simulator.get_mutable_context());
  } else if (integrator_type == "RungeKutta3") {
    integrator =
        simulator.reset_integrator<RungeKutta3Integrator<double>>(
            *diagram, simulator.get_mutable_context());
  } else {
    throw std::logic_error("Other integrator types are unstable for this case");
  }

  integrator->set_maximum_step_size(time_step);
  integrator->set_target_accuracy(0.1);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.f);
  simulator.Initialize();
  simulator.StepTo(2.0);

  PRINT_VAR(integrator->get_num_steps_taken());
  PRINT_VAR(integrator->get_num_derivative_evaluations());
  PRINT_VAR(integrator->get_num_step_shrinkages_from_error_control());
  PRINT_VAR(integrator->get_num_step_shrinkages_from_substep_failures());
  PRINT_VAR(integrator->get_num_substep_failures());
  PRINT_VAR(integrator->get_largest_step_size_taken());
  PRINT_VAR(integrator->get_smallest_adapted_step_size_taken());


  return 0;
}

}  // namespace
}  // namespace n_link_pendulum
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::n_link_pendulum::do_main(); }
