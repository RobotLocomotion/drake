#include <gflags/gflags.h>

#include "drake/examples/box/box_geometry.h"
#include "drake/examples/box/box_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/second_order_implicit_euler_integrator.h"
#include "drake/systems/analysis/radau_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/sine.h"
#include <fstream>

namespace drake {
namespace examples {
namespace box {
namespace {
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_time, 4.0,
              "Desired duration of the simulation. [s].");

// Integration parameters:
DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'radau1', 'implicit_euler' (ec), 'second_order_implicit_euler', 'semi_explicit_euler',"
              "'runge_kutta2', 'runge_kutta3' (ec), 'bogacki_shampine3' (ec), 'radau'");

DEFINE_string(run_filename, "boxout",
              "Filename for output. \".csv\" will be postpended.");

DEFINE_string(meta_filename, "boxsim",
                "Filename for meta output. \".csv\" will be postpended.");

DEFINE_double(max_time_step, 3e-3,
              "Maximum time step used for the integrators. [s]. "
              "If negative, a value based on parameter penetration_allowance "
              "is used.");

DEFINE_bool(fixed_step, true, "Set true to force fixed timesteps.");
DEFINE_bool(autodiff, false, "Set true to use AutoDiff in Jacobian computation (also disables visualization).");
DEFINE_double(fixed_tolerance, 1.e-5, "Tolerance for Newton iterations of fixed implicit integrators.");

DEFINE_double(accuracy, 1.0e-2, "Sets the simulation accuracy for variable step"
              "size integrators with error control.");

DEFINE_double(force_amplitude, 4.0,
              "sinusoidal force amplitude (N)");

DEFINE_double(force_freq, 1.0,
              "sinusoidal force freq (Hz)");

DEFINE_double(box_x0, 0.0,
              "initial position for box (m)");
DEFINE_double(box_v0, 0.0,
              "initial velocity for box (m/s)");

DEFINE_double(box_m, 0.33,
              "box mass (kg)");

DEFINE_double(box_d, 0.,
              "box viscous damping (s^-1)");

DEFINE_double(box_l, 0.12,
              "box length (m)");

DEFINE_double(box_f_n, 3.234,
              "box normal force (N)");

DEFINE_double(box_mu_s, 1.0,
              "box static friction coefficient");

DEFINE_double(box_v_s, 1.0e-2,
              "The maximum slipping speed allowed during stiction. (m/s)");

DEFINE_bool(visualize, true, "Set 'false' to disable visualization.");

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto source = builder.AddSystem<systems::Sine>(FLAGS_force_amplitude /* amplitude */, 
                                                  2 * M_PI * FLAGS_force_freq /* omega */, 
                                                  M_PI / 2.0 /* phase */, 1 /* vector size */);
  source->set_name("source");
  double inv_mass = FLAGS_box_m > 0 ? 1.0 / FLAGS_box_m : 0.0;
  auto box = builder.AddSystem<BoxPlant>(inv_mass, FLAGS_box_l, FLAGS_box_d, FLAGS_box_f_n,
                                         FLAGS_box_mu_s, FLAGS_box_mu_s, FLAGS_box_v_s );
  box->set_name("box");
  builder.Connect(source->get_output_port(0), box->get_input_port());
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  auto logger = systems::LogOutput(box->get_state_output_port(), &builder);
  auto input_logger = systems::LogOutput(source->get_output_port(0), &builder);
  
  if( FLAGS_visualize )
  {
    BoxGeometry::AddToBuilder(
        &builder, *box, scene_graph, "0");
    ConnectDrakeVisualizer(&builder, *scene_graph);
  }
  auto diagram = builder.Build();


  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& box_context =
      diagram->GetMutableSubsystemContext(*box,
                                          &simulator.get_mutable_context());
  
  drake::VectorX<double> initState(2);
  initState << FLAGS_box_x0 /* position */, FLAGS_box_v0 /* velocity */;
  box->set_initial_state(&box_context, initState);


  systems::IntegratorBase<double>* integrator{nullptr};

  // (ANTE TODO): set Jacobian scheme to kAutomatic via set_jacobian_computation_scheme
  // Ante TODO: investigate target accuracy
  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
    if(FLAGS_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
            *diagram, FLAGS_max_time_step, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    integrator =
        simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "bogacki_shampine3") {
    integrator =
        simulator.reset_integrator<systems::BogackiShampine3Integrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    integrator =
        simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
            *diagram, FLAGS_max_time_step, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "radau1") {
    integrator =
        simulator.reset_integrator<systems::RadauIntegrator<double,1>>(
            *diagram, &simulator.get_mutable_context());
    if(FLAGS_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
  } else if (FLAGS_integration_scheme == "second_order_implicit_euler") {
    integrator =
        simulator.reset_integrator<systems::SecondOrderImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
  } else if (FLAGS_integration_scheme == "radau") {
    integrator =
        simulator.reset_integrator<systems::RadauIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
    if(FLAGS_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }

  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
            "' not supported for this example.");
  }
  
  integrator->set_maximum_step_size(FLAGS_max_time_step);
  if (integrator->supports_error_estimation())
    integrator->set_fixed_step_mode( FLAGS_fixed_step );
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);
  
  if(FLAGS_fixed_step || integrator->get_fixed_step_mode())
  {
    integrator->set_target_accuracy(FLAGS_fixed_tolerance);
  }

  // The error controlled integrators might need to take very small time steps
  // to compute a solution to the desired accuracy. Therefore, to visualize
  // these very short transients, we publish every time step.
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_time);
  bool discrete = false;
  if ( discrete ) {
    DRAKE_UNREACHABLE();
    fmt::print("Used time stepping with dt={}\n", FLAGS_max_time_step);
    fmt::print("Number of time steps taken = {:d}\n",
               simulator.get_num_steps_taken());
  } else {
    fmt::print("Stats for integrator {}:\n", FLAGS_integration_scheme);
    fmt::print("Number of time steps taken = {:d}\n",
               integrator->get_num_steps_taken());
    fmt::print("Number of derivative evals = {:d}\n",
               integrator->get_num_derivative_evaluations());
    if (integrator->get_fixed_step_mode()) {
      fmt::print("Fixed time steps taken\n");
    }
    else
    {
      fmt::print("Initial time step taken = {:10.6g} s\n",
                 integrator->get_actual_initial_step_size_taken());
      fmt::print("Largest time step taken = {:10.6g} s\n",
                 integrator->get_largest_step_size_taken());
      fmt::print("Smallest adapted step size = {:10.6g} s\n",
                 integrator->get_smallest_adapted_step_size_taken());
      fmt::print("Number of steps shrunk due to error control = {:d}\n",
                 integrator->get_num_step_shrinkages_from_error_control());
    }
  }
  DRAKE_DEMAND(logger->sample_times().rows() == input_logger->sample_times().rows());
  box->StoreEigenCSVwFriction(FLAGS_run_filename + ".csv", logger->sample_times(), logger->data(), (input_logger->data()).transpose());

  std::ofstream file(FLAGS_meta_filename + ".csv");
  file << "steps, duration, max_dt, num_der_eval\n";
  file << integrator->get_num_steps_taken() << ", " << FLAGS_simulation_time << ", " << FLAGS_max_time_step << ", " << integrator->get_num_derivative_evaluations() << std::endl;
  file.close();
  return 0;
}

}  // namespace
}  // namespace box
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::box::DoMain();
}
