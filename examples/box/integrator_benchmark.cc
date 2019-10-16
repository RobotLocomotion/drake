#include <gflags/gflags.h>

#include "drake/examples/box/box_geometry.h"
#include "drake/examples/box/box_plant.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
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
DEFINE_double(simulation_time, 2.5,
              "Desired duration of the simulation. [s].");

// Integration parameters:
DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'fixed_implicit_euler', 'implicit_euler' (ec), 'semi_explicit_euler',"
              "'runge_kutta2', 'runge_kutta3' (ec), 'bogacki_shampine3' (ec), 'radau'");

DEFINE_string(run_filename, "boxout",
              "Filename for output. \".csv\" will be postpended.");

DEFINE_string(meta_filename, "boxsim",
                "Filename for meta output. \".csv\" will be postpended.");

DEFINE_string(errors_filename, "boxlerr",
                "Filename for local error output. \".csv\" will be postpended.");

DEFINE_double(max_time_step, 1.0e-3,
              "Maximum time step used for the integrators. [s]. "
              "Must be at most error_reporting_step.");

DEFINE_bool(iteration_limit, false, "Set true to use iteration limiter.");
DEFINE_bool(fixed_step, false, "Set true to force fixed timesteps.");
DEFINE_bool(autodiff, true, "Set true to use AutoDiff in Jacobian computation.");
DEFINE_double(fixed_tolerance, 1.e-5, "Tolerance for Newton iterations of fixed implicit integrators.");

DEFINE_string(truth_integration_scheme, "runge_kutta3",
              "Integration scheme for computing truth (fixed). Available options are: "
              "'fixed_implicit_euler', 'implicit_euler' (ec), 'semi_explicit_euler',"
              "'runge_kutta2', 'runge_kutta3' (ec), 'bogacki_shampine3' (ec), 'radau'");
DEFINE_bool(truth_autodiff, true, "Set true to use AutoDiff in Jacobian computation in truth.");
DEFINE_double(truth_integration_step, 3.0e-7,
              "Timestep size for integrating the truth.");

DEFINE_bool(truth_fixed_step, true, "Use fixed steps for truth.");
DEFINE_double(truth_accuracy, 1e-17, "Target accuracy for truth.");
DEFINE_double(error_reporting_step, 1.0e-2,
              "Period between which local error is calculated.");


DEFINE_bool(visualize, false, "Set true to visualize");
DEFINE_bool(log_values, false, "Set true to log");

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

DEFINE_double(box_v_s, 1.0e-4,
              "The maximum slipping speed allowed during stiction. (m/s)");


void StoreEigenCSV(const std::string& filename, const Eigen::VectorXd& times, const Eigen::MatrixXd& data,
                const Eigen::MatrixXi& metadata, const BoxPlant<double>& box) {
  /* csv format from  https://stackoverflow.com/questions/18400596/how-can-a-eigen-matrix-be-written-to-file-in-csv-format */
  const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision,
                                  Eigen::DontAlignCols, ", ", "\n");
  DRAKE_DEMAND(times.rows() == data.rows());
  DRAKE_DEMAND(times.rows() == metadata.rows());
   VectorX<double> sim_friction(data.rows());
   VectorX<double> truth_friction(data.rows());
  for( int i = 0; i < data.rows(); i++)
  {
    double sim_velocity = data(i, 1);
    double truth_velocity = data(i, 3);
    sim_friction(i) = box.CalcFrictionFromVelocity(sim_velocity);
    truth_friction(i) = box.CalcFrictionFromVelocity(truth_velocity);
  } 
  std::ofstream file(filename);
  file << "t, n_der, n_steps, truth_n_steps, sim box_x, sim box_v, sim f_t, box_x, box_v, f_t" << std::endl;
  /* horizontally concatenate times and data */
  MatrixX<double> OutMatrix(times.rows(), times.cols() + metadata.cols() + data.cols()
                            + sim_friction.cols() + truth_friction.cols());
  OutMatrix << times, metadata.cast<double>(), data.block(0,0,data.rows(), 2),
     sim_friction, data.block(0,2,data.rows(), 2) , truth_friction; 
     /* can also do this with blocks */
  file << OutMatrix.format(CSVFormat);
  file.close();
}
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
  systems::SignalLogger<double>* logger = nullptr;
  systems::SignalLogger<double>* input_logger = nullptr;
  if( FLAGS_log_values )
  {
    logger = systems::LogOutput(box->get_state_output_port(), &builder);
    input_logger = systems::LogOutput(source->get_output_port(0), &builder);
  }
  if( FLAGS_visualize )
  {
    BoxGeometry::AddToBuilder(
        &builder, *box, scene_graph, "0");
    ConnectDrakeVisualizer(&builder, *scene_graph);
  }
  auto diagram = builder.Build();


  systems::Simulator<double> simulator(*diagram);
  systems::Simulator<double> truth_simulator(*diagram);
  systems::Context<double>& box_context =
      diagram->GetMutableSubsystemContext(*box,
                                          &simulator.get_mutable_context());
  
  drake::VectorX<double> initState(2);
  initState << FLAGS_box_x0 /* position */, FLAGS_box_v0 /* velocity */;
  box->set_initial_state(&box_context, initState);

  systems::Context<double>& truth_context = truth_simulator.get_mutable_context();
  truth_context.get_mutable_state().SetFrom(simulator.get_context().get_state());

  std::cout << "initial state: \n" << truth_context.get_continuous_state_vector().CopyToVector() << std::endl;
  std::cout << "box friction: " << box->CalcFrictionFromVelocity(truth_context.get_continuous_state_vector().GetAtIndex(1)) << std::endl;
  systems::IntegratorBase<double>* integrator{nullptr};
  systems::IntegratorBase<double>* truth_integrator{nullptr}; unused(truth_integrator);



  if (FLAGS_integration_scheme == "implicit_euler") {
    integrator =
        simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
            *diagram, &simulator.get_mutable_context());
    if(FLAGS_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
    if(FLAGS_fixed_step)
    {
      integrator->set_target_accuracy(FLAGS_fixed_tolerance);
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
  } else if (FLAGS_integration_scheme == "fixed_implicit_euler") {
    integrator =
        simulator.reset_integrator<systems::RadauIntegrator<double,1>>(
            *diagram, &simulator.get_mutable_context());
    if(FLAGS_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
    if(FLAGS_fixed_step)
    {
      integrator->set_target_accuracy(FLAGS_fixed_tolerance);
    }
  } else if (FLAGS_integration_scheme == "radau") {
    integrator =
        simulator.reset_integrator<systems::RadauIntegrator<double,2>>(
            *diagram, &simulator.get_mutable_context());
    if(FLAGS_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
    if(FLAGS_fixed_step)
    {
      integrator->set_target_accuracy(FLAGS_fixed_tolerance);
    }
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
            "' not supported for this example.");
  }
      // Set the iteration limiter method.
    auto iteration_limiter = [box](const systems::Context<double>&, const systems::ContinuousState<double>& x_k,
       const systems::ContinuousState<double>& x_kp1) -> double {
         Eigen::VectorXd x0 = x_k.CopyToVector();
         Eigen::VectorXd dx = x_kp1.CopyToVector() - x0;
         return box->CalcIterationLimiterAlpha(x0, dx);
      };
  if(FLAGS_iteration_limit)
  {
    integrator->set_iteration_limiter(iteration_limiter);
  }
  integrator->set_maximum_step_size(FLAGS_max_time_step);
  if (integrator->supports_error_estimation())
    integrator->set_fixed_step_mode( FLAGS_fixed_step );
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);

  if (FLAGS_visualize || FLAGS_log_values)
  {
    // The error controlled integrators might need to take very small time steps
    // to compute a solution to the desired accuracy. Therefore, to visualize
    // these very short transients, we publish every time step.
    simulator.set_publish_every_time_step(true);
  }
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();



  if (FLAGS_truth_integration_scheme == "implicit_euler") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>(
            *diagram, &truth_simulator.get_mutable_context());
    if(FLAGS_truth_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(truth_integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
  } else if (FLAGS_truth_integration_scheme == "runge_kutta2") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::RungeKutta2Integrator<double>>(
            *diagram, FLAGS_truth_integration_step,
            &truth_simulator.get_mutable_context());
  } else if (FLAGS_truth_integration_scheme == "runge_kutta3") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::RungeKutta3Integrator<double>>(
            *diagram, &truth_simulator.get_mutable_context());
  } else if (FLAGS_truth_integration_scheme == "bogacki_shampine3") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::BogackiShampine3Integrator<double>>(
            *diagram, &truth_simulator.get_mutable_context());
  } else if (FLAGS_truth_integration_scheme == "semi_explicit_euler") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
            *diagram, FLAGS_truth_integration_step, &truth_simulator.get_mutable_context());
  } else if (FLAGS_truth_integration_scheme == "fixed_implicit_euler") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::RadauIntegrator<double,1>>(
            *diagram, &truth_simulator.get_mutable_context());
    if(FLAGS_truth_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(truth_integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
  } else if (FLAGS_truth_integration_scheme == "radau") {
    truth_integrator =
        truth_simulator.reset_integrator<systems::RadauIntegrator<double>>(
            *diagram, &truth_simulator.get_mutable_context());
    if(FLAGS_truth_autodiff)
    {
      static_cast<systems::ImplicitIntegrator<double>*>(truth_integrator)->set_jacobian_computation_scheme(
        systems::ImplicitIntegrator<double>::JacobianComputationScheme::kAutomatic);
    }
  } else {
    throw std::runtime_error(
        "Truth integration scheme '" + FLAGS_truth_integration_scheme +
            "' not supported for this example.");
  }
  
  truth_integrator->set_maximum_step_size(FLAGS_truth_integration_step);
  if (truth_integrator->supports_error_estimation())
    truth_integrator->set_fixed_step_mode( FLAGS_truth_fixed_step );
  if (!truth_integrator->get_fixed_step_mode())
    truth_integrator->set_target_accuracy(FLAGS_truth_accuracy);
  
  truth_simulator.set_target_realtime_rate(0.0);
  truth_simulator.Initialize();

  int nsteps = std::ceil(FLAGS_simulation_time / FLAGS_error_reporting_step);
  int nstate = simulator.get_context().get_continuous_state_vector().size();
  int nmetadata = 3; /* der evals for sim, num steps for sim, num steps for truth */
  Eigen::VectorXd times(nsteps+1);
  Eigen::MatrixXd error_results(nsteps+1, 2 * nstate );
  Eigen::MatrixXi error_meta(nsteps+1, nmetadata);
  double time = 0;
  for(int next_step_ind = 1; next_step_ind <= nsteps; ++next_step_ind)
  {
    double next_time = time + next_step_ind * FLAGS_error_reporting_step;
    if ( next_step_ind == nsteps )
    {
      next_time = FLAGS_simulation_time;
    }
    systems::Context<double>& curr_truth_context = truth_simulator.get_mutable_context();
    curr_truth_context.get_mutable_state().SetFrom(simulator.get_context().get_state());
    simulator.AdvanceTo(next_time);
    truth_simulator.AdvanceTo(next_time);
    auto& simstate = simulator.get_context().get_continuous_state_vector();
    auto& truthstate = truth_simulator.get_context().get_continuous_state_vector();
    times(next_step_ind) = next_time;
    error_results(next_step_ind , 0) = simstate[0] ;
    error_results(next_step_ind , 1) = simstate[1] ;

    error_results(next_step_ind , 2) = truthstate[0] ;
    error_results(next_step_ind , 3) = truthstate[1] ;
    error_meta(next_step_ind , 0) = integrator->get_num_derivative_evaluations();
    error_meta(next_step_ind , 1) = integrator->get_num_steps_taken();
    error_meta(next_step_ind , 2) = truth_integrator->get_num_steps_taken();

  }

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

  StoreEigenCSV(FLAGS_errors_filename + ".csv", times, error_results, error_meta, *box);

  if(FLAGS_log_values)
  {
    DRAKE_DEMAND(logger->sample_times().rows() == input_logger->sample_times().rows());
    box->StoreEigenCSVwFriction(FLAGS_run_filename + ".csv", logger->sample_times(), logger->data(), (input_logger->data()).transpose());
  }

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
