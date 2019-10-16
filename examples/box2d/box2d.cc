#include <memory>
#include <string>

#include <gflags/gflags.h>
#include "fmt/ostream.h"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/radau_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/sine.h"
#include <fstream>

namespace drake {
namespace examples {
namespace box2d {
namespace {

using Eigen::Vector3d;
using geometry::SceneGraph;
using geometry::Sphere;
using lcm::DrakeLcm;
using math::RigidTransformd;
using math::RollPitchYawd;
using multibody::Body;
using multibody::CoulombFriction;
using multibody::ConnectContactResultsToDrakeVisualizer;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::PrismaticJoint;
using systems::ImplicitEulerIntegrator;
using systems::RungeKutta2Integrator;
using systems::RungeKutta3Integrator;
using systems::SemiExplicitEulerIntegrator;
using systems::Sine;

// TODO(amcastro-tri): Consider moving this large set of parameters to a
// configuration file (e.g. YAML).
DEFINE_double(target_realtime_rate, 0.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 2.5,
              "Desired duration of the simulation. [s].");

DEFINE_double(box_z0, 0.000,
              "The initial height of the box. [m].");

DEFINE_double(box_w0, 0.0,
              "The initial vertical velocity of the box. [m].");

DEFINE_double(box_x0, 0.0,
              "The initial x translation of the box. [m].");

DEFINE_double(box_v0, 0.0,
              "The initial x velocity of the box. [m].");

DEFINE_bool(use_friction, true, "Simulate with friction.");
DEFINE_bool(use_linear_friction, true, "Simulate with linear friction.");
DEFINE_bool(use_discrete_states, false, "uses discrete implicit stribeck");
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
DEFINE_bool(autodiff, false, "Set true to use AutoDiff in Jacobian computation.");
DEFINE_double(fixed_tolerance, 1.e-4, "Tolerance for Newton iterations of fixed implicit integrators.");

DEFINE_bool(full_newton, false, "set this to ensure implicit integrators do the full Newton-Raphson.");
DEFINE_bool(convergence_control, false, "set this to allow convergence control.");

DEFINE_string(truth_integration_scheme, "runge_kutta2",
              "Integration scheme for computing truth (fixed). Available options are: "
              "'fixed_implicit_euler', 'implicit_euler' (ec), 'semi_explicit_euler',"
              "'runge_kutta2', 'runge_kutta3' (ec), 'bogacki_shampine3' (ec), 'radau'");
DEFINE_bool(truth_autodiff, false, "Set true to use AutoDiff in Jacobian computation in truth.");
DEFINE_double(truth_integration_step, 3.0e-7,
              "Timestep size for integrating the truth.");

DEFINE_bool(truth_fixed_step, true, "Use fixed steps for truth.");
DEFINE_double(truth_accuracy, 1e-17, "Target accuracy for truth.");
DEFINE_double(error_reporting_step, 1.0e-2,
              "Period between which local error is calculated.");

DEFINE_bool(use_hydroelastics_model, false, "use hydroelastics for contact.");

DEFINE_bool(visualize, false, "Set true to visualize");
//DEFINE_bool(log_values, false, "Set true to log");

DEFINE_double(accuracy, 1.0e-2, "Sets the simulation accuracy for variable step"
              "size integrators with error control.");

// Contact parameters
DEFINE_double(penetration_allowance, 1.0e+1,
              "Penetration allowance. [m]. "
              "See MultibodyPlant::set_penetration_allowance().");
DEFINE_double(v_stiction_tolerance, 1.0e-4,
              "The maximum slipping speed allowed during stiction. [m/s]");

DEFINE_double(elastic_modulus, 2.05021814e4, "Elastic modulus, in Pa.");
DEFINE_double(dissipation, 2.0, "dissipation, in s/m.");    
// Gripping force.
DEFINE_double(vertical_force, 0, "fixed vertical force");

// Parameters for shaking the mug.
DEFINE_double(x_force_amplitude, 4.0, "The amplitude of the sine force [N].");
DEFINE_double(x_force_frequency, 1.0, "The frequency of the sine force [Hz].");

const double kGravity = 9.8; /* m/s^2 */

double CalcFrictionFromVelocity(double velocity) {
  using std::sqrt;
  using std::min;
  using std::max;
  double rel_tolerance = 0.01; /* from implicit stribeck solver code */
  double eps = rel_tolerance * FLAGS_v_stiction_tolerance;
  double v_t = velocity;
  double v_t_eps = sqrt( v_t * v_t + eps * eps ) ;
  double x = v_t_eps / FLAGS_v_stiction_tolerance;
  double mu = 1.0 * max(min(x, 1.0), x * (2.0 - x));
  return - mu * v_t / v_t_eps * 0.33 * kGravity; /* MASS FROM FILE */
}
void StoreEigenCSV(const std::string& filename, const Eigen::VectorXd& times, const Eigen::MatrixXd& data,
                const Eigen::MatrixXi& metadata) {
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
    sim_friction(i) = CalcFrictionFromVelocity(sim_velocity);
    truth_friction(i) = CalcFrictionFromVelocity(truth_velocity);
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

int do_main() {
  systems::DiagramBuilder<double> builder;


  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  DRAKE_DEMAND(FLAGS_max_time_step > 0);

  MultibodyPlant<double>& plant =
      FLAGS_use_discrete_states ?
      *builder.AddSystem<MultibodyPlant>(FLAGS_max_time_step) :
      *builder.AddSystem<MultibodyPlant>();
  
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  Parser parser(&plant);
  std::string full_name;

  if(FLAGS_use_friction)
  {
    full_name = FindResourceOrThrow("drake/examples/box2d/box2d.sdf");
  }
  else
  {
    full_name = FindResourceOrThrow("drake/examples/box2d/box2d_no_friction.sdf");
  }
  parser.AddModelFromFile(full_name);

  /* const PrismaticJoint<double>& translate_joint =
      plant.GetJointByName<PrismaticJoint>("x_translate_joint");
  const Vector3d axis = translate_joint.translation_axis(); */
  /* use -9.8 m/s^2 z-hat for gravity */
  plant.mutable_gravity_field().set_gravity_vector(Vector3d(0.0, 0.0, -kGravity));

  // Add the pads.
/*  const Body<double>& left_finger = plant.GetBodyByName("left_finger");
  const Body<double>& right_finger = plant.GetBodyByName("right_finger"); */
  if (FLAGS_use_hydroelastics_model) {
    plant.use_hydroelastic_model();
    
    const auto& moving_box = plant.GetBodyByName("moving_box");
    const std::vector<geometry::GeometryId>& collision_geos = plant.GetCollisionGeometriesForBody(moving_box);
    for( auto geo_id : collision_geos)
    {
      plant.set_elastic_modulus(geo_id, FLAGS_elastic_modulus);
      plant.set_hydroelastics_dissipation(geo_id, FLAGS_dissipation);
    }
    
  }
  plant.use_linear_friction(FLAGS_use_linear_friction);
  // Now the model is complete.
  plant.Finalize();

  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction_tolerance);

  // If the user specifies a time step, we use that, otherwise estimate a
  // maximum time step based on the compliance of the contact model.
  // The maximum time step is estimated to resolve this time scale with at
  // least 30 time steps. Usually this is a good starting point for fixed step
  // size integrators to be stable.
  const double max_time_step =
      FLAGS_max_time_step > 0 ? FLAGS_max_time_step :
      plant.get_contact_penalty_method_time_scale() / 30;

  // Print maximum time step and the time scale introduced by the compliance in
  // the contact model as a reference to the user.
  fmt::print("Maximum time step = {:10.6f} s\n", max_time_step);
  fmt::print("Compliance time scale = {:10.6f} s\n",
             plant.get_contact_penalty_method_time_scale());

  // from simple_griper.sdf, there are two actuators. One actuator on the
  // prismatic joint named "finger_sliding_joint" to actuate the left finger and
  // a second actuator on the prismatic joint named "translate_joint" to impose
  // motions of the gripper.
  DRAKE_DEMAND(plant.num_actuators() == 2);
  DRAKE_DEMAND(plant.num_actuated_dofs() == 2);

    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  
  DrakeLcm lcm;
  if(FLAGS_visualize)
  {
    geometry::ConnectDrakeVisualizer(&builder, scene_graph, &lcm);
    // Publish contact results for visualization.
    // (Currently only available when time stepping.)
    if (FLAGS_use_discrete_states)
      ConnectContactResultsToDrakeVisualizer(&builder, plant, &lcm);
  }
  // Sinusoidal force input. We want the gripper to follow a trajectory of the
  // form x(t) = X0 * sin(ω⋅t). By differentiating once, we can compute the
  // velocity initial condition, and by differentiating twice, we get the input
  // force we need to apply.
  // The mass of the mug is ignored.
  // TODO(amcastro-tri): add a PD controller to precisely control the
  // trajectory of the gripper. Even better, add a motion constraint when MBP
  // supports it.

  // Notice we are using the same Sine source to:
  //   1. Generate a harmonic forcing of the box
  //   2. Impose a constant vertical force on the box
  const Vector2<double> amplitudes(FLAGS_vertical_force, FLAGS_x_force_amplitude);
  const Vector2<double> frequencies(0.0, 2.0 * M_PI * FLAGS_x_force_frequency);
  const Vector2<double> phases(M_PI_2, M_PI_2);
  const auto& harmonic_force = *builder.AddSystem<Sine>(
      amplitudes, frequencies, phases);

  builder.Connect(harmonic_force.get_output_port(0),
                  plant.get_actuation_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // get box height joint
  const PrismaticJoint<double>& vertical_translation =
      plant.GetJointByName<PrismaticJoint>("vertical_dof");

  // Set initial box height.
  vertical_translation.set_translation(&plant_context, FLAGS_box_z0);
  vertical_translation.set_translation_rate(&plant_context, FLAGS_box_w0);
  
  // get x dof joint
  const PrismaticJoint<double>& x_translation =
      plant.GetJointByName<PrismaticJoint>("x_translate_dof");

  // Set initial box x position
  x_translation.set_translation(&plant_context, FLAGS_box_x0);
  x_translation.set_translation_rate(&plant_context, FLAGS_box_v0);
  
  // Set up simulators.
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  systems::Simulator<double> truth_simulator(*diagram);
  systems::Context<double>& truth_context = truth_simulator.get_mutable_context();
  truth_context.get_mutable_state().SetFrom(simulator.get_context().get_state());

  systems::IntegratorBase<double>* integrator{nullptr};
  systems::IntegratorBase<double>* truth_integrator{nullptr}; 


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
    static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_reuse(!FLAGS_full_newton);

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
  } else if (FLAGS_integration_scheme == "fixed_implicit_euler" || FLAGS_integration_scheme == "radau1") {
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
    static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_reuse(!FLAGS_full_newton);
  } else if (FLAGS_integration_scheme == "radau" || FLAGS_integration_scheme == "radau3") {
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
    static_cast<systems::ImplicitIntegrator<double>*>(integrator)->set_reuse(!FLAGS_full_newton);
  } else {
    throw std::runtime_error(
        "Integration scheme '" + FLAGS_integration_scheme +
            "' not supported for this example.");
  }

      // Set the iteration limiter method.
  std::unique_ptr<systems::Context<double>> scratch_ctx_k = plant.CreateDefaultContext();
  std::unique_ptr<systems::Context<double>> scratch_ctx_kp1 = plant.CreateDefaultContext();
    auto iteration_limiter = [&diagram, &plant, &scratch_ctx_k, &scratch_ctx_kp1](const systems::Context<double>& ctx0,
    const systems::ContinuousState<double>& x_k, const systems::ContinuousState<double>& x_kp1) -> double {
         const systems::Context<double>& plant_ctx0 = diagram->GetSubsystemContext(plant, ctx0);
         /* this method is very poorly named but gets the subsystem continuous state */
         Eigen::VectorXd v_k = diagram->GetSubsystemDerivatives(plant, x_k).get_generalized_velocity().CopyToVector();
         Eigen::VectorXd v_kp1 = diagram->GetSubsystemDerivatives(plant, x_kp1).get_generalized_velocity().CopyToVector();
         return plant.CalcIterationLimiterAlpha(plant_ctx0, v_k, v_kp1, scratch_ctx_k.get(), scratch_ctx_kp1.get());
      };
  if(FLAGS_iteration_limit && !FLAGS_use_discrete_states)
  {
    integrator->set_iteration_limiter(iteration_limiter);
  } 
  integrator->set_maximum_step_size(FLAGS_max_time_step);
  if (integrator->supports_error_estimation())
    integrator->set_fixed_step_mode( FLAGS_fixed_step );
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);

  integrator->set_convergence_control(FLAGS_convergence_control);
  if (FLAGS_visualize )
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
  /* TODO ANTE: This is temporary */
  nstate = 2;
  int nmetadata = 3; /* der evals for sim, num steps for sim, num steps for truth */
  Eigen::VectorXd times = Eigen::VectorXd::Zero(nsteps+1);
  Eigen::MatrixXd error_results = Eigen::MatrixXd::Zero(nsteps+1, 2 * nstate );
  Eigen::MatrixXi error_meta = Eigen::MatrixXi::Zero(nsteps+1, nmetadata);
  double time = 0;
  int progress_out_rate = nsteps / 25;
  // TODO: NEED TO STORE STATE AT TIME ZERO
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

    const systems::Context<double>& sim_plant_context =
      diagram->GetSubsystemContext(plant, simulator.get_context());
    const systems::Context<double>& truth_plant_context =
      diagram->GetSubsystemContext(plant, truth_simulator.get_context());
    auto simstate = plant.GetPositionsAndVelocities(sim_plant_context);
    auto truthstate = plant.GetPositionsAndVelocities(truth_plant_context);

    times(next_step_ind) = next_time;

    error_results(next_step_ind , 0) = simstate[1] ; /* x translation */
    error_results(next_step_ind , 1) = simstate[3] ; /* x velocity */

    error_results(next_step_ind , 2) = truthstate[1] ; /* x translation */
    error_results(next_step_ind , 3) = truthstate[3] ; /* x velocity */

    error_meta(next_step_ind , 0) = integrator->get_num_derivative_evaluations();
    error_meta(next_step_ind , 1) = integrator->get_num_steps_taken();
    error_meta(next_step_ind , 2) = truth_integrator->get_num_steps_taken();
    if(next_step_ind % progress_out_rate == 0)
    {
      std::stringstream to_out;
      to_out << FLAGS_integration_scheme << (FLAGS_fixed_step ? " fixed, " : " ec, ") << FLAGS_max_time_step << ": " << next_time << " s.";
      std::cout << to_out.str() << std::endl;
    }
    
  }

  if (FLAGS_use_discrete_states) {
    fmt::print("Used time stepping with dt={}\n", FLAGS_max_time_step);
    fmt::print("Number of time steps taken = {:d}\n",
               simulator.get_num_steps_taken());
    fmt::print("Number of derivative evals = {:d}\n",
               integrator->get_num_derivative_evaluations());
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
  // get plant state vector
  std::cout << "\nZ Height: " << vertical_translation.get_translation(plant_context) << " m" << std::endl;
  std::cout << "\nX Translation: " << x_translation.get_translation(plant_context) << " m" << std::endl;
  std::cout << "\nPosition states: \n" << plant.GetPositions(plant_context) << std::endl;
  std::cout << "\nZ Velocity: " << vertical_translation.get_translation_rate(plant_context) << " m/s" << std::endl;
  std::cout << "\nX Velocity: " << x_translation.get_translation_rate(plant_context) << " m/s" << std::endl;
  std::cout << "\nVelocity states: \n" << plant.GetVelocities(plant_context) << std::endl;
  
  std::cout << "\nState vector: \n" <<  plant.GetPositionsAndVelocities(plant_context) << std::endl;
  std::cout << "\nNumber of input ports: " << plant.num_input_ports() << std::endl;
  std::cout << "\nActuation input vector: \n" << plant.get_actuation_input_port().Eval(plant_context) << std::endl;


  StoreEigenCSV(FLAGS_errors_filename + ".csv", times, error_results, error_meta);
  std::ofstream file(FLAGS_meta_filename + ".csv");
  file << "steps, duration, max_dt, num_der_eval\n";
  file << integrator->get_num_steps_taken() << ", " << FLAGS_simulation_time << ", " << FLAGS_max_time_step << ", " << integrator->get_num_derivative_evaluations() << std::endl;
  file.close();
  return 0;
}

}  // namespace
}  // namespace box2d
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Demo used to exercise MultibodyPlant's contact modeling in a gripping "
      "scenario. SceneGraph is used for both visualization and contact "
      "handling. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::box2d::do_main();
}
