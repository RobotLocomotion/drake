#pragma once

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/traj_opt/examples/yaml_config.h"
#include "drake/traj_opt/problem_definition.h"
#include "drake/traj_opt/trajectory_optimizer.h"

namespace drake {
namespace traj_opt {
namespace examples {

using geometry::DrakeVisualizerd;
using geometry::SceneGraph;
using multibody::AddMultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::Parser;
using systems::DiagramBuilder;

/**
 * Abstract base class for trajectory optimization examples.
 */
class TrajOptExample {
 public:
  virtual ~TrajOptExample() = default;

  /**
   * Solve the optimizaiton problem, as defined by the parameters in the given
   * YAML file.
   *
   * @param options_file YAML file containing cost funciton definition, solver
   * parameters, etc., with fields as defined in yaml_config.h.
   */
  void SolveTrajectoryOptimization(const std::string options_file) const {
    // Load parameters from file
    TrajOptExampleParams options = yaml::LoadYamlFile<TrajOptExampleParams>(
        FindResourceOrThrow(options_file));

    // Create a system model
    // N.B. we need a whole diagram, including scene_graph, to handle contact
    DiagramBuilder<double> builder;
    MultibodyPlantConfig config;
    config.time_step = options.time_step;
    auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
    CreatePlantModel(&plant);
    plant.Finalize();
    const int nq = plant.num_positions();
    const int nv = plant.num_positions();

    auto diagram = builder.Build();
    std::unique_ptr<systems::Context<double>> diagram_context =
        diagram->CreateDefaultContext();
    systems::Context<double>& plant_context =
        diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    // Set up an optimization problem
    ProblemDefinition opt_prob;
    opt_prob.num_steps = options.num_steps;
    opt_prob.q_init = Eigen::Map<VectorXd>(options.q_init.data(), nq);
    opt_prob.v_init = Eigen::Map<VectorXd>(options.v_init.data(), nv);
    opt_prob.Qq = Eigen::Map<VectorXd>(options.Qq.data(), nq).asDiagonal();
    opt_prob.Qv = Eigen::Map<VectorXd>(options.Qv.data(), nv).asDiagonal();
    opt_prob.Qf_q = Eigen::Map<VectorXd>(options.Qfq.data(), nq).asDiagonal();
    opt_prob.Qf_v = Eigen::Map<VectorXd>(options.Qfv.data(), nv).asDiagonal();
    opt_prob.R = Eigen::Map<VectorXd>(options.R.data(), nv).asDiagonal();
    opt_prob.q_nom = Eigen::Map<VectorXd>(options.q_nom.data(), nq);
    opt_prob.v_nom = Eigen::Map<VectorXd>(options.v_nom.data(), nv);

    // Set our solver parameters
    // TODO(vincekurtz): consider separate functions mapping options to opt_prob
    // and mapping options to solver_params
    SolverParameters solver_params;

    if (options.linesearch == "backtracking") {
      solver_params.linesearch_method = LinesearchMethod::kBacktracking;
    } else if (options.linesearch == "armijo") {
      solver_params.linesearch_method = LinesearchMethod::kArmijo;
    } else {
      throw std::runtime_error(
          fmt::format("Unknown linesearch method '{}'", options.linesearch));
    }

    if (options.method == "linesearch") {
      solver_params.method = SolverMethod::kLinesearch;
    } else if (options.method == "trust_region") {
      solver_params.method = SolverMethod::kTrustRegion;
    } else {
      throw std::runtime_error(
          fmt::format("Unknown solver method '{}'", options.method));
    }

    solver_params.max_iterations = options.max_iters;
    solver_params.max_linesearch_iterations = 60;
    solver_params.print_debug_data = options.print_debug_data;
    solver_params.linesearch_plot_every_iteration =
        options.linesearch_plot_every_iteration;

    solver_params.proximal_operator = options.proximal_operator;
    solver_params.rho_proximal = options.rho_proximal;

    // Set contact parameters
    // TODO(vincekurtz): figure out a better place to set these
    solver_params.F = options.F;
    solver_params.delta = options.delta;
    solver_params.n = options.n;

    // Set parameters for making contour plot of the first two variables
    solver_params.save_contour_data = options.save_contour_data;
    solver_params.contour_q1_min = options.contour_q1_min;
    solver_params.contour_q1_max = options.contour_q1_max;
    solver_params.contour_q2_min = options.contour_q2_min;
    solver_params.contour_q2_max = options.contour_q2_max;

    // Parameters for making line plots of the first variable
    solver_params.save_lineplot_data = options.save_lineplot_data;
    solver_params.lineplot_q_min = options.lineplot_q_min;
    solver_params.lineplot_q_max = options.lineplot_q_max;

    // Establish an initial guess
    const VectorXd qT_guess = Eigen::Map<VectorXd>(options.q_guess.data(), nq);
    std::vector<VectorXd> q_guess;
    double lambda = 0;
    for (int t = 0; t <= options.num_steps; ++t) {
      lambda = (1.0 * t) / (1.0 * options.num_steps);
      q_guess.push_back((1 - lambda) * opt_prob.q_init + lambda * qT_guess);
    }
    if (options.play_initial_guess) {
      PlayBackTrajectory(q_guess, options.time_step);
    }

    // Solve the optimzation problem
    TrajectoryOptimizer<double> optimizer(&plant, &plant_context, opt_prob,
                                          solver_params);

    TrajectoryOptimizerSolution<double> solution;
    TrajectoryOptimizerStats<double> stats;
    SolverFlag status = optimizer.Solve(q_guess, &solution, &stats);
    if (status != SolverFlag::kSuccess) {
      std::cout << "Solver failed!" << std::endl;
    } else {
      std::cout << "Solved in " << stats.solve_time << " seconds." << std::endl;
    }

    // Report maximum torques on all DoFs
    VectorXd tau_max = VectorXd::Zero(nv);
    VectorXd abs_tau_t = VectorXd::Zero(nv);
    for (int t = 0; t < options.num_steps; ++t) {
      abs_tau_t = solution.tau[t].cwiseAbs();
      for (int i = 0; i < nv; ++i) {
        if (abs_tau_t(i) > tau_max(i)) {
          tau_max(i) = abs_tau_t(i);
        }
      }
    }
    std::cout << std::endl;
    std::cout << "Max torques: " << tau_max.transpose() << std::endl;

    // Report desired and final state
    std::cout << std::endl;
    std::cout << "q_nom : " << opt_prob.q_nom.transpose() << std::endl;
    std::cout << "q[T]  : " << solution.q[options.num_steps].transpose()
              << std::endl;
    std::cout << std::endl;
    std::cout << "v_nom : " << opt_prob.v_nom.transpose() << std::endl;
    std::cout << "v[T]  : " << solution.v[options.num_steps].transpose()
              << std::endl;

    // Save stats to CSV for later plotting
    if (options.save_solver_stats_csv) {
      stats.SaveToCsv("solver_stats.csv");
    }

    // Play back the result on the visualizer
    if (options.play_optimal_trajectory) {
      PlayBackTrajectory(solution.q, options.time_step);
    }
  }

 private:
  /**
   * Create a MultibodyPlant model of the system that we're optimizing over.
   * This is the only method that needs to be overwritten to specialize to
   * different systems.
   *
   * @param plant the MultibodyPlant that we'll add the system to.
   */
  virtual void CreatePlantModel(MultibodyPlant<double>*) const {}

  /**
   * Play back the given trajectory on the Drake visualizer
   *
   * @param q sequence of generalized positions defining the trajectory
   * @param time_step time step (seconds) for the discretization
   */
  void PlayBackTrajectory(const std::vector<VectorXd>& q,
                          const double time_step) const {
    // Create a system diagram that includes the plant and is connected to
    // the Drake visualizer
    DiagramBuilder<double> builder;
    MultibodyPlantConfig config;
    config.time_step = time_step;

    auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);
    CreatePlantModel(&plant);
    plant.Finalize();

    DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

    auto diagram = builder.Build();
    std::unique_ptr<systems::Context<double>> diagram_context =
        diagram->CreateDefaultContext();
    systems::Context<double>& plant_context =
        diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    const VectorXd u = VectorXd::Zero(plant.num_actuators());
    plant.get_actuation_input_port().FixValue(&plant_context, u);

    // Step through q, setting the plant positions at each step accordingly
    const int N = q.size();
    for (int t = 0; t < N; ++t) {
      diagram_context->SetTime(t * time_step);
      plant.SetPositions(&plant_context, q[t]);
      diagram->Publish(*diagram_context);

      // Hack to make the playback roughly realtime
      // TODO(vincekurtz): add realtime rate option?
      std::this_thread::sleep_for(std::chrono::duration<double>(time_step));
    }
  }
};

}  // namespace examples
}  // namespace traj_opt
}  // namespace drake
