#pragma once

#include <string>
#include <vector>

#include "drake/common/yaml/yaml_io.h"

namespace drake {
namespace traj_opt {
namespace examples {

/**
 * A simple object which stores parameters that define an optimization problem
 * and various options, and can be loaded from a YAML file.
 *
 * See, e.g., spinner.yaml for an explanation of each field, and
 * https://drake.mit.edu/doxygen_cxx/group__yaml__serialization.html for details
 * on loading options from YAML.
 */
struct TrajOptExampleParams {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(q_init));
    a->Visit(DRAKE_NVP(v_init));
    a->Visit(DRAKE_NVP(q_nom));
    a->Visit(DRAKE_NVP(v_nom));
    a->Visit(DRAKE_NVP(q_guess));
    a->Visit(DRAKE_NVP(Qq));
    a->Visit(DRAKE_NVP(Qv));
    a->Visit(DRAKE_NVP(R));
    a->Visit(DRAKE_NVP(Qfq));
    a->Visit(DRAKE_NVP(Qfv));
    a->Visit(DRAKE_NVP(time_step));
    a->Visit(DRAKE_NVP(num_steps));
    a->Visit(DRAKE_NVP(max_iters));
    a->Visit(DRAKE_NVP(linesearch));
    a->Visit(DRAKE_NVP(gradients_method));
    a->Visit(DRAKE_NVP(method));
    a->Visit(DRAKE_NVP(proximal_operator));
    a->Visit(DRAKE_NVP(rho_proximal));
    a->Visit(DRAKE_NVP(play_optimal_trajectory));
    a->Visit(DRAKE_NVP(play_initial_guess));
    a->Visit(DRAKE_NVP(linesearch_plot_every_iteration));
    a->Visit(DRAKE_NVP(print_debug_data));
    a->Visit(DRAKE_NVP(save_solver_stats_csv));
    a->Visit(DRAKE_NVP(F));
    a->Visit(DRAKE_NVP(delta));
    a->Visit(DRAKE_NVP(stiffness_exponent));
    a->Visit(DRAKE_NVP(dissipation_velocity));
    a->Visit(DRAKE_NVP(dissipation_exponent));
    a->Visit(DRAKE_NVP(stiction_velocity));
    a->Visit(DRAKE_NVP(friction_coefficient));
    a->Visit(DRAKE_NVP(save_contour_data));
    a->Visit(DRAKE_NVP(contour_q1_min));
    a->Visit(DRAKE_NVP(contour_q1_max));
    a->Visit(DRAKE_NVP(contour_q2_min));
    a->Visit(DRAKE_NVP(contour_q2_max));
    a->Visit(DRAKE_NVP(save_lineplot_data));
    a->Visit(DRAKE_NVP(lineplot_q_min));
    a->Visit(DRAKE_NVP(lineplot_q_max));
  }
  std::vector<double> q_init;
  std::vector<double> v_init;
  std::vector<double> q_nom;
  std::vector<double> v_nom;
  std::vector<double> q_guess;
  std::vector<double> Qq;
  std::vector<double> Qv;
  std::vector<double> R;
  std::vector<double> Qfq;
  std::vector<double> Qfv;
  double time_step;
  int num_steps;
  int max_iters;
  std::string linesearch;
  std::string method;
  std::string gradients_method{"forward_differences"};
  bool proximal_operator = false;
  double rho_proximal = 1e-8;
  bool play_optimal_trajectory = true;
  bool play_initial_guess = true;
  bool linesearch_plot_every_iteration = false;
  bool print_debug_data = false;
  bool save_solver_stats_csv = true;
  double F = 1.0;
  double delta = 0.01;
  double stiffness_exponent = 2;
  double dissipation_velocity = 0.1;
  double dissipation_exponent = 1.0;
  double stiction_velocity = 0.05;
  double friction_coefficient = 0.0;
  bool save_contour_data = false;
  double contour_q1_min = 0;
  double contour_q1_max = 1;
  double contour_q2_min = 0;
  double contour_q2_max = 1;
  bool save_lineplot_data = false;
  double lineplot_q_min = 0;
  double lineplot_q_max = 1;
};

}  // namespace examples
}  // namespace traj_opt
}  // namespace drake
