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
    a->Visit(DRAKE_NVP(n));
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
  bool proximal_operator;
  double rho_proximal;
  bool play_optimal_trajectory;
  bool play_initial_guess;
  bool linesearch_plot_every_iteration;
  bool print_debug_data;
  bool save_solver_stats_csv;
  double F;
  double delta;
  double n;
  bool save_contour_data;
  double contour_q1_min;
  double contour_q1_max;
  double contour_q2_min;
  double contour_q2_max;
  bool save_lineplot_data;
  double lineplot_q_min;
  double lineplot_q_max;
};

}  // namespace examples
}  // namespace traj_opt
}  // namespace drake
