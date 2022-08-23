#pragma once

namespace drake {
namespace traj_opt {

enum LinesearchMethod {
  // Simple backtracking linesearch with Armijo's condition
  kArmijo,

  // Backtracking linesearch that tries to find a local minimum
  kBacktracking
};

enum SolverMethod {
  kLinesearch,
  kTrustRegion
};

struct SolverParameters {
  // Which overall optimization strategy to use - linesearch or trust region
  // TODO(vincekurtz): better name for this?
  SolverMethod method = SolverMethod::kTrustRegion;

  // Which linesearch strategy to use
  LinesearchMethod linesearch_method = LinesearchMethod::kArmijo;

  // Maximum number of Gauss-Newton iterations
  int max_iterations = 100;

  // Maximum number of linesearch iterations
  int max_linesearch_iterations = 50;

  // Flag for whether to print out iteration data
  bool verbose = true;

  // Flag for whether to print (and compute) additional slow-to-compute
  // debugging info, like the condition number, at each iteration
  bool print_debug_data = false;

  // Flag for whether to record linesearch data to a file at each iteration (for
  // later plotting). This saves a file called "linesearch_data_[k].csv" for
  // each iteration, where k is the iteration number. This file can then be
  // found somewhere in drake/bazel-out/.
  bool linesearch_plot_every_iteration = false;

  // Flag for whether to add a proximal operator term,
  //
  //      1/2 * rho * (q_k - q_{k-1})' * diag(H) * (q_k - q_{k-1})
  //
  // to the cost, where q_{k-1} are the decision variables at iteration {k-1}
  // and H_{k-1} is the Hessian at iteration k-1.
  bool proximal_operator = false;

  // Scale factor for the proximal operator cost
  double rho_proximal = 1e-8;

  // Contact model parameters
  // TODO(vincekurtz): this is definitely the wrong place to specify the contact
  // model - figure out the right place and put these parameters there
  double F = 1.0;       // force (in Newtons) at delta meters of penetration
  double delta = 0.01;  // penetration distance at which we apply F newtons
  double n = 2;         // polynomial scaling factor
};

}  // namespace traj_opt
}  // namespace drake
