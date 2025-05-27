#include "drake/systems/analysis/convex_integrator.h"

#include "drake/multibody/contact_solvers/newton_with_bisection.h"

namespace drake {
namespace systems {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using multibody::contact_solvers::internal::Bracket;
using multibody::contact_solvers::internal::DoNewtonWithBisectionFallback;

template <typename T>
ConvexIntegrator<T>::ConvexIntegrator(const System<T>& system,
                                      Context<T>* context)
    : IntegratorBase<T>(system, context) {}

template <typename T>
ConvexIntegrator<T>::ConvexIntegrator(const System<T>& system,
                                      MultibodyPlant<T>* plant,
                                      Context<T>* context)
    : IntegratorBase<T>(system, context) {
  set_plant(plant);
}

template <typename T>
void ConvexIntegrator<T>::DoInitialize() {
  using std::isnan;
  if (plant_ == nullptr) {
    throw std::runtime_error(
        "ConvexIntegrator: MultibodyPlant not set. You must either use the "
        "constructor that specifies a plant, or call ConvexIntegrator::"
        "set_plant() before initialization.");
  }

  // Get the plant context from the overall context. This will throw if plant()
  // is not part of the system diagram.
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);

  // For now, the convex integrator only supports systems where the only
  // second-order state (q, v) is from the MultibodyPlant.
  const int nq = this->get_context().get_continuous_state().num_q();
  const int nv = this->get_context().get_continuous_state().num_v();
  DRAKE_THROW_UNLESS(nq == plant().num_positions());
  DRAKE_THROW_UNLESS(nv == plant().num_velocities());

  // Allocate and initialize SAP problem objects
  // TODO(vincekurtz): don't hardcode the initial time step.
  builder_ = std::make_unique<PooledSapBuilder<T>>(plant());
  builder_->UpdateModel(plant_context, 0.01, &model_);
  model_.ResizeData(&data_);

  // Allocate memory for the solver statistics.
  stats_.Reserve(solver_parameters_.max_iterations);

  // Set up the CSV file and write a header, if logging is enabled.
  if (log_solver_stats_) {
    log_file_.open("convex_integrator_stats.csv");
    log_file_
        << "time,iteration,cost,gradient_norm,ls_iterations,alpha,step_size\n";
  }
}

template <typename T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  // Get plant context storing initial state [q₀, v₀].
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);

  // Set up the convex optimization problem minᵥ ℓ(v; q₀, v₀, h)
  PooledSapModel<T>& model = get_model();
  builder().UpdateModel(plant_context, h, &model);

  // The initial guess is the current velocity v₀.
  // TODO(vincekurtz): pre-allocate v
  VectorX<T> v = plant().GetVelocities(plant_context);

  // Solve the optimization problem for next-step velocities v = min ℓ(v).
  if (!SolveWithGuess(model, &v)) {
    throw std::runtime_error("ConvexIntegrator: optimization failed.");
  }

  // Log and print solver statistics, as requested.
  if (print_solver_stats_) PrintSolverStats();
  if (log_solver_stats_) LogSolverStats();
  total_solver_iterations_ += stats_.iterations + 1;  // zero-indexed
  total_ls_iterations_ += std::accumulate(stats_.ls_iterations.begin(),
                                          stats_.ls_iterations.end(), 0);

  // Advance configurations q = q₀ + h N(q₀) v
  // TODO(vincekurtz): pre-allocate q
  VectorX<T> q(plant().num_positions());
  plant().MapVelocityToQDot(plant_context, h * v, &q);
  q += plant().GetPositions(plant_context);

  // Set the updated state and time
  // TODO(vincekurtz): update the non-plant states z.
  Context<T>& mutable_context = *this->get_mutable_context();
  Context<T>& mutable_plant_context =
      plant().GetMyMutableContextFromRoot(&mutable_context);

  plant().SetPositions(&mutable_plant_context, q);
  plant().SetVelocities(&mutable_plant_context, v);

  mutable_context.SetTime(mutable_context.get_time() + h);

  return true;
}

template <typename T>
bool ConvexIntegrator<T>::SolveWithGuess(const PooledSapModel<T>&,
                                         VectorX<T>*) {
  // Eventually we could propagate gradients with the implicit function theorem
  // to support AutoDiffXd. For now we'll throw if anything other than double is
  // used.
  throw std::logic_error("ConvexIntegrator only supports T = double.");
}

template <>
bool ConvexIntegrator<double>::SolveWithGuess(
    const PooledSapModel<double>& model, VectorXd* v_guess) {
  SapData<double>& data = get_data();
  VectorXd& v = *v_guess;
  model.ResizeData(&data);
  DRAKE_DEMAND(data.num_velocities() == v.size());

  // Tolerance for a more relaxed convergence check under looser accuracies.
  const double k_dot_tol =
      solver_parameters_.kappa * this->get_accuracy_in_use();

  // TODO(vincekurtz): pre-allocate the search direction data
  VectorXd dv(v.size());
  double alpha{NAN};
  int ls_iterations{0};

  stats_.Reset();
  for (int k = 0; k < solver_parameters_.max_iterations; ++k) {
    // Compute the cost, gradient, and Hessian.
    // TODO(vincekurtz): re-use the old Hessian
    model.CalcData(v, &data);

    // Update the statistics we have available so far
    stats_.iterations = k;
    stats_.cost.push_back(data.cache().cost);
    stats_.gradient_norm.push_back(data.cache().gradient.norm());

    // Early convergence check. Allows for early exit if v_guess is already
    // optimal, or a single Newton step for simple unconstrained problems. This
    // is necessary because our main convergence criterion requires comparing dv
    // over several iterations.
    if (stats_.gradient_norm.back() < solver_parameters_.tolerance) {
      // TODO(vincekurtz): consider using the SAP momentum residual rather than
      // the gradient norm.
      stats_.ls_iterations.push_back(0);
      stats_.alpha.push_back(NAN);
      stats_.step_size.push_back(NAN);
      return true;
    }

    // Compute the search direction via Newton step dv = -H⁻¹ g
    const VectorXd& g = data.cache().gradient;
    const MatrixXd H = data.cache().hessian.MakeDenseMatrix();
    dv = H.ldlt().solve(-g);

    // Compute the step size with linesearch
    PerformExactLineSearch(model, v, dv, &alpha, &ls_iterations);

    // Update the decision variables (velocities)
    dv *= alpha;
    v += dv;

    // Log the remaining solver statistics.
    stats_.ls_iterations.push_back(ls_iterations);
    stats_.alpha.push_back(alpha);
    stats_.step_size.push_back(dv.norm());

    // Convergence check from on [Hairer, 1996], Eq. IV.8.10.
    if (k > 0) {
      // N.B. this only comes into effect in the second iteration, since we need
      // both ||Δvₖ|| and ||Δvₖ₋₁|| to compute θ = ||Δvₖ|| / ||Δvₖ₋₁||.
      const double theta = stats_.step_size[k] / stats_.step_size[k - 1];
      const double eta = theta / (1.0 - theta);
      if ((theta < 1.0) && (eta < k_dot_tol)) {
        return true;
      }

      // TODO(vincekurtz): use theta to trigger hessian re-computation.
    }
  }

  return false;  // Failed to converge.
}

template <typename T>
void ConvexIntegrator<T>::PerformExactLineSearch(const PooledSapModel<T>&,
                                                 const VectorX<T>&,
                                                 const VectorX<T>&, T*, int*) {
  throw std::logic_error(
      "ConvexIntegrator: PerformExactLineSearch only supports T = double.");
}

template <>
void ConvexIntegrator<double>::PerformExactLineSearch(
    const PooledSapModel<double>& model, const VectorXd& v, const VectorXd& dv,
    double* alpha_ptr, int* num_iterations_ptr) {
  SapData<double>& data = get_data();

  // Initialize the step size and number of iterations.
  double& alpha = *alpha_ptr;
  int& num_iterations = *num_iterations_ptr;
  alpha = solver_parameters_.alpha_max;
  num_iterations = 0;

  double dell_dalpha{NAN};
  double d2ell_dalpha2{NAN};

  // First we'll evaluate ∂ℓ/∂α at α = 0. This should be strictly negative,
  // since the Hessian is positive definite.
  // N.B. we already have the gradient at α = 0 cached from solving for the
  // search direction earlier.
  const double ell0 = data.cache().cost;
  const double dell_dalpha0 = data.cache().gradient.dot(dv);
  if (dell_dalpha >= 0) {
    throw std::logic_error(
        "ConvexIntegrator: the cost does not decrease along the search "
        "direction. This is usually caused by an excessive accumulation of "
        "round-off errors for ill-conditioned systems. Consider revisiting "
        "your model.");
  }

  // N.B. we'll use this for normalization later.
  const double dell_scale = -dell_dalpha0;

  // Next we'll evaluate ℓ, ∂ℓ/∂α, and ∂²ℓ/∂α² at α = α_max. If the cost is
  // still decreasing here, we just accept α_max.
  const double ell = model.CalcCostAlongLine(v, dv, alpha, &data, &dell_dalpha,
                                             &d2ell_dalpha2);
  if (dell_dalpha <= 0) {
    return;  // α = α_max and num_iterations = 0 are set above.
  }

  // TODO(vincekurtz): add a check to enable full Newton steps very close to
  // machine epsilon, as in SapSolver. We'll need to be mindful of the fact that
  // the cost can be negative in the pooled SAP formulation.

  // Set the initial guess for linesearch based on a cubic hermite spline
  // between α = 0 and α = α_max. This spline takes the form 
  // p(t) = a t³ + b t² + c t + d, where p(0) = ℓ(0), p(1) = ℓ(α_max).
  // TODO(vincekurtz): deal with the case of α_max != 1 properly
  const double a = 2 * ell0 - 2 * ell + dell_dalpha0 + dell_dalpha;
  const double b = -3 * ell0 + 3 * ell - 2 * dell_dalpha0 - dell_dalpha;
  const double c = dell_dalpha0;
  double alpha_guess = (- 2*b + std::sqrt(4*b*b - 12*a*c)) / (6 * a);
  DRAKE_DEMAND(alpha_guess >= 0);
  alpha_guess = std::min(alpha_guess, solver_parameters_.alpha_max);

  // We've exhausted all of the early exit conditions, so now we move on to the
  // Newton method with bisection fallback. To do so, we define an anonymous
  // function that computes the value and gradient of f(α) = −ℓ'(α)/ℓ'₀.
  // Normalizing in this way reduces round-off errors, ensuring f(0) = -1.
  auto cost_and_gradient = [&model, &data, &v, &dv, &dell_scale](double x) {
    double dell;
    double d2ell;
    model.CalcCostAlongLine(v, dv, x, &data, &dell, &d2ell);
    return std::make_pair(dell / dell_scale, d2ell / dell_scale);
  };

  // The initial bracket is [0, α_max], since we already know that ℓ'(0) < 0 and
  // ℓ'(α_max) > 0. Values at the endpoints of the bracket are f(0) = -1 (by
  // definition) and f(α_max) = dell_dalpha / dell_scale, because we just
  // computed dell_dalpha = ℓ'(α_max) above.
  const Bracket bracket(0.0, -1.0, alpha, dell_dalpha / dell_scale);

  // TODO(vincekurtz): scale linesearch tolerance based on accuracy.
  const double alpha_tolerance = solver_parameters_.ls_tolerance * alpha_guess;
  std::tie(alpha, num_iterations) = DoNewtonWithBisectionFallback(
      cost_and_gradient, bracket, alpha_guess, alpha_tolerance,
      solver_parameters_.ls_tolerance, solver_parameters_.max_iterations);
}

template <typename T>
void ConvexIntegrator<T>::PrintSolverStats() const {
  fmt::print("ConvexIntegrator: {} iters\n", stats_.iterations + 1);
  for (int k = 0; k <= stats_.iterations; ++k) {
    fmt::print(
        "  Iteration {}: cost = {}, gradient norm = {}, ls_iterations = {}, "
        "alpha = {}, step size = {}\n",
        k, stats_.cost[k], stats_.gradient_norm[k], stats_.ls_iterations[k],
        stats_.alpha[k], stats_.step_size[k]);
  }
}

template <typename T>
void ConvexIntegrator<T>::LogSolverStats() {
  DRAKE_THROW_UNLESS(log_file_.is_open());
  const T time = this->get_context().get_time();
  for (int k = 0; k <= stats_.iterations; ++k) {
    log_file_ << time << "," << k << "," << stats_.cost[k] << ","
              << stats_.gradient_norm[k] << "," << stats_.ls_iterations[k]
              << "," << stats_.alpha[k] << "," << stats_.step_size[k] << "\n";
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
