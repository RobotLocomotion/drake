#include "drake/systems/analysis/convex_integrator.h"

#include <chrono>

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
  search_direction_.resize(model_.num_velocities());

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
  VectorXd& dv = search_direction_;
  model.ResizeData(&data);
  dv.resize(data.num_velocities());
  DRAKE_DEMAND(data.num_velocities() == v.size());

  // Tolerance for a more relaxed convergence check under looser accuracies.
  const double k_dot_tol =
      solver_parameters_.kappa * this->get_accuracy_in_use();

  double alpha{NAN};
  int ls_iterations{0};

  stats_.Reset();
  for (int k = 0; k < solver_parameters_.max_iterations; ++k) {
    // Compute the cost and gradient
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
    // TODO(vincekurtz): re-use the old Hessian
    const VectorXd& g = data.cache().gradient;

    auto start = std::chrono::high_resolution_clock::now();
    if (k == 0) {
      hessian_ = model.MakeHessian(data);
    } else {
      model.UpdateHessian(data, hessian_.get());
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    const double make_hessian_time = elapsed.count();

    start = std::chrono::high_resolution_clock::now();
    if (k == 0) {
      hessian_factorization_.SetMatrix(*hessian_);
    } else {
      hessian_factorization_.UpdateMatrix(*hessian_);
    }
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    const double set_matrix_time = elapsed.count();

    start = std::chrono::high_resolution_clock::now();
    if (!hessian_factorization_.Factor()) {
      throw std::runtime_error("Hessian factorization failed!");
    }
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    const double factor_time = elapsed.count();

    start = std::chrono::high_resolution_clock::now();
    dv = -g;
    hessian_factorization_.SolveInPlace(&dv);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    const double solve_time = elapsed.count();

    // fmt::print("k: {}, MakeHessian: {}, SetMatrix: {}, Factor: {}, Solve:
    // {}\n",
    //            k, make_hessian_time, set_matrix_time, factor_time,
    //            solve_time);
    (void)make_hessian_time;
    (void)set_matrix_time;
    (void)factor_time;
    (void)solve_time;

    // Compute the step size with linesearch
    std::tie(alpha, ls_iterations) = PerformExactLineSearch(model, v, dv);

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
std::pair<T, int> ConvexIntegrator<T>::PerformExactLineSearch(
    const PooledSapModel<T>&, const VectorX<T>&, const VectorX<T>&) {
  throw std::logic_error(
      "ConvexIntegrator: PerformExactLineSearch only supports T = double.");
}

template <>
std::pair<double, int> ConvexIntegrator<double>::PerformExactLineSearch(
    const PooledSapModel<double>& model, const VectorXd& v,
    const VectorXd& dv) {
  SapData<double>& data = get_data();
  const double alpha_max = solver_parameters_.alpha_max;

  // Allocate first and second derivatives of ℓ(α)
  double dell{NAN};
  double d2ell{NAN};

  // First we'll evaluate ∂ℓ/∂α at α = 0. This should be strictly negative,
  // since the Hessian is positive definite. This is cheap since we already have
  // the gradient at α = 0 cached from solving for the search direction earlier.
  // N.B. We use a new dell0 rather than dell declared above, b/c both dell and
  // dell0 will be used to generate an initial guess.
  const double ell0 = data.cache().cost;
  const double dell0 = data.cache().gradient.dot(dv);
  if (dell0 >= 0) {
    throw std::logic_error(
        "ConvexIntegrator: the cost does not decrease along the search "
        "direction. This is usually caused by an excessive accumulation of "
        "round-off errors for ill-conditioned systems. Consider revisiting "
        "your model.");
  }

  // Next we'll evaluate ℓ, ∂ℓ/∂α, and ∂²ℓ/∂α² at α = α_max. If the cost is
  // still decreasing here, we just accept α_max.
  double ell =
      model.CalcCostAlongLine(v, dv, alpha_max, &data, &dell, &d2ell);
  if (dell <= std::numeric_limits<double>::epsilon()) {
    return std::make_pair(alpha_max, 0);
  }

  // TODO(vincekurtz): add a check to enable full Newton steps very close to
  // machine epsilon, as in SapSolver. We'll need to be mindful of the fact that
  // the cost can be negative in the pooled SAP formulation.

  double alpha_guess;
  if (dell <= std::sqrt(std::numeric_limits<double>::epsilon())) {
    // We're barely decreasing at α_max, so the fancy spline method below
    // doesn't work very well. Instead, we'll set the guess based on a quadratic
    // approximation around α_max.
    alpha_guess = alpha_max - dell / d2ell;
  } else {
    // Set the initial guess for linesearch based on a cubic hermite spline
    // between α = 0 and α = α_max. This spline takes the form
    // p(t) = a t³ + b t² + c t + d, where
    //   p(0) = ℓ(0),
    //   p(1) = ℓ(α_max),
    //   p'(0) = α_max⋅ℓ'(0),
    //   p'(1) = α_max⋅ℓ'(α_max).
    // We can then find the analytical minimum in [0, α_max], and use that to
    // establish an initial guess for linesearch.
    const double a = 2 * ell0 - 2 * ell + dell0 * alpha_max + dell * alpha_max;
    const double b =
        -3 * ell0 + 3 * ell - 2 * dell0 * alpha_max - dell * alpha_max;
    const double c = dell0 * alpha_max;
    // N.B. throws if a solution cannot be found in [0, 1]
    alpha_guess = SolveQuadraticInUnitInterval(3 * a, 2 * b, c);
    alpha_guess *= alpha_max;
  }

  // Set up prerequisites for an efficient CalcCostAlongLine
  SapData<double>& scratch = scratch_data_;
  model.ResizeData(&scratch);
  SearchDirectionData<double>& search_data = search_direction_data_;
  model.UpdateSearchDirection(data, dv, &search_data);
   
  // DEBUG: check against old version
  model.CalcData(v, &data);
  model.CalcData(v, &scratch);
  ell = model.CalcCostAlongLine(alpha_guess, data, search_data, &scratch, &dell, &d2ell);
  fmt::print("ell: {}, dell: {}, d2ell: {}\n", ell, dell, d2ell);
  ell = model.CalcCostAlongLine(v, dv, alpha_guess, &data, &dell, &d2ell);
  fmt::print("ell: {}, dell: {}, d2ell: {}\n", ell, dell, d2ell);
  fmt::print("\n\n");

  // We've exhausted all of the early exit conditions, so now we move on to the
  // Newton method with bisection fallback. To do so, we define an anonymous
  // function that computes the value and gradient of f(α) = −ℓ'(α)/ℓ'₀.
  // Normalizing in this way reduces round-off errors, ensuring f(0) = -1.
  const double dell_scale = -dell0;
  auto cost_and_gradient = [&model, &data, &search_data, &scratch, &dell_scale](double x) {
    double dell_dalpha;
    double d2ell_dalpha2;
    model.CalcCostAlongLine(x, data, search_data, &scratch, &dell_dalpha, &d2ell_dalpha2);
    return std::make_pair(dell_dalpha / dell_scale, d2ell_dalpha2 / dell_scale);
  };

  // The initial bracket is [0, α_max], since we already know that ℓ'(0) < 0 and
  // ℓ'(α_max) > 0. Values at the endpoints of the bracket are f(0) = -1 (by
  // definition) and f(α_max) = dell / dell_scale, because we just set dell =
  // ℓ'(α_max) above.
  const Bracket bracket(0.0, -1.0, alpha_max, dell / dell_scale);

  // TODO(vincekurtz): scale linesearch tolerance based on accuracy.
  const double alpha_tolerance = solver_parameters_.ls_tolerance * alpha_guess;
  return DoNewtonWithBisectionFallback(
      cost_and_gradient, bracket, alpha_guess, alpha_tolerance,
      solver_parameters_.ls_tolerance, solver_parameters_.max_ls_iterations);
}

template <typename T>
T ConvexIntegrator<T>::SolveQuadraticInUnitInterval(const T& a, const T& b,
                                                    const T& c) const {
  using std::sqrt;

  T s;
  if (a < std::numeric_limits<T>::epsilon()) {
    // If a ≈ 0, just solve b x + c = 0.
    s = -c / b;
  } else {
    // Normalize everything by a
    const T b_tilde = b / a;
    const T c_tilde = c / a;

    const T discriminant = b_tilde * b_tilde - 4 * c_tilde;
    DRAKE_DEMAND(discriminant >= 0);  // must have a real root

    // Try the larger root first.
    s = (-b_tilde + sqrt(discriminant)) / 2.0;
    if (s > 1.0) {
      fmt::print("s: {}\n", s);
      s = (-b_tilde - sqrt(discriminant)) / 2.0;
    }
  }

  // The solution must be in [0, 1]
  if (s < 0.0 || s > 1.0) {
    throw std::runtime_error(
        "ConvexIntegrator: quadratic root falls outside [0, 1].");
  }

  return s;
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
