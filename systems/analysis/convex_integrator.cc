#include "drake/systems/analysis/convex_integrator.h"

#include "drake/math/quaternion.h"
#include "drake/multibody/contact_solvers/newton_with_bisection.h"

namespace drake {
namespace systems {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using multibody::Joint;
using multibody::JointIndex;
using multibody::contact_solvers::internal::Bracket;
using multibody::contact_solvers::internal::DoNewtonWithBisectionFallback;

template <typename T>
ConvexIntegrator<T>::ConvexIntegrator(const System<T>& system,
                                      Context<T>* context)
    : IntegratorBase<T>(system, context) {}

template <typename T>
void ConvexIntegrator<T>::DoInitialize() {
  using std::isnan;
  if (plant_ == nullptr) {
    throw std::runtime_error(
        "ConvexIntegrator: MultibodyPlant not set. You must call "
        "ConvexIntegrator::set_plant() before initialization.");
  }

  // Convex integration works best at loose accuracies.
  const double kDefaultAccuracy = 1e-1;

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

  // Set the initial time step and accuracy.
  if (isnan(this->get_initial_step_size_target())) {
    if (isnan(this->get_maximum_step_size()))
      throw std::logic_error(
          "Neither initial step size target nor maximum "
          "step size has been set!");
    this->request_initial_step_size_target(this->get_maximum_step_size());
  }

  double working_accuracy = this->get_target_accuracy();
  if (isnan(working_accuracy)) working_accuracy = kDefaultAccuracy;
  this->set_accuracy_in_use(working_accuracy);

  // Allocate and initialize SAP problem objects
  builder_ = std::make_unique<PooledSapBuilder<T>>(plant());
  const T& dt = this->get_initial_step_size_target();
  builder_->UpdateModel(plant_context, dt, &model_);
  model_.ResizeData(&data_);
  search_direction_.resize(model_.num_velocities());
  f_ext_ = std::make_unique<MultibodyForces<T>>(plant());

  // Allocate intermediate states for error control
  x_next_full_ = this->get_system().AllocateTimeDerivatives();
  x_next_half_1_ = this->get_system().AllocateTimeDerivatives();
  x_next_half_2_ = this->get_system().AllocateTimeDerivatives();

  // Allocate memory for the solver statistics.
  stats_.Reserve(solver_parameters_.max_iterations);

  // Set up the CSV file and write a header, if logging is enabled.
  if (solver_parameters_.log_solver_stats) {
    log_file_.open("convex_integrator_stats.csv");
    log_file_
        << "time,iteration,cost,gradient_norm,ls_iterations,alpha,step_size\n";
  }
}

template <typename T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  Context<T>& context = *this->get_mutable_context();
  ContinuousState<T>& x_next = context.get_mutable_continuous_state();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  const T t0 = context.get_time();

  // TODO(vincekurtz): consider delaying this to encourage cache hits

  // Solve for the full step x_{t+h}. We'll need this regardless of whether
  // error control is enabled or not.
  // TODO(vincekurtz): consider pre-allocating v_guess
  VectorX<T> v_guess = plant().GetVelocities(plant_context);
  ComputeNextContinuousState(h, v_guess, x_next_full_.get());

  if (this->get_fixed_step_mode()) {
    // We're using fixed step mode, so we can just set the state to x_{t+h} and
    // move on. No need for error estimation.
    // N.B. this is slightly faster than x_next.SetFrom(*x_next_full_), because
    // it saves an intermediate Eigen representation.
    x_next.get_mutable_vector().SetFrom(x_next_full_->get_vector());
    context.SetTime(t0 + h);
  } else {
    // We're using error control, and will compare with two half-sized steps.

    // First half-step to (t + h/2) uses the average of v_t and v_{t+1} as the
    // initial guess
    v_guess += x_next_full_->get_generalized_velocity().CopyToVector();
    v_guess /= 2.0;
    ComputeNextContinuousState(0.5 * h, v_guess, x_next_half_1_.get());

    // For the second half-step to (t + h), we need to start from (t + h/2). So
    // we'll first set the system state to the result of the first half-step.
    x_next.get_mutable_vector().SetFrom(x_next_half_1_->get_vector());
    context.SetTimeAndNoteContinuousStateChange(t0 + 0.5 * h);

    // Now we can take the second half-step. We'll use the solution of the full
    // step as our initial guess here.
    v_guess = x_next_full_->get_generalized_velocity().CopyToVector();
    ComputeNextContinuousState(0.5 * h, v_guess, x_next_half_2_.get());

    // Set the state to the result of the second half-step (since this is more
    // accurate than the full step, and we have it anyway).
    x_next.get_mutable_vector().SetFrom(x_next_half_2_->get_vector());
    context.SetTimeAndNoteContinuousStateChange(t0 + h);

    // Estimate the error as the difference between the full step and the
    // two half-steps.
    ContinuousState<T>& err = *this->get_mutable_error_estimate();
    err.get_mutable_vector().SetFrom(x_next_full_->get_vector());
    err.get_mutable_vector().PlusEqScaled(-1.0, x_next_half_2_->get_vector());
  }

  return true;  // step was successful
}

template <typename T>
void ConvexIntegrator<T>::ComputeNextContinuousState(
    const T& h, const VectorX<T>& v_guess, ContinuousState<T>* x_next) {
  // Get plant context storing initial state [q₀, v₀].
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);

  // TODO(vincekurtz): pre-allocate these
  VectorX<T> Ku(plant().num_velocities());
  VectorX<T> bu(plant().num_velocities());
  VectorX<T> Ke(plant().num_velocities());
  VectorX<T> be(plant().num_velocities());

  // Linearize any external systems (e.g., controllers) connected to the plant,
  //     τ = B u(x) + τₑₓₜ(x),
  //       ≈ clamp(-Kᵤ v + bᵤ) - Kₑ v + bₑ,
  // TODO(vincekurtz): check model instance specific external force ports
  if (plant().num_actuators() > 0 ||
      plant().get_applied_generalized_force_input_port().HasValue(
          plant_context) ||
      plant().get_applied_spatial_force_input_port().HasValue(plant_context)) {
    // Only do the linearization if a controller is connected
    LinearizeExternalSystem(h, &Ku, &bu, &Ke, &be);
  } else {
    Ku.setZero();
    bu.setZero();
    Ke.setZero();
    be.setZero();
  }

  // Set up the convex optimization problem minᵥ ℓ(v; q₀, v₀, h)
  PooledSapModel<T>& model = get_model();
  builder().UpdateModel(plant_context, h, &model);
  
  // TODO(vincekurtz): only add these constraints if the associated ports are
  // actually connected.
  builder().AddActuationGains(Ku, bu, &model);
  builder().AddExternalGains(Ke, be, &model);

  // TODO(vincekurtz): pre-allocate v
  VectorX<T> v = v_guess;

  // Solve the optimization problem for next-step velocities v = min ℓ(v).
  if (!SolveWithGuess(model, &v)) {
    throw std::runtime_error("ConvexIntegrator: optimization failed.");
  }

  // Accumulate solver statistics, log as requested.
  if (solver_parameters_.log_solver_stats) LogSolverStats();
  total_solver_iterations_ += stats_.iterations;
  total_ls_iterations_ += std::accumulate(stats_.ls_iterations.begin(),
                                          stats_.ls_iterations.end(), 0);

  // q = q₀ + h N(q₀) v
  // TODO(vincekurtz): pre-allocate q
  VectorX<T> q(plant().num_positions());
  AdvancePlantConfiguration(h, v, &q);

  // Set the updated state
  // TODO(vincekurtz): update the non-plant states z.
  x_next->get_mutable_generalized_position().SetFromVector(q);
  x_next->get_mutable_generalized_velocity().SetFromVector(v);
}

template <typename T>
void ConvexIntegrator<T>::AdvancePlantConfiguration(const T& h,
                                                    const VectorX<T>& v,
                                                    VectorX<T>* q) const {
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  const VectorX<T>& q0 = plant().GetPositions(plant_context);

  // q = q₀ + h N(q₀) v
  plant().MapVelocityToQDot(plant_context, h * v, q);  // δq = h N(q₀) v
  *q += q0;                                            // q = q₀ + δq

  // Normalize quaternions. Note that the more exact solution would be
  //   q = exp(h q̇ * q̅₀) * q₀,
  // where "*" and "exp" are quaternion multiplication and exponentiation, and
  // q̅₀ is the conjugate of q₀. However, just normalizing
  //   q = q₀ + δq,
  // is a fine approximation for small h, and error control ensure that the
  // resulting error doesn't get out of hand.
  for (JointIndex joint_index : plant().GetJointIndices()) {
    const Joint<T>& joint = plant().get_joint(joint_index);

    if (joint.type_name() == "quaternion_floating") {
      const int i = joint.position_start();
      q->template segment<4>(i).normalize();
    }
  }
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

  // Gradient convergence tolerance is scaled by D = diag(M)^{-1/2}, so that all
  // entries of g̃ = Dg and ṽ = D⁻¹v have the same units [Castro 2021, IV.E].
  //
  // Convergence is achieved when either the (normalized) gradient is small
  //   ‖D g‖ ≤ ε max(1, ‖D r‖),
  // or the normalized) step size is sufficiently small,
  //   η ‖D⁻¹ Δv‖ ≤ ε max(1, ‖D r‖).
  // where η = θ / (1 − θ), θ = ‖D⁻¹ Δvₖ‖ / ‖D⁻¹ Δvₖ₋₁‖, as per [Hairer, 1996].
  const VectorXd& D = model.params().D;
  const double scale = std::max(1.0, (D.asDiagonal() * model.r()).norm());

  // Without error control, we'll set ε to the user-defined tolerance. With
  // error control, we'll scale ε with the desired accuracy. This is typically
  // quite a bit looser, and ensures that we don't waste effort on tight
  // convergence.
  const double eps =
      this->get_fixed_step_mode()
          ? solver_parameters_.tolerance
          : solver_parameters_.kappa * this->get_accuracy_in_use();

  double alpha{NAN};
  int ls_iterations{0};
  double eta = 1.0;

  stats_.Reset(this->get_context().get_time());
  if (solver_parameters_.print_solver_stats) {
    fmt::print("ConvexIntegrator: solving at t = {:.4f}\n",
               this->get_context().get_time());
  }

  for (int k = 0; k < solver_parameters_.max_iterations; ++k) {
    // Compute the cost and gradient
    model.CalcData(v, &data);
    const double grad_norm = (D.asDiagonal() * data.cache().gradient).norm();

    // We'll print stats before doing any convergence checks. That ensures that
    // we get a printout even v_guess is already good enough.
    if (solver_parameters_.print_solver_stats) {
      const double step_size = (k == 0) ? NAN : stats_.step_size.back();
      fmt::print(
          "  k: {}, cost: {}, gradient: {:e}, step: {:e}, ls_iterations: {}, "
          "alpha: {}\n",
          k, data.cache().cost, grad_norm, step_size, ls_iterations, alpha);
    }

    // Gradient-based convergence check. Allows for early exit if v_guess is
    // already close enough to the solution.
    if (grad_norm < eps * scale) {
      return true;
    }

    // Step-size-based convergence check. This is only valid after the first
    // iteration has been completed, since we need ||D⁻¹ Δvₖ||
    if (k > 0) {
      // For k = 1, we have η = 1, so this is equivalent to ||D⁻¹ Δvₖ|| < tol.
      // Otherwise we use η = θ/(1−θ) (see Hairer 1996, p.120).
      if (eta * stats_.step_size.back() < eps * scale) {
        return true;
      }
    }

    // Choose whether to re-compute the Hessian factorization using Equation
    // IV.8.11 of [Hairer, 1996]. This essentially predicts whether we'll
    // converge within (k_max - k) iterations, assuming linear convergence.
    if (k > 1) {
      const double dvk = stats_.step_size[k - 1];    // ||D⁻¹ Δvₖ||
      const double dvkm1 = stats_.step_size[k - 2];  // ||D⁻¹ Δvₖ₋₁||
      const double theta = dvk / dvkm1;

      // For the step-size-based convergence check η ‖D⁻¹ Δv‖ ≤ ε max(1, ‖D r‖),
      // we need η \in (0, 1]. Therefore we only use η = θ/(1−θ) when θ < 1.0,
      // and otherwise make the (conservative) choice of η = 1.0.
      // N.B. unlike in the Newton-Raphson steps discussed in [Hairer, 1996],
      // it is possible that θ ≥ 1 without diverging, thanks to linesearch.
      eta = (theta < 1.0) ? theta / (1.0 - theta) : 1.0;

      const int k_max = solver_parameters_.max_iterations_for_hessian_reuse;
      const double anticipated_residual =
          std::pow(theta, k_max - k) / (1 - theta) * dvk;

      if (anticipated_residual > eps * scale || theta >= 1.0) {
        // We likely won't converge in time at this (linear) rate, so we should
        // use a fresh Hessian in hopes of faster (quadratic) convergence.
        reuse_hessian_factorization_ = false;
      }
    }

    // For iterations other than the first, we know the sparsity pattern is
    // unchanged, so we can reuse it without checking. The sparsity pattern is
    // often but not always the same between solves, so we need to perform the
    // full check when k = 0.
    bool reuse_sparsity_pattern = (k > 0) || !SparsityPatternChanged(model);

    // Hessian reuse is enabled when all of the following hold:
    //   1. reuse is enabled in the solver parameters,
    //   2. the sparsity pattern is unchanged,
    //   3. the "anticipated residual" heuristics indicate that we'll converge
    //      in time under a linear convergence assumption.
    bool reuse_hessian = solver_parameters_.enable_hessian_reuse &&
                         reuse_sparsity_pattern && reuse_hessian_factorization_;

    // Compute the search direction dv = -H⁻¹ g
    ComputeSearchDirection(model, data, &dv, reuse_hessian,
                           reuse_sparsity_pattern);

    // If the sparsity pattern changed, store it for future checks.
    if (!reuse_sparsity_pattern) {
      previous_sparsity_pattern_ =
          std::make_unique<BlockSparsityPattern>(model.sparsity_pattern());
    }

    // Compute the step size with linesearch
    std::tie(alpha, ls_iterations) = PerformExactLineSearch(model, data, dv);

    // Update the decision variables (velocities)
    dv *= alpha;
    v += dv;

    // Finalize solver stats that we now have after taking the step
    stats_.iterations++;
    stats_.cost.push_back(data.cache().cost);
    stats_.gradient_norm.push_back(data.cache().gradient.norm());
    stats_.ls_iterations.push_back(ls_iterations);
    stats_.alpha.push_back(alpha);
    stats_.step_size.push_back((D.cwiseInverse().asDiagonal() * dv).norm());
  }

  return false;  // Failed to converge.
}

template <typename T>
std::pair<T, int> ConvexIntegrator<T>::PerformExactLineSearch(
    const PooledSapModel<T>&, const SapData<T>&, const VectorX<T>&) {
  throw std::logic_error(
      "ConvexIntegrator: PerformExactLineSearch only supports T = double.");
}

template <>
std::pair<double, int> ConvexIntegrator<double>::PerformExactLineSearch(
    const PooledSapModel<double>& model, const SapData<double>& data,
    const VectorXd& dv) {
  const double alpha_max = solver_parameters_.alpha_max;

  // Set up prerequisites for an efficient CalcCostAlongLine
  SapData<double>& scratch = scratch_data_;
  model.ResizeData(&scratch);
  SearchDirectionData<double>& search_data = search_direction_data_;
  model.UpdateSearchDirection(data, dv, &search_data);

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
  const double ell = model.CalcCostAlongLine(alpha_max, data, search_data,
                                             &scratch, &dell, &d2ell);
  if (dell <= std::numeric_limits<double>::epsilon()) {
    return std::make_pair(alpha_max, 0);
  }

  // TODO(vincekurtz): add a check to enable full Newton steps very close to
  // machine epsilon, as in SapSolver. We'll need to be mindful of the fact that
  // the cost can be negative in the pooled SAP formulation.

  // Set the initial guess for linesearch based on a cubic hermite spline
  // between α = 0 and α = α_max. This spline takes the form
  // p(t) = a t³ + b t² + c t + d, where
  //   p(0) = ℓ(0),
  //   p(1) = ℓ(α_max),
  //   p'(0) = α_max⋅ℓ'(0),
  //   p'(1) = α_max⋅ℓ'(α_max).
  // We can then find the analytical minimum in [0, α_max], and use that to
  // establish an initial guess for linesearch.
  const double a = 2 * (ell0 - ell) + dell0 * alpha_max + dell * alpha_max;
  const double b = -3 * (ell0 - ell) - 2 * dell0 * alpha_max - dell * alpha_max;
  const double c = dell0 * alpha_max;
  // N.B. throws if a solution cannot be found in [0, 1]
  const double alpha_guess =
      alpha_max * SolveQuadraticInUnitInterval(3 * a, 2 * b, c);

  // We've exhausted all of the early exit conditions, so now we move on to the
  // Newton method with bisection fallback. To do so, we define an anonymous
  // function that computes the value and gradient of f(α) = −ℓ'(α)/ℓ'₀.
  // Normalizing in this way reduces round-off errors, ensuring f(0) = -1.
  const double dell_scale = -dell0;
  auto cost_and_gradient = [&model, &data, &search_data, &scratch,
                            &dell_scale](double x) {
    double dell_dalpha;
    double d2ell_dalpha2;
    model.CalcCostAlongLine(x, data, search_data, &scratch, &dell_dalpha,
                            &d2ell_dalpha2);
    return std::make_pair(dell_dalpha / dell_scale, d2ell_dalpha2 / dell_scale);
  };

  // The initial bracket is [0, α_max], since we already know that ℓ'(0) < 0 and
  // ℓ'(α_max) > 0. Values at the endpoints of the bracket are f(0) = -1 (by
  // definition) and f(α_max) = dell / dell_scale, because we just set dell =
  // ℓ'(α_max) above.
  const Bracket bracket(0.0, -1.0, alpha_max, dell / dell_scale);

  // TODO(vincekurtz): scale linesearch tolerance based on accuracy.
  const double alpha_tolerance = solver_parameters_.ls_tolerance;
  return DoNewtonWithBisectionFallback(
      cost_and_gradient, bracket, alpha_guess, alpha_tolerance,
      solver_parameters_.ls_tolerance, solver_parameters_.max_ls_iterations);
}

template <typename T>
T ConvexIntegrator<T>::SolveQuadraticInUnitInterval(const T& a, const T& b,
                                                    const T& c) const {
  using std::sqrt;

  // Sign function that returns 1 for positive numbers, -1 for
  // negative numbers, and 0 for zero.
  auto sign = [](const T& x) {
    return (x > 0) - (x < 0);
  };

  T s;
  if (a < std::numeric_limits<T>::epsilon()) {
    // If a ≈ 0, just solve b x + c = 0.
    s = -c / b;
  } else {
    // Use the numerically stable root-finding method described here:
    // https://math.stackexchange.com/questions/866331/
    const T discriminant = b * b - 4 * a * c;
    DRAKE_DEMAND(discriminant >= 0);  // must have a real root
    s = -b - sign(b) * sqrt(discriminant);
    s /= 2 * a;
    // If the first root is outside [0, 1], try the second root.
    if (s < 0.0 || s > 1.0) {
      s = c / (a * s);
    }
  }

  // The solution must be in [0, 1]
  if (s < 0.0 || s > 1.0) {
    fmt::print("a: {}, b: {}, c: {}\n", a, b, c);
    fmt::print("s: {}\n", s);
    throw std::runtime_error(
        "ConvexIntegrator: quadratic root falls outside [0, 1].");
  }

  return s;
}

template <typename T>
void ComputeSearchDirection(const PooledSapModel<T>&, const SapData<T>&,
                            VectorX<T>*, bool, bool) {
  throw std::logic_error(
      "ConvexIntegrator: ComputeSearchDirection only supports T = double.");
}

template <>
void ConvexIntegrator<double>::ComputeSearchDirection(
    const PooledSapModel<double>& model, const SapData<double>& data,
    VectorXd* dv, bool reuse_factorization, bool reuse_sparsity_pattern) {
  DRAKE_ASSERT(dv != nullptr);

  if (!reuse_factorization) {
    // Compute the H and set up the factorization.
    if (reuse_sparsity_pattern) {
      model.UpdateHessian(data, hessian_.get());
      hessian_factorization_.UpdateMatrix(*hessian_);
    } else {
      hessian_ = model.MakeHessian(data);
      hessian_factorization_.SetMatrix(*hessian_);
    }

    // Factorize H
    if (!hessian_factorization_.Factor()) {
      throw std::runtime_error(
          "ConvexIntegrator: Hessian factorization failed!");
    }
    total_hessian_factorizations_++;
    reuse_hessian_factorization_ = true;
  }

  // Compute the search direction dv = -H⁻¹ g
  *dv = -data.cache().gradient;
  hessian_factorization_.SolveInPlace(dv);
}

template <typename T>
void ConvexIntegrator<T>::LogSolverStats() {
  DRAKE_THROW_UNLESS(log_file_.is_open());
  for (int k = 0; k < stats_.iterations; ++k) {
    log_file_ << stats_.time << "," << k << "," << stats_.cost[k] << ","
              << stats_.gradient_norm[k] << "," << stats_.ls_iterations[k]
              << "," << stats_.alpha[k] << "," << stats_.step_size[k] << "\n";
  }
}

template <typename T>
bool ConvexIntegrator<T>::SparsityPatternChanged(
    const PooledSapModel<T>& model) const {
  if (previous_sparsity_pattern_ == nullptr) {
    return true;  // No previous sparsity pattern to compare against.
  }
  return (model.sparsity_pattern().neighbors() !=
          previous_sparsity_pattern_->neighbors()) ||
         (model.sparsity_pattern().block_sizes() !=
          previous_sparsity_pattern_->block_sizes());
}

template <typename T>
void ConvexIntegrator<T>::CalcExternalForces(const Context<T>& context,
                                                    VectorX<T>* tau) {
  MultibodyForces<T>& forces = *f_ext_;
  forces.SetZero();
  plant().AddAppliedExternalSpatialForces(context, &forces);
  plant().AddAppliedExternalGeneralizedForces(context, &forces);
  plant().CalcGeneralizedForces(context, forces, tau);
}

template <typename T>
void ConvexIntegrator<T>::CalcActuationForces(const Context<T>& context,
                                              VectorX<T>* tau) {
  // TODO(vincekurtz): get rid of this allocation
  const MatrixX<T> B = plant().MakeActuationMatrix();
  *tau = B * plant().AssembleActuationInput(context);
}

template <typename T>
void ConvexIntegrator<T>::LinearizeExternalSystem(const T& h, VectorX<T>* Ku,
                                                  VectorX<T>* bu,
                                                  VectorX<T>* Ke,
                                                  VectorX<T>* be) {
  using std::abs;

  // Get some useful sizes
  const Context<T>& context = this->get_context();
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  const ContinuousState<T>& state = context.get_continuous_state();
  const int nq = state.num_q();
  const int nv = state.num_v();

  // TODO(vincekurtz): pre-allocate these, and/or consider doing away with
  // Ku_full entirely.
  MatrixX<T> Ku_full(nv, nv);
  MatrixX<T> Ke_full(nv, nv);
  VectorX<T> gu0(nv);
  VectorX<T> ge0(nv);
  MatrixX<T> N = plant().MakeVelocityToQDotMap(plant_context);

  // Compute some quantities that depend on the current state, before messing
  // with the state with finite differences
  const VectorX<T> v0 = state.get_generalized_velocity().CopyToVector();
  CalcActuationForces(plant_context, &gu0);
  CalcExternalForces(plant_context, &ge0);

  // Allocate a perturbed state x = [q; v; z] and outputs 
  //    gu(x) = B u(x), 
  //    ge(x) = τₑₓₜ(x)
  const VectorX<T> x = state.CopyToVector();
  VectorX<T> x_prime = x;
  VectorX<T> gu_prime = gu0;
  VectorX<T> ge_prime = ge0;

  // We'll want to perturb q and v separately, so we'll go ahead and grab these
  // references.
  auto q = x.head(nq);
  auto v = x.segment(nq, nv);
  auto q_prime = x_prime.head(nq);
  auto v_prime = x_prime.segment(nq, nv);

  // Compute τ = D(v − v₀) + g₀ with forward differences differences
  Context<T>* mutable_context = this->get_mutable_context();
  ContinuousState<T>& mutable_state =
      mutable_context->get_mutable_continuous_state();
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());

  for (int i = 0; i < nv; ++i) {
    // Choose a step size (following implicit_integrator.cc)
    const T abs_vi = abs(v(i));
    T dvi(abs_vi);
    if (dvi <= 1) {
      dvi = eps;
    } else {
      dvi = eps * abs_vi;
    }

    // Ensure that v' and v differ by an exactly representable number
    v_prime(i) = v(i) + dvi;
    dvi = v_prime(i) - v(i);

    // Perturb q as well, using the fact that q' = q + h N dv
    q_prime = q + h * N * (v_prime - v);

    // Put x' in the context and mark the state as stale
    mutable_state.SetFromVector(x_prime);
    mutable_context->NoteContinuousStateChange();

    // Compute the relevant matrix entries for actuation inputs:
    //   Ku_full = dgu/dv
    CalcActuationForces(plant_context, &gu_prime);
    Ku_full.col(i) = (gu_prime - gu0) / dvi;

    // Same thing, but for external systems:
    //  Ke_full = dge/dv
    CalcExternalForces(plant_context, &ge_prime);
    Ke_full.col(i) = (ge_prime - ge0) / dvi;

    // Reset the state for the next iteration
    v_prime(i) = v(i);
  }

  // Reset the context back to how we found it
  mutable_state.SetFromVector(x);
  mutable_context->NoteContinuousStateChange();

  // Use the diagonal projection for both K and b ensures that any
  // non-convex portion of the dynamics is treated explicitly.
  (*Ku) = (-Ku_full).diagonal().cwiseMax(0);
  (*bu) = gu0 + (*Ku).asDiagonal() * v0;

  (*Ke) = (-Ke_full).diagonal().cwiseMax(0);
  (*be) = ge0 + (*Ke).asDiagonal() * v0;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
