#include "drake/systems/analysis/convex_integrator.h"

namespace drake {
namespace systems {

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

  // print iters, cost, gradient norm, step size, and alpha
  fmt::print(
      "ConvexIntegrator: iters = {}, cost = {}, gradient norm = {}, "
      "step size = {}, alpha = {}\n",
      stats_.k, stats_.cost, stats_.gradient_norm, stats_.step_size,
      stats_.alpha);

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

  // TODO(vincekurtz): pre-allocate the search direction dv
  VectorXd dv(v.size());

  stats_.Reset();
  for (stats_.k = 0; stats_.k < solver_parameters_.max_iterations; ++stats_.k) {
    // Compute the cost, gradient, and Hessian.
    // TODO(vincekurtz): re-use the old Hessian
    model.CalcData(v, &data);

    // Update the statistics we have available so far
    stats_.cost = data.cache().cost;
    stats_.gradient_norm = data.cache().gradient.norm();

    // Early convergence check. Allows for early exit if v_guess is already
    // optimal, or a single Newton step for simple unconstrained problems. This
    // is necessary because our main convergence criterion requires comparing dv
    // over several iterations.
    if (stats_.gradient_norm < solver_parameters_.tolerance) {
      // TODO(vincekurtz): consider using the SAP momentum residual rather than
      // the gradient norm.
      return true;
    }

    // Compute the search direction via Newton step dv = -H⁻¹ g
    const VectorXd& g = data.cache().gradient;
    const MatrixXd& H = data.cache().hessian;
    dv = H.ldlt().solve(-g);

    // Compute the step size with linesearch
    stats_.num_ls_iterations = 0;
    stats_.alpha = 1.0;
    dv *= stats_.alpha;

    // Update the decision variables (velocities)
    v += dv;

    // Convergence check from on [Hairer, 1996], Eq. IV.8.10.
    stats_.last_step_size = stats_.step_size;
    stats_.step_size = dv.norm();
    if (stats_.k > 0) {
      // N.B. this only comes into effect in the second iteration, since we need
      // both step_norm and last_step_norm to be set.
      stats_.theta = stats_.step_size / stats_.last_step_size;
      if ((stats_.theta < 1.0) &&
          ((stats_.theta / (1.0 - stats_.theta)) < k_dot_tol)) {
        return true;
      }

      // TODO(vincekurtz): use theta to trigger hessian re-computation.
    }
  }

  return false;  // Failed to converge.
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
