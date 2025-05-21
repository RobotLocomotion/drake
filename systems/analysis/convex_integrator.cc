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

  // TODO(vincekurtz): pre-allocate dv
  VectorXd dv = v;              // Search direction
  double alpha = 1.0;           // Step size (linesearch parameter)
  double step_norm = 0.0;       // Size of the current step
  double last_step_norm = 0.0;  // Size of the previous step
  double theta;                 // ratio of current and previous step sizes

  for (int k = 0; k < solver_parameters_.max_iterations; ++k) {
    // Compute the cost, gradient, and Hessian.
    // TODO(vincekurtz): re-use the old Hessian
    model.CalcData(v, &data);

    // Early convergence check. Allows for early exit if v_guess is already
    // optimal, or a single Newton step for simple unconstrained problems. This
    // is necessary because our main convergence criterion requires comparing dv
    // over several iterations.
    const VectorXd& g = data.cache().gradient;
    if (g.norm() < solver_parameters_.abs_tolerance +
                       solver_parameters_.rel_tolerance * g.norm()) {
      // TODO(vincekurtz): consider using the SAP momentum residual rather than
      // the gradient norm.
      return true;  // Converged!
    }

    // Compute the search direction via Newton step dv = -H⁻¹ g
    const MatrixXd& H = data.cache().hessian;
    dv = H.ldlt().solve(-g);

    // Compute the step size with linesearch
    dv *= alpha;

    // Update the decision variables (velocities)
    v += dv;

    // Convergence check from on [Hairer, 1996], Eq. IV.8.10.
    last_step_norm = step_norm;
    step_norm = dv.norm();
    if (k > 0) {
      // N.B. this only comes into effect in the second iteration, since we need
      // both step_norm and last_step_norm to be set.
      theta = step_norm / last_step_norm;
      if ((theta < 1.0) && ((theta / (1.0 - theta)) < k_dot_tol)) {
        return true;  // Converged!
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
