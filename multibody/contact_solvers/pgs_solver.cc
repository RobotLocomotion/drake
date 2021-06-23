#include "drake/multibody/contact_solvers/pgs_solver.h"

#include <algorithm>

#include "drake/common/unused.h"
namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
PgsSolver<T>::PgsSolver() = default;

template <typename T>
PgsSolver<T>::~PgsSolver() = default;

template <typename T>
ContactSolverStatus PgsSolver<T>::SolveWithGuess(
    const T& time_step, const SystemDynamicsData<T>& dynamics_data,
    const PointContactData<T>& contact_data, const VectorX<T>& v_guess,
    ContactSolverResults<T>* results) {
  unused(time_step);
  PreProcessData(dynamics_data, contact_data);
  // Aliases to pre-processed (const) data.
  const auto& mu = contact_data.get_mu();
  const auto& v_star = pre_proc_data_.v_star;
  const auto& vc_star = pre_proc_data_.vc_star;
  const auto& Dinv = pre_proc_data_.Dinv;
  // Since the Delassus operator W is symmetric, we have W = Wᵀ.
  // We need to access rows of the Delassus operator in the Gauss-Seidel solve
  // and W is stored in column major order, so we use Wᵀ instead of W for better
  // data locality.
  const auto& W = pre_proc_data_.W.transpose();

  // Aliases to solver's (mutable) state.
  auto& v = state_.mutable_v();
  auto& gamma = state_.mutable_gamma();

  // Aliases to solver parameters.
  const int max_iters = parameters_.max_iterations;
  const double omega = parameters_.omega;
  const int nc = contact_data.num_contacts();

  // Set initial guess for impulses.
  gamma.setZero();

  // Early exit if there's no contact.
  if (nc == 0) {
    // The velocity post contact is the same as that for pre-contact if there's
    // no contact.
    v = v_star;
    // Generalized contact force is zero if there's no contact.
    tau_c_.setZero();
    CopyContactResults(results);
    return ContactSolverStatus::kSuccess;
  }

  // Set initial guess for velocities.
  v = v_guess;

  // Below we use index k to denote the iteration. Hereinafter we'll adopt the
  // convention of appending a trailing _kp ("k plus") to refer to the next
  // iterate k+1.
  State state_kp(state_);  // Next iteration, k+1, state.
  // Aliases to the mutable next iteration state.
  VectorX<T>& v_kp = state_kp.mutable_v();
  VectorX<T>& gamma_kp = state_kp.mutable_gamma();

  // Contact velocity initialized to the value before the contact solve.
  vc_ = vc_star;
  VectorX<T> vc_kp(3 * nc);  // Contact velocity at state_kp.
  for (int k = 0; k < max_iters; ++k) {
    // Initialize the contact velocity to the value before any contact impulse
    // is applied.
    vc_kp = vc_star;
    // Add the contribution of all contact impulse to the contact velocity at
    // each contact point in a Gauss-Seidel fashion. Namely, for each contact
    // point i, the contact impulse contribution from j is evaluated at the k's
    // iteration if j > i and is evaluated at the k+1's iteration if j < i.
    for (int i = 0; i < nc; ++i) {
      const int i3 = 3 * i;
      auto vci_kp = vc_kp.template segment<3>(i3);
      // Contribution from contact impulses 0, 1, ..., i-1.
      vci_kp += W.block(i3, 0, 3, i3) * gamma_kp.head(i3);
      // Contribution from contact impulses i, i+1, i+2, ..., nc.
      vci_kp += W.block(i3, i3, 3, 3 * (nc - i)) * gamma.tail(3 * (nc - i));
      const auto& gammai = gamma.template segment<3>(i3);
      auto gammai_kp = gamma_kp.template segment<3>(i3);
      // Get the k+1's iteration of the contact impulse at contact point i.
      gammai_kp =
          gammai - omega * Dinv.template segment<3>(i3).asDiagonal() * vci_kp;
      // Project the contact impulse at contact point i into the friction cone.
      gammai_kp = ProjectImpulse(vci_kp, gammai_kp, mu(i));
    }
    // Update generalized velocities; v = v* + A⁻¹⋅Jᵀ⋅γ.
    contact_data.get_Jc().MultiplyByTranspose(gamma_kp,
                                              &tau_c_);  // tau_c = Jᵀ⋅γ
    dynamics_data.get_Ainv().Multiply(tau_c_, &v_kp);  // v_kp = A⁻¹⋅Jᵀ⋅γ
    v_kp += v_star;  // v_kp = v* + A⁻¹⋅Jᵀ⋅γ
    // Update contact velocities; vc = J⋅v.
    contact_data.get_Jc().Multiply(v_kp, &vc_kp);

    // Verify convergence.
    const bool converged = VerifyConvergenceCriteria(
        nc, vc_, vc_kp, gamma, gamma_kp, &stats_.vc_err, &stats_.gamma_err);
    ++stats_.iterations;

    // Update state for the next iteration.
    state_ = state_kp;
    vc_ = vc_kp;
    if (converged) {
      CopyContactResults(results);
      return ContactSolverStatus::kSuccess;
    }
  }
  // Return the results from the last iteration and report the solver failed to
  // converge.
  CopyContactResults(results);
  return ContactSolverStatus::kFailure;
}

template <typename T>
void PgsSolver<T>::PreProcessedData::Resize(int nv, int nc) {
  W.resize(3 * nc, 3 * nc);
  vc_star.resize(3 * nc);
  v_star.resize(nv);
  Wii_norm.resize(nc);
  Dinv.resize(3 * nc);
}

template <typename T>
void PgsSolver<T>::CopyContactResults(ContactSolverResults<T>* results) const {
  const int nv = state_.v().size();
  const int nc = state_.gamma().size() / 3;
  results->Resize(nv, nc);
  results->v_next = state_.v();
  ExtractNormal(vc_, &(results->vn));
  ExtractTangent(vc_, &(results->vt));
  ExtractNormal(state_.gamma(), &(results->fn));
  ExtractTangent(state_.gamma(), &(results->ft));
  results->tau_contact = tau_c_;
}

template <typename T>
void PgsSolver<T>::PreProcessData(const SystemDynamicsData<T>& dynamics_data,
                                  const PointContactData<T>& contact_data) {
  const int nc = contact_data.num_contacts();
  const int nv = dynamics_data.num_velocities();
  state_.Resize(nv, nc);
  pre_proc_data_.Resize(nv, nc);
  tau_c_.resize(nv);
  vc_.resize(3 * nc);

  // Generalized velocities when contact forces are zero.
  auto& v_star = pre_proc_data_.v_star;
  v_star = dynamics_data.get_v_star();

  if (nc != 0) {
    // Contact velocities when contact forces are zero.
    auto& vc_star = pre_proc_data_.vc_star;
    contact_data.get_Jc().Multiply(v_star, &vc_star);

    auto& W = pre_proc_data_.W;
    this->FormDelassusOperatorMatrix(contact_data.get_Jc(),
                                     dynamics_data.get_Ainv(),
                                     contact_data.get_Jc(), &W);

    // Compute scaling factors, one per contact.
    auto& Wii_norm = pre_proc_data_.Wii_norm;
    auto& Dinv = pre_proc_data_.Dinv;
    using std::max;
    for (int i = 0; i < nc; ++i) {
      // 3x3 diagonal block. It might be singular, but definitely non-zero.
      // That's why we use an RMS norm.
      const Matrix3<T> Wii = W.block(3 * i, 3 * i, 3, 3);
      Wii_norm(i) = Wii.norm() / 3;  // 3 = sqrt(9).
      // Diagonal approximation of the inverse of the Wii. We want a diagonal
      // approximation that represents the tangential and normal components of
      // the Delassus operator block, but limit them with Wii_norm to avoid
      // division by zero.
      Dinv.template segment<3>(3 * i) =
          Vector3<T>(1.0 / max((Wii(0, 0) + Wii(1, 1)) / 2, Wii_norm(i)),
                     1.0 / max((Wii(0, 0) + Wii(1, 1)) / 2, Wii_norm(i)),
                     1.0 / (max(Wii(2, 2), Wii_norm(i))));
    }
  }
}

template <typename T>
bool PgsSolver<T>::VerifyConvergenceCriteria(
    int num_contacts, const VectorX<T>& vc, const VectorX<T>& vc_kp,
    const VectorX<T>& gamma, const VectorX<T>& gamma_kp, double* vc_err,
    double* gamma_err) const {
  using std::max;
  const auto& Wii_norm = pre_proc_data_.Wii_norm;
  bool converged = true;
  *vc_err = 0;
  *gamma_err = 0;
  for (int ic = 0; ic < num_contacts; ++ic) {
    const int ic3 = ic * 3;
    auto within_error_bounds = [&p = parameters_](double error, double scale) {
      const double bound(p.abs_tolerance + p.rel_tolerance * scale);
      return error < bound;
    };
    // Check velocity convergence.
    const auto vci = vc.template segment<3>(ic3);
    const auto vci_kp = vc_kp.template segment<3>(ic3);
    const double vc_norm =
        ExtractDoubleOrThrow(vci.template lpNorm<Eigen::Infinity>());
    const double vci_err =
        ExtractDoubleOrThrow((vci_kp - vci).template lpNorm<Eigen::Infinity>());
    *vc_err = max(*vc_err, vci_err);
    if (converged && !within_error_bounds(vci_err, vc_norm)) {
      converged = false;
    }

    // Check impulse convergence. The criteria is the same as that for
    // velocities, but with an absolute tolerance inferred from the absolute
    // tolerance for velocities:
    // gamma_abs_tolerance = vc_abs_tolerance / ‖Wᵢ‖ᵣₘₛ.
    const auto gi = gamma.template segment<3>(ic3);
    const auto gi_kp = gamma_kp.template segment<3>(ic3);
    const double g_norm = ExtractDoubleOrThrow(
        gi.template lpNorm<Eigen::Infinity>() * Wii_norm(ic));
    const double g_err = ExtractDoubleOrThrow(
        (gi_kp - gi).template lpNorm<Eigen::Infinity>() * Wii_norm(ic));
    *gamma_err = max(*gamma_err, g_err);
    if (converged && !within_error_bounds(g_err, g_norm)) {
      converged = false;
    }
  }
  return converged;
}

template <typename T>
Vector3<T> PgsSolver<T>::ProjectImpulse(
    const Eigen::Ref<const Vector3<T>>& vc,
    const Eigen::Ref<const Vector3<T>>& gamma, const T& mu) const {
  const T& pi = gamma(2);                    // Normal component.
  if (pi <= 0.0) return Vector3<T>::Zero();  // No contact.

  const auto beta = gamma.template head<2>();  // Tangential component.
  if (beta.norm() <= mu * pi) return gamma;    // Inside the cone.

  // Non-zero impulse lies outside the cone. We'll project it.
  using std::sqrt;
  // We use the absolute tolerance as a velocity scale to use in a velocity
  // soft norm.
  const double v_eps = parameters_.abs_tolerance;
  const double v_eps2 = v_eps * v_eps;
  // Alias to tangential velocity.
  const auto vt = vc.template head<2>();
  // Compute a direction.
  const T vt_soft_norm = sqrt(vt.squaredNorm() + v_eps2);
  const Vector2<T> t_hat = vt / vt_soft_norm;
  // Project with Principle of maximum dissipation.
  const Vector2<T> projected_beta = -mu * pi * t_hat;
  return Vector3<T>(projected_beta(0), projected_beta(1), pi);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::PgsSolver);
