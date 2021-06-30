#pragma once

#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

struct PgsSolverParameters {
  // Over-relaxation parameter, in (0, 1]
  double omega{1};
  // Absolute contact velocity tolerance, m/s. See VerifyConvergenceCriteria().
  double abs_tolerance{1.0e-4};
  // Relative contact velocity tolerance, unitless. See
  // VerifyConvergenceCriteria().
  double rel_tolerance{1.0e-4};
  // Maximum number of PGS iterations.
  int max_iterations{100};
};

struct PgsSolverStats {
  int iterations{0};      // Number of PGS iterations.
  double vc_err{0.0};     // Error in the contact velocities, m/s.
  double gamma_err{0.0};  // Error in the contact impulses, Ns.
};

/* Implement the Projected Gauss-Seidel contact solver described in Algorithm 1
 in [Duriez, 2005]. The implementated algorithm differs from that
 described in the paper in two major aspects.
  1. [Duriez, 2005] uses the pair displacement δ and force f. The implemented
 algorithm uses the pair velocity v and impulse γ. As a result, the Delassus
 operators in the two version differ by a factor of dt².
  2. In the implemented algorithm, the convergence criterion considers the
 absolute *and* the relative error of both the change in contact velocities and
 the contact impulse.

 [Duriez, 2005] Duriez, Christian, et al. "Realistic haptic rendering of
 interacting deformable objects in virtual environments." IEEE transactions on
 visualization and computer graphics 12.1 (2005): 36-47.

 @tparam_nonsymbolic_scalar */
template <typename T>
class PgsSolver final : public ContactSolver<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PgsSolver);

  PgsSolver();

  ~PgsSolver() final;

  void set_parameters(const PgsSolverParameters& parameters) {
    parameters_ = parameters;
  }

  ContactSolverStatus SolveWithGuess(const T& time_step,
                                     const SystemDynamicsData<T>& dynamics_data,
                                     const PointContactData<T>& contact_data,
                                     const VectorX<T>& v_guess,
                                     ContactSolverResults<T>* results) final;

  const PgsSolverStats& get_solver_stats() const { return stats_; }

 private:
  /* The internal state of the PGS solver, consisting of v, the generalized
   velocities of the dofs in contact and gamma, the contact impulse at each
   contact point. */
  class State {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(State);
    State() = default;
    /* Resizes the state to accommodate for `nv` generalized velocities and `nc`
     contact points. */
    void Resize(int nv, int nc) {
      v_.resize(nv);
      gamma_.resize(3 * nc);
    }
    const VectorX<T>& v() const { return v_; }
    VectorX<T>& mutable_v() { return v_; }
    const VectorX<T>& gamma() const { return gamma_; }
    VectorX<T>& mutable_gamma() { return gamma_; }

   private:
    /* Generalized velocities. */
    VectorX<T> v_;
    /* Contact impulses. */
    VectorX<T> gamma_;
  };

  // All this data must remain const after the call to PreProcessData().
  struct PreProcessedData {
    // The Delassus operator.
    Eigen::SparseMatrix<T> W;
    // The contact velocity solution to the problem when there is no contact.
    VectorX<T> vc_star;
    // The generalized velocities before the solve.
    VectorX<T> v_star;
    // Norms of the 3x3 block diagonal blocks of matrix W, of size nc. Used to
    // roughly convert units of impulses to units of contact velocities when
    // verifying convergence.
    VectorX<T> Wii_norm;
    // Approximation to the inverse of the diagonal of W, of size nc.
    VectorX<T> Dinv;

    // Resizes `this` data given the number of generalized velocities
    // `nv` and number of contact points `nc`.
    void Resize(int nv, int nc);
  };

  /* Copies the current state of the solver into the given `results`. */
  void CopyContactResults(ContactSolverResults<T>* results) const;

  /* Sets the scratch data to correct sizes and compute preprocessed data that
   remains constant in SolveWithGuess(). */
  void PreProcessData(const SystemDynamicsData<T>& dynamics_data,
                      const PointContactData<T>& contact_data);

  /* Returns true if the change in contact velocity and the change in contact
   impulse from one iteration to the next is smaller than the absolute and
   relative error threshold. More specifically, the iterations are considered
   as converged if

    1. ‖vcᵢᵏ - vcᵢᵏ⁺¹‖∞ < abs_tolerance + rel_tolerance * ‖vcᵢᵏ‖∞, and
    2. ‖γᵢᵏ − γᵢᵏ⁺¹‖∞  < abs_tolerance / ‖Wᵢᵢ‖ᵣₘₛ + rel_tolerance * ‖γᵢᵏ‖∞

   where `abs_tolerance` and `rel_tolerance` are the absolute and relative
   tolerances specified in PgsSolverParameters, ‖Wᵢᵢ‖ᵣₘₛ is used to
   roughly convert the contact impulses into the unit of velocities, and
   vcᵢᵏ(γᵢᵏ) stands for the i-th contact point velocity (impulse) at the k-th
   iteration.

   @param num_contacts    The number of contact points.
   @param vc              The contact velocity at iteration k.
   @param vc_kp           The contact velocity at iteration k+1.
   @param gamma           The contact impulse at iteration k.
   @param gamma_kp        The contact impulse at iteration k+1.
   @param[out] vc_err     The maximum error in contact velocity in infinity
                          norm.
   @param[out] gamma_err  The maximum error in contact impulse in infinity norm.
  */
  bool VerifyConvergenceCriteria(int num_contacts, const VectorX<T>& vc,
                                 const VectorX<T>& vc_kp,
                                 const VectorX<T>& gamma,
                                 const VectorX<T>& gamma_kp, double* vc_err,
                                 double* gamma_err) const;

  /* Returns the impulse in contact space at a single contact point that lies
   in the friction cone given the contact velocity `vc`, the contact impulse
   `gamma` and the friction coefficient `mu`. */
  Vector3<T> ProjectImpulse(const Eigen::Ref<const Vector3<T>>& vc,
                            const Eigen::Ref<const Vector3<T>>& gamma,
                            const T& mu) const;

  PgsSolverParameters parameters_;
  // The preprocessed data that remains constant in SolveWithGuess().
  PreProcessedData pre_proc_data_;
  // The current state of the solver.
  State state_;
  PgsSolverStats stats_;
  // Workspace/scratch data.
  VectorX<T> tau_c_;  // Generalized contact impulses.
  VectorX<T> vc_;     // Contact velocities.
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::PgsSolver);
