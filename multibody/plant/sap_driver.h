#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

// Forward declaration.
template <typename>
class MultibodyPlant;

namespace internal {

// Forward declaration.
template <typename>
class CompliantContactManager;

template <typename T>
struct ContactProblemCache {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactProblemCache);
  // Cache entry to an empty contact problem.
  explicit ContactProblemCache(double time_step) {
    sap_problem =
        std::make_unique<contact_solvers::internal::SapContactProblem<T>>(
            time_step, std::vector<MatrixX<T>>(), VectorX<T>());
    // `sap_problem_locked` is a transformation of the problem stored in
    // `sap_problem` that eliminates DoFs and constraints given by joint locking
    // data from the plant. When joint locking is preset, the locked problem
    // is solved and its results expanded into results for the original.
    sap_problem_locked =
        std::make_unique<contact_solvers::internal::SapContactProblem<T>>(
            time_step, std::vector<MatrixX<T>>(), VectorX<T>());
  }
  copyable_unique_ptr<contact_solvers::internal::SapContactProblem<T>>
      sap_problem;
  // Start/end constraint index for PD controller constraints in sap_problem.
  int pd_controller_constraints_start{0};
  int num_pd_controller_constraints{0};

  copyable_unique_ptr<contact_solvers::internal::SapContactProblem<T>>
      sap_problem_locked;

  // TODO(amcastro-tri): consider removing R_WC from the contact problem cache
  // and instead cache DiscreteContactPair separately.
  std::vector<math::RotationMatrix<T>> R_WC;

  contact_solvers::internal::ReducedMapping mapping;
};

// Performs the computations needed by CompliantContactManager for discrete
// updates using the SAP solver. A const manager is provided at construction so
// that the driver has access to the const model and computation services
// agnostic to the solver type, such as geometry queries and/or kinematics.
// Mutable access to the manager is provided during DeclareCacheEntries() to
// allow the declaration of system-level cache entries.
// @tparam_nonsymbolic_scalar
template <typename T>
class SapDriver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapDriver);

  // Constructs a driver with the provided `near_rigid_threshold`.
  // This is a dimensionless positive parameter typically in the range [0.0,
  // 1.0] that controls the amount of regularization used to avoid
  // ill-conditioning. Refer to [Castro et al., 2021] for details. A value of
  // zero effectively turns-off this additional regularization.
  // @pre manager is not nullptr.
  // @pre near_rigid_threshold is positive.
  explicit SapDriver(const CompliantContactManager<T>* manager,
                     double near_rigid_threshold = 1.0);

  ~SapDriver();

  void set_sap_solver_parameters(
      const contact_solvers::internal::SapSolverParameters& parameters);

  // With this function the manager provided at construction gives `this` driver
  // the opportunity to declare system level cache entries.
  // @pre `mutable_manager` must point to the same manager provided at
  // construction.
  void DeclareCacheEntries(CompliantContactManager<T>* mutable_manager);

  // Computes the SAP solver results to advance the discrete dynamics from the
  // state stored in `context`.
  void CalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const;

  // Computes the aggregated multibody forces to step the discrete dynamics from
  // the state in `context`. This includes force elements evaluated at
  // `context`, implicit joint reflected inertia and damping and constraint
  // forces.
  void CalcDiscreteUpdateMultibodyForces(const systems::Context<T>& context,
                                         MultibodyForces<T>* forces) const;

  // Computes the actuation applied to the multibody system when stepping the
  // discrete dynamics from the state stored in `context`. This includes the
  // actuation from implicit PD controllers.
  void CalcActuation(const systems::Context<T>& context,
                     VectorX<T>* actuation) const;

  // Evaluates a cache entry storing the SapContactProblem to be solved at the
  // state stored in `context`.
  const ContactProblemCache<T>& EvalContactProblemCache(
      const systems::Context<T>& context) const;

 private:
  // Provide private access for unit testing only.
  friend class SapDriverTest;

  const CompliantContactManager<T>& manager() const { return *manager_; }

  const MultibodyPlant<T>& plant() const { return manager().plant(); }

  const SpanningForest& get_forest() const { return manager().get_forest(); }

  // Computes the linearized momentum equation matrix A to build the SAP
  // contact problem. The matrices for deformable bodies come after those for
  // rigid bodies. Refer to SapContactProblem's class documentation for details.
  void CalcLinearDynamicsMatrix(const systems::Context<T>& context,
                                std::vector<MatrixX<T>>* A) const;

  // Given the previous state x0 stored in `context`, this method computes the
  // "free motion" velocities, denoted v*.
  void CalcFreeMotionVelocities(const systems::Context<T>& context,
                                VectorX<T>* v_star) const;

  // Adds contact constraints for the configuration stored in `context` into
  // `problem`. This method returns the orientation of the contact frame in the
  // world frame for each contact constraint added to `problem`. That is, the
  // i-th entry in the return vector corresponds to the orientation R_WC contact
  // frame in the world frame for the i-th contact constraint added to
  // `problem`.
  std::vector<math::RotationMatrix<T>> AddContactConstraints(
      const systems::Context<T>& context,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // Adds limit constraints for the configuration stored in `context` into
  // `problem`. Limit constraints are only added when the state q₀ for a
  // particular joint is "close" to the joint's limits (qₗ,qᵤ). To decide when
  // the state q₀ is close to the joint's limits, this method estimates a window
  // (wₗ,wᵤ) for the expected value of the configuration q at the next time
  // step. Lower constraints are considered whenever qₗ > wₗ and upper
  // constraints are considered whenever qᵤ < wᵤ. This window (wₗ,wᵤ) is
  // estimated based on the current velocity v₀ and the free motion velocities
  // v*, provided with `v_star`.
  // Since the implementation uses the current velocity v₀ to estimate whether
  // the constraint should be enabled, it is at least as good as a typical
  // continuous collision detection method. It could mispredict under conditions
  // of strong acceleration (it is assuming constant velocity across a step).
  // Still, at typical robotics step sizes and rates it would be surprising to
  // see that happen, and if it did the limit would come on in the next step.
  // TODO(amcastro-tri): Consider using the acceleration at t₀ to get a second
  // order prediction for the configuration at the next time step.
  // @pre problem must not be nullptr.
  void AddLimitConstraints(
      const systems::Context<T>& context, const VectorX<T>& v_star,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // Adds holonomic constraints to model couplers specified in the
  // MultibodyPlant.
  void AddCouplerConstraints(
      const systems::Context<T>& context,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // Adds holonomic constraints to model distance constraints specified in the
  // MultibodyPlant.
  void AddDistanceConstraints(
      const systems::Context<T>& context,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // Adds holonomic constraint equations to model ball constraints specified in
  // the MultibodyPlant.
  // @throws std::exception if both bodies have invalid tree indices from the
  // tree topology (i.e. both bodies are welded to `world`).
  void AddBallConstraints(
      const systems::Context<T>& context,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // Adds holonomic constraint equations to model weld constraints specified in
  // the MultibodyPlant.
  // @throws std::exception if both bodies have invalid tree indices from the
  // tree topology (i.e. body bodies are welded to `world`).
  void AddWeldConstraints(
      const systems::Context<T>& context,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  void AddPdControllerConstraints(
      const systems::Context<T>& context,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // Adds holonomic constraints to model fixed constraints for deformable bodies
  // specified in the DeformableModel.
  void AddFixedConstraints(
      const systems::Context<T>& context,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // Adds unilateral holonomic constraints to model tendons constraints
  // specified in the MultibodyPlant.
  // @pre All joints for each constraint have a valid tree index.
  // @throws std::exception if more than two kinematic trees are represented by
  // the joints of any one tendon constraint.
  void AddTendonConstraints(
      const systems::Context<T>& context,
      contact_solvers::internal::SapContactProblem<T>* problem) const;

  // This method takes SAP results for a given `problem` and loads forces due to
  // contact only into `contact_results`. `contact_results` is properly resized
  // on output.
  // @pre contact_results is not nullptr.
  // @pre All `num_contacts` contact constraints in `problem` were added before
  // any other SAP constraint. This requirement is imposed by this driver which
  // adds constraints (with AddContactConstraints()) to the contact problem
  // before any other constraints are added. See the implementation of
  // CalcContactProblemCache(), which is responsible for adding constraints in
  // this particular order.
  void PackContactSolverResults(
      const systems::Context<T>& context,
      const contact_solvers::internal::SapContactProblem<T>& problem,
      int num_contacts,
      const contact_solvers::internal::SapSolverResults<T>& sap_results_locked,
      contact_solvers::internal::ContactSolverResults<T>* contact_results)
      const;

  // Adds the contribution from `clique_values` for `clique` into the full
  // vector of generalized velocities/forces for the entire system (including
  // both rigid and deformable dofs).
  // @param[in] clique  Index for a clique of generalized velocities. All trees
  //                    in the multibody system are indexed first, followed by
  //                    all deformable bodies.
  // @pre size of `clique_values` must be compatible with the number of dofs in
  // `clique`.
  void AddCliqueContribution(const systems::Context<T>& context, int clique,
                             const Eigen::Ref<const VectorX<T>>& clique_values,
                             EigenPtr<VectorX<T>> values) const;

  // Adds the contact forces on degrees of freedom in `clique` to the given
  // generalized contact force vector, `tau_contact`.
  // @pre 0 <= clique < number of cliques in the SAP problem in `context`.
  // @param[in] context           The context of the MultibodyPlant.
  // @param[in] clique            The clique index of the degrees of freedom of
  //                              interest.
  // @param[in] contact_jacobian  The contact jacobian J, must have exactly 3
  //                              rows.
  // @param[in] impulse           The contact impulse at the contact point.
  // @param[in, out] tau_contact  The contact force for all degrees of freedom
  //                              in the SAP problem to be written to.
  void AddToGeneralizedContactForce(const systems::Context<T>& context,
                                    int clique,
                                    const MatrixX<T>& contact_jacobian,
                                    const Vector3<T>& impulse,
                                    EigenPtr<VectorX<T>> tau_contact) const;

  // Computes the necessary data to describe the SAP contact problem. Additional
  // information such as the orientation of each contact frame in the world is
  // also computed here so that it can be used at a later stage to compute
  // contact results.
  // All contact constraints are added before any other constraint types. This
  // manager assumes this ordering of the constraints in order to extract
  // contact impulses for reporting contact results.
  void CalcContactProblemCache(const systems::Context<T>& context,
                               ContactProblemCache<T>* cache) const;

  // Computes the discrete update from the state stored in the context. The
  // resulting next time step velocities and constraint impulses are stored in
  // `sap_results`.
  void CalcSapSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::SapSolverResults<T>* sap_results) const;

  // Eval version of  SapSolverResults().
  const contact_solvers::internal::SapSolverResults<T>& EvalSapSolverResults(
      const systems::Context<T>& context) const;

  // The driver only has mutable access at construction time, when it can
  // declare additional state, cache entries, ports, etc. After construction,
  // the driver only has const access to the manager.
  const CompliantContactManager<T>* const manager_{nullptr};
  // Near rigid regime parameter for contact constraints.
  const double near_rigid_threshold_;
  systems::CacheIndex contact_problem_;
  systems::CacheIndex sap_results_;
  // Parameters for SAP.
  contact_solvers::internal::SapSolverParameters sap_parameters_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
