#pragma once

#include <fstream>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/plant/discrete_contact_data.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {

using geometry::GeometryId;
using math::RigidTransform;
using multibody::BodyIndex;
using multibody::JacobianWrtVariable;
using multibody::MultibodyForces;
using multibody::MultibodyPlant;
using multibody::contact_solvers::internal::BlockSparseMatrix;
using multibody::contact_solvers::internal::HessianFactorizationCache;
using multibody::contact_solvers::internal::SapContactProblem;
using multibody::contact_solvers::internal::SapHessianFactorizationType;
using multibody::contact_solvers::internal::SapModel;
using multibody::contact_solvers::internal::SapSolverParameters;
using multibody::contact_solvers::internal::SapSolverResults;
using multibody::contact_solvers::internal::SapSolverStatus;
using multibody::contact_solvers::internal::SapStatistics;
using multibody::internal::DiscreteContactData;
using multibody::internal::DiscreteContactPair;
using multibody::internal::GetInternalTree;
using multibody::internal::MultibodyTree;
using multibody::internal::MultibodyTreeTopology;

/**
 * An improved version of HessianFactorizationCache that keeps track of the
 * number of constraints. This allows us to re-use the factorization whenever
 * possible.
 */
class HessianFactorization : public HessianFactorizationCache {
 public:
  HessianFactorization() = default;

  HessianFactorization(SapHessianFactorizationType type,
                       const std::vector<MatrixX<double>>* A,
                       const BlockSparseMatrix<double>* J,
                       const SapModel<double>& model)
      : HessianFactorizationCache(type, A, J) {
    num_cliques_ = model.num_cliques();
    num_velocities_ = model.num_velocities();
    num_constraints_ = model.num_constraints();
    num_constraint_equations_ = model.num_constraint_equations();
  }

  // Check whether the sparsity pattern/constraints match the current model.
  bool matches(const SapModel<double>& model) const {
    return num_cliques_ == model.num_cliques() &&
           num_velocities_ == model.num_velocities() &&
           num_constraints_ == model.num_constraints() &&
           num_constraint_equations_ == model.num_constraint_equations();
  }

 private:
  int num_cliques_{0};
  int num_velocities_{0};
  int num_constraints_{0};
  int num_constraint_equations_{0};
};

/**
 * An experimental implicit integrator that solves a convex SAP problem to
 * advance the state, rather than relying on non-convex Newton-Raphson.
 */
template <class T>
class ConvexIntegrator final : public IntegratorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConvexIntegrator);

  ~ConvexIntegrator() override = default;

  /**
   * Constructs the experimental convex integrator.
   *
   * @param system the overall system diagram to simulate. Must be a Diagram
   *               with a MultibodyPlant as a subsystem (and MbP must be the
   *               only subsystem with interesting dynamics.)
   * @param context context for the overall system.
   *
   * N.B. Although this is an implicit integration scheme, we inherit from
   * IntegratorBase rather than ImplicitIntegrator because the way we compute
   * the Jacobian (Hessian for us) is completely different, and MultibodyPlant
   * specific.
   */
  explicit ConvexIntegrator(const System<T>& system,
                            Context<T>* context = nullptr);

  bool supports_error_estimation() const final { return true; }

  // TODO(vincekurtz): figure out the actual order of this error estimate. Right
  // now this is a guess based on ImplicitEulerIntegrator.
  int get_error_estimate_order() const final { return 2; }

  // Get a reference to the plant used for SAP computations
  const MultibodyPlant<T>& plant() const { return *plant_; }

  // Get the number of Hessian factorizations
  int get_num_hessian_factorizations() const {
    return num_hessian_factorizations_;
  }

  // Get the number of (convex) newton iterations
  int get_num_solver_iterations() const { return num_solver_iterations_; }

  // Enable (false) or disable (true) Hessian re-use between iterations and time
  // steps. Naming convention follows ImplicitIntegrator.
  void set_use_full_newton(bool flag) { use_full_newton_ = flag; }

  // Check whether Hessian re-use is enabled.
  bool get_use_full_newton() const { return use_full_newton_; }

  // Enable/disable writing iteration data to a CSV file
  void set_write_to_csv(bool flag) { write_to_csv_ = flag; }

 private:
  friend class ConvexIntegratorTester;

  // Struct used to store the result of computing the search direction. Clone of
  // SapSolver::SearchDirectionData
  struct SearchDirectionData {
    SearchDirectionData(int num_velocities, int num_constraint_equations) {
      dv.resize(num_velocities);
      dp.resize(num_velocities);
      dvc.resize(num_constraint_equations);
      d2ellA_dalpha2 = NAN;
    }
    VectorX<T> dv;          // Search direction.
    VectorX<T> dp;          // Momentum update Δp = A⋅Δv.
    VectorX<T> dvc;         // Constraints velocities update, Δvc=J⋅Δv.
    T d2ellA_dalpha2{NAN};  // d²ellA/dα² = Δvᵀ⋅A⋅Δv.
  };

  // Allocate the workspace
  void DoInitialize() final;

  // The main integration step, sets x_{t+h} and the error estimate in
  // this->context.
  bool DoStep(const T& h) override;

  // Solve the SAP problem to compute x_{t+h} at a given step size. This will be
  // called multiple times for each DoStep to compute the error estimate.
  //
  // @param h the time step to use
  // @param v_guess the initial guess for the MbP plant velocities.
  // @param x_next the output continuous state, includes both the plant and any
  //        external systems
  void CalcNextContinuousState(const T& h, const VectorX<T>& v_guess,
                               ContinuousState<T>* x_next);

  // Create the sap problem, including contact constraints, for a particular
  // step size h.
  SapContactProblem<T> MakeSapContactProblem(const Context<T>& context,
                                             const T& h);

  // Adds contact constraints to the SAP problem.
  void AddContactConstraints(const Context<T>& context,
                             SapContactProblem<T>* problem);

  // Adds dummy constraints to the SAP problem. These do nothing, but ensure
  // that all DoFs are participating and the size of the problem won't vary
  // between time steps.
  void AddDummyConstraints(SapContactProblem<T>* problem) const;

  // Compute signed distances and jacobians. While we store this in a
  // DiscreteContactData struct (basically copying DiscreteUpdateManager), this
  // is computed using the plant's continuous state.
  void CalcContactPairs(
      const Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* result) const;

  // Copied from DiscreteContactManager, but using the continuous state
  void AppendDiscreteContactPairsForPointContact(
      const Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* result) const;

  // Copied from DiscreteContactManager, but using the continuous state
  void AppendDiscreteContactPairsForHydroelasticContact(
      const Context<T>& context,
      DiscreteContactData<DiscreteContactPair<T>>* result) const
    requires scalar_predicate<T>::is_bool;

  // Clone of SapSolver::SolveWithGuessImpl, but allows for Hessian re-use.
  SapSolverStatus SolveWithGuess(const SapContactProblem<T>& problem,
                                 const VectorX<T>& v_guess,
                                 SapSolverResults<T>* result);

  // Clone of SapSolver::SolveWithGuessImpl, but allows for Hessian re-use.
  SapSolverStatus SolveWithGuessImpl(const SapModel<T>& model,
                                     Context<T>* context)
    requires std::is_same_v<T, double>;

  // Clone of SapSolver::PackSapSolverResults
  void PackSapSolverResults(const SapModel<T>& model, const Context<T>& context,
                            SapSolverResults<T>* results) const;

  // Clone of SapSolver::CalcStoppingCriteriaResidual
  void CalcStoppingCriteriaResidual(const SapModel<T>& model,
                                    const Context<T>& context,
                                    T* momentum_residual,
                                    T* momentum_scale) const;

  // Clone of SapSolver::CalcSearchDirectionData, but allows for selective
  // Hessian updates.
  void CalcSearchDirectionData(const SapModel<T>& model,
                               const Context<T>& context,
                               SearchDirectionData* search_direction_data)
    requires std::is_same_v<T, double>;

  // Update the Hessian factorization based on the given SAP model. This is a
  // loose clone of SapModel::CalcHessianFactorizationCache.
  void CalcHessianFactorization(const SapModel<T>& model,
                                const Context<T>& context,
                                HessianFactorization* hessian)
    requires std::is_same_v<T, double>;

  // Clone of SapSolver::PerformExactLineSearch
  std::pair<T, int> PerformExactLineSearch(
      const SapModel<T>& model, const systems::Context<T>& context,
      const SearchDirectionData& search_direction_data,
      systems::Context<T>* scratch_workspace) const
    requires std::is_same_v<T, double>;

  // Clone of SapSolver::CalcCostAlongLine
  T CalcCostAlongLine(const SapModel<T>& model,
                      const systems::Context<T>& context,
                      const SearchDirectionData& search_direction_data,
                      const T& alpha, systems::Context<T>* scratch,
                      T* dell_dalpha = nullptr, T* d2ell_dalpha2 = nullptr,
                      VectorX<T>* d2ell_dalpha2_scratch = nullptr) const;

  // Linearize the external (e.g. controller) system around the current state.
  //
  // The original nonlinear controller
  //     τ = B u = B g(x)
  // is approximated as
  //     τ = τ₀ − Ãv,
  // where Ã is symmetric and positive definite, and we use the fact that q = q0
  // + h N v to write everything in terms of velocities.
  //
  // We do the linearization via finite differences
  void LinearizeExternalSystem(const T& h, MatrixX<T>* A_tilde,
                               VectorX<T>* tau0);

  // Project the given (square) matrix to a nearby symmetric positive
  // (semi)-definite matrix.
  void ProjectSPD(MatrixX<T>* M) const;

  // Tree topology used for defining the sparsity pattern in A.
  const MultibodyTreeTopology& tree_topology() const {
    return GetInternalTree(plant()).get_topology();
  }

  // Various methods exposed from the plant
  const MultibodyTree<T>& internal_tree() const {
    return plant().internal_tree();
  }

  double default_contact_stiffness() const {
    return plant().penalty_method_contact_parameters_.geometry_stiffness;
  }

  double default_contact_dissipation() const {
    return plant().penalty_method_contact_parameters_.dissipation;
  }

  BodyIndex FindBodyByGeometryId(GeometryId geometry_id) const {
    return plant().FindBodyByGeometryId(geometry_id);
  }

  // Intermediate states for error control, which compares a single large
  // step (x_next_full_) to the result of two smaller steps (x_next_half_2_).
  std::unique_ptr<ContinuousState<T>> x_next_full_;    // x_{t+h}
  std::unique_ptr<ContinuousState<T>> x_next_half_1_;  // x_{t+h/2}
  std::unique_ptr<ContinuousState<T>> x_next_half_2_;  // x_{t+h/2+h/2}

  // Stored Hessian factorization. Computing this is expensive, so we reuse it
  // whenever possible.
  HessianFactorization hessian_factorization_;
  std::vector<MatrixX<T>> A_;  // Hack to keep these from going out of scope
  BlockSparseMatrix<T> J_;

  // Flag for enabling/disabling hessian re-use
  bool use_full_newton_{false};

  // Flag for Hessian factorization re-use (changes between iterations)
  bool refresh_hessian_{false};

  // Various plotting/debugging tools
  bool write_to_csv_{false};
  std::ofstream csv_file_;
  T time_{0.0};
  T time_step_{0.0};
  int solve_phase_{0};  // 0 = full , 1 = first half-step, 2 = second half-step

  // Plant model, since convex integration is specific to MbP
  const MultibodyPlant<T>* plant_;

  // SAP solver parameters
  SapSolverParameters sap_parameters_;

  // SAP solver statistics
  mutable SapStatistics sap_stats_;

  // Simulator statistics
  int64_t num_hessian_factorizations_{0};
  int64_t num_solver_iterations_{0};

  // Scratch space for intermediate calculations
  struct Workspace {
    // Used in DoStep
    VectorX<T> v;  // generalized velocities

    // Used in CalcNextContinuousState
    VectorX<T> q;                     // generalized positions
    VectorX<T> z;                     // controller states
    SapSolverResults<T> sap_results;  // Container for convex solve results

    // Used in MakeSapContactProblem
    MatrixX<T> A_dense;         // dense linear dynamics matrix
    std::vector<MatrixX<T>> A;  // linear dynamics matrix
    VectorX<T> v_star;          // velocities of the unconstrained system
    MatrixX<T> M;               // mass matrix
    VectorX<T> k;               // coriolis terms from inverse dynamics
    std::unique_ptr<MultibodyForces<T>> f_ext;  // external forces (gravity)
    MatrixX<T> A_tilde;  // SPD Hessian correction term, A_tilde = h P + h^2 Q
    VectorX<T> tau0;     // Explicit external forces, tau0 = B g0 + P v0

    // Used in LinearizeExternalSystem
    VectorX<T> g0;
    MatrixX<T> B;
    MatrixX<T> D;
    MatrixX<T> N;
    MatrixX<T> P;
    MatrixX<T> Q;

    // Used in AddContactConstraint
    DiscreteContactData<DiscreteContactPair<T>> contact_data;
  } workspace_;
};

// Forward-declare specializations, prior to DRAKE_DECLARE... below.
// We use these to specialize functions that do not support AutoDiffXd.
template <>
SapSolverStatus ConvexIntegrator<double>::SolveWithGuess(
    const SapContactProblem<double>&, const VectorX<double>&,
    SapSolverResults<double>*);
template <>
SapSolverStatus ConvexIntegrator<AutoDiffXd>::SolveWithGuess(
    const SapContactProblem<AutoDiffXd>&, const VectorX<AutoDiffXd>&,
    SapSolverResults<AutoDiffXd>*);

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
