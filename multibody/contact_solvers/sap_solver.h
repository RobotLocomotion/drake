#pragma once

#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// This class implements the Semi-Analytic Primal (SAP) solver described in
// [Castro et al., 2021].
//
// SAP uses the convex approximation of contact constraints by [Anitescu, 2006],
// with constraint regularization and analytical inverse dynamics introduced by
// [Todorov, 2014]. However SAP introduces a primal formulation in velocities
// instead of in impulses as done in previous work. This leads to a numerical
// scheme that warm-starts very effectively using velocities from the previous
// time step. In addition, SAP uses regularization to model physical compliance
// rather than to introduce constraint stabilization as previously done by
// [Todorov, 2014]. Please refer to [Castro et al., 2021] for details.
//
// - [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
//   Unconstrained Convex Formulation of Compliant Contact. Available online at
//   https://arxiv.org/abs/2110.10107.
// - [Anitescu, 2006] Anitescu M., 2006. Optimization-based simulation of
//   nonsmooth rigid multibody dynamics. Mathematical Programming, 105(1),
//   pp.113-143.
// - [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
//   dynamics with contacts and constraints: Theory and implementation in
//   MuJoCo. In 2014 IEEE International Conference on Robotics and Automation
//   (ICRA) (pp. 6054-6061). IEEE.
//
// TODO(amcastro-tri): enable AutoDiffXd support, if only for dense matrices so
// that we can test the long term performant solution.
// @tparam_double_only
template <typename T>
class SapSolver final : public ContactSolver<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapSolver);

  SapSolver() = default;
  ~SapSolver() final = default;

  // Solve the contact problem specified by the input data. See
  // ContactSolver::SolveWithGuess() for details. Currently, only `T = double`
  // is supported. An exception is thrown if `T != double`.
  ContactSolverStatus SolveWithGuess(const T& time_step,
                                     const SystemDynamicsData<T>& dynamics_data,
                                     const PointContactData<T>& contact_data,
                                     const VectorX<T>& v_guess,
                                     ContactSolverResults<T>* result) final;

 private:
  friend class SapSolverTester;

  // Structure used to store input data pre-processed for computation. For
  // mathematical quantities, we attempt to follow the notation introduced in
  // [Castro et al., 2021] as best as we can, though with ASCII and Unicode
  // symbols.
  struct PreProcessedData {
    // Constructs an empty data.
    PreProcessedData() = default;

    // @param nv Number of generalized velocities.
    // @param nc Number of contact constraints.
    // @param dt The discrete time step used for simulation.
    PreProcessedData(double dt, int nv_in, int nc_in) : time_step(dt) {
      Resize(nv_in, nc_in);
    }

    // @param nv_in Number of generalized velocities.
    // @param nc_in Number of contact constraints.
    void Resize(int nv_in, int nc_in) {
      nv = nv_in;
      nc = nc_in;
    }

    T time_step{NAN};             // Discrete time step used by the solver.
    int nv{0};                    // Number of generalized velocities.
    int nc{0};                    // Number of contacts.
    BlockSparseMatrix<T> J;       // Jacobian as block-sparse matrix.
  };

  // Pack solution into ContactSolverResults. Where v is the vector of
  // generalized velocities, vc is the vector of contact velocities and gamma is
  // the vector of generalized contact impulses.
  static void PackContactResults(const PreProcessedData& data,
                                 const VectorX<T>& v, const VectorX<T>& vc,
                                 const VectorX<T>& gamma,
                                 ContactSolverResults<T>* result);
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

extern template class ::drake::multibody::contact_solvers::internal::SapSolver<
    double>;
