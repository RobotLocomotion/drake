#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/point_contact_data.h"
#include "drake/multibody/contact_solvers/system_dynamics_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// The result from ContactSolver::SolveWithGuess() used to report the
// success or failure of the solver.
enum class ContactSolverStatus {
  // Successful computation.
  kSuccess = 0,

  // The solver could not find a solution at the specified tolerances.
  kFailure = 1,
};

// This class defines a general interface for all of our contact solvers. By
// having a common interface, client code such as MultibodyPlant only needs to
// learn how to talk to %ContactSolver, allowing to swap contact solvers that
// share this common interface without having to re-wire the client's
// internals.
//
// <h3> Mechanical Systems and State </h3>
//
// In what follows, we describe the state of the system by the vector of
// generalized positions q, or configuration, and the vector of generalized
// velocities v. q is a vector of size `nq` and v a vector of size `nv`. Even
// though in general `nq != nv`, the kinematic mapping `q̇ = N(q)⋅v` relates
// the generalized velocities to time derivatives of the generalized positions.
//
// In the absence of contact, we write the dynamics of a general mechanical
// system as: <pre>
//   M(q)⋅v̇ = τ(t,x,u)                                               (1)
//        q̇ = N(q)⋅v
// </pre>
// where `x = [q; v]` is the full state of the system, `M(q)` is its mass
// matrix, and the generalized forces `τ(t,x,u)` are further split into a state
// dependent term `τₓ(t,x)` and externally applied actuation `τᵤ(u)` as: <pre>
//   τ(t,x,u) = τₓ(t,x) + τᵤ(u)
//      τᵤ(u) = B⋅u(t) +  Jᵀ(q)⋅Fₑₓₜ
// </pre>
// where `u(t)` applies external __actuation__ to specific mobilities through
// mapping `B`, independent of state and time, from actuation to generalized
// forces. The term `Jᵀ(q)⋅Fₑₓₜ` accounts for externally applied spatial
// forces.
//
// Consider the dynamics of rigid multibody systems. For this case `τₓ(t,x)`
// contains the Coriolis and centrifugal contributions `C(q,v)` (actually
// -C(q,v)), gravity `τg(q)`, and state dependent terms related to the modeling
// of forcing elements such as drag, springs and dampers.
//
// Another example of mechanical system is that of a FEM model for which
// `τₓ(t,x)` contains the contribution due to internal stresses in the
// deformable object. In this case, the configuration `q` will correspond to
// the Lagrangian coordinates of material points in the solid. Moreover q̇ = v,
// i.e. `N(q)` is the identity mapping.
//
// Finally, it is evident that this same framework allows for a model
// containing both rigid and deformable objects.
//
// TODO(amcastro-tri): add sections specific to bilateral and unilateral
// constraints.
//
// <h3> Mechanical Systems with Frictional Contact </h3>
//
// Mechanical systems with contact are subject to additional constraints to
// enforce non-interpenetration and to model friction. Therefore Eq. (1) is
// augmented to: <pre>
//   M(q)⋅v̇ = τ(t,x,u) + Jcᵀ⋅fc                                            (2)
//        q̇ = N(q)⋅v                                                       (3)
//   s.t. Contact constraints.                                             (4)
// </pre>
// where fc concatenates all nc contact forces fcᵢ ∈ ℝ³ into a vector of
// size 3nc and `Jc`, of size `3nc x nv`, is the "contact Jacobian" defined
// such that contact velocities vc are given by vc = Jc⋅v.
//
// With "Contact constraints" we mean:
// - Contact forces follow Coulomb's law of friction, i.e. fcᵢ is inside the
//   friction cone.
// - The friction component of fc, which we refer to as ft, satisfies the
//   principle of maximum dissipation for sliding contacts.
// - The normal component of fc, which we refer to as fn, is non-negative
//   i.e. always a repulsive force (adhesive or “sticky" contact needs special
//   consideration).
// The unknowns in this formulation are the full state `x=[q;v]` and contact
// forces `fc`. These are solved from Eqs. (2)-(3) subject to the contact
// constraints (4).
//
// %ContactSolver's interface leaves open how exactly these constraints are
// imposed so that specific solvers have the freedom to choose other model
// approximations. For instance, we allow solvers to accommodate for the convex
// relaxation of friction [Anitescu, 2006], regularization of constraints
// [Lacoursiere et al. 2011] or compliant contact [Castro et al., 2019]. While
// the general formulation of frictional contact is a Non-linear
// Complementarity Problem (NCP), %ContactSolver's interface makes no
// assumption on the underlying solver therefore also allowing for optimization
// based methods, [Todorov, 2014; Kaufman et al., 2008].
//
// <h3> Time Discretization </h3>
//
// Our objective is to discretize the differential-algebraic system of
// equations (DAEs), Eqs. (2)-(4), in time in order to advance the solution
// from state `x₀=[q₀;v₀]` at time `t₀` to state `x=[q;v]` at time
// `t = t₀ + dt`, where dt is a pre-specified time step.
// The framework proposed below allows the discretization of the continuous
// component of the DAEs, Eq. (1), using a variety of discretization schemes
// with different stability properties and/or order of accuracy.
//
// To be concrete, consider an implicit Euler method applied to Eqs. (2)-(4)
// written as: <pre>
//  M(q₀)⋅v = M(q₀)⋅v₀ + dt⋅τᵤ(u₀) + dt⋅τₓ(t,q,v) + Jcᵀ(q₀)⋅γ              (5)
//     q(v) = q₀ + dt⋅N(q₀)⋅v                                              (6)
//   s.t. Contact constraints.                                             (7)
// </pre>
// where:
// - We defined the contact impulses as `γ = dt⋅fc`.
// - This particular example is not fully implicit in that `M`, `Jcᵀ` and `N`
//   are "frozen" at `t₀`. This is a very popular approximation choice also
//   consistently 1ˢᵗ order with the rest of the fully implicit terms.
// - State dependent forces `τₓ` are fully implicit.
// - Actuation `τᵤ` is "frozen" at `t₀`. This will generally be true given that
//   since these are external to the physics engine, we will have no means to
//   provide either an implicit or higher order approximation.
// - In this particular example, we decided to "freeze" the kinematic mapping
//   at N(q₀), an approximation consistent with the order of accuracy of the
//   scheme.
//
// In general, for any time stepping scheme, we can write: <pre>
//   F(v) = Jcᵀ⋅γ                                                          (8)
//   s.t. Contact constraints.                                             (9)
// </pre>
// For the particular example scheme outlined in Eqs. (5)-(7) we have: <pre>
//        F(v) = M(q₀)⋅(v - v₀) - dt⋅τₓ(t,q(v),v) - dt⋅τᵤ(u₀)
//   with q(v) = q₀ + dt⋅N(q₀)⋅v.
// </pre>
//
// %ContactSolver works with the model equations in the form of Eqs. (8)-(9)
// once a particular time discretization scheme was made, and it is agnostic to
// the particular mechanical system model.
//
// <h3> Solving the Discrete Contact problem </h3>
//
// A general approach for solving the contact problem will include a predictor
// step to compute velocities v* satisfying the predictor equations
// `F(v*) = 0`. That is, v* corresponds to the velocities the system would
// evolve with in the absence of contact forces, see for instance [Duriez,
// 2013] for a case in which `F(v*) = 0` is highly non-linear. The next step
// velocity is then approximated as `v = v* + Δv` where Δv is computed in a
// corrector step satisfying the equations: <pre>
//   F(v* + Δv) = F(v* + Δv) = Jcᵀ⋅γ
//   s.t. Contact constraints
// </pre>
// We can linearize the discrete momentum equation `F(v)` at v*, leading to:
// <pre>
//   F(v*) + A⋅Δv + O(‖Δv‖²) = Jcᵀ⋅γ
// </pre>
// where we defined A = ∂F/∂v(v*) as the Jacobian of F with respect to
// generalized velocities v, evaluated at v*. From now on we will neglect the
// term O(‖Δv‖²) leading to a first order approximation for the impulses. Since
// v* satisfies the predictor's equation F(v*) = 0, the equation for the
// impulses simplifies to: <pre>
//   A⋅Δv = Jcᵀ⋅γ
//   s.t. Contact constraints
// </pre>
// Notice that even when this approximation is O(dt) for the impulses, the
// predictor step to compute v* from `F(v*) = 0` can use a higher order time
// discretization scheme. A low order scheme on the constraints has a very
// desired stabilization side effect, since higher order schemes might lead to
// instabilities. However, we can still use a high order scheme on the
// continuous terms of the dynamics represented in `F(v)`.
//
// As an example of application, consider the rigid multibody dynamics
// equations discretized using an explicit approach for all non-contact forces,
// as for instance in [Castro et al., 2019]. In this case F(v) takes the
// form: <pre>
//   F(v) = M(q₀)⋅(v−v₀) − dt⋅τ₀
// </pre>
// where τ₀ includes external forces as well as Coriolis and centrifugal terms.
// In this case A = ∇F = M, v* = v₀ + dt⋅M⁻¹⋅τ₀.
//
// As a second example, consider the simulation of deformable bodies with
// frictional contact for which, without diving into the details, the momentum
// equations can be briefly summarized as:
// <pre>
//   F(v) = M⋅(v−v₀) + dt⋅Fᵢₙₜ(q(v), v)
//   q(v) = q₀ + dt⋅v
// </pre>
// where with `Fᵢₙₜ(q, v)` we denote the force terms containing the
// contribution due to internal stresses in the deformable object. Using a
// predictor as in [Duriez, 2013], `F(v*) = 0`, leads to the system's dynamics
// matrix `A(v*) = M + dt⋅D + dt²⋅K`, where `M` is the mass matrix, `D =
// ∂Fᵢₙₜ/∂v` is the damping matrix and `K = ∂Fᵢₙₜ/∂q` is the stiffness matrix.
// For the modeling of large deformations `Fᵢₙₜ(q, v)` is a non-linear function
// of both q and v and therefore v* requires the solution of the non-linear
// system of equations `Fᵢₙₜ(q(v*), v*) = 0` usually with a Newton method. As a
// side effect of this solution the operator form of `A⁻¹` at v* will be
// available, typically as a factorization of the sparse matrix `A`.
//
// <h3> Problem data </h3>
// %ContactSolver needs the following information to properly define the
// contact problem:
// - The inverse operator A⁻¹ and the predicted velocity v*. This essentially
//   defines the "dynamics" of the system. This is specified as a
//   SystemDynamicsData with SetSystemDynamicsData().
// - Contact information. Initial penetration, contact Jacobian, possibly
//   stiffness and damping, friction coefficients, etc. This is specified as a
//   PointContactData with SetPointContactData().
//
// Once the problem is set, SolveWithGuess() is used to invoke the solver.
// Methods such as CopyImpulses() and CopyVelocities() are then used to
// retrieve the solution γ and v, respectively.
//
// <h3> References: </h3>
// - [Anitescu, 2006] Anitescu, M., 2006. Optimization-based simulation of
// nonsmooth rigid multibody dynamics. Mathematical Programming, 105(1),
// pp.113-143.
// - [Castro et al., 2019] Castro, A.M., Qu, A., Kuppuswamy, N., Alspach, A.
// and Sherman, M., 2020. A Transition-Aware Method for the Simulation of
// Compliant Contact with Regularized Friction. IEEE Robotics and Automation
// Letters, 5(2), pp.1859-1866.
// - [Duriez, 2013] Duriez, C., 2013. Real-time haptic simulation of medical
// procedures involving deformations and device-tissue interactions (Doctoral
// dissertation).
// - [Kaufman et al., 2008] Kaufman, D.M., Sueda, S., James, D.L. and Pai,
// D.K., 2008. Staggered projections for frictional contact in multibody
// systems. In ACM SIGGRAPH Asia 2008 papers (pp. 1-11).
// - [Lacoursiere et al. 2011] Lacoursiere, C. and Linde, M., 2011. Spook: a
// variational time-stepping scheme for rigid multibody systems subject to dry
// frictional contacts. UMINF report, 11.
// - [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
// dynamics with contacts and constraints: Theory and implementation in MuJoCo.
// In 2014 IEEE International Conference on Robotics and Automation (ICRA) (pp.
// 6054-6061). IEEE.
//
// @tparam_nonsymbolic_scalar
template <typename T>
class ContactSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactSolver);
  ContactSolver() = default;
  virtual ~ContactSolver() = default;

  // Generic interface to invoke the contact solver given an initial guess
  // `v_guess`.
  // @param time_step Length of the time interval in which impulses act.
  // @param dynamics_data Provides pointers to the dynamics data of the system.
  // @param contact_data Provides pointers to the discrete contact set.
  // @param v_guess Initial guess for the solver.  Some solvers might decide to
  // ingore this guess, refer to each solver specific documentation to find out
  // how this gets used.
  // @param results On output it must store the solution to the contact problem.
  virtual ContactSolverStatus SolveWithGuess(
      const T& time_step, const SystemDynamicsData<T>& dynamics_data,
      const PointContactData<T>& contact_data, const VectorX<T>& v_guess,
      ContactSolverResults<T>* result) = 0;

 protected:
  // Helper method to form the Delassus operator. Most solvers will need to
  // form it whether if used directly, as part of a pre-processing stage or to
  // just determine scaling factors.
  //
  // Computes W = G * Ainv * Jᵀ each j-th column at a time by multiplying with
  // basis vector ej (zero vector with a "1" at the j-th element).
  //
  // Typically Ainv will be the linear operator corresponding to the inverse of
  // the dynamics matrix A as described in this class's documentation.
  // J and G will usually correspond to the contact constraints Jacobian Jc as
  // described in the class's documentation, though some schemes might build a
  // different approximation of W in which J and G are different.
  //
  // @pre G must have size 3nc x nv.
  // @pre Ainv must have size nv x nv.
  // @pre J must have size 3nc x nv.
  // @pre J must provide an implementation to MultiplyByTranspose().
  // @pre W is not nullptr and is of size 3nc x 3nc.
  void FormDelassusOperatorMatrix(const LinearOperator<T>& G,
                                  const LinearOperator<T>& Ainv,
                                  const LinearOperator<T>& J,
                                  Eigen::SparseMatrix<T>* W) const {
    const int num_velocities = Ainv.rows();
    const int num_impulses = J.rows();
    DRAKE_DEMAND(G.rows() == num_impulses);
    DRAKE_DEMAND(G.cols() == num_velocities);
    DRAKE_DEMAND(Ainv.rows() == num_velocities);
    DRAKE_DEMAND(Ainv.cols() == num_velocities);
    DRAKE_DEMAND(J.rows() == num_impulses);
    DRAKE_DEMAND(J.cols() == num_velocities);
    DRAKE_DEMAND(W->rows() == num_impulses);
    DRAKE_DEMAND(W->cols() == num_impulses);

    Eigen::SparseVector<T> ej(num_impulses);
    // N.B. ej.makeCompressed() is not available for SparseVector.
    ej.coeffRef(0) = 1.0;  // Effectively allocate one non-zero entry.

    Eigen::SparseVector<T> JTcolj(num_velocities);
    Eigen::SparseVector<T> AinvJTcolj(num_velocities);
    Eigen::SparseVector<T> Wcolj(num_impulses);
    // Reserve maximum number of non-zeros.
    JTcolj.reserve(num_velocities);
    AinvJTcolj.reserve(num_velocities);
    Wcolj.reserve(num_impulses);

    // Loop over the j-th column.
    for (int j = 0; j < W->cols(); ++j) {
      // By changing the inner index, we change what entry is the non-zero with
      // value 1.0.
      *ej.innerIndexPtr() = j;

      // Reset to nnz = 0. Memory is not freed.
      JTcolj.setZero();
      AinvJTcolj.setZero();
      Wcolj.setZero();

      // Apply each operator in sequence.
      J.MultiplyByTranspose(ej, &JTcolj);  // JTcolj = Jᵀ * ej
      Ainv.Multiply(JTcolj, &AinvJTcolj);
      G.Multiply(AinvJTcolj, &Wcolj);
      W->col(j) = Wcolj;
    }
  }
};
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::ContactSolver)
