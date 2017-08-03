#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include "drake/multibody/rigid_contact/rigid_contact_problem_data.h"
#include "drake/solvers/moby_lcp_solver.h"

namespace drake {
namespace multibody {
namespace rigid_contact {

/// Solves rigid contact problems for contact forces. Specifically, given
/// problem data corresponding to one or more rigid or multi-rigid bodies
/// constrained unilaterally and acted upon by Coulomb friction, this class
/// computes the contact forces.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// They are already available to link against in the containing library.
template <typename T>
class RigidContactSolver {
 public:
  RigidContactSolver() = default;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RigidContactSolver)

  /// Solves the appropriate contact problem at the acceleration level.
  /// @param cfm The non-negative regularization factor to apply to the contact
  ///            problem (i.e., the underlying complementarity problem), also
  ///            known as the "constraint force mixing" parameter.
  /// @param problem_data The data used to compute the contact forces.
  /// @param cf The computed contact forces, on return, in a packed storage
  ///           format. The first `nc` elements of @p cf correspond to the
  ///           magnitudes of the contact forces applied along the normals of
  ///           the `nc` contact points. The remaining elements of @p cf
  ///           correspond to the frictional forces along the `r` spanning
  ///           directions at each non-sliding point of contact. The first `r`
  ///           values (after the initial `nc` elements) correspond to the first
  ///           non-sliding contact, the next `r` values correspond to the
  ///           second non-sliding contact, etc.
  /// @pre Contact data has been computed.
  /// @throws a std::runtime_error if the contact forces cannot be computed
  ///         (due to, e.g., an "inconsistent" rigid contact configuration).
  /// @throws a std::logic_error if @p cf is null or @p cfm is negative.
  void SolveContactProblem(double cfm,
      const RigidContactAccelProblemData<T>& problem_data,
      VectorX<T>* cf) const;

  /// Solves the appropriate impact problem at the velocity level.
  /// @param cfm The non-negative regularization factor to apply to the impact
  ///            problem (i.e., the underlying complementarity problem), also
  ///            known as the "constraint force mixing" parameter.
  /// @param problem_data The data used to compute the impulsive contact forces.
  /// @param cf The computed impulsive forces, on return, in a packed storage
  ///           format. The first `nc` elements of @p cf correspond to the
  ///           magnitudes of the contact impulses applied along the normals of
  ///           the `nc` contact points. The remaining elements of @p cf
  ///           correspond to the frictional impulses along the `r` spanning
  ///           directions at each point of contact. The first `r`
  ///           values (after the initial `nc` elements) correspond to the first
  ///           contact, the next `r` values correspond to the second contact,
  ///           etc.
  /// @pre Contact data has been computed.
  /// @throws a std::runtime_error if the contact forces cannot be computed
  ///         (due to, e.g., the effects of roundoff error in attempting to
  ///         solve the complementarity problem); in such cases, it is
  ///         recommended to increase @p cfm and attempt again.
  /// @throws a std::logic_error if @p cf is null or @p cfm is negative.
  void SolveImpactProblem(double cfm,
                           const RigidContactVelProblemData<T>& problem_data,
                           VectorX<T>* cf) const;

  /// Computes the generalized force on the system from the contact forces given
  /// in packed storage.
  /// @param problem_data The data used to compute the contact forces.
  /// @param cf The computed contact forces, in the packed storage
  ///           format described in documentation for SolveContactProblem.
  /// @param[out] generalized_force The generalized force acting on the system
  ///             from the contact wrench is stored here, on return.
  /// @throws std::logic_error if @p generalized_force is null or @p cf
  ///         vector is incorrectly sized.
  static void ComputeGeneralizedForceFromContactForces(
      const RigidContactAccelProblemData<T>& problem_data,
      const VectorX<T>& cf,
      VectorX<T>* generalized_force);

  /// Computes the generalized impulse on the system from the contact impulses
  /// given in packed storage.
  /// @param problem_data The data used to compute the contact impulses.
  /// @param cf The computed contact impulses, in the packed storage
  ///           format described in documentation for SolveImpactProblem.
  /// @param[out] generalized_impulse The generalized impulse acting on the
  ///             system from the contact wrench is stored here, on return.
  /// @throws std::logic_error if @p generalized_impulse is null or @p cf
  ///         vector is incorrectly sized.
  static void ComputeGeneralizedImpulseFromContactImpulses(
      const RigidContactVelProblemData<T>& problem_data,
      const VectorX<T>& cf,
      VectorX<T>* generalized_impulse);

  /// Computes the system generalized acceleration, given the external forces
  /// (stored in @p problem_data) and the contact forces.
  /// @param cf The computed contact forces, in the packed storage
  ///           format described in documentation for SolveContactProblem.
  /// @throws std::logic_error if @p generalized_acceleration is null or
  ///         @p cf vector is incorrectly sized.
  static void ComputeGeneralizedAcceleration(
      const RigidContactAccelProblemData<T>& problem_data,
      const VectorX<T>& cf,
      VectorX<T>* generalized_acceleration);

  /// Computes the change to the system generalized velocity from contact
  /// impulses.
  /// @param cf The computed contact impulses, in the packed storage
  ///           format described in documentation for SolveImpactProblem.
  /// @throws std::logic_error if @p generalized_delta_v is null or
  ///         @p cf vector is incorrectly sized.
  static void ComputeGeneralizedVelocityChange(
      const RigidContactVelProblemData<T>& problem_data,
      const VectorX<T>& cf,
      VectorX<T>* generalized_delta_v);

  /// Gets the contact forces expressed in each contact frame *for 2D contact
  /// problems* from the "packed" solution returned by SolveContactProblem().
  /// @param cf the output from SolveContactProblem()
  /// @param problem_data the problem data input to SolveContactProblem()
  /// @param contact_frames the contact frames corresponding to the contacts.
  ///        The first column of each matrix should give the contact normal,
  ///        while the second column gives a contact tangent. For sliding
  ///        contacts, the contact tangent should point along the direction of
  ///        sliding. For non-sliding contacts, the tangent direction should be
  ///        that used to determine @p problem_data.F. All vectors should be
  ///        expressed in the global frame.
  /// @param[out] contact_forces a non-null vector of a doublet of values, where
  ///             the iᵗʰ element represents the force along each basis
  ///             vector in the iᵗʰ contact frame.
  /// @throws std::logic_error if @p contact_forces is null, if
  ///         @p contact_forces is not empty, if @p cf is not the
  ///         proper size, if the number of tangent directions is not one per
  ///         non-sliding contact (indicating that the contact problem might not
  ///         be 2D), if the number of contact frames is not equal to the number
  ///         of contacts, or if a contact frame does not appear to be
  ///         orthonormal.
  /// @note On return, the contact force at the iᵗʰ contact point expressed
  ///       in the world frame is @p contact_frames[i] * @p contact_forces[i].
  static void CalcContactForcesInContactFrames(
      const VectorX<T>& cf, const RigidContactAccelProblemData<T>& problem_data,
      const std::vector<Matrix2<T>>& contact_frames,
      std::vector<Vector2<T>>* contact_forces);

  /// Gets the contact impulses expressed in each contact frame *for 2D contact
  /// problems* from the "packed" solution returned by SolveImpactProblem().
  /// @param cf the output from SolveImpactProblem()
  /// @param problem_data the problem data input to SolveImpactProblem()
  /// @param contact_frames the contact frames corresponding to the contacts.
  ///        The first column of each matrix should give the contact normal,
  ///        while the second column gives a contact tangent (specifically, the
  ///        tangent direction used to determine @p problem_data.F). All
  ///        vectors should be expressed in the global frame.
  /// @param[out] contact_impulses a non-null vector of a doublet of values,
  ///             where the iᵗʰ element represents the impulsive force along
  ///             each basis vector in the iᵗʰ contact frame.
  /// @throws std::logic_error if @p contact_impulses is null, if
  ///         @p contact_impulses is not empty, if @p cf is not the
  ///         proper size, if the number of tangent directions is not one per
  ///         contact (indicating that the contact problem might not be 2D), if
  ///         the number of contact frames is not equal to the number
  ///         of contacts, or if a contact frame does not appear to be
  ///         orthonormal.
  /// @note On return, the contact impulse at the iᵗʰ contact point expressed
  ///       in the world frame is @p contact_frames[i] * @p contact_impulses[i].
  static void CalcImpactForcesInContactFrames(
      const VectorX<T>& cf, const RigidContactVelProblemData<T>& problem_data,
      const std::vector<Matrix2<T>>& contact_frames,
      std::vector<Vector2<T>>* contact_impulses);

 private:
  void FormImpactingContactLCP(
      const RigidContactVelProblemData<T>& problem_data,
      MatrixX<T>* MM, VectorX<T>* qq) const;
  void FormSustainedContactLCP(
      const RigidContactAccelProblemData<T>& problem_data,
      MatrixX<T>* MM, VectorX<T>* qq) const;

  drake::solvers::MobyLCPSolver<T> lcp_;
};

template <typename T>
void RigidContactSolver<T>::SolveContactProblem(double cfm,
    const RigidContactAccelProblemData<T>& problem_data,
    VectorX<T>* cf) const {
  using std::max;
  using std::abs;

  if (!cf)
    throw std::logic_error("cf (output parameter) is null.");
  if (cfm < 0.0) {
    throw std::logic_error("Constraint force mixing (CFM) parameter is "
                               "negative.");
  }

  // Alias problem data.
  const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;

  // Get numbers of friction directions and types of contacts.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);

  // Look for fast exit.
  if (num_contacts == 0) {
    cf->resize(0);
    return;
  }

  // Initialize contact force vector.
  cf->resize(num_contacts + num_spanning_vectors);

  // Set up the linear complementarity problem.
  MatrixX<T> MM;
  VectorX<T> qq;
  FormSustainedContactLCP(problem_data, &MM, &qq);

  // Regularize the LCP matrix as necessary.
  const int num_vars = qq.size();
  MM += MatrixX<T>::Identity(num_vars, num_vars) * cfm;

  // Get the zero tolerance for solving the LCP.
  const T zero_tol = max(cfm, lcp_.ComputeZeroTolerance(MM));

  // Solve the LCP and compute the values of the slack variables.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, zero_tol);
  VectorX<T> ww = MM * zz + qq;

  // NOTE: This LCP might not be solvable due to inconsistent configurations.
  // Check the answer and throw a runtime error if it's no good.
  // LCP constraints are zz ≥ 0, ww ≥ 0, zzᵀww = 0. Since the zero tolerance
  // is used to check a single element for zero (within a single pivoting
  // operation), we must compensate for the number of pivoting operations and
  // the problem size. zzᵀww must use a looser tolerance to account for the
  // num_vars multiplies.
  const int npivots = lcp_.get_num_pivots();
  if (!success || (zz.size() > 0 &&
      (zz.minCoeff() < -num_vars * npivots * zero_tol ||
      ww.minCoeff() < -num_vars * npivots * zero_tol ||
      abs(zz.dot(ww)) > num_vars * num_vars * npivots * zero_tol))) {
    throw std::runtime_error("Unable to solve LCP- it may be unsolvable.");
  }

  // Get the contact forces in the contact frame.
  cf->segment(0, num_contacts) = zz.segment(0, num_contacts);
  cf->segment(num_contacts, num_spanning_vectors) =
      zz.segment(num_contacts, num_spanning_vectors) -
      zz.segment(num_contacts + num_spanning_vectors, num_spanning_vectors);
}

template <typename T>
void RigidContactSolver<T>::SolveImpactProblem(
    double cfm,
    const RigidContactVelProblemData<T>& problem_data,
    VectorX<T>* cf) const {
  using std::max;
  using std::abs;

  if (!cf)
    throw std::logic_error("cf (output parameter) is null.");
  if (cfm < 0.0) {
    throw std::logic_error("Constraint force mixing (CFM) parameter is "
                               "negative.");
  }

  // Get numbers of friction directions and types of contacts.
  const int num_contacts = problem_data.mu.size();
  if (static_cast<size_t>(num_contacts) != problem_data.r.size()) {
    throw std::logic_error("Number of elements in 'r' does not match number"
                               "of elements in 'mu'");
  }
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);

  // Look for fast exit.
  if (num_contacts == 0) {
    cf->resize(0);
    return;
  }

  // If no impact, do not apply the impact model.
  if ((problem_data.N * problem_data.v).minCoeff() >= 0) {
    cf->setZero(num_contacts + num_spanning_vectors);
    return;
  }

  // Initialize contact force vector.
  cf->resize(num_contacts + num_spanning_vectors);

  // Set up the linear complementarity problem.
  MatrixX<T> MM;
  VectorX<T> qq;
  FormImpactingContactLCP(problem_data, &MM, &qq);

  // Regularize the LCP matrix as necessary.
  const int num_vars = qq.size();
  MM += MatrixX<T>::Identity(num_vars, num_vars) * cfm;

  // Get the tolerance for zero used by the LCP solver.
  const T zero_tol = max(cfm, lcp_.ComputeZeroTolerance(MM));

  // Solve the LCP and compute the values of the slack variables.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, zero_tol);
  VectorX<T> ww = MM * zz + qq;

  // NOTE: This LCP should always be solvable.
  // Check the answer and throw a runtime error if it's no good.
  // LCP constraints are zz ≥ 0, ww ≥ 0, zzᵀww = 0. Since the zero tolerance
  // is used to check a single element for zero (within a single pivoting
  // operation), we must compensate for the number of pivoting operations and
  // the problem size. zzᵀww must use a looser tolerance to account for the
  // num_vars multiplies.
  const int npivots = lcp_.get_num_pivots();
  if (!success || (zz.size() > 0 &&
      (zz.minCoeff() < -num_vars * npivots * zero_tol ||
          ww.minCoeff() < -num_vars * npivots * zero_tol ||
          abs(zz.dot(ww)) > num_vars * num_vars * npivots * zero_tol))) {
    throw std::runtime_error("Unable to solve LCP- more regularization might "
                                 "be necessary.");
  }

  // Get the contact forces in the contact frame.
  cf->segment(0, num_contacts) = zz.segment(0, num_contacts);
  cf->segment(num_contacts, num_spanning_vectors) =
      zz.segment(num_contacts, num_spanning_vectors) -
          zz.segment(num_contacts + num_spanning_vectors, num_spanning_vectors);
}

// Forms the LCP matrix and vector, which is used to determine the contact
// forces (and can also be used to determine the active set of constraints at
// the acceleration-level).
template <class T>
void RigidContactSolver<T>::FormSustainedContactLCP(
    const RigidContactAccelProblemData<T>& problem_data,
    MatrixX<T>* MM, VectorX<T>* qq) const {
  DRAKE_DEMAND(MM);
  DRAKE_DEMAND(qq);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                 problem_data.r.end(), 0);

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Alias matrices / vectors to make accessing them less clunky.
  const MatrixX<T>& N = problem_data.N;
  const MatrixX<T>& F = problem_data.F;
  const VectorX<T>& Ndot_x_v = problem_data.Ndot_x_v;
  const VectorX<T>& Fdot_x_v = problem_data.Fdot_x_v;
  const VectorX<T>& mu_non_sliding = problem_data.mu_non_sliding;

  // Construct a matrix similar to E in Anitscu and Potra 1997. This matrix
  // will be used to specify the constraints:
  // 0 ≤ μ⋅fN - E⋅fF ⊥ λ ≥ 0 and
  // 0 ≤ e⋅λ + F⋅dv/dt ⊥ fF ≥ 0,
  // where λ can roughly be interpreted as the remaining tangential acceleration
  // at the non-sliding contacts after frictional forces have been applied and
  // e is a vector of ones (i.e., a segment of the appropriate column of E).
  MatrixX<T> E = MatrixX<T>::Zero(num_spanning_vectors * 2, num_non_sliding);
  for (int i = 0, j = 0; i < num_non_sliding; ++i) {
    const int num_tangent_dirs = problem_data.r[i] * 2;
    E.col(i).segment(j, num_tangent_dirs).setOnes();
    j += num_tangent_dirs;
  }

  // Construct the LCP matrix. First do the "normal contact direction" rows:
  // N⋅M⁻¹⋅(Nᵀ - μQᵀ)  N⋅M⁻¹⋅Dᵀ  0
  // D⋅M⁻¹⋅Nᵀ          D⋅M⁻¹⋅Dᵀ  E
  // μ                 -Eᵀ       0
  // where D = |  F |
  //           | -F |
  const int nc = num_contacts;          // Alias these vars for more...
  const int nr = num_spanning_vectors;  //   readable construction...
  const int nk = nr * 2;                //    of MM/qq.
  const int num_vars = nc + nk + num_non_sliding;
  MatrixX<T> M_inv_x_FT = problem_data.solve_inertia(F.transpose());
  MM->resize(num_vars, num_vars);
  MM->block(0, 0, nc, nc) = N *
      problem_data.solve_inertia(problem_data.N_minus_mu_Q.transpose());
  MM->block(0, nc, nc, nr) = N * M_inv_x_FT;
  MM->block(0, nc + nr, nc, nr) = -MM->block(0, nc, nc, nr);
  MM->block(0, nc + nk, num_non_sliding, num_non_sliding).setZero();

  // Now construct the un-negated tangent contact direction rows (everything
  // but last block column).
  MM->block(nc, 0, nr, nc) = MM->block(0, nc, nc, nr).transpose().eval();
  MM->block(nc, nc, nr, num_spanning_vectors) = F * M_inv_x_FT;
  MM->block(nc, nc + nr, num_spanning_vectors, nr) =
      -MM->block(nc, nc, nr, num_spanning_vectors);

  // Now construct the negated tangent contact direction rows (everything but
  // last block column). These negated tangent contact directions allow the
  // LCP to compute forces applied along the negative x-axis.
  MM->block(nc + nr, 0, nr, nc + nk) = -MM->block(nc, 0, nr, nc + nk);

  // Construct the last block column for the last set of rows (see Anitescu and
  // Potra, 1997).
  MM->block(nc, nc + nk, nk, num_non_sliding) = E;

  // Construct the last two rows, which provide the friction "cone" constraint.
  MM->block(nc + nk, 0, num_non_sliding, num_non_sliding) =
      Eigen::DiagonalMatrix<T, Eigen::Dynamic>(mu_non_sliding);
  MM->block(nc + nk, nc, num_non_sliding, nk) = -E.transpose();
  MM->block(nc + nk, nc + nk, num_non_sliding, num_non_sliding).setZero();

  // Construct the LCP vector:
  // N⋅M⁻¹⋅fext + dN/dt⋅v
  // D⋅M⁻¹⋅fext + dD/dt⋅v
  // 0
  // where, as above, D is defined as [F -F]
  VectorX<T> M_inv_x_f = problem_data.solve_inertia(problem_data.f);
  qq->resize(num_vars, 1);
  qq->segment(0, nc) = N * M_inv_x_f + Ndot_x_v;
  qq->segment(nc, nr) = F * M_inv_x_f + Fdot_x_v;
  qq->segment(nc + nr, num_spanning_vectors) = -qq->segment(nc, nr);
  qq->segment(nc + nk, num_non_sliding).setZero();
}

// Forms the LCP matrix and vector, which is used to determine the collisional
// impulses.
template <class T>
void RigidContactSolver<T>::FormImpactingContactLCP(
    const RigidContactVelProblemData<T>& problem_data,
    MatrixX<T>* MM, VectorX<T>* qq) const {
  DRAKE_DEMAND(MM);
  DRAKE_DEMAND(qq);

  // Get numbers of contacts.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);

  // Problem matrices and vectors are nearly identical to:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Alias matrices / vectors to make accessing them less clunky.
  const MatrixX<T>& N = problem_data.N;
  const MatrixX<T>& F = problem_data.F;
  const VectorX<T>& mu = problem_data.mu;

  // Construct the matrix E in Anitscu and Potra 1997. This matrix
  // will be used to specify the constraints:
  // 0 ≤ μ⋅fN - E⋅fF ⊥ λ ≥ 0 and
  // 0 ≤ e⋅λ + F⋅v ⊥ fF ≥ 0,
  // where λ can roughly be interpreted as the remaining tangential velocity
  // at the impacting contacts after frictional impulses have been applied and
  // e is a vector of ones (i.e., a segment of the appropriate column of E).
  MatrixX<T> E = MatrixX<T>::Zero(num_spanning_vectors * 2, num_contacts);
  for (int i = 0, j = 0; i < num_contacts; ++i) {
    const int num_tangent_dirs = problem_data.r[i] * 2;
    E.col(i).segment(j, num_tangent_dirs).setOnes();
    j += num_tangent_dirs;
  }

  // Construct the LCP matrix. First do the "normal contact direction" rows:
  // N⋅M⁻¹⋅Nᵀ  N⋅M⁻¹⋅Dᵀ  0
  // D⋅M⁻¹⋅Nᵀ  D⋅M⁻¹⋅Dᵀ  E
  // μ         -Eᵀ       0
  // where D = |  F |
  //           | -F |
  const int nc = num_contacts;          // Alias these vars for more...
  const int nr = num_spanning_vectors;  //   readable construction...
  const int nk = nr * 2;                //    of MM/qq.
  const int num_vars = nc * 2 + nk;
  MatrixX<T> M_inv_x_FT = problem_data.solve_inertia(F.transpose());
  MM->resize(num_vars, num_vars);
  MM->block(0, 0, nc, nc) = N * problem_data.solve_inertia(
      problem_data.N.transpose());
  MM->block(0, nc, nc, nr) = N * M_inv_x_FT;
  MM->block(0, nc + nr, nc, nr) = -MM->block(0, nc, nc, nr);
  MM->block(0, nc + nk, nc, nc).setZero();

  // Now construct the un-negated tangent contact direction rows (everything
  // but last block column).
  MM->block(nc, 0, nr, nc) = MM->block(0, nc, nc, nr).transpose().eval();
  MM->block(nc, nc, nr, nr) = F * M_inv_x_FT;
  MM->block(nc, nc + nr, nr, nr) = -MM->block(nc, nc, nr, nr);

  // Now construct the negated tangent contact direction rows (everything but
  // last block column). These negated tangent contact directions allow the
  // LCP to compute forces applied along the negative x-axis.
  MM->block(nc + nr, 0, nr, nc + nk) = -MM->block(nc, 0, nr, nc + nk);

  // Construct the last block column for the last set of rows (see Anitescu and
  // Potra, 1997).
  MM->block(nc, nc + nk, nk, nc) = E;

  // Construct the last two rows, which provide the friction "cone" constraint.
  MM->block(nc + nk, 0, nc, nc) = Eigen::DiagonalMatrix<T, Eigen::Dynamic>(mu);
  MM->block(nc + nk, nc, nc, nk) = -E.transpose();
  MM->block(nc + nk, nc + nk, nc, nc).setZero();

  // Construct the LCP vector:
  // N⋅v
  // D⋅v
  // 0
  // where, as above, D is defined as [F -F]
  qq->resize(num_vars, 1);
  qq->segment(0, nc) = N * problem_data.v;
  qq->segment(nc, nr) = F * problem_data.v;
  qq->segment(nc + nr, nc) = -qq->segment(nc, nr);
  qq->segment(nc + nk, nc).setZero();
}

template <class T>
void RigidContactSolver<T>::ComputeGeneralizedForceFromContactForces(
    const RigidContactAccelProblemData<T>& problem_data,
    const VectorX<T>& cf,
    VectorX<T>* generalized_force) {
  if (!generalized_force)
    throw std::logic_error("generalized_force vector is null.");

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                 problem_data.r.end(), 0);

  // Verify cf is the correct size.
  const int num_vars = num_contacts + num_spanning_vectors;
  if (cf.size() != num_vars)
    throw std::logic_error("cf (contact force) parameter incorrectly sized.");

  /// Get the normal and non-sliding contact forces.
  const Eigen::Ref<const VectorX<T>> f_normal = cf.segment(0, num_contacts);
  const Eigen::Ref<const VectorX<T>> f_non_sliding_frictional = cf.segment(
      num_contacts, num_vars - num_contacts);

  /// Compute the generalized force.
  *generalized_force = problem_data.N_minus_mu_Q.transpose() * f_normal +
      problem_data.F.transpose() * f_non_sliding_frictional;
}

template <class T>
void RigidContactSolver<T>::ComputeGeneralizedImpulseFromContactImpulses(
    const RigidContactVelProblemData<T>& problem_data,
    const VectorX<T>& cf,
    VectorX<T>* generalized_impulse) {
  if (!generalized_impulse)
    throw std::logic_error("generalized_impulse vector is null.");

  // Get number of contacts.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);
  const int num_vars = num_contacts + num_spanning_vectors;
  if (num_vars != cf.size())
    throw std::logic_error("Unexpected packed contact force vector dimension.");

  /// Get the normal and tangential contact impulses.
  const Eigen::Ref<const VectorX<T>> f_normal = cf.segment(0, num_contacts);
  const Eigen::Ref<const VectorX<T>> f_frictional = cf.segment(
      num_contacts, num_vars - num_contacts);

  /// Compute the generalized impules.
  *generalized_impulse = problem_data.N.transpose() * f_normal +
                         problem_data.F.transpose() * f_frictional;
}

template <class T>
void RigidContactSolver<T>::ComputeGeneralizedAcceleration(
    const RigidContactAccelProblemData<T>& problem_data,
    const VectorX<T>& cf,
    VectorX<T>* generalized_acceleration) {
  if (!generalized_acceleration)
    throw std::logic_error("generalized_acceleration vector is null.");

  VectorX<T> generalized_force;
  ComputeGeneralizedForceFromContactForces(problem_data, cf,
                                           &generalized_force);
  *generalized_acceleration = problem_data.solve_inertia(problem_data.f +
                                                         generalized_force);
}

template <class T>
void RigidContactSolver<T>::ComputeGeneralizedVelocityChange(
    const RigidContactVelProblemData<T>& problem_data,
    const VectorX<T>& cf,
    VectorX<T>* generalized_delta_v) {

  if (!generalized_delta_v)
    throw std::logic_error("generalized_delta_v vector is null.");

  VectorX<T> generalized_impulse;
  ComputeGeneralizedImpulseFromContactImpulses(problem_data, cf,
                                               &generalized_impulse);
  *generalized_delta_v = problem_data.solve_inertia(generalized_impulse);
}

template <class T>
void RigidContactSolver<T>::CalcContactForcesInContactFrames(
    const VectorX<T>& cf, const RigidContactAccelProblemData<T>& problem_data,
    const std::vector<Matrix2<T>>& contact_frames,
    std::vector<Vector2<T>>* contact_forces) {
  using std::abs;

  // Loose tolerance for unit vectors and orthogonality.
  const double loose_eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Verify that contact_forces is non-null and is empty.
  if (!contact_forces)
    throw std::logic_error("Vector of contact forces is null.");
  if (!contact_forces->empty())
    throw std::logic_error("Vector of contact forces is not empty.");

  // Verify that cf is the correct size.
  const int num_non_sliding_contacts = problem_data.non_sliding_contacts.size();
  const int num_contacts = problem_data.sliding_contacts.size() +
      num_non_sliding_contacts;
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);
  const int num_vars = num_contacts + num_spanning_vectors;
  if (num_vars != cf.size())
    throw std::logic_error("Unexpected packed contact force vector dimension.");

  // Verify that the problem is indeed two-dimensional.
  if (num_spanning_vectors != num_non_sliding_contacts) {
    throw std::logic_error("Problem data 'r' indicates contact problem is not "
                               "two-dimensional");
  }

  // Verify that the correct number of contact frames has been specified.
  if (contact_frames.size() != static_cast<size_t>(num_contacts)) {
    throw std::logic_error("Number of contact frames does not match number of "
                               "contacts.");
  }

  // Verify that sliding contact indices are sorted.
  DRAKE_ASSERT(std::is_sorted(problem_data.sliding_contacts.begin(),
                              problem_data.sliding_contacts.end()));

  // Resize the force vector.
  contact_forces->resize(contact_frames.size());

  // Set the forces.
  for (int i = 0, sliding_index = 0, non_sliding_index = 0; i < num_contacts;
       ++i) {
    // Alias the force.
    Vector2<T>& contact_force_i = (*contact_forces)[i];

    // Get the contact normal and tangent.
    const Vector2<T> contact_normal = contact_frames[i].col(0);
    const Vector2<T> contact_tangent = contact_frames[i].col(1);

    // Verify that each direction is of unit length.
    if (abs(contact_normal.norm() - 1) > loose_eps)
      throw std::runtime_error("Contact normal apparently not unit length.");
    if (abs(contact_tangent.norm() - 1) > loose_eps)
      throw std::runtime_error("Contact tangent apparently not unit length.");

    // Verify that the two directions are orthogonal.
    if (abs(contact_normal.dot(contact_tangent)) > loose_eps) {
      std::ostringstream oss;
      oss << "Contact normal (" << contact_normal.transpose() << ") and ";
      oss << "contact tangent (" << contact_tangent.transpose() << ") ";
      oss << "insufficiently orthogonal.";
      throw std::logic_error(oss.str());
    }

    // Initialize the contact force expressed in the global frame.
    Vector2<T> f0(0, 0);

    // Add in the contact normal.
    f0 += contact_normal * cf[i];

    // Determine whether the contact is sliding.
    const bool is_sliding = std::binary_search(
        problem_data.sliding_contacts.begin(),
        problem_data.sliding_contacts.end(), i);

    // Subtract/add the tangential force in the world frame.
    if (is_sliding) {
      f0 -= contact_tangent * cf[i] * problem_data.mu_sliding[sliding_index++];
    } else {
      f0 += contact_tangent * cf[num_contacts + non_sliding_index++];
    }

    // Compute the contact force in the contact frame.
    contact_force_i = contact_frames[i].transpose() * f0;
  }
}

template <class T>
void RigidContactSolver<T>::CalcImpactForcesInContactFrames(
    const VectorX<T>& cf, const RigidContactVelProblemData<T>& problem_data,
    const std::vector<Matrix2<T>>& contact_frames,
    std::vector<Vector2<T>>* contact_impulses) {
  using std::abs;

  // Loose tolerance for unit vectors and orthogonality.
  const double loose_eps = std::sqrt(std::numeric_limits<double>::epsilon());

  // Verify that contact_impulses is non-null and is empty.
  if (!contact_impulses)
    throw std::logic_error("Vector of contact impulses is null.");
  if (!contact_impulses->empty())
    throw std::logic_error("Vector of contact impulses is not empty.");

  // Verify that cf is the correct size.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors = std::accumulate(problem_data.r.begin(),
                                                   problem_data.r.end(), 0);
  const int num_vars = num_contacts + num_spanning_vectors;
  if (num_vars != cf.size())
    throw std::logic_error("Unexpected packed contact force vector dimension.");

  // Verify that the problem is indeed two-dimensional.
  if (num_spanning_vectors != num_contacts) {
    throw std::logic_error("Problem data 'r' indicates contact problem is not "
                               "two-dimensional");
  }

  // Verify that the correct number of contact frames has been specified.
  if (contact_frames.size() != static_cast<size_t>(num_contacts)) {
    throw std::logic_error("Number of contact frames does not match number of "
                               "contacts.");
  }

  // Resize the impulse vector.
  contact_impulses->resize(contact_frames.size());

  // Set the impulses.
  for (int i = 0, tangent_index = 0; i < num_contacts; ++i) {
    // Alias the impulse.
    Vector2<T>& contact_impulse_i = (*contact_impulses)[i];

    // Get the contact normal and tangent.
    const Vector2<T> contact_normal = contact_frames[i].col(0);
    const Vector2<T> contact_tangent = contact_frames[i].col(1);

    // Verify that each direction is of unit length.
    if (abs(contact_normal.norm() - 1) > loose_eps)
      throw std::runtime_error("Contact normal apparently not unit length.");
    if (abs(contact_tangent.norm() - 1) > loose_eps)
      throw std::runtime_error("Contact tangent apparently not unit length.");

    // Verify that the two directions are orthogonal.
    if (abs(contact_normal.dot(contact_tangent)) > loose_eps) {
      std::ostringstream oss;
      oss << "Contact normal (" << contact_normal.transpose() << ") and ";
      oss << "contact tangent (" << contact_tangent.transpose() << ") ";
      oss << "insufficiently orthogonal.";
      throw std::logic_error(oss.str());
    }

    // Compute the contact impulse expressed in the global frame.
    Vector2<T> j0 = contact_normal * cf[i] + contact_tangent *
        cf[num_contacts + tangent_index++];

    // Compute the contact impulse in the contact frame.
    contact_impulse_i = contact_frames[i].transpose() * j0;
  }
}

}  // namespace rigid_contact
}  // namespace multibody
}  // namespace drake
