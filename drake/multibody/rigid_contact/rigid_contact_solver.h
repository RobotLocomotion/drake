#pragma once

#include <algorithm>
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
  ///            known as the "constraint force mixing" parameter. Aborts if
  ///            @p cfm is negative.
  /// @param problem_data The data used to compute the contact forces.
  /// @param cf The computed contact forces, on return, in a packed storage
  ///           format. The first `nc` elements of @p cf correspond to the
  ///           magnitudes of the contact forces applied along the normals of
  ///           the `nc` contact points. The remaining elements of @p cf
  ///           correspond to the frictional forces along the `r` spanning
  ///           directions at each non-sliding point of contact. The first `r`
  ///           values (after the initial `nc` elements) correspond to the first
  ///           non-sliding contact, the next `r` values correspond to the
  ///           second non-sliding contct, etc.
  /// @pre Contact data has been computed.
  /// @throws a std::runtime_error if the contact forces cannot be computed
  ///         (due to, e.g., an "inconsistent" rigid contact configuration).
  void SolveContactProblem(double cfm,
      const RigidContactAccelProblemData<T>& problem_data,
      VectorX<T>* cf) const;

 private:
  void FormSustainedContactLCP(
      const RigidContactAccelProblemData<T>& problem_data,
      MatrixX<T>* MM, Eigen::Matrix<T, Eigen::Dynamic, 1>* qq) const;

  drake::solvers::MobyLCPSolver<T> lcp_;
};

template <typename T>
void RigidContactSolver<T>::SolveContactProblem(double cfm,
    const RigidContactAccelProblemData<T>& problem_data,
    VectorX<T>* cf) const {
  using std::max;
  using std::abs;
  DRAKE_DEMAND(cf);
  DRAKE_DEMAND(cfm >= 0.0);

  // Alias problem data.
  const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;

  // Get numbers of friction directions and types of contacts.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;
  const int nr = std::accumulate(problem_data.r.begin(),
                                 problem_data.r.end(), 0);

  // Look for fast exit.
  if (nc == 0) {
    cf->resize(0);
    return;
  }

  // Initialize contact force vector.
  cf->resize(nc + nr);

  // Set up the linear complementarity problem.
  MatrixX<T> MM;
  VectorX<T> qq;
  FormSustainedContactLCP(problem_data, &MM, &qq);

  // Regularize the LCP matrix as necessary.
  const int nvars = qq.size();
  MM += MatrixX<T>::Identity(nvars, nvars) * cfm;

  // Get the zero tolerance for solving the LCP.
  const T zero_tol = max(cfm, lcp_.ComputeZeroTolerance(MM));

  // Solve the LCP and compute the values of the slack variables.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, zero_tol);
  VectorX<T> ww = MM * zz + qq;

  // NOTE: This LCP might not be solvable due to inconsistent configurations.
  // Check the answer and throw a runtime error if it's no good.
  if (!success || (zz.size() > 0 && (zz.minCoeff() < -10 * zero_tol ||
      ww.minCoeff() < -10 * zero_tol ||
      abs(zz.dot(ww)) > nvars * 100 * zero_tol))) {
    throw std::runtime_error("Unable to solve LCP- it may be unsolvable.");
  }

  // Get the contact forces in the contact frame.
  cf->segment(0, nc) = zz.segment(0, nc);
  cf->segment(nc, nr) = zz.segment(nc, nr) - zz.segment(nc + nr, nr);
}

// Forms the LCP matrix and vector, which is used to determine the contact
// forces (and can also be used to determine the active set of constraints at
// the acceleration-level).
template <class T>
void RigidContactSolver<T>::FormSustainedContactLCP(
    const RigidContactAccelProblemData<T>& problem_data,
    MatrixX<T>* MM, Eigen::Matrix<T, Eigen::Dynamic, 1>* qq) const {
  DRAKE_DEMAND(MM);
  DRAKE_DEMAND(qq);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int nc = num_sliding + num_non_sliding;
  const int nr = std::accumulate(problem_data.r.begin(),
                                 problem_data.r.end(), 0);
  const int nk = nr * 2;

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
  // will be used to specify the constraints 0 ≤ μ⋅fN - E⋅fF ⊥ λ ≥ 0 and
  // 0 ≤ e⋅λ + F⋅dv/dt ⊥ fF ≥ 0.
  MatrixX<T> E = MatrixX<T>::Zero(nk, num_non_sliding);
  for (int i = 0, j = 0; i < num_non_sliding; ++i) {
    const int num_tangent_dirs = problem_data.r[i];
    E.col(i).segment(j, num_tangent_dirs).setOnes();
    j += num_tangent_dirs;
  }

  // Construct the LCP matrix. First do the "normal contact direction" rows:
  // N⋅M⁻¹⋅(Nᵀ - μQᵀ)  N⋅M⁻¹⋅Dᵀ  0
  // D⋅M⁻¹⋅Nᵀ          D⋅M⁻¹⋅Dᵀ  E
  // μ                 -Eᵀ       0
  // where D = [F -F]
  const int nvars = nc + nk + num_non_sliding;
  MatrixX<T> M_inv_x_FT = problem_data.solve_inertia(F.transpose());
  MM->resize(nvars, nvars);
  MM->block(0, 0, nc, nc) = N *
      problem_data.solve_inertia(problem_data.N_minus_mu_Q.transpose());
  MM->block(0, nc, nc, nr) = N * M_inv_x_FT;
  MM->block(0, nc + nr, nc, nr) = -MM->block(0, nc, nc, nr);
  MM->block(0, nc + nk, num_non_sliding, num_non_sliding).setZero();

  // Now construct the un-negated tangent contact direction rows (everything
  // but last block column).
  MM->block(nc, 0, nr, nc) = MM->block(0, nc, nc, nr).transpose();
  MM->block(nc, nc, nr, nr) = F * M_inv_x_FT;
  MM->block(nc, nc + nr, nr, nr) = -MM->block(nc, nc, nr, nr);

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
  qq->resize(nvars, 1);
  qq->segment(0, nc) = N * M_inv_x_f + Ndot_x_v;
  qq->segment(nc, nr) = F * M_inv_x_f + Fdot_x_v;
  qq->segment(nc + nr, nr) = -qq->segment(nc, nr);
  qq->segment(nc + nk, num_non_sliding).setZero();
}

}  // namespace rigid_contact
}  // namespace multibody
}  // namespace drake
