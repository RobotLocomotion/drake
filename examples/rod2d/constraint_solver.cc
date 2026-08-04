#include "drake/examples/rod2d/constraint_solver.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>

#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace examples {
namespace rod2d {

template <typename T>
ConstraintSolver<T>::ConstraintSolver() = default;

template <typename T>
void ConstraintSolver<T>::ComputeGeneralizedAcceleration(
    const ConstraintAccelProblemData<T>& problem_data, const VectorX<T>& cf,
    VectorX<T>* generalized_acceleration) {
  ComputeGeneralizedAccelerationFromConstraintForces(problem_data, cf,
                                                     generalized_acceleration);
  (*generalized_acceleration) += problem_data.solve_inertia(problem_data.tau);
}

template <typename T>
template <typename ProblemData>
void ConstraintSolver<T>::DetermineNewPartialInertiaSolveOperator(
    const ProblemData* problem_data, int num_generalized_velocities,
    const Eigen::CompleteOrthogonalDecomposition<MatrixX<T>>* delassus_QTZ,
    std::function<MatrixX<T>(const MatrixX<T>&)>* A_solve) {
  const int num_eq_constraints = problem_data->kG.size();

  *A_solve = [problem_data, delassus_QTZ, num_eq_constraints,
              num_generalized_velocities](const MatrixX<T>& X) -> MatrixX<T> {
    // ************************************************************************
    // See DetermineNewFullInertiaSolveOperator() for block inversion formula.
    // ************************************************************************

    // Set the result matrix.
    const int C_rows = num_generalized_velocities;
    const int E_cols = num_eq_constraints;
    MatrixX<T> result(C_rows + E_cols, X.cols());

    // Begin computation of components of C (upper left hand block of inverse
    // of A): compute M⁻¹ X
    const MatrixX<T> iM_X = problem_data->solve_inertia(X);

    // Compute G M⁻¹ X
    MatrixX<T> G_iM_X(E_cols, X.cols());
    for (int i = 0; i < X.cols(); ++i)
      G_iM_X.col(i) = problem_data->G_mult(iM_X.col(i));

    // Compute (GM⁻¹Gᵀ)⁻¹GM⁻¹X
    const MatrixX<T> Del_G_iM_X = delassus_QTZ->solve(G_iM_X);

    // Compute Gᵀ(GM⁻¹Gᵀ)⁻¹GM⁻¹X
    const MatrixX<T> GT_Del_G_iM_X = problem_data->G_transpose_mult(Del_G_iM_X);

    // Compute M⁻¹Gᵀ(GM⁻¹Gᵀ)⁻¹GM⁻¹X
    const MatrixX<T> iM_GT_Del_G_iM_X =
        problem_data->solve_inertia(GT_Del_G_iM_X);

    // Compute the result
    result = iM_X - iM_GT_Del_G_iM_X;

    return result;
  };
}

template <typename T>
template <typename ProblemData>
void ConstraintSolver<T>::DetermineNewFullInertiaSolveOperator(
    const ProblemData* problem_data, int num_generalized_velocities,
    const Eigen::CompleteOrthogonalDecomposition<MatrixX<T>>* delassus_QTZ,
    std::function<MatrixX<T>(const MatrixX<T>&)>* A_solve) {
  // Get the number of equality constraints.
  const int num_eq_constraints = problem_data->kG.size();

  *A_solve = [problem_data, delassus_QTZ, num_eq_constraints,
              num_generalized_velocities](const MatrixX<T>& B) -> MatrixX<T> {
    // From a block matrix inversion,
    // | M  -Gᵀ |⁻¹ | Y | = |  C  E || Y | = | CY + EZ   |
    // | G   0  |   | Z |   | -Eᵀ F || Z |   | -EᵀY + FZ |
    // where E  ≡ M⁻¹Gᵀ(GM⁻¹Gᵀ)⁻¹
    //       C  ≡ M⁻¹ - M⁻¹Gᵀ(GM⁻¹Gᵀ)⁻¹GM⁻¹ = M⁻¹ - M⁻¹GᵀE
    //      -Eᵀ ≡ -(GM⁻¹Gᵀ)⁻¹GM⁻¹
    //       F  ≡ (GM⁻¹Gᵀ)⁻¹
    //       B  ≡ | Y |
    //            | Z |

    // Set the result matrix (X).
    const int C_rows = num_generalized_velocities;
    const int E_cols = num_eq_constraints;
    MatrixX<T> X(C_rows + E_cols, B.cols());

    // Name the blocks of B and X.
    const auto Y = B.topRows(C_rows);
    const auto Z = B.bottomRows(B.rows() - C_rows);
    auto X_top = X.topRows(C_rows);
    auto X_bot = X.bottomRows(X.rows() - C_rows);

    // 1. Begin computation of components of C.
    // Compute M⁻¹ Y
    const MatrixX<T> iM_Y = problem_data->solve_inertia(Y);

    // Compute G M⁻¹ Y
    MatrixX<T> G_iM_Y(E_cols, Y.cols());
    for (int i = 0; i < Y.cols(); ++i)
      G_iM_Y.col(i) = problem_data->G_mult(iM_Y.col(i));

    // Compute (GM⁻¹Gᵀ)⁻¹GM⁻¹Y
    const MatrixX<T> Del_G_iM_Y = delassus_QTZ->solve(G_iM_Y);

    // Compute Gᵀ(GM⁻¹Gᵀ)⁻¹GM⁻¹Y
    const MatrixX<T> GT_Del_G_iM_Y = problem_data->G_transpose_mult(Del_G_iM_Y);

    // Compute M⁻¹Gᵀ(GM⁻¹Gᵀ)⁻¹GM⁻¹Y
    const MatrixX<T> iM_GT_Del_G_iM_Y =
        problem_data->solve_inertia(GT_Del_G_iM_Y);

    // 2. Begin computation of components of E
    // Compute (GM⁻¹Gᵀ)⁻¹Z
    const MatrixX<T> Del_Z = delassus_QTZ->solve(Z);

    // Compute Gᵀ(GM⁻¹Gᵀ)⁻¹Z
    const MatrixX<T> GT_Del_Z = problem_data->G_transpose_mult(Del_Z);

    // Compute M⁻¹Gᵀ(GM⁻¹Gᵀ)⁻¹Z = EZ
    const MatrixX<T> iM_GT_Del_Z = problem_data->solve_inertia(GT_Del_Z);

    // Set the top block of the result.
    X_top = iM_Y - iM_GT_Del_G_iM_Y + iM_GT_Del_Z;

    // Set the bottom block of the result.
    X_bot = delassus_QTZ->solve(Z) - Del_G_iM_Y;

    return X;
  };
}

template <typename T>
template <typename ProblemData>
ProblemData* ConstraintSolver<T>::UpdateProblemDataForUnilateralConstraints(
    const ProblemData& problem_data,
    std::function<const MatrixX<T>(const MatrixX<T>&)> modified_inertia_solve,
    int gv_dim, ProblemData* modified_problem_data) {
  // Verify that the modified problem data points to something.
  DRAKE_DEMAND(modified_problem_data != nullptr);

  // Get the number of equality constraints.
  const int num_eq_constraints = problem_data.kG.size();

  // Construct a new problem data.
  if (num_eq_constraints == 0) {
    // Just point to the original problem data.
    return const_cast<ProblemData*>(&problem_data);
  } else {
    // Alias the modified problem data so that we don't need to change its
    // pointer.
    ProblemData& new_data = *modified_problem_data;

    // Copy most of the data unchanged.
    new_data = problem_data;

    // Construct zero functions.
    auto zero_fn = [](const VectorX<T>&) -> VectorX<T> {
      return VectorX<T>(0);
    };
    auto zero_gv_dim_fn = [gv_dim](const VectorX<T>&) -> VectorX<T> {
      return VectorX<T>::Zero(gv_dim);
    };

    // Remove the bilateral constraints.
    new_data.kG.resize(0);
    new_data.G_mult = zero_fn;
    new_data.G_transpose_mult = zero_gv_dim_fn;

    // Update the inertia function pointer.
    new_data.solve_inertia = modified_inertia_solve;
    return &new_data;
  }
}

template <class T>
void ConstraintSolver<T>::FormAndSolveConstraintLinearSystem(
    const ConstraintAccelProblemData<T>& problem_data,
    const VectorX<T>& trunc_neg_invA_a, VectorX<T>* cf) const {
  using std::abs;
  using std::max;

  DRAKE_DEMAND(cf != nullptr);

  // Alias problem data.
  const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;

  // Get numbers of friction directions and types of contacts.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_eq_constraints = problem_data.kG.size();

  // Initialize contact force vector.
  cf->resize(num_contacts + num_spanning_vectors + num_limits +
             num_eq_constraints);

  // Using Equations (f) and (g) from the comments in
  // FormAndSolveConstraintLcp() and defining C as the upper left block of A⁻¹,
  // the linear system is defined as MM*z + qq = 0, where:
  //
  // MM ≡ | NC(Nᵀ-μQᵀ)  NCDᵀ   NCLᵀ |
  //      | DC(Nᵀ-μQᵀ)  DCDᵀ   DCLᵀ |
  //      | LC(Nᵀ-μQᵀ)  LCDᵀ   LCLᵀ |
  //
  // qq ≡ | kᴺ + |N 0|A⁻¹a |
  //      | kᴰ + |D 0|A⁻¹a |
  //      | kᴸ + |L 0|A⁻¹a |

  // @TODO(edrumwri): Consider checking whether or not the constraints are
  // satisfied to a user-specified tolerance; a set of constraint equations that
  // are dependent upon time (e.g., prescribed motion constraints) might not be
  // fully satisfiable.

  // Set up the linear system.
  MatrixX<T> MM;
  VectorX<T> qq;
  FormSustainedConstraintLinearSystem(problem_data, trunc_neg_invA_a, &MM, &qq);

  // Solve the linear system.
  Eigen::CompleteOrthogonalDecomposition<MatrixX<T>> MM_QTZ(MM);
  cf->head(qq.size()) = MM_QTZ.solve(-qq);
}

template <typename T>
void ConstraintSolver<T>::FormAndSolveConstraintLcp(
    const ConstraintAccelProblemData<T>& problem_data,
    const VectorX<T>& trunc_neg_invA_a, VectorX<T>* cf) const {
  using std::abs;
  using std::max;

  DRAKE_DEMAND(cf != nullptr);

  // Alias problem data.
  const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;

  // Get numbers of friction directions and types of contacts.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int nk = num_spanning_vectors * 2;
  const int num_limits = problem_data.kL.size();
  const int num_eq_constraints = problem_data.kG.size();

  // Initialize contact force vector.
  cf->resize(num_contacts + num_spanning_vectors + num_limits +
             num_eq_constraints);

  // The constraint problem is a mixed linear complementarity problem of the
  // form:
  // (a)    Au + Xv + a = 0
  // (b)    Yu + Bv + b ≥ 0
  // (c)              v ≥ 0
  // (d) vᵀ(b + Yu + Bv) = 0
  // where u are "free" variables. If the matrix A is nonsingular, u can be
  // solved for:
  // (e) u = -A⁻¹ (a + Xv)
  // allowing the mixed LCP to be converted to a "pure" LCP (qq, MM) by:
  // (f) qq = b - YA⁻¹a
  // (g) MM = B - YA⁻¹X

  // Our mixed linear complementarity problem takes the specific form:
  // (1) | M  -Gᵀ  -(Nᵀ-μQᵀ) -Dᵀ  0  -Lᵀ | | v̇  | + | -f  | = | 0 |
  //     | G   0    0         0   0   0  | | fG | + |  kᴳ | = | 0 |
  //     | N   0    0         0   0   0  | | fN | + |  kᴺ | = | α |
  //     | D   0    0         0   E   0  | | fD | + |  kᴰ | = | β |
  //     | 0   0    μ        -Eᵀ  0   0  | |  λ | + |   0 | = | γ |
  //     | L   0    0         0   0   0  | | fL | + |  kᴸ | = | δ |
  // (2) 0 ≤ fN  ⊥  α ≥ 0
  // (3) 0 ≤ fD  ⊥  β ≥ 0
  // (4) 0 ≤ λ   ⊥  γ ≥ 0
  // (5) 0 ≤ fL  ⊥  δ ≥ 0

  // --------------------------------------------------------------------------
  // Converting the MLCP to a pure LCP:
  // --------------------------------------------------------------------------

  // From the notation above in Equations (a)-(d):
  // A ≡ | M  -Gᵀ|   a ≡ | -f  |   X ≡ |-(Nᵀ-μQᵀ) -Dᵀ  0  -Lᵀ |
  //     | G   0 |       |  kᴳ |       | 0         0   0   0  |
  //
  // Y ≡ | N   0 |   b ≡ |  kᴺ |   B ≡ | 0    0   0   0  |
  //     | D   0 |       |  kᴰ |       | 0    0   E   0  |
  //     | 0   0 |       |  0  |       | μ   -Eᵀ  0   0  |
  //     | L   0 |       |  kᴸ |       | 0    0   0   0  |
  //
  // u ≡ | v̇  |      v ≡ | fN |
  //     | fG |          | fD |
  //                     |  λ |
  //                     | fL |
  //
  // Therefore, using Equations (f) and (g) and defining C as the upper left
  // block of A⁻¹, the pure LCP (qq,MM) is defined as:
  //
  // MM ≡ | NC(Nᵀ-μQᵀ)  NCDᵀ   0   NCLᵀ |
  //      | DC(Nᵀ-μQᵀ)  DCDᵀ   E   DCLᵀ |
  //      | μ          -Eᵀ     0   0    |
  //      | LC(Nᵀ-μQᵀ)  LCDᵀ   0   LCLᵀ |
  //

  // qq ≡ | kᴺ + |N 0|A⁻¹a |
  //      | kᴰ + |D 0|A⁻¹a |
  //      |       0        |
  //      | kᴸ + |L 0|A⁻¹a |
  //

  // --------------------------------------------------------------------------
  // Using the LCP solution to solve the MLCP.
  // --------------------------------------------------------------------------

  // From Equation (e) and the solution to the LCP (v), we can solve for u using
  // the following equations:
  // Xv + a = | -(Nᵀ-μQᵀ)fN - DᵀfD - LᵀfL - f |
  //          |              kᴳ               |
  //

  // TODO(edrumwri): Consider checking whether or not the constraints are
  // satisfied to a user-specified tolerance; a set of constraint equations that
  // are dependent upon time (e.g., prescribed motion constraints) might not be
  // fully satisfiable.

  // Set up the pure linear complementarity problem.
  MatrixX<T> MM;
  VectorX<T> qq;
  FormSustainedConstraintLcp(problem_data, trunc_neg_invA_a, &MM, &qq);

  // Get the zero tolerance for solving the LCP.
  const T zero_tol = lcp_.ComputeZeroTolerance(MM);

  // Solve the LCP and compute the values of the slack variables.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, zero_tol);
  VectorX<T> ww = MM * zz + qq;
  const double max_dot =
      (zz.size() > 0) ? (zz.array() * ww.array()).abs().maxCoeff() : 0.0;

  // NOTE: This LCP might not be solvable due to inconsistent configurations.
  // Check the answer and throw a runtime error if it's no good.
  // LCP constraints are zz ≥ 0, ww ≥ 0, zzᵀww = 0. Since the zero tolerance
  // is used to check a single element for zero (within a single pivoting
  // operation), we must compensate for the number of pivoting operations and
  // the problem size. zzᵀww must use a looser tolerance to account for the
  // num_vars multiplies.
  const int num_vars = qq.size();
  const int npivots = std::max(1, lcp_.get_num_pivots());
  if (!success ||
      (zz.size() > 0 &&
       (zz.minCoeff() < -num_vars * npivots * zero_tol ||
        ww.minCoeff() < -num_vars * npivots * zero_tol ||
        max_dot > max(T(1), zz.maxCoeff()) * max(T(1), ww.maxCoeff()) *
                      num_vars * npivots * zero_tol))) {
    throw std::runtime_error("Unable to solve LCP- it may be unsolvable.");
  }

  // Alias constraint force segments.
  const auto fN = zz.segment(0, num_contacts);
  const auto fD_plus = zz.segment(num_contacts, num_spanning_vectors);
  const auto fD_minus =
      zz.segment(num_contacts + num_spanning_vectors, num_spanning_vectors);
  const auto fL = zz.segment(num_contacts + num_non_sliding + nk, num_limits);

  // Get the constraint forces in the specified packed storage format.
  cf->segment(0, num_contacts) = fN;
  cf->segment(num_contacts, num_spanning_vectors) = fD_plus - fD_minus;
  cf->segment(num_contacts + num_spanning_vectors, num_limits) = fL;
}

template <typename T>
void ConstraintSolver<T>::SolveConstraintProblem(
    const ConstraintAccelProblemData<T>& problem_data, VectorX<T>* cf) const {
  using std::abs;
  using std::max;

  if (!cf) throw std::logic_error("cf (output parameter) is null.");

  // Alias problem data.
  const std::vector<int>& sliding_contacts = problem_data.sliding_contacts;
  const std::vector<int>& non_sliding_contacts =
      problem_data.non_sliding_contacts;

  // Get numbers of friction directions, types of contacts, and other data.
  const int num_sliding = sliding_contacts.size();
  const int num_non_sliding = non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_limits = problem_data.kL.size();
  const int num_eq_constraints = problem_data.kG.size();
  const int num_generalized_velocities = problem_data.tau.size();
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);

  // Look for fast exit.
  if (num_contacts == 0 && num_limits == 0 && num_eq_constraints == 0) {
    cf->resize(0);
    return;
  }

  // Look for a second fast exit. (We avoid this calculation if there are
  // bilateral constraints because it's too hard to determine a workable
  // tolerance at this point).
  const VectorX<T> candidate_accel =
      problem_data.solve_inertia(problem_data.tau);
  const VectorX<T> N_eval =
      problem_data.N_mult(candidate_accel) + problem_data.kN;
  const VectorX<T> L_eval =
      problem_data.L_mult(candidate_accel) + problem_data.kL;
  if ((num_contacts == 0 || N_eval.minCoeff() >= 0) &&
      (num_limits == 0 || L_eval.minCoeff() >= 0) &&
      (num_eq_constraints == 0)) {
    cf->setZero();
    return;
  }

  // Prepare to set up the functionals to compute Ax = b, where A is the
  // blocked saddle point matrix containing the generalized inertia matrix
  // and the bilateral constraints *assuming there are bilateral constraints*.
  // If there are no bilateral constraints, A_solve and fast_A_solve will
  // simply point to the inertia solve operator.
  MlcpToLcpData mlcp_to_lcp_data;

  if (num_eq_constraints > 0) {
    // Form the Delassus matrix for the bilateral constraints.
    MatrixX<T> Del(num_eq_constraints, num_eq_constraints);
    MatrixX<T> iM_GT(num_generalized_velocities, num_eq_constraints);
    ComputeInverseInertiaTimesGT(problem_data.solve_inertia,
                                 problem_data.G_transpose_mult,
                                 num_eq_constraints, &iM_GT);
    ComputeConstraintSpaceComplianceMatrix(problem_data.G_mult,
                                           num_eq_constraints, iM_GT, Del);

    // Compute the complete orthogonal factorization.
    mlcp_to_lcp_data.delassus_QTZ.compute(Del);

    // Determine a new "inertia" solve operator, which solves AX = B, where
    // A = | M  -Gᵀ |
    //     | G   0  |
    // using a least-squares solution to accommodate rank-deficiency in G. This
    // will allow transforming the mixed LCP into a pure LCP.
    DetermineNewFullInertiaSolveOperator(
        &problem_data, num_generalized_velocities,
        &mlcp_to_lcp_data.delassus_QTZ, &mlcp_to_lcp_data.A_solve);

    // Determine a new "inertia" solve operator, using only the upper left block
    // of A⁻¹ (denoted C) to exploit zero blocks in common operations.
    DetermineNewPartialInertiaSolveOperator(
        &problem_data, num_generalized_velocities,
        &mlcp_to_lcp_data.delassus_QTZ, &mlcp_to_lcp_data.fast_A_solve);
  } else {
    mlcp_to_lcp_data.A_solve = problem_data.solve_inertia;
    mlcp_to_lcp_data.fast_A_solve = problem_data.solve_inertia;
  }

  // Copy the problem data and then update it to account for bilateral
  // constraints.
  const int gv_dim = problem_data.tau.size();
  ConstraintAccelProblemData<T> modified_problem_data(gv_dim +
                                                      num_eq_constraints);
  ConstraintAccelProblemData<T>* data_ptr = &modified_problem_data;
  data_ptr = UpdateProblemDataForUnilateralConstraints(
      problem_data, mlcp_to_lcp_data.fast_A_solve, gv_dim, data_ptr);

  // Compute a and A⁻¹a.
  VectorX<T> a(problem_data.tau.size() + num_eq_constraints);
  a.head(problem_data.tau.size()) = -problem_data.tau;
  a.tail(num_eq_constraints) = problem_data.kG;
  const VectorX<T> invA_a = mlcp_to_lcp_data.A_solve(a);
  const VectorX<T> trunc_neg_invA_a = -invA_a.head(problem_data.tau.size());

  // Determine which problem formulation to use.
  if (problem_data.use_complementarity_problem_solver) {
    FormAndSolveConstraintLcp(problem_data, trunc_neg_invA_a, cf);
  } else {
    FormAndSolveConstraintLinearSystem(problem_data, trunc_neg_invA_a, cf);
  }

  // Alias constraint force segments.
  const auto fN = cf->segment(0, num_contacts);
  const auto fF = cf->segment(num_contacts, num_spanning_vectors);
  const auto fL = cf->segment(num_contacts + num_spanning_vectors, num_limits);

  // Determine the accelerations and the bilateral constraint forces.
  //     Au + Xv + a = 0
  //     Yu + Bv + b ≥ 0
  //               v ≥ 0
  // vᵀ(b + Yu + Bv) = 0
  // where u are "free" variables (corresponding to the time derivative of
  // velocities concatenated with bilateral constraint forces). If the matrix A
  // is nonsingular, u can be solved for:
  //      u = -A⁻¹ (a + Xv)
  // allowing the mixed LCP to be converted to a "pure" LCP (q, M) by:
  // q = b - DA⁻¹a
  // M = B - DA⁻¹C
  if (num_eq_constraints > 0) {
    // In this case, Xv = -(Nᵀ + μQᵀ)fN - DᵀfD - LᵀfL and a = | -f |.
    //                                                        | kG |
    const VectorX<T> Xv = -data_ptr->N_minus_muQ_transpose_mult(fN) -
                          data_ptr->F_transpose_mult(fF) -
                          data_ptr->L_transpose_mult(fL);
    VectorX<T> aug = a;
    aug.head(Xv.size()) += Xv;
    const VectorX<T> u = -mlcp_to_lcp_data.A_solve(aug);
    auto lambda = cf->segment(num_contacts + num_spanning_vectors + num_limits,
                              num_eq_constraints);
    lambda = u.tail(num_eq_constraints);
  }
}

template <typename T>
void ConstraintSolver<T>::SolveImpactProblem(
    const ConstraintVelProblemData<T>& problem_data, VectorX<T>* cf) const {
  using std::abs;
  using std::max;

  if (!cf) throw std::logic_error("cf (output parameter) is null.");

  // Get number of contacts and limits.
  const int num_contacts = problem_data.mu.size();
  if (static_cast<size_t>(num_contacts) != problem_data.r.size()) {
    throw std::logic_error(
        "Number of elements in 'r' does not match number"
        "of elements in 'mu'");
  }
  const int num_limits = problem_data.kL.size();
  const int num_eq_constraints = problem_data.kG.size();

  // Look for fast exit.
  if (num_contacts == 0 && num_limits == 0 && num_eq_constraints == 0) {
    cf->resize(0);
    return;
  }

  // Get number of tangent spanning vectors.
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);

  // Determine the pre-impact velocity.
  const VectorX<T> v = problem_data.solve_inertia(problem_data.Mv);

  // If no impact and no bilateral constraints, do not apply the impact model.
  // (We avoid this calculation if there are bilateral constraints because it's
  // too hard to determine a workable tolerance at this point).
  const VectorX<T> N_eval = problem_data.N_mult(v) + problem_data.kN;
  const VectorX<T> L_eval = problem_data.L_mult(v) + problem_data.kL;
  if ((num_contacts == 0 || N_eval.minCoeff() >= 0) &&
      (num_limits == 0 || L_eval.minCoeff() >= 0) &&
      (num_eq_constraints == 0)) {
    cf->setZero(num_contacts + num_spanning_vectors + num_limits);
    return;
  }

  // Initialize contact force vector.
  cf->resize(num_contacts + num_spanning_vectors + num_limits +
             num_eq_constraints);

  // Construct the operators required to "factor out" the bilateral constraints
  // through conversion of a mixed linear complementarity problem into a "pure"
  // linear complementarity problem. See
  // ConstructLinearEquationSolversForMlcp() for more information.
  MlcpToLcpData mlcp_to_lcp_data;
  ConstructLinearEquationSolversForMlcp(problem_data, &mlcp_to_lcp_data);

  // Copy the problem data and then update it to account for bilateral
  // constraints.
  const int gv_dim = problem_data.Mv.size();
  ConstraintVelProblemData<T> modified_problem_data(gv_dim +
                                                    num_eq_constraints);
  ConstraintVelProblemData<T>* data_ptr = &modified_problem_data;
  data_ptr = UpdateProblemDataForUnilateralConstraints(
      problem_data, mlcp_to_lcp_data.fast_A_solve, gv_dim, data_ptr);

  // Compute a and A⁻¹a.
  const VectorX<T>& Mv = problem_data.Mv;
  VectorX<T> a(Mv.size() + num_eq_constraints);
  a.head(Mv.size()) = -Mv;
  a.tail(num_eq_constraints) = problem_data.kG;
  const VectorX<T> invA_a = mlcp_to_lcp_data.A_solve(a);
  const VectorX<T> trunc_neg_invA_a = -invA_a.head(Mv.size());

  // Construct the linear complementarity problem.
  MatrixX<T> MM;
  VectorX<T> qq;
  FormImpactingConstraintLcp(problem_data, trunc_neg_invA_a, &MM, &qq);

  // Get the tolerance for zero used by the LCP solver.
  const T zero_tol = lcp_.ComputeZeroTolerance(MM);

  // Solve the LCP and compute the values of the slack variables.
  VectorX<T> zz;
  bool success = lcp_.SolveLcpLemke(MM, qq, &zz, -1, zero_tol);
  VectorX<T> ww = MM * zz + qq;
  const T max_dot =
      (zz.size() > 0) ? (zz.array() * ww.array()).abs().maxCoeff() : 0.0;

  // Check the answer and solve using progressive regularization if necessary.
  const int num_vars = qq.size();
  const int npivots = std::max(lcp_.get_num_pivots(), 1);
  if (!success ||
      (zz.size() > 0 &&
       (zz.minCoeff() < -num_vars * npivots * zero_tol ||
        ww.minCoeff() < -num_vars * npivots * zero_tol ||
        max_dot > max(T(1), zz.maxCoeff()) * max(T(1), ww.maxCoeff()) *
                      num_vars * npivots * zero_tol))) {
    // Report difficulty
    DRAKE_LOGGER_DEBUG(
        "Unable to solve impacting problem LCP without "
        "progressive regularization");
    DRAKE_LOGGER_DEBUG("zero tolerance for z/w: {}",
                       num_vars * npivots * zero_tol);
    DRAKE_LOGGER_DEBUG("Solver reports success? {}", success);
    DRAKE_LOGGER_DEBUG("minimum z: {}", zz.minCoeff());
    DRAKE_LOGGER_DEBUG("minimum w: {}", ww.minCoeff());
    DRAKE_LOGGER_DEBUG("zero tolerance for <z,w>: {}",
                       max(T(1), zz.maxCoeff()) * max(T(1), ww.maxCoeff()) *
                           num_vars * npivots * zero_tol);
    DRAKE_LOGGER_DEBUG("z'w: {}", max_dot);

    // Use progressive regularization to solve.
    const int min_exp = -16;      // Minimum regularization factor: 1e-16.
    const unsigned step_exp = 1;  // Regularization progressively increases by a
                                  // factor of ten.
    const int max_exp = 1;        // Maximum regularization: 1e1.
    const double piv_tol = -1;    // Make solver compute the pivot tolerance.
    if (!lcp_.SolveLcpLemkeRegularized(MM, qq, &zz, min_exp, step_exp, max_exp,
                                       piv_tol, zero_tol)) {
      throw std::runtime_error("Progressively regularized LCP solve failed.");
    } else {
      ww = MM * zz + qq;
      DRAKE_LOGGER_DEBUG("minimum z: {}", zz.minCoeff());
      DRAKE_LOGGER_DEBUG("minimum w: {}", ww.minCoeff());
      DRAKE_LOGGER_DEBUG("z'w: ", (zz.array() * ww.array()).abs().maxCoeff());
    }
  }

  // Construct the packed force vector.
  PopulatePackedConstraintForcesFromLcpSolution(problem_data, mlcp_to_lcp_data,
                                                zz, a, cf);
}

template <typename T>
void ConstraintSolver<T>::ConstructLinearEquationSolversForMlcp(
    const ConstraintVelProblemData<T>& problem_data,
    MlcpToLcpData* mlcp_to_lcp_data) {
  // --------------------------------------------------------------------------
  // Using the LCP solution to solve the MLCP.
  // --------------------------------------------------------------------------

  // From Equation (e) and the solution to the LCP (v), we can solve for u using
  // the following equations:
  // Xv + a = | -NᵀfN - DᵀfD - LᵀfL - Mv |
  //          |            kᴳ            |
  //

  // Prepare to set up the functionals to compute Ax = b, where A is the
  // blocked saddle point matrix containing the generalized inertia matrix
  // and the bilateral constraints *assuming there are bilateral constraints*.
  // If there are no bilateral constraints, A_solve and fast_A_solve will
  // simply point to the inertia solve operator.

  // Form the Delassus matrix for the bilateral constraints.
  const int num_generalized_velocities = problem_data.Mv.size();
  const int num_eq_constraints = problem_data.kG.size();
  if (num_eq_constraints > 0) {
    MatrixX<T> Del(num_eq_constraints, num_eq_constraints);
    MatrixX<T> iM_GT(num_generalized_velocities, num_eq_constraints);
    ComputeInverseInertiaTimesGT(problem_data.solve_inertia,
                                 problem_data.G_transpose_mult,
                                 num_eq_constraints, &iM_GT);
    ComputeConstraintSpaceComplianceMatrix(problem_data.G_mult,
                                           num_eq_constraints, iM_GT, Del);

    // Compute the complete orthogonal factorization.
    mlcp_to_lcp_data->delassus_QTZ.compute(Del);

    // Determine a new "inertia" solve operator, which solves AX = B, where
    // A = | M  -Gᵀ |
    //     | G   0  |
    // using the newly reduced set of constraints. This will allow transforming
    // the mixed LCP into a pure LCP.
    DetermineNewFullInertiaSolveOperator(
        &problem_data, num_generalized_velocities,
        &mlcp_to_lcp_data->delassus_QTZ, &mlcp_to_lcp_data->A_solve);

    // Determine a new "inertia" solve operator, using only the upper left block
    // of A⁻¹ to exploit zeros in common operations.
    DetermineNewPartialInertiaSolveOperator(
        &problem_data, num_generalized_velocities,
        &mlcp_to_lcp_data->delassus_QTZ, &mlcp_to_lcp_data->fast_A_solve);
  } else {
    mlcp_to_lcp_data->A_solve = problem_data.solve_inertia;
    mlcp_to_lcp_data->fast_A_solve = problem_data.solve_inertia;
  }
}

// Populates the packed constraint force vector from the solution to the
// linear complementarity problem (LCP).
// @param problem_data the constraint problem data.
// @param a reference to a MlcpToLcpData object.
// @param a the vector `a` output from UpdateDiscretizedTimeLcp().
// @param[out] cf the constraint forces, on return.
// @pre cf is non-null.
template <typename T>
void ConstraintSolver<T>::PopulatePackedConstraintForcesFromLcpSolution(
    const ConstraintVelProblemData<T>& problem_data,
    const MlcpToLcpData& mlcp_to_lcp_data, const VectorX<T>& zz,
    const VectorX<T>& a, VectorX<T>* cf) {
  PopulatePackedConstraintForcesFromLcpSolution(problem_data, mlcp_to_lcp_data,
                                                zz, a, 1.0, cf);
}

template <typename T>
void ConstraintSolver<T>::PopulatePackedConstraintForcesFromLcpSolution(
    const ConstraintVelProblemData<T>& problem_data,
    const MlcpToLcpData& mlcp_to_lcp_data, const VectorX<T>& zz,
    const VectorX<T>& a, double dt, VectorX<T>* cf) {
  // Resize the force vector.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_eq_constraints = problem_data.kG.size();
  cf->resize(num_contacts + num_spanning_vectors + num_limits +
             num_eq_constraints);

  // Quit early if zz is empty.
  if (zz.size() == 0) {
    cf->setZero();
    if (num_eq_constraints > 0) {
      const VectorX<T> u = -mlcp_to_lcp_data.A_solve(a);
      auto lambda = cf->segment(
          num_contacts + num_spanning_vectors + num_limits, num_eq_constraints);

      // Transform the impulsive forces to non-impulsive forces.
      lambda = u.tail(num_eq_constraints) / dt;
      DRAKE_LOGGER_DEBUG("Bilateral constraint forces/impulses: {}",
                         fmt_eigen(lambda));
    }

    return;
  }

  // Alias constraint force segments.
  const auto fN = zz.segment(0, num_contacts);
  const auto fD_plus = zz.segment(num_contacts, num_spanning_vectors);
  const auto fD_minus =
      zz.segment(num_contacts + num_spanning_vectors, num_spanning_vectors);
  const auto fL =
      zz.segment(num_contacts * 2 + num_spanning_vectors * 2, num_limits);
  const auto fF = cf->segment(num_contacts, num_spanning_vectors);

  // Get the constraint forces in the specified packed storage format.
  cf->segment(0, num_contacts) = fN;
  cf->segment(num_contacts, num_spanning_vectors) = fD_plus - fD_minus;
  cf->segment(num_contacts + num_spanning_vectors, num_limits) = fL;
  DRAKE_LOGGER_DEBUG("Normal contact forces/impulses: {}", fmt_eigen(fN));
  DRAKE_LOGGER_DEBUG("Frictional contact forces/impulses: {}",
                     fmt_eigen(fD_plus - fD_minus));
  DRAKE_LOGGER_DEBUG(
      "Generic unilateral constraint "
      "forces/impulses: {}",
      fmt_eigen(fL));

  // Determine the new velocity and the bilateral constraint forces/
  // impulses.
  //     Au + Xv + a = 0
  //     Yu + Bv + b ≥ 0
  //               v ≥ 0
  // vᵀ(b + Yu + Bv) = 0
  // where u are "free" variables (corresponding to new velocities
  // concatenated with bilateral constraint forces/impulses). If
  // the matrix A is nonsingular, u can be solved for:
  //      u = -A⁻¹ (a + Xv)
  // allowing the mixed LCP to be converted to a "pure" LCP (q, M) by:
  // q = b - DA⁻¹a
  // M = B - DA⁻¹C
  if (num_eq_constraints > 0) {
    // In this case, Xv = -NᵀfN - DᵀfD -LᵀfL and a = | -Mv(t) |.
    //                                               |   kG   |
    // First, make the forces impulsive.
    const VectorX<T> Xv = (-problem_data.N_transpose_mult(fN) -
                           problem_data.F_transpose_mult(fF) -
                           problem_data.L_transpose_mult(fL)) *
                          dt;
    VectorX<T> aug = a;
    aug.head(Xv.size()) += Xv;
    const VectorX<T> u = -mlcp_to_lcp_data.A_solve(aug);
    auto lambda = cf->segment(num_contacts + num_spanning_vectors + num_limits,
                              num_eq_constraints);

    // Transform the impulsive forces back to non-impulsive forces.
    lambda = u.tail(num_eq_constraints) / dt;
    DRAKE_LOGGER_DEBUG("Bilateral constraint forces/impulses: {}",
                       fmt_eigen(lambda));
  }
}

template <typename T>
void ConstraintSolver<T>::UpdateDiscretizedTimeLcp(
    const ConstraintVelProblemData<T>& problem_data, double dt,
    MlcpToLcpData* mlcp_to_lcp_data, VectorX<T>* a, MatrixX<T>* MM,
    VectorX<T>* qq) {
  DRAKE_DEMAND(MM != nullptr);
  DRAKE_DEMAND(qq != nullptr);
  DRAKE_DEMAND(a != nullptr);

  // Look for early exit.
  if (qq->rows() == 0) return;

  // Recompute the linear equation solvers, if necessary.
  if (problem_data.kG.size() > 0) {
    ConstructLinearEquationSolversForMlcp(problem_data, mlcp_to_lcp_data);
  }

  // Compute a and A⁻¹a.
  const int num_eq_constraints = problem_data.kG.size();
  const VectorX<T>& Mv = problem_data.Mv;
  a->resize(Mv.size() + num_eq_constraints);
  a->head(Mv.size()) = -Mv;
  a->tail(num_eq_constraints) = problem_data.kG;
  const VectorX<T> invA_a = mlcp_to_lcp_data->A_solve(*a);
  const VectorX<T> trunc_neg_invA_a = -invA_a.head(Mv.size());

  // Look for quick exit.
  if (qq->rows() == 0) return;

  // Get numbers of contacts.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();

  // Alias these variables for more readable construction of MM and qq.
  const int nc = num_contacts;
  const int nr = num_spanning_vectors;
  const int nk = nr * 2;
  const int nl = num_limits;

  // Alias operators to make accessing them less clunky.
  const auto N = problem_data.N_mult;
  const auto F = problem_data.F_mult;
  const auto L = problem_data.L_mult;

  // Verify that all gamma vectors are either empty or non-negative.
  const VectorX<T>& gammaN = problem_data.gammaN;
  const VectorX<T>& gammaF = problem_data.gammaF;
  const VectorX<T>& gammaE = problem_data.gammaE;
  const VectorX<T>& gammaL = problem_data.gammaL;
  DRAKE_DEMAND(gammaN.size() == 0 || gammaN.minCoeff() >= 0);
  DRAKE_DEMAND(gammaF.size() == 0 || gammaF.minCoeff() >= 0);
  DRAKE_DEMAND(gammaE.size() == 0 || gammaE.minCoeff() >= 0);
  DRAKE_DEMAND(gammaL.size() == 0 || gammaL.minCoeff() >= 0);

  // Scale the Delassus matrices, which are all but the third row (block) and
  // third column (block) of the following matrix.
  // N⋅M⁻¹⋅Nᵀ  N⋅M⁻¹⋅Dᵀ  0   N⋅M⁻¹⋅Lᵀ
  // D⋅M⁻¹⋅Nᵀ  D⋅M⁻¹⋅Dᵀ  E   D⋅M⁻¹⋅Lᵀ
  // μ         -Eᵀ       0   0
  // L⋅M⁻¹⋅Nᵀ  L⋅M⁻¹⋅Dᵀ  0   L⋅M⁻¹⋅Lᵀ
  // where D = |  F |
  //           | -F |
  const int nr2 = nr * 2;
  MM->topLeftCorner(nc + nr2, nc + nr2) *= dt;
  MM->bottomLeftCorner(nl, nc + nr2) *= dt;
  MM->topRightCorner(nc + nr2, nl) *= dt;
  MM->bottomRightCorner(nl, nl) *= dt;

  // Regularize the LCP matrix.
  MM->topLeftCorner(nc, nc) += Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaN);
  MM->block(nc, nc, nr, nr) += Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaF);
  MM->block(nc + nr, nc + nr, nr, nr) +=
      Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaF);
  MM->block(nc + nk, nc + nk, nc, nc) +=
      Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaE);
  MM->block(nc * 2 + nk, nc * 2 + nk, nl, nl) +=
      Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaL);

  // Update qq.
  qq->segment(0, nc) = N(trunc_neg_invA_a) + problem_data.kN;
  qq->segment(nc, nr) = F(trunc_neg_invA_a) + problem_data.kF;
  qq->segment(nc + nr, nr) = -qq->segment(nc, nr);
  qq->segment(nc + nk, nc).setZero();
  qq->segment(nc * 2 + nk, num_limits) = L(trunc_neg_invA_a) + problem_data.kL;
}

template <typename T>
void ConstraintSolver<T>::ConstructBaseDiscretizedTimeLcp(
    const ConstraintVelProblemData<T>& problem_data,
    MlcpToLcpData* mlcp_to_lcp_data, MatrixX<T>* MM, VectorX<T>* qq) {
  DRAKE_DEMAND(MM != nullptr);
  DRAKE_DEMAND(qq != nullptr);
  DRAKE_DEMAND(mlcp_to_lcp_data != nullptr);

  // Get number of contacts and limits.
  const int num_contacts = problem_data.mu.size();
  if (static_cast<size_t>(num_contacts) != problem_data.r.size()) {
    throw std::logic_error(
        "Number of elements in 'r' does not match number"
        "of elements in 'mu'");
  }
  const int num_limits = problem_data.kL.size();
  const int num_eq_constraints = problem_data.kG.size();

  // Look for fast exit.
  if (num_contacts == 0 && num_limits == 0 && num_eq_constraints == 0) {
    MM->resize(0, 0);
    qq->resize(0);
    return;
  }

  // If no impact and no bilateral constraints, construct an empty matrix
  // and vector. (We avoid this possible shortcut if there are bilateral
  // constraints because it's too hard to determine a workable tolerance at
  // this point).
  const VectorX<T> v = problem_data.solve_inertia(problem_data.Mv);
  const VectorX<T> N_eval = problem_data.N_mult(v) + problem_data.kN;
  const VectorX<T> L_eval = problem_data.L_mult(v) + problem_data.kL;
  if ((num_contacts == 0 || N_eval.minCoeff() >= 0) &&
      (num_limits == 0 || L_eval.minCoeff() >= 0) &&
      (num_eq_constraints == 0)) {
    MM->resize(0, 0);
    qq->resize(0);
    return;
  }

  // Determine the "A" and fast "A" solution operators, which allow us to
  // solve the mixed linear complementarity problem by first solving a "pure"
  // linear complementarity problem. See @ref Velocity-level-MLCPs in
  // Doxygen documentation (above).
  ConstructLinearEquationSolversForMlcp(problem_data, mlcp_to_lcp_data);

  // Allocate storage for a.
  VectorX<T> a(problem_data.Mv.size() + num_eq_constraints);

  // Compute a and A⁻¹a.
  const VectorX<T>& Mv = problem_data.Mv;
  a.head(Mv.size()) = -Mv;
  a.tail(num_eq_constraints) = problem_data.kG;
  const VectorX<T> invA_a = mlcp_to_lcp_data->A_solve(a);
  const VectorX<T> trunc_neg_invA_a = -invA_a.head(Mv.size());

  // Set up the linear complementarity problem.
  FormImpactingConstraintLcp(problem_data, trunc_neg_invA_a, MM, qq);
}

template <class T>
void ConstraintSolver<T>::ComputeConstraintSpaceComplianceMatrix(
    std::function<VectorX<T>(const VectorX<T>&)> A_mult, int a,
    const MatrixX<T>& iM_BT, Eigen::Ref<MatrixX<T>> A_iM_BT) {
  const int b = iM_BT.cols();
  DRAKE_DEMAND(A_iM_BT.rows() == a && A_iM_BT.cols() == b);

  // Look for fast exit.
  if (a == 0 || b == 0) return;

  VectorX<T> iM_bT;  // Intermediate result vector.

  for (int i = 0; i < b; ++i) {
    iM_bT = iM_BT.col(i);
    A_iM_BT.col(i) = A_mult(iM_bT);
  }
}

template <class T>
void ConstraintSolver<T>::ComputeInverseInertiaTimesGT(
    std::function<MatrixX<T>(const MatrixX<T>&)> M_inv_mult,
    std::function<VectorX<T>(const VectorX<T>&)> G_transpose_mult, int m,
    MatrixX<T>* iM_GT) {
  DRAKE_DEMAND(iM_GT != nullptr);
  DRAKE_DEMAND(iM_GT->cols() == m);

  VectorX<T> basis(m);  // Basis vector.
  VectorX<T> gT;        // Intermediate result vector.

  // Look for fast exit.
  if (m == 0) return;

  for (int i = 0; i < m; ++i) {
    // Get the i'th column of G.
    basis.setZero();
    basis[i] = 1;
    gT = G_transpose_mult(basis);
    iM_GT->col(i) = M_inv_mult(gT);
  }
}

template <class T>
void ConstraintSolver<T>::CheckAccelConstraintMatrix(
    const ConstraintAccelProblemData<T>& problem_data, const MatrixX<T>& MM) {
  // Get numbers of types of contacts.
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();

  // Alias operators and vectors to make accessing them less clunky.
  auto FT = problem_data.F_transpose_mult;
  auto L = problem_data.L_mult;
  auto iM = problem_data.solve_inertia;

  // Alias these variables for more readable construction of MM and qq.
  const int ngv = problem_data.tau.size();  // generalized velocity dimension.
  const int nr = num_spanning_vectors;
  const int nk = nr * 2;
  const int nl = num_limits;
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;

  // Get the block of M that was set through a transposition operation.
  Eigen::Ref<const MatrixX<T>> L_iM_FT =
      MM.block(num_contacts + nk + num_non_sliding, num_contacts, nl, nr);

  // Compute the block from scratch.
  MatrixX<T> L_iM_FT_true(nl, nr);
  MatrixX<T> iM_FT(ngv, nr);
  ComputeInverseInertiaTimesGT(iM, FT, nr, &iM_FT);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_FT, L_iM_FT_true);

  // Determine the zero tolerance.
  const double zero_tol =
      std::numeric_limits<double>::epsilon() * MM.norm() * MM.rows();

  // Check that the blocks are nearly equal.
  DRAKE_ASSERT((L_iM_FT - L_iM_FT_true).norm() < zero_tol);
}

template <class T>
void ConstraintSolver<T>::FormSustainedConstraintLinearSystem(
    const ConstraintAccelProblemData<T>& problem_data,
    const VectorX<T>& trunc_neg_invA_a, MatrixX<T>* MM, VectorX<T>* qq) {
  DRAKE_DEMAND(MM != nullptr);
  DRAKE_DEMAND(qq != nullptr);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Alias operators and vectors to make accessing them less clunky.
  auto N = problem_data.N_mult;
  auto F = problem_data.F_mult;
  auto FT = problem_data.F_transpose_mult;
  auto L = problem_data.L_mult;
  auto LT = problem_data.L_transpose_mult;
  auto iM = problem_data.solve_inertia;
  const VectorX<T>& kN = problem_data.kN;
  const VectorX<T>& kF = problem_data.kF;
  const VectorX<T>& kL = problem_data.kL;

  // Alias these variables for more readable construction of MM and qq.
  const int ngv = problem_data.tau.size();  // generalized velocity dimension.
  const int nc = num_contacts;
  const int nr = num_spanning_vectors;
  const int nl = num_limits;
  const int num_vars = nc + nr + nl;

  // Precompute some matrices that will be reused repeatedly.
  MatrixX<T> iM_NT_minus_muQT(ngv, nc), iM_FT(ngv, nr), iM_LT(ngv, nl);
  ComputeInverseInertiaTimesGT(iM, problem_data.N_minus_muQ_transpose_mult, nc,
                               &iM_NT_minus_muQT);
  ComputeInverseInertiaTimesGT(iM, FT, nr, &iM_FT);
  ComputeInverseInertiaTimesGT(iM, LT, nl, &iM_LT);

  // Name the blocks of the matrix, which takes the form:
  // N⋅M⁻¹⋅(Nᵀ - μₛQᵀ)  N⋅M⁻¹⋅Fᵀ  N⋅M⁻¹⋅Lᵀ
  // F⋅M⁻¹⋅(Nᵀ - μₛQᵀ)  F⋅M⁻¹⋅Fᵀ  D⋅M⁻¹⋅Lᵀ
  // L⋅M⁻¹⋅(Nᵀ - μₛQᵀ)  L⋅M⁻¹⋅Fᵀ  L⋅M⁻¹⋅Lᵀ
  MM->resize(num_vars, num_vars);
  Eigen::Ref<MatrixX<T>> N_iM_NT_minus_muQT = MM->block(0, 0, nc, nc);
  Eigen::Ref<MatrixX<T>> N_iM_FT = MM->block(0, nc, nc, nr);
  Eigen::Ref<MatrixX<T>> N_iM_LT = MM->block(0, nc + nr, nc, nl);
  Eigen::Ref<MatrixX<T>> F_iM_NT_minus_muQT = MM->block(nc, 0, nr, nc);
  Eigen::Ref<MatrixX<T>> F_iM_FT = MM->block(nc, nc, nr, nr);
  Eigen::Ref<MatrixX<T>> F_iM_LT = MM->block(nc, nc + nr, nr, nl);
  Eigen::Ref<MatrixX<T>> L_iM_NT_minus_muQT = MM->block(nc + nr, 0, nl, nc);
  Eigen::Ref<MatrixX<T>> L_iM_FT = MM->block(nc + nr, nc, nl, nr);
  Eigen::Ref<MatrixX<T>> L_iM_LT = MM->block(nc + nr, nc + nr, nl, nl);

  // Compute the blocks.
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_NT_minus_muQT,
                                         N_iM_NT_minus_muQT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_FT, N_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_LT, N_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_NT_minus_muQT,
                                         F_iM_NT_minus_muQT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_FT, F_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_LT, F_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_NT_minus_muQT,
                                         L_iM_NT_minus_muQT);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_LT, L_iM_LT);
  L_iM_FT = F_iM_LT.transpose().eval();

  // Construct the vector:
  // N⋅A⁻¹⋅a + kN
  // F⋅A⁻¹⋅a + kD
  // L⋅A⁻¹⋅a + kL
  qq->resize(num_vars, 1);
  qq->segment(0, nc) = N(trunc_neg_invA_a) + kN;
  qq->segment(nc, nr) = F(trunc_neg_invA_a) + kF;
  qq->segment(nc + nr, num_limits) = L(trunc_neg_invA_a) + kL;
}

template <class T>
void ConstraintSolver<T>::FormSustainedConstraintLcp(
    const ConstraintAccelProblemData<T>& problem_data,
    const VectorX<T>& trunc_neg_invA_a, MatrixX<T>* MM, VectorX<T>* qq) {
  DRAKE_DEMAND(MM != nullptr);
  DRAKE_DEMAND(qq != nullptr);

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();

  // Problem matrices and vectors are mildly adapted from:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Alias operators and vectors to make accessing them less clunky.
  auto N = problem_data.N_mult;
  auto F = problem_data.F_mult;
  auto FT = problem_data.F_transpose_mult;
  auto L = problem_data.L_mult;
  auto LT = problem_data.L_transpose_mult;
  auto iM = problem_data.solve_inertia;
  const VectorX<T>& kN = problem_data.kN;
  const VectorX<T>& kF = problem_data.kF;
  const VectorX<T>& kL = problem_data.kL;
  const VectorX<T>& mu_non_sliding = problem_data.mu_non_sliding;

  // Construct a matrix similar to E in [Anitescu 1997]. This matrix will be
  // used to specify the constraints (adapted from [Anitescu 1997] Eqn 2.7):
  // 0 ≤  μₙₛ fNᵢ - eᵀ fF  ⊥  λᵢ ≥ 0 and
  // 0 ≤ e λᵢ + F dv/dt + dF/dt v ⊥ fF ≥ 0,
  // where scalar λᵢ can roughly be interpreted as the remaining tangential
  // acceleration at non-sliding contact i after frictional forces have been
  // applied and e is a vector of ones (i.e., a segment of the appropriate
  // column of E). Note that this matrix differs from the exact definition of
  // E in [Anitescu 1997] to reflect the different layout of the LCP matrix
  // from [Anitescu 1997] (the latter puts all spanning directions corresponding
  // to a contact in one block; we put half of the spanning directions
  // corresponding to a contact into one block and the other half into another
  // block).
  MatrixX<T> E = MatrixX<T>::Zero(num_spanning_vectors, num_non_sliding);
  for (int i = 0, j = 0; i < num_non_sliding; ++i) {
    E.col(i).segment(j, problem_data.r[i]).setOnes();
    j += problem_data.r[i];
  }

  // Alias these variables for more readable construction of MM and qq.
  const int ngv = problem_data.tau.size();  // generalized velocity dimension.
  const int nc = num_contacts;
  const int nr = num_spanning_vectors;
  const int nk = nr * 2;
  const int nl = num_limits;
  const int num_vars = nc + nk + num_non_sliding + nl;

  // Precompute some matrices that will be reused repeatedly.
  MatrixX<T> iM_NT_minus_muQT(ngv, nc), iM_FT(ngv, nr), iM_LT(ngv, nl);
  ComputeInverseInertiaTimesGT(iM, problem_data.N_minus_muQ_transpose_mult, nc,
                               &iM_NT_minus_muQT);
  ComputeInverseInertiaTimesGT(iM, FT, nr, &iM_FT);
  ComputeInverseInertiaTimesGT(iM, LT, nl, &iM_LT);

  // Prepare blocks of the LCP matrix, which takes the form:
  // N⋅M⁻¹⋅(Nᵀ - μₛQᵀ)  N⋅M⁻¹⋅Dᵀ  0   N⋅M⁻¹⋅Lᵀ
  // D⋅M⁻¹⋅(Nᵀ - μₛQᵀ)  D⋅M⁻¹⋅Dᵀ  E   D⋅M⁻¹⋅Lᵀ
  // μ                 -Eᵀ        0   0
  // L⋅M⁻¹⋅(Nᵀ - μₛQᵀ)  L⋅M⁻¹⋅Dᵀ  0   L⋅M⁻¹⋅Lᵀ
  // where D = |  F |
  //           | -F |
  MM->resize(num_vars, num_vars);
  Eigen::Ref<MatrixX<T>> N_iM_NT_minus_muQT = MM->block(0, 0, nc, nc);
  Eigen::Ref<MatrixX<T>> N_iM_FT = MM->block(0, nc, nc, nr);
  Eigen::Ref<MatrixX<T>> N_iM_LT =
      MM->block(0, nc + nk + num_non_sliding, nc, nl);
  Eigen::Ref<MatrixX<T>> F_iM_NT_minus_muQT = MM->block(nc, 0, nr, nc);
  Eigen::Ref<MatrixX<T>> F_iM_FT = MM->block(nc, nc, nr, nr);
  Eigen::Ref<MatrixX<T>> F_iM_LT =
      MM->block(nc, nc + nk + num_non_sliding, nr, nl);
  Eigen::Ref<MatrixX<T>> L_iM_NT_minus_muQT =
      MM->block(nc + nk + num_non_sliding, 0, nl, nc);
  Eigen::Ref<MatrixX<T>> L_iM_LT =
      MM->block(nc + nk + num_non_sliding, nc + nk + num_non_sliding, nl, nl);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_NT_minus_muQT,
                                         N_iM_NT_minus_muQT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_FT, N_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_LT, N_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_NT_minus_muQT,
                                         F_iM_NT_minus_muQT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_FT, F_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_LT, F_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_NT_minus_muQT,
                                         L_iM_NT_minus_muQT);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_LT, L_iM_LT);

  // Construct the LCP matrix. First do the "normal contact direction" rows.
  MM->block(0, nc + nr, nc, nr) = -MM->block(0, nc, nc, nr);
  MM->block(0, nc + nk, nc, num_non_sliding).setZero();

  // Now construct the un-negated tangent contact direction rows.
  MM->block(nc, nc + nr, num_spanning_vectors, nr) =
      -MM->block(nc, nc, nr, num_spanning_vectors);
  MM->block(nc, nc + nk, num_spanning_vectors, num_non_sliding) = E;

  // Now construct the negated tangent contact direction rows. These negated
  // tangent contact directions allow the LCP to compute forces applied along
  // the negative x-axis. E will have to be reset to un-negate it.
  MM->block(nc + nr, 0, nr, MM->cols()) = -MM->block(nc, 0, nr, MM->cols());
  MM->block(nc + nr, nc + nk, num_spanning_vectors, num_non_sliding) = E;

  // Construct the next block, which provides the friction "cone" constraint.
  const std::vector<int>& ns_contacts = problem_data.non_sliding_contacts;
  MM->block(nc + nk, 0, num_non_sliding, nc).setZero();
  for (int i = 0; static_cast<size_t>(i) < ns_contacts.size(); ++i)
    (*MM)(nc + nk + i, ns_contacts[i]) = mu_non_sliding[i];
  MM->block(nc + nk, nc, num_non_sliding, num_spanning_vectors) =
      -E.transpose();
  MM->block(nc + nk, nc + num_spanning_vectors, num_non_sliding,
            num_spanning_vectors) = -E.transpose();
  MM->block(nc + nk, nc + nk, num_non_sliding, num_non_sliding + nl).setZero();

  // Construct the last row block, which provides the generic unilateral
  // constraints.
  MM->block(nc + nk + num_non_sliding, 0, nl, nc + nk + num_non_sliding) =
      MM->block(0, nc + nk + num_non_sliding, nc + nk + num_non_sliding, nl)
          .transpose()
          .eval();

  // Check the transposed blocks of the LCP matrix.
  DRAKE_ASSERT_VOID(CheckAccelConstraintMatrix(problem_data, *MM));

  // Construct the LCP vector:
  // N⋅A⁻¹⋅a + kN
  // D⋅A⁻¹⋅a + kD
  // 0
  // L⋅A⁻¹⋅a + kL
  // where, as above, D is defined as [F -F] (and kD is defined as [kF -kF].
  qq->resize(num_vars, 1);
  qq->segment(0, nc) = N(trunc_neg_invA_a) + kN;
  qq->segment(nc, nr) = F(trunc_neg_invA_a) + kF;
  qq->segment(nc + nr, nr) = -qq->segment(nc, nr);
  qq->segment(nc + nk, num_non_sliding).setZero();
  qq->segment(nc + nk + num_non_sliding, num_limits) = L(trunc_neg_invA_a) + kL;
}

template <class T>
void ConstraintSolver<T>::CheckVelConstraintMatrix(
    const ConstraintVelProblemData<T>& problem_data, const MatrixX<T>& MM) {
  // Get numbers of contacts.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();

  // Alias operators and vectors to make accessing them less clunky.
  const auto N = problem_data.N_mult;
  const auto NT = problem_data.N_transpose_mult;
  const auto F = problem_data.F_mult;
  const auto FT = problem_data.F_transpose_mult;
  const auto L = problem_data.L_mult;
  const auto LT = problem_data.L_transpose_mult;
  auto iM = problem_data.solve_inertia;

  // Alias these variables for more readable construction of MM and qq.
  const int ngv = problem_data.Mv.size();  // generalized velocity dimension.
  const int nr = num_spanning_vectors;
  const int nk = nr * 2;
  const int nl = num_limits;

  // Get blocks of M that were set through a transposition operation.
  Eigen::Ref<const MatrixX<T>> F_iM_NT =
      MM.block(num_contacts, 0, nr, num_contacts);
  Eigen::Ref<const MatrixX<T>> L_iM_NT =
      MM.block(num_contacts * 2 + nk, 0, nl, num_contacts);
  Eigen::Ref<const MatrixX<T>> L_iM_FT =
      MM.block(num_contacts * 2 + nk, num_contacts, nl, nr);

  // Compute the blocks from scratch.
  MatrixX<T> F_iM_NT_true(nr, num_contacts), L_iM_NT_true(nl, num_contacts);
  MatrixX<T> L_iM_FT_true(nl, nr);
  MatrixX<T> iM_NT(ngv, num_contacts), iM_FT(ngv, nr);
  ComputeInverseInertiaTimesGT(iM, NT, num_contacts, &iM_NT);
  ComputeInverseInertiaTimesGT(iM, FT, nr, &iM_FT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_NT, F_iM_NT_true);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_NT, L_iM_NT_true);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_FT, L_iM_FT_true);

  // Determine the zero tolerance.
  const double zero_tol =
      std::numeric_limits<double>::epsilon() * MM.norm() * MM.rows();

  // Check that the blocks are nearly equal. Note: these tests are necessary
  // because Eigen does not correctly compute the norm of an empty matrix.
  DRAKE_ASSERT(F_iM_NT.rows() == 0 || F_iM_NT.cols() == 0 ||
               (F_iM_NT - F_iM_NT_true).norm() < zero_tol);
  DRAKE_ASSERT(L_iM_NT.rows() == 0 || L_iM_NT.cols() == 0 ||
               (L_iM_NT - L_iM_NT_true).norm() < zero_tol);
  DRAKE_ASSERT(L_iM_FT.rows() == 0 || L_iM_FT.cols() == 0 ||
               (L_iM_FT - L_iM_FT_true).norm() < zero_tol);
}

template <class T>
void ConstraintSolver<T>::FormImpactingConstraintLcp(
    const ConstraintVelProblemData<T>& problem_data,
    const VectorX<T>& trunc_neg_invA_a, MatrixX<T>* MM, VectorX<T>* qq) {
  DRAKE_DEMAND(MM != nullptr);
  DRAKE_DEMAND(qq != nullptr);

  // Get numbers of contacts.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();

  // Problem matrices and vectors are nearly identical to:
  // M. Anitescu and F. Potra. Formulating Dynamic Multi-Rigid Body Contact
  // Problems as Solvable Linear Complementarity Problems. Nonlinear Dynamics,
  // 14, 1997.

  // Alias operators and vectors to make accessing them less clunky.
  const auto N = problem_data.N_mult;
  const auto NT = problem_data.N_transpose_mult;
  const auto F = problem_data.F_mult;
  const auto FT = problem_data.F_transpose_mult;
  const auto L = problem_data.L_mult;
  const auto LT = problem_data.L_transpose_mult;
  auto iM = problem_data.solve_inertia;
  const VectorX<T>& mu = problem_data.mu;
  const VectorX<T>& gammaN = problem_data.gammaN;
  const VectorX<T>& gammaF = problem_data.gammaF;
  const VectorX<T>& gammaE = problem_data.gammaE;
  const VectorX<T>& gammaL = problem_data.gammaL;

  // Construct the matrix E in [Anitscu 1997]. This matrix will be used to
  // specify the constraints:
  // 0 ≤ μ⋅fN - E⋅fF ⊥ λ ≥ 0 and
  // 0 ≤ e⋅λ + F⋅v ⊥ fF ≥ 0,
  // where λ can roughly be interpreted as the remaining tangential velocity
  // at the impacting contacts after frictional impulses have been applied and
  // e is a vector of ones (i.e., a segment of the appropriate column of E).
  // Note that this matrix differs from the exact definition of E in
  // [Anitescu 1997] to reflect the different layout of the LCP matrix from
  // [Anitescu 1997] (the latter puts all spanning directions corresponding
  // to a contact in one block; we put half of the spanning directions
  // corresponding to a contact into one block and the other half into another
  // block).
  MatrixX<T> E = MatrixX<T>::Zero(num_spanning_vectors, num_contacts);
  for (int i = 0, j = 0; i < num_contacts; ++i) {
    E.col(i).segment(j, problem_data.r[i]).setOnes();
    j += problem_data.r[i];
  }

  // Alias these variables for more readable construction of MM and qq.
  const int ngv = problem_data.Mv.size();  // generalized velocity dimension.
  const int nc = num_contacts;
  const int nr = num_spanning_vectors;
  const int nk = nr * 2;
  const int nl = num_limits;

  // Precompute some matrices that will be reused repeatedly.
  MatrixX<T> iM_NT(ngv, nc), iM_FT(ngv, nr), iM_LT(ngv, nl);
  ComputeInverseInertiaTimesGT(iM, NT, nc, &iM_NT);
  ComputeInverseInertiaTimesGT(iM, FT, nr, &iM_FT);
  ComputeInverseInertiaTimesGT(iM, LT, nl, &iM_LT);

  // Prepare blocks of the LCP matrix, which takes the form:
  // N⋅M⁻¹⋅Nᵀ  N⋅M⁻¹⋅Dᵀ  0   N⋅M⁻¹⋅Lᵀ
  // D⋅M⁻¹⋅Nᵀ  D⋅M⁻¹⋅Dᵀ  E   D⋅M⁻¹⋅Lᵀ
  // μ         -Eᵀ       0   0
  // L⋅M⁻¹⋅Nᵀ  L⋅M⁻¹⋅Dᵀ  0   L⋅M⁻¹⋅Lᵀ
  // where D = |  F |
  //           | -F |
  const int num_vars = nc * 2 + nk + num_limits;
  MM->resize(num_vars, num_vars);
  Eigen::Ref<MatrixX<T>> N_iM_NT = MM->block(0, 0, nc, nc);
  Eigen::Ref<MatrixX<T>> N_iM_FT = MM->block(0, nc, nc, nr);
  Eigen::Ref<MatrixX<T>> N_iM_LT = MM->block(0, nc * 2 + nk, nc, nl);
  Eigen::Ref<MatrixX<T>> F_iM_FT = MM->block(nc, nc, nr, nr);
  Eigen::Ref<MatrixX<T>> F_iM_LT = MM->block(nc, nc * 2 + nk, nr, nl);
  Eigen::Ref<MatrixX<T>> L_iM_LT = MM->block(nc * 2 + nk, nc * 2 + nk, nl, nl);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_NT, N_iM_NT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_FT, N_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(N, nc, iM_LT, N_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_FT, F_iM_FT);
  ComputeConstraintSpaceComplianceMatrix(F, nr, iM_LT, F_iM_LT);
  ComputeConstraintSpaceComplianceMatrix(L, nl, iM_LT, L_iM_LT);

  // Construct the LCP matrix. First do the "normal contact direction" rows:
  MM->block(0, nc + nr, nc, nr) = -MM->block(0, nc, nc, nr);
  MM->block(0, nc + nk, nc, nc).setZero();

  // Now construct the un-negated tangent contact direction rows.
  MM->block(nc, 0, nr, nc) = MM->block(0, nc, nc, nr).transpose().eval();
  MM->block(nc, nc + nr, nr, nr) = -MM->block(nc, nc, nr, nr);
  MM->block(nc, nc + nk, num_spanning_vectors, nc) = E;

  // Now construct the negated tangent contact direction rows. These negated
  // tangent contact directions allow the LCP to compute forces applied along
  // the negative x-axis. E will have to be reset to un-negate it.
  MM->block(nc + nr, 0, nr, MM->cols()) = -MM->block(nc, 0, nr, MM->cols());
  MM->block(nc + nr, nc + nk, num_spanning_vectors, nc) = E;

  // Construct the next two row blocks, which provide the friction "cone"
  // constraint.
  MM->block(nc + nk, 0, nc, nc) = Eigen::DiagonalMatrix<T, Eigen::Dynamic>(mu);
  MM->block(nc + nk, nc, nc, num_spanning_vectors) = -E.transpose();
  MM->block(nc + nk, nc + num_spanning_vectors, nc, num_spanning_vectors) =
      -E.transpose();
  MM->block(nc + nk, nc + nk, nc, nc + nl).setZero();

  // Construct the last row block, which provides the generic unilateral
  // constraints.
  MM->block(nc * 2 + nk, 0, nl, nc * 2 + nk) =
      MM->block(0, nc * 2 + nk, nc * 2 + nk, nl).transpose().eval();

  // Check the transposed blocks of the LCP matrix.
  DRAKE_ASSERT_VOID(CheckVelConstraintMatrix(problem_data, *MM));

  // Verify that all gamma vectors are either empty or non-negative.
  DRAKE_DEMAND(gammaN.size() == 0 || gammaN.minCoeff() >= 0);
  DRAKE_DEMAND(gammaF.size() == 0 || gammaF.minCoeff() >= 0);
  DRAKE_DEMAND(gammaE.size() == 0 || gammaE.minCoeff() >= 0);
  DRAKE_DEMAND(gammaL.size() == 0 || gammaL.minCoeff() >= 0);

  // Regularize the LCP matrix.
  MM->topLeftCorner(nc, nc) += Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaN);
  MM->block(nc, nc, nr, nr) += Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaF);
  MM->block(nc + nr, nc + nr, nr, nr) +=
      Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaF);
  MM->block(nc + nk, nc + nk, nc, nc) +=
      Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaE);
  MM->block(nc * 2 + nk, nc * 2 + nk, nl, nl) +=
      Eigen::DiagonalMatrix<T, Eigen::Dynamic>(gammaL);

  // Construct the LCP vector:
  // NA⁻¹a + kN
  // DA⁻¹a + kD
  // 0
  // LA⁻¹a + kL
  // where, as above, D is defined as [F -F] (and kD = [kF -kF]).
  qq->resize(num_vars, 1);
  qq->segment(0, nc) = N(trunc_neg_invA_a) + problem_data.kN;
  qq->segment(nc, nr) = F(trunc_neg_invA_a) + problem_data.kF;
  qq->segment(nc + nr, nr) = -qq->segment(nc, nr);
  qq->segment(nc + nk, nc).setZero();
  qq->segment(nc * 2 + nk, num_limits) = L(trunc_neg_invA_a) + problem_data.kL;
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedForceFromConstraintForces(
    const ConstraintAccelProblemData<T>& problem_data, const VectorX<T>& cf,
    VectorX<T>* generalized_force) {
  if (!generalized_force)
    throw std::logic_error("generalized_force vector is null.");

  // Get numbers of types of contacts.
  const int num_sliding = problem_data.sliding_contacts.size();
  const int num_non_sliding = problem_data.non_sliding_contacts.size();
  const int num_contacts = num_sliding + num_non_sliding;
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_bilat_constraints = problem_data.kG.size();

  // Verify cf is the correct size.
  const int num_vars =
      num_contacts + num_spanning_vectors + num_limits + num_bilat_constraints;
  if (cf.size() != num_vars) {
    throw std::logic_error(
        "cf (constraint force) parameter incorrectly"
        "sized.");
  }

  /// Get the normal and non-sliding contact forces.
  const Eigen::Ref<const VectorX<T>> f_normal = cf.segment(0, num_contacts);
  const Eigen::Ref<const VectorX<T>> f_non_sliding_frictional =
      cf.segment(num_contacts, num_spanning_vectors);

  /// Get the limit forces.
  const Eigen::Ref<const VectorX<T>> f_limit =
      cf.segment(num_contacts + num_spanning_vectors, num_limits);

  // Get the bilateral constraint forces.
  const Eigen::Ref<const VectorX<T>> f_bilat = cf.segment(
      num_contacts + num_spanning_vectors + num_limits, num_bilat_constraints);

  /// Compute the generalized force.
  *generalized_force = problem_data.N_minus_muQ_transpose_mult(f_normal) +
                       problem_data.F_transpose_mult(f_non_sliding_frictional) +
                       problem_data.L_transpose_mult(f_limit) +
                       problem_data.G_transpose_mult(f_bilat);
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedForceFromConstraintForces(
    const ConstraintVelProblemData<T>& problem_data, const VectorX<T>& cf,
    VectorX<T>* generalized_force) {
  if (!generalized_force)
    throw std::logic_error("generalized_force vector is null.");

  // Look for fast exit.
  if (cf.size() == 0) {
    generalized_force->setZero(problem_data.Mv.size(), 1);
    return;
  }

  // Get number of contacts.
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_bilat_constraints = problem_data.kG.size();

  // Verify cf is the correct size.
  const int num_vars =
      num_contacts + num_spanning_vectors + num_limits + num_bilat_constraints;
  if (num_vars != cf.size()) {
    throw std::logic_error(
        "Unexpected packed constraint force vector"
        " dimension.");
  }

  /// Get the normal and tangential contact forces.
  const Eigen::Ref<const VectorX<T>> f_normal = cf.segment(0, num_contacts);
  const Eigen::Ref<const VectorX<T>> f_frictional =
      cf.segment(num_contacts, num_spanning_vectors);

  /// Get the limit forces.
  const Eigen::Ref<const VectorX<T>> f_limit =
      cf.segment(num_contacts + num_spanning_vectors, num_limits);

  // Get the bilateral constraint forces.
  const Eigen::Ref<const VectorX<T>> f_bilat = cf.segment(
      num_contacts + num_spanning_vectors + num_limits, num_bilat_constraints);

  /// Compute the generalized forces.
  *generalized_force = problem_data.N_transpose_mult(f_normal) +
                       problem_data.F_transpose_mult(f_frictional) +
                       problem_data.L_transpose_mult(f_limit) +
                       problem_data.G_transpose_mult(f_bilat);
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedAcceleration(
    const ConstraintVelProblemData<T>& problem_data, const VectorX<T>& v,
    const VectorX<T>& cf, double dt, VectorX<T>* generalized_acceleration) {
  DRAKE_DEMAND(dt > 0);

  // Keep from allocating storage by reusing `generalized_acceleration`; at
  // first, it will hold the generalized force from constraint forces.
  ComputeGeneralizedForceFromConstraintForces(problem_data, cf,
                                              generalized_acceleration);

  // Using a first-order approximation to velocity, the new velocity is:
  // v(t+dt) = v(t) + dt * ga
  //         = inv(M) * (M * v(t) + dt * gf)
  // where ga is the generalized acceleration and gf is the generalized force.
  // Note: we have no way to break apart the Mv term. But, we can instead
  // compute v(t+dt) and then solve for the acceleration.
  const VectorX<T> vplus = problem_data.solve_inertia(
      problem_data.Mv + dt * (*generalized_acceleration));
  *generalized_acceleration = (vplus - v) / dt;
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedAccelerationFromConstraintForces(
    const ConstraintAccelProblemData<T>& problem_data, const VectorX<T>& cf,
    VectorX<T>* generalized_acceleration) {
  if (!generalized_acceleration)
    throw std::logic_error("generalized_acceleration vector is null.");

  VectorX<T> generalized_force;
  ComputeGeneralizedForceFromConstraintForces(problem_data, cf,
                                              &generalized_force);
  *generalized_acceleration = problem_data.solve_inertia(generalized_force);
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedAccelerationFromConstraintForces(
    const ConstraintVelProblemData<T>& problem_data, const VectorX<T>& cf,
    VectorX<T>* generalized_acceleration) {
  if (!generalized_acceleration)
    throw std::logic_error("generalized_acceleration vector is null.");

  VectorX<T> generalized_force;
  ComputeGeneralizedForceFromConstraintForces(problem_data, cf,
                                              &generalized_force);
  *generalized_acceleration = problem_data.solve_inertia(generalized_force);
}

template <class T>
void ConstraintSolver<T>::ComputeGeneralizedVelocityChange(
    const ConstraintVelProblemData<T>& problem_data, const VectorX<T>& cf,
    VectorX<T>* generalized_delta_v) {
  if (!generalized_delta_v)
    throw std::logic_error("generalized_delta_v vector is null.");

  VectorX<T> generalized_impulse;
  ComputeGeneralizedForceFromConstraintForces(problem_data, cf,
                                              &generalized_impulse);
  *generalized_delta_v = problem_data.solve_inertia(generalized_impulse);
}

template <class T>
void ConstraintSolver<T>::CalcContactForcesInContactFrames(
    const VectorX<T>& cf, const ConstraintAccelProblemData<T>& problem_data,
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
  const int num_contacts =
      problem_data.sliding_contacts.size() + num_non_sliding_contacts;
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_bilat_constraints = problem_data.kG.size();
  const int num_vars =
      num_contacts + num_spanning_vectors + num_limits + num_bilat_constraints;
  if (num_vars != cf.size()) {
    throw std::logic_error(
        "Unexpected packed constraint force vector "
        "dimension.");
  }

  // Verify that the problem is indeed two-dimensional.
  if (num_spanning_vectors != num_non_sliding_contacts) {
    throw std::logic_error(
        "Problem data 'r' indicates contact problem is not "
        "two-dimensional");
  }

  // Verify that the correct number of contact frames has been specified.
  if (contact_frames.size() != static_cast<size_t>(num_contacts)) {
    throw std::logic_error(
        "Number of contact frames does not match number of "
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
      throw std::logic_error(fmt::format(
          "Contact normal ({}) and contact tangent ({}) insufficiently "
          "orthogonal.",
          fmt_eigen(contact_normal), fmt_eigen(contact_tangent)));
    }

    // Initialize the contact force expressed in the global frame.
    Vector2<T> f0(0, 0);

    // Add in the contact normal.
    f0 += contact_normal * cf[i];

    // Determine whether the contact is sliding.
    const bool is_sliding =
        std::binary_search(problem_data.sliding_contacts.begin(),
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
void ConstraintSolver<T>::CalcContactForcesInContactFrames(
    const VectorX<T>& cf, const ConstraintVelProblemData<T>& problem_data,
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
  const int num_contacts = problem_data.mu.size();
  const int num_spanning_vectors =
      std::accumulate(problem_data.r.begin(), problem_data.r.end(), 0);
  const int num_limits = problem_data.kL.size();
  const int num_bilat_constraints = problem_data.kG.size();
  const int num_vars =
      num_contacts + num_spanning_vectors + num_limits + num_bilat_constraints;
  if (num_vars != cf.size()) {
    throw std::logic_error(
        "Unexpected packed constraint force vector "
        "dimension.");
  }

  // Verify that the problem is indeed two-dimensional.
  if (num_spanning_vectors != num_contacts) {
    throw std::logic_error(
        "Problem data 'r' indicates contact problem is not "
        "two-dimensional");
  }

  // Verify that the correct number of contact frames has been specified.
  if (contact_frames.size() != static_cast<size_t>(num_contacts)) {
    throw std::logic_error(
        "Number of contact frames does not match number of "
        "contacts.");
  }

  // Resize the force vector.
  contact_forces->resize(contact_frames.size());

  // Set the forces.
  for (int i = 0, tangent_index = 0; i < num_contacts; ++i) {
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
      throw std::logic_error(fmt::format(
          "Contact normal ({}) and contact tangent ({}) insufficiently "
          "orthogonal.",
          fmt_eigen(contact_normal), fmt_eigen(contact_tangent)));
    }

    // Compute the contact force expressed in the global frame.
    Vector2<T> j0 = contact_normal * cf[i] +
                    contact_tangent * cf[num_contacts + tangent_index++];

    // Compute the contact force in the contact frame.
    contact_force_i = contact_frames[i].transpose() * j0;
  }
}

template class ConstraintSolver<double>;

}  // namespace rod2d
}  // namespace examples
}  // namespace drake
