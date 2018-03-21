#pragma once

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

/// Non-template class for UnrevisedLemkeSolver<T> constants.
class UnrevisedLemkeSolverId {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrevisedLemkeSolverId);
  UnrevisedLemkeSolverId() = delete;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();
};

/// A class for the Unrevised Implementation of Lemke Algorithm's for solving
/// Linear Complementarity Problems (LCPs). See MobyLcpSolver for a description
/// of LCPs.
template <class T>
class UnrevisedLemkeSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnrevisedLemkeSolver)

  UnrevisedLemkeSolver() = default;
  ~UnrevisedLemkeSolver() override = default;

  /// Calculates the zero tolerance that the solver would compute if the user
  /// does not specify a tolerance.
  template <class U>
  static U ComputeZeroTolerance(const MatrixX<U>& M) {
    return M.rows() * M.template lpNorm<Eigen::Infinity>() *
        (10 * std::numeric_limits<double>::epsilon());
  }

  /// Lemke's Algorithm for solving LCPs in the matrix class E, which contains
  /// all strictly semimonotone matrices, all P-matrices, and all strictly
  /// copositive matrices. Lemke's Algorithm is described in [Cottle 1992],
  /// Section 4.4.
  ///
  /// Although this solver is theoretically guaranteed to give a solution to
  /// the LCPs described above, cycling from degeneracy could cause the solver
  /// to fail (by exceeding the maximum number of iterations). Additionally,
  /// the solver can be applied with occasional success to problems outside of
  /// its guaranteed matrix classes. For these reasons, the solver returns a
  /// flag indicating success/failure.
  /// @param[in] M the LCP matrix.
  /// @param[in] q the LCP vector.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use the basis implied by z's
  ///                value as a solution. This strategy can prove exceptionally
  ///                fast if solutions differ little between successive calls.
  ///                If the solver fails (returns `false`),
  ///                `z` will be set to the zero vector on return.
  /// @param[out] num_pivots the number of pivots used, on return.
  /// @param[in] zero_tol The tolerance for testing against zero. If the
  ///            tolerance is negative (default) the solver will determine a
  ///            generally reasonable tolerance.
  /// @returns `true` if the solver **believes** it has computed a solution
  ///          (which it determines by the ability to "pivot out" the
  ///          "artificial" variable (see [Cottle 1992]) and `false` otherwise.
  /// @warning The caller should verify that the algorithm has solved the LCP to
  ///          the desired tolerances on returns indicating success.
  /// @throws std::logic_error if M is not square or the dimensions of M do not
  ///         match the length of q.
  ///
  /// * [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
  ///                      Complementarity Problem. Academic Press, 1992.
  bool SolveLcpLemke(const MatrixX<T>& M, const VectorX<T>& q,
                     VectorX<T>* z, int* num_pivots,
                     const T& zero_tol = T(-1)) const;

  /// Lemke's Algorithm for solving LCPs in the matrix class E, which contains
  /// all strictly semimonotone matrices, all P-matrices, and all strictly
  /// copositive matrices. Lemke's Algorithm is described in [Cottle 1992],
  /// Section 4.4.
  ///
  /// This implementation wraps that algorithm with a Tikhonov-type
  /// regularization approach. Specifically, this implementation repeatedly
  /// attempts to solve the LCP:<pre>
  /// (M + Iα)z + q = w
  /// z ≥ 0
  /// w ≥ 0
  /// zᵀw = 0
  /// </pre>
  /// where I is the identity matrix and α ≪ 1, using geometrically increasing
  /// values of α, until the LCP is solved. See SolveLcpFastRegularized() for
  /// description of the regularization process and the function parameters,
  /// which are identical. See SolveLcpLemke() for a description of Lemke's
  /// Algorithm. See SolveLcpFastRegularized() for a description of all
  /// calling parameters other than @p z, which apply equally well to this
  /// function.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use z's value as a starting
  ///                solution. **This warmstarting is generally not
  ///                recommended**: it has a predisposition to lead to a failing
  ///                pivoting sequence.
  ///
  /// @sa SolveLcpFastRegularized()
  /// @sa SolveLcpLemke()
  ///
  /// * [Cottle 1992]      R. Cottle, J.-S. Pang, and R. Stone. The Linear
  ///                      Complementarity Problem. Academic Press, 1992.
  bool SolveLcpLemkeRegularized(const MatrixX<T>& M,
                                const VectorX<T>& q, VectorX<T>* z,
                                int* num_pivots,
                                int min_exp = -20, int step_exp = 1,
                                int max_exp = 1, const T& piv_tol = T(-1),
                                const T& zero_tol = T(-1)) const;

  bool available() const override { return true; }

  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

 private:
  struct LemkeIndexSets {
    std::vector<int> alpha, alpha_prime;
    std::vector<int> bar_alpha, bar_alpha_prime;
    std::vector<int> beta, beta_prime;
    std::vector<int> bar_beta, bar_beta_prime;
  };

  // A structure for holding a linear complementarity problem variable.
  struct LCPVariable {
    bool z{true};        // Is this a z variable or a w variable?
    int index{-1};       // Index of the variable in the problem, 0...n. Index 0
                         // with z=true corresponds to the artificial variable.

    bool operator<(const LCPVariable& v) const {
      if (!z && v.z) {
        return true;
      } else {
        if (z && !v.z) {
          return false;
        }

        // Both variables are z or both are w.
        return index < v.index;
      }
    }
  };

  static bool IsEachUnique(const std::vector<LCPVariable>& vars);
  void LemkePivot(const MatrixX<T>& M, const VectorX<T>& q,
      int driving_index, VectorX<T>* M_bar_col, VectorX<T>* q_bar) const;
  void ConstructLemkeSolution(const MatrixX<T>& M, const VectorX<T>& q,
      int artificial_index, VectorX<T>* z) const;
  static int FindComplementIndex(
      const LCPVariable& query,
      const std::vector<LCPVariable>& indep_variables);
  void DetermineIndexSets() const;

  typedef std::vector<LCPVariable> LCPVariableVector;

  // Structure for mapping a vector of independent variables to a selection
  // index.
  class LCPVariableVectorComparator {
   public:
    // This does a lexicographic comparison, using z variables first and then
    // w variables.
    bool operator()(
        const LCPVariableVector& v1, const LCPVariableVector& v2) const {
      DRAKE_DEMAND(v1.size() == v2.size());

      // Copy the vectors.
      sorted1_ = v1;
      sorted2_ = v2;

      // Determine the variables in sorted order because we want to consider
      // all permutations of a set of variables as the same.
      std::sort(sorted1_.begin(), sorted1_.end());
      std::sort(sorted2_.begin(), sorted2_.end());

      // Now do a lexicographic comparison.
      for (int i = 0; i < static_cast<int>(v1.size()); ++i) {
        if (v1[i] < v2[i]) {
          return true;
        } else {
          if (v2[i] < v1[i])
            return false;
        }
      }

      // If still here, they're equal.
      return false;
    }

   private:
    // Two temporary vectors for storing sorted versions of vectors.
    mutable LCPVariableVector sorted1_, sorted2_;
  };

  // Maps the independent variables to the selection taken to prevent cycling.
  mutable std::map<LCPVariableVector, int, LCPVariableVectorComparator>
      selections_;

  // These temporary matrices and vectors are members to facilitate minimizing
  // memory allocations/deallocations. Changing their value between invocations
  // of the LCP solver will not change the resulting computation.
  mutable MatrixX<T> M_alpha_beta_, M_prime_alpha_beta_;
  mutable VectorX<T> q_alpha_, q_bar_alpha_, q_prime_beta_prime_,
      q_prime_bar_alpha_prime_, e_, M_prime_driving_beta_prime_,
      M_prime_driving_bar_alpha_prime_, g_alpha_, g_bar_alpha_;

  // The index sets for the Lemke Algorithm and is a member variable to
  // permit warmstarting. Changing the index set between invocations of the LCP
  // solver will not change the resulting computation.
  mutable LemkeIndexSets index_sets_;

  // The partitions of independent and dependent variables (denoted z' and w',
  // respectively, in [Dai and Drumwright 2018]). These have been made member
  // variables to permit warmstarting. Changing these sets between invocations
  // of the LCP solver will not change the resulting computation.
  mutable std::vector<LCPVariable> indep_variables_, dep_variables_;
};

}  // end namespace solvers
}  // end namespace drake
