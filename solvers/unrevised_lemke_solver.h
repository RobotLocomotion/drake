#pragma once

#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <utility>
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
  /// copositive matrices. The solver can be applied with occasional success to
  /// problems outside of its guaranteed matrix classes. Lemke's Algorithm is
  /// described in [Cottle 1992], Section 4.4.
  ///
  /// The solver will denote failure on return if it exceeds a problem-size
  /// dependent number of iterations.
  /// @param[in] M the LCP matrix.
  /// @param[in] q the LCP vector.
  /// @param[in,out] z the solution to the LCP on return (if the solver
  ///                succeeds). If the length of z is equal to the length of q,
  ///                the solver will attempt to use the basis from the last
  ///                solution. This strategy can prove exceptionally
  ///                fast if solutions differ little between successive calls.
  ///                If the solver fails (returns `false`),
  ///                `z` will be set to the zero vector on return.
  /// @param[out] num_pivots the number of pivots used, on return.
  /// @param[in] zero_tol The tolerance for testing against zero. If the
  ///            tolerance is negative (default) the solver will determine a
  ///            generally reasonable tolerance.
  /// @param[in] piv_tol The tolerance for testing against zero, specifically
  ///            used for the purpose of finding variables for pivoting. If the
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
  class LCPVariable {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LCPVariable)
    LCPVariable() {}
    LCPVariable(bool z, int index) : z_{z}, index_{index} {}

    bool z() const { return z_; }
    int index() const { return index_; }

    // Gets the complement of this variable.
    LCPVariable Complement() const {
      DRAKE_ASSERT(index_ >= 0);
      LCPVariable comp;
      comp.z_ = !z_;
      comp.index_ = index_;
      return comp;
    }

    // Compares two LCP variables for equality.
    bool operator==(const LCPVariable& v) const {
      DRAKE_ASSERT(index_ >= 0 && v.index_ >= 0);
      return (z_ == v.z_ && index_ == v.index_);
    }

    // Comparison operator for using LCPVariable as a key.
    bool operator<(const LCPVariable& v) const {
      DRAKE_ASSERT(index_ >= 0 && v.index_ >= 0);
      if (index_ < v.index_) {
        return true;
      } else {
        if (index_ > v.index_) {
          return false;
        } else {
          // If here, the indices are equal. We will arbitrarily order w before
          // z (alphabetical ordering).
          return (!z_ && v.z_);
        }
      }
    }

   private:
    bool z_{true};        // Is this a z variable or a w variable?
    int index_{-1};       // Index of the variable in the problem, 0...n. n
                          // indicates that the variable is artificial. -1
                          // indicates that the index is uninitialized.
  };

  static bool IsEachUnique(const std::vector<LCPVariable>& vars);
  bool LemkePivot(const MatrixX<T>& M, const VectorX<T>& q, int driving_index,
                  T zero_tol, VectorX<T>* M_bar_col, VectorX<T>* q_bar) const;
  bool ConstructLemkeSolution(const MatrixX<T>& M, const VectorX<T>& q,
      int artificial_index, T zero_tol, VectorX<T>* z) const;
  int FindComplementIndex(
      const LCPVariable& query,
      const std::vector<LCPVariable>& indep_variables) const;
  void DetermineIndexSets() const;
  void DetermineIndexSetsHelper(
      const std::vector<LCPVariable>& variables, bool z,
      std::vector<int>* variable_set,
      std::vector<int>* variable_set_prime) const;

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
        if (sorted1_[i] < sorted2_[i]) {
          return true;
        } else {
          if (sorted2_[i] < sorted1_[i])
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

  // Note: the mutable variables below are used in place of local variables both
  // to minimize heap allocations during the LCP solution process and to
  // facilitate warmstarting.

  // Temporary variable for determining index sets.
  mutable std::vector<std::pair<int, int>> variable_and_array_indices_;

  // Mapping from an LCP variable to the index of that variable in
  // indep_variables.
  mutable std::map<LCPVariable, int> indep_variables_indices_;

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

