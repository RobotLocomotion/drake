#include "drake/solvers/unrevised_lemke_solver.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

using drake::log;

// Note: this code makes extensive use of the following document:
// [1] Dai, H. and Drumwright, E. Computing the Principal Pivoting Transform
//     for Solving Linear Complementarity Problems with Lemke's Algorithm.
//     (2018, located in doc/pivot_column.pdf).

namespace drake {
namespace solvers {

namespace {

// A linear system solver that accommodates the inability of the LU
// factorization to be AutoDiff'd (true in Eigen 3, at least). For double types,
// the faster LU factorization and solve is used. For other types, QR
// factorization and solve is used.
template <class T>
class LinearSolver {
 public:
  explicit LinearSolver(const MatrixX<T>& m);

  VectorX<T> Solve(const VectorX<T>& v) const;
  MatrixX<T> Solve(const MatrixX<T>& v) const;

 private:
  Eigen::ColPivHouseholderQR<MatrixX<T>> qr_;
  Eigen::PartialPivLU<MatrixX<double>> lu_;
};

template <class T>
LinearSolver<T>::LinearSolver(const MatrixX<T>& M) {
  if (M.rows() > 0)
    qr_ = Eigen::ColPivHouseholderQR<MatrixX<T>>(M);
}

template <>
LinearSolver<double>::LinearSolver(
    const MatrixX<double>& M) {
  if (M.rows() > 0)
    lu_ = Eigen::PartialPivLU<MatrixX<double>>(M);
}

template <class T>
VectorX<T> LinearSolver<T>::Solve(
    const VectorX<T>& v) const {
  if (v.rows() == 0) {
    DRAKE_DEMAND(qr_.rows() == 0);
    return VectorX<T>(0);
  }
  return qr_.solve(v);
}

template <class T>
MatrixX<T> LinearSolver<T>::Solve(
    const MatrixX<T>& m) const {
  if (m.rows() == 0) {
    DRAKE_DEMAND(qr_.rows() == 0);
    return MatrixX<T>(0, m.cols());
  }
  return qr_.solve(m);
}

template <>
VectorX<double> LinearSolver<double>::Solve(
    const VectorX<double>& v) const {
  if (v.rows() == 0) {
    DRAKE_DEMAND(lu_.rows() == 0);
    return VectorX<double>(0);
  }
  return lu_.solve(v);
}

template <>
MatrixX<double> LinearSolver<double>::Solve(
    const MatrixX<double>& m) const {
  if (m.rows() == 0) {
    DRAKE_DEMAND(lu_.rows() == 0);
    return MatrixX<double>(0, m.cols());
  }
  return lu_.solve(m);
}
}  // anonymous namespace

template <>
SolutionResult
    UnrevisedLemkeSolver<Eigen::AutoDiffScalar<drake::Vector1d>>::Solve(
    MathematicalProgram&) const {
  DRAKE_ABORT_MSG("UnrevisedLemkeSolver cannot yet be used in a "
                  "MathematicalProgram while templatized as an AutoDiff");
  return SolutionResult::kUnknownError;
}

template <>
SolutionResult
    UnrevisedLemkeSolver<Eigen::AutoDiffScalar<Eigen::VectorXd>>::Solve(
    MathematicalProgram&) const {
  DRAKE_ABORT_MSG("UnrevisedLemkeSolver cannot yet be used in a "
                  "MathematicalProgram while templatized as an AutoDiff");
  return SolutionResult::kUnknownError;
}

template <typename T>
// NOLINTNEXTLINE(*)  Don't lint old, non-style-compliant code below.
SolutionResult UnrevisedLemkeSolver<T>::Solve(MathematicalProgram& prog) const {
  // This solver imposes restrictions that its problem:
  //
  // (1) Contains only linear complementarity constraints,
  // (2) Has no element of any decision variable appear in more than one
  //     constraint, and
  // (3) Has every element of every decision variable in a constraint.
  //
  // Restriction 1 could reasonably be relaxed by reformulating other
  // constraint types that can be expressed as LCPs (eg, convex QLPs),
  // although this would also entail adding an output stage to convert
  // the LCP results back to the desired form.  See eg. @RussTedrake on
  // how to convert a linear equality constraint of n elements to an
  // LCP of 2n elements.
  //
  // There is no obvious way to relax restriction 2.
  //
  // Restriction 3 could reasonably be relaxed to simply let unbound
  // variables sit at 0.

  DRAKE_ASSERT(prog.generic_constraints().empty());
  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.GetAllLinearConstraints().empty());
  DRAKE_ASSERT(prog.bounding_box_constraints().empty());

  const auto& bindings = prog.linear_complementarity_constraints();

  // Assert that the available LCPs cover the program and no two LCPs cover
  // the same variable.
  for (int i = 0; i < static_cast<int>(prog.num_vars()); ++i) {
    int coverings = 0;
    for (const auto& binding : bindings) {
      if (binding.ContainsVariable(prog.decision_variable(i))) {
        coverings++;
      }
    }
    DRAKE_ASSERT(coverings == 1);
  }

  // Solve each individual LCP, writing the result back to the decision
  // variables through the binding and returning true iff all LCPs are
  // feasible.
  //
  // If any is infeasible, returns false and does not alter the decision
  // variables.
  //

  // We don't actually indicate different results.
  SolverResult solver_result(UnrevisedLemkeSolverId::id());

  // Create a dummy variable for the number of pivots used.
  int num_pivots = 0;

  Eigen::VectorXd x_sol(prog.num_vars());
  for (const auto& binding : bindings) {
    Eigen::VectorXd constraint_solution(binding.GetNumElements());
    const std::shared_ptr<LinearComplementarityConstraint> constraint =
        binding.evaluator();
    bool solved = SolveLcpLemke(
        constraint->M(), constraint->q(), &constraint_solution, &num_pivots);
    if (!solved) {
      prog.SetSolverResult(solver_result);
      return SolutionResult::kUnknownError;
    }
    for (int i = 0; i < binding.evaluator()->num_vars(); ++i) {
      const int variable_index =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      x_sol(variable_index) = constraint_solution(i);
    }
    solver_result.set_optimal_cost(0.0);
  }
  return SolutionResult::kSolutionFound;
}

// Utility function for copying part of a matrix (designated by the indices
// in rows and cols) from `in`, augmented with a single column of "ones" (i.e.,
// the "covering vector"), to a target matrix, `out`.
template <class T>
void UnrevisedLemkeSolver<T>::SelectSubColumnWithCovering(
    const MatrixX<T>& in,
    const std::vector<int>& rows,
    int column, VectorX<T>* out) {
  DRAKE_ASSERT(ValidateIndices(rows, in.rows()));

  const int num_rows = rows.size();
  out->resize(num_rows);

  // Look for the covering vector first.
  if (column == in.cols()) {
    out->setOnes();
    return;
  }

  DRAKE_DEMAND(column < in.cols() && column >= 0);
  const auto in_column = in.col(column);
  for (int i = 0; i < num_rows; i++) {
    DRAKE_ASSERT(rows[i] < in_column.size());
    (*out)[i] = in_column[rows[i]];
  }
}

// Utility function for copying selected rows of the column vector `in` to
// the vector `out`, which is resized as necessary.
template <class T>
void UnrevisedLemkeSolver<T>::SelectSubVector(const VectorX<T>& in,
    const std::vector<int>& rows, VectorX<T>* out) {
  DRAKE_ASSERT(ValidateIndices(rows, in.size()));

  const int num_rows = rows.size();
  out->resize(num_rows);
  for (int i = 0; i < num_rows; i++) {
    DRAKE_ASSERT(rows[i] < in.rows());
    (*out)(i) = in(rows[i]);
  }
}

// Utility function for copying the vector `v_sub` to selected rows of the
// column vector `v`. Asserts that the size of `v_sub` is equal to the size of
// `indices`.
template <class T>
void UnrevisedLemkeSolver<T>::SetSubVector(const VectorX<T>& v_sub,
    const std::vector<int>& indices, VectorX<T>* v) {
  DRAKE_DEMAND(indices.size() == static_cast<size_t>(v_sub.size()));
  DRAKE_ASSERT(ValidateIndices(indices, v->size()));
  for (size_t i = 0; i < indices.size(); ++i)
    (*v)[indices[i]] = v_sub[i];
}

// Function for checking whether a set of indices that specify a view into
// a vector is valid. Returns `true` if row_indices are unique and each element
// lies in [0, vector_size-1] and `false` otherwise.
template <class T>
bool UnrevisedLemkeSolver<T>::ValidateIndices(
    const std::vector<int>& row_indices, int vector_size) {
  // Don't check anything for empty vectors.
  if (row_indices.empty())
    return true;

  // Sort the vector first.
  std::vector<int> sorted_row_indices = row_indices;
  std::sort(sorted_row_indices.begin(), sorted_row_indices.end());

  // Validate the maximum and minimum elements.
  if (sorted_row_indices.back() >= vector_size)
    return false;
  if (sorted_row_indices.front() < 0)
    return false;

  // Make sure that the vector is unique.
  return std::unique(sorted_row_indices.begin(), sorted_row_indices.end()) ==
         sorted_row_indices.end();
}

// Function for checking whether a set of indices that specify a view into
// a matrix is valid. Returns `true` if each element of row_indices is unique
// lies in [0, num_rows-1] and if each element of col_indices is unique and
// lies in [0, num_cols-1]. Returns `false` otherwise.
template <class T>
bool UnrevisedLemkeSolver<T>::ValidateIndices(
    const std::vector<int>& row_indices,
    const std::vector<int>& col_indices, int num_rows, int num_cols) {
  return ValidateIndices(row_indices, num_rows) &&
         ValidateIndices(col_indices, num_cols);
}

// Utility function for copying part of a matrix- designated by the indices
// in rows and cols from `in` and augmented with a single column of "ones"
// (i.e., the "covering vector")- to a target matrix, `out`.
template <class T>
void UnrevisedLemkeSolver<T>::SelectSubMatrixWithCovering(
    const MatrixX<T>& in,
    const std::vector<int>& rows,
    const std::vector<int>& cols, MatrixX<T>* out) {
  const int num_rows = rows.size();
  const int num_cols = cols.size();
  DRAKE_ASSERT(ValidateIndices(rows, cols, in.rows(), in.cols() + 1));
  out->resize(num_rows, num_cols);

  for (int i = 0; i < num_rows; i++) {
    const auto row_in = in.row(rows[i]);

    // `row_out` is a "view" into `out`: any modifications to row_out are
    // reflected in `out`.
    auto row_out = out->row(i);
    for (int j = 0; j < num_cols; j++) {
      if (cols[j] < in.cols()) {
        DRAKE_ASSERT(cols[j] >= 0);
        row_out(j) = row_in(cols[j]);
      } else {
        DRAKE_ASSERT(cols[j] == in.cols());
        row_out(j) = 1.0;
      }
    }
  }
}

// Determines the various index sets defined in Section 1.1 of [1].
template <class T>
void UnrevisedLemkeSolver<T>::DetermineIndexSets() const {
  // Helper for determining index sets.
  auto DetermineIndexSetsHelper = [this](
      const std::vector<LCPVariable>& variables, bool is_z,
      std::vector<int>* variable_set,
      std::vector<int>* variable_set_prime) {
    variable_and_array_indices_.clear();
    for (int i = 0; i < static_cast<int>(variables.size()); ++i) {
      if (variables[i].z() == is_z)
        variable_and_array_indices_.emplace_back(variables[i].index(), i);
    }
    std::sort(variable_and_array_indices_.begin(),
              variable_and_array_indices_.end());

    // Construct the set and the primed set.
    for (const auto& variable_and_array_index_pair :
        variable_and_array_indices_) {
      variable_set->push_back(variable_and_array_index_pair.first);
      variable_set_prime->push_back(variable_and_array_index_pair.second);
    }
  };

  // Clear all sets.
  index_sets_.alpha.clear();
  index_sets_.alpha_bar.clear();
  index_sets_.alpha_prime.clear();
  index_sets_.alpha_bar_prime.clear();
  index_sets_.beta.clear();
  index_sets_.beta_bar.clear();
  index_sets_.beta_prime.clear();
  index_sets_.beta_bar_prime.clear();

  DetermineIndexSetsHelper(indep_variables_, false,
      &index_sets_.alpha, &index_sets_.alpha_prime);
  DetermineIndexSetsHelper(dep_variables_, false,
      &index_sets_.alpha_bar, &index_sets_.alpha_bar_prime);
  DetermineIndexSetsHelper(dep_variables_, true,
                           &index_sets_.beta, &index_sets_.beta_prime);
  DetermineIndexSetsHelper(indep_variables_, true,
                           &index_sets_.beta_bar, &index_sets_.beta_bar_prime);
}

// Verifies that each element of the pivoting set is unique. This is an
// expensive operation and should only be executed in Debug mode.
template <class T>
bool UnrevisedLemkeSolver<T>::IsEachUnique(
    const std::vector<LCPVariable>& vars) {
  // Copy the set.
  std::vector<LCPVariable> vars_copy = vars;
  std::sort(vars_copy.begin(), vars_copy.end());
  return (std::unique(vars_copy.begin(), vars_copy.end()) == vars_copy.end());
}

// Performs the pivoting operation, which is described in [1]. `M_prime_col`
// can be null, if the updated column of M' (pivoted version of M)
// is not needed and the driving_index corresponds to the artificial variable.
template <typename T>
bool UnrevisedLemkeSolver<T>::LemkePivot(
    const MatrixX<T>& M,
    const VectorX<T>& q,
    int driving_index,
    T zero_tol,
    VectorX<T>* M_prime_col,
    VectorX<T>* q_prime) const {
  DRAKE_DEMAND(q_prime);

  const int kArtificial = M.rows();  // Artificial variable index.
  DRAKE_DEMAND(driving_index >= 0 && driving_index <= kArtificial);

  // Verify that each member in the independent and dependent sets is unique.
  DRAKE_ASSERT(IsEachUnique(indep_variables_));
  DRAKE_ASSERT(IsEachUnique(dep_variables_));

  // If the driving index does not correspond to the artificial variable,
  // M_prime_col must be non-null.
  if (!indep_variables_[driving_index].z() ||
      indep_variables_[driving_index].index() != kArtificial) {
    DRAKE_DEMAND(M_prime_col);
  }

  // Determine the sets.
  DetermineIndexSets();

  // Note: It is feasible to do a low-rank update to the factorization below,
  // since alpha and beta should change by no more than a single index between
  // consecutive pivots. Eigen only supports low-rank updates to Cholesky
  // factorizations at the moment, however.

  // Compute matrix and vector views.
  SelectSubMatrixWithCovering(M, index_sets_.alpha, index_sets_.beta,
                              &M_alpha_beta_);
  SelectSubMatrixWithCovering(M, index_sets_.alpha_bar, index_sets_.beta,
                  &M_alpha_bar_beta_);
  SelectSubVector(q, index_sets_.alpha, &q_alpha_);
  SelectSubVector(q, index_sets_.alpha_bar, &q_alpha_bar_);

  // Equation (2) from [1].
  // Note: this equation, and those below, must be kept up-to-date with [1].
  LinearSolver<T> fMab(M_alpha_beta_);  // Factorized M_alpha_beta_.
  q_prime_beta_prime_ = -fMab.Solve(q_alpha_);

  // Check whether the solution is sufficiently close. We need to do this
  // because partial pivoting LU does not estimate rank (and, from prior
  // experience in solving LCPs), loss of rank need not lead to errors in
  // solving the LCP. We assume that if the factorization is good enough to
  // solve this linear system, it's good enough to solve the subsequent
  // linear system (below).
  if ((M_alpha_beta_ * q_prime_beta_prime_ + q_alpha_).norm() > zero_tol)
    return false;

  // Equation (3) from [1].
  q_prime_alpha_bar_prime_ = M_alpha_bar_beta_ * q_prime_beta_prime_ +
      q_alpha_bar_;

  // Set the components of q'.
  SetSubVector(q_prime_beta_prime_, index_sets_.beta_prime, q_prime);
  SetSubVector(q_prime_alpha_bar_prime_, index_sets_.alpha_bar_prime, q_prime);

  DRAKE_SPDLOG_DEBUG(log(), "q': {}", q_prime->transpose());

  // If it is not necessary to compute the column of M, quit now.
  if (!M_prime_col)
    return true;

  // Examine the driving variable.
  if (!indep_variables_[driving_index].z()) {
    DRAKE_SPDLOG_DEBUG(log(), "Driving case #1: driving variable from w");
    // Case from Section 2.2.1.
    // Determine gamma by determining the position of the driving variable
    // in INDEPENDENT W (as defined in [1]).
    const int n = static_cast<int>(indep_variables_.size());
    int gamma = 0;
    for (int i = 0; i < n; ++i) {
      if (!indep_variables_[i].z()) {
        if (indep_variables_[i].index() <
            indep_variables_[driving_index].index()) {
          ++gamma;
        }
      }
    }


    // From Equation (4) in [1].
    DRAKE_ASSERT(index_sets_.alpha[gamma] ==
        indep_variables_[driving_index].index());

    // Set the unit vector.
    e_.setZero(index_sets_.beta.size());
    e_[gamma] = 1.0;

    // Equation (5).
    M_prime_driving_beta_prime_ = fMab.Solve(e_);

    // Equation (6).
    M_prime_driving_alpha_bar_prime_ = M_alpha_bar_beta_ *
        M_prime_driving_beta_prime_;
  } else {
    DRAKE_SPDLOG_DEBUG(log(), "Driving case #2: driving variable from z");

    // Case from Section 2.2.2 of [1].
    // Determine zeta.
    const int zeta = indep_variables_[driving_index].index();

    // Compute g_alpha and g_alpha_bar.
    SelectSubColumnWithCovering(M, index_sets_.alpha, zeta, &g_alpha_);
    SelectSubColumnWithCovering(M, index_sets_.alpha_bar, zeta, &g_alpha_bar_);

    // Equation (7).
    M_prime_driving_beta_prime_ = -fMab.Solve(g_alpha_);

    // Equation (8).
    M_prime_driving_alpha_bar_prime_ = g_alpha_bar_ +
        M_alpha_bar_beta_ * M_prime_driving_beta_prime_;
  }

  SetSubVector(M_prime_driving_beta_prime_, index_sets_.beta_prime,
               M_prime_col);
  SetSubVector(M_prime_driving_alpha_bar_prime_, index_sets_.alpha_bar_prime,
               M_prime_col);

  DRAKE_SPDLOG_DEBUG(log(), "M' (driving): {}", M_prime_col->transpose());
  return true;
}

// Method for finding the index of the complement of an LCP variable in
// a tuple (strictly speaking, an unsorted vector) of independent variables.
// Aborts if the index is not found in the set or the variable is the articial
// variable (it never should be).
template <class T>
int UnrevisedLemkeSolver<T>::FindComplementIndex(
    const LCPVariable& query,
    const std::vector<LCPVariable>& indep_variables) const {
  // Verify that the query is not the artificial variable.
  const int kArtificial = static_cast<int>(indep_variables.size() - 1);
  DRAKE_DEMAND(!(query.z() && query.index() == kArtificial));

  const auto iter = indep_variables_indices_.find(query.Complement());
  DRAKE_DEMAND(iter != indep_variables_indices_.end());
  return iter->second;
}

// Computes the solution using the current index sets. `z` must simply be
// non-null; it will be resized as necessary. Returns `true` if able to
// construct the solution and `false` if unable to find the solution to a
// necessary system of linear equations.
template <class T>
bool UnrevisedLemkeSolver<T>::ConstructLemkeSolution(
    const MatrixX<T>& M,
    const VectorX<T>& q,
    int artificial_index,
    T zero_tol,
    VectorX<T>* z) const {
  DRAKE_DEMAND(z);
  const int n = q.rows();

  // Compute the solution.
  VectorX<T> q_prime(n);
  if (!LemkePivot(M, q, artificial_index, zero_tol, nullptr, &q_prime))
    return false;

  z->setZero(n);
  for (int i = 0; i < static_cast<int>(dep_variables_.size()); ++i) {
    if (dep_variables_[i].z())
      (*z)[dep_variables_[i].index()] = q_prime[i];
  }
  return true;
}

// Computes the blocking index using the minimum ratio test. Returns `true`
// if successful, `false` if not (due to, e.g., the driving variable being
// "unblocked" or a cycle being detected). If `false`, `blocking_index` will
// set to -1 on return.
template <typename T>
bool UnrevisedLemkeSolver<T>::FindBlockingIndex(
    const T& zero_tol, const VectorX<T>& matrix_col, const VectorX<T>& ratios,
    int* blocking_index) const {
  DRAKE_DEMAND(blocking_index);
  DRAKE_DEMAND(ratios.size() == matrix_col.size());
  DRAKE_DEMAND(zero_tol > 0);

  T min_ratio = std::numeric_limits<double>::infinity();
  *blocking_index = -1;
  for (int i = 0; i < matrix_col.size(); ++i) {
    if (matrix_col[i] < -zero_tol) {
      DRAKE_SPDLOG_DEBUG(log(), "Ratio for index {}: {}", i, ratios[i]);
      if (ratios[i] < min_ratio) {
        min_ratio = ratios[i];
        *blocking_index = i;
      }
    }
  }

  if (*blocking_index < 0) {
    SPDLOG_DEBUG(log(), "driving variable is unblocked- algorithm failed");
    return false;
  }

  // Determine all variables within the zero tolerance of the minimum ratio.
  std::vector<int> blocking_indices;
  for (int i = 0; i < matrix_col.size(); ++i) {
    if (matrix_col[i] < -zero_tol) {
      DRAKE_SPDLOG_DEBUG(log(), "Ratio for index {}: {}", i, ratios[i]);
      if (ratios[i] < min_ratio + zero_tol)
        blocking_indices.push_back(i);
    }
  }

  // If there are multiple blocking variables, replace the blocking index with
  // the cycling selection.
  if (blocking_indices.size() > 1) {
    auto& index = selections_[indep_variables_];

    // Verify that we have not run out of indices to select, which means that
    // cycling would be occurring, in spite of cycling prevention.
    if (index >= static_cast<int>(blocking_indices.size())) {
      DRAKE_SPDLOG_DEBUG(log(), "Cycling detected- indicating failure.");
      *blocking_index = -1;
      return false;
    }
    *blocking_index = blocking_indices[index];
    ++index;
  }

  return true;
}

template <typename T>
bool UnrevisedLemkeSolver<T>::IsSolution(
    const MatrixX<T>& M, const VectorX<T>& q, const VectorX<T>& z,
    T zero_tol) {
  using std::abs;

  const T mod_zero_tol = (zero_tol > 0) ? zero_tol : ComputeZeroTolerance(M);

  // Find the minima of z and w.
  const T min_z = z.minCoeff();
  const auto w = M * z + q;
  const T min_w = w.minCoeff();

  // Compute the dot product of z and w.
  const T dot = w.dot(z);
  const int n = q.size();
  return (min_z > -mod_zero_tol && min_w > -mod_zero_tol &&
          abs(dot) < 10*n*mod_zero_tol);
}

// Note: maintainers should read Section 4.4 - 4.4.5 of [Cottle, 1992] to
// understand Lemke's Algorithm (and this function).
template <typename T>
bool UnrevisedLemkeSolver<T>::SolveLcpLemke(const MatrixX<T>& M,
                                     const VectorX<T>& q, VectorX<T>* z,
                                     int* num_pivots,
                                     const T& zero_tol) const {
  using std::max;
  using std::abs;
  DRAKE_DEMAND(num_pivots);

  DRAKE_SPDLOG_DEBUG(log(),
      "UnrevisedLemkeSolver::SolveLcpLemke() entered, M: {}, "
      "q: {}, ", M, q.transpose());

  const int n = q.size();
  const int max_pivots = 50 * n;  // O(n) pivots expected for solvable problems.

  if (M.rows() != n || M.cols() != n)
    throw std::logic_error("M's dimensions do not match that of q.");

  // Update the pivots.
  *num_pivots = 0;

  // Look for immediate exit.
  if (n == 0) {
    DRAKE_SPDLOG_DEBUG(log(), "-- LCP is zero dimensional");
    z->resize(0);
    return true;
  }

  // Denote the index of the artificial variable (i.e., the variable denoted
  // z₀ in [Cottle 1992], p. 266). Because z₀ is prone to being confused with
  // the first dimension of z in 0-indexed languages like C++, we refer to the
  // artificial variable as zₙ in this implementation (it is denoted zₙ₊₁ in
  // [1], as that document uses the 1-indexing prevalent in algorithmic
  // descriptions).
  const int kArtificial = n;

  // Compute a sensible value for zero tolerance if none is given.
  T mod_zero_tol = zero_tol;
  if (mod_zero_tol <= 0)
    mod_zero_tol = ComputeZeroTolerance(M);

  // Checks to see whether the trivial solution z = 0 to the LCP w = Mz + q
  // solves the LCP.
  if (q.minCoeff() > -mod_zero_tol) {
    z->setZero(q.size());
    SPDLOG_DEBUG(log(), " -- trivial solution found");
    SPDLOG_DEBUG(log(), "UnrevisedLemkeSolver::SolveLcpLemke() exited");
    return true;
  }

  // Clear the cycling selections.
  selections_.clear();

  // If 'n' is identical to the size of the last problem solved, try using the
  // indices from the last problem solved.
  if (static_cast<size_t>(n) == dep_variables_.size()) {
    // Verify that the last call found a solution (indicated by the presence
    // of zn in the independent set).
    int zn_index = -1;
    for (int i = 0;
         i < static_cast<int>(indep_variables_.size()) && zn_index < 0; ++i) {
      if (indep_variables_[i].z() && indep_variables_[i].index() == kArtificial)
        zn_index = i;
    }

    if (zn_index >= 0) {
      // Compute the candidate solution.
      if (ConstructLemkeSolution(M, q, zn_index, mod_zero_tol, z) &&
          IsSolution(M, q, *z, mod_zero_tol)) {
        // If the solution is good, return now, indicating only one pivot
        // (in the solution construction) was performed.
        ++(*num_pivots);
        return true;
      } else {
        DRAKE_SPDLOG_DEBUG(log(),
            "Failed to solve linear system implied by last solution");
      }
    }
  }

  // Set the LCP variables. Start with all z variables independent and all w
  // variables dependent.
  indep_variables_.resize(n+1);
  dep_variables_.resize(n);
  for (int i = 0; i < n; ++i) {
    dep_variables_[i] = LCPVariable(false, i);
    indep_variables_[i] = LCPVariable(true, i);
  }
  // z needs one more variable (the artificial variable), whose index we
  // denote as n to keep it from corresponding to any actual vector index.
  indep_variables_[n] = LCPVariable(true, n);

  // Compute zn*, the smallest value of the artificial variable zn for which
  // w = q + zn >= 0. Let blocking denote a component of w that equals
  // zero when zn = zn*.
  int blocking_index = -1;
  DRAKE_DEMAND(FindBlockingIndex(mod_zero_tol, q, q, &blocking_index));

  // Pivot blocking, artificial. Note that we rely upon the dependent variables
  // being ordered sequentially in both arrays.
  LCPVariable blocking = dep_variables_[blocking_index];
  int driving_index = blocking.index();
  std::swap(dep_variables_[blocking_index], indep_variables_[kArtificial]);
  DRAKE_SPDLOG_DEBUG(log(), "First blocking variable {}{}",
                     ((blocking.z()) ? "z" : "w"), blocking.index());
  DRAKE_SPDLOG_DEBUG(log(), "First driving variable (artificial)");

  // Initialize the independent variable indices. We do this after the initial
  // variable swap for simplicity.
  for (int i = 0; i < static_cast<int>(indep_variables_.size()); ++i)
    indep_variables_indices_[indep_variables_[i]] = i;

  // Output the independent and dependent variable tuples.
  #ifdef SPDLOG_DEBUG_ON
  auto to_string = [](const std::vector<LCPVariable>& vars) -> std::string {
    std::ostringstream oss;
    for (int i = 0; i < static_cast<int>(vars.size()); ++i)
      oss << ((vars[i].z()) ? "z" : "w") << vars[i].index() << " ";
    return oss.str();
  };
  DRAKE_SPDLOG_DEBUG(log(), "Independent set variables: {}",
      to_string(indep_variables_));
  DRAKE_SPDLOG_DEBUG(log(), "Dependent set variables: {}",
      to_string(dep_variables_));
  #endif

  // Pivot up to the maximum number of times.
  VectorX<T> q_prime(n), M_prime_col(n);
  while (++(*num_pivots) < max_pivots) {
    DRAKE_SPDLOG_DEBUG(log(), "New driving variable {}{}",
                       ((indep_variables_[driving_index].z()) ? "z" : "w"),
                       indep_variables_[driving_index].index());

    // Compute the permuted q and driving column of the permuted M matrix.
    if (!LemkePivot(
        M, q, driving_index, mod_zero_tol, &M_prime_col, &q_prime)) {
      DRAKE_SPDLOG_DEBUG(log(), "Linear system solve failed.");
      z->setZero(n);
      return false;
    }

    // Find the blocking variable.
    if (!FindBlockingIndex(
        mod_zero_tol, M_prime_col,
        -(q_prime.array() / M_prime_col.array()).matrix(), &blocking_index)) {
      z->setZero(n);
      return false;
    }
    blocking = dep_variables_[blocking_index];
    DRAKE_SPDLOG_DEBUG(log(), "Blocking variable {}{}",
                       ((blocking.z()) ? "z" : "w"), blocking.index());

    // See whether the artificial variable blocks the driving variable.
    if (blocking.index() == kArtificial) {
      DRAKE_DEMAND(blocking.z());

      // Pivot zn with the driving variable.
      std::swap(dep_variables_[blocking_index],
                indep_variables_[driving_index]);

      // Compute the permuted q, and convert it into a solution.
      if (ConstructLemkeSolution(M, q, driving_index, mod_zero_tol, z)) {
        if (IsSolution(M, q, *z))
          return true;

        DRAKE_SPDLOG_DEBUG(log(),
            "Solution not computed to requested tolerance");
        z->setZero(n);
        return false;
      }

      // Otherwise, indicate failure.
      DRAKE_SPDLOG_DEBUG(log(),
          "Linear system solver failed to construct Lemke solution");
      z->setZero(n);
      return false;
    }

    // Pivot the blocking variable and the driving variable.
    std::swap(dep_variables_[blocking_index],
              indep_variables_[driving_index]);

    // Update the index map.
    auto indep_variables_indices_iter = indep_variables_indices_.find(
        dep_variables_[blocking_index]);
    indep_variables_indices_.erase(indep_variables_indices_iter);
    indep_variables_indices_[indep_variables_[driving_index]] =
        driving_index;

    // Make the driving variable the complement of the blocking variable.
    driving_index = FindComplementIndex(blocking, indep_variables_);

    DRAKE_SPDLOG_DEBUG(log(), "Independent set variables: {}",
        to_string(indep_variables_));
    DRAKE_SPDLOG_DEBUG(log(), "Dependent set variables: {}",
        to_string(dep_variables_));
  }

  // If here, the maximum number of pivots has been exceeded.
  z->setZero(n);
  DRAKE_SPDLOG_DEBUG(log(), "Maximum number of pivots exceeded");
  return false;
}

template <typename T>
SolverId UnrevisedLemkeSolver<T>::solver_id() const {
  return UnrevisedLemkeSolverId::id();
}

SolverId UnrevisedLemkeSolverId::id() {
  static const never_destroyed<SolverId> singleton{"Unrevised Lemke"};
  return singleton.access();
}

}  // namespace solvers
}  // namespace drake

// Instantiate templates.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::solvers::UnrevisedLemkeSolver)
