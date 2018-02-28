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
#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

using drake::log;

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

// Checks to see whether the trivial solution z = 0 to the LCP w = Mz + q
// solves the LCP.
template <typename Scalar>
bool CheckLemkeTrivial(int n, const Scalar& zero_tol, const VectorX<Scalar>& q,
                       VectorX<Scalar>* z) {
  // see whether trivial solution exists
  if (q.minCoeff() > -zero_tol) {
    z->resize(n);
    z->fill(0);
    return true;
  }

  return false;
}

// Function for checking whether a set of indices that specify a view into
// a vector is valid.
bool ValidateIndices(const std::vector<int>& row_indices, int vector_size) {
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
// a matrix is valid.
bool ValidateIndices(
    const std::vector<int>& row_indices,
    const std::vector<int>& col_indices, int num_rows, int num_cols) {
  return ValidateIndices(row_indices, num_rows) &&
         ValidateIndices(col_indices, num_cols);
}

// Utility function for copying part of a matrix (designated by the indices
// in rows and cols) from `in`, augmented with a single column of "ones" (i.e.,
// the "covering vector"), to a target matrix, `out`. This template approach
// allows selecting parts of both sparse and dense matrices for input; only
// a dense matrix is returned. Since the matrix to be copied looks like this:
// | 1 in |
// selecting column 0, will copy a vector of ones and selecting column i, for
// i > 0, will copy column i - 1 of `in`.
template <typename Derived, typename T>
void SelectSubMatrixWithCovering(const Eigen::MatrixBase<Derived>& in,
                     const std::vector<int>& rows,
                     const std::vector<int>& cols, MatrixX<T>* out) {
  const int num_rows = rows.size();
  const int num_cols = cols.size();
  DRAKE_ASSERT(ValidateIndices(rows, cols, in.rows(), in.cols() + 1));
  out->resize(num_rows, num_cols);

  for (int i = 0; i < num_rows; i++) {
    const auto row_in = in.row(rows[i]);

    // row_out is a "view" into out: any modifications to row_out are reflected
    // in out.
    auto row_out = out->row(i);
    for (int j = 0; j < num_cols; j++) {
      if (cols[j] > 0) {
        row_out(j) = row_in(cols[j] - 1);
      } else {
        row_out(j) = 1.0;
      }
    }
  }
}

// Utility function for copying part of a matrix (designated by the indices
// in rows and cols) from `in`, augmented with a single column of "ones" (i.e.,
// the "covering vector"), to a target matrix, `out`. This template approach
// allows selecting parts of both sparse and dense matrices for input; only
// a dense matrix is returned. Since the matrix to be copied looks like this:
// | 1 in |
// selecting column 0, will return a vector of ones and selecting column i, for
// i > 0, will return column i - 1 of `in`.
template <typename Derived, typename T>
void SelectSubColumnWithCovering(const Eigen::MatrixBase<Derived>& in,
                                 const std::vector<int>& rows,
                                 int column, VectorX<T>* out) {
  const int num_rows = rows.size();
  DRAKE_ASSERT(ValidateIndices(rows, in.rows()));
  out->resize(num_rows);

  // Look for the covering vector first.
  if (column == 0) {
    out->setOnes();
    return;
  } else {
    const auto in_column = in.col(column - 1);
    for (int i = 0; i < num_rows; i++) {
      (*out)[i] = in_column[rows[i]];
    }
  }
}

// Selects the dimensions of `in` in `rows`.
template <typename T>
void SelectSubVector(const VectorX<T>& in,
                     const std::vector<int>& rows, VectorX<T>* out) {
  const int num_rows = rows.size();
  DRAKE_ASSERT(ValidateIndices(rows, in.rows()));
  out->resize(num_rows);
  for (int i = 0; i < num_rows; i++) {
    (*out)(i) = in(rows[i]);
  }
}

// Sets the dimensions of `v` specified in `indices` to the consecutive
// entries of `v_sub`.
template <typename T>
void SetSubVector(const std::vector<int>& rows, const VectorX<T>& v_sub,
                  VectorX<T>* v) {
  DRAKE_ASSERT(ValidateIndices(rows, v->rows()));
  DRAKE_DEMAND(rows.size() == static_cast<size_t>(v_sub.size()));
  for (size_t i = 0; i < rows.size(); ++i)
    (*v)[rows[i]] = v_sub[i];
}

}  // anonymous namespace

template <>
SolutionResult
    UnrevisedLemkeSolver<Eigen::AutoDiffScalar<drake::Vector1d>>::Solve(
// NOLINTNEXTLINE(*)  Don't lint old, non-style-compliant code below.
    MathematicalProgram&) const {
  DRAKE_ABORT_MSG("UnrevisedLemkeSolver cannot yet be used in a "
                  "MathematicalProgram while templatized as an AutoDiff");
  return SolutionResult::kUnknownError;
}

// TODO(edrumwri): Break the following code out into a special
// UnrevisedLemkeSolverMathematicalProgram class.
template <typename T>
// NOLINTNEXTLINE(*)  Don't lint old, non-style-compliant code below.
SolutionResult UnrevisedLemkeSolver<T>::Solve(MathematicalProgram& prog) const {
  // TODO(ggould-tri) This solver currently imposes restrictions that its
  // problem:
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
  // TODO(ggould-tri) This could also be solved by constructing a single large
  // square matrix and vector, and then copying the elements of the individual
  // Ms and qs into the appropriate places.  That would be equivalent to this
  // implementation but might perform better if the solver were to parallelize
  // internally.

  // We don't actually indicate different results.
  prog.SetSolverId(UnrevisedLemkeSolverId::id());

  for (const auto& binding : bindings) {
    Eigen::VectorXd constraint_solution(binding.GetNumElements());
    const std::shared_ptr<LinearComplementarityConstraint> constraint =
        binding.constraint();
    int unused;
    bool solved = SolveLcpLemkeRegularized(
        constraint->M(), constraint->q(), &constraint_solution, &unused);
    if (!solved) {
      return SolutionResult::kUnknownError;
    }
    prog.SetDecisionVariableValues(binding.variables(), constraint_solution);
    prog.SetOptimalCost(0.0);
  }
  return SolutionResult::kSolutionFound;
}

// Determines the various index sets.
template <class T>
void UnrevisedLemkeSolver<T>::DetermineIndexSets() const {
  // Clear all sets.
  index_sets_.alpha.clear();
  index_sets_.bar_alpha.clear();
  index_sets_.alpha_prime.clear();
  index_sets_.bar_alpha_prime.clear();
  index_sets_.beta.clear();
  index_sets_.bar_beta.clear();
  index_sets_.beta_prime.clear();
  index_sets_.bar_beta_prime.clear();

  // Vector that will be used to store the variable and array indices.
  std::vector<std::pair<int, int>> variable_and_array_indices;

  // Prepare to determine alpha and alpha_prime.
  variable_and_array_indices.clear();
  for (int i = 0; i < static_cast<int>(indep_variables_.size()); ++i) {
    if (!indep_variables_[i].z)
      variable_and_array_indices.emplace_back(indep_variables_[i].index - 1, i);
  }
  std::sort(variable_and_array_indices.begin(),
            variable_and_array_indices.end());

  // Set alpha and alpha_prime.
  for (const auto& variable_and_array_index_pair : variable_and_array_indices) {
    index_sets_.alpha.push_back(variable_and_array_index_pair.first);
    index_sets_.alpha_prime.push_back(variable_and_array_index_pair.second);
  }

  // Prepare to determine bar_alpha and bar_alpha_prime.
  variable_and_array_indices.clear();
  for (int i = 0; i < static_cast<int>(dep_variables_.size()); ++i) {
    if (!dep_variables_[i].z)
      variable_and_array_indices.emplace_back(dep_variables_[i].index - 1, i);
  }
  std::sort(variable_and_array_indices.begin(),
            variable_and_array_indices.end());

  // Set bar_alpha and bar_alpha_prime.
  for (const auto& variable_and_array_index_pair : variable_and_array_indices) {
    index_sets_.bar_alpha.push_back(variable_and_array_index_pair.first);
    index_sets_.bar_alpha_prime.push_back(variable_and_array_index_pair.second);
  }

  // Prepare to determine beta and beta_prime.
  variable_and_array_indices.clear();
  for (int i = 0; i < static_cast<int>(dep_variables_.size()); ++i) {
    if (dep_variables_[i].z) {
      variable_and_array_indices.emplace_back(dep_variables_[i].index, i);
    }
  }
  std::sort(variable_and_array_indices.begin(),
            variable_and_array_indices.end());

  for (const auto& variable_and_array_index_pair : variable_and_array_indices) {
    index_sets_.beta.push_back(variable_and_array_index_pair.first);
    index_sets_.beta_prime.push_back(variable_and_array_index_pair.second);
  }

  // Prepare to determine bar_beta and bar_beta_prime.
  variable_and_array_indices.clear();
  for (int i = 0; i < static_cast<int>(indep_variables_.size()); ++i) {
    if (indep_variables_[i].z) {
      variable_and_array_indices.emplace_back(dep_variables_[i].index, i);
    }
  }
  std::sort(variable_and_array_indices.begin(),
            variable_and_array_indices.end());

  for (const auto& variable_and_array_index_pair : variable_and_array_indices) {
    index_sets_.bar_beta.push_back(variable_and_array_index_pair.first);
    index_sets_.bar_beta_prime.push_back(variable_and_array_index_pair.second);
  }
}

// Verifies that each element of the pivoting set is unique for debugging
// purposes. This is an expensive operation and should only be executed in
// debug mode.
template <class T>
bool UnrevisedLemkeSolver<T>::IsEachUnique(
    const std::vector<LCPVariable>& vars) {
  // Copy the set.
  std::vector<LCPVariable> vars_copy = vars;

  // Sort the vector.
  std::sort(vars_copy.begin(), vars_copy.end(),
            [](const LCPVariable& a, const LCPVariable& b) -> bool {
    if (a.index < b.index) {
      return true;
    } else {
      if (a.index > b.index) {
        return false;
      } else {
        // If here, the indices are equal. We will arbitrarily order w before
        // z (alphabetical ordering). The assumption that follows is that
        // the two will not be equal.
        if (!a.z) {
          DRAKE_DEMAND(b.z);
          return true;
        } else {
          DRAKE_DEMAND(!b.z);
          return false;
        }
      }
    }
  });

  // Verify that all elements are unique.
  return (std::unique(vars_copy.begin(), vars_copy.end(),
          [](const LCPVariable& a, const LCPVariable& b) -> bool {
    return (a.index == b.index && a.z == b.z);
  }) == vars_copy.end());
}

template <typename T>
void UnrevisedLemkeSolver<T>::LemkePivot(
    const MatrixX<T>& M,
    const VectorX<T>& q,
    int driving_index,
    VectorX<T>* M_prime_col,
    VectorX<T>* q_prime) const {
  DRAKE_DEMAND(q_prime);

  const int kArtificial = 0;

  // Verify that each member in the independent and dependent sets is unique.
  DRAKE_ASSERT(IsEachUnique(indep_variables_));
  DRAKE_ASSERT(IsEachUnique(dep_variables_));

  // If the driving index does not correspond to the artificial variable,
  // M_prime_col must be non-null.
  if (!indep_variables_[driving_index].z ||
      indep_variables_[driving_index].index != kArtificial) {
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
  SelectSubMatrixWithCovering(M, index_sets_.bar_alpha, index_sets_.beta,
                  &M_prime_alpha_beta_);
  SelectSubVector(q, index_sets_.alpha, &q_alpha_);
  SelectSubVector(q, index_sets_.bar_alpha, &q_bar_alpha_);

  // Equation (10).
  LinearSolver<T> fMab(M_alpha_beta_);  // Factorized M_alpha_beta_.
  q_prime_beta_prime_ = -fMab.Solve(q_alpha_);

  // Equation (11).
  q_prime_bar_alpha_prime_ = M_prime_alpha_beta_ * q_prime_beta_prime_ +
      q_bar_alpha_;

  // Set the components of q'.
  SetSubVector(index_sets_.beta_prime, q_prime_beta_prime_, q_prime);
  SetSubVector(index_sets_.bar_alpha_prime, q_prime_bar_alpha_prime_, q_prime);

  DRAKE_SPDLOG_DEBUG(log(), "q': {}", q_prime->transpose());

  // If it is not necessary to compute the column of M, quit now.
  if (!M_prime_col)
    return;

  // Examine the driving variable.
  if (!indep_variables_[driving_index].z) {
    DRAKE_SPDLOG_DEBUG(log(), "Driving case #1: driving variable from w");
    // Case from Section 2.2.1.
    // Determine gamma by determining the position of the driving variable
    // in INDEPENDENT W.
    int gamma = 0;
    for (int i = 0; i < static_cast<int>(indep_variables_.size()); ++i) {
      if (!indep_variables_[i].z) {
        if (indep_variables_[i].index < indep_variables_[driving_index].index)
          ++gamma;
      }
    }

    // Set the unit vector.
    e_.setZero(index_sets_.beta.size());
    e_[gamma] = 1.0;

    // Equation (15).
    M_prime_driving_beta_prime_ = fMab.Solve(e_);

    // Equation (16).
    M_prime_driving_bar_alpha_prime_ = M_prime_alpha_beta_ *
        M_prime_driving_beta_prime_;
  } else {
    DRAKE_SPDLOG_DEBUG(log(), "Driving case #2: driving variable from z");

    // Case from Section 2.2.2.
    // Determine zeta.
    int zeta = indep_variables_[driving_index].index;

    // Compute g_alpha and g_bar_alpha.
    SelectSubColumnWithCovering(M, index_sets_.alpha, zeta, &g_alpha_);
    SelectSubColumnWithCovering(M, index_sets_.bar_alpha, zeta, &g_bar_alpha_);

    // Equation (19).
    M_prime_driving_beta_prime_ = -fMab.Solve(g_alpha_);

    // Equation (20).
    M_prime_driving_bar_alpha_prime_ = g_bar_alpha_ +
        M_prime_alpha_beta_ * M_prime_driving_beta_prime_;
  }

  SetSubVector(index_sets_.beta_prime, M_prime_driving_beta_prime_,
               M_prime_col);
  SetSubVector(index_sets_.bar_alpha_prime, M_prime_driving_bar_alpha_prime_,
               M_prime_col);

  DRAKE_SPDLOG_DEBUG(log(), "M' (driving): {}", M_prime_col->transpose());
}

// O(n) method for finding the index of the complement of an LCP variable in
// a set (strictly speaking, an unsorted vector) of indices. Aborts if the
// index is not found in the set.
template <class T>
int UnrevisedLemkeSolver<T>::FindComplementIndex(
    const LCPVariable& query, const std::vector<LCPVariable>& indep_variables) {
  // Verify that the query is not the artificial variable.
  DRAKE_DEMAND(!(query.z && query.index == 0));

  for (int i = 0; i < static_cast<int>(indep_variables.size()); ++i) {
    if (indep_variables[i].z != query.z &&
        indep_variables[i].index == query.index) {
      return i;
    }
  }

  DRAKE_ABORT();
  return -1;
}

template <class T>
void UnrevisedLemkeSolver<T>::ConstructLemkeSolution(
    const MatrixX<T>& M,
    const VectorX<T>& q,
    int artificial_index,
    VectorX<T>* z) const {
  const int n = q.rows();

  // Compute the solution.
  VectorX<T> q_prime(n);
  LemkePivot(M, q, artificial_index, nullptr, &q_prime);

  z->setZero(n);
  for (int i = 0; i < static_cast<int>(dep_variables_.size()); ++i) {
    if (dep_variables_[i].z)
      (*z)[dep_variables_[i].index - 1] = q_prime[i];
  }
}

template <typename T>
bool UnrevisedLemkeSolver<T>::SolveLcpLemke(const MatrixX<T>& M,
                                     const VectorX<T>& q, VectorX<T>* z,
                                     int* num_pivots,
                                     const T& piv_tol,
                                     const T& zero_tol) const {
  using std::max;
  using std::abs;
  DRAKE_DEMAND(num_pivots);

  SPDLOG_DEBUG(log(), "UnrevisedLemkeSolver::SolveLcpLemke() entered, M: {}, "
      "q: {}, ", M, q.transpose());

  const int n = q.size();
//  const int max_pivots = std::min(1000, 50 * n);
  const int max_pivots = std::numeric_limits<int>::max();

  if (M.rows() != n || M.cols() != n)
    throw std::logic_error("M's dimensions do not match that of q.");

  // Update the pivots.
  *num_pivots = 0;

  // Look for immediate exit.
  if (n == 0) {
    z->resize(0);
    return true;
  }

  // Denote the index of the artificial variable.
  const int kArtificial = 0;

  // Compute a sensible value for zero tolerance if none is given.
  T mod_zero_tol = zero_tol;
  if (mod_zero_tol <= 0)
    mod_zero_tol = ComputeZeroTolerance(M);

  if (CheckLemkeTrivial(n, mod_zero_tol, q, z)) {
    SPDLOG_DEBUG(log(), " -- trivial solution found");
    SPDLOG_DEBUG(log(), "UnrevisedLemkeSolver::SolveLcpLemke() exited");
    return true;
  }

  // Clear the cyling selections.
  selections_.clear();

  // If 'n' is identical to the size of the last problem solved, try using the
  // indices from the last problem solved.
  if (static_cast<size_t>(n) == dep_variables_.size()) {
    // Verify that the last call found a solution (indicated by the presence
    // of zn in the independent set).
    int zn_index = -1;
    for (int i = 0;
         i < static_cast<int>(indep_variables_.size()) && zn_index < 0; ++i) {
      if (indep_variables_[i].z &&indep_variables_[i].index == kArtificial)
        zn_index = i;
    }

    if (zn_index >= 0) {
      // Compute the candidate solution.
      ConstructLemkeSolution(M, q, zn_index, z);

      // Find the minima of z and w.
      const T min_z = z->minCoeff();
      const auto w = M * (*z) + q;
      const T min_w = w.minCoeff();

      // Compute z and w.
      const T dot = w.dot(*z);

      // If the solution is good, return now, indicating only one pivot
      // was performed.
      if (min_z > -zero_tol && min_w > -zero_tol && abs(dot) < 10*n*zero_tol) {
        ++(*num_pivots);
        return true;
      }
    }
  }

  // Set the LCP variables. Start with all z variables independent and all w
  // variables dependent.
  indep_variables_.resize(n+1);
  dep_variables_.resize(n);
  for (int i = 0; i < n; ++i) {
    dep_variables_[i].z = false;
    indep_variables_[i].z = true;
    dep_variables_[i].index = i+1;
    indep_variables_[i].index = i;
  }

  // z needs one more variable.
  indep_variables_[n].z = true;
  indep_variables_[n].index = n;

  // Compute zn*, the smallest value of the artificial variable zn for which
  // w = q + zn >= 0. Let blocking denote a component of w that equals
  // zero when zn = zn*.
  int blocking_index = -1;
  T znstar = 0;
  for (int i = 0; i < n; ++i) {
    if (q[i] < 0) {
      if (-q[i] > znstar) {
        znstar = -q[i];
        blocking_index = i;
      }
    }
  }
  DRAKE_DEMAND(blocking_index >= 0);

  // Pivot blocking, artificial
  LCPVariable blocking = dep_variables_[blocking_index];
  int driving_index = FindComplementIndex(blocking, indep_variables_);
  std::swap(dep_variables_[blocking_index], indep_variables_[0]);
  DRAKE_SPDLOG_DEBUG(log(), "First blocking variable {}{}",
                     ((blocking.z) ? "z" : "w"), blocking.index);
  DRAKE_SPDLOG_DEBUG(log(), "First driving variable (artificial)");

  // Output the independent and dependent variable tuples.
  auto to_string = [](const std::vector<LCPVariable>& vars) -> std::string {
    std::ostringstream oss;
    for (int i = 0; i < static_cast<int>(vars.size()); ++i)
      oss << ((vars[i].z) ? "z" : "w") << vars[i].index << " ";
    return oss.str();
  };
  DRAKE_SPDLOG_DEBUG(log(), "Independent set variables: {}",
      to_string(indep_variables_));
  DRAKE_SPDLOG_DEBUG(log(), "Dependent set variables: {}",
      to_string(dep_variables_));

  // Pivot up to the maximum number of times.
  VectorX<T> q_prime(n), M_prime_col(n);
  while (++(*num_pivots) < max_pivots) {
    DRAKE_SPDLOG_DEBUG(log(), "New driving variable {}{}",
                       ((indep_variables_[driving_index].z) ? "z" : "w"),
                       indep_variables_[driving_index].index);

    // Compute the permuted q and driving column of the permuted M matrix.
    LemkePivot(M, q, driving_index, &M_prime_col, &q_prime);

    // Perform the minimum ratio test.
    T min_ratio = std::numeric_limits<double>::infinity();
    blocking_index = -1;
    for (int i = 0; i < M_prime_col.size(); ++i) {
      if (M_prime_col[i] < -mod_zero_tol) {
        const T ratio = -q_prime[i] / M_prime_col[i];
        DRAKE_SPDLOG_DEBUG(log(), "Ratio for index {}: {}", i, ratio);
        if (ratio < min_ratio) {
          min_ratio = ratio;
          blocking_index = i;
        }
      }
    }

    if (blocking_index < 0) {
      SPDLOG_DEBUG(log(), "driving variable is unblocked- algorithm failed");
      z->setZero(n);
      return false;
    }

    // Determine all variables within the zero tolerance of the minimum ratio.
    std::vector<int> blocking_indices;
    for (int i = 0; i < M_prime_col.size(); ++i) {
      if (M_prime_col[i] < -mod_zero_tol) {
        const T ratio = -q_prime[i] / M_prime_col[i];
        DRAKE_SPDLOG_DEBUG(log(), "Ratio for index {}: {}", i, ratio);
        if (ratio < min_ratio + mod_zero_tol)
          blocking_indices.push_back(i);
      }
    }

    // If there are multiple blocking variables, replace the blocking index with
    // the cycling selection.
    if (blocking_indices.size() > 1) {
      auto& index = selections_[indep_variables_];
      DRAKE_DEMAND(index < static_cast<int>(blocking_indices.size()));
      blocking_index = blocking_indices[index];
      ++index;
    }

    // Get the blocking variable.
    blocking = dep_variables_[blocking_index];
    DRAKE_SPDLOG_DEBUG(log(), "Blocking variable {}{}",
                       ((blocking.z) ? "z" : "w"), blocking.index);

    // See whether the artificial variable blocks the driving variable.
    if (blocking.index == kArtificial) {
      DRAKE_DEMAND(blocking.z);

      // Pivot zn with the driving variable.
      std::swap(dep_variables_[blocking_index],
                indep_variables_[driving_index]);

      // Compute the permuted q, and convert it into a solution.
      ConstructLemkeSolution(M, q, driving_index, z);
      return true;
    } else {
      // Pivot the blocking variable and the driving variable.
      std::swap(dep_variables_[blocking_index],
                indep_variables_[driving_index]);

      // Make the driving variable the complement of the blocking variable.
      driving_index = FindComplementIndex(blocking, indep_variables_);
    }

    DRAKE_SPDLOG_DEBUG(log(), "Independent set variables: {}",
        to_string(indep_variables_));
    DRAKE_SPDLOG_DEBUG(log(), "Dependent set variables: {}",
        to_string(dep_variables_));
  }

  // If here, the maximum number of pivots has been exceeded.
  z->setZero(n);
  return false;
}

template <class T>
bool UnrevisedLemkeSolver<T>::SolveLcpLemkeRegularized(const MatrixX<T>& M,
                              const VectorX<T>& q, VectorX<T>* z,
                              int* num_pivots,
                              int min_exp, int step_exp,
                              int max_exp, const T& piv_tol,
                              const T& zero_tol) const {
  DRAKE_ABORT();
  return true;
}


template <typename T>
SolverId UnrevisedLemkeSolver<T>::solver_id() const {
  return UnrevisedLemkeSolverId::id();
}

SolverId UnrevisedLemkeSolverId::id() {
  static const never_destroyed<SolverId> singleton{"Unrevised Lemke"};
  return singleton.access();
}

// Instantiate templates.
template class UnrevisedLemkeSolver<double>;
template class
    drake::solvers::UnrevisedLemkeSolver<
        Eigen::AutoDiffScalar<drake::Vector1d>>;

}  // namespace solvers
}  // namespace drake
