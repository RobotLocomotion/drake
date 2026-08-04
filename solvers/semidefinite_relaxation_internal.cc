#include "drake/solvers/semidefinite_relaxation_internal.h"

#include <algorithm>
#include <functional>
#include <initializer_list>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/fmt_eigen.h"
#include "drake/common/symbolic/decompose.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/program_attribute.h"

namespace drake {
namespace solvers {
namespace internal {

using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Triplet;
using Eigen::VectorXd;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

namespace {
const double kInf = std::numeric_limits<double>::infinity();

// Given a mapping from decision variables in a program to indices, return all
// the indices corresponding to the variable vars. This method is useful as in
// DoMakeSemidefiniteRelaxation, we sort the decision variables in the
// semidefinite variables (and hence the implied constraints).
std::vector<int> FindDecisionVariableIndices(
    const std::map<Variable, int>& variables_to_sorted_indices,
    const VectorXDecisionVariable& vars) {
  std::vector<int> indices;
  indices.reserve(vars.rows());
  for (const auto& v : vars) {
    indices.emplace_back(variables_to_sorted_indices.at(v));
  }
  return indices;
}

}  // namespace

void ValidateProgramIsSupported(const MathematicalProgram& prog) {
  std::string unsupported_message{};
  const ProgramAttributes supported_attributes(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kQuadraticConstraint});
  if (!AreRequiredAttributesSupported(prog.required_capabilities(),
                                      supported_attributes,
                                      &unsupported_message)) {
    throw std::runtime_error(fmt::format(
        "MakeSemidefiniteRelaxation() does not (yet) support this program: {}.",
        unsupported_message));
  }
}

bool CheckProgramHasNonConvexQuadratics(const MathematicalProgram& prog) {
  return std::any_of(prog.quadratic_costs().begin(),
                     prog.quadratic_costs().end(),
                     [](const auto& cost) {
                       return !cost.evaluator()->is_convex();
                     }) ||
         std::any_of(prog.quadratic_constraints().begin(),
                     prog.quadratic_constraints().end(),
                     [](const auto& constraint) {
                       return !constraint.evaluator()->is_convex();
                     });
}

void InitializeSemidefiniteRelaxationForProg(
    const MathematicalProgram& prog, const Variable& one,
    MathematicalProgram* relaxation, MatrixX<Variable>* X,
    std::map<Variable, int>* variables_to_sorted_indices,
    std::optional<int> group_number) {
  DRAKE_DEMAND(relaxation != nullptr);
  DRAKE_DEMAND(X != nullptr);
  DRAKE_DEMAND(variables_to_sorted_indices != nullptr);
  // Build a symmetric matrix X of decision variables using the original
  // program variables (so that GetSolution, etc, works using the original
  // variables).
  std::string name =
      group_number.has_value() ? fmt::format("Y{}", group_number.value()) : "Y";
  // We sort the variables so that the matrix X is ordered in a predictable way.
  // This makes it easier when using the sparsity groups to make the
  // semidefinite matrices agree.
  VectorX<Variable> sorted_variables = prog.decision_variables();
  std::sort(sorted_variables.data(),
            sorted_variables.data() + sorted_variables.size(),
            std::less<Variable>{});
  // X = xxᵀ; x = [prog.decision_vars(); 1].
  X->resize(prog.num_vars() + 1, prog.num_vars() + 1);
  X->topLeftCorner(prog.num_vars(), prog.num_vars()) =
      relaxation->NewSymmetricContinuousVariables(prog.num_vars(), name);
  X->bottomLeftCorner(1, prog.num_vars()) = sorted_variables.transpose();
  X->topRightCorner(prog.num_vars(), 1) = sorted_variables;
  // X(-1,-1) = 1.
  (*X)(prog.num_vars(), prog.num_vars()) = one;

  int i = 0;
  variables_to_sorted_indices->clear();
  for (const auto& v : sorted_variables) {
    (*variables_to_sorted_indices)[v] = i++;
  }

  relaxation->AddPositiveSemidefiniteConstraint(*X);
}

void DoLinearizeQuadraticCostsAndConstraints(
    const MathematicalProgram& prog, const MatrixXDecisionVariable& X,
    const std::map<Variable, int>& variables_to_sorted_indices,
    MathematicalProgram* relaxation) {
  DRAKE_DEMAND(relaxation != nullptr);
  // Returns the {a, vars} in relaxation, such that a' vars = 0.5*tr(QY). This
  // assumes Q=Q', which is ensured by QuadraticCost and QuadraticConstraint.
  auto half_trace_QY = [&X, &variables_to_sorted_indices](
                           const Eigen::MatrixXd& Q,
                           const VectorXDecisionVariable& binding_vars)
      -> std::pair<VectorXd, VectorX<Variable>> {
    const int N = binding_vars.size();
    const int num_vars = N * (N + 1) / 2;
    const std::vector<int> indices =
        FindDecisionVariableIndices(variables_to_sorted_indices, binding_vars);

    VectorXd a = VectorXd::Zero(num_vars);
    VectorX<Variable> y(num_vars);
    int count = 0;
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j <= i; ++j) {
        // tr(QY) = ∑ᵢ ∑ⱼ Qᵢⱼ Yⱼᵢ.
        a[count] = ((i == j) ? 0.5 : 1.0) * Q(i, j);
        y[count] = X(indices[i], indices[j]);
        ++count;
      }
    }
    return {a, y};
  };

  // Remove the quadratic cost in relaxation and replace it with a linear cost
  // on the semidefinite variables i.e.
  // 0.5 y'Qy + b'y + c -> 0.5 tr(QY) + b'y + c
  for (const auto& binding : prog.quadratic_costs()) {
    relaxation->RemoveCost(binding);
    const int N = binding.variables().size();
    const int num_vars = N + (N * (N + 1) / 2);
    std::pair<VectorXd, VectorX<Variable>> quadratic_terms =
        half_trace_QY(binding.evaluator()->Q(), binding.variables());
    VectorXd a(num_vars);
    VectorX<Variable> vars(num_vars);
    a << quadratic_terms.first, binding.evaluator()->b();
    vars << quadratic_terms.second, binding.variables();
    relaxation->AddLinearCost(a, binding.evaluator()->c(), vars);
  }

  // Remove the quadratic constraints and replace them with a linear  constraint
  // on the semidefinite variables i.e.
  // lb ≤ 0.5 y'Qy + b'y ≤ ub -> lb ≤ 0.5 tr(QY) + b'y ≤ ub.
  // if lb == ub then add as an equality constraint instead.
  for (const auto& binding : prog.quadratic_constraints()) {
    relaxation->RemoveConstraint(binding);
    const int N = binding.variables().size();
    const int num_vars = N + (N * (N + 1) / 2);
    std::pair<VectorXd, VectorX<Variable>> quadratic_terms =
        half_trace_QY(binding.evaluator()->Q(), binding.variables());
    VectorXd a(num_vars);
    VectorX<Variable> vars(num_vars);
    a << quadratic_terms.first, binding.evaluator()->b();
    vars << quadratic_terms.second, binding.variables();
    if (binding.evaluator()->lower_bound() ==
        binding.evaluator()->upper_bound()) {
      relaxation->AddLinearEqualityConstraint(
          a.transpose(), binding.evaluator()->lower_bound(), vars);
    } else {
      relaxation->AddLinearConstraint(a.transpose(),
                                      binding.evaluator()->lower_bound(),
                                      binding.evaluator()->upper_bound(), vars);
    }
  }
}

void DoAddImpliedLinearConstraints(
    const MathematicalProgram& prog, const MatrixXDecisionVariable& X,
    const std::map<Variable, int>& variables_to_sorted_indices,
    MathematicalProgram* relaxation) {
  DRAKE_DEMAND(relaxation != nullptr);
  // Assemble one big Ay <= b matrix from all bounding box constraints
  // and linear constraints
  // TODO(bernhardpg): Consider special-casing linear equality constraints
  // that are added as bounding box or linear constraints with lb == ub
  int num_constraints = 0;
  int nnz = 0;
  for (const auto& binding : prog.bounding_box_constraints()) {
    for (int i = 0; i < binding.evaluator()->num_constraints(); ++i) {
      if (std::isfinite(binding.evaluator()->lower_bound()[i])) {
        ++num_constraints;
      }
      if (std::isfinite(binding.evaluator()->upper_bound()[i])) {
        ++num_constraints;
      }
    }
    nnz += binding.evaluator()->get_sparse_A().nonZeros();
  }
  for (const auto& binding : prog.linear_constraints()) {
    for (int i = 0; i < binding.evaluator()->num_constraints(); ++i) {
      if (std::isfinite(binding.evaluator()->lower_bound()[i])) {
        ++num_constraints;
      }
      if (std::isfinite(binding.evaluator()->upper_bound()[i])) {
        ++num_constraints;
      }
    }
    nnz += binding.evaluator()->get_sparse_A().nonZeros();
  }

  std::vector<Triplet<double>> A_triplets;
  A_triplets.reserve(nnz);
  SparseMatrix<double> A(num_constraints, prog.num_vars());
  VectorXd b(num_constraints);

  int constraint_idx = 0;
  for (const auto& binding : prog.bounding_box_constraints()) {
    const std::vector<int> indices = FindDecisionVariableIndices(
        variables_to_sorted_indices, binding.variables());
    for (int i = 0; i < binding.evaluator()->num_constraints(); ++i) {
      if (std::isfinite(binding.evaluator()->lower_bound()[i])) {
        A_triplets.push_back(Triplet<double>(constraint_idx, indices[i], -1.0));
        b(constraint_idx++) = -binding.evaluator()->lower_bound()[i];
      }
      if (std::isfinite(binding.evaluator()->upper_bound()[i])) {
        A_triplets.push_back(Triplet<double>(constraint_idx, indices[i], 1.0));
        b(constraint_idx++) = binding.evaluator()->upper_bound()[i];
      }
    }
  }

  for (const auto& binding : prog.linear_constraints()) {
    const std::vector<int> indices = FindDecisionVariableIndices(
        variables_to_sorted_indices, binding.variables());
    // TODO(hongkai-dai): Consider using the SparseMatrix iterators.
    for (int i = 0; i < binding.evaluator()->num_constraints(); ++i) {
      if (std::isfinite(binding.evaluator()->lower_bound()[i])) {
        for (int j = 0; j < binding.evaluator()->num_vars(); ++j) {
          if (binding.evaluator()->get_sparse_A().coeff(i, j) != 0) {
            A_triplets.push_back(Triplet<double>(
                constraint_idx, indices[j],
                -binding.evaluator()->get_sparse_A().coeff(i, j)));
          }
        }
        b(constraint_idx++) = -binding.evaluator()->lower_bound()[i];
      }
      if (std::isfinite(binding.evaluator()->upper_bound()[i])) {
        for (int j = 0; j < binding.evaluator()->num_vars(); ++j) {
          if (binding.evaluator()->get_sparse_A().coeff(i, j) != 0) {
            A_triplets.push_back(Triplet<double>(
                constraint_idx, indices[j],
                binding.evaluator()->get_sparse_A().coeff(i, j)));
          }
        }
        b(constraint_idx++) = binding.evaluator()->upper_bound()[i];
      }
    }
  }
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());

  // 0 ≤ (Ay-b)(Ay-b)ᵀ, implemented with
  // -bbᵀ ≤ AYAᵀ - b(Ay)ᵀ - (Ay)bᵀ.
  // TODO(russt): Avoid the symbolic computation here.
  // TODO(russt): Avoid the dense matrix.
  const MatrixX<Expression> AYAT =
      A * X.topLeftCorner(prog.num_vars(), prog.num_vars()) * A.transpose();
  const VectorX<Variable> y = X.col(prog.num_vars()).head(prog.num_vars());

  const VectorX<Expression> rhs_flat_tril =
      math::ToLowerTriangularColumnsFromMatrix(AYAT - b * (A * y).transpose() -
                                               A * y * b.transpose());
  const VectorXd bbT_flat_tril =
      math::ToLowerTriangularColumnsFromMatrix(-b * b.transpose());

  relaxation->AddLinearConstraint(
      rhs_flat_tril, bbT_flat_tril,
      VectorXd::Constant(bbT_flat_tril.size(), kInf));
}

void DoAddImpliedLinearEqualityConstraints(
    const MathematicalProgram& prog, const MatrixXDecisionVariable& X,
    const std::map<Variable, int>& variables_to_sorted_indices,
    MathematicalProgram* relaxation) {
  DRAKE_DEMAND(relaxation != nullptr);
  // Linear equality constraints.
  // Ay = b => (Ay-b)yᵀ = Ayyᵀ - byᵀ = 0.
  for (const auto& binding : prog.linear_equality_constraints()) {
    const int N = binding.variables().size();
    const std::vector<int> indices = FindDecisionVariableIndices(
        variables_to_sorted_indices, binding.variables());
    VectorX<Variable> vars(N + 1);
    // Add the constraints one column at a time:
    // Ayx_j - bx_j = 0.
    MatrixX<double> Ab(binding.evaluator()->num_constraints(), N + 1);
    // TODO(Alexandre.Amice) make this only access the sparse matrix.
    Ab.leftCols(N) = binding.evaluator()->GetDenseA();
    Ab.col(N) = -binding.evaluator()->lower_bound();
    // We don't need to do the last column of X.
    for (int j = 0; j < static_cast<int>(X.cols()) - 1; ++j) {
      for (int i = 0; i < N; ++i) {
        vars[i] = X(indices[i], j);
      }
      vars[N] = X(prog.num_vars(), j);
      relaxation->AddLinearEqualityConstraint(
          Ab, VectorXd::Zero(binding.evaluator()->num_constraints()), vars);
    }
  }
}

Eigen::SparseMatrix<double> SparseKroneckerProduct(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::SparseMatrix<double>& B) {
  Eigen::SparseMatrix<double> C(A.rows() * B.rows(), A.cols() * B.cols());
  std::vector<Eigen::Triplet<double>> C_triplets;
  C_triplets.reserve(A.nonZeros() * B.nonZeros());
  C.reserve(A.nonZeros() * B.nonZeros());
  for (int iA = 0; iA < A.outerSize(); ++iA) {
    for (SparseMatrix<double>::InnerIterator itA(A, iA); itA; ++itA) {
      for (int iB = 0; iB < B.outerSize(); ++iB) {
        for (SparseMatrix<double>::InnerIterator itB(B, iB); itB; ++itB) {
          C_triplets.emplace_back(itA.row() * B.rows() + itB.row(),
                                  itA.col() * B.cols() + itB.col(),
                                  itA.value() * itB.value());
        }
      }
    }
  }
  C.setFromTriplets(C_triplets.begin(), C_triplets.end());
  return C;
}

SparseMatrix<double> GetWAdjForTril(const int r) {
  DRAKE_DEMAND(r > 0);
  // Y is a symmetric matrix of size (r-1) hence we have (r choose 2) lower
  // triangular entries.
  const int Y_tril_size = (r * (r - 1)) / 2;

  std::vector<Triplet<double>> W_adj_triplets;
  // The map operates on the diagonal twice, and then on one of the columns
  // without the first element once.
  W_adj_triplets.reserve(2 * (r - 1) + (r - 2));

  int idx = 0;
  for (int i = 0; idx < Y_tril_size; ++i) {
    W_adj_triplets.emplace_back(0, idx, 1);
    W_adj_triplets.emplace_back(1, idx, idx > 0 ? -1 : 1);
    idx += (r - 1) - i;
  }

  for (int i = 2; i < r; ++i) {
    W_adj_triplets.emplace_back(i, i - 1, 2);
  }
  SparseMatrix<double> W_adj(r, Y_tril_size);
  W_adj.setFromTriplets(W_adj_triplets.begin(), W_adj_triplets.end());
  return W_adj;
}

namespace {
template <typename T,
          typename = std::enable_if_t<std::is_same_v<T, Expression> ||
                                      std::is_same_v<T, Variable>>>
void DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint(
    const Eigen::Ref<const MatrixX<T>>& X, MathematicalProgram* prog) {
  for (int i = 0; i < X.cols(); ++i) {
    // TODO(Alexandre.Amice) AddLorentzConeConstraint only has a MatrixBase
    // version of the method not an Eigen::Ref version and so we need to make
    // this temporary copy rather than being able to call
    // prog->AddLorentzConeConstraint(X.col(i));
    const VectorX<T> x = X.col(i);
    prog->AddLorentzConeConstraint(x);
  }
}
}  //  namespace

void AddMatrixIsLorentzByPositiveOrthantSeparableConstraint(
    const Eigen::Ref<const MatrixX<Variable>>& X, MathematicalProgram* prog) {
  DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint<Variable>(X, prog);
}

void AddMatrixIsLorentzByPositiveOrthantSeparableConstraint(
    const Eigen::Ref<const MatrixX<Expression>>& X, MathematicalProgram* prog) {
  DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint<Expression>(X, prog);
}

void AddMatrixIsPositiveOrthantByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<Variable>>& X, MathematicalProgram* prog) {
  DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint<Variable>(
      X.transpose(), prog);
}

void AddMatrixIsPositiveOrthantByLorentzSeparableConstraint(
    const Eigen::Ref<const MatrixX<Expression>>& X, MathematicalProgram* prog) {
  DoAddMatrixIsLorentzByPositiveOrthantSeparableConstraint<Expression>(
      X.transpose(), prog);
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
