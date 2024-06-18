#include "drake/solvers/semidefinite_relaxation.h"

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
#include "drake/math/matrix_util.h"
#include "drake/solvers/program_attribute.h"
#include "drake/solvers/semidefinite_relaxation_internal.h"

namespace drake {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Triplet;
using Eigen::VectorXd;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

namespace {

// TODO(AlexandreAmice) Move all these methods to
// semidefinite_relaxation_internal.

// Constructs the semidefinite relaxation of the program prog and adds it to
// relaxation. We assume that the program attributes of prog are already
// validated and that relaxation already contains all the variables and
// constraints of prog. The variable one is already constrained to be equal to
// one. This is passed so it can be re-used across semidefinite variables in the
// sparse version of MakeSemidefiniteRelaxation. Returns the X matrix of the
// semidefinite relaxation.
MatrixXDecisionVariable DoMakeSemidefiniteRelaxation(
    const MathematicalProgram& prog, const Variable& one,
    MathematicalProgram* relaxation,
    std::optional<int> group_number = std::nullopt) {
  // Build a symmetric matrix X of decision variables using the original
  // program variables (so that GetSolution, etc, works using the original
  // variables).
  relaxation->AddDecisionVariables(prog.decision_variables());
  MatrixX<Variable> X(prog.num_vars() + 1, prog.num_vars() + 1);
  // X = xxᵀ; x = [prog.decision_vars(); 1].
  std::string name =
      group_number.has_value() ? fmt::format("Y{}", group_number.value()) : "Y";
  X.topLeftCorner(prog.num_vars(), prog.num_vars()) =
      relaxation->NewSymmetricContinuousVariables(prog.num_vars(), name);
  // We sort the variables so that the matrix X is ordered in a predictable way.
  // This makes it easier when using the sparsity groups to make the
  // semidefinite matrices agree.
  VectorX<Variable> sorted_variables = prog.decision_variables();
  std::sort(sorted_variables.data(),
            sorted_variables.data() + sorted_variables.size(),
            std::less<Variable>{});
  X.topRightCorner(prog.num_vars(), 1) = sorted_variables;
  X.bottomLeftCorner(1, prog.num_vars()) = sorted_variables.transpose();

  std::map<Variable, int> variables_to_sorted_indices;
  int i = 0;
  for (const auto& v : sorted_variables) {
    variables_to_sorted_indices[v] = i++;
  }

  // X(-1,-1) = 1.
  X(prog.num_vars(), prog.num_vars()) = one;

  // X ≽ 0.
  relaxation->AddPositiveSemidefiniteConstraint(X);

  internal::DoLinearizeQuadraticCostsAndConstraints(
      prog, X, variables_to_sorted_indices, relaxation);
  internal::DoAddImpliedLinearConstraints(prog, X, variables_to_sorted_indices,
                                          relaxation);
  internal::DoAddImpliedLinearEqualityConstraints(
      prog, X, variables_to_sorted_indices, relaxation);
  return X;
}
}  // namespace

std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog) {
  internal::ValidateProgramIsSupported(prog);
  auto relaxation = prog.Clone();
  const Variable one("one");
  relaxation->AddDecisionVariables(Vector1<Variable>(one));
  relaxation->AddLinearEqualityConstraint(one, 1);
  DoMakeSemidefiniteRelaxation(prog, one, relaxation.get());
  return relaxation;
}

std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog,
    const std::vector<symbolic::Variables>& variable_groups) {
  auto relaxation = prog.Clone();
  const Variable one("one");
  relaxation->AddDecisionVariables(Vector1<Variable>(one));
  relaxation->AddLinearEqualityConstraint(one, 1);

  // The semidefinite relaxation of each variable group will be computed
  // individually and any variables which overlap in the programs will later be
  // made to agree using equality constraints. The container programs in this
  // map are used to store all the costs and constraints needed to compute the
  // semidefinite relaxation of each variable group.
  std::map<symbolic::Variables, solvers::MathematicalProgram>
      groups_to_container_programs;
  std::map<symbolic::Variables, MatrixXDecisionVariable>
      groups_to_psd_variables;

  for (const auto& group : variable_groups) {
    groups_to_container_programs.try_emplace(group);
    VectorXDecisionVariable group_vec(group.size());
    int i = 0;
    for (const auto& v : group) {
      group_vec(i) = v;
      ++i;
    }
    groups_to_container_programs.at(group).AddDecisionVariables(group_vec);
  }

  for (const auto& constraint : prog.GetAllConstraints()) {
    const Variables constraint_variables{constraint.variables()};
    for (const auto& group : variable_groups) {
      if (constraint_variables.IsSubsetOf(group)) {
        // There is no need to add constraint_variables to the
        // container_program, since the variables are a subset of the group and
        // therefore already in the program.
        groups_to_container_programs.at(group).AddConstraint(constraint);
      }
    }
  }
  for (const auto& cost : prog.GetAllCosts()) {
    const Variables cost_variables{cost.variables()};
    for (const auto& group : variable_groups) {
      if (cost_variables.IsSubsetOf(group)) {
        groups_to_container_programs.at(group).AddCost(cost);
        // If the variables in this cost are a subset of multiple variable
        // groups, then these variables will correspond to submatrices of the
        // relaxed PSD variables. Since later, we will enforce that all these
        // submatrices be equal, we only need to add the cost exactly once.
        break;
      }
    }
  }

  int group_number = 0;
  for (const auto& [group, container_program] : groups_to_container_programs) {
    groups_to_psd_variables.emplace(
        group, DoMakeSemidefiniteRelaxation(container_program, one,
                                            relaxation.get(), group_number));
    ++group_number;
  }

  // Now constrain the semidefinite variables to agree where they overlap.
  for (auto it = groups_to_psd_variables.begin();
       it != groups_to_psd_variables.end(); ++it) {
    for (auto it2 = std::next(it); it2 != groups_to_psd_variables.end();
         ++it2) {
      const Variables common_variables = intersect(it->first, it2->first);
      if (!common_variables.empty()) {
        auto get_submatrix_of_variables =
            [&common_variables](const MatrixXDecisionVariable& X) {
              std::set<int> submatrix_indices;
              for (const auto& v : common_variables) {
                for (int i = 0; i < X.rows() - 1; ++i) {
                  if (X(i, X.cols() - 1).equal_to(v)) {
                    submatrix_indices.insert(i);
                    break;
                  }
                }
              }
              return math::ExtractPrincipalSubmatrix(X, submatrix_indices);
            };
        relaxation->AddLinearEqualityConstraint(
            get_submatrix_of_variables(it->second) ==
            get_submatrix_of_variables(it2->second));
      }
    }
  }

  if (internal::CheckProgramHasNonConvexQuadratics(*relaxation)) {
    throw std::runtime_error(
        "There is a non-convex cost or constraint in the program whose "
        "variables do not overlap with any variable groups. Therefore, these "
        "costs or constraints would not be converted to convex, semidefinite "
        "constraints and so the returned program would not be convex. Consider "
        "further specifying the variable groups.");
  }

  return relaxation;
}

}  // namespace solvers
}  // namespace drake
