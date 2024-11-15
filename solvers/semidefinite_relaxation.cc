#include "drake/solvers/semidefinite_relaxation.h"

#include <map>
#include <optional>
#include <set>

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

// Validate that we can construct the semidefinite relaxation of this program
// and prepare a relaxation program. The relaxation program will be initialized
// as a clone of the program prog, with the additional variable "one"
// constrained to be equal to one. This relaxation is processed by
// DoMakeSemidefiniteRelaxation in different ways.
std::unique_ptr<MathematicalProgram> InitializeRelaxation(
    const MathematicalProgram& prog, const Variable& one) {
  internal::ValidateProgramIsSupported(prog);
  auto relaxation = prog.Clone();
  relaxation->AddDecisionVariables(Vector1<Variable>(one));
  relaxation->AddLinearEqualityConstraint(one, 1);
  return relaxation;
}

// Constructs the semidefinite relaxation of the program sub_prog and adds it to
// relaxation. We assume that sub_prog is a sub-program of some larger program
// prog and relaxation was initialized by a call to InitializeRelaxation on
// prog. By sub-program, we mean that sub_prog contains only variables and
// constraints found in prog.
//
// This is equivalent to assuming that the program attributes of sub_prog are
// already validated, that relaxation already contains all the variables and
// constraints of sub_prog, and that the variable one is already constrained to
// be equal to one. The variable one is passed so it can be re-used across
// semidefinite variables in the sparse version of MakeSemidefiniteRelaxation.
//
// Returns the X matrix of the semidefinite relaxation.
MatrixXDecisionVariable DoMakeSemidefiniteRelaxation(
    const MathematicalProgram& sub_prog, const Variable& one,
    const SemidefiniteRelaxationOptions& options,
    MathematicalProgram* relaxation,
    std::optional<int> group_number = std::nullopt) {
  MatrixX<Variable> X;
  std::map<Variable, int> variables_to_sorted_indices;
  internal::InitializeSemidefiniteRelaxationForProg(
      sub_prog, one, relaxation, &X, &variables_to_sorted_indices,
      group_number);

  internal::DoLinearizeQuadraticCostsAndConstraints(
      sub_prog, X, variables_to_sorted_indices, relaxation);

  if (options.add_implied_linear_constraints) {
    internal::DoAddImpliedLinearConstraints(
        sub_prog, X, variables_to_sorted_indices, relaxation);
  }
  if (options.add_implied_linear_equality_constraints) {
    internal::DoAddImpliedLinearEqualityConstraints(
        sub_prog, X, variables_to_sorted_indices, relaxation);
  }
  return X;
}
}  // namespace

std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog,
    const SemidefiniteRelaxationOptions& options) {
  const Variable one("one");
  auto relaxation = InitializeRelaxation(prog, one);
  DoMakeSemidefiniteRelaxation(prog, one, options, relaxation.get());
  return relaxation;
}

std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog,
    const std::vector<symbolic::Variables>& variable_groups,
    const SemidefiniteRelaxationOptions& options) {
  const Variable one("one");
  auto relaxation = InitializeRelaxation(prog, one);

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
        group, DoMakeSemidefiniteRelaxation(container_program, one, options,
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
