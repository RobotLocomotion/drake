#include "drake/solvers/branch_and_bound.h"

namespace drake {
namespace solvers {
bool MathProgHasBinaryVariables(const MathematicalProgram& prog) {
  std::cout << "Call has binary variables.\n";
  for (int i = 0; i < prog.num_vars(); ++i) {
    if (prog.decision_variable(i).get_type() ==
        symbolic::Variable::Type::BINARY) {
      return true;
    }
  }
  return false;
}

MixedIntegerBranchAndBoundNode::MixedIntegerBranchAndBoundNode(
    const MathematicalProgram& prog,
    const Eigen::Ref<const VectorXDecisionVariable>& binary_variables)
    : prog_{prog.Clone()},
      left_child_{nullptr},
      right_child_{nullptr},
      parent_{nullptr},
      binary_var_index_{-1},
      binary_var_value_{-1},
      remaining_binary_variables_{} {
  // Check if there are still binary variables.
  DRAKE_ASSERT(!MathProgHasBinaryVariables(*prog_));
  // Add binary_variables to remaining_binary_variables_
  for (int i = 0; i < binary_variables.rows(); ++i) {
    remaining_binary_variables_.push_back(binary_variables(i));
  }
}

bool MixedIntegerBranchAndBoundNode::IsRoot() const {
  return parent_ == nullptr;
}

template <typename Constraint>
Binding<Constraint> ReplaceBoundVariables(
    const Binding<Constraint>& binding,
    const std::unordered_map<symbolic::Variable::Id, symbolic::Variable>&
        map_old_vars_to_new_vars) {
  const auto& old_bound_vars = binding.variables();
  VectorXDecisionVariable new_bound_vars(old_bound_vars.rows());
  for (int i = 0; i < new_bound_vars.rows(); ++i) {
    new_bound_vars(i) = map_old_vars_to_new_vars.at(old_bound_vars(i).get_id());
  }
  return Binding<Constraint>(binding.constraint(), new_bound_vars);
}

template <typename Cost>
void AddVectorOfCostToNewProgram(
    const std::vector<Binding<Cost>>& costs,
    const std::unordered_map<symbolic::Variable::Id, symbolic::Variable>&
        map_old_vars_to_new_vars,
    MathematicalProgram* new_prog) {
  for (const auto& cost : costs) {
    new_prog->AddCost(ReplaceBoundVariables(cost, map_old_vars_to_new_vars));
  }
}

template <typename Constraint>
void AddVectorOfConstraintToNewProgram(
    const std::vector<Binding<Constraint>>& constraints,
    const std::unordered_map<symbolic::Variable::Id, symbolic::Variable>&
        map_old_vars_to_new_vars,
    MathematicalProgram* new_prog) {
  for (const auto& constraint : constraints) {
    new_prog->AddConstraint(
        ReplaceBoundVariables(constraint, map_old_vars_to_new_vars));
  }
}

std::pair<std::unique_ptr<MixedIntegerBranchAndBoundNode>,
          std::unordered_map<symbolic::Variable::Id, symbolic::Variable>>
MixedIntegerBranchAndBoundNode::ConstructRootNode(
    const MathematicalProgram& prog) {
  // Construct a new optimization program, same as prog, but relaxing the binary
  // constraint to 0 ≤ y ≤ 1.
  MathematicalProgram new_prog;
  // First check the decision variables of prog. Construct a new set of decision
  // variables with the same name as those in prog, but the ID will be
  // different.
  const auto& prog_vars = prog.decision_variables();
  VectorXDecisionVariable new_vars(prog_vars.rows());
  std::unordered_map<symbolic::Variable::Id, symbolic::Variable>
      map_old_vars_to_new_vars;
  std::vector<int> binary_variable_indices{};
  for (int i = 0; i < prog_vars.rows(); ++i) {
    // If the prog_vars(i) is of type CONTINUOUS, then new_vars(i) is also
    // CONTINUOUS.
    switch (prog_vars(i).get_type()) {
      case symbolic::Variable::Type::CONTINUOUS: {
        new_vars(i) = symbolic::Variable(prog_vars(i).get_name(),
                                         symbolic::Variable::Type::CONTINUOUS);
        map_old_vars_to_new_vars.emplace_hint(
            map_old_vars_to_new_vars.end(), prog_vars(i).get_id(), new_vars(i));
        break;
      }
      case symbolic::Variable::Type::BINARY: {
        new_vars(i) = symbolic::Variable(prog_vars(i).get_name(),
                                         symbolic::Variable::Type::CONTINUOUS);
        map_old_vars_to_new_vars.emplace_hint(
            map_old_vars_to_new_vars.end(), prog_vars(i).get_id(), new_vars(i));
        binary_variable_indices.push_back(i);
        break;
      }
      default: {
        throw std::runtime_error(
            "This variable type is not supported in branch and bound.");
      }
    }
  }
  new_prog.WithVariables(new_vars);
  if (binary_variable_indices.empty()) {
    throw std::runtime_error(
        "No binary variable found in the optimization program.\n");
  }
  const int num_binary_variables = binary_variable_indices.size();
  VectorXDecisionVariable binary_variables(num_binary_variables);
  for (int i = 0; i < num_binary_variables; ++i) {
    binary_variables(i) = new_vars(binary_variable_indices[i]);
  }
  new_prog.AddBoundingBoxConstraint(Eigen::VectorXd::Zero(num_binary_variables),
                                    Eigen::VectorXd::Ones(num_binary_variables),
                                    binary_variables);

  // Add the indeterminates
  new_prog.WithIndeterminates(prog.indeterminates());

  // Now add all the costs in prog to new_prog.
  AddVectorOfCostToNewProgram(prog.generic_costs(), map_old_vars_to_new_vars,
                              &new_prog);
  AddVectorOfCostToNewProgram(prog.linear_costs(), map_old_vars_to_new_vars,
                              &new_prog);
  AddVectorOfCostToNewProgram(prog.quadratic_costs(), map_old_vars_to_new_vars,
                              &new_prog);

  // Now add all the constraints in prog to new_prog.
  // TODO(hongkai.dai): Make sure that all constraints stored in
  // MathematicalProgram are added. In the future, if we add more constraint
  // type to MathematicalProgram, we need to add that constraint to new_prog
  // here as well.
  AddVectorOfConstraintToNewProgram(prog.generic_constraints(),
                                    map_old_vars_to_new_vars, &new_prog);
  AddVectorOfConstraintToNewProgram(prog.linear_constraints(),
                                    map_old_vars_to_new_vars, &new_prog);
  AddVectorOfConstraintToNewProgram(prog.linear_equality_constraints(),
                                    map_old_vars_to_new_vars, &new_prog);
  AddVectorOfConstraintToNewProgram(prog.bounding_box_constraints(),
                                    map_old_vars_to_new_vars, &new_prog);
  AddVectorOfConstraintToNewProgram(prog.lorentz_cone_constraints(),
                                    map_old_vars_to_new_vars, &new_prog);
  AddVectorOfConstraintToNewProgram(prog.rotated_lorentz_cone_constraints(),
                                    map_old_vars_to_new_vars, &new_prog);
  AddVectorOfConstraintToNewProgram(prog.positive_semidefinite_constraints(),
                                    map_old_vars_to_new_vars, &new_prog);
  AddVectorOfConstraintToNewProgram(prog.linear_matrix_inequality_constraints(),
                                    map_old_vars_to_new_vars, &new_prog);
  AddVectorOfConstraintToNewProgram(prog.linear_complementarity_constraints(),
                                    map_old_vars_to_new_vars, &new_prog);

  // Set the initial guess.
  new_prog.SetInitialGuessForAllVariables(prog.initial_guess());
  const optional<SolverId> prog_solver_id = prog.GetSolverId();
  if (prog_solver_id) {
    new_prog.SetSolverId(prog_solver_id.value());
  }
  // TODO(hongkai.dai) Set the solver options as well.

  MixedIntegerBranchAndBoundNode* node =
      new MixedIntegerBranchAndBoundNode(new_prog, binary_variables);
  return std::make_pair(std::unique_ptr<MixedIntegerBranchAndBoundNode>(node),
                        map_old_vars_to_new_vars);
}
}  // namespace solvers
}  // namespace drake
