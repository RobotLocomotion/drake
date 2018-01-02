#include "drake/solvers/branch_and_bound.h"

#include <algorithm>

#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace solvers {
bool MathProgHasBinaryVariables(const MathematicalProgram& prog) {
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
    const std::list<symbolic::Variable>& binary_variables)
    : prog_{prog.Clone()},
      left_child_{nullptr},
      right_child_{nullptr},
      parent_{nullptr},
      fixed_binary_variable_{},
      fixed_binary_value_{-1},
      remaining_binary_variables_{binary_variables},
      optimal_solution_is_integral_{OptimalSolutionIsIntegral::kUnknown} {
  // Check if there are still binary variables.
  DRAKE_ASSERT(!MathProgHasBinaryVariables(*prog_));
  // Set Gurobi DualReductions to 0, to differentiate infeasible from unbounded.
  prog_->SetSolverOption(GurobiSolver::id(), "DualReductions", 0);
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

  std::list<symbolic::Variable> binary_variables_list;
  for (int i = 0; i < binary_variables.rows(); ++i) {
    binary_variables_list.push_back(binary_variables(i));
  }
  MixedIntegerBranchAndBoundNode* node =
      new MixedIntegerBranchAndBoundNode(new_prog, binary_variables_list);
  return std::make_pair(std::unique_ptr<MixedIntegerBranchAndBoundNode>(node),
                        map_old_vars_to_new_vars);
}

bool MixedIntegerBranchAndBoundNode::IsOptimalSolutionIntegral() {
  switch (optimal_solution_is_integral_) {
    case OptimalSolutionIsIntegral::kTrue: {
      return true;
    }
    case OptimalSolutionIsIntegral::kFalse: {
      return false;
    }
    case OptimalSolutionIsIntegral::kUnknown: {
      // Check if the solution to the remaining binary variables are all either
      // 0 or 1.
      for (const auto& var : remaining_binary_variables_) {
        const double binary_var_val{prog_->GetSolution(var)};
        if (std::isnan(binary_var_val)) {
          throw std::runtime_error(
              "The solution contains NAN, either the problem is not solved "
              "yet, or the problem is infeasible, unbounded, or contains "
              "numerical error.");
        }
        const double integral_tol = 1E-3;
        if (binary_var_val > integral_tol &&
            binary_var_val < 1 - integral_tol) {
          optimal_solution_is_integral_ = OptimalSolutionIsIntegral::kFalse;
          return false;
        }
      }
      optimal_solution_is_integral_ = OptimalSolutionIsIntegral::kTrue;
      return true;
    }
  }
}

bool IsVariableInList(const std::list<symbolic::Variable>& binary_variable_list,
                      const symbolic::Variable& binary_variable) {
  for (const auto& var : binary_variable_list) {
    if (var.equal_to(binary_variable)) {
      return true;
    }
  }
  return false;
}

void MixedIntegerBranchAndBoundNode::FixBinaryVariable(
    const symbolic::Variable& binary_variable, int binary_value) {
  DRAKE_ASSERT(binary_value == 0 || binary_value == 1);
  // Add constraint y == 0 or y == 1.
  prog_->AddBoundingBoxConstraint(binary_value, binary_value, binary_variable);
  // Remove binary_variable from remaining_binary_variables_
  bool found_binary_variable = false;
  for (auto it = remaining_binary_variables_.begin();
       it != remaining_binary_variables_.end(); ++it) {
    if (it->equal_to(binary_variable)) {
      found_binary_variable = true;
      remaining_binary_variables_.erase(it);
      break;
    }
  }
  if (!found_binary_variable) {
    std::ostringstream oss;
    oss << binary_variable
        << " is not a remaining binary variable in this node.\n";
    throw std::runtime_error(oss.str());
  }
  // Set fixed_binary_variable_ and fixed_binary_value_.
  fixed_binary_variable_ = binary_variable;
  fixed_binary_value_ = binary_value;
}

void MixedIntegerBranchAndBoundNode::Branch(
    const symbolic::Variable& binary_variable) {
  left_child_.reset(
      new MixedIntegerBranchAndBoundNode(*prog_, remaining_binary_variables_));
  right_child_.reset(
      new MixedIntegerBranchAndBoundNode(*prog_, remaining_binary_variables_));
  left_child_->FixBinaryVariable(binary_variable, 0);
  right_child_->FixBinaryVariable(binary_variable, 1);
  left_child_->parent_ = this;
  right_child_->parent_ = this;
}

MixedIntegerBranchAndBound::MixedIntegerBranchAndBound(const MathematicalProgram& prog) 
  : root_{nullptr},
    map_old_vars_to_new_vars_{},
    best_upper_bound_{std::numeric_limits<double>::infinity()},
    best_lower_bound_{-std::numeric_limits<double>::infinity()},
    active_leaves_{}{
    std::tie(root_, map_old_vars_to_new_vars_) = MixedIntegerBranchAndBoundNode::ConstructRootNode(prog);
}

const symbolic::Variable& MixedIntegerBranchAndBound::NewVariable(const symbolic::Variable& old_variable) const {
  const auto it = map_old_vars_to_new_vars_.find(old_variable.get_id());
  if (it == map_old_vars_to_new_vars_.end()) {
    std::ostringstream oss;
    oss << old_variable << " is not a variable in the original mixed-integer problem.\n";
    throw std::runtime_error(oss.str());
  }
  return it->second;
}

MixedIntegerBranchAndBoundNode* MixedIntegerBranchAndBound::PickBranchingNode() const {
  // If active_leaves is empty, then the branch-and-bound should terminate, no need to branch further.
  DRAKE_ASSERT(!active_leaves_.empty());
  switch (pick_node_) {
    case PickNode::kMinLowerBound: {
       return PickMinLowerBoundNode();
     }
    case PickNode::kDepthFirst: {
       return PickDepthFirstNode();
     }
    case PickNode::kUserDefined: {
       return pick_branching_node_userfun_(*root_);
     }
  }
}

MixedIntegerBranchAndBoundNode* MixedIntegerBranchAndBound::PickMinLowerBoundNode() {
  return *(std::min_element(active_leaves_.begin(), active_leaves_.end(), [](ScsNode* node1, ScsNode* node2) { return node1->prog()->GetOptimalCost() < node2->prog()->GetOptimalCost(); }));
}
}  // namespace solvers
}  // namespace drake
