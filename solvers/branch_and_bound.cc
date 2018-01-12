#include "drake/solvers/branch_and_bound.h"

#include <algorithm>
#include <limits>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/unused.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/scs_solver.h"

namespace drake {
namespace solvers {
/** Determines if the mathematical program has binary variables.
 */
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
    const std::list<symbolic::Variable>& binary_variables,
    const SolverId& solver_id)
    : prog_{prog.Clone()},
      left_child_{nullptr},
      right_child_{nullptr},
      parent_{nullptr},
      fixed_binary_variable_{},
      fixed_binary_value_{-1},
      remaining_binary_variables_{binary_variables},
      solution_result_{SolutionResult::kUnknownError},
      optimal_solution_is_integral_{OptimalSolutionIsIntegral::kUnknown},
      solver_id_{solver_id} {
  if (solver_id != GurobiSolver::id() && solver_id != ScsSolver::id()) {
    throw std::runtime_error(fmt::format(
        "MixedIntegerBranchAndBoundNode does not support solver {}.",
        solver_id.name()));
  }
  // Check if there are still binary variables.
  DRAKE_ASSERT(!MathProgHasBinaryVariables(*prog_));
  // Set Gurobi DualReductions to 0, to differentiate infeasible from unbounded.
  prog_->SetSolverOption(GurobiSolver::id(), "DualReductions", 0);
}

bool MixedIntegerBranchAndBoundNode::IsRoot() const {
  return parent_ == nullptr;
}

// Replaces the variables bound with the constraint with new variables.
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

// Adds a vector of costs to a mathematical program.
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

// Adds a vector of constraints to a mathematical program.
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
    const MathematicalProgram& prog, const SolverId& solver_id) {
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
  new_prog.AddDecisionVariables(new_vars);
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
  new_prog.AddIndeterminates(prog.indeterminates());

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
  // TODO(hongkai.dai) Set the solver options as well.

  std::list<symbolic::Variable> binary_variables_list;
  for (int i = 0; i < binary_variables.rows(); ++i) {
    binary_variables_list.push_back(binary_variables(i));
  }
  MixedIntegerBranchAndBoundNode* node = new MixedIntegerBranchAndBoundNode(
      new_prog, binary_variables_list, solver_id);
  if (solver_id == GurobiSolver::id()) {
    GurobiSolver gurobi_solver;
    if (!gurobi_solver.available()) {
      throw std::runtime_error("Gurobi is not available.");
    }
    node->solution_result_ = gurobi_solver.Solve(*(node->prog_));
  } else if (solver_id == ScsSolver::id()) {
    ScsSolver scs_solver;
    if (!scs_solver.available()) {
      throw std::runtime_error("Scs is not available.");
    }
    node->solution_result_ = scs_solver.Solve(*(node->prog_));
  }
  if (node->solution_result_ == SolutionResult::kSolutionFound) {
    node->CheckOptimalSolutionIsIntegral();
  }
  return std::make_pair(std::unique_ptr<MixedIntegerBranchAndBoundNode>(node),
                        map_old_vars_to_new_vars);
}

void MixedIntegerBranchAndBoundNode::CheckOptimalSolutionIsIntegral() {
  // Check if the solution to the remaining binary variables are all either
  // 0 or 1.
  if (solution_result_ != SolutionResult::kSolutionFound) {
    throw std::runtime_error("The program does not have an optimal solution.");
  }
  for (const auto& var : remaining_binary_variables_) {
    const double binary_var_val{prog_->GetSolution(var)};
    if (std::isnan(binary_var_val)) {
      throw std::runtime_error(
          "The solution contains NAN, either the problem is not solved "
          "yet, or the problem is infeasible, unbounded, or contains "
          "numerical error.");
    }
    const double integral_tol = 1E-3;
    if (binary_var_val > integral_tol && binary_var_val < 1 - integral_tol) {
      optimal_solution_is_integral_ = OptimalSolutionIsIntegral::kFalse;
      return;
    }
  }
  optimal_solution_is_integral_ = OptimalSolutionIsIntegral::kTrue;
}

bool MixedIntegerBranchAndBoundNode::optimal_solution_is_integral() const {
  if (solution_result_ != SolutionResult::kSolutionFound) {
    throw std::runtime_error("The optimal solution is not found.");
  }
  switch (optimal_solution_is_integral_) {
    case OptimalSolutionIsIntegral::kTrue: {
      return true;
    }
    case OptimalSolutionIsIntegral::kFalse: {
      return false;
    }
    case OptimalSolutionIsIntegral::kUnknown: {
      throw std::runtime_error(
          "Call CheckOptimalSolutionIsIntegral() before calling this "
          "function.");
    }
  }
  // It is impossible to reach this DRAKE_ABORT(), but gcc throws the error
  // Werror=return-type, if we do not have it here.
  DRAKE_ABORT();
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
  left_child_.reset(new MixedIntegerBranchAndBoundNode(
      *prog_, remaining_binary_variables_, solver_id_));
  right_child_.reset(new MixedIntegerBranchAndBoundNode(
      *prog_, remaining_binary_variables_, solver_id_));
  left_child_->FixBinaryVariable(binary_variable, 0);
  right_child_->FixBinaryVariable(binary_variable, 1);
  left_child_->parent_ = this;
  right_child_->parent_ = this;
  if (solver_id_ == GurobiSolver::id()) {
    GurobiSolver gurobi_solver;
    if (!gurobi_solver.available()) {
      throw std::runtime_error("Gurobi is not available.");
    }
    left_child_->solution_result_ = gurobi_solver.Solve(*(left_child_->prog_));
    right_child_->solution_result_ =
        gurobi_solver.Solve(*(right_child_->prog_));
  } else if (solver_id_ == ScsSolver::id()) {
    ScsSolver scs_solver;
    if (!scs_solver.available()) {
      throw std::runtime_error("Scs is not available.");
    }
    left_child_->solution_result_ = scs_solver.Solve(*(left_child_->prog_));
    right_child_->solution_result_ = scs_solver.Solve(*(right_child_->prog_));
  }
  if (left_child_->solution_result_ == SolutionResult::kSolutionFound) {
    left_child_->CheckOptimalSolutionIsIntegral();
  }
  if (right_child_->solution_result_ == SolutionResult::kSolutionFound) {
    right_child_->CheckOptimalSolutionIsIntegral();
  }
}

MixedIntegerBranchAndBound::MixedIntegerBranchAndBound(
    const MathematicalProgram& prog, const SolverId& solver_id)
    : root_{nullptr},
      map_old_vars_to_new_vars_{},
      best_upper_bound_{std::numeric_limits<double>::infinity()},
      best_lower_bound_{-std::numeric_limits<double>::infinity()},
      solutions_{} {
  std::tie(root_, map_old_vars_to_new_vars_) =
      MixedIntegerBranchAndBoundNode::ConstructRootNode(prog, solver_id);
  if (root_->solution_result() == SolutionResult::kSolutionFound) {
    best_lower_bound_ = root_->prog()->GetOptimalCost();
    // If an integral solution is found, then update the best solutions,
    // together with the best upper bound.
    if (root_->optimal_solution_is_integral()) {
      UpdateIntegralSolution(
          root_->prog()->GetSolution(root_->prog()->decision_variables()),
          root_->prog()->GetOptimalCost());
    }
  }
}

SolutionResult MixedIntegerBranchAndBound::Solve() {
  // First check the status of the root node. If the root node is infeasible,
  // then the MIP is infeasible.
  if (root_->solution_result() == SolutionResult::kInfeasibleConstraints) {
    return SolutionResult::kInfeasibleConstraints;
  }
  if (IsConverged()) {
    return SolutionResult::kSolutionFound;
  }
  // If the optimal solution to the root node is not integral, do some
  // post-processing on the non-integral solution, and try to find an integral
  // solution of the MIP (but not necessarily optimal).
  // I did not put this part in the class constructor, where we construct and
  // solve the root node, because the strategy to search for the integral
  // solution can be specified after the constructor call.
  if (root_->solution_result() == SolutionResult::kSolutionFound &&
      !root_->optimal_solution_is_integral()) {
    SearchIntegralSolution(*root_);
  }
  MixedIntegerBranchAndBoundNode* branching_node = PickBranchingNode();
  while (branching_node) {
    // Found a branching node, branch on this node. If no branching node is
    // found, then every leaf node is fathomed, the branch-and-bound process
    // should terminate.
    const symbolic::Variable* branching_variable =
        PickBranchingVariable(*branching_node);
    BranchAndUpdate(branching_node, *branching_variable);
    if (IsConverged()) {
      return SolutionResult::kSolutionFound;
    }
    branching_node = PickBranchingNode();
  }
  // No node to branch.
  if (best_lower_bound_ == -std::numeric_limits<double>::infinity()) {
    return SolutionResult::kUnbounded;
  }
  if (best_lower_bound_ == std::numeric_limits<double>::infinity()) {
    return SolutionResult::kInfeasibleConstraints;
  }
  throw std::runtime_error(
      "Unknown result. The problem is not optimal, infeasible, nor unbounded.");
}

double MixedIntegerBranchAndBound::GetOptimalCost() const {
  if (solutions_.empty()) {
    throw std::runtime_error(
        "The branch-and-bound does not find an optimal solution.");
  }
  return solutions_.front().first;
}

double MixedIntegerBranchAndBound::GetSubOptimalCost(
    int nth_suboptimal_cost) const {
  if (nth_suboptimal_cost < 0 ||
      nth_suboptimal_cost >= static_cast<int>(solutions().size()) - 1) {
    throw std::runtime_error(
        fmt::format("Cannot access {}'th sub-optimal cost. The branch and "
                    "bound process only finds {} solution(s).",
                    nth_suboptimal_cost, solutions().size()));
  }
  auto it = solutions().begin();
  ++it;
  for (int suboptimal_cost_count = 0;
       suboptimal_cost_count < nth_suboptimal_cost; ++suboptimal_cost_count) {
    ++it;
  }
  return it->first;
}

double MixedIntegerBranchAndBound::GetSolution(
    const symbolic::Variable& mip_var, int nth_best_solution) const {
  if (nth_best_solution < 0 ||
      nth_best_solution >= static_cast<int>(solutions().size())) {
    throw std::runtime_error(
        fmt::format("Cannot access {}'th integral solution. The "
                    "branch-and-bound process only finds {} solution(s).",
                    nth_best_solution, solutions().size()));
  }
  const int variable_index =
      root_->prog()->FindDecisionVariableIndex(GetNewVariable(mip_var));

  auto it = solutions().begin();
  for (int best_solution_count = 0; best_solution_count < nth_best_solution;
       ++best_solution_count) {
    ++it;
  }
  return it->second(variable_index);
}

const symbolic::Variable& MixedIntegerBranchAndBound::GetNewVariable(
    const symbolic::Variable& old_variable) const {
  const auto it = map_old_vars_to_new_vars_.find(old_variable.get_id());
  if (it == map_old_vars_to_new_vars_.end()) {
    std::ostringstream oss;
    oss << old_variable
        << " is not a variable in the original mixed-integer problem.\n";
    throw std::runtime_error(oss.str());
  }
  return it->second;
}

MixedIntegerBranchAndBoundNode* MixedIntegerBranchAndBound::PickBranchingNode()
    const {
  switch (pick_node_) {
    case NodeSelectionMethod::kMinLowerBound: {
      return PickMinLowerBoundNode();
    }
    case NodeSelectionMethod::kDepthFirst: {
      return PickDepthFirstNode();
    }
    case NodeSelectionMethod::kUserDefined: {
      if (node_selection_userfun_) {
        return node_selection_userfun_(*root_);
      } else {
        throw std::runtime_error(
            "The user defined function should not be null.");
      }
    }
  }
  // It is impossible to reach this DRAKE_ABORT(), but gcc throws the error
  // Werror=return-type, if we do not have it here.
  DRAKE_ABORT();
}

namespace {
// Pick the non-fathomed leaf node in the tree with the smallest optimal cost.
MixedIntegerBranchAndBoundNode* PickMinLowerBoundNodeInSubTree(
    const MixedIntegerBranchAndBound& bnb,
    const MixedIntegerBranchAndBoundNode& root) {
  if (root.IsLeaf()) {
    if (bnb.IsLeafNodeFathomed(root)) {
      return nullptr;
    }
    return const_cast<MixedIntegerBranchAndBoundNode*>(&root);
  } else {
    MixedIntegerBranchAndBoundNode* left_min_lower_bound_node =
        PickMinLowerBoundNodeInSubTree(bnb, *(root.left_child()));
    MixedIntegerBranchAndBoundNode* right_min_lower_bound_node =
        PickMinLowerBoundNodeInSubTree(bnb, *(root.right_child()));
    if (left_min_lower_bound_node && right_min_lower_bound_node) {
      return left_min_lower_bound_node->prog()->GetOptimalCost() <
                     right_min_lower_bound_node->prog()->GetOptimalCost()
                 ? left_min_lower_bound_node
                 : right_min_lower_bound_node;
    } else if (left_min_lower_bound_node) {
      return left_min_lower_bound_node;
    } else if (right_min_lower_bound_node) {
      return right_min_lower_bound_node;
    }
    return nullptr;
  }
}

MixedIntegerBranchAndBoundNode* PickDepthFirstNodeInSubTree(
    const MixedIntegerBranchAndBound& bnb,
    const MixedIntegerBranchAndBoundNode& root) {
  if (root.IsLeaf()) {
    if (bnb.IsLeafNodeFathomed(root)) {
      return nullptr;
    }
    return const_cast<MixedIntegerBranchAndBoundNode*>(&root);
  } else {
    MixedIntegerBranchAndBoundNode* left_deepest_node =
        PickDepthFirstNodeInSubTree(bnb, *(root.left_child()));
    MixedIntegerBranchAndBoundNode* right_deepest_node =
        PickDepthFirstNodeInSubTree(bnb, *(root.right_child()));
    if (left_deepest_node && right_deepest_node) {
      return left_deepest_node->remaining_binary_variables().size() >
                     right_deepest_node->remaining_binary_variables().size()
                 ? right_deepest_node
                 : left_deepest_node;
    } else if (left_deepest_node) {
      return left_deepest_node;
    } else if (right_deepest_node) {
      return right_deepest_node;
    }
    return nullptr;
  }
}

double BestLowerBoundInSubTree(const MixedIntegerBranchAndBound& bnb,
                               const MixedIntegerBranchAndBoundNode& root) {
  if (root.IsLeaf()) {
    if (bnb.IsLeafNodeFathomed(root)) {
      switch (root.solution_result()) {
        case SolutionResult::kSolutionFound:
          return root.prog()->GetOptimalCost();
        case SolutionResult::kUnbounded:
          return -std::numeric_limits<double>::infinity();
        case SolutionResult::kInfeasibleConstraints:
          return std::numeric_limits<double>::infinity();
        default:
          throw std::runtime_error(
              "Cannot obtain the best lower bound for this fathomed leaf "
              "node.");
      }
    }
    return root.prog()->GetOptimalCost();
  } else {
    const double left_best_lower_bound =
        BestLowerBoundInSubTree(bnb, *(root.left_child()));
    const double right_best_lower_bound =
        BestLowerBoundInSubTree(bnb, *(root.right_child()));
    return left_best_lower_bound < right_best_lower_bound
               ? left_best_lower_bound
               : right_best_lower_bound;
  }
}
}  // namespace

MixedIntegerBranchAndBoundNode*
MixedIntegerBranchAndBound::PickMinLowerBoundNode() const {
  return PickMinLowerBoundNodeInSubTree(*this, *root_);
}

MixedIntegerBranchAndBoundNode* MixedIntegerBranchAndBound::PickDepthFirstNode()
    const {
  // The deepest node has the most number of binary variables being fixed.
  return PickDepthFirstNodeInSubTree(*this, *root_);
}

const symbolic::Variable* MixedIntegerBranchAndBound::PickBranchingVariable(
    const MixedIntegerBranchAndBoundNode& node) const {
  switch (pick_variable_) {
    case VariableSelectionMethod::kMostAmbivalent:
      return PickMostAmbivalentAsBranchingVariable(node);
    case VariableSelectionMethod::kLeastAmbivalent:
      return PickLeastAmbivalentAsBranchingVariable(node);
    case VariableSelectionMethod::kUserDefined:
      if (variable_selection_userfun_) {
        return variable_selection_userfun_(node);
      }
      throw std::runtime_error(
          "The user should specify the function to pick a branching variable "
          "through SetUserDefinedVariableSelectionFunction.");
  }
  // It is impossible to reach this DRAKE_ABORT(), but gcc throws the error
  // Werror=return-type, if we do not have it here.
  DRAKE_ABORT();
}

namespace {
const symbolic::Variable* PickMostOrLeastAmbivalentAsBranchingVariable(
    const MixedIntegerBranchAndBoundNode& node,
    MixedIntegerBranchAndBound::VariableSelectionMethod pick_variable) {
  DRAKE_ASSERT(pick_variable == MixedIntegerBranchAndBound::
                                    VariableSelectionMethod::kMostAmbivalent ||
               pick_variable == MixedIntegerBranchAndBound::
                                    VariableSelectionMethod::kLeastAmbivalent);
  if (node.solution_result() == SolutionResult::kSolutionFound) {
    double value = pick_variable == MixedIntegerBranchAndBound::
                                        VariableSelectionMethod::kMostAmbivalent
                       ? std::numeric_limits<double>::infinity()
                       : -std::numeric_limits<double>::infinity();
    const symbolic::Variable* return_var{nullptr};
    for (const auto& var : node.remaining_binary_variables()) {
      const double var_value = node.prog()->GetSolution(var);
      const double var_value_to_half = std::abs(var_value - 0.5);
      if (pick_variable == MixedIntegerBranchAndBound::VariableSelectionMethod::
                               kMostAmbivalent &&
          var_value_to_half < value) {
        value = var_value_to_half;
        return_var = &var;
      } else if (pick_variable ==
                     MixedIntegerBranchAndBound::VariableSelectionMethod::
                         kLeastAmbivalent &&
                 var_value_to_half > value) {
        value = var_value_to_half;
        return_var = &var;
      }
    }
    return return_var;
  } else if (node.solution_result() == SolutionResult::kUnbounded) {
    return &(node.remaining_binary_variables().front());
  }
  throw std::runtime_error(
      "The problem is neither optimal nor unbounded. Cannot pick a branching "
      "variable.");
}
}  // namespace

const symbolic::Variable*
MixedIntegerBranchAndBound::PickMostAmbivalentAsBranchingVariable(
    const MixedIntegerBranchAndBoundNode& node) const {
  return PickMostOrLeastAmbivalentAsBranchingVariable(
      node,
      MixedIntegerBranchAndBound::VariableSelectionMethod::kMostAmbivalent);
}

const symbolic::Variable*
MixedIntegerBranchAndBound::PickLeastAmbivalentAsBranchingVariable(
    const MixedIntegerBranchAndBoundNode& node) const {
  return PickMostOrLeastAmbivalentAsBranchingVariable(
      node,
      MixedIntegerBranchAndBound::VariableSelectionMethod::kLeastAmbivalent);
}

bool MixedIntegerBranchAndBound::IsLeafNodeFathomed(
    const MixedIntegerBranchAndBoundNode& leaf_node) const {
  if (!leaf_node.IsLeaf()) {
    throw std::runtime_error("Not a leaf node.");
  }
  if (leaf_node.solution_result() == SolutionResult::kInfeasibleConstraints) {
    return true;
  }
  if (leaf_node.prog()->GetOptimalCost() > best_upper_bound_) {
    return true;
  }
  if (leaf_node.solution_result() == SolutionResult::kSolutionFound &&
      leaf_node.optimal_solution_is_integral()) {
    return true;
  }
  if (leaf_node.remaining_binary_variables().empty()) {
    return true;
  }
  return false;
}

void MixedIntegerBranchAndBound::BranchAndUpdate(
    MixedIntegerBranchAndBoundNode* node,
    const symbolic::Variable& branching_variable) {
  node->Branch(branching_variable);
  // Update the best lower and upper bounds.
  // The best lower bound is the minimal among all the optimal costs of the
  // non-fathomed leaf nodes.
  best_lower_bound_ = BestLowerBoundInSubTree(*this, *root_);
  // If either the left or the right children finds integral solution, then
  // we can potentially update the best upper bound, and insert the solutions
  // to the list solutions_;
  if (node->left_child()->solution_result() == SolutionResult::kSolutionFound) {
    if (node->left_child()->optimal_solution_is_integral()) {
      const double child_node_optimal_cost =
          node->left_child()->prog()->GetOptimalCost();
      const Eigen::VectorXd x_sol = node->left_child()->prog()->GetSolution(
          node->left_child()->prog()->decision_variables());
      UpdateIntegralSolution(x_sol, child_node_optimal_cost);
    } else {
      SearchIntegralSolution(*(node->left_child()));
    }
  }
  if (node->right_child()->solution_result() ==
      SolutionResult::kSolutionFound) {
    if (node->right_child()->optimal_solution_is_integral()) {
      const double child_node_optimal_cost =
          node->right_child()->prog()->GetOptimalCost();
      const Eigen::VectorXd x_sol = node->right_child()->prog()->GetSolution(
          node->right_child()->prog()->decision_variables());
      UpdateIntegralSolution(x_sol, child_node_optimal_cost);
    } else {
      SearchIntegralSolution(*(node->right_child()));
    }
  }
}

void MixedIntegerBranchAndBound::UpdateIntegralSolution(
    const Eigen::Ref<const Eigen::VectorXd>& solution, double cost) {
  if (solutions_.empty()) {
    solutions_.emplace_back(cost, solution);
  } else {
    if (cost < solutions_.back().first) {
      // Insert the pair <cost, solution> in the right place in solutions_.
      for (auto it = solutions_.begin(); it != solutions_.end(); ++it) {
        if (it->first > cost) {
          solutions_.emplace(it, cost, solution);
          break;
        }
      }
    } else if (static_cast<int>(solutions_.size()) < max_num_solutions_) {
      solutions_.emplace_back(cost, solution);
    }
    if (static_cast<int>(solutions_.size()) > max_num_solutions_) {
      solutions_.pop_back();
    }
  }
  best_upper_bound_ = std::min(best_upper_bound_, solutions_.begin()->first);
}

bool MixedIntegerBranchAndBound::IsConverged() const {
  if (best_upper_bound_ - best_lower_bound_ <= absolute_gap_tol_) {
    return true;
  }
  if ((best_upper_bound_ - best_lower_bound_) / std::abs(best_lower_bound_) <=
      relative_gap_tol_) {
    return true;
  }
  return false;
}

void MixedIntegerBranchAndBound::SearchIntegralSolution(
    const MixedIntegerBranchAndBoundNode& node) {
  // TODO(hongkai.dai): create a new program that fix the remaining binary
  // variables to either 0 or 1, and solve for the continuous variables. If this
  // optimization problem is feasible, then the optimal solution is a feasible
  // solution to the MIP, and we get an upper bound on the MIP optimal cost.
  unused(node);
}
}  // namespace solvers
}  // namespace drake
