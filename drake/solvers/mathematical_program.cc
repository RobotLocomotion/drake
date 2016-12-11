#include "drake/solvers/mathematical_program.h"

#include "drake/math/matrix_util.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/linear_system_solver.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {

namespace {

// Solver for simple linear systems of equalities
AttributesSet kLinearSystemSolverCapabilities = kLinearEqualityConstraint;

// Solver for equality-constrained QPs
AttributesSet kEqualityConstrainedQPCapabilities =
    (kQuadraticCost | kLinearCost | kLinearEqualityConstraint);

// Solver for Linear Complementarity Problems (LCPs)
AttributesSet kMobyLcpCapabilities = kLinearComplementarityConstraint;

// Gurobi solver capabilities.
AttributesSet kGurobiCapabilities =
    (kLinearEqualityConstraint | kLinearConstraint | kLorentzConeConstraint |
     kRotatedLorentzConeConstraint | kLinearCost | kQuadraticCost |
     kBinaryVariable);

// Mosek solver capabilities.
AttributesSet kMosekCapabilities =
    (kLinearEqualityConstraint | kLinearConstraint | kLorentzConeConstraint |
     kRotatedLorentzConeConstraint | kLinearCost | kQuadraticCost |
     kPositiveSemidefiniteConstraint | kBinaryVariable);

// Solvers for generic systems of constraints and costs.
AttributesSet kGenericSolverCapabilities =
    (kGenericCost | kGenericConstraint | kQuadraticCost | kQuadraticConstraint |
     kLorentzConeConstraint | kRotatedLorentzConeConstraint | kLinearCost |
     kLinearConstraint | kLinearEqualityConstraint);

// Returns true iff no capabilities are in required and not in available.
bool is_satisfied(AttributesSet required, AttributesSet available) {
  return ((required & ~available) == kNoCapabilities);
}

}  // anon namespace

enum {
  INITIAL_VARIABLE_ALLOCATION_NUM = 100
};  // not const static int because the VectorXd constructor takes a reference
// to int so it is odr-used (see
// https://gcc.gnu.org/wiki/VerboseDiagnostics#missing_static_const_definition)

MathematicalProgram::MathematicalProgram()
    : num_vars_(0),
      x_initial_guess_(
          static_cast<Eigen::Index>(INITIAL_VARIABLE_ALLOCATION_NUM)),
      solver_result_(0),
      required_capabilities_(kNoCapabilities),
      ipopt_solver_(new IpoptSolver()),
      nlopt_solver_(new NloptSolver()),
      snopt_solver_(new SnoptSolver()),
      moby_lcp_solver_(new MobyLCPSolver()),
      linear_system_solver_(new LinearSystemSolver()),
      equality_constrained_qp_solver_(new EqualityConstrainedQPSolver()),
      gurobi_solver_(new GurobiSolver()),
      mosek_solver_(new MosekSolver()) {}

DecisionVariableMatrixX MathematicalProgram::AddVariables(
    DecisionVariableScalar::VarType type, int rows, int cols, bool is_symmetric,
    const std::vector<std::string>& names) {
  DecisionVariableMatrixX decision_variable_matrix(rows, cols);
  AddVariables_impl(type, names, is_symmetric, decision_variable_matrix);
  return decision_variable_matrix;
}

DecisionVariableVectorX MathematicalProgram::AddVariables(
    DecisionVariableScalar::VarType type, int rows,
    const std::vector<std::string>& names) {
  return AddVariables(type, rows, 1, false, names);
}

DecisionVariableVectorX MathematicalProgram::AddContinuousVariables(
    std::size_t rows, const std::vector<std::string>& names) {
  return AddVariables(DecisionVariableScalar::VarType::CONTINUOUS, rows, names);
}

const DecisionVariableMatrixX MathematicalProgram::AddContinuousVariables(
    std::size_t rows, std::size_t cols, const std::vector<std::string>& names) {
  return AddVariables(DecisionVariableScalar::VarType::CONTINUOUS, rows, cols,
                      false, names);
}

DecisionVariableVectorX MathematicalProgram::AddContinuousVariables(
    std::size_t rows, const std::string& name) {
  std::vector<std::string> names(rows);
  for (int i = 0; i < static_cast<int>(rows); ++i) {
    names[i] = name + std::to_string(i);
  }
  return AddContinuousVariables(rows, names);
}

const DecisionVariableMatrixX MathematicalProgram::AddContinuousVariables(
    std::size_t rows, std::size_t cols, const std::string& name) {
  std::vector<std::string> names(rows * cols);
  int count = 0;
  for (int j = 0; j < static_cast<int>(cols); ++j) {
    for (int i = 0; i < static_cast<int>(rows); ++i) {
      names[count] =
          name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      ++count;
    }
  }
  return AddContinuousVariables(rows, cols, names);
}

DecisionVariableMatrixX MathematicalProgram::AddBinaryVariables(
    size_t rows, size_t cols, const std::vector<std::string>& names) {
  return AddVariables(DecisionVariableScalar::VarType::BINARY, rows, cols,
                      false, names);
}

DecisionVariableMatrixX MathematicalProgram::AddBinaryVariables(
    size_t rows, size_t cols, const std::string& name) {
  std::vector<std::string> names = std::vector<std::string>(rows * cols);
  for (int i = 0; i < static_cast<int>(names.size()); ++i) {
    names[i] = name + std::to_string(i);
  }
  return AddBinaryVariables(rows, cols, names);
}

DecisionVariableMatrixX MathematicalProgram::AddSymmetricContinuousVariables(
    size_t rows, const std::vector<std::string>& names) {
  return AddVariables(DecisionVariableScalar::VarType::CONTINUOUS, rows, rows,
                      true, names);
}

DecisionVariableMatrixX MathematicalProgram::AddSymmetricContinuousVariables(
    size_t rows, const std::string& name) {
  std::vector<std::string> names(rows * (rows + 1) / 2);
  int count = 0;
  for (int j = 0; j < static_cast<int>(rows); ++j) {
    for (int i = j; i < static_cast<int>(rows); ++i) {
      names[count] =
          name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      ++count;
    }
  }
  return AddVariables(DecisionVariableScalar::VarType::CONTINUOUS, rows, rows,
                      true, names);
}

DecisionVariableVectorX MathematicalProgram::AddBinaryVariables(
    size_t rows, const std::string& name) {
  std::vector<std::string> names = std::vector<std::string>(rows);
  for (int i = 0; i < static_cast<int>(rows); ++i) {
    names[i] = name + std::to_string(i);
  }
  return AddVariables(DecisionVariableScalar::VarType::BINARY, rows, names);
}

void MathematicalProgram::AddCost(const std::shared_ptr<Constraint>& obj,
                                  const VariableListRef& vars) {
  VariableList var_list(vars);
  DRAKE_ASSERT(var_list.column_vectors_only());
  required_capabilities_ |= kGenericCost;
  generic_costs_.push_back(Binding<Constraint>(obj, var_list));
}

void MathematicalProgram::AddCost(const std::shared_ptr<LinearConstraint>& obj,
                                  const VariableListRef& vars) {
  VariableList var_list(vars);
  DRAKE_ASSERT(var_list.column_vectors_only());
  required_capabilities_ |= kLinearCost;
  int var_dim = var_list.size();
  DRAKE_ASSERT(obj->A().rows() == 1 && obj->A().cols() == var_dim);
  linear_costs_.push_back(Binding<LinearConstraint>(obj, var_list));
}

void MathematicalProgram::AddCost(
    const std::shared_ptr<QuadraticConstraint>& obj,
    const VariableListRef& vars) {
  VariableList var_list(vars);
  DRAKE_ASSERT(var_list.column_vectors_only());
  required_capabilities_ |= kQuadraticCost;
  int var_dim = var_list.size();
  DRAKE_ASSERT(obj->Q().rows() == var_dim && obj->b().rows() == var_dim);
  quadratic_costs_.push_back(Binding<QuadraticConstraint>(obj, var_list));
}

void MathematicalProgram::AddConstraint(std::shared_ptr<Constraint> con,
                                        const VariableListRef& vars) {
  required_capabilities_ |= kGenericConstraint;
  generic_constraints_.push_back(Binding<Constraint>(con, vars));
}

void MathematicalProgram::AddConstraint(std::shared_ptr<LinearConstraint> con,
                                        const VariableListRef& vars) {
  VariableList var_list(vars);
  DRAKE_ASSERT(var_list.column_vectors_only());
  required_capabilities_ |= kLinearConstraint;
  int var_dim = var_list.size();
  DRAKE_ASSERT(con->A().cols() == var_dim);
  linear_constraints_.push_back(Binding<LinearConstraint>(con, var_list));
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<LinearEqualityConstraint> con,
    const VariableListRef& vars) {
  VariableList var_list(vars);
  DRAKE_ASSERT(var_list.column_vectors_only());
  required_capabilities_ |= kLinearEqualityConstraint;
  int var_dim = var_list.size();
  DRAKE_ASSERT(con->A().cols() == var_dim);
  linear_equality_constraints_.push_back(
      Binding<LinearEqualityConstraint>(con, var_list));
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<LorentzConeConstraint> con, const VariableListRef& vars) {
  VariableList var_list(vars);
  DRAKE_ASSERT(var_list.column_vectors_only());
  DRAKE_ASSERT(var_list.size() >= 2);
  required_capabilities_ |= kLorentzConeConstraint;
  lorentz_cone_constraint_.push_back(
      Binding<LorentzConeConstraint>(con, var_list));
}

std::shared_ptr<LorentzConeConstraint>
MathematicalProgram::AddLorentzConeConstraint(const VariableListRef& vars) {
  auto constraint = std::make_shared<LorentzConeConstraint>();
  AddConstraint(constraint, vars);
  return constraint;
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<RotatedLorentzConeConstraint> con,
    const VariableListRef& vars) {
  VariableList var_list(vars);
  DRAKE_ASSERT(var_list.column_vectors_only());
  DRAKE_ASSERT(var_list.size() >= 3);
  required_capabilities_ |= kRotatedLorentzConeConstraint;
  rotated_lorentz_cone_constraint_.push_back(
      Binding<RotatedLorentzConeConstraint>(con, var_list));
}

std::shared_ptr<RotatedLorentzConeConstraint>
MathematicalProgram::AddRotatedLorentzConeConstraint(
    const VariableListRef& vars) {
  auto constraint = std::make_shared<RotatedLorentzConeConstraint>();
  AddConstraint(constraint, vars);
  return constraint;
}

std::shared_ptr<Constraint> MathematicalProgram::AddPolynomialConstraint(
    const VectorXPoly& polynomials,
    const std::vector<Polynomiald::VarType>& poly_vars,
    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
    const VariableListRef& vars) {
  // Polynomials that are actually affine (a sum of linear terms + a
  // constant) can be special-cased.  Other polynomials are treated as
  // generic for now.
  // TODO(ggould-tri) There may be other such special easy cases.
  VariableList var_list(vars);
  DRAKE_ASSERT(var_list.column_vectors_only());
  bool all_affine = true;
  for (int i = 0; i < polynomials.rows(); i++) {
    if (!polynomials[i].IsAffine()) {
      all_affine = false;
      break;
    }
  }
  if (all_affine) {
    Eigen::MatrixXd linear_constraint_matrix =
        Eigen::MatrixXd::Zero(polynomials.rows(), poly_vars.size());
    Eigen::VectorXd linear_constraint_lb = lb;
    Eigen::VectorXd linear_constraint_ub = ub;
    for (int poly_num = 0; poly_num < polynomials.rows(); poly_num++) {
      for (const auto& monomial : polynomials[poly_num].GetMonomials()) {
        if (monomial.terms.size() == 0) {
          linear_constraint_lb[poly_num] -= monomial.coefficient;
          linear_constraint_ub[poly_num] -= monomial.coefficient;
        } else if (monomial.terms.size() == 1) {
          const Polynomiald::VarType term_var = monomial.terms[0].var;
          int var_num =
              (std::find(poly_vars.begin(), poly_vars.end(), term_var) -
               poly_vars.begin());
          DRAKE_ASSERT(var_num < static_cast<int>(poly_vars.size()));
          linear_constraint_matrix(poly_num, var_num) = monomial.coefficient;
        } else {
          DRAKE_ABORT();  // Can't happen (unless isAffine() lied to us).
        }
      }
    }
    if (ub == lb) {
      auto constraint = std::make_shared<LinearEqualityConstraint>(
          linear_constraint_matrix, linear_constraint_ub);
      AddConstraint(constraint, vars);
      return constraint;
    } else {
      auto constraint = std::make_shared<LinearConstraint>(
          linear_constraint_matrix, linear_constraint_lb, linear_constraint_ub);
      AddConstraint(constraint, vars);
      return constraint;
    }
  } else {
    auto constraint =
        std::make_shared<PolynomialConstraint>(polynomials, poly_vars, lb, ub);
    AddConstraint(constraint, vars);
    return constraint;
  }
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<PositiveSemidefiniteConstraint> con,
    const Eigen::Ref<const DecisionVariableMatrixX> symmetric_matrix_var) {
  required_capabilities_ |= kPositiveSemidefiniteConstraint;
  DRAKE_ASSERT(drake::math::IsSymmetric(symmetric_matrix_var));
  positive_semidefinite_constraint_.push_back(
      Binding<PositiveSemidefiniteConstraint>(con, {symmetric_matrix_var}));
}

std::shared_ptr<PositiveSemidefiniteConstraint>
MathematicalProgram::AddPositiveSemidefiniteConstraint(
    const Eigen::Ref<const DecisionVariableMatrixX> symmetric_matrix_var) {
  auto constraint = std::make_shared<PositiveSemidefiniteConstraint>(
      symmetric_matrix_var.rows());
  AddConstraint(constraint, symmetric_matrix_var);
  return constraint;
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<LinearMatrixInequalityConstraint> con,
    const VariableListRef& vars) {
  required_capabilities_ |= kPositiveSemidefiniteConstraint;
  VariableList var_list(vars);
  DRAKE_ASSERT(var_list.column_vectors_only());
  DRAKE_ASSERT(con->F().size() == var_list.size() + 1);
  linear_matrix_inequality_constraint_.push_back(
      Binding<LinearMatrixInequalityConstraint>(con, var_list));
}

std::shared_ptr<LinearMatrixInequalityConstraint>
MathematicalProgram::AddLinearMatrixInequalityConstraint(
    const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
    const VariableListRef& vars) {
  auto constraint = std::make_shared<LinearMatrixInequalityConstraint>(F);
  AddConstraint(constraint, vars);
  return constraint;
}

SolutionResult MathematicalProgram::Solve() {
  // This implementation is simply copypasta for now; in the future we will
  // want to tweak the order of preference of solvers based on the types of
  // constraints present.

  if (is_satisfied(required_capabilities_, kLinearSystemSolverCapabilities) &&
      linear_system_solver_->available()) {
    // TODO(ggould-tri) Also allow quadratic objectives whose matrix is
    // Identity: This is the objective function the solver uses anyway when
    // underconstrainted, and is fairly common in real-world problems.
    return linear_system_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_,
                          kEqualityConstrainedQPCapabilities) &&
             equality_constrained_qp_solver_->available()) {
    return equality_constrained_qp_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kMosekCapabilities) &&
             mosek_solver_->available()) {
    // TODO(hongkai.dai@tri.global): based on my limited experience, Mosek is
    // faster than Gurobi for convex optimization problem. But we should run
    // a more thorough comparison.
    return mosek_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGurobiCapabilities) &&
             gurobi_solver_->available()) {
    return gurobi_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kMobyLcpCapabilities) &&
             moby_lcp_solver_->available()) {
    return moby_lcp_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             snopt_solver_->available()) {
    return snopt_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             ipopt_solver_->available()) {
    return ipopt_solver_->Solve(*this);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             nlopt_solver_->available()) {
    return nlopt_solver_->Solve(*this);
  } else {
    throw std::runtime_error(
        "MathematicalProgram::Solve: "
        "No solver available for the given optimization problem!");
  }
}

}  // namespace solvers
}  // namespace drake
