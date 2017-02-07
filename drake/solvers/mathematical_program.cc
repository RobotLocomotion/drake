#include "drake/solvers/mathematical_program.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/symbolic_expression.h"
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

using std::map;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::shared_ptr;
using std::string;
using std::unordered_map;
using std::vector;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;

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

MatrixXDecisionVariable MathematicalProgram::NewVariables(
    VarType type, int rows, int cols, bool is_symmetric,
    const std::vector<std::string>& names) {
  MatrixXDecisionVariable decision_variable_matrix(rows, cols);
  NewVariables_impl(type, names, is_symmetric, decision_variable_matrix);
  return decision_variable_matrix;
}

VectorXDecisionVariable MathematicalProgram::NewVariables(
    VarType type, int rows, const std::vector<std::string>& names) {
  return NewVariables(type, rows, 1, false, names);
}

VectorXDecisionVariable MathematicalProgram::NewContinuousVariables(
    std::size_t rows, const std::vector<std::string>& names) {
  return NewVariables(VarType::CONTINUOUS, rows, names);
}

MatrixXDecisionVariable MathematicalProgram::NewContinuousVariables(
    size_t rows, size_t cols, const std::vector<std::string>& names) {
  return NewVariables(VarType::CONTINUOUS, rows, cols, false, names);
}

VectorXDecisionVariable MathematicalProgram::NewContinuousVariables(
    size_t rows, const std::string& name) {
  std::vector<std::string> names(rows);
  for (int i = 0; i < static_cast<int>(rows); ++i) {
    names[i] = name + "(" + std::to_string(i) + ")";
  }
  return NewContinuousVariables(rows, names);
}

MatrixXDecisionVariable MathematicalProgram::NewContinuousVariables(
    size_t rows, size_t cols, const std::string& name) {
  std::vector<std::string> names(rows * cols);
  int count = 0;
  for (int j = 0; j < static_cast<int>(cols); ++j) {
    for (int i = 0; i < static_cast<int>(rows); ++i) {
      names[count] =
          name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      ++count;
    }
  }
  return NewContinuousVariables(rows, cols, names);
}

MatrixXDecisionVariable MathematicalProgram::NewBinaryVariables(
    size_t rows, size_t cols, const std::vector<std::string>& names) {
  return NewVariables(VarType::BINARY, rows, cols, false, names);
}

MatrixXDecisionVariable MathematicalProgram::NewBinaryVariables(
    size_t rows, size_t cols, const std::string& name) {
  std::vector<std::string> names = std::vector<std::string>(rows * cols);
  int count = 0;
  for (int j = 0; j < static_cast<int>(cols); ++j) {
    for (int i = 0; i < static_cast<int>(rows); ++i) {
      names[count] =
          name + "(" + std::to_string(i) + "," + std::to_string(j) + ")";
      ++count;
    }
  }
  return NewBinaryVariables(rows, cols, names);
}

MatrixXDecisionVariable MathematicalProgram::NewSymmetricContinuousVariables(
    size_t rows, const std::vector<std::string>& names) {
  return NewVariables(VarType::CONTINUOUS, rows, rows, true, names);
}

MatrixXDecisionVariable MathematicalProgram::NewSymmetricContinuousVariables(
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
  return NewVariables(VarType::CONTINUOUS, rows, rows, true, names);
}

VectorXDecisionVariable MathematicalProgram::NewBinaryVariables(
    size_t rows, const std::string& name) {
  std::vector<std::string> names = std::vector<std::string>(rows);
  for (int i = 0; i < static_cast<int>(rows); ++i) {
    names[i] = name + "(" + std::to_string(i) + ")";
  }
  return NewVariables(VarType::BINARY, rows, names);
}

namespace {

class SymbolicError : public runtime_error {
 public:
  SymbolicError(const Expression& e, const string& msg)
      : runtime_error{make_string(e, msg)} {}
  SymbolicError(const Expression& e, const double lb, const double ub,
                const string& msg)
      : runtime_error{make_string(e, lb, ub, msg)} {}

 private:
  static string make_string(const Expression& e, const string& msg) {
    ostringstream oss;
    oss << "Constraint " << e << " is " << msg << ".";
    return oss.str();
  }
  static string make_string(const Expression& e, const double lb,
                            const double ub, const string& msg) {
    ostringstream oss;
    oss << "Constraint " << lb << " <= " << e << " <= " << ub << " is " << msg
        << ".";
    return oss.str();
  }
};

// Given an expression @p e, extract all variables inside e, append these
// variables to @p vars if they are not included in @p vars yet.
// @param[in] e.  A symbolic expression.
// @param[out] vars.  The variables in @p e but not included in @p vars before
// this call, will be appended to @p vars.
// @param[out] map_var_to_index. map_var_to_index is of the same size as @p
// vars, and map_var_to_index[vars(i).get_id()] = i.
void ExtractVariablesFromExpression(
    const Expression& e, VectorXDecisionVariable* vars,
    unordered_map<Variable::Id, int>* map_var_to_index) {
  DRAKE_DEMAND(static_cast<int>(map_var_to_index->size()) == vars->size());
  for (const Variable& var : e.GetVariables()) {
    if (map_var_to_index->find(var.get_id()) == map_var_to_index->end()) {
      map_var_to_index->emplace(var.get_id(), vars->size());
      vars->conservativeResize(vars->size() + 1, Eigen::NoChange);
      (*vars)(vars->size() - 1) = var;
    }
  }
}

/** Decomposes a linear combination @p e = c0 + c1 * v1 + ... cn * vn into
   *  the followings:
   *
   *     constant term      : c0
   *     coefficient vector : [c1, ..., cn]
   *     variable vector    : [v1, ..., vn]
   *
   *  Then, it returns a pair (c0, [c1, ..., cn]). A map from variable ID to
   *  int, @p map_var_to_index, is used to decide a variable's index in a linear
   *  combination.
   *
   *  \pre{@c coeffs is a row vector of double, whose length matches with the
   *          size of @c map_var_to_index.}
   * @tparam Derived An Eigen row vector of doubles.
   * @param[in] e The symbolic linear expression
   * @param[in] map_var_to_index A mapping from variable ID to variable index,
   * such that map_var_to_index[vi.get_ID()] = i.
   * @param[out] coeffs A row vector. coeffs(i) = ci.
   * @param[out] constant_term c0 in the equation above.
   */
template <typename Derived>
void DecomposeLinearExpression(
    const Expression& e,
    const std::unordered_map<Variable::Id, int>& map_var_to_index,
    const Eigen::MatrixBase<Derived>& coeffs, double* constant_term) {
  static_assert(std::is_same<typename Derived::Scalar, double>::value,
                "coeffs must be a matrix of double.");
  DRAKE_DEMAND(coeffs.rows() == 1);
  DRAKE_DEMAND(coeffs.cols() == static_cast<int>(map_var_to_index.size()));
  if (is_addition(e)) {
    *constant_term = get_constant_in_addition(e);
    const std::map<Expression, double>& expr_to_coeff_map{
        get_expr_to_coeff_map_in_addition(e)};
    for (const pair<Expression, double>& p : expr_to_coeff_map) {
      if (is_variable(p.first)) {
        const Variable& var{get_variable(p.first)};
        const double coeff{p.second};
        const_cast<Eigen::MatrixBase<Derived>&>(coeffs)(
            0, map_var_to_index.at(var.get_id())) = coeff;
      } else {
        std::ostringstream oss;
        oss << "Expression " << e << " is non-linear.";
        throw std::runtime_error(oss.str());
      }
    }
  } else if (is_multiplication(e)) {
    const double c = get_constant_in_multiplication(e);
    const std::map<Expression, Expression>& map_base_to_exponent =
        get_base_to_exponent_map_in_multiplication(e);
    if (map_base_to_exponent.size() == 1) {
      const pair<Expression, Expression>& p = *map_base_to_exponent.begin();
      if (!is_variable(p.first) || !is_one(p.second)) {
        throw SymbolicError(e, "is not linear");
      } else {
        const Variable& var = get_variable(p.first);
        const_cast<Eigen::MatrixBase<Derived>&>(coeffs)(
            map_var_to_index.at(var.get_id())) = c;
        *constant_term = 0;
      }
    } else {
      // There are more than one base, like x^2 * y^3
      throw SymbolicError(e, "is not linear");
    }
  } else if (is_variable(e)) {
    // Just a single variable.
    const Variable& var{get_variable(e)};
    const_cast<Eigen::MatrixBase<Derived>&>(coeffs)(
        map_var_to_index.at(var.get_id())) = 1;
    *constant_term = 0;
  } else if (is_constant(e)) {
    *constant_term = get_constant_value(e);
  } else {
    throw SymbolicError(e, "is not linear");
  }
}

/**
 * Given a vector of linear expressions v, decompose it to
 * \f$ v = A vars + b \f$
 * @param[in] v A vector of linear expressions
 * @param[out] A The matrix containing the linear coefficients.
 * @param[out] b The vector containing all the constant terms.
 * @param[out] vars All variables.
 */
void DecomposeLinearExpression(const Eigen::Ref<const VectorX<Expression>>& v,
                               Eigen::MatrixXd* A, Eigen::VectorXd* b,
                               VectorXDecisionVariable* vars) {
  // 0. Setup map_var_to_index and var_vec.
  unordered_map<Variable::Id, int> map_var_to_index;
  for (int i = 0; i < v.size(); ++i) {
    ExtractVariablesFromExpression(v(i), vars, &map_var_to_index);
  }

  // 2. Construct decompose v as
  // v = A * vars + b
  *A = Eigen::MatrixXd::Zero(v.rows(), vars->rows());
  *b = Eigen::VectorXd::Zero(v.rows());
  for (int i{0}; i < v.size(); ++i) {
    const Expression& e_i{v(i)};
    DecomposeLinearExpression(e_i, map_var_to_index, A->row(i), b->data() + i);
  }
}
}  // anonymous namespace

void MathematicalProgram::AddCost(const Binding<Constraint>& binding) {
  required_capabilities_ |= kGenericCost;
  generic_costs_.push_back(binding);
}

void MathematicalProgram::AddCost(
    const std::shared_ptr<Constraint>& obj,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddCost(Binding<Constraint>(obj, vars));
}

void MathematicalProgram::AddCost(const Binding<LinearConstraint>& binding) {
  required_capabilities_ |= kLinearCost;
  DRAKE_ASSERT(binding.constraint()->num_constraints() == 1 &&
               binding.constraint()->A().cols() ==
                   static_cast<int>(binding.GetNumElements()));
  linear_costs_.push_back(binding);
}

void MathematicalProgram::AddCost(
    const std::shared_ptr<LinearConstraint>& obj,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddCost(Binding<LinearConstraint>(obj, vars));
}

Binding<LinearConstraint> MathematicalProgram::AddLinearCost(
    const Expression& e) {
  VectorXDecisionVariable var(0);
  unordered_map<Variable::Id, int> map_var_to_index;
  ExtractVariablesFromExpression(e, &var, &map_var_to_index);
  DRAKE_ASSERT(IsDecisionVariable(var));
  Eigen::RowVectorXd c(var.size());
  double constant_term;
  DecomposeLinearExpression(e, map_var_to_index, c, &constant_term);
  // The constant term is ignored now.
  // TODO(hongkai.dai): support adding constant term to the cost.
  return Binding<LinearConstraint>(AddLinearCost(c, var), var);
}

std::shared_ptr<LinearConstraint> MathematicalProgram::AddLinearCost(
    const Eigen::Ref<const Eigen::VectorXd>& c,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto cost = std::make_shared<LinearConstraint>(
      c.transpose(), drake::Vector1<double>::Constant(
                         -std::numeric_limits<double>::infinity()),
      drake::Vector1<double>::Constant(
          std::numeric_limits<double>::infinity()));
  AddCost(cost, vars);
  return cost;
}

void MathematicalProgram::AddCost(const Binding<QuadraticConstraint>& binding) {
  required_capabilities_ |= kQuadraticCost;
  DRAKE_ASSERT(binding.constraint()->Q().rows() ==
                   static_cast<int>(binding.GetNumElements()) &&
               binding.constraint()->b().rows() ==
                   static_cast<int>(binding.GetNumElements()));
  quadratic_costs_.push_back(binding);
}

void MathematicalProgram::AddCost(
    const std::shared_ptr<QuadraticConstraint>& obj,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddCost(Binding<QuadraticConstraint>(obj, vars));
}

std::shared_ptr<QuadraticConstraint> MathematicalProgram::AddQuadraticErrorCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& x_desired,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto cost = std::make_shared<QuadraticConstraint>(
      2 * Q, -2 * Q * x_desired, -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity());
  AddCost(cost, vars);
  return cost;
}

std::shared_ptr<QuadraticConstraint> MathematicalProgram::AddQuadraticCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto cost = std::make_shared<QuadraticConstraint>(
      Q, b, -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity());
  AddCost(cost, vars);
  return cost;
}

void MathematicalProgram::AddConstraint(const Binding<Constraint>& binding) {
  required_capabilities_ |= kGenericConstraint;
  generic_constraints_.push_back(binding);
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Expression& e, const double lb, const double ub) {
  return AddLinearConstraint(drake::Vector1<Expression>(e),
                             drake::Vector1<double>(lb),
                             drake::Vector1<double>(ub));
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Eigen::Ref<const drake::VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub) {
  DRAKE_ASSERT(v.rows() == lb.rows() && v.rows() == ub.rows());

  // Setup map_var_to_index and var_vec.
  // such that map_var_to_index[var(i)] = i
  unordered_map<Variable::Id, int> map_var_to_index;
  VectorXDecisionVariable vars(0);
  for (int i = 0; i < v.size(); ++i) {
    ExtractVariablesFromExpression(v(i), &vars, &map_var_to_index);
  }
  DRAKE_ASSERT(IsDecisionVariable(vars));

  // Construct A, new_lb, new_ub. map_var_to_index is used here.
  Eigen::MatrixXd A{Eigen::MatrixXd::Zero(v.size(), vars.size())};
  Eigen::VectorXd new_lb{v.size()};
  Eigen::VectorXd new_ub{v.size()};
  for (int i{0}; i < v.size(); ++i) {
    const Expression& e_i{v(i)};
    const double lb_i{lb(i)};
    const double ub_i{ub(i)};
    if (is_addition(e_i)) {
      double constant_term{0.0};
      DecomposeLinearExpression(e_i, map_var_to_index, A.row(i),
                                &constant_term);
      new_lb(i) = lb(i) - constant_term;
      new_ub(i) = ub(i) - constant_term;
    } else if (is_multiplication(e_i)) {
      // i-th constraint should be lb <= c * var_i <= ub, where c is a constant.
      const double c = get_constant_in_multiplication(e_i);
      const std::map<Expression, Expression>& map_base_to_exponent =
          get_base_to_exponent_map_in_multiplication(e_i);
      if (map_base_to_exponent.size() == 1) {
        const pair<Expression, Expression>& p = *map_base_to_exponent.begin();
        if (!is_variable(p.first) || !is_one(p.second)) {
          throw SymbolicError(e_i,
                              "non-linear but called with AddLinearConstraint");
        } else {
          const Variable& var_i = get_variable(p.first);
          if (v.size() == 1) {
            // Add a bounding box constraint lb/c <= v <= ub/c
            if (c > 0) {
              new_lb(i) = lb(i) / c;
              new_ub(i) = ub(i) / c;
            } else {
              new_lb(i) = ub(i) / c;
              new_ub(i) = lb(i) / c;
            }
            return Binding<BoundingBoxConstraint>(
                AddBoundingBoxConstraint(new_lb(i), new_ub(i), var_i), vars);
          } else {
            A(i, map_var_to_index[var_i.get_id()]) = c;
            new_lb(i) = lb(i);
            new_ub(i) = ub(i);
          }
        }
      } else {
        // There is more than one base, like x^2*y^3
        throw SymbolicError(e_i,
                            "non-linear but called with AddLinearConstraint");
      }
    } else if (is_variable(e_i)) {
      // i-th constraint is lb <= var_i <= ub
      const Variable& var_i{get_variable(e_i)};
      if (v.size() == 1) {
        // If this is the only constraint, we call AddBoundingBoxConstraint.
        return Binding<BoundingBoxConstraint>{
            AddBoundingBoxConstraint(lb(i), ub(i), var_i), vars};
      } else {
        A(i, map_var_to_index[var_i.get_id()]) = 1;
        new_lb(i) = lb(i);
        new_ub(i) = ub(i);
      }
    } else if (is_constant(e_i)) {
      const double c_i{get_constant_value(e_i)};
      if (lb_i <= c_i && c_i <= ub_i) {
        throw SymbolicError(e_i, lb_i, ub_i,
                            "trivial but called with AddLinearConstraint");
      } else {
        throw SymbolicError(
            e_i, lb_i, ub_i,
            "unsatisfiable but called with AddLinearConstraint");
      }
    } else {
      throw SymbolicError(e_i, lb_i, ub_i,
                          "non-linear but called with AddLinearConstraint");
    }
  }
  return Binding<LinearConstraint>{AddLinearConstraint(A, new_lb, new_ub, vars),
                                   vars};
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Formula& f) {
  if (is_equal_to(f)) {
    // e1 == e2
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return AddLinearEqualityConstraint(e1 - e2, 0.0);
  } else if (is_greater_than_or_equal_to(f)) {
    // e1 >= e2  ->  e1 - e2 >= 0  ->  0 <= e1 - e2 <= ∞
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return AddLinearConstraint(e1 - e2, 0.0,
                               std::numeric_limits<double>::infinity());
  } else if (is_less_than_or_equal_to(f)) {
    // e1 <= e2  ->  0 <= e2 - e1  ->  0 <= e2 - e1 <= ∞
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return AddLinearConstraint(e2 - e1, 0.0,
                               std::numeric_limits<double>::infinity());
  }
  ostringstream oss;
  oss << "MathematicalProgram::AddLinearConstraint is called with a formula "
      << f << " which is not a relational formula using one of {==, <=, >=} "
              "operators.";
  throw runtime_error(oss.str());
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<Constraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddConstraint(Binding<Constraint>(con, vars));
}

void MathematicalProgram::AddConstraint(
    const Binding<LinearConstraint>& binding) {
  required_capabilities_ |= kLinearConstraint;
  DRAKE_ASSERT(binding.constraint()->A().cols() ==
               static_cast<int>(binding.GetNumElements()));
  linear_constraints_.push_back(binding);
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<LinearConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddConstraint(Binding<LinearConstraint>(con, vars));
}

std::shared_ptr<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  std::shared_ptr<LinearConstraint> con =
      std::make_shared<LinearConstraint>(A, lb, ub);
  AddConstraint(Binding<LinearConstraint>(con, vars));
  return con;
}

void MathematicalProgram::AddConstraint(
    const Binding<LinearEqualityConstraint>& binding) {
  required_capabilities_ |= kLinearEqualityConstraint;
  DRAKE_ASSERT(binding.constraint()->A().cols() ==
               static_cast<int>(binding.GetNumElements()));
  linear_equality_constraints_.push_back(binding);
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<LinearEqualityConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(Binding<LinearEqualityConstraint>(con, vars));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(const Expression& e,
                                                 double b) {
  return AddLinearEqualityConstraint(drake::Vector1<symbolic::Expression>(e),
                                     drake::Vector1d(b));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::DoAddLinearEqualityConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  DRAKE_DEMAND(v.rows() == b.rows());
  VectorXDecisionVariable vars(0);
  unordered_map<Variable::Id, int> map_var_to_index;
  for (int i = 0; i < v.rows(); ++i) {
    ExtractVariablesFromExpression(v(i), &vars, &map_var_to_index);
  }
  // TODO(hongkai.dai): use sparse matrix.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(v.rows(), vars.rows());
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(v.rows());
  for (int i = 0; i < v.rows(); ++i) {
    double constant_term(0);
    DecomposeLinearExpression(v(i), map_var_to_index, A.row(i), &constant_term);
    beq(i) = b(i) - constant_term;
  }
  return Binding<LinearEqualityConstraint>(
      AddLinearEqualityConstraint(A, beq, vars), vars);
}

std::shared_ptr<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
    const Eigen::Ref<const Eigen::VectorXd>& beq,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  std::shared_ptr<LinearEqualityConstraint> constraint =
      std::make_shared<LinearEqualityConstraint>(Aeq, beq);
  AddConstraint(Binding<LinearEqualityConstraint>(constraint, vars));
  return constraint;
}

void MathematicalProgram::AddConstraint(
    const Binding<BoundingBoxConstraint>& binding) {
  required_capabilities_ |= kLinearConstraint;
  DRAKE_ASSERT(binding.constraint()->num_constraints() ==
               binding.GetNumElements());
  bbox_constraints_.push_back(binding);
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<BoundingBoxConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddConstraint(Binding<BoundingBoxConstraint>(con, vars));
}

void MathematicalProgram::AddConstraint(
    const Binding<LorentzConeConstraint>& binding) {
  required_capabilities_ |= kLorentzConeConstraint;
  lorentz_cone_constraint_.push_back(binding);
}

Binding<LorentzConeConstraint> MathematicalProgram::AddLorentzConeConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v) {
  DRAKE_DEMAND(v.rows() >= 2);
  Eigen::MatrixXd A{};
  Eigen::VectorXd b(v.size());
  VectorXDecisionVariable vars{};
  DecomposeLinearExpression(v, &A, &b, &vars);
  DRAKE_ASSERT(IsDecisionVariable(vars));
  DRAKE_DEMAND(vars.rows() >= 1);
  return Binding<LorentzConeConstraint>(AddLorentzConeConstraint(A, b, vars),
                                        vars);
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<LorentzConeConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(Binding<LorentzConeConstraint>(con, vars));
}

std::shared_ptr<LorentzConeConstraint>
MathematicalProgram::AddLorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  std::shared_ptr<LorentzConeConstraint> constraint =
      std::make_shared<LorentzConeConstraint>(A, b);
  AddConstraint(Binding<LorentzConeConstraint>(constraint, vars));
  return constraint;
}

void MathematicalProgram::AddConstraint(
    const Binding<RotatedLorentzConeConstraint>& binding) {
  required_capabilities_ |= kRotatedLorentzConeConstraint;
  rotated_lorentz_cone_constraint_.push_back(binding);
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<RotatedLorentzConeConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddConstraint(Binding<RotatedLorentzConeConstraint>(con, vars));
}

Binding<RotatedLorentzConeConstraint>
MathematicalProgram::AddRotatedLorentzConeConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v) {
  DRAKE_DEMAND(v.rows() >= 3);
  Eigen::MatrixXd A{};
  Eigen::VectorXd b(v.size());
  VectorXDecisionVariable vars{};
  DecomposeLinearExpression(v, &A, &b, &vars);
  DRAKE_DEMAND(vars.rows() >= 1);
  DRAKE_ASSERT(IsDecisionVariable(vars));
  return Binding<RotatedLorentzConeConstraint>(
      AddRotatedLorentzConeConstraint(A, b, vars), vars);
}

std::shared_ptr<RotatedLorentzConeConstraint>
MathematicalProgram::AddRotatedLorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  std::shared_ptr<RotatedLorentzConeConstraint> constraint =
      std::make_shared<RotatedLorentzConeConstraint>(A, b);
  AddConstraint(constraint, vars);
  return constraint;
}

std::shared_ptr<BoundingBoxConstraint>
MathematicalProgram::AddBoundingBoxConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  std::shared_ptr<BoundingBoxConstraint> constraint =
      std::make_shared<BoundingBoxConstraint>(lb, ub);
  AddConstraint(Binding<BoundingBoxConstraint>(constraint, vars));
  return constraint;
}

void MathematicalProgram::AddConstraint(
    const Binding<LinearComplementarityConstraint>& binding) {
  required_capabilities_ |= kLinearComplementarityConstraint;

  // Linear Complementarity Constraint cannot currently coexist with any
  // other types of constraint or cost.
  // (TODO(ggould-tri) relax this to non-overlapping bindings, possibly by
  // calling multiple solvers.)
  DRAKE_ASSERT(generic_constraints_.empty());
  DRAKE_ASSERT(generic_costs_.empty());
  DRAKE_ASSERT(quadratic_costs_.empty());
  DRAKE_ASSERT(linear_costs_.empty());
  DRAKE_ASSERT(linear_constraints_.empty());
  DRAKE_ASSERT(linear_equality_constraints_.empty());
  DRAKE_ASSERT(bbox_constraints_.empty());
  DRAKE_ASSERT(lorentz_cone_constraint_.empty());
  DRAKE_ASSERT(rotated_lorentz_cone_constraint_.empty());

  linear_complementarity_constraints_.push_back(binding);
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<LinearComplementarityConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddConstraint(Binding<LinearComplementarityConstraint>(con, vars));
}

std::shared_ptr<LinearComplementarityConstraint>
MathematicalProgram::AddLinearComplementarityConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& M,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  std::shared_ptr<LinearComplementarityConstraint> constraint =
      std::make_shared<LinearComplementarityConstraint>(M, q);
  AddConstraint(constraint, vars);
  return constraint;
}

std::shared_ptr<Constraint> MathematicalProgram::AddPolynomialConstraint(
    const VectorXPoly& polynomials,
    const std::vector<Polynomiald::VarType>& poly_vars,
    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  // Polynomials that are actually affine (a sum of linear terms + a
  // constant) can be special-cased.  Other polynomials are treated as
  // generic for now.
  // TODO(ggould-tri) There may be other such special easy cases.
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
    const Binding<PositiveSemidefiniteConstraint>& binding) {
  required_capabilities_ |= kPositiveSemidefiniteConstraint;
  DRAKE_ASSERT(
      drake::math::IsSymmetric(Eigen::Map<const MatrixXDecisionVariable>(
          binding.variables().data(), binding.constraint()->matrix_rows(),
          binding.constraint()->matrix_rows())));
  positive_semidefinite_constraint_.push_back(binding);
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<PositiveSemidefiniteConstraint> con,
    const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var) {
  required_capabilities_ |= kPositiveSemidefiniteConstraint;
  DRAKE_ASSERT(drake::math::IsSymmetric(symmetric_matrix_var));
  int num_rows = symmetric_matrix_var.rows();
  // TODO(hongkai.dai): this dynamic memory allocation/copying is ugly.
  VectorXDecisionVariable flat_symmetric_matrix_var(num_rows * num_rows);
  for (int i = 0; i < num_rows; ++i) {
    flat_symmetric_matrix_var.segment(i * num_rows, num_rows) =
        symmetric_matrix_var.col(i);
  }
  positive_semidefinite_constraint_.push_back(
      Binding<PositiveSemidefiniteConstraint>(con, flat_symmetric_matrix_var));
}

std::shared_ptr<PositiveSemidefiniteConstraint>
MathematicalProgram::AddPositiveSemidefiniteConstraint(
    const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var) {
  auto constraint = std::make_shared<PositiveSemidefiniteConstraint>(
      symmetric_matrix_var.rows());
  AddConstraint(constraint, symmetric_matrix_var);
  return constraint;
}

void MathematicalProgram::AddConstraint(
    const Binding<LinearMatrixInequalityConstraint>& binding) {
  required_capabilities_ |= kPositiveSemidefiniteConstraint;
  DRAKE_ASSERT(static_cast<int>(binding.constraint()->F().size()) ==
               static_cast<int>(binding.GetNumElements()) + 1);
  linear_matrix_inequality_constraint_.push_back(binding);
}

void MathematicalProgram::AddConstraint(
    std::shared_ptr<LinearMatrixInequalityConstraint> con,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddConstraint(Binding<LinearMatrixInequalityConstraint>(con, vars));
}

std::shared_ptr<LinearMatrixInequalityConstraint>
MathematicalProgram::AddLinearMatrixInequalityConstraint(
    const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto constraint = std::make_shared<LinearMatrixInequalityConstraint>(F);
  AddConstraint(constraint, vars);
  return constraint;
}

size_t MathematicalProgram::FindDecisionVariableIndex(
    const Variable& var) const {
  auto it = decision_variable_index_.find(var.get_id());
  DRAKE_DEMAND(it != decision_variable_index_.end());
  return it->second;
}

MathematicalProgram::VarType MathematicalProgram::DecisionVariableType(
    const Variable& var) const {
  return decision_variable_type_[FindDecisionVariableIndex(var)];
}

double MathematicalProgram::GetSolution(const Variable& var) const {
  return x_values_[FindDecisionVariableIndex(var)];
}

void MathematicalProgram::SetDecisionVariableValues(
    const Eigen::Ref<const Eigen::VectorXd>& values) {
  SetDecisionVariableValues(decision_variables_, values);
}

void MathematicalProgram::SetDecisionVariableValues(
    const Eigen::Ref<const VectorXDecisionVariable>& variables,
    const Eigen::Ref<const Eigen::VectorXd>& values) {
  DRAKE_ASSERT(values.rows() == variables.rows());
  for (int i = 0; i < values.rows(); ++i) {
    x_values_[FindDecisionVariableIndex(variables(i))] = values(i);
  }
}

void MathematicalProgram::SetDecisionVariableValue(const Variable& var,
                                                   double value) {
  x_values_[FindDecisionVariableIndex(var)] = value;
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
    throw runtime_error(
        "MathematicalProgram::Solve: "
        "No solver available for the given optimization problem!");
  }
}

}  // namespace solvers
}  // namespace drake
