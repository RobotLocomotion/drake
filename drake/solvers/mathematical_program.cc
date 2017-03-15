#include "drake/solvers/mathematical_program.h"

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/monomial.h"
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
using std::numeric_limits;
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

// Given an expression `e`, extracts all variables inside `e`.
// @param[in] e A symbolic expression.
// @retval pair pair.first is the variables in `e`. pair.second is the mapping
// from the variable ID to the index in pair.first, such that
// pair.second[pair.first(i).get_id()] = i
std::pair<VectorXDecisionVariable, unordered_map<Variable::Id, int>>
ExtractVariablesFromExpression(const Expression& e) {
  int var_count = 0;
  const symbolic::Variables var_set = e.GetVariables();
  VectorXDecisionVariable vars(var_set.size());
  unordered_map<Variable::Id, int> map_var_to_index{};
  map_var_to_index.reserve(var_set.size());
  for (const Variable& var : var_set) {
    map_var_to_index.emplace(var.get_id(), var_count);
    vars(var_count++) = var;
  }
  return std::pair<VectorXDecisionVariable, unordered_map<Variable::Id, int>>(
      vars, map_var_to_index);
}

// Given an expression `e`, extract all variables inside `e`, append these
// variables to `vars` if they are not included in `vars` yet.
// @param[in] e  A symbolic expression.
// @param[in,out] vars  As an input, `vars` contain the variables before
// extracting expression `e`. As an output, the variables in `e` that were not
// included in `vars`, will be appended to the end of `vars`.
// @param[in,out] map_var_to_index. map_var_to_index is of the same size as
// `vars`, and map_var_to_index[vars(i).get_id()] = i. This invariance holds
// for map_var_to_index both as the input and as the output.
void ExtractAndAppendVariablesFromExpression(
    const Expression &e, VectorXDecisionVariable* vars,
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
 *  Then, it extracts the coefficient and the constant term.
 *  A map from variable ID to int, @p map_var_to_index, is used to decide a
 *  variable's index in a linear combination.
 *
 *  \pre{1. @c coeffs is a row vector of double, whose length matches with the
 *          size of @c map_var_to_index.
 *       2. e.is_polynomial() is true.
 *       3. e is a linear expression.}
 * @tparam Derived An Eigen row vector type with Derived::Scalar == double.
 * @param[in] e The symbolic linear expression
 * @param[in] map_var_to_index A mapping from variable ID to variable index,
 * such that map_var_to_index[vi.get_ID()] = i.
 * @param[out] coeffs A row vector. coeffs(i) = ci.
 * @param[out] constant_term c0 in the equation above.
 * @return num_variable. Number of variables in the expression. 2 * x(0) + 3
 * has 1 variable, 2 * x(0) + 3 * x(1) - 2 * x(0) has 1 variable.
 */
template <typename Derived>
typename std::enable_if<std::is_same<typename Derived::Scalar, double>::value,
                        int>::type
DecomposeLinearExpression(
    const Expression& e,
    const std::unordered_map<Variable::Id, int>& map_var_to_index,
    const Eigen::MatrixBase<Derived>& coeffs, double* constant_term) {
  DRAKE_DEMAND(coeffs.rows() == 1);
  DRAKE_DEMAND(coeffs.cols() == static_cast<int>(map_var_to_index.size()));
  if (!e.is_polynomial()) {
    std::ostringstream oss;
    oss << "Expression " << e << "is not a polynomial.\n";
    throw runtime_error(oss.str());
  }
  const symbolic::Variables& vars = e.GetVariables();
  const auto& monomial_to_coeff_map =
      symbolic::DecomposePolynomialIntoMonomial(e, vars);
  int num_variable = 0;
  for (const auto& p : monomial_to_coeff_map) {
    const auto& p_monomial = p.first;
    DRAKE_ASSERT(is_constant(p.second));
    const double p_coeff = symbolic::get_constant_value(p.second);
    if (p_monomial.total_degree() > 1) {
      std::ostringstream oss;
      oss << "Expression " << e << " is non-linear.";
      throw std::runtime_error(oss.str());
    } else if (p_monomial.total_degree() == 1) {
      // Linear coefficient.
      const auto& p_monomial_powers = p_monomial.get_powers();
      DRAKE_DEMAND(p_monomial_powers.size() == 1);
      const Variable::Id var_id = p_monomial_powers.begin()->first;
      const_cast<Eigen::MatrixBase<Derived>&>(coeffs)(
          map_var_to_index.at(var_id)) = p_coeff;
      if (p_coeff != 0) {
        ++num_variable;
      }
    } else {
      // Constant term.
      *constant_term = p_coeff;
    }
  }
  return num_variable;
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
    ExtractAndAppendVariablesFromExpression(v(i), vars, &map_var_to_index);
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

/**
 * Given a quadratic expression `e` represented by its monomial to coefficient
 * map, decompose it into the form
 * e = 0.5 * x' * Q * x + b' * x + c
 * @param[in] monomial_to_coeff_map. Map the monomial to the coefficient, this
 * is the result of calling DecomposePolynomialIntoMonomial(e).
 * @param[in] map_var_to_index maps variables in
 * monomial_to_coeff_map.GetVariables() to the index in the vector `x`.
 * @param[in] num_variables The number of variables in the expression.
 * @param Q[out] The Hessian of the quadratic expression. @pre The size of Q
 * should be `num_variables * num_variables`.
 * @param b[out] The linear term of the quadratic expression. @pre The size of
 * `b` should be `num_variables * 1`.
 * @param c[out] The constant term of the quadratic expression.
 */
void DecomposeQuadraticExpressionWithMonomialToCoeffMap(
    const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map,
    const unordered_map<Variable::Id, int>& map_var_to_index,
    int num_variables,
    Eigen::MatrixXd* Q, Eigen::VectorXd* b, double* c) {
  DRAKE_DEMAND(Q->rows() == num_variables);
  DRAKE_DEMAND(Q->cols() == num_variables);
  DRAKE_DEMAND(b->rows() == num_variables);
  Q->setZero();
  b->setZero();
  *c = 0;
  for (const auto& p : monomial_to_coeff_map) {
    DRAKE_ASSERT(is_constant(p.second));
    DRAKE_DEMAND(!is_zero(p.second));
    const double coefficient = get_constant_value(p.second);
    const symbolic::Monomial& p_monomial = p.first;
    if (p_monomial.total_degree() > 2) {
      ostringstream oss;
      oss << p.first << " has order higher than 2, cannot be handled by "
          "DecomposeQuadraticExpressionWithMonomialToCoeffMap" << std::endl;
      throw std::runtime_error(oss.str());
    }
    const auto& monomial_powers = p_monomial.get_powers();
    if (monomial_powers.size() == 2) {
      // cross terms.
      auto it = monomial_powers.begin();
      const int x1_index = map_var_to_index.at(it->first);
      DRAKE_DEMAND(it->second == 1);
      ++it;
      const int x2_index = map_var_to_index.at(it->first);
      DRAKE_DEMAND(it->second == 1);
      (*Q)(x1_index, x2_index) += coefficient;
      (*Q)(x2_index, x1_index) = (*Q)(x1_index, x2_index);
    } else if (monomial_powers.size() == 1) {
      // Two cases
      // 1. quadratic term a*x^2
      // 2. linear term b*x
      auto it = monomial_powers.begin();
      DRAKE_DEMAND(it->second == 2 || it->second == 1);
      const int x_index = map_var_to_index.at(it->first);
      if (it->second == 2) {
        // quadratic term a * x^2
        (*Q)(x_index, x_index) += 2 * coefficient;
      } else if (it->second == 1) {
        // linear term b * x.
        (*b)(x_index) += coefficient;
      }
    } else {
      // constant term.
      *c += coefficient;
    }
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
  CheckIsDecisionVariable(binding.variables());
  linear_costs_.push_back(binding);
}

void MathematicalProgram::AddCost(
    const std::shared_ptr<LinearConstraint>& obj,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddCost(Binding<LinearConstraint>(obj, vars));
}

Binding<LinearConstraint> MathematicalProgram::AddLinearCost(
    const Expression& e) {

  auto p = ExtractVariablesFromExpression(e);
  const VectorXDecisionVariable& var = p.first;
  const auto& map_var_to_index = p.second;
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
                         -numeric_limits<double>::infinity()),
      drake::Vector1<double>::Constant(
          numeric_limits<double>::infinity()));
  AddCost(cost, vars);
  return cost;
}

void MathematicalProgram::AddCost(const Binding<QuadraticConstraint>& binding) {
  required_capabilities_ |= kQuadraticCost;
  DRAKE_ASSERT(binding.constraint()->Q().rows() ==
                   static_cast<int>(binding.GetNumElements()) &&
               binding.constraint()->b().rows() ==
                   static_cast<int>(binding.GetNumElements()));
  CheckIsDecisionVariable(binding.variables());
  quadratic_costs_.push_back(binding);
}

void MathematicalProgram::AddCost(
    const std::shared_ptr<QuadraticConstraint>& obj,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  AddCost(Binding<QuadraticConstraint>(obj, vars));
}

Binding<QuadraticConstraint> AddQuadraticCostWithMonomialToCoeffMap(
    const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map,
    const VectorXDecisionVariable& vars_vec,
    const unordered_map<Variable::Id, int>& map_var_to_index,
    MathematicalProgram* prog) {
  // We want to write the expression e in the form 0.5 * x' * Q * x + b' * x + c
  // TODO(hongkai.dai): use a sparse matrix to represent Q and b.
  Eigen::MatrixXd Q(vars_vec.size(), vars_vec.size());
  Eigen::VectorXd b(vars_vec.size());
  double constant_term;
  DecomposeQuadraticExpressionWithMonomialToCoeffMap(
      monomial_to_coeff_map, map_var_to_index, vars_vec.size(), &Q, &b,
      &constant_term);
  // Now add the quadratic constraint 0.5 * x' * Q * x + b' * x
  return Binding<QuadraticConstraint>(prog->AddQuadraticCost(Q, b, vars_vec),
                                      vars_vec);
}

Binding<QuadraticConstraint> MathematicalProgram::AddQuadraticCost(
    const symbolic::Expression& e) {
  // First build an Eigen vector, that contains all the bound variables.
  const symbolic::Variables& vars = e.GetVariables();
  auto p = ExtractVariablesFromExpression(e);
  const auto& vars_vec = p.first;
  const auto& map_var_to_index = p.second;

  // Now decomposes the expression into coefficients and monomials.
  const symbolic::MonomialToCoefficientMap&
      monomial_to_coeff_map =
          symbolic::DecomposePolynomialIntoMonomial(e, vars);
  return AddQuadraticCostWithMonomialToCoeffMap(monomial_to_coeff_map, vars_vec,
                                                map_var_to_index, this);
}

std::shared_ptr<QuadraticConstraint> MathematicalProgram::AddQuadraticErrorCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& x_desired,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto cost = std::make_shared<QuadraticConstraint>(
      2 * Q, -2 * Q * x_desired, -numeric_limits<double>::infinity(),
      numeric_limits<double>::infinity());
  AddCost(cost, vars);
  return cost;
}

std::shared_ptr<QuadraticConstraint> MathematicalProgram::AddQuadraticCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto cost = std::make_shared<QuadraticConstraint>(
      Q, b, -numeric_limits<double>::infinity(),
      numeric_limits<double>::infinity());
  AddCost(cost, vars);
  return cost;
}

Binding<Constraint> MathematicalProgram::AddCost(const Expression& e) {
  if (!e.is_polynomial()) {
    std::ostringstream oss;
    oss << "Expression " << e << " is not a polynomial. Currently AddCost does "
                                 "not support non-polynomial expression.\n";
    throw std::runtime_error(oss.str());
  }
  const symbolic::Variables& vars = e.GetVariables();
  const symbolic::MonomialToCoefficientMap& monomial_to_coeff_map =
      symbolic::DecomposePolynomialIntoMonomial(e, vars);
  int total_degree = 0;
  for (const auto& p : monomial_to_coeff_map) {
    total_degree = std::max(total_degree, p.first.total_degree());
  }


  auto e_extracted = ExtractVariablesFromExpression(e);
  const VectorXDecisionVariable& vars_vec = e_extracted.first;
  const auto& map_var_to_index = e_extracted.second;

  if (total_degree > 2) {
    std::ostringstream oss;
    oss << "Expression " << e << " has degree higher than 2. Currently AddCost "
                                 "only supports quadratic or linear "
                                 "expressions.\n";
    throw std::runtime_error(oss.str());
  } else if (total_degree == 2) {
    return  AddQuadraticCostWithMonomialToCoeffMap(
        monomial_to_coeff_map, vars_vec, map_var_to_index, this);
  } else {
    Eigen::VectorXd c(vars_vec.size());
    c.setZero();
    for (const auto& p : monomial_to_coeff_map) {
      if (p.first.total_degree() == 1) {
        const Variable::Id var_id = p.first.get_powers().begin()->first;
        DRAKE_DEMAND(is_constant(p.second));
        c(map_var_to_index.at(var_id)) += get_constant_value(p.second);
      }
    }
    auto lin_cost = AddLinearCost(c, vars_vec);
    return Binding<Constraint>(lin_cost, vars_vec);
  }
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
    ExtractAndAppendVariablesFromExpression(v(i), &vars, &map_var_to_index);
  }

  // Construct A, new_lb, new_ub. map_var_to_index is used here.
  Eigen::MatrixXd A{Eigen::MatrixXd::Zero(v.size(), vars.size())};
  Eigen::VectorXd new_lb{v.size()};
  Eigen::VectorXd new_ub{v.size()};
  // We will determine if lb <= v <= ub is a bounding box constraint, namely
  // x_lb <= x <= x_ub.
  bool is_v_bounding_box = true;
  for (int i = 0; i < v.size(); ++i) {
    double constant_term = 0;
    int num_vi_variables = DecomposeLinearExpression(v(i), map_var_to_index,
                                                     A.row(i), &constant_term);
    if (num_vi_variables == 0 &&
        !(lb(i) <= constant_term && constant_term <= ub(i))) {
      // Unsatisfiable constraint with no variables, such as 1 <= 0 <= 2
      throw SymbolicError(v(i), lb(i), ub(i),
          "unsatisfiable but called with AddLinearConstraint");

    } else {
      new_lb(i) = lb(i) - constant_term;
      new_ub(i) = ub(i) - constant_term;
      if (num_vi_variables != 1) {
        is_v_bounding_box = false;
      }
    }
  }
  if (is_v_bounding_box) {
    // If every lb(i) <= v(i) <= ub(i) is a bounding box constraint, then
    // formulate a bounding box constraint x_lb <= x <= x_ub
    VectorXDecisionVariable bounding_box_x(v.size());
    for (int i = 0; i < v.size(); ++i) {
      // v(i) is in the form of c * x
      double x_coeff = 0;
      for (const auto& x : v(i).GetVariables()) {
        const double coeff = A(i, map_var_to_index[x.get_id()]);
        if (coeff != 0) {
          x_coeff += coeff;
          bounding_box_x(i) = x;
        }
      }
      if (x_coeff > 0) {
        new_lb(i) /= x_coeff;
        new_ub(i) /= x_coeff;
      } else {
        const double lb_i = new_lb(i);
        new_lb(i) = new_ub(i) / x_coeff;
        new_ub(i) = lb_i / x_coeff;
      }
    }
    return Binding<BoundingBoxConstraint>(
        AddBoundingBoxConstraint(new_lb, new_ub, bounding_box_x),
        bounding_box_x);
  } else {
    return Binding<LinearConstraint>{
        AddLinearConstraint(A, new_lb, new_ub, vars), vars};
  }
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
                               numeric_limits<double>::infinity());
  } else if (is_less_than_or_equal_to(f)) {
    // e1 <= e2  ->  0 <= e2 - e1  ->  0 <= e2 - e1 <= ∞
    const Expression& e1{get_lhs_expression(f)};
    const Expression& e2{get_rhs_expression(f)};
    return AddLinearConstraint(e2 - e1, 0.0,
                               numeric_limits<double>::infinity());
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
  CheckIsDecisionVariable(binding.variables());
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
  return AddLinearEqualityConstraint(drake::Vector1<Expression>(e),
                                     Vector1d(b));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::DoAddLinearEqualityConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  DRAKE_DEMAND(v.rows() == b.rows());
  VectorXDecisionVariable vars(0);
  unordered_map<Variable::Id, int> map_var_to_index;
  for (int i = 0; i < v.rows(); ++i) {
    ExtractAndAppendVariablesFromExpression(v(i), &vars, &map_var_to_index);
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
  CheckIsDecisionVariable(binding.variables());
  lorentz_cone_constraint_.push_back(binding);
}

Binding<LorentzConeConstraint> MathematicalProgram::AddLorentzConeConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v) {
  DRAKE_DEMAND(v.rows() >= 2);
  Eigen::MatrixXd A{};
  Eigen::VectorXd b(v.size());
  VectorXDecisionVariable vars{};
  DecomposeLinearExpression(v, &A, &b, &vars);
  DRAKE_DEMAND(vars.rows() >= 1);
  return Binding<LorentzConeConstraint>(AddLorentzConeConstraint(A, b, vars),
                                        vars);
}

Binding<LorentzConeConstraint> MathematicalProgram::AddLorentzConeConstraint(
    const symbolic::Expression& linear_expr,
    const symbolic::Expression& quadratic_expr) {
  const auto& quadratic_p = ExtractVariablesFromExpression(quadratic_expr);
  const auto& quadratic_vars = quadratic_p.first;
  const auto& quadratic_var_to_index_map = quadratic_p.second;
  const auto& monomial_to_coeff_map = symbolic::DecomposePolynomialIntoMonomial(
      quadratic_expr, quadratic_expr.GetVariables());
  Eigen::MatrixXd Q(quadratic_vars.size(), quadratic_vars.size());
  Eigen::VectorXd b(quadratic_vars.size());
  double a;
  DecomposeQuadraticExpressionWithMonomialToCoeffMap(
      monomial_to_coeff_map, quadratic_var_to_index_map, quadratic_vars.size(),
      &Q, &b, &a);
  // The constraint that the linear expression v1 satisfying
  // v1 >= sqrt(0.5 * x' * Q * x + b' * x + a), is equivalent to the vector
  // [z; y] being within a Lorentz cone, where
  // z = v1
  // y = [1/sqrt(2) * (R * x + R⁻ᵀb); sqrt(a - 0.5 * bᵀ * Q⁻¹ * a)]
  // R is the matrix satisfying Rᵀ * R = Q

  VectorX<Expression> expr{};

  double constant;  // constant is a - 0.5 * bᵀ * Q⁻¹ * a
  // If Q is strictly positive definite, then use LLT
  Eigen::LLT<Eigen::MatrixXd> llt_Q(Q.selfadjointView<Eigen::Upper>());
  if (llt_Q.info() == Eigen::Success) {
    Eigen::MatrixXd R = llt_Q.matrixU();
    expr.resize(2 + R.rows());
    expr(0) = linear_expr;
    expr.segment(1, R.rows()) =
        1.0 / std::sqrt(2) * (R * quadratic_vars + llt_Q.matrixL().solve(b));
    constant = a - 0.5 * b.dot(llt_Q.solve(b));
  } else {
    // Q is not strictly positive definite.
    // First check if Q is zero.
    const bool is_Q_zero = (Q.array() == 0).all();

    if (is_Q_zero) {
      // Now check if the linear term b is zero. If both Q and b are zero, then
      // add the linear constraint linear_expr >= sqrt(a); otherwise throw a
      // runtime error.
      const bool is_b_zero = (b.array() == 0).all();
      if (!is_b_zero) {
        ostringstream oss;
        oss << "Expression " << quadratic_expr
            << " is not quadratic, cannot call AddLorentzConeConstraint.\n";
        throw std::runtime_error(oss.str());
      } else {
        if (a < 0) {
          ostringstream oss;
          oss << "Expression " << quadratic_expr
              << " is negative, cannot call AddLorentzConeConstraint.\n";
          throw std::runtime_error(oss.str());
        }
        Vector2<Expression> expr_constant_quadratic(linear_expr, std::sqrt(a));
        return AddLorentzConeConstraint(expr_constant_quadratic);
      }
    }
    // Q is not strictly positive, nor is it zero. Use LDLT to decompose Q
    // into R * Rᵀ.
    // Question: is there a better way to compute R * x and R⁻ᵀb? The following
    // code is really ugly.
    Eigen::LDLT<Eigen::MatrixXd> ldlt_Q(Q.selfadjointView<Eigen::Upper>());
    if (ldlt_Q.info() != Eigen::Success || !ldlt_Q.isPositive()) {
      std::ostringstream oss;
      oss << "Expression" << quadratic_expr
          << " does not have a positive semidefinite Hessian. Cannot be called "
              "with AddLorentzConeConstraint.\n";
      throw std::runtime_error(oss.str());
    }
    Eigen::MatrixXd R1 = ldlt_Q.matrixU();
    for (int i = 0; i < R1.rows(); ++i) {
      for (int j = 0; j < i; ++j) {
        R1(i, j) = 0;
      }
      const double d_sqrt = std::sqrt(ldlt_Q.vectorD()(i));
      for (int j = i; j < R1.cols(); ++j) {
        R1(i, j) *= d_sqrt;
      }
    }
    Eigen::MatrixXd R = R1 * ldlt_Q.transpositionsP();

    expr.resize(2 + R1.rows());
    expr(0) = linear_expr;
    // expr.segment(1, R1.rows()) = 1/sqrt(2) * (R * x + R⁻ᵀb)
    expr.segment(1, R1.rows()) =
        1.0 / std::sqrt(2) *
            (R * quadratic_vars
                + R.transpose().fullPivHouseholderQr().solve(b));
    constant = a - 0.5 * b.dot(ldlt_Q.solve(b));
  }
  if (constant < 0) {
    std::ostringstream oss;
    oss << "Expression " << quadratic_expr
        << " is not guaranteed to be non-negative, cannot call it with "
            "AddLorentzConeConstraint.\n";
    throw std::runtime_error(oss.str());
  }
  expr(expr.rows() - 1) = std::sqrt(constant);
  return AddLorentzConeConstraint(expr);
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
  CheckIsDecisionVariable(binding.variables());
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
  if (it == decision_variable_index_.end()) {
    std::ostringstream oss;
    oss << var << " is not a decision variable in the mathematical program, "
                  "when calling GetSolution.\n";
    throw std::runtime_error(oss.str());
  }
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
