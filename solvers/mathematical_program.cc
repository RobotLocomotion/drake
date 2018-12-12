#include "drake/solvers/mathematical_program.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/linear_system_solver.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/sos_basis_generator.h"
#include "drake/solvers/symbolic_extraction.h"
// Note that the file mathematical_program_api.cc also contains some of the
// implementation of mathematical_program.h

namespace drake {
namespace solvers {

using std::enable_if;
using std::endl;
using std::find;
using std::is_same;
using std::make_pair;
using std::make_shared;
using std::map;
using std::numeric_limits;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::unordered_map;
using std::vector;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;
using symbolic::Variables;

using internal::CreateBinding;
using internal::DecomposeLinearExpression;
using internal::SymbolicError;

constexpr double MathematicalProgram::kGlobalInfeasibleCost;
constexpr double MathematicalProgram::kUnboundedCost;

MathematicalProgram::MathematicalProgram()
    : x_initial_guess_(0),
      optimal_cost_(numeric_limits<double>::quiet_NaN()),
      lower_bound_cost_(-numeric_limits<double>::infinity()),
      required_capabilities_{},
      ipopt_solver_(new IpoptSolver()),
      nlopt_solver_(new NloptSolver()),
      snopt_solver_(new SnoptSolver()),
      moby_lcp_solver_(new MobyLCPSolver<double>()),
      linear_system_solver_(new LinearSystemSolver()),
      equality_constrained_qp_solver_(new EqualityConstrainedQPSolver()),
      gurobi_solver_(new GurobiSolver()),
      mosek_solver_(new MosekSolver()),
      osqp_solver_(new OsqpSolver()),
      scs_solver_(new ScsSolver()) {}

std::unique_ptr<MathematicalProgram> MathematicalProgram::Clone() const {
  // The constructor of MathematicalProgram will construct each solver. It
  // also sets x_values_ and x_initial_guess_ to default values.
  auto new_prog = std::make_unique<MathematicalProgram>();
  // Add variables and indeterminates
  // AddDecisionVariables and AddIndeterminates also set
  // decision_variable_index_ and indeterminate_index_ properly.
  new_prog->AddDecisionVariables(decision_variables_);
  new_prog->AddIndeterminates(indeterminates_);
  // Add costs
  new_prog->generic_costs_ = generic_costs_;
  new_prog->quadratic_costs_ = quadratic_costs_;
  new_prog->linear_costs_ = linear_costs_;

  // Add constraints
  new_prog->generic_constraints_ = generic_constraints_;
  new_prog->linear_constraints_ = linear_constraints_;
  new_prog->linear_equality_constraints_ = linear_equality_constraints_;
  new_prog->bbox_constraints_ = bbox_constraints_;
  new_prog->lorentz_cone_constraint_ = lorentz_cone_constraint_;
  new_prog->rotated_lorentz_cone_constraint_ =
      rotated_lorentz_cone_constraint_;
  new_prog->positive_semidefinite_constraint_ =
      positive_semidefinite_constraint_;
  new_prog->linear_matrix_inequality_constraint_ =
      linear_matrix_inequality_constraint_;
  new_prog->linear_complementarity_constraints_ =
      linear_complementarity_constraints_;

  new_prog->x_initial_guess_ = x_initial_guess_;
  new_prog->solver_id_ = solver_id_;
  new_prog->solver_options_ = solver_options_;

  new_prog->required_capabilities_ = required_capabilities_;
  return new_prog;
}

MatrixXDecisionVariable MathematicalProgram::NewVariables(
    VarType type, int rows, int cols, bool is_symmetric,
    const vector<string>& names) {
  MatrixXDecisionVariable decision_variable_matrix(rows, cols);
  NewVariables_impl(type, names, is_symmetric, decision_variable_matrix);
  return decision_variable_matrix;
}

MatrixXDecisionVariable MathematicalProgram::NewSymmetricContinuousVariables(
    int rows, const string& name) {
  vector<string> names(rows * (rows + 1) / 2);
  int count = 0;
  for (int j = 0; j < static_cast<int>(rows); ++j) {
    for (int i = j; i < static_cast<int>(rows); ++i) {
      names[count] = name + "(" + to_string(i) + "," + to_string(j) + ")";
      ++count;
    }
  }
  return NewVariables(VarType::CONTINUOUS, rows, rows, true, names);
}

void MathematicalProgram::AddDecisionVariables(
    const Eigen::Ref<const VectorXDecisionVariable>& decision_variables) {
  const int num_existing_decision_vars = num_vars();
  for (int i = 0; i < decision_variables.rows(); ++i) {
    if (decision_variables(i).is_dummy()) {
      throw std::runtime_error(fmt::format(
          "decision_variables({}) should not be a dummy variable", i));
    }
    if (decision_variable_index_.find(decision_variables(i).get_id()) !=
        decision_variable_index_.end()) {
      throw std::runtime_error(fmt::format("{} is already a decision variable.",
                                           decision_variables(i)));
    }
    if (indeterminates_index_.find(decision_variables(i).get_id()) !=
        indeterminates_index_.end()) {
      throw std::runtime_error(fmt::format("{} is already an indeterminate.",
                                           decision_variables(i)));
    }
    decision_variable_index_.insert(std::make_pair(
        decision_variables(i).get_id(), num_existing_decision_vars + i));
  }
  decision_variables_.conservativeResize(num_existing_decision_vars +
                                         decision_variables.rows());
  decision_variables_.tail(decision_variables.rows()) = decision_variables;
  AppendNanToEnd(decision_variables.rows(), &x_values_);
  AppendNanToEnd(decision_variables.rows(), &x_initial_guess_);
}

symbolic::Polynomial MathematicalProgram::NewFreePolynomial(
    const Variables& indeterminates, const int degree,
    const string& coeff_name) {
  const drake::VectorX<symbolic::Monomial> m{
      MonomialBasis(indeterminates, degree)};
  const VectorXDecisionVariable coeffs{
      NewContinuousVariables(m.size(), coeff_name)};
  symbolic::Polynomial p;
  for (int i = 0; i < m.size(); ++i) {
    p.AddProduct(coeffs(i), m(i));  // p += coeffs(i) * m(i);
  }
  return p;
}

std::pair<symbolic::Polynomial, Binding<PositiveSemidefiniteConstraint>>
MathematicalProgram::NewSosPolynomial(
    const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis) {
  const MatrixXDecisionVariable Q{
      NewSymmetricContinuousVariables(monomial_basis.size())};
  const auto psd_binding = AddPositiveSemidefiniteConstraint(Q);
  // TODO(hongkai.dai): ideally we should compute p in one line as
  // monomial_basis.dot(Q * monomial_basis). But as explained in #10200, this
  // one line version is too slow, so we use this double for loops to compute
  // the matrix product by hand. I will revert to the one line version when it
  // is fast.
  symbolic::Polynomial p{};
  for (int i = 0; i < Q.rows(); ++i) {
    p.AddProduct(Q(i, i), pow(monomial_basis(i), 2));
    for (int j = i + 1; j < Q.cols(); ++j) {
      p.AddProduct(2 * Q(i, j), monomial_basis(i) * monomial_basis(j));
    }
  }
  return make_pair(p, psd_binding);
}

pair<symbolic::Polynomial, Binding<PositiveSemidefiniteConstraint>>
MathematicalProgram::NewSosPolynomial(const Variables& indeterminates,
                                      const int degree) {
  DRAKE_DEMAND(degree > 0 && degree % 2 == 0);
  const drake::VectorX<symbolic::Monomial> x{
      MonomialBasis(indeterminates, degree / 2)};
  return NewSosPolynomial(x);
}

MatrixXIndeterminate MathematicalProgram::NewIndeterminates(
    int rows, int cols, const vector<string>& names) {
  MatrixXIndeterminate indeterminates_matrix(rows, cols);
  NewIndeterminates_impl(names, indeterminates_matrix);
  return indeterminates_matrix;
}

VectorXIndeterminate MathematicalProgram::NewIndeterminates(
    int rows, const std::vector<std::string>& names) {
  return NewIndeterminates(rows, 1, names);
}

VectorXIndeterminate MathematicalProgram::NewIndeterminates(
    int rows, const string& name) {
  vector<string> names(rows);
  for (int i = 0; i < static_cast<int>(rows); ++i) {
    names[i] = name + "(" + to_string(i) + ")";
  }
  return NewIndeterminates(rows, names);
}

MatrixXIndeterminate MathematicalProgram::NewIndeterminates(
    int rows, int cols, const string& name) {
  vector<string> names(rows * cols);
  int count = 0;
  for (int j = 0; j < static_cast<int>(cols); ++j) {
    for (int i = 0; i < static_cast<int>(rows); ++i) {
      names[count] = name + "(" + to_string(i) + "," + to_string(j) + ")";
      ++count;
    }
  }
  return NewIndeterminates(rows, cols, names);
}

void MathematicalProgram::AddIndeterminates(
    const Eigen::Ref<const VectorXDecisionVariable>& new_indeterminates) {
  const int num_old_indeterminates = num_indeterminates();
  for (int i = 0; i < new_indeterminates.rows(); ++i) {
    if (new_indeterminates(i).is_dummy()) {
      throw std::runtime_error(fmt::format(
          "new_indeterminates({}) should not be a dummy variable.", i));
    }
    if (indeterminates_index_.find(new_indeterminates(i).get_id()) !=
            indeterminates_index_.end() ||
        decision_variable_index_.find(new_indeterminates(i).get_id()) !=
            decision_variable_index_.end()) {
      throw std::runtime_error(
          fmt::format("{} already exists in the optimization program.",
                      new_indeterminates(i)));
    }
    if (new_indeterminates(i).get_type() !=
        symbolic::Variable::Type::CONTINUOUS) {
      throw std::runtime_error("indeterminate should of type CONTINUOUS.\n");
    }
    indeterminates_index_.insert(std::make_pair(new_indeterminates(i).get_id(),
                                                num_old_indeterminates + i));
  }
  indeterminates_.conservativeResize(num_old_indeterminates +
                                     new_indeterminates.rows());
  indeterminates_.tail(new_indeterminates.rows()) = new_indeterminates;
}

Binding<VisualizationCallback> MathematicalProgram::AddVisualizationCallback(
    const VisualizationCallback::CallbackFunction &callback,
    const Eigen::Ref<const VectorXDecisionVariable> &vars) {
  visualization_callbacks_.push_back(
      internal::CreateBinding<VisualizationCallback>(
          make_shared<VisualizationCallback>(vars.size(), callback), vars));
  required_capabilities_.insert(ProgramAttribute::kCallback);
  return visualization_callbacks_.back();
}

Binding<Cost> MathematicalProgram::AddCost(const Binding<Cost>& binding) {
  // See AddCost(const Binding<Constraint>&) for explanation
  Cost* cost = binding.evaluator().get();
  if (dynamic_cast<QuadraticCost*>(cost)) {
    return AddCost(internal::BindingDynamicCast<QuadraticCost>(binding));
  } else if (dynamic_cast<LinearCost*>(cost)) {
    return AddCost(internal::BindingDynamicCast<LinearCost>(binding));
  } else {
    CheckBinding(binding);
    required_capabilities_.insert(ProgramAttribute::kGenericCost);
    generic_costs_.push_back(binding);
    return generic_costs_.back();
  }
}

Binding<LinearCost> MathematicalProgram::AddCost(
    const Binding<LinearCost>& binding) {
  CheckBinding(binding);
  required_capabilities_.insert(ProgramAttribute::kLinearCost);
  linear_costs_.push_back(binding);
  return linear_costs_.back();
}

Binding<LinearCost> MathematicalProgram::AddLinearCost(const Expression& e) {
  return AddCost(internal::ParseLinearCost(e));
}

Binding<LinearCost> MathematicalProgram::AddLinearCost(
    const Eigen::Ref<const Eigen::VectorXd>& a, double b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddCost(make_shared<LinearCost>(a, b), vars);
}

Binding<QuadraticCost> MathematicalProgram::AddCost(
    const Binding<QuadraticCost>& binding) {
  CheckBinding(binding);
  required_capabilities_.insert(ProgramAttribute::kQuadraticCost);
  DRAKE_ASSERT(binding.evaluator()->Q().rows() ==
                   static_cast<int>(binding.GetNumElements()) &&
               binding.evaluator()->b().rows() ==
                   static_cast<int>(binding.GetNumElements()));
  quadratic_costs_.push_back(binding);
  return quadratic_costs_.back();
}

Binding<QuadraticCost> MathematicalProgram::AddQuadraticCost(
    const Expression& e) {
  return AddCost(internal::ParseQuadraticCost(e));
}

Binding<QuadraticCost> MathematicalProgram::AddQuadraticErrorCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& x_desired,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddCost(MakeQuadraticErrorCost(Q, x_desired), vars);
}

Binding<QuadraticCost> MathematicalProgram::AddQuadraticCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b, double c,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddCost(make_shared<QuadraticCost>(Q, b, c), vars);
}

Binding<QuadraticCost> MathematicalProgram::AddQuadraticCost(
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddQuadraticCost(Q, b, 0., vars);
}

Binding<PolynomialCost> MathematicalProgram::AddPolynomialCost(
    const Expression& e) {
  auto binding = AddCost(internal::ParsePolynomialCost(e));
  return internal::BindingDynamicCast<PolynomialCost>(binding);
}

Binding<Cost> MathematicalProgram::AddCost(const Expression& e) {
  return AddCost(internal::ParseCost(e));
}

Binding<Constraint> MathematicalProgram::AddConstraint(
    const Binding<Constraint>& binding) {
  // TODO(eric.cousineau): Use alternative to RTTI.
  // Move kGenericConstraint, etc. to Constraint. Dispatch based on this
  // information. As it is, this causes extra work when we explicitly want a
  // generic constraint.

  // If we get here, then this was possibly a dynamically-simplified
  // constraint. Determine correct container. As last resort, add to generic
  // constraints.
  Constraint* constraint = binding.evaluator().get();
  // Check constraints types in reverse order, such that classes that inherit
  // from other classes will not be prematurely added to less specific (or
  // incorrect) container.
  if (dynamic_cast<LinearMatrixInequalityConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<LinearMatrixInequalityConstraint>(
            binding));
  } else if (dynamic_cast<PositiveSemidefiniteConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<PositiveSemidefiniteConstraint>(binding));
  } else if (dynamic_cast<RotatedLorentzConeConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<RotatedLorentzConeConstraint>(binding));
  } else if (dynamic_cast<LorentzConeConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<LorentzConeConstraint>(binding));
  } else if (dynamic_cast<LinearConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<LinearConstraint>(binding));
  } else {
    CheckBinding(binding);
    required_capabilities_.insert(ProgramAttribute::kGenericConstraint);
    generic_constraints_.push_back(binding);
    return generic_constraints_.back();
  }
}

Binding<Constraint> MathematicalProgram::AddConstraint(const Expression& e,
                                                       const double lb,
                                                       const double ub) {
  return AddConstraint(internal::ParseConstraint(e, lb, ub));
}

Binding<Constraint> MathematicalProgram::AddConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub) {
  return AddConstraint(internal::ParseConstraint(v, lb, ub));
}

Binding<Constraint> MathematicalProgram::AddConstraint(
    const set<Formula>& formulas) {
  return AddConstraint(internal::ParseConstraint(formulas));
}

Binding<Constraint> MathematicalProgram::AddConstraint(const Formula& f) {
  return AddConstraint(internal::ParseConstraint(f));
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Expression& e, const double lb, const double ub) {
  Binding<Constraint> binding = internal::ParseConstraint(e, lb, ub);
  Constraint* constraint = binding.evaluator().get();
  if (dynamic_cast<LinearConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<LinearConstraint>(binding));
  } else {
    std::stringstream oss;
    oss << "Expression " << e << " is non-linear.";
    throw std::runtime_error(oss.str());
  }
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub) {
  Binding<Constraint> binding = internal::ParseConstraint(v, lb, ub);
  Constraint* constraint = binding.evaluator().get();
  if (dynamic_cast<LinearConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<LinearConstraint>(binding));
  } else {
    std::stringstream oss;
    oss << "Expression " << v << " is non-linear.";
    throw std::runtime_error(oss.str());
  }
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Formula& f) {
  Binding<Constraint> binding = internal::ParseConstraint(f);
  Constraint* constraint = binding.evaluator().get();
  if (dynamic_cast<LinearConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<LinearConstraint>(binding));
  } else {
    std::stringstream oss;
    oss << "Formula " << f << " is non-linear.";
    throw std::runtime_error(oss.str());
  }
}

Binding<LinearConstraint> MathematicalProgram::AddConstraint(
    const Binding<LinearConstraint>& binding) {
  // Because the ParseConstraint methods can return instances of
  // LinearEqualityConstraint or BoundingBoxConstraint, do a dynamic check
  // here.
  LinearConstraint* constraint = binding.evaluator().get();
  if (dynamic_cast<BoundingBoxConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<BoundingBoxConstraint>(binding));
  } else if (dynamic_cast<LinearEqualityConstraint*>(constraint)) {
    return AddConstraint(
        internal::BindingDynamicCast<LinearEqualityConstraint>(binding));
  } else {
    // TODO(eric.cousineau): This is a good assertion... But seems out of place,
    // possibly redundant w.r.t. the binding infrastructure.
    DRAKE_ASSERT(binding.evaluator()->A().cols() ==
                 static_cast<int>(binding.GetNumElements()));
    CheckBinding(binding);
    required_capabilities_.insert(ProgramAttribute::kLinearConstraint);
    linear_constraints_.push_back(binding);
    return linear_constraints_.back();
  }
}

Binding<LinearConstraint> MathematicalProgram::AddLinearConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(make_shared<LinearConstraint>(A, lb, ub), vars);
}

Binding<LinearEqualityConstraint> MathematicalProgram::AddConstraint(
    const Binding<LinearEqualityConstraint>& binding) {
  DRAKE_ASSERT(binding.evaluator()->A().cols() ==
               static_cast<int>(binding.GetNumElements()));
  CheckBinding(binding);
  required_capabilities_.insert(ProgramAttribute::kLinearEqualityConstraint);
  linear_equality_constraints_.push_back(binding);
  return linear_equality_constraints_.back();
}

Binding<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(const Expression& e,
                                                 double b) {
  return AddConstraint(internal::ParseLinearEqualityConstraint(e, b));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(const set<Formula>& formulas) {
  return AddConstraint(internal::ParseLinearEqualityConstraint(formulas));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(const Formula& f) {
  return AddConstraint(internal::ParseLinearEqualityConstraint(f));
}

Binding<LinearEqualityConstraint>
MathematicalProgram::AddLinearEqualityConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& Aeq,
    const Eigen::Ref<const Eigen::VectorXd>& beq,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  return AddConstraint(make_shared<LinearEqualityConstraint>(Aeq, beq), vars);
}

Binding<BoundingBoxConstraint> MathematicalProgram::AddConstraint(
    const Binding<BoundingBoxConstraint>& binding) {
  CheckBinding(binding);
  DRAKE_ASSERT(binding.evaluator()->num_outputs() ==
               static_cast<int>(binding.GetNumElements()));
  required_capabilities_.insert(ProgramAttribute::kLinearConstraint);
  bbox_constraints_.push_back(binding);
  return bbox_constraints_.back();
}

Binding<LorentzConeConstraint> MathematicalProgram::AddConstraint(
    const Binding<LorentzConeConstraint>& binding) {
  CheckBinding(binding);
  required_capabilities_.insert(ProgramAttribute::kLorentzConeConstraint);
  lorentz_cone_constraint_.push_back(binding);
  return lorentz_cone_constraint_.back();
}

Binding<LorentzConeConstraint> MathematicalProgram::AddLorentzConeConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v) {
  return AddConstraint(internal::ParseLorentzConeConstraint(v));
}

Binding<LorentzConeConstraint> MathematicalProgram::AddLorentzConeConstraint(
    const Expression& linear_expression, const Expression& quadratic_expression,
    double tol) {
  return AddConstraint(internal::ParseLorentzConeConstraint(
      linear_expression, quadratic_expression, tol));
}

Binding<LorentzConeConstraint> MathematicalProgram::AddLorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  shared_ptr<LorentzConeConstraint> constraint =
      make_shared<LorentzConeConstraint>(A, b);
  return AddConstraint(Binding<LorentzConeConstraint>(constraint, vars));
}

Binding<RotatedLorentzConeConstraint> MathematicalProgram::AddConstraint(
    const Binding<RotatedLorentzConeConstraint>& binding) {
  CheckBinding(binding);
  required_capabilities_.insert(
      ProgramAttribute::kRotatedLorentzConeConstraint);
  rotated_lorentz_cone_constraint_.push_back(binding);
  return rotated_lorentz_cone_constraint_.back();
}

Binding<RotatedLorentzConeConstraint>
MathematicalProgram::AddRotatedLorentzConeConstraint(
    const symbolic::Expression& linear_expression1,
    const symbolic::Expression& linear_expression2,
    const symbolic::Expression& quadratic_expression, double tol) {
  auto binding = internal::ParseRotatedLorentzConeConstraint(
      linear_expression1, linear_expression2, quadratic_expression, tol);
  AddConstraint(binding);
  return binding;
}

Binding<RotatedLorentzConeConstraint>
MathematicalProgram::AddRotatedLorentzConeConstraint(
    const Eigen::Ref<const VectorX<Expression>>& v) {
  auto binding = internal::ParseRotatedLorentzConeConstraint(v);
  AddConstraint(binding);
  return binding;
}

Binding<RotatedLorentzConeConstraint>
MathematicalProgram::AddRotatedLorentzConeConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  shared_ptr<RotatedLorentzConeConstraint> constraint =
      make_shared<RotatedLorentzConeConstraint>(A, b);
  return AddConstraint(constraint, vars);
}

Binding<BoundingBoxConstraint> MathematicalProgram::AddBoundingBoxConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& lb,
    const Eigen::Ref<const Eigen::VectorXd>& ub,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  shared_ptr<BoundingBoxConstraint> constraint =
      make_shared<BoundingBoxConstraint>(lb, ub);
  return AddConstraint(Binding<BoundingBoxConstraint>(constraint, vars));
}

Binding<LinearComplementarityConstraint> MathematicalProgram::AddConstraint(
    const Binding<LinearComplementarityConstraint>& binding) {
  CheckBinding(binding);

  required_capabilities_.insert(
      ProgramAttribute::kLinearComplementarityConstraint);

  linear_complementarity_constraints_.push_back(binding);
  return linear_complementarity_constraints_.back();
}

Binding<LinearComplementarityConstraint>
MathematicalProgram::AddLinearComplementarityConstraint(
    const Eigen::Ref<const Eigen::MatrixXd>& M,
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  shared_ptr<LinearComplementarityConstraint> constraint =
      make_shared<LinearComplementarityConstraint>(M, q);
  return AddConstraint(constraint, vars);
}

Binding<Constraint> MathematicalProgram::AddPolynomialConstraint(
    const VectorXPoly& polynomials,
    const vector<Polynomiald::VarType>& poly_vars, const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto constraint =
      internal::MakePolynomialConstraint(polynomials, poly_vars, lb, ub);
  return AddConstraint(constraint, vars);
}

Binding<PositiveSemidefiniteConstraint> MathematicalProgram::AddConstraint(
    const Binding<PositiveSemidefiniteConstraint>& binding) {
  CheckBinding(binding);
  DRAKE_ASSERT(math::IsSymmetric(Eigen::Map<const MatrixXDecisionVariable>(
      binding.variables().data(), binding.evaluator()->matrix_rows(),
      binding.evaluator()->matrix_rows())));
  required_capabilities_.insert(
      ProgramAttribute::kPositiveSemidefiniteConstraint);
  positive_semidefinite_constraint_.push_back(binding);
  return positive_semidefinite_constraint_.back();
}

Binding<PositiveSemidefiniteConstraint> MathematicalProgram::AddConstraint(
    shared_ptr<PositiveSemidefiniteConstraint> con,
    const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var) {
  DRAKE_ASSERT(math::IsSymmetric(symmetric_matrix_var));
  int num_rows = symmetric_matrix_var.rows();
  // TODO(hongkai.dai): this dynamic memory allocation/copying is ugly.
  // TODO(eric.cousineau): See if Eigen::Map<> can be used (column-major)
  VectorXDecisionVariable flat_symmetric_matrix_var(num_rows * num_rows);
  for (int i = 0; i < num_rows; ++i) {
    flat_symmetric_matrix_var.segment(i * num_rows, num_rows) =
        symmetric_matrix_var.col(i);
  }
  return AddConstraint(CreateBinding(con, flat_symmetric_matrix_var));
}

Binding<PositiveSemidefiniteConstraint>
MathematicalProgram::AddPositiveSemidefiniteConstraint(
    const Eigen::Ref<const MatrixXDecisionVariable>& symmetric_matrix_var) {
  auto constraint =
      make_shared<PositiveSemidefiniteConstraint>(symmetric_matrix_var.rows());
  return AddConstraint(constraint, symmetric_matrix_var);
}

Binding<LinearMatrixInequalityConstraint> MathematicalProgram::AddConstraint(
    const Binding<LinearMatrixInequalityConstraint>& binding) {
  CheckBinding(binding);
  DRAKE_ASSERT(static_cast<int>(binding.evaluator()->F().size()) ==
               static_cast<int>(binding.GetNumElements()) + 1);
  required_capabilities_.insert(
      ProgramAttribute::kPositiveSemidefiniteConstraint);
  linear_matrix_inequality_constraint_.push_back(binding);
  return linear_matrix_inequality_constraint_.back();
}

Binding<LinearMatrixInequalityConstraint>
MathematicalProgram::AddLinearMatrixInequalityConstraint(
    const vector<Eigen::Ref<const Eigen::MatrixXd>>& F,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) {
  auto constraint = make_shared<LinearMatrixInequalityConstraint>(F);
  return AddConstraint(constraint, vars);
}

MatrixX<symbolic::Expression>
MathematicalProgram::AddPositiveDiagonallyDominantMatrixConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Expression>>& X) {
  // First create the slack variables Y with the same size as X, Y being the
  // symmetric matrix representing the absolute value of X.
  const int num_X_rows = X.rows();
  DRAKE_DEMAND(X.cols() == num_X_rows);
  auto Y_upper = NewContinuousVariables((num_X_rows - 1) * num_X_rows / 2,
                                        "diagonally_dominant_slack");
  MatrixX<symbolic::Expression> Y(num_X_rows, num_X_rows);
  int Y_upper_count = 0;
  // Fill in the upper triangle of Y.
  for (int j = 0; j < num_X_rows; ++j) {
    for (int i = 0; i < j; ++i) {
      Y(i, j) = Y_upper(Y_upper_count);
      Y(j, i) = Y(i, j);
      ++Y_upper_count;
    }
    // The diagonal entries of Y.
    Y(j, j) = X(j, j);
  }
  // Add the constraint that Y(i, j) >= |X(i, j) + X(j, i) / 2|
  for (int i = 0; i < num_X_rows; ++i) {
    for (int j = i + 1; j < num_X_rows; ++j) {
      AddLinearConstraint(Y(i, j) >= (X(i, j) + X(j, i)) / 2);
      AddLinearConstraint(Y(i, j) >= -(X(i, j) + X(j, i)) / 2);
    }
  }

  // Add the constraint X(i, i) >= sum_j Y(i, j), j ≠ i
  for (int i = 0; i < num_X_rows; ++i) {
    symbolic::Expression y_sum = 0;
    for (int j = 0; j < num_X_rows; ++j) {
      if (j == i) {
        continue;
      }
      y_sum += Y(i, j);
    }
    AddLinearConstraint(X(i, i) >= y_sum);
  }
  return Y;
}

std::vector<std::vector<Matrix2<symbolic::Expression>>>
MathematicalProgram::AddScaledDiagonallyDominantMatrixConstraint(
    const Eigen::Ref<const MatrixX<symbolic::Expression>>& X) {
  const int n = X.rows();
  DRAKE_DEMAND(X.cols() == n);
  std::vector<std::vector<Matrix2<symbolic::Expression>>> M(n);
  for (int i = 0; i < n; ++i) {
    M[i].resize(n);
    for (int j = i + 1; j < n; ++j) {
      // Since M[i][j](0, 1) = X(i, j), we only need to declare new variables
      // for the diagonal entries of M[i][j].
      auto M_ij_diagonal = NewContinuousVariables<2>(
          "sdd_slack_M[" + std::to_string(i) + "][" + std::to_string(j) + "]");
      M[i][j](0, 0) = M_ij_diagonal(0);
      M[i][j](1, 1) = M_ij_diagonal(1);
      M[i][j](0, 1) = (X(i, j) + X(j, i)) / 2;
      M[i][j](1, 0) = M[i][j](0, 1);
      AddRotatedLorentzConeConstraint(Vector3<symbolic::Expression>(
          M[i][j](0, 0), M[i][j](1, 1), M[i][j](0, 1)));
    }
  }
  for (int i = 0; i < n; ++i) {
    symbolic::Expression diagonal_sum = 0;
    for (int j = 0; j < i; ++j) {
      diagonal_sum += M[j][i](1, 1);
    }
    for (int j = i + 1; j < n; ++j) {
      diagonal_sum += M[i][j](0, 0);
    }
    AddLinearEqualityConstraint(X(i, i) - diagonal_sum, 0);
  }
  return M;
}

// Note that FindDecisionVariableIndex is implemented in
// mathematical_program_api.cc instead of this file.

// Note that FindIndeterminateIndex is implemented in
// mathematical_program_api.cc instead of this file.

pair<Binding<PositiveSemidefiniteConstraint>, Binding<LinearEqualityConstraint>>
MathematicalProgram::AddSosConstraint(
    const symbolic::Polynomial& p,
    const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis) {
  const auto pair = NewSosPolynomial(monomial_basis);
  const symbolic::Polynomial& sos_poly{pair.first};
  const Binding<PositiveSemidefiniteConstraint>& psd_binding{pair.second};
  const auto leq_binding = AddLinearEqualityConstraint(sos_poly == p);
  return make_pair(psd_binding, leq_binding);
}

pair<Binding<PositiveSemidefiniteConstraint>, Binding<LinearEqualityConstraint>>
MathematicalProgram::AddSosConstraint(const symbolic::Polynomial& p) {
  return AddSosConstraint(
      p, ConstructMonomialBasis(p));
}

pair<Binding<PositiveSemidefiniteConstraint>, Binding<LinearEqualityConstraint>>
MathematicalProgram::AddSosConstraint(
    const symbolic::Expression& e,
    const Eigen::Ref<const VectorX<symbolic::Monomial>>& monomial_basis) {
  return AddSosConstraint(
      symbolic::Polynomial{e, symbolic::Variables{indeterminates_}},
      monomial_basis);
}

pair<Binding<PositiveSemidefiniteConstraint>, Binding<LinearEqualityConstraint>>
MathematicalProgram::AddSosConstraint(const symbolic::Expression& e) {
  return AddSosConstraint(
      symbolic::Polynomial{e, symbolic::Variables{indeterminates_}});
}

double MathematicalProgram::GetSolution(const Variable& var) const {
  return x_values_[FindDecisionVariableIndex(var)];
}

double MathematicalProgram::GetSolution(
    const Variable& var, const MathematicalProgramResult& result) const {
  if (result.get_x_val().rows() != num_vars()) {
    throw std::invalid_argument("result.get_x_val().rows() = " +
                                std::to_string(result.get_x_val().rows()) +
                                ", num_vars() = " + std::to_string(num_vars()) +
                                ", they should be equal.");
  }
  return result.get_x_val()(FindDecisionVariableIndex(var));
}

namespace {
template <typename T>
T GetSolutionForExpressionOrPolynomial(const MathematicalProgram& prog,
                                       const T& p,
                                       const symbolic::Variables vars) {
  symbolic::Environment::map map_decision_vars;
  for (const auto& var : vars) {
    map_decision_vars.emplace(var, prog.GetSolution(var));
  }
  return p.EvaluatePartial(symbolic::Environment(map_decision_vars));
}
}  // namespace

symbolic::Expression MathematicalProgram::SubstituteSolution(
    const symbolic::Expression& e) const {
  symbolic::Environment::map map_decision_vars;
  for (const auto& var : e.GetVariables()) {
    const auto it = decision_variable_index_.find(var.get_id());
    if (it != decision_variable_index_.end()) {
      map_decision_vars.emplace(var, x_values_[it->second]);
    } else if (indeterminates_index_.find(var.get_id()) ==
               indeterminates_index_.end()) {
      // var is not a decision variable or an indeterminate in the optimization
      // program.
      std::ostringstream oss;
      oss << var << " is not a decision variable or an indeterminate of the "
                    "optimization program.\n";
      throw std::runtime_error(oss.str());
    }
  }
  return e.EvaluatePartial(symbolic::Environment(map_decision_vars));
}

symbolic::Polynomial MathematicalProgram::SubstituteSolution(
    const symbolic::Polynomial& p) const {
  symbolic::Environment::map map_decision_vars;
  for (const auto& var : p.decision_variables()) {
    map_decision_vars.emplace(var, GetSolution(var));
  }
  return p.EvaluatePartial(symbolic::Environment(map_decision_vars));
}

double MathematicalProgram::GetInitialGuess(
    const symbolic::Variable& decision_variable) const {
  return x_initial_guess_[FindDecisionVariableIndex(decision_variable)];
}

void MathematicalProgram::SetInitialGuess(
    const symbolic::Variable& decision_variable, double variable_guess_value) {
  x_initial_guess_(FindDecisionVariableIndex(decision_variable)) =
      variable_guess_value;
}
// Note that SetDecisionVariableValue and SetDecisionVariableValues are
// implemented in mathematical_program_api.cc instead of this file.

SolutionResult MathematicalProgram::Solve() {
  const SolverId solver_id = ChooseBestSolver(*this);
  if (solver_id == LinearSystemSolver::id()) {
    return linear_system_solver_->Solve(*this);
  } else if (solver_id == EqualityConstrainedQPSolver::id()) {
    return equality_constrained_qp_solver_->Solve(*this);
  } else if (solver_id == MosekSolver::id()) {
    return mosek_solver_->Solve(*this);
  } else if (solver_id == GurobiSolver::id()) {
    return gurobi_solver_->Solve(*this);
  } else if (solver_id == OsqpSolver::id()) {
    return osqp_solver_->Solve(*this);
  } else if (solver_id == MobyLcpSolverId::id()) {
    return moby_lcp_solver_->Solve(*this);
  } else if (solver_id == SnoptSolver::id()) {
    return snopt_solver_->Solve(*this);
  } else if (solver_id == IpoptSolver::id()) {
    return ipopt_solver_->Solve(*this);
  } else if (solver_id == NloptSolver::id()) {
    return nlopt_solver_->Solve(*this);
  } else if (solver_id == ScsSolver::id()) {
    return scs_solver_->Solve(*this);
  } else {
    throw std::runtime_error("Unknown solver.");
  }
}

void MathematicalProgram::AppendNanToEnd(int new_var_size, Eigen::VectorXd* v) {
  v->conservativeResize(v->rows() + new_var_size);
  v->tail(new_var_size).fill(std::numeric_limits<double>::quiet_NaN());
}
}  // namespace solvers
}  // namespace drake
