#include "drake/solvers/conic_standard_forms.h"

#include "drake/common/never_destroyed.h"
#include "drake/common/ssize.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
using Eigen::MatrixXd;
using Eigen::VectorXd;
using symbolic::Variable;
const double kInf = std::numeric_limits<double>::infinity();

namespace {
// If the program is compatible with conic standard forms.
bool CheckSupported(const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          // Supported Constraints.
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kPositiveSemidefiniteConstraint,
          // Supported Costs.
          ProgramAttribute::kLinearCost,
          //          ProgramAttribute::kQuadraticCost,
          //          ProgramAttribute::kL2NormCost
      });
  return internal::CheckConvexSolverAttributes(
      prog, solver_capabilities.access(), "ConicStandardForm", nullptr);
}

// Given a mathematical program with convex cost, convert all costs to epigraph
// form. For example, if the program has the convex quadratic cost min xᵀQx
// we recast is as
// min         t
// subject to xᵀQx ≤ t
void ConvertAllNonLinearCostsToEpigraph(MathematicalProgram* prog) {
  const std::vector<Binding<Cost>> original_costs{prog->GetAllCosts()};
  // Contains all the epigraph variables
  std::vector<Variable> epigraph_variables;
  // Contains all coefficients of the epigraph variables in the cost.
  std::vector<double> cost_vect;
  // This is an upper bound on the number of epigraph variables.
  epigraph_variables.reserve(original_costs.size());
  cost_vect.reserve(original_costs.size());

  auto remove_cost = [&epigraph_variables,
                      &prog](const Binding<Cost>& binding) {
    prog->RemoveCost(binding);
    epigraph_variables.emplace_back(
        Variable("t" + std::to_string(epigraph_variables.size())));
  } for (const auto& binding : prog->quadratic_costs()) {
    remove_cost(binding);
    const auto& quadratic_cost = binding.evaluator().get();
    const int n = quadratic_cost->Q().rows() + 1;
    Eigen::MatrixXd Q_new = Eigen::MatrixXd::Zero(n, n);
    Q_new.topLeftCorner(n - 1, n - 1) = quadratic_cost->Q();
    Eigen::VectorXd b_new{n};
    b_new.head(n - 1) = quadratic_cost->b();
    b_new(n - 1) = -1;
    VectorX<Variable> vars(n);
    vars.head(n - 1) = binding.variables();
    vars(n - 1) = *epigraph_variables.crbegin();
    prog->AddQuadraticAsRotatedLorentzConeConstraint(Q_new, b_new,
                                                     quadratic_cost->c(), vars);
    cost_vect.emplace_back(1);
  }
  for (const auto& binding : prog->l2norm_costs()) {
    remove_cost(binding);
    const auto& l2_norm_cost = binding.evaluator().get();

    //  ||Ax+b||₂ ≤ t is equivalent to
    //  0.5 [x,-t]ᵀ[[2AᵀA, 0],[0,2]][x,-t] + [2b, 0][x, -t] + bᵀb ≤ 0
    const int n = l2_norm_cost->get_sparse_A().rows() + 1;
    Eigen::MatrixXd Q_new = Eigen::MatrixXd::Zero(n, n);
    Q_new.topLeftCorner(n - 1, n - 1) =
        2 * l2_norm_cost->get_sparse_A().transpose() *
        l2_norm_cost->get_sparse_A();
    Q_new(n - 1, n - 1) = 2;
    Eigen::VectorXd b_new{n};
    b_new.head(n - 1) = 2 * l2_norm_cost->b();
    b_new(n - 1) = 0;
    const double c = (l2_norm_cost->b() * l2_norm_cost->b())(0);
    VectorX<Variable> vars(n);
    vars.head(n - 1) = binding.variables();
    vars(n - 1) = *epigraph_variables.crbegin();
    prog->AddQuadraticAsRotatedLorentzConeConstraint(Q_new, b_new, c, vars);
    //  Notice that we need to use -t to make the hessian PSD. Therefore,
    //  we need to maximize -t.
    cost_vect.emplace_back(-1);
  }
  const Eigen::Map<VectorX<Variable>> epigraph_variables_eigen{
      epigraph_variables.data(), ssize(epigraph_variables)};
  const Eigen::Map<Eigen::VectorXd> cost{cost_vect.data(), ssize(cost_vect)};
  prog->AddDecisionVariables(epigraph_variables_eigen);
  prog->AddLinearCost(cost, epigraph_variables_eigen);
}  // namespace

std::unordered_map<Binding<L2NormCost>, symbolic::Variable>
ParseL2NormCostsToEpigraphForm(MathematicalProgram* prog) {
  // Loop through all unsupported costs and rewrite them into constraints.
  std::unordered_map<Binding<L2NormCost>, symbolic::Variable> costs_to_slack;
  auto slacks =
      prog->NewContinuousVariables(costs_to_slack.size(), "l2_norm_cost_slack");
  int ctr{0};
  for (const auto& binding : prog->l2norm_costs()) {
    const int m = binding.evaluator()->get_sparse_A().rows();
    const int n = binding.evaluator()->get_sparse_A().cols();
    const Vector<Variable, 1> cur_slack(slacks(ctr++));
    prog->AddLinearCost(Vector1d::Ones(), cur_slack);
    // |Ax+b|² ≤ slack, written as a Lorentz cone with z = [slack; Ax+b].
    MatrixXd A = MatrixXd::Zero(m + 1, n + 1);
    A(0, 0) = 1;
    A.bottomRightCorner(m, n) = binding.evaluator()->GetDenseA();
    VectorXd b(m + 1);
    b << 0, binding.evaluator()->b();
    prog->AddLorentzConeConstraint(A, b, {cur_slack, binding.variables()});
    costs_to_slack.insert({binding, cur_slack(0)});
  }
  return costs_to_slack;
};
//
// std::map<Binding<L1NormCost>, symbolic::Variable>
// ParseL1NormCostsToEpigraphForm(MathematicalProgram* prog) {
//  std::unordered_map<Binding<L1NormCost>, symbolic::Variable> costs_to_slack;
//  auto slacks =
//      prog->NewContinuousVariables(costs_to_slack.size(),
//      "l1_norm_cost_slack");
//  for (const auto &binding: prog->generic_costs()) {
//    const Cost *cost = binding.evaluator().get();
//    if (const auto *l1c = dynamic_cast<const L1NormCost *>(cost)) {
//      const int m = l1c->A().rows();
//      const int n = l1c->A().cols();
//      auto slack = prog->NewContinuousVariables(m, "slack");
//      prog->AddLinearCost(VectorXd::Ones(m), slack);
//      // Ax + b ≤ slack, written as [A,-I][x;slack] ≤ -b.
//      MatrixXd A = MatrixXd::Zero(m, m + n);
//      A << l1c->A(), -MatrixXd::Identity(m, m);
//      prog->AddLinearConstraint(A, VectorXd::Constant(m, -kInf), -l1c->b(),
//                                {binding.variables(), slack});
//      // -(Ax + b) ≤ slack, written as [A,I][x;slack] ≥ -b.
//      A.rightCols(m) = MatrixXd::Identity(m, m);
//      prog->AddLinearConstraint(A, -l1c->b(), VectorXd::Constant(m, kInf),
//                                {binding.variables(), slack});
//      to_remove.insert(binding);
//    }
//  }
//};
// std::map<Binding<QuadraticCost>, symbolic::Variable>
// ParseQuadraticCostsToEpigraphForm(MathematicalProgram* prog);
//
//{
//  // Use Mosek's requirements to test the program attributes.
//  solvers::MosekSolver mosek;
//  if (mosek.AreProgramAttributesSatisfied(*prog)) {
//    return;
//  }
//
//  const double kInf = std::numeric_limits<double>::infinity();
//
//  // Loop through all unsupported costs and rewrite them into constraints.
//  std::unordered_set<Binding<Cost>> to_remove;
//
//  for (const auto& binding : prog->l2norm_costs()) {
//    const int m = binding.evaluator()->A().rows();
//    const int n = binding.evaluator()->A().cols();
//    auto slack = prog->NewContinuousVariables<1>("slack");
//    prog->AddLinearCost(Vector1d::Ones(), slack);
//    // |Ax+b|² ≤ slack, written as a Lorentz cone with z = [slack; Ax+b].
//    MatrixXd A = MatrixXd::Zero(m + 1, n + 1);
//    A(0, 0) = 1;
//    A.bottomRightCorner(m, n) = binding.evaluator()->A();
//    VectorXd b(m + 1);
//    b << 0, binding.evaluator()->b();
//    prog->AddLorentzConeConstraint(A, b, {slack, binding.variables()});
//    to_remove.insert(binding);
//  }
//
//  for (const auto& binding : prog->generic_costs()) {
//    const Cost* cost = binding.evaluator().get();
//    if (const auto* l1c = dynamic_cast<const L1NormCost*>(cost)) {
//      const int m = l1c->A().rows();
//      const int n = l1c->A().cols();
//      auto slack = prog->NewContinuousVariables(m, "slack");
//      prog->AddLinearCost(VectorXd::Ones(m), slack);
//      // Ax + b ≤ slack, written as [A,-I][x;slack] ≤ -b.
//      MatrixXd A = MatrixXd::Zero(m, m + n);
//      A << l1c->A(), -MatrixXd::Identity(m, m);
//      prog->AddLinearConstraint(A, VectorXd::Constant(m, -kInf), -l1c->b(),
//                                {binding.variables(), slack});
//      // -(Ax + b) ≤ slack, written as [A,I][x;slack] ≥ -b.
//      A.rightCols(m) = MatrixXd::Identity(m, m);
//      prog->AddLinearConstraint(A, -l1c->b(), VectorXd::Constant(m, kInf),
//                                {binding.variables(), slack});
//      to_remove.insert(binding);
//    } else if (const auto* linfc = dynamic_cast<const LInfNormCost*>(cost))
//    {
//      const int m = linfc->A().rows();
//      const int n = linfc->A().cols();
//      auto slack = prog->NewContinuousVariables<1>("slack");
//      prog->AddLinearCost(Vector1d::Ones(), slack);
//      // ∀i, aᵢᵀx + bᵢ ≤ slack, written as [A,-1][x;slack] ≤ -b.
//      MatrixXd A = MatrixXd::Zero(m, n + 1);
//      A << linfc->A(), VectorXd::Constant(m, -1);
//      prog->AddLinearConstraint(A, VectorXd::Constant(m, -kInf),
//      -linfc->b(),
//                                {binding.variables(), slack});
//      // ∀i, -(aᵢᵀx + bᵢ) ≤ slack, written as [A,1][x;slack] ≥ -b.
//      A.col(A.cols() - 1) = VectorXd::Ones(m);
//      prog->AddLinearConstraint(A, -linfc->b(), VectorXd::Constant(m, kInf),
//                                {binding.variables(), slack});
//      to_remove.insert(binding);
//    } else if (const auto* pqc =
//                   dynamic_cast<const PerspectiveQuadraticCost*>(cost)) {
//      const int m = pqc->A().rows();
//      const int n = pqc->A().cols();
//      auto slack = prog->NewContinuousVariables<1>("slack");
//      prog->AddLinearCost(Vector1d::Ones(), slack);
//      // Written as rotated Lorentz cone with z = [slack; Ax+b].
//      MatrixXd A = MatrixXd::Zero(m + 1, n + 1);
//      A(0, 0) = 1;
//      A.bottomRightCorner(m, n) = pqc->A();
//      VectorXd b(m + 1);
//      b << 0, pqc->b();
//      prog->AddRotatedLorentzConeConstraint(A, b, {slack,
//      binding.variables()}); to_remove.insert(binding);
//    }
//  }

//  for (const auto& b : to_remove) {
//    prog->RemoveCost(b);
//  }
//
//  if (!mosek.AreProgramAttributesSatisfied(*prog)) {
//    throw std::runtime_error(fmt::format(
//        "SolveConvexRestriction failed to generate a convex problem: {}",
//        mosek.ExplainUnsatisfiedProgramAttributes(*prog)));
//  }
//}

std::unique_ptr<MathematicalProgram> ParseToConicStandardForm(
    const MathematicalProgram& original_prog) {
  std::unique_ptr<MathematicalProgram> prog = original_prog.Clone();
  CheckSupported(*prog);
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b;
  Eigen::SparseMatrix<double> Aeq;
  Eigen::VectorXd beq;
  AggregateConvexConstraints(prog, &A, &b, &Aeq, &beq);

  CastAllNonLinearCostsToEpigraph(prog.get());

  //  int num_x = original_prog.num_vars();
  // `c` is the coefficient in the linear cost cᵀx+d
  Eigen::SparseVector<double> c;
  double d;
  VectorX<symbolic::Variable> vars;
  AggregateLinearCosts(prog->linear_costs(), &c, &vars, &d);

  // We will build this sparse matrix from the triplets recording its non-zero
  // entries.
  std::vector<Eigen::Triplet<double>> A_triplets;
  // A_row_count will increment, when we add each constraint.
  int A_row_count = 0;
  std::vector<double> b;

  // Parse the linear constraints
  std::vector<std::vector<std::pair<int, int>>>
      unused_linear_constraint_dual_indices;
  int num_linear_constraint_rows = 0;
  internal::ParseLinearConstraints(*prog, &A_triplets, &b, &A_row_count,
                                   &unused_linear_constraint_dual_indices,
                                   &num_linear_constraint_rows);

  // Parse Lorentz cone and rotated Lorentz cone constraint
  std::vector<int> second_order_cone_length;
  // y[lorentz_cone_y_start_indices[i]:
  //   lorentz_cone_y_start_indices[i] + second_order_cone_length[i]]
  // are the dual variables for prog.lorentz_cone_constraints()[i].
  std::vector<int> lorentz_cone_y_start_indices;
  std::vector<int> rotated_lorentz_cone_y_start_indices;
  internal::ParseSecondOrderConeConstraints(
      prog, &A_triplets, &b, &A_row_count, &second_order_cone_length,
      &lorentz_cone_y_start_indices, &rotated_lorentz_cone_y_start_indices);
  for (const int soc_length : second_order_cone_length) {
    cones.push_back(clarabel::SecondOrderConeT<double>(soc_length));

    return prog;
  }
}  // namespace solvers
}  // namespace drake