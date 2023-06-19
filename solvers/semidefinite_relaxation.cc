#include "drake/solvers/semidefinite_relaxation.h"

#include <initializer_list>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "drake/solvers/program_attribute.h"

namespace drake {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using symbolic::Expression;
using symbolic::Variable;

namespace {

const double kInf = std::numeric_limits<double>::infinity();

// TODO(russt): This can be replaced by vars(indices) once we have Eigen 3.4.
VectorXDecisionVariable GetVariablesByIndex(
    const Eigen::Ref<const VectorXDecisionVariable>& vars,
    std::vector<int> indices) {
  VectorXDecisionVariable new_vars(indices.size());
  for (int i = 0; i < ssize(indices); ++i) {
    new_vars[i] = vars[indices[i]];
  }
  return new_vars;
}

}  // namespace

std::unique_ptr<MathematicalProgram> MakeSemidefiniteRelaxation(
    const MathematicalProgram& prog) {
  std::string unsupported_message{};
  const ProgramAttributes supported_attributes(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kQuadraticConstraint});
  if (!AreRequiredAttributesSupported(prog.required_capabilities(),
                                      supported_attributes,
                                      &unsupported_message)) {
    throw std::runtime_error(fmt::format(
        "MakeSemidefiniteRelaxation does not (yet) support this program: {}.",
        unsupported_message));
  }

  auto relaxation = std::make_unique<MathematicalProgram>();

  // Build a symmetric matrix X of decision variables using the original
  // program variables (so that GetSolution, etc, works using the original
  // variables).
  relaxation->AddDecisionVariables(prog.decision_variables());
  MatrixX<Variable> X(prog.num_vars() + 1, prog.num_vars() + 1);
  // X = xxᵀ; x = [prog.decision_vars(); 1].
  X.topLeftCorner(prog.num_vars(), prog.num_vars()) =
      relaxation->NewSymmetricContinuousVariables(prog.num_vars(), "Y");
  X.topRightCorner(prog.num_vars(), 1) = prog.decision_variables();
  X.bottomLeftCorner(1, prog.num_vars()) =
      prog.decision_variables().transpose();
  // X(-1,-1) = 1.
  Variable one = relaxation->NewContinuousVariables<1>("one")[0];
  X(prog.num_vars(), prog.num_vars()) = one;
  relaxation->AddBoundingBoxConstraint(1, 1,
                                       X(prog.num_vars(), prog.num_vars()));
  // X ≽ 0.
  relaxation->AddPositiveSemidefiniteConstraint(X);

  auto x = X.col(prog.num_vars());

  // Returns the linear variables y in the relaxation corresponding to variables
  // y in prog.
  auto linear_vars = [&x, &prog](const VectorXDecisionVariable& prog_vars) {
    return GetVariablesByIndex(x, prog.FindDecisionVariableIndices(prog_vars));
  };

  // Returns the quadratic variables Y = yy^T in the relaxation corresponding
  // to the variables y in prog.
  auto quadratic_vars = [&X, &prog](const VectorXDecisionVariable& prog_vars) {
    const int N = prog_vars.size();
    const std::vector<int> indices =
        prog.FindDecisionVariableIndices(prog_vars);
    MatrixX<Variable> Y(N, N);
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j < N; ++j) {
        Y(i, j) = X(indices[i], indices[j]);
      }
    }
    return Y;
  };

  // Returns the {a, vars} in relaxation, such that a' vars = 0.5*tr(QY). This
  // assumes Q=Q', which is ensured by QuadraticCost and QuadraticConstraint.
  auto half_trace_QY = [&X, &prog](const Eigen::MatrixXd& Q,
                                   const VectorXDecisionVariable& prog_vars)
      -> std::pair<VectorXd, VectorX<Variable>> {
    const int N = prog_vars.size();
    const int num_vars = N * (N + 1) / 2;
    const std::vector<int> indices =
        prog.FindDecisionVariableIndices(prog_vars);
    VectorXd a = VectorXd::Zero(num_vars);
    VectorX<Variable> y(num_vars);
    int count = 0;
    for (int i = 0; i < N; ++i) {
      for (int j = 0; j <= i; ++j) {
        // tr(QY) = ∑ᵢ ∑ⱼ Qᵢⱼ Yⱼᵢ.
        a[count] = ((i == j) ? 0.5 : 1.0) * Q(i, j);
        y[count] = X(i, j);
        count++;
      }
    }
    return {a, y};
  };

  // Linear costs => Linear costs.
  for (const auto& binding : prog.linear_costs()) {
    relaxation->AddCost(Binding<LinearCost>(binding.evaluator(),
                                            linear_vars(binding.variables())));
  }
  // Quadratic costs.
  // 0.5 y'Qy + b'y + c => 0.5 tr(QY) + b'y + c
  for (const auto& binding : prog.quadratic_costs()) {
    const int N = binding.variables().size();
    const int num_vars = N + (N * (N + 1) / 2);
    std::pair<VectorXd, VectorX<Variable>> quadratic_terms =
        half_trace_QY(binding.evaluator()->Q(), binding.variables());
    VectorXd a(num_vars);
    VectorX<Variable> vars(num_vars);
    a << quadratic_terms.first, binding.evaluator()->b();
    vars << quadratic_terms.second, linear_vars(binding.variables());
    relaxation->AddLinearCost(a, binding.evaluator()->c(), vars);
  }

  // Linear constraints.
  // lb ≤ Ay ≤ ub => lb ≤ Ay ≤ ub, and
  // 0 ≤ (Ay-b)(Ay-b)ᵀ, for both b=lb and b=ub.
  // The second constraint is implemented as -bbᵀ ≤ AYAᵀ - b(Ay)ᵀ - Aybᵀ.
  for (const auto& binding : prog.linear_constraints()) {
    const MatrixX<Variable> Y = quadratic_vars(binding.variables());
    const MatrixX<Variable> y = linear_vars(binding.variables());
    relaxation->AddConstraint(binding.evaluator(), y);
    // TODO(russt): Avoid the symbolic computation here if speed is ever an
    // issue.

    // TODO(russt): Could only add the lower triangular constraints
    // (MathematicalProgram::AddLinearEqualityConstraint has this option, but
    // AddLinearConstraint does not yet).
    const MatrixX<Expression> AYAT =
        binding.evaluator()->GetDenseA() * Y *
        binding.evaluator()->GetDenseA().transpose();
    if (binding.evaluator()->lower_bound().array().isFinite().any()) {
      const auto& b = binding.evaluator()->lower_bound();
      const MatrixX<Expression> AybT =
          binding.evaluator()->get_sparse_A() * y * b.transpose();
      relaxation->AddLinearConstraint(
          AYAT - AybT - AybT.transpose(), -b * b.transpose(),
          MatrixXd::Constant(b.size(), b.size(), kInf));
    }
    if (binding.evaluator()->upper_bound().array().isFinite().any()) {
      const auto& b = binding.evaluator()->upper_bound();
      const MatrixX<Expression> AybT =
          binding.evaluator()->get_sparse_A() * y * b.transpose();
      relaxation->AddLinearConstraint(
          AYAT - AybT - AybT.transpose(), -b * b.transpose(),
          MatrixXd::Constant(b.size(), b.size(), kInf));
    }
  }

  // Linear equality constraints.
  // Ay = b => Ay = b, and 0 ≤ (Ay-b)(Ay-b)ᵀ.
  for (const auto& binding : prog.linear_equality_constraints()) {
    const MatrixX<Variable> Y = quadratic_vars(binding.variables());
    const MatrixX<Variable> y = linear_vars(binding.variables());
    relaxation->AddConstraint(binding.evaluator(), y);
    // TODO(russt): Avoid the symbolic computation here if speed is ever an
    // issue.
    const MatrixX<Expression> AYAT =
        binding.evaluator()->get_sparse_A() * Y *
        binding.evaluator()->get_sparse_A().transpose();
    const auto& b = binding.evaluator()->lower_bound();
    const MatrixX<Expression> AybT =
        binding.evaluator()->get_sparse_A() * y * b.transpose();
    relaxation->AddLinearEqualityConstraint(AYAT - AybT - AybT.transpose(),
                                            -b * b.transpose(), true);
  }

  // Quadratic constraints.
  // lb ≤ 0.5 y'Qy + b'y ≤ ub => lb ≤ 0.5 tr(QY) + b'y ≤ ub
  for (const auto& binding : prog.quadratic_constraints()) {
    const int N = binding.variables().size();
    const int num_vars = N + (N * (N + 1) / 2);
    std::pair<VectorXd, VectorX<Variable>> quadratic_terms =
        half_trace_QY(binding.evaluator()->Q(), binding.variables());
    VectorXd a(num_vars);
    VectorX<Variable> vars(num_vars);
    a << quadratic_terms.first, binding.evaluator()->b();
    vars << quadratic_terms.second, linear_vars(binding.variables());
    relaxation->AddLinearConstraint(a.transpose(),
                                    binding.evaluator()->lower_bound(),
                                    binding.evaluator()->upper_bound(), vars);
  }

  return relaxation;
}

}  // namespace solvers
}  // namespace drake
