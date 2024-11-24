#include "drake/geometry/optimization/spectrahedron.h"

#include <limits>
#include <memory>

#include <fmt/format.h>

#include "drake/solvers/get_program_type.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::LinearConstraint;
using solvers::MathematicalProgram;
using solvers::ProgramAttribute;
using solvers::ProgramAttributes;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Variable;

namespace {

const double kInf = std::numeric_limits<double>::infinity();

// TODO(russt): This can be replaced by vars(indices) once we have Eigen 3.4.
VectorXDecisionVariable GetVariablesByIndex(
    const Eigen::Ref<const VectorXDecisionVariable>& vars,
    const std::vector<int>& indices) {
  VectorXDecisionVariable new_vars(indices.size());
  for (int i = 0; i < ssize(indices); ++i) {
    new_vars[i] = vars[indices[i]];
  }
  return new_vars;
}

}  // namespace

Spectrahedron::Spectrahedron() : ConvexSet(0, false) {}

Spectrahedron::Spectrahedron(const MathematicalProgram& prog)
    : ConvexSet(prog.num_vars(), false) {
  for (const ProgramAttribute& attr : prog.required_capabilities()) {
    if (!supported_attributes().contains(attr)) {
      throw std::runtime_error(fmt::format(
          "Spectrahedron does not support MathematicalPrograms that require "
          "ProgramAttribute {}. If that attribute is convex, it might be "
          "possible to add that support.",
          attr));
    }
  }
  sdp_ = prog.Clone();
  // Remove any objective functions.
  for (const auto& binding : sdp_->GetAllCosts()) {
    sdp_->RemoveCost(binding);
  }
}

Spectrahedron::~Spectrahedron() = default;

const ProgramAttributes& Spectrahedron::supported_attributes() {
  static const never_destroyed<ProgramAttributes> kSupportedAttributes{
      ProgramAttributes{ProgramAttribute::kLinearCost,
                        ProgramAttribute::kLinearConstraint,
                        ProgramAttribute::kLinearEqualityConstraint,
                        ProgramAttribute::kPositiveSemidefiniteConstraint}};
  return kSupportedAttributes.access();
}

std::unique_ptr<ConvexSet> Spectrahedron::DoClone() const {
  return std::make_unique<Spectrahedron>(*this);
}

std::optional<bool> Spectrahedron::DoIsBoundedShortcut() const {
  // CSDP cannot handle an unconstrained spectrahedron (i.e. a set whose only
  // constraints are positive semidefinite constraints), so we must check for
  // that case explicitly.
  auto constraints = sdp_->GetAllConstraints();
  auto psd_constraints = sdp_->positive_semidefinite_constraints();
  if (constraints.size() == psd_constraints.size()) {
    // If there are no constraints, then the set is bounded iff its ambient
    // dimension is zero. If the ambient dimension is zero, it's caught in
    // IsBounded, so we know the set will be unbounded.
    return false;
  }
  return std::nullopt;
}

std::optional<bool> Spectrahedron::DoPointInSetShortcut(
    const Eigen::Ref<const VectorXd>& x, double tol) const {
  return sdp_->CheckSatisfied(sdp_->GetAllConstraints(), x, tol);
}

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
Spectrahedron::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  DRAKE_DEMAND(x.size() == sdp_->num_vars());
  VectorX<Variable> new_vars;
  std::vector<Binding<Constraint>> new_constraints;
  for (const auto& binding : sdp_->GetAllConstraints()) {
    new_constraints.push_back(prog->AddConstraint(
        binding.evaluator(),
        GetVariablesByIndex(
            x, sdp_->FindDecisionVariableIndices(binding.variables()))));
  }
  return {std::move(new_vars), std::move(new_constraints)};
}

std::vector<Binding<Constraint>>
Spectrahedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  DRAKE_DEMAND(x.size() == sdp_->num_vars());
  std::vector<Binding<Constraint>> constraints;

  // Helper function that given a binding.variables() returns the corresponding
  // subset of variables from `x` with `t` tacked on the end.
  auto stack_xt = [&x, &t, this](const VectorXDecisionVariable& bind_vars) {
    VectorXDecisionVariable xt(bind_vars.size() + 1);
    xt << GetVariablesByIndex(x, sdp_->FindDecisionVariableIndices(bind_vars)),
        t;
    return xt;
  };

  // TODO(russt): Support SparseMatrix constraints.
  for (const auto& binding : sdp_->linear_equality_constraints()) {
    // Ax = t*b, implemented as
    // [A,-b]*[x;t] == 0.
    VectorXDecisionVariable vars = stack_xt(binding.variables());
    MatrixXd Ab(binding.evaluator()->num_constraints(), vars.size());
    Ab.leftCols(binding.evaluator()->num_vars()) =
        binding.evaluator()->get_sparse_A();
    Ab.rightCols<1>() = -binding.evaluator()->lower_bound();

    VectorXd zeros = VectorXd::Zero(binding.evaluator()->num_constraints());
    constraints.emplace_back(
        prog->AddLinearEqualityConstraint(Ab, zeros, vars));
  }

  std::vector<Binding<LinearConstraint>> linear_inequality_constraints =
      sdp_->linear_constraints();
  // Treat bounding box constraints as general linear inequality constraints.
  linear_inequality_constraints.insert(linear_inequality_constraints.end(),
                                       sdp_->bounding_box_constraints().begin(),
                                       sdp_->bounding_box_constraints().end());
  for (const auto& binding : linear_inequality_constraints) {
    // t*lb ≤ Ax ≤ t*ub, implemented as
    // [A,-lb]*[x;t] ≥ 0, [A,-ub]*[x;t] ≤ 0.
    VectorXDecisionVariable vars = stack_xt(binding.variables());
    MatrixXd Ab(binding.evaluator()->num_constraints(), vars.size());
    Ab.leftCols(binding.evaluator()->num_vars()) =
        binding.evaluator()->get_sparse_A();
    if (binding.evaluator()->lower_bound().array().isFinite().any()) {
      Ab.rightCols<1>() = -binding.evaluator()->lower_bound();

      VectorXd zeros = VectorXd::Zero(binding.evaluator()->num_constraints());
      VectorXd infs =
          VectorXd::Constant(binding.evaluator()->num_constraints(), kInf);

      constraints.emplace_back(
          prog->AddLinearConstraint(Ab, zeros, infs, vars));
    }
    if (binding.evaluator()->upper_bound().array().isFinite().any()) {
      Ab.rightCols<1>() = -binding.evaluator()->upper_bound();

      VectorXd zeros = VectorXd::Zero(binding.evaluator()->num_constraints());
      VectorXd infs =
          VectorXd::Constant(binding.evaluator()->num_constraints(), kInf);

      constraints.emplace_back(
          prog->AddLinearConstraint(Ab, -infs, zeros, vars));
    }
  }

  for (const auto& binding : sdp_->positive_semidefinite_constraints()) {
    // These constraints get added without modification -- a non-negative
    // scaling of the PSD cone is just the PSD cone.
    VectorXDecisionVariable vars = GetVariablesByIndex(
        x, sdp_->FindDecisionVariableIndices(binding.variables()));
    constraints.emplace_back(prog->AddConstraint(
        binding.evaluator(),
        Map<MatrixX<Variable>>(vars.data(), binding.evaluator()->matrix_rows(),
                               binding.evaluator()->matrix_rows())));
  }
  return constraints;
}

std::vector<Binding<Constraint>>
Spectrahedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A,
    const Eigen::Ref<const VectorXd>& b, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  DRAKE_DEMAND(A.rows() == this->ambient_dimension());
  DRAKE_DEMAND(A.cols() == x.size());
  DRAKE_DEMAND(b.size() == this->ambient_dimension());
  DRAKE_DEMAND(c.size() == t.size());
  std::vector<Binding<Constraint>> constraints;

  // Helper function that extracts the rows of A and b that are relevant for
  // this binding.
  auto binding_Ab = [&A, &b, this](const VectorXDecisionVariable& bind_vars) {
    std::vector<int> indices = sdp_->FindDecisionVariableIndices(bind_vars);
    MatrixXd this_A(bind_vars.size(), A.cols());
    VectorXd this_b(bind_vars.size());
    for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
      const int index = indices[i];
      this_A.row(i) = A.row(index);
      this_b(i) = b(index);
    }
    return std::pair(this_A, this_b);
  };

  MatrixXd this_A;
  VectorXd this_b;

  for (const auto& binding : sdp_->linear_equality_constraints()) {
    // A_c (A x + b) = (c' t + d) b_c, written as
    // [A_c * A, -b_c * c'][x;t] = d * b_c - A_c * b
    // A̅ [x;t] = b̅,
    const int num_constraints = binding.evaluator()->num_constraints();
    std::tie(this_A, this_b) = binding_Ab(binding.variables());
    MatrixXd A_bar(num_constraints, x.size() + t.size());
    A_bar.leftCols(x.size()) = binding.evaluator()->get_sparse_A() * this_A;
    A_bar.rightCols(t.size()) =
        -binding.evaluator()->upper_bound() * c.transpose();
    VectorXd b_bar = d * binding.evaluator()->upper_bound() -
                     binding.evaluator()->get_sparse_A() * this_b;
    constraints.emplace_back(
        prog->AddLinearEqualityConstraint(A_bar, b_bar, {x, t}));
  }

  std::vector<Binding<LinearConstraint>> linear_inequality_constraints =
      sdp_->linear_constraints();
  // Treat bounding box constraints as general linear inequality constraints.
  linear_inequality_constraints.insert(linear_inequality_constraints.end(),
                                       sdp_->bounding_box_constraints().begin(),
                                       sdp_->bounding_box_constraints().end());
  for (const auto& binding : linear_inequality_constraints) {
    // (c' t + d) b_c ≤ A_c (A x + b), written as
    // d * b_c - A_c * b ≤ [A_c * A, -b_c * c'][x;t]
    // b̅ ≤ A̅ [x;t]
    const int num_constraints = binding.evaluator()->num_constraints();
    std::tie(this_A, this_b) = binding_Ab(binding.variables());
    MatrixXd A_bar(num_constraints, x.size() + t.size());
    A_bar.leftCols(x.size()) = binding.evaluator()->get_sparse_A() * this_A;
    VectorXd b_bar = d * binding.evaluator()->lower_bound() -
                     binding.evaluator()->get_sparse_A() * this_b;
    if (b_bar.array().isFinite().any()) {
      A_bar.rightCols(t.size()) =
          -binding.evaluator()->lower_bound() * c.transpose();
      constraints.emplace_back(prog->AddLinearConstraint(
          A_bar, b_bar, VectorXd::Constant(num_constraints, kInf), {x, t}));
    }
    // Then again for the upper bound
    // A_c (A x + b) ≤ (c' t + d) b_c, written as
    // [A_c * A, -b_c * c'][x;t] ≤ d * b_c - A_c * b
    // A̅ [x;t] ≤ b̅,
    b_bar = d * binding.evaluator()->upper_bound() -
            binding.evaluator()->get_sparse_A() * this_b;
    if (b_bar.array().isFinite().any()) {
      A_bar.rightCols(t.size()) =
          -binding.evaluator()->upper_bound() * c.transpose();
      constraints.emplace_back(prog->AddLinearConstraint(
          A_bar, VectorXd::Constant(num_constraints, -kInf), b_bar, {x, t}));
    }
  }

  for (const auto& binding : sdp_->positive_semidefinite_constraints()) {
    // A * x + b ∈ (c' * t + d) S => reshaped(A * x + b) is PSD
    std::tie(this_A, this_b) = binding_Ab(binding.variables());
    VectorX<Expression> Axplusb = this_A * x + this_b;
    const int num_rows = binding.evaluator()->matrix_rows();
    Map<MatrixX<Expression>> S(Axplusb.data(), num_rows, num_rows);
    constraints.emplace_back(prog->AddLinearMatrixInequalityConstraint(S));
  }
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
Spectrahedron::DoToShapeWithPose() const {
  // I could potentially visualize the 2x2 case in three dimensions (as a mesh
  // if nothing else).
  throw std::runtime_error(
      "ToShapeWithPose is not supported by Spectrahedron.");
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
