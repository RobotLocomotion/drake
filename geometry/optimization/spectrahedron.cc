#include "drake/geometry/optimization/spectrahedron.h"

#include <limits>
#include <memory>

#include <fmt/format.h>

#include "drake/solvers/get_program_type.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::ProgramAttribute;
using solvers::ProgramAttributes;
using solvers::VectorXDecisionVariable;
using symbolic::Variable;

namespace {

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

Spectrahedron::Spectrahedron() : ConvexSet(0) {}

Spectrahedron::Spectrahedron(const MathematicalProgram& prog)
    : ConvexSet(prog.num_vars()) {
  for (const ProgramAttribute& attr : prog.required_capabilities()) {
    if (supported_attributes().count(attr) < 1) {
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

bool Spectrahedron::DoIsBounded() const {
  throw std::runtime_error(
      "Spectrahedron::IsBounded() is not implemented yet.");
}

bool Spectrahedron::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                                 double tol) const {
  return sdp_->CheckSatisfied(sdp_->GetAllConstraints(), x, tol);
}

void Spectrahedron::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  DRAKE_DEMAND(x.size() == sdp_->num_vars());
  for (const auto& binding : sdp_->GetAllConstraints()) {
    prog->AddConstraint(
        binding.evaluator(),
        GetVariablesByIndex(
            x, sdp_->FindDecisionVariableIndices(binding.variables())));
  }
}

std::vector<Binding<Constraint>>
Spectrahedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  DRAKE_DEMAND(x.size() == sdp_->num_vars());
  std::vector<Binding<Constraint>> constraints;
  const double kInf = std::numeric_limits<double>::infinity();

  // Helper function that given a binding.variables() returns the corresponding
  // subset of variables from `x` with `t` tacked on the end.
  auto stack_xt = [&x, &t, this](const VectorXDecisionVariable& bind_vars) {
    VectorXDecisionVariable xt(bind_vars.size() + 1);
    xt << GetVariablesByIndex(x, sdp_->FindDecisionVariableIndices(bind_vars)),
        t;
    return xt;
  };

  // TODO(russt): Support SparseMatrix constraints.
  for (const auto& binding : sdp_->bounding_box_constraints()) {
    // t*lb ≤ x ≤ t*ub, implemented as
    // [I,-lb]*[x;t] ≥ 0, [I,-ub]*[x;t] ≤ 0.
    VectorXDecisionVariable vars = stack_xt(binding.variables());
    MatrixXd Ab =
        MatrixXd::Identity(binding.evaluator()->num_constraints(), vars.size());
    // TODO(russt): Handle individual elements that are infinite.
    if (binding.evaluator()->lower_bound().array().isFinite().any()) {
      Ab.rightCols<1>() = -binding.evaluator()->lower_bound();
      prog->AddLinearConstraint(Ab, 0, kInf, vars);
    }
    if (binding.evaluator()->upper_bound().array().isFinite().any()) {
      Ab.rightCols<1>() = -binding.evaluator()->upper_bound();
      prog->AddLinearConstraint(Ab, -kInf, 0, vars);
    }
  }
  for (const auto& binding : sdp_->linear_equality_constraints()) {
    // Ax = t*b, implemented as
    // [A,-lb]*[x;t] == 0.
    VectorXDecisionVariable vars = stack_xt(binding.variables());
    MatrixXd Ab(binding.evaluator()->num_constraints(), vars.size());
    Ab.leftCols(binding.evaluator()->num_vars()) =
        binding.evaluator()->GetDenseA();
    Ab.rightCols<1>() = -binding.evaluator()->lower_bound();
    prog->AddLinearEqualityConstraint(Ab, 0, vars);
  }
  for (const auto& binding : sdp_->linear_constraints()) {
    // t*lb <= Ax = t*ub, implemented as
    // [A,-lb]*[x;t] ≥ 0, [A,-ub]*[x;t] ≤ 0.
    VectorXDecisionVariable vars = stack_xt(binding.variables());
    MatrixXd Ab(binding.evaluator()->num_constraints(), vars.size());
    Ab.leftCols(binding.evaluator()->num_vars()) =
        binding.evaluator()->GetDenseA();
    if (binding.evaluator()->lower_bound().array().isFinite().any()) {
      Ab.rightCols<1>() = -binding.evaluator()->lower_bound();
      prog->AddLinearConstraint(Ab, 0, kInf, vars);
    }
    if (binding.evaluator()->upper_bound().array().isFinite().any()) {
      Ab.rightCols<1>() = -binding.evaluator()->upper_bound();
      prog->AddLinearConstraint(Ab, -kInf, 0, vars);
    }
  }
  for (const auto& binding : sdp_->positive_semidefinite_constraints()) {
    // These constraints get added without modification -- a non-negative
    // scaling of the PSD cone is just the PSD cone.
    VectorXDecisionVariable vars = GetVariablesByIndex(
        x, sdp_->FindDecisionVariableIndices(binding.variables()));
    constraints.emplace_back(prog->AddConstraint(
        binding.evaluator(),
        Eigen::Map<MatrixX<Variable>>(vars.data(),
                                      binding.evaluator()->matrix_rows(),
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
  unused(prog, A, b, c, d, x, t);
  throw std::runtime_error(
      "Spectrahedron::PointInNonnegativeScalingConstraints() is not "
      "implemented yet for the case where A, b, c, and d are passed in as "
      "arguments.");
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
