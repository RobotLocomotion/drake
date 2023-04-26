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
using solvers::VectorXDecisionVariable;
using symbolic::Variable;


Spectrahedron::Spectrahedron()
    : ConvexSet(&ConvexSetCloner<Spectrahedron>, 0) {}

Spectrahedron::Spectrahedron(const MathematicalProgram& prog)
    : ConvexSet(&ConvexSetCloner<Spectrahedron>, prog.num_vars()) {
  DRAKE_DEMAND(solvers::GetProgramType(prog) == solvers::ProgramType::kSDP);
  sdp_ = prog.Clone();
  // Remove any objective functions.
  for (const auto& binding : sdp_->GetAllCosts()) {
    sdp_->RemoveCost(binding);
  }
}

Spectrahedron::~Spectrahedron() = default;

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
        x(sdp_->FindDecisionVariableIndices(binding.variables())));
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

  // TODO(russt): Support SparseMatrix constraints.
  for (const auto& binding : sdp_->bounding_box_constraints()) {
    // t*lb ≤ x ≤ t*ub, implemented as
    // [I,-lb]*[x;t] ≥ 0, [I,-ub]*[x;t] ≤ 0.
    VectorXDecisionVariable vars(binding.evaluator()->num_vars()+1);
    vars << x(sdp_->FindDecisionVariableIndices(binding.variables())), t;
    MatrixXd Ab = MatrixXd::Identity(binding.evaluator()->num_constraints(),
                binding.evaluator()->num_vars() + 1);
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
    VectorXDecisionVariable vars(binding.evaluator()->num_vars()+1);
    vars << x(sdp_->FindDecisionVariableIndices(binding.variables())), t;
    MatrixXd Ab(binding.evaluator()->num_constraints(),
                binding.evaluator()->num_vars() + 1);
    Ab.leftCols(binding.evaluator()->num_vars()) =
        binding.evaluator()->GetDenseA();
    Ab.rightCols<1>() = -binding.evaluator()->lower_bound();
    prog->AddLinearConstraint(Ab, 0, 0, vars);
  }
  for (const auto& binding : sdp_->linear_constraints()) {
    // t*lb <= Ax = t*ub, implemented as
    // [A,-lb]*[x;t] ≥ 0, [A,-ub]*[x;t] ≤ 0.
    VectorXDecisionVariable vars(binding.evaluator()->num_vars()+1);
    vars << x(sdp_->FindDecisionVariableIndices(binding.variables())), t;
    MatrixXd Ab(binding.evaluator()->num_constraints(),
                binding.evaluator()->num_vars() + 1);
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
  for (const auto& binding : sdp_->lorentz_cone_constraints()) {
    unused(binding);
    throw std::runtime_error(
        "Spectrahedron::PointInNonnegativeScalingConstraints() is not "
        "implemented yet for Lorentz cone constraints.");
  }
  for (const auto& binding : sdp_->rotated_lorentz_cone_constraints()) {
    unused(binding);
    throw std::runtime_error(
        "Spectrahedron::PointInNonnegativeScalingConstraints() is not "
        "implemented yet for rotated Lorentz cone constraints.");
  }
  for (const auto& binding : sdp_->positive_semidefinite_constraints()) {
    // These constraints get added without modification -- a non-negative
    // scaling of the PSD cone is just the PSD cone.
    constraints.emplace_back(prog->AddConstraint(
        binding.evaluator(),
        x(sdp_->FindDecisionVariableIndices(binding.variables()))
            .reshaped(binding.evaluator()->matrix_rows(),
                      binding.evaluator()->matrix_rows())));
  }
  return constraints;
}

std::vector<Binding<Constraint>>
Spectrahedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A_x,
    const Eigen::Ref<const VectorXd>& b, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  throw std::runtime_error(
      "Spectrahedron::PointInNonnegativeScalingConstraints() is not "
      "implemented yet for the case where A_x, b, c, and d are passed in as "
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
