#include "drake/geometry/optimization/convex_set.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "drake/solvers/solution_result.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

ConvexSet::ConvexSet(
    std::function<std::unique_ptr<ConvexSet>(const ConvexSet&)> cloner,
    int ambient_dimension)
    : cloner_(std::move(cloner)), ambient_dimension_(ambient_dimension) {
  DRAKE_DEMAND(ambient_dimension >= 0);
}

ConvexSet::~ConvexSet() = default;

std::unique_ptr<ConvexSet> ConvexSet::Clone() const { return cloner_(*this); }

bool ConvexSet::IntersectsWith(const ConvexSet& other) const {
  DRAKE_THROW_UNLESS(other.ambient_dimension() == this->ambient_dimension());
  solvers::MathematicalProgram prog{};
  const auto& x = prog.NewContinuousVariables(this->ambient_dimension(), "x");
  this->AddPointInSetConstraints(&prog, x);
  other.AddPointInSetConstraints(&prog, x);
  solvers::MathematicalProgramResult result = solvers::Solve(prog);
  return result.is_success();
}

std::vector<solvers::Binding<solvers::Constraint>>
ConvexSet::AddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
  DRAKE_DEMAND(x.size() == ambient_dimension());
  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      DoAddPointInNonnegativeScalingConstraints(prog, x, t);
  constraints.emplace_back(prog->AddBoundingBoxConstraint(
      0, std::numeric_limits<double>::infinity(), t));
  return constraints;
}

std::vector<solvers::Binding<solvers::Constraint>>
ConvexSet::AddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::RowVectorXd>& b,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const {
  DRAKE_DEMAND(A.rows() == ambient_dimension());
  DRAKE_DEMAND(A.cols() == x.size());
  DRAKE_DEMAND(b.cols() == t.size());
  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      DoAddPointInNonnegativeScalingConstraints(prog, A, x, b, t);
  constraints.emplace_back(prog->AddLinearConstraint(
      b, 0, std::numeric_limits<double>::infinity(), t));
  return constraints;
}

std::vector<solvers::Binding<solvers::Constraint>>
ConvexSet::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const Eigen::Ref<const Eigen::RowVectorXd>& b,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const {
  const int n = ambient_dimension();
  // x' ∈ t' S
  solvers::VectorXDecisionVariable x_scaled =
      prog->NewContinuousVariables(n, "x_scaled");
  symbolic::Variable t_scaled = prog->NewContinuousVariables<1>("t_scaled")[0];
  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      DoAddPointInNonnegativeScalingConstraints(prog, x_scaled, t_scaled);
  // x' = A * x
  Eigen::MatrixXd Ax_scaled(n, n + x.size());
  Ax_scaled.leftCols(n) = Eigen::MatrixXd::Identity(n, n);
  Ax_scaled.rightCols(x.size()) = -A;
  constraints.emplace_back(prog->AddLinearEqualityConstraint(
      Ax_scaled, Eigen::VectorXd::Zero(n), {x_scaled, x}));
  // t' = b * t
  Eigen::RowVectorXd bt_scaled(1 + t.size());
  bt_scaled[0] = 1.0;
  bt_scaled.tail(t.size()) = -b;
  constraints.emplace_back(prog->AddLinearEqualityConstraint(
      bt_scaled, Vector1d::Zero(), {Vector1<symbolic::Variable>(t_scaled), t}));
  return constraints;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
