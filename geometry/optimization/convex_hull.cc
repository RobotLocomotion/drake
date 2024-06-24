#include "drake/geometry/optimization/convex_hull.h"

#include <limits>
#include <memory>

#include "drake/solvers/solve.h"

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

int GetAmbientDimension(const ConvexSets& sets) {
  if (sets.empty()) {
    return 0;
  }
  const int ambient_dimension = sets[0]->ambient_dimension();
  for (const copyable_unique_ptr<ConvexSet>& set : sets) {
    DRAKE_THROW_UNLESS(set != nullptr);
    DRAKE_THROW_UNLESS(set->ambient_dimension() == ambient_dimension);
  }
  return ambient_dimension;
}

std::vector<symbolic::Variable> AddNewVariables(
    const std::vector<solvers::Binding<solvers::Constraint>>& bindings,
    std::vector<symbolic::Variable>* existing_variables) {
  std::vector<symbolic::Variable> new_vars;
  for (const auto& binding : bindings) {
    const auto& vars = binding.variables();
    for (int i = 0; i < vars.size(); ++i) {
      const auto& var = vars(i);
      // If the variable is not in the existing_variables, then add it to
      if (std::any_of(existing_variables->begin(), existing_variables->end(),
                      [&var](const symbolic::Variable& v) {
                        return v.equal_to(var);
                      })) {
        continue;
      }
      new_vars.push_back(var);
      existing_variables->push_back(var);
    }
  }
  return new_vars;
}

}  // namespace

ConvexHull::ConvexHull(const ConvexSets& sets)
    : ConvexSet(GetAmbientDimension(sets), false), sets_(sets) {}

ConvexHull::~ConvexHull() = default;

const ConvexSet& ConvexHull::element(int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < std::ssize(sets_));
  return *sets_[index];
}

std::unique_ptr<ConvexSet> ConvexHull::DoClone() const {
  return std::make_unique<ConvexHull>(*this);
}

std::optional<bool> ConvexHull::DoIsBoundedShortcut() const {
  // The convex hull is bounded if and only if all the participating sets are
  // bounded.
  for (const auto& set : sets_) {
    if (!set->IsBounded()) {
      return false;
    }
  }
  return true;
}

bool ConvexHull::DoIsEmpty() const {
  if (sets_.empty()) {
    return true;
  }
  // The convex hull is empty if one of the participating sets is empty.
  for (const auto& set : sets_) {
    if (set->IsEmpty()) {
      return true;
    }
  }
  return false;
}

std::optional<VectorXd> ConvexHull::DoMaybeGetPoint() const {
  if (IsEmpty()) {
    return std::nullopt;
  }
  Eigen::VectorXd sum = Eigen::VectorXd::Zero(ambient_dimension());
  for (const auto& set : sets_) {
    const auto maybe_point = set->MaybeGetPoint();
    if (maybe_point.has_value()) {
      return maybe_point;
    }
  }
  return std::nullopt;
}

bool ConvexHull::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                              double tol) const {
  // Check the feasibility of |x - ∑ᵢ xᵢ| <= tol*1, xᵢ ∈ 𝛼ᵢXᵢ, 𝛼ᵢ ≥ 0, ∑ᵢ 𝛼ᵢ =
  // 1, where 1 is a vector of ones and |.| is interpreted element-wise.
  MathematicalProgram prog;
  const int n = std::ssize(sets_);
  const int d = ambient_dimension();
  auto xz = prog.NewContinuousVariables(n + 1, d, "x");
  auto alpha = prog.NewContinuousVariables(n, "alpha");
  // constraint I: x - ∑ᵢ xᵢ <= z -->  ∑ᵢ xᵢ + z >= x
  // constraint II: -x + ∑ᵢ xᵢ <= z -->  ∑ᵢ xᵢ - z <= x
  Eigen::VectorXd a_1 = Eigen::VectorXd::Ones(n + 1);
  Eigen::VectorXd a_2{a_1};
  a_2(n) = -1;
  // constraint I: inf >= A_1 * xz >= x
  const double inf = std::numeric_limits<double>::infinity();
  for (int i = 0; i < d; ++i) {
    prog.AddLinearConstraint(a_1, x(i), inf, xz.col(i));
    prog.AddLinearConstraint(a_2, -inf, x(i), xz.col(i));
  }
  // constraint: ∑ᵢ αᵢ = 1
  prog.AddLinearEqualityConstraint(Eigen::MatrixXd::Ones(1, n),
                                   VectorXd::Ones(1), alpha);
  // constraint: 1 > αᵢ >= 0
  prog.AddBoundingBoxConstraint(0, 1, alpha);
  // constraint: |z| <= tol
  auto z = xz.row(n);
  prog.AddBoundingBoxConstraint(-tol, tol, z);
  // add the constraints for each convex set
  for (int i = 0; i < n; ++i) {
    sets_[i]->AddPointInNonnegativeScalingConstraints(&prog, xz.row(i),
                                                      alpha(i));
  }
  // check the feasibility
  const auto result = solvers::Solve(prog);
  return result.is_success();
}

std::pair<VectorX<symbolic::Variable>,
          std::vector<solvers::Binding<solvers::Constraint>>>
ConvexHull::DoAddPointInSetConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const {
  // Add the constraint that vars = ∑ᵢ xᵢ, xᵢ ∈ 𝛼ᵢXᵢ, 𝛼ᵢ ≥ 0, ∑ᵢ 𝛼ᵢ = 1.
  const int n = std::ssize(sets_);
  const int d = ambient_dimension();
  // The new variable is n * d, each row is a variable for a convex set
  auto x = prog->NewContinuousVariables(n, d, "x");
  auto alpha = prog->NewContinuousVariables(n, "alpha");
  prog->AddBoundingBoxConstraint(0, 1, alpha);
  std::vector<solvers::Binding<solvers::Constraint>> new_bindings;
  // constraint: x - ∑ᵢ xᵢ == 0
  Eigen::VectorXd a(n + 1);
  a.head(n) = Eigen::VectorXd::Ones(n);
  a(n) = -1;
  for (int i = 0; i < d; ++i) {
    Eigen::Ref<const solvers::VectorXDecisionVariable> vars_i =
        vars.segment(i, 1);
    solvers::VariableRefList vars_list{x.col(i), vars_i};
    new_bindings.push_back(prog->AddLinearEqualityConstraint(a, 0, vars_list));
  }
  // constraint: ∑ᵢ αᵢ = 1
  new_bindings.push_back(prog->AddLinearEqualityConstraint(
      Eigen::MatrixXd::Ones(1, n), VectorXd::Ones(1), alpha));
  // alpha and x should already be added
  auto new_vars = std::vector<symbolic::Variable>(alpha.data(),
                                                  alpha.data() + alpha.size());
  new_vars.insert(new_vars.end(), x.data(), x.data() + x.size());
  // add the constraints for each convex set
  for (int i = 0; i < n; ++i) {
    auto cons = sets_[i]->AddPointInNonnegativeScalingConstraints(
        prog, x.row(i), alpha(i));
    new_bindings.insert(new_bindings.end(), cons.begin(), cons.end());
    AddNewVariables(cons, &new_vars);
  }
  return std::make_pair(std::move(Eigen::Map<VectorX<symbolic::Variable>>(
                            new_vars.data(), new_vars.size())),
                        std::move(new_bindings));
}

std::vector<solvers::Binding<solvers::Constraint>>
ConvexHull::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars,
    const symbolic::Variable& t) const {
  // Add the constraint that t >= 0, x = ∑ᵢ xᵢ, xᵢ ∈ 𝛼ᵢXᵢ, 𝛼ᵢ ≥ 0, ∑ᵢ 𝛼ᵢ = t.
  const int n = std::ssize(sets_);
  const int d = ambient_dimension();
  // The new variable is n * d, each row is a variable for a convex set
  auto x = prog->NewContinuousVariables(n, d, "x");
  auto alpha = prog->NewContinuousVariables(n, "alpha");
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<solvers::Binding<solvers::Constraint>> new_bindings;
  // Constraint I: -x + ∑ᵢ xᵢ == 0
  Eigen::VectorXd a(n + 1);
  a.head(n) = Eigen::VectorXd::Ones(n);
  a(n) = -1;
  for (int i = 0; i < d; ++i) {
    Eigen::Ref<const solvers::VectorXDecisionVariable> vars_i =
        vars.segment(i, 1);
    solvers::VariableRefList vars_list{x.col(i), vars_i};
    new_bindings.push_back(prog->AddLinearEqualityConstraint(a, 0, vars_list));
  }
  // Constraint II: ∑ᵢ αᵢ = t
  solvers::VectorXDecisionVariable alpha_t_vec(n + 1);
  alpha_t_vec.head(n) = alpha;
  alpha_t_vec(n) = t;
  new_bindings.push_back(prog->AddLinearEqualityConstraint(a, 0, alpha_t_vec));
  // t and alpha should be positive
  new_bindings.push_back(prog->AddBoundingBoxConstraint(0, inf, alpha_t_vec));
  // finally add the constraints for each convex set
  for (int i = 0; i < n; ++i) {
    auto cons = sets_[i]->AddPointInNonnegativeScalingConstraints(
        prog, x.row(i), alpha(i));
    new_bindings.insert(new_bindings.end(), cons.begin(), cons.end());
  }
  return new_bindings;
}

std::vector<solvers::Binding<solvers::Constraint>>
ConvexHull::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const Eigen::MatrixXd>& A_x,
    const Eigen::Ref<const Eigen::VectorXd>& b_x,
    const Eigen::Ref<const Eigen::VectorXd>& c, double d,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const {
  // Add the constraint that A_x * x + b_x = ∑ᵢ sᵢ, sᵢ ∈ 𝛼ᵢSᵢ, 𝛼ᵢ ≥ 0,
  // ∑ᵢ 𝛼ᵢ = c't + d.
  const int dim = ambient_dimension();
  const int n = std::ssize(sets_);
  // The new variable is n * d, each row is a variable for a convex set
  auto s = prog->NewContinuousVariables(n, dim, "s");
  auto alpha = prog->NewContinuousVariables(n, "alpha");
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<solvers::Binding<solvers::Constraint>> new_bindings;
  // constraint: A_x * x - ∑ᵢ sᵢ == -b_x
  DRAKE_DEMAND(A_x.rows() == dim);
  for (int i = 0; i < dim; ++i) {
    Eigen::Ref<const solvers::VectorXDecisionVariable> s_i = s.col(i);
    // A_x.row(i) * x - (1, ..., 1) sᵢ == -b_x[i]
    solvers::VectorXDecisionVariable x_s_i(dim + n);
    x_s_i.head(dim) = x;
    x_s_i.tail(n) = s_i;
    Eigen::VectorXd a_sum(A_x.cols() + n);
    a_sum.head(A_x.cols()) = A_x.row(i);
    a_sum.tail(n) = -Eigen::VectorXd::Ones(n);
    new_bindings.push_back(
        prog->AddLinearEqualityConstraint(a_sum, -b_x[i], x_s_i));
  }
  // constraint: ∑ᵢ αᵢ = c't + d
  const int p = c.size();
  Eigen::VectorXd a_con(n + p);
  a_con.head(n) = Eigen::VectorXd::Ones(n);
  a_con.tail(p) = -c;
  solvers::VectorXDecisionVariable alpha_t_vec(n + p);
  alpha_t_vec.head(n) = alpha;
  alpha_t_vec.tail(p) = t;
  new_bindings.push_back(
      prog->AddLinearEqualityConstraint(a_con, d, alpha_t_vec));
  // c't + d and alpha should be positive
  new_bindings.push_back(prog->AddBoundingBoxConstraint(0, inf, alpha));
  new_bindings.push_back(prog->AddLinearConstraint(c.transpose(), -d, inf, t));
  // finally add the constraints for each convex set
  for (int i = 0; i < n; ++i) {
    auto cons = sets_[i]->AddPointInNonnegativeScalingConstraints(
        prog, s.row(i), alpha(i));
    new_bindings.insert(new_bindings.end(), cons.begin(), cons.end());
  }
  return new_bindings;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
ConvexHull::DoToShapeWithPose() const {
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for ConvexHull.");
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
