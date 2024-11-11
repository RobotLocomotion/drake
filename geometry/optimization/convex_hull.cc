#include "drake/geometry/optimization/convex_hull.h"

#include <limits>
#include <memory>

#include <drake/common/symbolic/expression/variables.h>

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

// Add the new variables to the existing_variables from the bindings.
void AddNewVariables(
    const std::vector<solvers::Binding<solvers::Constraint>>& bindings,
    symbolic::Variables* existing_variables) {
  for (const auto& binding : bindings) {
    const auto& vars = binding.variables();
    existing_variables->insert(symbolic::Variables(vars));
  }
}

// Given a vector of convex sets, return a vector of non-empty convex sets if
// remove_empty_sets is true, otherwise return the original vector.
ConvexSets MakeParticipartingSets(const ConvexSets& sets,
                                  const bool remove_empty_sets) {
  if (!remove_empty_sets) {
    return sets;
  }
  ConvexSets non_empty_sets;
  for (const auto& set : sets) {
    if (!set->IsEmpty()) {
      non_empty_sets.push_back(set);
    }
  }
  return non_empty_sets;
}
}  // namespace

ConvexHull::ConvexHull(const ConvexSets& sets, const bool remove_empty_sets)
    : ConvexSet(GetAmbientDimension(sets), false),
      sets_(sets),
      participating_sets_{MakeParticipartingSets(sets_, remove_empty_sets)},
      empty_sets_removed_(remove_empty_sets) {}

ConvexHull::~ConvexHull() = default;

const ConvexSet& ConvexHull::element(int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < std::ssize(sets_));
  return *sets_[index];
}

std::unique_ptr<ConvexSet> ConvexHull::DoClone() const {
  return std::make_unique<ConvexHull>(*this);
}

std::optional<bool> ConvexHull::DoIsBoundedShortcutParallel(
    Parallelism parallelism) const {
  for (const auto& s : sets_) {
    if (!s->IsBounded(parallelism)) {
      return false;
    }
  }
  return true;
}

bool ConvexHull::DoIsEmpty() const {
  if (empty_sets_removed_) {
    return participating_sets_.empty();
  }
  // If empty_sets_removed_ is false, then we reconstruct the
  // participating_sets_ and check if it is empty.
  return ConvexHull(sets_, true).IsEmpty();
}

std::optional<VectorXd> ConvexHull::DoMaybeGetPoint() const {
  return std::nullopt;
}

bool ConvexHull::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                              double tol) const {
  // Check the feasibility of |x - ∑ᵢ xᵢ| <= tol * 1, xᵢ ∈ 𝛼ᵢXᵢ, 𝛼ᵢ ≥ 0, ∑ᵢ 𝛼ᵢ =
  // 1, where 1 is a vector of ones and |.| is interpreted element-wise.
  MathematicalProgram prog;
  const int n = std::ssize(participating_sets_);
  const int d = ambient_dimension();
  // x_sets is d * n, each column is a variable for a convex set
  auto x_sets = prog.NewContinuousVariables(d, n, "x_sets");
  auto alpha = prog.NewContinuousVariables(n, "alpha");
  // constraint: x - tol * 1  ≤ ∑ᵢ xᵢ ≤ x + tol * 1
  Eigen::VectorXd ones = Eigen::VectorXd::Ones(n);
  for (int i = 0; i < d; ++i) {
    prog.AddLinearConstraint(Eigen::VectorXd::Ones(n), x(i) - tol, x(i) + tol,
                             x_sets.row(i));
  }
  // constraint: ∑ᵢ αᵢ = 1
  prog.AddLinearEqualityConstraint(Eigen::MatrixXd::Ones(1, n),
                                   VectorXd::Ones(1), alpha);
  // constraint: 1 ≥ αᵢ ≥ 0
  prog.AddBoundingBoxConstraint(0, 1, alpha);
  // Add the constraints for each convex set.
  for (int i = 0; i < n; ++i) {
    participating_sets_[i]->AddPointInNonnegativeScalingConstraints(
        &prog, x_sets.col(i), alpha(i));
  }
  // Check feasibility.
  const auto result = solvers::Solve(prog);
  return result.is_success();
}

std::pair<VectorX<symbolic::Variable>,
          std::vector<solvers::Binding<solvers::Constraint>>>
ConvexHull::DoAddPointInSetConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const {
  // Add the constraint that vars = ∑ᵢ xᵢ, xᵢ ∈ 𝛼ᵢXᵢ, 𝛼ᵢ ≥ 0, ∑ᵢ 𝛼ᵢ = 1.
  const int n = std::ssize(participating_sets_);
  const int d = ambient_dimension();
  // The variable is d * n, each column is a variable for a convex set
  auto x_sets = prog->NewContinuousVariables(d, n, "x_sets");
  auto alpha = prog->NewContinuousVariables(n, "alpha");
  std::vector<solvers::Binding<solvers::Constraint>> new_bindings;
  new_bindings.push_back(prog->AddBoundingBoxConstraint(0, 1, alpha));
  // constraint: vars - ∑ᵢ xᵢ == 0 -> (1 ... 1 -1)(x_sets[i, :], vars) = 0
  Eigen::VectorXd a(n + 1);
  a.head(n) = Eigen::VectorXd::Ones(n);
  a(n) = -1;
  for (int i = 0; i < d; ++i) {
    Eigen::Ref<const solvers::VectorXDecisionVariable> vars_i =
        vars.segment(i, 1);
    solvers::VectorXDecisionVariable x_i_vars(n + 1);
    x_i_vars.head(n) = x_sets.row(i);
    x_i_vars(n) = vars(i);
    new_bindings.push_back(prog->AddLinearEqualityConstraint(a, 0, x_i_vars));
  }
  // constraint: ∑ᵢ αᵢ = 1
  new_bindings.push_back(prog->AddLinearEqualityConstraint(
      Eigen::MatrixXd::Ones(1, n), VectorXd::Ones(1), alpha));
  auto new_vars = drake::symbolic::Variables();
  // add the constraints for each convex set
  for (int i = 0; i < n; ++i) {
    auto cons = participating_sets_[i]->AddPointInNonnegativeScalingConstraints(
        prog, x_sets.col(i), alpha(i));
    new_bindings.insert(new_bindings.end(), cons.begin(), cons.end());
    AddNewVariables(cons, &new_vars);
  }
  // Convert to std::vector<symbolic::Variable> because sets do not have
  // contiguous memory.
  std::vector<symbolic::Variable> new_vars_vec(new_vars.size());
  std::move(new_vars.begin(), new_vars.end(), new_vars_vec.begin());
  return std::make_pair(Eigen::Map<VectorX<symbolic::Variable>>(
                            new_vars_vec.data(), new_vars_vec.size()),
                        std::move(new_bindings));
}

std::vector<solvers::Binding<solvers::Constraint>>
ConvexHull::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
  // Add the constraint that t >= 0, x = ∑ᵢ xᵢ, xᵢ ∈ 𝛼ᵢXᵢ, 𝛼ᵢ ≥ 0, ∑ᵢ 𝛼ᵢ = t.
  const int n = std::ssize(participating_sets_);
  const int d = ambient_dimension();
  // The new variable is d * n, each row is a variable for a convex set
  auto x_sets = prog->NewContinuousVariables(d, n, "x_sets");
  auto alpha = prog->NewContinuousVariables(n, "alpha");
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<solvers::Binding<solvers::Constraint>> new_bindings;
  // Constraint I: -x + ∑ᵢ xᵢ == 0
  Eigen::VectorXd a(n + 1);
  a.head(n) = Eigen::VectorXd::Ones(n);
  a(n) = -1;
  for (int i = 0; i < d; ++i) {
    solvers::VectorXDecisionVariable x_sets_i_x(n + 1);
    x_sets_i_x.head(n) = x_sets.row(i);
    x_sets_i_x(n) = x(i);
    new_bindings.push_back(prog->AddLinearEqualityConstraint(a, 0, x_sets_i_x));
  }
  // Constraint II: ∑ᵢ αᵢ = t
  solvers::VectorXDecisionVariable alpha_t_vec(n + 1);
  alpha_t_vec.head(n) = alpha;
  alpha_t_vec(n) = t;
  new_bindings.push_back(prog->AddLinearEqualityConstraint(a, 0, alpha_t_vec));
  // alpha should be positive.
  new_bindings.push_back(prog->AddBoundingBoxConstraint(0, inf, alpha));
  // Finally add the constraints for each convex set.
  for (int i = 0; i < n; ++i) {
    auto cons = participating_sets_[i]->AddPointInNonnegativeScalingConstraints(
        prog, x_sets.col(i), alpha(i));
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
  // Add the constraint that A_x * x + b_x = ∑ᵢ xᵢ, xᵢ ∈ 𝛼ᵢXᵢ, 𝛼ᵢ ≥ 0,
  // ∑ᵢ 𝛼ᵢ = c't + d.
  const int dim = ambient_dimension();
  const int n = std::ssize(participating_sets_);
  // The new variable is dim * n, each column belongs to a convex set.
  auto x_sets = prog->NewContinuousVariables(dim, n, "x_sets");
  auto alpha = prog->NewContinuousVariables(n, "alpha");
  std::vector<solvers::Binding<solvers::Constraint>> new_bindings;
  // constraint: A_x * x - ∑ᵢ sᵢ == -b_x
  for (int i = 0; i < dim; ++i) {
    // A_x.row(i) * x - (1, ..., 1) xᵢ == -b_x[i]
    solvers::VectorXDecisionVariable x_sets_i_(dim + n);
    x_sets_i_.head(dim) = x;
    x_sets_i_.tail(n) = x_sets.row(i);
    Eigen::VectorXd a_sum(A_x.cols() + n);
    a_sum.head(A_x.cols()) = A_x.row(i);
    a_sum.tail(n) = -Eigen::VectorXd::Ones(n);
    new_bindings.push_back(
        prog->AddLinearEqualityConstraint(a_sum, -b_x[i], x_sets_i_));
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
  // Add the inclusion constraints for each convex set.
  for (int i = 0; i < n; ++i) {
    auto cons = participating_sets_[i]->AddPointInNonnegativeScalingConstraints(
        prog, x_sets.col(i), alpha(i));
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
