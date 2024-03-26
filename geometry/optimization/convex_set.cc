#include "drake/geometry/optimization/convex_set.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/solution_result.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::LinearCost;
using solvers::MathematicalProgram;
using solvers::SolutionResult;
using solvers::VariableRefList;
using solvers::VectorXDecisionVariable;

namespace {

bool SolverReturnedWithoutError(
    const solvers::MathematicalProgramResult& result) {
  const SolutionResult status = result.get_solution_result();
  return status == SolutionResult::kSolutionFound ||
         status == SolutionResult::kDualInfeasible ||
         status == SolutionResult::kInfeasibleOrUnbounded;
}

}  // namespace

ConvexSet::ConvexSet(int ambient_dimension, bool has_exact_volume)
    : ambient_dimension_(ambient_dimension),
      has_exact_volume_(has_exact_volume) {
  DRAKE_THROW_UNLESS(ambient_dimension >= 0);
}

ConvexSet::~ConvexSet() = default;

bool ConvexSet::IntersectsWith(const ConvexSet& other) const {
  DRAKE_THROW_UNLESS(other.ambient_dimension() == this->ambient_dimension());
  if (ambient_dimension() == 0) {
    return !other.IsEmpty() && !this->IsEmpty();
  }
  solvers::MathematicalProgram prog{};
  const auto& x = prog.NewContinuousVariables(this->ambient_dimension(), "x");
  this->AddPointInSetConstraints(&prog, x);
  other.AddPointInSetConstraints(&prog, x);
  solvers::MathematicalProgramResult result = solvers::Solve(prog);
  return result.is_success();
}

bool ConvexSet::GenericDoIsBounded() const {
  // The empty set is bounded. We check it first, to ensure that the program
  // is feasible, so SolutionResult::kInfeasibleOrUnbounded or
  // SolutionResult::kDualInfeasible indicates unbounded, as solvers may not
  // always explicitly return that the primal is unbounded.
  if (IsEmpty()) {
    return true;
  }
  // Let a variable x be contained in the convex set. Iteratively try to
  // minimize or maximize x[i] for each dimension i. If any solves are
  // unbounded, the set is not bounded.
  MathematicalProgram prog;
  VectorXDecisionVariable x =
      prog.NewContinuousVariables(ambient_dimension(), "x");
  AddPointInSetConstraints(&prog, x);
  Binding<LinearCost> objective =
      prog.AddLinearCost(VectorXd::Zero(ambient_dimension()), x);

  VectorXd objective_vector(ambient_dimension());
  for (int i = 0; i < ambient_dimension(); ++i) {
    objective_vector.setZero();
    objective_vector[i] = 1;
    objective.evaluator()->UpdateCoefficients(objective_vector);
    const auto result = solvers::Solve(prog);
    if (result.get_solution_result() == solvers::SolutionResult::kUnbounded ||
        result.get_solution_result() ==
            solvers::SolutionResult::kInfeasibleOrUnbounded ||
        result.get_solution_result() ==
            solvers::SolutionResult::kDualInfeasible) {
      return false;
    }

    objective_vector[i] = -1;
    objective.evaluator()->UpdateCoefficients(objective_vector);
    const auto result2 = solvers::Solve(prog);
    if (result2.get_solution_result() == solvers::SolutionResult::kUnbounded ||
        result2.get_solution_result() ==
            solvers::SolutionResult::kInfeasibleOrUnbounded ||
        result2.get_solution_result() ==
            solvers::SolutionResult::kDualInfeasible) {
      return false;
    }
  }
  return true;
}

std::optional<std::pair<std::vector<double>, Eigen::MatrixXd>>
ConvexSet::Projection(const Eigen::Ref<const Eigen::MatrixXd>& points) const {
  DRAKE_THROW_UNLESS(points.rows() == ambient_dimension());
  if (ambient_dimension() == 0) {
    if (this->IsEmpty()) {
      return std::nullopt;
    }
    std::vector<double> distances(points.cols(), 0.0);
    return std::make_pair(distances, points);
  }
  std::vector<double> distances(points.cols(), 0.0);
  Eigen::MatrixXd projected_points(points.rows(), points.cols());
  std::vector<Eigen::VectorXd> unprojected_points_vect;
  std::vector<int> unprojected_inds;
  unprojected_points_vect.reserve(points.cols());

  const auto [shortcut_distances, shortcut_projected_points] =
      ProjectionShortcut(points);
  for (int i = 0; i < points.cols(); ++i) {
    if (shortcut_distances[i].has_value()) {
      distances[i] = shortcut_distances[i].value();
      projected_points.col(i) = shortcut_projected_points.col(i);
    } else {
      unprojected_inds.emplace_back(i);
      unprojected_points_vect.push_back(points.col(i));
    }
  }
  Eigen::MatrixXd unprojected_points(projected_points.rows(),
                                     ssize(unprojected_inds));
  for (int i = 0; i < ssize(unprojected_points_vect); ++i) {
    unprojected_points.col(i) = unprojected_points_vect[i];
  }
  const auto maybe_project = GenericDoProjection(unprojected_points);
  if (!maybe_project.has_value()) {
    return std::nullopt;
  }
  const auto [remaining_distances, remaining_projected_points] =
      maybe_project.value();
  for (int i = 0; i < ssize(unprojected_inds); ++i) {
    distances[unprojected_inds[i]] = remaining_distances[i];
    projected_points.col(unprojected_inds[i]) =
        remaining_projected_points.col(i);
  }
  return std::make_pair(distances, projected_points);
}

std::optional<std::pair<std::vector<double>, Eigen::MatrixXd>>
ConvexSet::GenericDoProjection(
    const Eigen::Ref<const Eigen::MatrixXd>& points) const {
  MathematicalProgram prog;
  MatrixX<symbolic::Variable> projected_points_vars(points.rows(),
                                                    points.cols());
  std::vector<solvers::Binding<solvers::Cost>> distances_bindings;
  for (int i = 0; i < points.cols(); ++i) {
    projected_points_vars.col(i) =
        prog.NewContinuousVariables(ambient_dimension(), fmt::format("x{}", i));
    AddPointInSetConstraints(&prog, projected_points_vars.col(i));
    distances_bindings.emplace_back(prog.AddQuadraticErrorCost(
        Eigen::MatrixXd::Identity(ambient_dimension(), ambient_dimension()),
        points.col(i), projected_points_vars.col(i)));
  }
  const auto result = solvers::Solve(prog);
  if (!result.is_success()) {
    return std::nullopt;
  }
  const Eigen::MatrixXd projected_points =
      result.GetSolution(projected_points_vars);
  std::vector<double> distances(points.cols(), 0.0);
  for (int i = 0; i < points.cols(); ++i) {
    const double binding_cost = result.EvalBinding(distances_bindings[i])[0];
    // The distance is lower bounded by 0, but numerical sensitivity may place
    // us slightly negative.
    distances[i] = sqrt(std::max(0.0, binding_cost));
  }
  return std::make_pair(distances, projected_points);
}

std::pair<std::vector<std::optional<double>>, Eigen::MatrixXd>
ConvexSet::ProjectionShortcut(
    const Eigen::Ref<const Eigen::MatrixXd>& points) const {
  // If we have a fast point in set shortcut, use it first.
  const double kTol =
      1e-12;  // This is below the tolerance of most convex solvers.
  std::vector<std::optional<double>> distances(points.cols(), std::nullopt);
  Eigen::MatrixXd projected_points(points.rows(), points.cols());
  std::vector<Eigen::VectorXd> unprojected_points_vect;
  std::vector<int> unprojected_inds;
  unprojected_points_vect.reserve(points.cols());
  for (int i = 0; i < points.cols(); ++i) {
    const auto point_in_set_shortcut =
        DoPointInSetShortcut(points.col(i), kTol);
    if (point_in_set_shortcut.has_value() && point_in_set_shortcut.value()) {
      distances[i] = 0;
      projected_points.col(i) = points.col(i);
    } else {
      unprojected_inds.push_back(i);
      unprojected_points_vect.push_back(points.col(i));
    }
  }

  Eigen::MatrixXd unprojected_points(projected_points.rows(),
                                     ssize(unprojected_inds));
  for (int i = 0; i < ssize(unprojected_points_vect); ++i) {
    unprojected_points.col(i) = unprojected_points_vect[i];
  }
  const auto [distances_shortcut, projected_points_shortcut] =
      DoProjectionShortcut(unprojected_points);
  for (int i = 0; i < ssize(unprojected_inds); ++i) {
    distances[unprojected_inds[i]] = distances_shortcut[i];
    projected_points.col(unprojected_inds[i]) =
        projected_points_shortcut.col(i);
  }
  return {distances, projected_points};
}

bool ConvexSet::DoIsEmpty() const {
  if (ambient_dimension() == 0) {
    return false;
    // Zero-dimensional sets are considered to be nonempty by default. Sets
    // which can be zero-dimensional and empty must handle this behavior in
    // their derived implementation of DoIsEmpty. Note that the check here is
    // required, to ensure AddPointInSetConstraints is not called for a zero
    // dimensional set -- this would throw an error.
  }
  solvers::MathematicalProgram prog;
  auto point = prog.NewContinuousVariables(ambient_dimension());
  AddPointInSetConstraints(&prog, point);
  auto result = solvers::Solve(prog);
  auto status = result.get_solution_result();
  switch (status) {
    case solvers::SolutionResult::kSolutionFound:
      return false;
    case solvers::SolutionResult::kInfeasibleConstraints:
    case solvers::SolutionResult::kInfeasibleOrUnbounded:
      return true;
    default:
      throw std::runtime_error(
          fmt::format("ConvexSet::IsEmpty() has solution result {}. "
                      "We are uncertain if the set if empty or not.",
                      status));
      return true;
  }
}

std::optional<Eigen::VectorXd> ConvexSet::DoMaybeGetPoint() const {
  return std::nullopt;
}

std::optional<Eigen::VectorXd> ConvexSet::DoMaybeGetFeasiblePoint() const {
  DRAKE_DEMAND(ambient_dimension() > 0);
  solvers::MathematicalProgram prog;
  auto point = prog.NewContinuousVariables(ambient_dimension());
  AddPointInSetConstraints(&prog, point);
  auto result = solvers::Solve(prog);
  auto status = result.get_solution_result();
  if (status == solvers::SolutionResult::kSolutionFound) {
    return result.GetSolution(point);
  } else {
    return std::nullopt;
  }
}

bool ConvexSet::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                             double tol) const {
  const auto shorcut_result = DoPointInSetShortcut(x, tol);
  if (shorcut_result.has_value()) {
    return shorcut_result.value();
  }
  return GenericDoPointInSet(x, tol);
}

bool ConvexSet::GenericDoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                                    double tol) const {
  MathematicalProgram prog;
  VectorXDecisionVariable point =
      prog.NewContinuousVariables(ambient_dimension(), "x");
  AddPointInSetConstraints(&prog, point);
  prog.AddLinearEqualityConstraint(x == point);
  const auto result = solvers::Solve(prog);
  DRAKE_THROW_UNLESS(SolverReturnedWithoutError(result));
  if (result.is_success()) {
    const VectorXd x_sol = result.GetSolution(point);
    return is_approx_equal_abstol(x, x_sol, tol);
  }
  return false;
}

std::pair<VectorX<symbolic::Variable>,
          std::vector<solvers::Binding<solvers::Constraint>>>
ConvexSet::AddPointInSetConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const {
  DRAKE_THROW_UNLESS(vars.size() == ambient_dimension());
  DRAKE_THROW_UNLESS(ambient_dimension() > 0);
  return DoAddPointInSetConstraints(prog, vars);
}

std::vector<solvers::Binding<solvers::Constraint>>
ConvexSet::AddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
  DRAKE_THROW_UNLESS(ambient_dimension() > 0);
  DRAKE_THROW_UNLESS(x.size() == ambient_dimension());
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
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const Eigen::VectorXd>& c, double d,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const {
  DRAKE_THROW_UNLESS(ambient_dimension() > 0);
  DRAKE_THROW_UNLESS(A.rows() == ambient_dimension());
  DRAKE_THROW_UNLESS(A.rows() == b.rows());
  DRAKE_THROW_UNLESS(A.cols() == x.size());
  DRAKE_THROW_UNLESS(c.rows() == t.size());
  std::vector<solvers::Binding<solvers::Constraint>> constraints =
      DoAddPointInNonnegativeScalingConstraints(prog, A, b, c, d, x, t);
  constraints.emplace_back(prog->AddLinearConstraint(
      c.transpose(), -d, std::numeric_limits<double>::infinity(), t));
  return constraints;
}

std::optional<symbolic::Variable>
ConvexSet::HandleZeroAmbientDimensionConstraints(
    solvers::MathematicalProgram* prog, const ConvexSet& set,
    std::vector<solvers::Binding<solvers::Constraint>>* constraints) const {
  if (set.IsEmpty()) {
    drake::log()->warn(
        "A constituent set is empty, making the MathematicalProgram trivially"
        " infeasible.");
    solvers::VectorXDecisionVariable new_vars = prog->NewContinuousVariables(1);
    constraints->push_back(prog->AddBoundingBoxConstraint(1, -1, new_vars[0]));
    return new_vars[0];
  }
  return std::nullopt;
}

double ConvexSet::CalcVolume() const {
  if (!has_exact_volume()) {
    throw std::runtime_error(
        fmt::format("The class {} reports that it cannot report an exact "
                    "volume. Use CalcVolumeViaSampling() instead.",
                    NiceTypeName::Get(*this)));
  }
  if (ambient_dimension() == 0) {
    throw std::runtime_error(
        fmt::format("The instance defined from {} is a zero-dimensional set. "
                    "The volume is not well defined.",
                    NiceTypeName::Get(*this)));
  }
  return DoCalcVolume();
}

SampledVolume ConvexSet::CalcVolumeViaSampling(
    RandomGenerator* generator, const double desired_rel_accuracy,
    const int max_num_samples) const {
  if (ambient_dimension() == 0) {
    throw std::runtime_error(
        fmt::format("Attempting to calculate the volume of a zero-dimensional "
                    "set {}. This is not well-defined.",
                    NiceTypeName::Get(*this)));
  }
  if (!IsBounded()) {
    // return infinity, nan, 0 samples.
    return {.volume = std::numeric_limits<double>::infinity(),
            .rel_accuracy = std::numeric_limits<double>::quiet_NaN(),
            .num_samples = 0};
  }
  DRAKE_THROW_UNLESS(desired_rel_accuracy <= 1.0);
  DRAKE_THROW_UNLESS(desired_rel_accuracy >= 0);
  DRAKE_THROW_UNLESS(max_num_samples > 0);
  const auto aabb_opt = Hyperrectangle::MaybeCalcAxisAlignedBoundingBox(*this);
  // if aabb_opt is nullopt and the set is not infinity, then we have
  // a problem with the solver.
  DRAKE_DEMAND(aabb_opt.has_value());
  const Hyperrectangle& aabb = aabb_opt.value();
  int num_samples = 0;
  int num_hits = 0;
  double relative_accuracy_ub_squared = 1.0;
  const double desired_rel_accuracy_squared = std::pow(desired_rel_accuracy, 2);
  while (relative_accuracy_ub_squared > desired_rel_accuracy_squared &&
         num_samples < max_num_samples) {
    auto point = aabb.UniformSample(generator);
    ++num_samples;
    if (this->PointInSet(point)) {
      ++num_hits;
    }
    // p is the probability of hitting the set = num_hits/num_samples.
    if (num_hits > 0) {
      // Let p be the real probability of hitting the set. We are estimating p.
      // The standard deviation of the MonteCarlo estimate is sigma/sqrt(n),
      // where sigma is the standard deviation of the Bernoulli distribution.
      // The standard deviation of the Bernoulli distribution is sqrt((1-p)*p),
      // where p is the probability of success.  Therefore, the standard
      // deviation of the estimate is sqrt((1-p)*p/n). We don't know p, but the
      // max value of (1-p)*p is 1/4, which occurs at p = 1/2.
      // https://people.math.umass.edu/~lr7q/ps_files/teaching/math456/Chapter6.pdf
      // The standard deviation divided by the mean, so the error is
      // upper-bounded by sqrt(1/(4*n)). Therefore, the
      // relative_accuracy_squared < 1/(4*n*p) or simply 1/ (4 * num_hits).
      relative_accuracy_ub_squared = static_cast<double>(1) / (4 * num_hits);
    }
  }
  if (relative_accuracy_ub_squared > desired_rel_accuracy_squared) {
    drake::log()->warn(
        "Volume calculation did not converge to desired relative accuracy {}."
        "The tightest upper bound on relative accuracy achieved: {}",
        desired_rel_accuracy, std::sqrt(relative_accuracy_ub_squared));
  }
  const auto estimated_volume = aabb.CalcVolume() *
                                static_cast<double>(num_hits) /
                                static_cast<double>(num_samples);
  const auto relative_accuracy_ub = std::sqrt(relative_accuracy_ub_squared);
  return {.volume = estimated_volume,
          .rel_accuracy = relative_accuracy_ub,
          .num_samples = num_samples};
}

double ConvexSet::DoCalcVolume() const {
  throw std::runtime_error(
      fmt::format("The class {} has a defect -- has_exact_volume() is "
                  "reporting true, but DoCalcVolume has not been implemented.",
                  NiceTypeName::Get(*this)));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
