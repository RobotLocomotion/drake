#include "drake/geometry/optimization/geodesic_convexity.h"

#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Intersection;
using drake::solvers::MathematicalProgram;
using drake::solvers::Solve;
using drake::solvers::VectorXDecisionVariable;
using Eigen::VectorXd;

std::pair<double, double> internal::GetMinimumAndMaximumValueAlongDimension(
    const ConvexSet& region, int dimension) {
  // We formulate the problem in the vectorized overload instead of here so that
  // we can reuse the same MathematicalProgram for multiple evaluations.
  return internal::GetMinimumAndMaximumValueAlongDimension(
      region, std::vector<int>{dimension})[0];
}

std::vector<std::pair<double, double>>
internal::GetMinimumAndMaximumValueAlongDimension(const ConvexSet& region,
                                                  std::vector<int> dimensions) {
  std::vector<std::pair<double, double>> bounds;
  MathematicalProgram prog;
  VectorXDecisionVariable y =
      prog.NewContinuousVariables(region.ambient_dimension());
  region.AddPointInSetConstraints(&prog, y);
  VectorXd objective_vector = VectorXd::Zero(region.ambient_dimension());
  auto objective = prog.AddLinearCost(objective_vector, y);
  for (const auto& dimension : dimensions) {
    DRAKE_THROW_UNLESS(dimension >= 0 &&
                       dimension < region.ambient_dimension());
    objective_vector.setZero();
    objective_vector[dimension] = 1;
    objective.evaluator()->UpdateCoefficients(objective_vector);

    auto result = Solve(prog);
    if (!result.is_success()) {
      throw std::runtime_error(
          fmt::format("Failed to compute the lower bound of a convex set along "
                      "dimension {}!",
                      dimension));
    }
    const double lower_bound = result.GetSolution(y)[dimension];

    objective_vector[dimension] = -1;
    objective.evaluator()->UpdateCoefficients(objective_vector);

    result = Solve(prog);
    if (!result.is_success()) {
      throw std::runtime_error(
          fmt::format("Failed to compute the upper bound of a convex set along "
                      "dimension {}!",
                      dimension));
    }
    const double upper_bound = result.GetSolution(y)[dimension];
    bounds.push_back({lower_bound, upper_bound});
  }

  return bounds;
}

VectorXd internal::ComputeOffsetContinuousRevoluteJoints(
    const int num_positions, const std::vector<int>& continuous_revolute_joints,
    const std::vector<std::pair<double, double>>& continuous_bbox_A,
    const std::vector<std::pair<double, double>>& continuous_bbox_B) {
  internal::ThrowsForInvalidContinuousJointsList(num_positions,
                                                 continuous_revolute_joints);
  DRAKE_THROW_UNLESS(continuous_bbox_A.size() == continuous_bbox_B.size());
  VectorXd offset = Eigen::VectorXd::Zero(num_positions);

  for (int i = 0; i < ssize(continuous_revolute_joints); ++i) {
    const int joint_index = continuous_revolute_joints.at(i);
    if (continuous_bbox_A.at(i).first < continuous_bbox_B.at(i).first) {
      // In this case, the minimum value of convex_sets_A[i] along dimension
      // joint_index is smaller than the minimum value of convex_sets_B[j] along
      // dimension joint_index, so we must translate by a positive amount. By
      // the convexity radius property (which has already been checked), we know
      // that the width of each set is strictly less than π. So we need to
      // translate convex_sets_A[i] by some multiple of 2π such that the
      // difference between the maximum value in convex_sets_B[j] and the
      // minimum value in convex_sets_A[i] is less than 2π. This is computed
      // by taking that difference, dividing by 2π, and truncating.
      offset(joint_index) = 2 * M_PI *
                            std::floor((continuous_bbox_B.at(i).second -
                                        continuous_bbox_A.at(i).first) /
                                       (2 * M_PI));
    } else {
      // In this case, the minimum value of convex_sets_B[j] along dimension
      // joint_index is smaller than the minimum value of convex_sets_A[i] along
      // dimension joint_index. We do the same thing as above, but flip the
      // order of the sets. As a result, we also flip the sign of the resulting
      // translation.
      offset(joint_index) = -2 * M_PI *
                            std::floor((continuous_bbox_A.at(i).second -
                                        continuous_bbox_B.at(i).first) /
                                       (2 * M_PI));
    }
  }
  return offset;
}

void internal::ThrowsForInvalidContinuousJointsList(
    int num_positions, const std::vector<int>& continuous_revolute_joints) {
  for (int i = 0; i < ssize(continuous_revolute_joints); ++i) {
    // Make sure the unbounded revolute joints point to valid indices.
    if (continuous_revolute_joints[i] < 0 ||
        continuous_revolute_joints[i] >= num_positions) {
      throw std::runtime_error(fmt::format(
          "Each joint index in continuous_revolute_joints must lie in the "
          "interval [0, {}). Joint index {} (located at {}) violates this.",
          num_positions, continuous_revolute_joints[i], i));
    }
  }
  std::unordered_set<int> comparison(continuous_revolute_joints.begin(),
                                     continuous_revolute_joints.end());
  if (comparison.size() != continuous_revolute_joints.size()) {
    throw std::runtime_error(fmt::format(
        "continuous_revolute_joints must not contain duplicate entries."));
  }
}

bool CheckIfSatisfiesConvexityRadius(
    const ConvexSet& convex_set,
    const std::vector<int>& continuous_revolute_joints) {
  for (const int& j : continuous_revolute_joints) {
    auto [min_value, max_value] =
        internal::GetMinimumAndMaximumValueAlongDimension(convex_set, j);
    if (max_value - min_value >= M_PI) {
      return false;
    }
  }
  return true;
}

ConvexSets PartitionConvexSet(
    const ConvexSet& convex_set,
    const std::vector<int>& continuous_revolute_joints, const double epsilon) {
  DRAKE_THROW_UNLESS(epsilon > 0);
  DRAKE_THROW_UNLESS(epsilon < M_PI);
  internal::ThrowsForInvalidContinuousJointsList(convex_set.ambient_dimension(),
                                                 continuous_revolute_joints);
  // Boundedness along dimensions corresponding to continuous revolute joints
  // will be asserted by the GetMinimumAndMaximumValueAlongDimension calls
  // below.

  ConvexSets sets = MakeConvexSets(convex_set);
  const double convexity_radius_step = M_PI - epsilon;
  const int dim = convex_set.ambient_dimension();

  std::vector<std::pair<double, double>> bbox(dim, std::make_pair(0, 0));

  // We only populate the entries corresponding to continuous revolute joints,
  // since the lower and upper limits of other joints aren't needed.
  for (const int& i : continuous_revolute_joints) {
    bbox[i] = internal::GetMinimumAndMaximumValueAlongDimension(convex_set, i);
  }
  // The overall structure is to partition the set along each dimension
  // corresponding to a continuous revolute joint. The partitioning is done by
  // constructing axis-aligned bounding boxes, and intersecting them with the
  // original set.
  for (const int& i : continuous_revolute_joints) {
    const double min_value = bbox[i].first;
    const double max_value = bbox[i].second;
    if (max_value - min_value >= M_PI) {
      // Since we operate along one dimension at a time, all of the elements of
      // sets will potentially violate the convexity radius, and thus must be
      // partitioned. Sets which end up being empty are dropped. Because we are
      // adding the new ConvexSets into sets, we track where the list of old
      // sets ended, to prevent the infinite loop, and for later deletion.
      const int j_max = sets.size();
      for (int j = 0; j < j_max; ++j) {
        for (double lower_bound = min_value; lower_bound < max_value;
             lower_bound += convexity_radius_step - epsilon) {
          Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, dim);
          Eigen::VectorXd b = Eigen::VectorXd::Zero(2);
          A(0, i) = 1;
          b[0] = lower_bound + convexity_radius_step;
          A(1, i) = -1;
          b[1] = -lower_bound;
          HPolyhedron chunk_bbox(A, b);
          Intersection candidate_set(*sets[j], chunk_bbox);
          if (!candidate_set.IsEmpty()) {
            sets.push_back(copyable_unique_ptr<ConvexSet>(candidate_set));
          }
        }
      }
      // The sets which we have now partitioned are still stored between index 0
      // and index j_max. We can now delete them, since we've replaced them with
      // smaller pieces.
      sets.erase(sets.begin(), sets.begin() + j_max);
    }
  }
  return sets;
}

ConvexSets PartitionConvexSet(
    const ConvexSets& convex_sets,
    const std::vector<int>& continuous_revolute_joints, const double epsilon) {
  DRAKE_THROW_UNLESS(convex_sets.size() > 0);
  DRAKE_THROW_UNLESS(convex_sets[0] != nullptr);
  internal::ThrowsForInvalidContinuousJointsList(
      convex_sets[0]->ambient_dimension(), continuous_revolute_joints);

  int ambient_dimension = convex_sets[0]->ambient_dimension();
  for (int i = 1; i < ssize(convex_sets); ++i) {
    DRAKE_THROW_UNLESS(convex_sets[i] != nullptr);
    DRAKE_THROW_UNLESS(convex_sets[i]->ambient_dimension() ==
                       ambient_dimension);
  }

  ConvexSets sets;
  for (int i = 0; i < ssize(convex_sets); ++i) {
    ConvexSets new_sets = PartitionConvexSet(
        *convex_sets[i], continuous_revolute_joints, epsilon);
    sets.insert(sets.end(), new_sets.begin(), new_sets.end());
  }
  return sets;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
