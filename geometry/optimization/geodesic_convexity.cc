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

std::vector<std::tuple<int, int, Eigen::VectorXd>> CalcPairwiseIntersections(
    const ConvexSets& convex_sets_A, const ConvexSets& convex_sets_B,
    const std::vector<int>& continuous_revolute_joints) {
  DRAKE_THROW_UNLESS(convex_sets_A.size() > 0);
  DRAKE_THROW_UNLESS(convex_sets_B.size() > 0);
  const int dimension = convex_sets_A[0]->ambient_dimension();
  internal::ThrowsForInvalidContinuousJointsList(dimension,
                                                 continuous_revolute_joints);

  std::vector<std::tuple<int, int, Eigen::VectorXd>> edges;

  // Region bounding boxes along dimensions corresponding to continuous revolute
  // joints. If convex_sets_B == convex_sets_A, then we do not populate
  // region_minimum_and_maximum_values_B, and instead point to the corresponding
  // value in region_minimum_and_maximum_values_A.
  std::vector<std::vector<std::pair<double, double>>>
      region_minimum_and_maximum_values_A;
  std::vector<std::vector<std::pair<double, double>>>
      region_minimum_and_maximum_values_B;

  // Compute bounding boxes for convex_sets_A.
  for (int i = 0; i < ssize(convex_sets_A); ++i) {
    region_minimum_and_maximum_values_A.emplace_back(
        std::vector<std::pair<double, double>>());
    for (const int k : continuous_revolute_joints) {
      region_minimum_and_maximum_values_A.at(i).emplace_back(
          internal::GetMinimumAndMaximumValueAlongDimension(*convex_sets_A[i],
                                                            k));
    }
  }

  // Compute bounding boxes for convex_sets_B if distinct from convex_sets_A.
  bool convex_sets_A_and_B_are_identical = convex_sets_A == convex_sets_B;
  if (!convex_sets_A_and_B_are_identical) {
    for (int i = 0; i < ssize(convex_sets_B); ++i) {
      region_minimum_and_maximum_values_B.emplace_back(
          std::vector<std::pair<double, double>>());
      for (const int k : continuous_revolute_joints) {
        region_minimum_and_maximum_values_B.at(i).emplace_back(
            internal::GetMinimumAndMaximumValueAlongDimension(*convex_sets_B[i],
                                                              k));
      }
    }
  }

  VectorXd offset = Eigen::VectorXd::Zero(dimension);
  for (int i = 0; i < ssize(convex_sets_A); ++i) {
    for (int j = 0; j < ssize(convex_sets_B); ++j) {
      if (convex_sets_A_and_B_are_identical && j <= i) {
        // If we're computing intersections within convex_sets_A and j <= i,
        // then we've already checked if we need to add an edge when i and j
        // were flipped, and that set has already been added.
        continue;
      }
      const auto& bbox_A = region_minimum_and_maximum_values_A.at(i);
      // If convex_sets_A == convex_sets_B, then
      // region_minimum_and_maximum_values_B is empty, so we instead get the
      // bbox value from region_minimum_and_maximum_values_A.
      const auto& bbox_B = convex_sets_A_and_B_are_identical
                               ? region_minimum_and_maximum_values_A.at(j)
                               : region_minimum_and_maximum_values_B.at(j);

      offset.setZero();

      // First, we compute what the offset that should be applied to
      // convex_sets_A[i] to potentially make it overlap with convex_sets_B[j].
      for (int k = 0; k < ssize(continuous_revolute_joints); ++k) {
        const int joint_index = continuous_revolute_joints.at(k);
        if (bbox_A.at(k).first < bbox_B.at(k).first) {
          // In this case, the minimum value of convex_sets_A[i] along dimension
          // k is smaller than the minimum value of convex_sets_B[j] along
          // dimension k, so we must translate by a positive amount. By the
          // convexity radius property (which has already been checked), we know
          // that the width of each set is strictly less than π. So we need to
          // translate convex_sets_A[i] by some multiple of 2π such that the
          // difference between the maximum value in convex_sets_B[j] and the
          // minimum value in convex_sets_A[i] is less than 2π. This is computed
          // by taking that difference, dividing by 2π, and truncating.
          offset(joint_index) =
              2 * M_PI *
              std::floor((bbox_B.at(k).second - bbox_A.at(k).first) /
                         (2 * M_PI));
        } else {
          // In this case, the minimum value of convex_sets_B[j] along dimension
          // k is smaller than the minimum value of convex_sets_A[i] along
          // dimension k. We do the same thing as above, but flip the order of
          // the sets. As a result, we also flip the sign of the resulting
          // translation.
          offset(joint_index) =
              -2 * M_PI *
              std::floor((bbox_A.at(k).second - bbox_B.at(k).first) /
                         (2 * M_PI));
        }
      }

      // Now that we know the offset that is for each dimension, we actually
      // check if the sets intersect.
      MathematicalProgram prog;
      // TODO(cohnt) Once #20842 lands, reuse the Mathematical program
      // (modifying or deleting and re-adding constraints as needed).
      VectorXDecisionVariable x = prog.NewContinuousVariables(dimension);
      VectorXDecisionVariable y = prog.NewContinuousVariables(dimension);
      Eigen::MatrixXd Aeq(dimension, 2 * dimension);
      // Add x + offset == y by [-I, I][x; y] == [offset]
      Aeq.leftCols(dimension) =
          -Eigen::MatrixXd::Identity(dimension, dimension);
      Aeq.rightCols(dimension) =
          Eigen::MatrixXd::Identity(dimension, dimension);
      prog.AddLinearEqualityConstraint(Aeq, offset, {x, y});
      convex_sets_A.at(i)->AddPointInSetConstraints(&prog, x);
      convex_sets_B.at(j)->AddPointInSetConstraints(&prog, y);
      const auto result = Solve(prog);
      if (result.is_success()) {
        // Regions are overlapping, add edge (i, j). If we're adding edges
        // within convex_sets_A, also add edge (j, i), since edges are
        // considered bidirectional in that context.
        edges.emplace_back(i, j, offset);
        if (convex_sets_A_and_B_are_identical) {
          edges.emplace_back(j, i, -offset);
        }
      }
    }
  }

  return edges;
}

std::vector<std::tuple<int, int, Eigen::VectorXd>> CalcPairwiseIntersections(
    const ConvexSets& convex_sets,
    const std::vector<int>& continuous_revolute_joints) {
  return CalcPairwiseIntersections(convex_sets, convex_sets,
                                   continuous_revolute_joints);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
