#include "drake/geometry/optimization/geodesic_convexity.h"

#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/intersection.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Hyperrectangle;
using drake::geometry::optimization::Intersection;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
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

// TODO(cohnt): Find a way to reduce code reuse, since this function is
// extremely similar to the Hyperrectangle constructor.
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

  // This method is sometimes called in parallel, so we assert thread-safety.
  DRAKE_ASSERT(prog.IsThreadSafe());

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
  std::vector<std::pair<double, double>> bbox =
      internal::GetMinimumAndMaximumValueAlongDimension(
          convex_set, continuous_revolute_joints);
  for (const auto& [min_value, max_value] : bbox) {
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
  std::vector<std::pair<double, double>> bbox_values =
      internal::GetMinimumAndMaximumValueAlongDimension(
          convex_set, continuous_revolute_joints);
  for (int i = 0; i < ssize(bbox_values); ++i) {
    bbox[continuous_revolute_joints[i]] = bbox_values[i];
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

namespace {

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;

/* Check if two hyperrectangles intersect, when a given offset is applied to
the first one. */
bool HyperrectangleOffsetIntersection(const Hyperrectangle& h1,
                                      const Hyperrectangle& h2,
                                      const VectorXd& offset) {
  Hyperrectangle h1_offset(h1.lb() + offset, h1.ub() + offset);
  return h1_offset.MaybeGetIntersection(h2).has_value();
}

/* This function takes in a list of convex sets, and also a list of dimensions,
for which we want to know the upper and lower bounds of each set. It returns a
list of Hyperrectangle objects corresponding to the convex sets, for which the
upper and lower limit on each dimension corresponds to the highest and lowest
value attained by the corresponding convex set along that dimension.

Note that the entries of dimensions are expected to be unique and in increasing
order, but we do not check for this. */
std::vector<Hyperrectangle> BoundingBoxesForListOfSetsSomeDimensions(
    const ConvexSets& sets, const std::vector<int>& dimensions,
    Parallelism parallelism) {
  DRAKE_DEMAND(sets.size() > 0);
  for (const copyable_unique_ptr<ConvexSet>& set : sets) {
    DRAKE_THROW_UNLESS(set != nullptr);
  }
  int ambient_dimension = sets[0]->ambient_dimension();
  for (int i = 1; i < ssize(sets); ++i) {
    DRAKE_DEMAND(sets[i]->ambient_dimension() == ambient_dimension);
  }

  std::vector<Hyperrectangle> bboxes(sets.size());

  auto solve_ith = [&](const int thread_num, const int64_t i) {
    unused(thread_num);
    // Compute minimum and maximum value along the specified dimensions Other
    // dimensions will be left with lower == upper == 0.
    VectorXd lb = VectorXd::Zero(sets[0]->ambient_dimension());
    VectorXd ub = VectorXd::Zero(sets[0]->ambient_dimension());
    std::vector<std::pair<double, double>> bbox_values =
        internal::GetMinimumAndMaximumValueAlongDimension(*sets[i], dimensions);
    for (int j = 0; j < ssize(bbox_values); ++j) {
      lb[dimensions[j]] = bbox_values[j].first;
      ub[dimensions[j]] = bbox_values[j].second;
    }
    bboxes[i] = Hyperrectangle(lb, ub);
  };

  StaticParallelForIndexLoop(DegreeOfParallelism(parallelism.num_threads()), 0,
                             ssize(sets), solve_ith,
                             ParallelForBackend::BEST_AVAILABLE);

  return bboxes;
}

}  // namespace

std::pair<std::vector<std::pair<int, int>>, std::vector<Eigen::VectorXd>>
ComputePairwiseIntersections(const ConvexSets& convex_sets_A,
                             const ConvexSets& convex_sets_B,
                             const std::vector<int>& continuous_revolute_joints,
                             bool preprocess_bbox, Parallelism parallelism) {
  DRAKE_THROW_UNLESS(convex_sets_A.size() > 0);
  DRAKE_THROW_UNLESS(convex_sets_B.size() > 0);
  for (const copyable_unique_ptr<ConvexSet>& set : convex_sets_A) {
    DRAKE_THROW_UNLESS(set != nullptr);
  }
  for (const copyable_unique_ptr<ConvexSet>& set : convex_sets_B) {
    DRAKE_THROW_UNLESS(set != nullptr);
  }
  const int dimension = convex_sets_A[0]->ambient_dimension();
  internal::ThrowsForInvalidContinuousJointsList(dimension,
                                                 continuous_revolute_joints);

  // Bounding boxes along all dimensions (including joints which are not
  // continuous revolute). If preprocess_bbox is true, all lower and upper
  // limits will be computed and stored. Otherwise, only the entries
  // corresponding to continuous revolute joints will be populated, and all
  // lower and upper bounds will be set to zero. If
  // convex_sets_B == convex_sets_A, then we do not populate bboxes_B, and
  // instead point to the corresponding element in bboxes_A.
  std::vector<Hyperrectangle> bboxes_A, bboxes_B;

  std::vector<int> all_joints(dimension);
  std::iota(all_joints.begin(), all_joints.end(), 0);

  // Compute bounding boxes for convex_sets_A.
  if (preprocess_bbox) {
    bboxes_A = BoundingBoxesForListOfSetsSomeDimensions(
        convex_sets_A, all_joints, parallelism);
  } else {
    bboxes_A = BoundingBoxesForListOfSetsSomeDimensions(
        convex_sets_A, continuous_revolute_joints, parallelism);
  }

  // Compute bounding boxes for convex_sets_B if distinct from convex_sets_A.
  bool convex_sets_A_and_B_are_identical = convex_sets_A == convex_sets_B;
  if (!convex_sets_A_and_B_are_identical) {
    if (preprocess_bbox) {
      bboxes_B = BoundingBoxesForListOfSetsSomeDimensions(
          convex_sets_B, all_joints, parallelism);
    } else {
      bboxes_B = BoundingBoxesForListOfSetsSomeDimensions(
          convex_sets_B, continuous_revolute_joints, parallelism);
    }
  }

  return ComputePairwiseIntersections(
      convex_sets_A, convex_sets_B, continuous_revolute_joints, bboxes_A,
      convex_sets_A_and_B_are_identical ? bboxes_A : bboxes_B, parallelism);
}

std::pair<std::vector<std::pair<int, int>>, std::vector<Eigen::VectorXd>>
ComputePairwiseIntersections(const ConvexSets& convex_sets_A,
                             const ConvexSets& convex_sets_B,
                             const std::vector<int>& continuous_revolute_joints,
                             const std::vector<Hyperrectangle>& bboxes_A,
                             const std::vector<Hyperrectangle>& bboxes_B,
                             Parallelism parallelism) {
  DRAKE_THROW_UNLESS(convex_sets_A.size() > 0);
  DRAKE_THROW_UNLESS(convex_sets_B.size() > 0);
  DRAKE_THROW_UNLESS(convex_sets_A.size() == bboxes_A.size());
  DRAKE_THROW_UNLESS(convex_sets_B.size() == bboxes_B.size());
  for (const copyable_unique_ptr<ConvexSet>& set : convex_sets_A) {
    DRAKE_THROW_UNLESS(set != nullptr);
  }
  for (const copyable_unique_ptr<ConvexSet>& set : convex_sets_B) {
    DRAKE_THROW_UNLESS(set != nullptr);
  }
  const int dimension = convex_sets_A[0]->ambient_dimension();
  internal::ThrowsForInvalidContinuousJointsList(dimension,
                                                 continuous_revolute_joints);

  bool convex_sets_A_and_B_are_identical = convex_sets_A == convex_sets_B;

  // Ensure all set ambient dimensions match.
  for (int i = 0; i < ssize(convex_sets_A); ++i) {
    DRAKE_THROW_UNLESS(convex_sets_A[i]->ambient_dimension() ==
                       bboxes_A[i].ambient_dimension());
  }
  if (!convex_sets_A_and_B_are_identical) {
    for (int i = 0; i < ssize(convex_sets_B); ++i) {
      DRAKE_THROW_UNLESS(convex_sets_B[i]->ambient_dimension() ==
                         bboxes_B[i].ambient_dimension());
    }
  }

  // This vector will include a list of (ordered) pairs of sets which need to be
  // checked for intersection by solving a small optimization problem. We can
  // eliminate duplicates (if convex_sets_A == convex_sets_B) and sets with
  // non-overlapping bounding boxes.
  std::vector<std::pair<int, int>> candidate_edges;
  std::vector<VectorXd> candidate_edge_offsets;
  VectorXd offset = Eigen::VectorXd::Zero(dimension);
  for (int i = 0; i < ssize(convex_sets_A); ++i) {
    for (int j = 0; j < ssize(convex_sets_B); ++j) {
      if (convex_sets_A_and_B_are_identical && j <= i) {
        // If we're computing intersections within convex_sets_A and j <= i,
        // then we've already checked if we need to add an edge when i and j
        // were flipped, and that set has already been added.
        continue;
      }
      const auto& bbox_A = bboxes_A[i];
      // If convex_sets_A == convex_sets_B, then
      // region_minimum_and_maximum_values_B is empty, so we instead get the
      // bbox value from region_minimum_and_maximum_values_A.
      const auto& bbox_B =
          convex_sets_A_and_B_are_identical ? bboxes_A[j] : bboxes_B[j];

      offset.setZero();

      // First, we compute what the offset that should be applied to
      // convex_sets_A[i] to potentially make it overlap with convex_sets_B[j].
      for (int k = 0; k < ssize(continuous_revolute_joints); ++k) {
        const int joint_index = continuous_revolute_joints.at(k);
        if (bbox_A.lb()[joint_index] < bbox_B.lb()[joint_index]) {
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
              std::floor((bbox_B.ub()[joint_index] - bbox_A.lb()[joint_index]) /
                         (2 * M_PI));
        } else {
          // In this case, the minimum value of convex_sets_B[j] along dimension
          // k is smaller than the minimum value of convex_sets_A[i] along
          // dimension k. We do the same thing as above, but flip the order of
          // the sets. As a result, we also flip the sign of the resulting
          // translation.
          offset(joint_index) =
              -2 * M_PI *
              std::floor((bbox_A.ub()[joint_index] - bbox_B.lb()[joint_index]) /
                         (2 * M_PI));
        }
      }

      // Now that we know the offset for each dimension, we perform the cheap
      // axis-aligned bounding box intersection check, and premptively drop the
      // edge if that check fails. Otherwise, we need to use an optimization
      // problem to check the edge.

      // Note: if the user called the variant of this method which doesn't take
      // in bounding boxes and set the preprocess_bbox argument to false, then
      // the bounding box upper and lower limits will be set identically to zero
      // (except those which are computed for continuous revolute joints, as
      // required for that machinery). Thus, this check will trivially pass,
      // unless the values computed for continuous revolute joints happen to
      // confirm the sets don't intersect.
      if (!HyperrectangleOffsetIntersection(bbox_A, bbox_B, offset)) {
        continue;
      }

      candidate_edges.emplace_back(i, j);
      candidate_edge_offsets.emplace_back(offset);
    }
  }

  int n_candidates = ssize(candidate_edges);
  Eigen::MatrixXd Aeq(dimension, 2 * dimension);  // Aeq = [-I, I]
  Aeq.leftCols(dimension) = -Eigen::MatrixXd::Identity(dimension, dimension);
  Aeq.rightCols(dimension) = Eigen::MatrixXd::Identity(dimension, dimension);
  std::vector<MathematicalProgram> progs(n_candidates);
  for (int i = 0; i < n_candidates; ++i) {
    VectorXDecisionVariable x = progs[i].NewContinuousVariables(dimension);
    VectorXDecisionVariable y = progs[i].NewContinuousVariables(dimension);
    // Add x + offset == y by [-I, I][x; y] == [offset]
    progs[i].AddLinearEqualityConstraint(Aeq, candidate_edge_offsets[i],
                                         {x, y});
    convex_sets_A.at(candidate_edges[i].first)
        ->AddPointInSetConstraints(&(progs[i]), x);
    convex_sets_B.at(candidate_edges[i].second)
        ->AddPointInSetConstraints(&(progs[i]), y);
  }

  std::vector<const MathematicalProgram*> prog_ptrs;
  prog_ptrs.reserve(n_candidates);
  for (int i = 0; i < n_candidates; ++i) {
    prog_ptrs.push_back(&(progs[i]));
  }
  std::vector<MathematicalProgramResult> results = SolveInParallel(
      prog_ptrs, nullptr /* initial_guesses */, nullptr /* solver_options */,
      std::nullopt /* solver_id */, parallelism, false /* dynamic_schedule */);

  std::vector<std::pair<int, int>> edges;
  std::vector<Eigen::VectorXd> edge_offsets;
  for (int i = 0; i < n_candidates; ++i) {
    if (results[i].is_success()) {
      edges.emplace_back(candidate_edges[i]);
      edge_offsets.emplace_back(candidate_edge_offsets[i]);
      if (convex_sets_A_and_B_are_identical) {
        // Add the edge pointing in the opposite direction, with the negative
        // offset.
        edges.emplace_back(candidate_edges[i].second, candidate_edges[i].first);
        edge_offsets.emplace_back(-candidate_edge_offsets[i]);
      }
    }
  }

  return {edges, edge_offsets};
}

std::pair<std::vector<std::pair<int, int>>, std::vector<Eigen::VectorXd>>
ComputePairwiseIntersections(const ConvexSets& convex_sets,
                             const std::vector<int>& continuous_revolute_joints,
                             bool preprocess_bbox, Parallelism parallelism) {
  return ComputePairwiseIntersections(convex_sets, convex_sets,
                                      continuous_revolute_joints,
                                      preprocess_bbox, parallelism);
}

std::pair<std::vector<std::pair<int, int>>, std::vector<Eigen::VectorXd>>
ComputePairwiseIntersections(const ConvexSets& convex_sets,
                             const std::vector<int>& continuous_revolute_joints,
                             const std::vector<Hyperrectangle>& bboxes,
                             Parallelism parallelism) {
  return ComputePairwiseIntersections(convex_sets, convex_sets,
                                      continuous_revolute_joints, bboxes,
                                      bboxes, parallelism);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
