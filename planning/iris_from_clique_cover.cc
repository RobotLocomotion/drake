#include "drake/planning/iris_from_clique_cover.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <future>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include "drake/common/ssize.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/planning/visibility_graph.h"

namespace drake {
namespace planning {

using Eigen::SparseMatrix;
using Eigen::Triplet;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::IrisInConfigurationSpace;
using geometry::optimization::IrisOptions;
using graph_algorithms::MaxCliqueSolverBase;

namespace {
// A very simple implementation of a thread-safe asynchronous queue for use when
// exactly one threads fills the queue while one (or more) threads empties the
// queue.
//
// When the producer pushes onto the queue, they automatically signal to the
// consumers that a job is available. When the producer threads is done filling
// the queue, they signal this via the done_filling() method. i.e the producer
// code should be similar to: while(true) { queue.push(do_work(..)); if(done) {
//   queue.stop_filling()
// }
//
// Consumers threads should be implemented as:
// T elt;
// while(queue.pop(elt)) {
//  do_work(...)
// }
// The loop will exits when the producer thread has both signalled that no
// more jobs are being created, and when the underlying queue is completely
// empty. Note that this loop will not poll the pop method unnecessarily, as if
// the queue is filling, pop will wait until a job is available.
template <typename T>
class AsyncQueue {
 public:
  void push(const T& value) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(value);
    lock.unlock();
    condition_.notify_one();
  }

  // A modification of the pop method. Returns true if a job was successfully
  // acquired. If the queue is still filling, pop will wait for a job rather
  // than returning spuriously. Only returns false when both the queue is done
  // filling and when the queue is completely empty.
  // Marked no lint since the linter really dislikes passing a non-const
  // reference.
  bool pop(T& write_reference /*NOLINT*/) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] {
      return !queue_.empty() || !still_filling_;
    });
    if (!queue_.empty()) {
      write_reference = queue_.front();
      queue_.pop();
      return true;
    }
    return false;
  }

  // Signal that the producer thread will stop producing jobs.
  void done_filling() {
    still_filling_ = false;
    condition_.notify_all();
  }

  int size() { return queue_.size(); }

 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool still_filling_ = true;
};

// Sets mat(i, :) and mat(:, i) to false if mask(i) is true.
void MakeFalseRowsAndColumns(const VectorX<bool>& mask,
                             SparseMatrix<bool>* mat) {
  for (int j = 0; j < mat->outerSize(); ++j) {
    if (mask[j]) {
      for (SparseMatrix<bool>::InnerIterator it(*mat, j); it; ++it) {
        if (mask[it.index()]) {
          it.valueRef() = false;
        }
      }
    }
  }
  *mat = mat->pruned();
}

// Computes the largest clique in the graph represented by @p adjacency_matrix
// and adds this largest clique to @p computed_cliques. This clique is then
// removed from the adjacency matrix, and a new maximal clique is computed.
// This process continue until no clique of size @p minimum_clique can be
// found. At this point, set the value of @p clique_cover_computed to true.
//
// Note:
// 1. Access to @p computed_cliques must be locked by @p computed_cliques_lock
// as other worker threads will be asynchronously pulling off of this queue to
// build convex sets in IrisInConfigurationSpaceFromCliqueCover.
//
// 2. The adjacency matrix will be mutated by this method and be mostly useless
// after this method is called.
void ComputeGreedyTruncatedCliqueCover(
    const int minimum_clique_size,
    const graph_algorithms::MaxCliqueSolverBase& max_clique_solver,
    SparseMatrix<bool>* adjacency_matrix,
    AsyncQueue<VectorX<bool>>* computed_cliques) {
  float last_clique_size = std::numeric_limits<float>::infinity();
  int num_cliques = 0;
  log()->info("Solving Max Clique with {} points", adjacency_matrix->cols());
  while (last_clique_size > minimum_clique_size &&
         adjacency_matrix->nonZeros() > minimum_clique_size) {
    const VectorX<bool> max_clique =
        max_clique_solver.SolveMaxClique(*adjacency_matrix);
    last_clique_size = max_clique.template cast<int>().sum();
    if (last_clique_size > minimum_clique_size) {
      computed_cliques->push(max_clique);
      ++num_cliques;
      MakeFalseRowsAndColumns(max_clique, adjacency_matrix);
      log()->info(
          "Clique added to queue. There are {}/{} points left to cover.",
          adjacency_matrix->nonZeros() / 2, adjacency_matrix->cols());
    }
  }
  computed_cliques->done_filling();
  log()->info(
      "Finished adding cliques. Total of {} added. Number of cliques left to "
      "process = {}",
      num_cliques, computed_cliques->size());
}

// Pulls cliques from @p computed_cliques and constructs Iris regions by seeding
// Iris with the minimum circumscribed ellipse.
std::queue<HPolyhedron> IrisWorker(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& points, const int builder_id,
    const IrisFromCliqueCoverOptions& options,
    AsyncQueue<VectorX<bool>>* computed_cliques) {
  // Copy the iris options as we will change the value of the starting ellipse
  // in this worker.
  IrisOptions iris_options = options.iris_options;

  std::queue<HPolyhedron> ret{};
  VectorX<bool> current_clique;
  while (computed_cliques->pop(current_clique)) {
    const int clique_size = current_clique.template cast<int>().sum();
    Eigen::MatrixXd clique_points(points.rows(), clique_size);
    int clique_col = 0;
    for (int i = 0; i < ssize(current_clique); ++i) {
      if (current_clique(i)) {
        clique_points.col(clique_col) = points.col(i);
        ++clique_col;
      }
    }
    Hyperellipsoid clique_ellipse;
    try {
      clique_ellipse = Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(
          clique_points, options.rank_tol_for_lowner_john_ellipse);
    } catch (const std::runtime_error& e) {
      log()->info(
          "Iris builder thread {} failed to compute an ellipse for a clique.",
          builder_id, e.what());
      continue;
    }

    if (checker.CheckConfigCollisionFree(clique_ellipse.center())) {
      iris_options.starting_ellipse = clique_ellipse;
    } else {
      // Find the nearest clique member to the center that is not in collision.
      Eigen::Index nearest_point_col;
      (clique_points - clique_ellipse.center())
          .colwise()
          .norm()
          .maxCoeff(&nearest_point_col);
      Eigen::VectorXd center = clique_points.col(nearest_point_col);
      iris_options.starting_ellipse =
          Hyperellipsoid(center, clique_ellipse.A());
    }
    checker.UpdatePositions(iris_options.starting_ellipse->center(),
                            builder_id);
    log()->debug("Iris builder thread {} is constructing a set.", builder_id);
    ret.emplace(IrisInConfigurationSpace(
        checker.plant(), checker.plant_context(builder_id), iris_options));
    log()->debug("Iris builder thread  {} has constructed a set.", builder_id);
  }
  log()->debug("Iris builder thread {} has completed.", builder_id);
  return ret;
}

int ComputeMaxNumberOfCliquesInGreedyCliqueCover(
    const int num_vertices, const int minimum_clique_size) {
  // From "Restricted greedy clique decompositions and greedy clique
  // decompositions of K 4-free graphs" by Sean McGuinness, we have that the
  // most cliques that we could obtain from the greedy truncated clique
  // cover is the number of edges in the Turan graph T(num_vertices,
  // minimum_clique_size). This number is
  // 0.5* (1−1/r) * (num_vertices² − s²)  +  (s choose 2)
  // Where  num_vertices= q*minimum_clique_size+s
  const int q = std::floor(num_vertices / minimum_clique_size);
  const int s = num_vertices - q * minimum_clique_size;
  return static_cast<int>((1 - 1.0 / minimum_clique_size) *
                              (num_vertices * num_vertices - s * s) / 2 +
                          (s * (s + 1)) / 2);
}

// Approximately compute the fraction of `domain` covered by `sets` by sampling
// points uniformly at random in `domain` and checking whether the point lies in
// one of the sets in `sets`.
//
// The `generator` is used as the source of randomness for drawing points from
// domain.
//
// The sampled points are checked for inclusion  in `sets` in parallel, with the
// degree of parallelism determined by `parallelism`.
//
// The value of `last_polytope_sample` is used to initialize the distribution
// over the domain. The value of the final sample is written to this vector so
// that the MCMC sampling can continue. See @HPolyhedron for details.
double ApproximatelyComputeCoverage(const HPolyhedron& domain,
                                    const std::vector<HPolyhedron>& sets,
                                    const int num_samples,
                                    const double point_in_set_tol,
                                    const Parallelism& parallelism,
                                    RandomGenerator* generator,
                                    Eigen::VectorXd* last_polytope_sample) {
  double fraction_covered = 0.0;
  if (sets.empty()) {
    log()->info("Current Fraction of Domain Covered = 0");
    // Fail fast if there is nothing to check.
    return 0.0;
  }
  if (!sets.empty()) {
    Eigen::MatrixXd sampled_points(domain.ambient_dimension(), num_samples);
    for (int i = 0; i < sampled_points.cols(); ++i) {
      *last_polytope_sample =
          domain.UniformSample(generator, *last_polytope_sample);
      sampled_points.col(i) = *last_polytope_sample;
    }

    std::atomic<int> num_in_sets{0};
#if defined(_OPENMP)
#pragma omp parallel for num_threads(parallelism.num_threads()) schedule(static)
#endif
    for (int i = 0; i < sampled_points.cols(); ++i) {
      for (const auto& set : sets) {
        if (set.PointInSet(sampled_points.col(i), point_in_set_tol)) {
          num_in_sets.fetch_add(1);
          break;
        }
      }
    }
    fraction_covered = static_cast<double>(num_in_sets.load()) / num_samples;
  }
  log()->info("Current Fraction of Domain Covered = {}", fraction_covered);
  return fraction_covered;
}
}  // namespace

void IrisInConfigurationSpaceFromCliqueCover(
    const CollisionChecker& checker, const IrisFromCliqueCoverOptions& options,
    RandomGenerator* generator, std::vector<HPolyhedron>* sets) {
  DRAKE_THROW_UNLESS(options.coverage_termination_threshold > 0);
  DRAKE_THROW_UNLESS(options.iteration_limit > 0);

  const HPolyhedron& domain = options.iris_options.bounding_region.value_or(
      HPolyhedron::MakeBox(checker.plant().GetPositionLowerLimits(),
                           checker.plant().GetPositionUpperLimits()));
  Eigen::VectorXd last_polytope_sample = domain.UniformSample(generator);

  // Override the default options.
  const int minimum_clique_size =
      std::max(options.minimum_clique_size, checker.plant().num_positions());
  int num_points_per_visibility_round = std::max(
      options.num_points_per_visibility_round, 2 * minimum_clique_size);
  Parallelism num_builders{std::min(options.num_builders.num_threads(),
                                    checker.num_allocated_contexts())};
  Parallelism visibility_graph_parallelism{
      std::min(options.visibility_graph_parallelism.num_threads(),
               checker.num_allocated_contexts())};

  int num_iterations = 0;
  while (!ApproximatelyComputeCoverage(
             domain, *sets, options.num_points_per_coverage_check,
             options.point_in_set_tol, options.num_coverage_checkers, generator,
             &last_polytope_sample) &&
         num_iterations < options.iteration_limit) {
    Eigen::MatrixXd points(domain.ambient_dimension(),
                           num_points_per_visibility_round);
    for (int i = 0; i < points.cols(); ++i) {
      do {
        last_polytope_sample =
            domain.UniformSample(generator, last_polytope_sample);
      } while (  // while the last polytope sample is in any of the sets.
          std::any_of(sets->begin(), sets->end(),
                      [&last_polytope_sample](const HPolyhedron& set) -> bool {
                        return set.PointInSet(last_polytope_sample);
                      }));
      points.col(i) = last_polytope_sample;
    }

    Eigen::SparseMatrix<bool> visibility_graph =
        VisibilityGraph(checker, points, visibility_graph_parallelism);
    // Reserve more space for the newly built sets. Typically, we won't get
    // this worst case number of new cliques, so we only reserve half of the
    // worst case.
    sets->reserve(sets->size() +
                  ComputeMaxNumberOfCliquesInGreedyCliqueCover(
                      visibility_graph.cols(), options.minimum_clique_size) /
                      2);

    // The computed cliques from the max clique solver. These will get pulled
    // off the queue by the set builder workers to build the sets.
    AsyncQueue<VectorX<bool>> computed_cliques;

    // Compute truncated clique cover.
    std::thread clique_cover_thread{[&visibility_graph, &options,
                                     &computed_cliques]() {
      ComputeGreedyTruncatedCliqueCover(options.minimum_clique_size,
                                        *options.max_clique_solver,
                                        &visibility_graph, &computed_cliques);
    }};

    // Build convex sets.
    std::vector<std::future<std::queue<HPolyhedron>>> build_sets_future;
    build_sets_future.reserve(num_builders.num_threads());
    for (int i = 0; i < num_builders.num_threads(); ++i) {
      build_sets_future.emplace_back(
          std::async(std::launch::async, IrisWorker, std::ref(checker), points,
                     i, std::ref(options), &computed_cliques));
    }
    // The clique cover and the convex sets are computed asynchronously. Wait
    // for all the threads to join and then add the new sets to built sets.
    clique_cover_thread.join();
    int num_new_sets{0};
    for (auto& new_set_queue_future : build_sets_future) {
      std::queue<HPolyhedron> new_set_queue{new_set_queue_future.get()};
      while (!new_set_queue.empty()) {
        sets->push_back(std::move(new_set_queue.front()));
        new_set_queue.pop();
        ++num_new_sets;
      }
    }
    if (num_new_sets == 0) {
      num_points_per_visibility_round *= 2;
    }
    ++num_iterations;
  }
}
}  // namespace planning
}  // namespace drake
