#include "drake/planning/approximate_convex_cover_builder_base.h"

#include <condition_variable>
#include <future>
#include <mutex>
#include <queue>
#include <thread>

#include "drake/common/ssize.h"
#include "drake/common/drake_copyable.h"

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace drake {
namespace planning {

using Eigen::SparseMatrix;
using Eigen::Triplet;
using geometry::optimization::ConvexSet;
using graph_algorithms::MaxCliqueSolverBase;

ApproximateConvexCoverFromCliqueCoverOptions::
    ApproximateConvexCoverFromCliqueCoverOptions(
        std::unique_ptr<CoverageCheckerBase> coverage_checker,
        std::unique_ptr<PointSamplerBase> point_sampler,
        std::unique_ptr<AdjacencyMatrixBuilderBase> adjacency_matrix_builder,
        std::unique_ptr<MaxCliqueSolverBase> max_clique_solver,
        std::vector<std::unique_ptr<ConvexSetFromCliqueBuilderBase>>
            set_builder,
        int num_sampled_points, int minimum_clique_size)
    : coverage_checker_(std::move(coverage_checker)),
      point_sampler_(std::move(point_sampler)),
      adjacency_matrix_builder_(std::move(adjacency_matrix_builder)),
      max_clique_solver_(std::move(max_clique_solver)),
      set_builders_(std::move(set_builder)),
      num_sampled_points_(num_sampled_points),
      minimum_clique_size_(minimum_clique_size) {
  DRAKE_THROW_UNLESS(set_builders_.size() > 0);
};

namespace {
// Sets all the value of the rows and columns in mat to false for which
// mask(i) is true.
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
}

/*
 * Computes the largest clique in the graph represented by @p adjacency_matrix
 * and adds this largest clique to @p computed_cliques. This clique is then
 * removed from the adjacency matrix, and a new maximal clique is computed.
 * This process is completed until no clique of size @p minimum_clique can be
 * found.
 * At this point, set the value of @p clique_cover_computed to true.
 *
 * This method is intended to be used only in
 * ApproximateConvexCoverFromCliqueCover.
 *
 * Note:
 * 1. Access to @p computed_cliques must be locked by @p computed_cliques_lock
 * as other worker threads will be asynchronously pulling off of this queue to
 * build convex sets in ApproximateConvexCoverFromCliqueCover.
 *
 * 2. The adjacency matrix will be mutated by this method and be mostly
 useless
 * after this method is called.
 *
 * @p clique_cover_computed must be set to true on the exit of this method,
 * otherwise ApproximateConvexCoverFromCliqueCover will loop forever.
 */
void ComputeGreedyTruncatedCliqueCover(
    const int minimum_clique_size,
    const graph_algorithms::MaxCliqueSolverBase& max_clique_solver,
    SparseMatrix<bool>* adjacency_matrix,
    std::queue<VectorX<bool>>* computed_cliques,
    std::mutex* computed_cliques_mutex,
    std::condition_variable* computed_clique_condition_variable,
    bool* clique_cover_complete) {
  int last_clique_size = std::numeric_limits<int>::infinity();
  std::unique_lock<std::mutex> lock(*computed_cliques_mutex);
  while (last_clique_size > minimum_clique_size) {
    const VectorX<bool> max_clique =
        max_clique_solver.SolveMaxClique(*adjacency_matrix);
    lock.lock();
    computed_cliques->push(max_clique);
    lock.unlock();
    computed_clique_condition_variable->notify_one();
    MakeFalseRowsAndColumns(max_clique, adjacency_matrix);
  }
  *clique_cover_complete = true;
  if (!clique_cover_complete) {
    DRAKE_UNREACHABLE();
  }
}

/*
 * Pulls cliques from @p computed_cliques and constructs convex sets using @p
 * points contained in the clique by calling @set_builder BuildConvexSet
 method.
 *
 * This process continues until @p clique_cover_complete gets set to true.
 *
 * This method is intended to be used only in
 * ApproximateConvexCoverFromCliqueCover.
 *
 * Note:
 * 1. Access to @p computed_cliques must be locked by @p computed_cliques_lock
 * as other worker threads will be asynchronously pulling off of this queue to
 * build convex sets in ApproximateConvexCoverFromCliqueCover.
 */
std::queue<copyable_unique_ptr<ConvexSet>> SetBuilderWorker(
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    ConvexSetFromCliqueBuilderBase* set_builder,
    std::queue<VectorX<bool>>* computed_cliques,
    std::mutex* computed_cliques_mutex,
    std::condition_variable* computed_clique_condition_variable,
    bool* clique_cover_complete) {
  std::queue<copyable_unique_ptr<ConvexSet>> ret;
  std::unique_lock<std::mutex> lock(*computed_cliques_mutex);
  while (true) {
    // wait until either notified or clique_cover_complete is true.
    computed_clique_condition_variable->wait(lock, [&clique_cover_complete] {
      return *clique_cover_complete;
    });
    if (*clique_cover_complete) {
      break;
    } else {
      lock.lock();
      const VectorX<bool> current_clique = computed_cliques->front();
      computed_cliques->pop();
      lock.unlock();

      const int clique_size = current_clique.sum();
      Eigen::MatrixXd clique_points(points.rows(), clique_size);
      for (int i = 0; i < ssize(current_clique); ++i) {
        if (current_clique(i)) {
          clique_points.col(i) = points.col(i);
        }
      }
      ret.push(set_builder->BuildConvexSet(clique_points));
    }
  }
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

}  // namespace

void ApproximateConvexCoverFromCliqueCover(
    const ApproximateConvexCoverFromCliqueCoverOptions& options,
    ConvexSets* convex_sets) {
  //  const int num_threads =
  //      options.num_threads() < 1
  //          ? static_cast<int>(std::thread::hardware_concurrency())
  //          : options.num_threads();
  //  bool parallelize = options.num_threads() == 1;

  // The computed cliques from the max clique solver. These will get pulled off
  // the queue by the set builder workers to build the sets.
  std::queue<VectorX<bool>> computed_cliques;
  // Used to lock access to the computed_cliques to avoid the threads racing.
  std::mutex computed_cliques_mutex;
  // This boolean will be used to signal to the set builder worker threads that
  // no more cliques will be added to the queue, so they can return their
  // sets to the main thread.
  bool stop_workers = false;

  // Condition variable used to notify threads when new values are added to the
  // computed_cliques.
  std::condition_variable computed_clique_condition_variable;

//  std::vector<copyable_unique_ptr<ConvexSet>> built_sets;
  while (!options.coverage_checker()->CheckCoverage(*convex_sets)) {
    stop_workers = false;
    // Sample points according to some distribution.
    const Eigen::MatrixXd points =
        options.point_sampler()->SamplePoints(options.num_sampled_points());

    // Build graphs from which we will compute cliques.
    Eigen::SparseMatrix<bool> adjacency_matrix =
        options.adjacency_matrix_builder()->BuildAdjacencyMatrix(points);

    // Reserve more space in for the newly built sets. Typically, we won't get
    // this worst case number of new cliques, so we only reserve half of the
    // worst case.
    convex_sets->reserve(
        convex_sets->size() +
        ComputeMaxNumberOfCliquesInGreedyCliqueCover(
            adjacency_matrix.cols(), options.minimum_clique_size()) /
            2);

    // Compute truncated clique cover.
    std::thread clique_cover_thread{[&options, &adjacency_matrix,
                                     &computed_cliques, &computed_cliques_mutex,
                                     &computed_clique_condition_variable,
                                     &stop_workers]() {
      ComputeGreedyTruncatedCliqueCover(
          options.minimum_clique_size(), *options.max_clique_solver(),
          &adjacency_matrix, &computed_cliques, &computed_cliques_mutex,
          &computed_clique_condition_variable, &stop_workers);
    }};

    // Build convex sets.
    std::vector<std::future<std::queue<copyable_unique_ptr<ConvexSet>>>>
        build_sets_future;
    build_sets_future.reserve(options.num_set_builders());
    for (int i = 0; i < options.num_set_builders(); ++i) {
      build_sets_future.emplace_back(
          std::async(std::launch::async, SetBuilderWorker, points,
                     options.get_set_builder(i), &computed_cliques,
                     &computed_cliques_mutex,
                     &computed_clique_condition_variable, &stop_workers));
    }

    // The clique cover and the convex sets are computed asynchronously. Wait
    // for all the threads to join and then add the new sets to built sets.
    clique_cover_thread.join();
    for (auto& new_set_queue_future : build_sets_future) {
      std::queue<copyable_unique_ptr<ConvexSet>> new_set_queue{
          new_set_queue_future.get()};
      while (!new_set_queue.empty()) {
        convex_sets->push_back(std::move(new_set_queue.front()));
        new_set_queue.pop();
      }
    }
  }
}

// std::vector<copyable_unique_ptr<ConvexSet>>
// ApproximateConvexCoverFromCliqueCover(
//    std::unique_ptr<CoverageCheckerBase> coverage_checker,
//    std::unique_ptr<PointSamplerBase> point_sampler,
//    std::unique_ptr<AdjacencyMatrixBuilderBase> adjacency_matrix_builder,
//    std::unique_ptr<MaxCliqueSolverBase> max_clique_solver,
//    std::vector<std::unique_ptr<ConvexSetFromCliqueBuilderBase>> set_builders,
//    int num_sampled_points, int minimum_clique_size) {
//  // The computed cliques from the max clique solver. These will get pulled
//  off
//  // the queue by the set builder workers to build the sets.
//  std::queue<VectorX<bool>> computed_cliques;
//  // Used to lock access to the computed_cliques to avoid the threads racing.
//  std::mutex computed_cliques_mutex;
//  // This boolean will be used to signal to the set builder worker threads
//  that
//  // no more cliques will be added to the queue, so they can return their sets
//  // to the main thread.
//  bool stop_workers = false;
//
//  // Condition variable used to notify threads when new values are added to
//  the
//  // computed_cliques.
//  std::condition_variable computed_clique_condition_variable;
//
//  std::vector<copyable_unique_ptr<ConvexSet>> built_sets;
//  while (!coverage_checker->CheckCoverage(built_sets)) {
//    stop_workers = false;
//    // Sample points according to some distribution.
//    const Eigen::MatrixXd points =
//        point_sampler->SamplePoints(num_sampled_points);
//
//    // Build graphs from which we will compute cliques.
//    Eigen::SparseMatrix<bool> adjacency_matrix =
//        adjacency_matrix_builder->BuildAdjacencyMatrix(points);
//
//    // Reserve more space in for the newly built sets. Typically, we won't get
//    // this worst case number of new cliques, so we only reserve half of the
//    // worst case.
//    built_sets.reserve(built_sets.size() +
//                       ComputeMaxNumberOfCliquesInGreedyCliqueCover(
//                           adjacency_matrix.cols(), minimum_clique_size) /
//                           2);
//
//    // Compute truncated clique cover.
//    std::thread clique_cover_thread{
//        [&adjacency_matrix, &max_clique_solver, &minimum_clique_size,
//         &computed_cliques, &computed_cliques_mutex,
//         &computed_clique_condition_variable, &stop_workers]() {
//          ComputeGreedyTruncatedCliqueCover(
//              minimum_clique_size, *max_clique_solver, &adjacency_matrix,
//              &computed_cliques, &computed_cliques_mutex,
//              &computed_clique_condition_variable, &stop_workers);
//        }};
//
//    // Build convex sets.
//    std::vector<std::future<std::queue<copyable_unique_ptr<ConvexSet>>>>
//        build_sets_future;
//    build_sets_future.reserve(set_builders.size());
//    for (int i = 0; i < ssize(set_builders); ++i) {
//      build_sets_future.emplace_back(std::async(
//          std::launch::async, SetBuilderWorker, points,
//          set_builders.at(i).get(), &computed_cliques,
//          &computed_cliques_mutex, &computed_clique_condition_variable,
//          &stop_workers));
//    }
//
//    // The clique cover and the convex sets are computed asynchronously. Wait
//    // for all the threads to join and then add the new sets to built sets.
//    clique_cover_thread.join();
//    for (auto& new_set_queue_future : build_sets_future) {
//      std::queue<copyable_unique_ptr<ConvexSet>> new_set_queue{
//          new_set_queue_future.get()};
//      while (!new_set_queue.empty()) {
//        built_sets.push_back(std::move(new_set_queue.front()));
//        new_set_queue.pop();
//      }
//    }
//  }
//  return built_sets;
//}

}  // namespace planning
}  // namespace drake
