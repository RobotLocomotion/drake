#include "drake/planning/approximate_convex_cover_builder_base.h"

#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "drake/common/drake_copyable.h"
#include "drake/common/ssize.h"

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace drake {
namespace planning {

using Eigen::SparseMatrix;
using Eigen::Triplet;
using geometry::optimization::ConvexSet;
using graph_algorithms::MaxCliqueSolverBase;

// ApproximateConvexCoverFromCliqueCoverOptions::
//    ApproximateConvexCoverFromCliqueCoverOptions(
//        std::unique_ptr<CoverageCheckerBase> coverage_checker,
//        std::unique_ptr<PointSamplerBase> point_sampler,
//        std::unique_ptr<AdjacencyMatrixBuilderBase> adjacency_matrix_builder,
//        std::unique_ptr<MaxCliqueSolverBase> max_clique_solver,
//        std::vector<std::unique_ptr<ConvexSetFromCliqueBuilderBase>>
//            set_builder,
//        int num_sampled_points, int minimum_clique_size)
//    : coverage_checker_(std::move(coverage_checker)),
//      point_sampler_(std::move(point_sampler)),
//      adjacency_matrix_builder_(std::move(adjacency_matrix_builder)),
//      max_clique_solver_(std::move(max_clique_solver)),
//      set_builders_(std::move(set_builder)),
//      num_sampled_points_(num_sampled_points),
//      minimum_clique_size_(minimum_clique_size) {
//  DRAKE_THROW_UNLESS(set_builders_.size() > 0);
//};

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
  *mat = mat->pruned();
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
  //  std::cout << "Launching Greedy Clique Cover" << std::endl;
  float last_clique_size = std::numeric_limits<float>::infinity();
  std::unique_lock<std::mutex> lock{*computed_cliques_mutex, std::defer_lock};
  while (last_clique_size > minimum_clique_size &&
         adjacency_matrix->nonZeros() > minimum_clique_size) {
    const VectorX<bool> max_clique =
        max_clique_solver.SolveMaxClique(*adjacency_matrix);
    last_clique_size = max_clique.template cast<int>().sum();
    if (last_clique_size > minimum_clique_size) {
      lock.lock();
      computed_cliques->push(max_clique);
      std::cout << "Clique pushed" << std::endl;
      lock.unlock();
      computed_clique_condition_variable->notify_one();
      MakeFalseRowsAndColumns(max_clique, adjacency_matrix);
    }
  }
  *clique_cover_complete = true;
  computed_clique_condition_variable->notify_all();
  std::cout << "MAX CLIQUE EXITTED" << std::endl;
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
  std::cout << "set builder launched" << std::endl;
  std::queue<copyable_unique_ptr<ConvexSet>> ret;
  std::unique_lock<std::mutex> lock{*computed_cliques_mutex};
  while (true) {
    if (!*clique_cover_complete) {
      // wait until notified to wake up if the clique cover is not complete.
      std::cout << "starting wait" << std::endl;
      computed_clique_condition_variable->wait(lock);
      std::cout << "wait over" << std::endl;
    }
    if (*clique_cover_complete && computed_cliques->size() == 0) {
      std::cout << "breaking set builder" << std::endl;
      break;
    } else {
      // The mutex is locked by the wait method releasing.
      const VectorX<bool> current_clique = computed_cliques->front();
      if (!lock.owns_lock()) {
        lock.lock();
      }
      computed_cliques->pop();
      std::cout << "clique popped" << std::endl;
      std::cout << "lock is owned = " << lock.owns_lock() << std::endl;
      lock.unlock();
      std::cout << "lock unlocked" << std::endl;
      const int clique_size = current_clique.template cast<int>().sum();
      Eigen::MatrixXd clique_points(points.rows(), clique_size);
      int clique_col = 0;
      for (int i = 0; i < ssize(current_clique); ++i) {
        if (current_clique(i)) {
          clique_points.col(clique_col) = points.col(i);
          ++clique_col;
        }
      }
      ret.push(set_builder->BuildConvexSet(clique_points));
      std::cout << "set made" << std::endl;
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
    CoverageCheckerBase* coverage_checker, PointSamplerBase* point_sampler,
    AdjacencyMatrixBuilderBase* adjacency_matrix_builder,
    MaxCliqueSolverBase* max_clique_solver,
    const std::vector<std::unique_ptr<ConvexSetFromCliqueBuilderBase>>&
        set_builders,
    const ApproximateConvexCoverFromCliqueCoverOptions& options,
    ConvexSets* convex_sets) {
  // TODO(Alexandre.Amice) Make sure that the minimum clique size is at least as
  // big as the ambient dimension.
  //  std::cout << "entered ApproximateConvexCoverFromCliqueCover" << std::endl;

  int itr{0};
  //  auto coverage_check_start = std::chrono::high_resolution_clock::now();
  while (!coverage_checker->CheckCoverage(*convex_sets)) {
    //    auto coverage_check_end = std::chrono::high_resolution_clock::now();
    //    std::cout << fmt::format("Coverage check took {}ms",
    //                             std::chrono::duration<double, std::milli>(
    //                                 coverage_check_end -
    //                                 coverage_check_start) .count())
    //              << std::endl;
    //    std::cout << "Starting itr = " << itr << std::endl;

    // Sample points according to some distribution.
    //    auto point_sampler_start = std::chrono::high_resolution_clock::now();
    const Eigen::MatrixXd points =
        point_sampler->SamplePoints(options.num_sampled_points);
    //    auto point_sampler_end = std::chrono::high_resolution_clock::now();
    //    std::cout << fmt::format("points sampled in {}ms",
    //                             std::chrono::duration<double, std::milli>(
    //                                 point_sampler_end - point_sampler_start)
    //                                 .count())
    //              << std::endl;

    // Build graphs from which we will compute cliques.
    //    auto adjacency_matrix_start =
    //    std::chrono::high_resolution_clock::now();
    Eigen::SparseMatrix<bool> adjacency_matrix =
        adjacency_matrix_builder->BuildAdjacencyMatrix(points);
    //    auto adjacency_matrix_end = std::chrono::high_resolution_clock::now();
    //    std::cout << fmt::format("graph built sampled in {}ms",
    //                             std::chrono::duration<double, std::milli>(
    //                                 adjacency_matrix_end -
    //                                 adjacency_matrix_start) .count())
    //              << std::endl;

    // Reserve more space in for the newly built sets. Typically, we won't get
    // this worst case number of new cliques, so we only reserve half of the
    // worst case.
    convex_sets->reserve(
        convex_sets->size() +
        ComputeMaxNumberOfCliquesInGreedyCliqueCover(
            adjacency_matrix.cols(), options.minimum_clique_size) /
            2);

    // The computed cliques from the max clique solver. These will get pulled
    // off
    // the queue by the set builder workers to build the sets.
    std::queue<VectorX<bool>> computed_cliques;
    // Used to lock access to the computed_cliques to avoid the threads racing.
    std::mutex computed_cliques_mutex;
    // This boolean will be used to signal to the set builder worker threads
    // that no more cliques will be added to the queue, so they can return their
    // sets to the main thread.
    bool stop_workers = false;

    // Condition variable used to notify threads when new values are added to
    // the computed_cliques.
    std::condition_variable computed_clique_condition_variable;

    // Compute truncated clique cover.
    std::thread clique_cover_thread{
        [&adjacency_matrix, &max_clique_solver, &options, &computed_cliques,
         &computed_cliques_mutex, &computed_clique_condition_variable,
         &stop_workers]() {
          ComputeGreedyTruncatedCliqueCover(
              options.minimum_clique_size, *max_clique_solver,
              &adjacency_matrix, &computed_cliques, &computed_cliques_mutex,
              &computed_clique_condition_variable, &stop_workers);
        }};

    // The clique cover and the convex sets are computed asynchronously. Wait
    // for all the threads to join and then add the new sets to built sets.
    // TODO(Alexand.Amice) need to move this down after the build sets futures, but for now that's not working.
    clique_cover_thread.join();

    // Build convex sets.
    std::vector<std::future<std::queue<copyable_unique_ptr<ConvexSet>>>>
        build_sets_future;
    build_sets_future.reserve(set_builders.size());
    for (int i = 0; i < ssize(set_builders); ++i) {
      build_sets_future.emplace_back(std::async(
          std::launch::async, SetBuilderWorker, points,
          set_builders.at(i).get(), &computed_cliques, &computed_cliques_mutex,
          &computed_clique_condition_variable, &stop_workers));
    }



    //    std::cout << "clique_cover_thread joined" << std::endl;
    for (auto& new_set_queue_future : build_sets_future) {
      std::queue<copyable_unique_ptr<ConvexSet>> new_set_queue{
          new_set_queue_future.get()};
      //      std::cout << "std future got" << std::endl;
      while (!new_set_queue.empty()) {
        convex_sets->push_back(std::move(new_set_queue.front()));
        new_set_queue.pop();
      }
    }

    ++itr;
    //    coverage_check_start = std::chrono::high_resolution_clock::now();
    //    std::cout << "size of sets = " << convex_sets->size() << std::endl;
    //    for (const auto& set : *convex_sets) {
    ////      std::cout << "set is bounded = " << set->IsBounded() << std::endl;
    //    }
    //    std::cout << std::endl;
  }
}

}  // namespace planning
}  // namespace drake
