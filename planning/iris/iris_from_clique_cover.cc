#include "drake/planning/iris/iris_from_clique_cover.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <future>
#include <limits>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <utility>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/overloaded.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/planning/visibility_graph.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace planning {
using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::ParallelForBackend;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;
using Eigen::SparseMatrix;
using geometry::Meshcat;
using geometry::Rgba;
using geometry::Sphere;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::IrisNp;
using geometry::optimization::IrisOptions;
using graph_algorithms::MaxCliqueSolverBase;
using math::RigidTransform;

namespace {
// A very simple implementation of a thread-safe asynchronous queue for use when
// exactly one thread fills the queue while one or more threads empty the
// queue.
//
// When the producer pushes onto the queue, they automatically signal to the
// consumers that a job is available. When the producer thread is done filling
// the queue, they signal this via the done_filling() method. i.e the producer
// code should be similar to:
// while(true) {
//   queue.push(do_work(..));
//   if(done) {
//      queue.done_filling();
//      break;
//   }
// }
//
// Consumers threads should be implemented as:
// T elt;
// while(queue.pop(elt)) {
//  do_work(...)
// }
// The loop will exit when:
//   * The producer thread has both signalled that no more jobs are being
//   created.
//   * The underlying queue is completely empty.
// Note that this loop will not poll the pop method unnecessarily, as if
// the queue is filling, pop will wait until a job is available.
template <typename T>
class AsyncQueue {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AsyncQueue);

  AsyncQueue() : queue_{}, mutex_{}, condition_{}, still_filling_{true} {};

  void push(const T& value) {
    DRAKE_THROW_UNLESS(still_filling_);
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(value);
    condition_.notify_one();
  }

  // A modification of the pop method which acquires an element of type T if the
  // queue is non-empty. If the queue is empty, but still filling, then pop will
  // wait until an element becomes available. If the queue is empty and done
  // filling, this method will return nullopt.
  std::optional<T> pop() {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait(lock, [this] {
      return !queue_.empty() || !still_filling_;
    });
    if (!queue_.empty()) {
      const T ret = queue_.front();
      queue_.pop();
      return ret;
    }
    return std::nullopt;
  }

  // Signal that the producer thread will stop producing jobs.
  void done_filling() {
    still_filling_ = false;
    condition_.notify_all();
  }

  int size() const {
    // We need to lock the mutex since the queue size is not necessarily atomic.
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.size();
  }

 private:
  std::queue<T> queue_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic<bool> still_filling_ = true;
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

Meshcat* GetMeshcatFromOptions(
    const std::variant<IrisOptions, IrisNp2Options, IrisZoOptions>&
        iris_options) {
  return std::visit(
      overloaded{[](const IrisOptions& options) {
                   return options.meshcat.get();
                 },
                 [](const IrisNp2Options& options) {
                   return options.sampled_iris_options.meshcat.get();
                 },
                 [](const IrisZoOptions& options) {
                   return options.sampled_iris_options.meshcat.get();
                 }},
      iris_options);
}

// Computes the largest clique in the graph represented by @p adjacency_matrix
// and adds this largest clique to @p computed_cliques. This clique is then
// removed from the adjacency matrix, and a new maximal clique is computed.
// This process continues until no clique of size @p minimum_clique can be
// found. At this point, the queue calls done_filling() so that downstream
// workers know to expect no more cliques.
//
// Note: The adjacency matrix will be mutated by this method and will be mostly
// useless after this method is called.
void ComputeGreedyTruncatedCliqueCover(
    const int minimum_clique_size,
    const graph_algorithms::MaxCliqueSolverBase& max_clique_solver,
    SparseMatrix<bool>* adjacency_matrix,
    AsyncQueue<VectorX<bool>>* computed_cliques) {
  int last_clique_size = std::numeric_limits<int>::max();
  int num_cliques = 0;
  int num_points_left = adjacency_matrix->cols();
  const int num_points_original = adjacency_matrix->cols();
  while (last_clique_size > minimum_clique_size &&
         num_points_left > minimum_clique_size) {
    const VectorX<bool> max_clique =
        max_clique_solver.SolveMaxClique(*adjacency_matrix);
    last_clique_size = max_clique.template cast<int>().sum();
    log()->debug("Last Clique Size = {}", last_clique_size);
    num_points_left -= last_clique_size;
    if (last_clique_size >= minimum_clique_size) {
      computed_cliques->push(max_clique);
      ++num_cliques;
      MakeFalseRowsAndColumns(max_clique, adjacency_matrix);
      log()->debug(
          "Clique added to queue. There are {}/{} points left to cover.",
          num_points_left, num_points_original);
    }
  }
  // This line signals to the IRIS workers that no further cliques will be added
  // to the queue. Removing this line will cause an infinite loop.
  computed_cliques->done_filling();
  log()->debug(
      "Finished adding cliques. Total of {} cliques added. Number of cliques "
      "left to process = {}",
      num_cliques, computed_cliques->size());
}

// Pulls cliques from @p computed_cliques and constructs IRIS regions using the
// provided IrisOptions, but seeding IRIS with the minimum circumscribed
// ellipse of the clique. As this method may run in a separate thread, we
// provide an option to forcefully disable meshcat in IRIS. This must happen as
// meshcat cannot be written to outside the main thread.
std::queue<HPolyhedron> IrisWorker(
    const CollisionChecker& checker,
    const Eigen::Ref<const Eigen::MatrixXd>& points, const int builder_id,
    const IrisFromCliqueCoverOptions& options, const HPolyhedron& domain,
    AsyncQueue<VectorX<bool>>* computed_cliques, bool disable_meshcat = true) {
  // Copy the IrisOptions as we will change the value of the starting ellipse
  // in this worker.
  std::variant<IrisOptions, IrisNp2Options, IrisZoOptions> iris_options =
      options.iris_options;

  // Disable the IRIS meshcat option in this worker since we cannot write to
  // meshcat from a different thread.
  if (disable_meshcat) {
    std::visit(overloaded{[](IrisOptions& arg) {
                            arg.meshcat = nullptr;
                          },
                          [](IrisNp2Options& arg) {
                            arg.sampled_iris_options.meshcat = nullptr;
                          },
                          [](IrisZoOptions& arg) {
                            arg.sampled_iris_options.meshcat = nullptr;
                          }},
               iris_options);
  }

  std::queue<HPolyhedron> ret{};
  std::optional<VectorX<bool>> current_clique = computed_cliques->pop();
  while (current_clique.has_value()) {
    const int clique_size = current_clique->template cast<int>().sum();
    Eigen::MatrixXd clique_points(points.rows(), clique_size);
    int clique_col = 0;
    for (int i = 0; i < ssize(current_clique.value()); ++i) {
      if (current_clique.value()(i)) {
        clique_points.col(clique_col) = points.col(i);
        ++clique_col;
      }
    }
    Hyperellipsoid clique_ellipse;
    try {
      clique_ellipse = Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(
          clique_points,
          options.rank_tol_for_minimum_volume_circumscribed_ellipsoid);
    } catch (const std::runtime_error& e) {
      log()->debug(
          "Iris builder thread {} failed to compute an ellipse for a clique.",
          builder_id, e.what());
      current_clique = computed_cliques->pop();
      continue;
    }
    if (!checker.CheckConfigCollisionFree(clique_ellipse.center(),
                                          builder_id)) {
      Eigen::Index nearest_point_col;
      (clique_points.colwise() - clique_ellipse.center())
          .colwise()
          .norm()
          .minCoeff(&nearest_point_col);
      Eigen::VectorXd center = clique_points.col(nearest_point_col);
      clique_ellipse = Hyperellipsoid(clique_ellipse.A(), center);
    }
    checker.UpdatePositions(clique_ellipse.center(), builder_id);
    log()->debug("Iris builder thread {} is constructing a set.", builder_id);
    std::visit(
        overloaded{
            [&](IrisOptions& arg) {
              arg.starting_ellipse = clique_ellipse;
              ret.emplace(IrisNp(checker.plant(),
                                 checker.plant_context(builder_id), arg));
            },
            [&](IrisNp2Options& arg) {
              arg.sampled_iris_options.parallelism = options.parallelism;
              ret.emplace(IrisNp2(
                  dynamic_cast<const SceneGraphCollisionChecker&>(checker),
                  clique_ellipse, domain, arg));
            },
            [&](IrisZoOptions& arg) {
              arg.sampled_iris_options.parallelism = options.parallelism;
              ret.emplace(IrisZo(checker, clique_ellipse, domain, arg));
            }},
        iris_options);

    current_clique = computed_cliques->pop();
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
// The sampled points are checked for inclusion in `sets` in parallel, with the
// degree of parallelism determined by `parallelism`.
//
// The value of `last_polytope_sample` is used to initialize the distribution
// over the domain. The value of the final sample is written to this vector so
// that the MCMC sampling can continue. See @HPolyhedron for details.
double ApproximatelyComputeCoverage(
    const HPolyhedron& domain, const std::vector<HPolyhedron>& sets,
    const CollisionChecker& checker, const int num_samples,
    const double point_in_set_tol, const Parallelism& parallelism,
    RandomGenerator* generator, Eigen::VectorXd* last_polytope_sample) {
  double fraction_covered = 0.0;
  if (sets.empty()) {
    log()->info("Current Fraction of Domain Covered = 0");
    // Fail fast if there is nothing to check.
    return 0.0;
  }
  Eigen::MatrixXd sampled_points(domain.ambient_dimension(), num_samples);
  for (int i = 0; i < sampled_points.cols(); ++i) {
    do {
      *last_polytope_sample =
          domain.UniformSample(generator, *last_polytope_sample);
    } while (!checker.CheckConfigCollisionFree(*last_polytope_sample));
    sampled_points.col(i) = *last_polytope_sample;
  }

  std::atomic<int> num_in_sets{0};
  const auto point_in_cover = [&sets, &num_in_sets, &sampled_points,
                               &point_in_set_tol](const int thread_num,
                                                  const int i) {
    unused(thread_num);
    for (const auto& set : sets) {
      if (set.PointInSet(sampled_points.col(i), point_in_set_tol)) {
        num_in_sets.fetch_add(1);
        break;
      }
    }
  };
  StaticParallelForIndexLoop(DegreeOfParallelism(parallelism.num_threads()), 0,
                             sampled_points.cols(), point_in_cover,
                             ParallelForBackend::BEST_AVAILABLE);

  fraction_covered = static_cast<double>(num_in_sets.load()) / num_samples;

  log()->debug("Current Fraction of Domain Covered = {}", fraction_covered);
  return fraction_covered;
}

std::unique_ptr<planning::graph_algorithms::MaxCliqueSolverBase>
MakeDefaultMaxCliqueSolver() {
  return std::unique_ptr<planning::graph_algorithms::MaxCliqueSolverBase>(
      new planning::graph_algorithms::MaxCliqueSolverViaGreedy());
}

// Checks all the preconditions for running
// IrisInConfigurationSpaceFromCliqueCover.
void CheckIrisInConfigurationSpaceFromCliqueCoverPreconditions(
    const CollisionChecker& checker,
    const IrisFromCliqueCoverOptions& options) {
  DRAKE_THROW_UNLESS(options.coverage_termination_threshold > 0);
  DRAKE_THROW_UNLESS(options.iteration_limit > 0);

  // iris option specific checks
  std::visit(
      overloaded{
          [](const IrisOptions& iris_options) {
            DRAKE_THROW_UNLESS(iris_options.prog_with_additional_constraints ==
                               nullptr);
          },
          [&checker](const IrisNp2Options& iris_options) {
            DRAKE_THROW_UNLESS(iris_options.sampled_iris_options
                                   .prog_with_additional_constraints ==
                               nullptr);
            DRAKE_THROW_UNLESS(dynamic_cast<const SceneGraphCollisionChecker*>(
                                   &checker) != nullptr);
          },
          [](const IrisZoOptions& iris_options) {
            DRAKE_THROW_UNLESS(iris_options.sampled_iris_options
                                   .prog_with_additional_constraints ==
                               nullptr);
          }},
      options.iris_options);

  // Throw if a parameterization is used.
  const std::optional<IrisParameterizationFunction> parameterization =
      std::visit(
          overloaded{[](const IrisOptions& iris_options) {
                       unused(iris_options);
                       return std::optional<IrisParameterizationFunction>{};
                     },
                     [](const IrisNp2Options& iris_options) {
                       return std::optional<IrisParameterizationFunction>{
                           iris_options.parameterization};
                     },
                     [](const IrisZoOptions& iris_options) {
                       return std::optional<IrisParameterizationFunction>{
                           iris_options.parameterization};
                     }},
          options.iris_options);
  if (parameterization.has_value()) {
    const int ndim =
        parameterization->get_parameterization_dimension().value_or(
            checker.plant().num_positions());
    const Eigen::VectorXd parameterization_eval =
        parameterization->get_parameterization_double()(
            Eigen::VectorXd::Zero(ndim));
    drake::log()->debug("Parameterization eval = {}",
                        fmt_eigen(parameterization_eval));

    if (!parameterization_eval.isZero()) {
      throw std::runtime_error(
          "IrisFromCliqueCover does not yet support growing regions on "
          "a parameterized subspace ");
    }
  }

  // Note: Even though the iris_options.bounding_region may be
  // provided, IrisNp (currently) requires finite joint limits.
  DRAKE_THROW_UNLESS(
      checker.plant().GetPositionLowerLimits().array().isFinite().all());
  DRAKE_THROW_UNLESS(
      checker.plant().GetPositionUpperLimits().array().isFinite().all());
}

}  // namespace

void IrisInConfigurationSpaceFromCliqueCover(
    const CollisionChecker& checker, const IrisFromCliqueCoverOptions& options,
    RandomGenerator* generator, std::vector<HPolyhedron>* sets,
    const planning::graph_algorithms::MaxCliqueSolverBase*
        max_clique_solver_ptr) {
  CheckIrisInConfigurationSpaceFromCliqueCoverPreconditions(checker, options);

  const HPolyhedron default_domain =
      HPolyhedron::MakeBox(checker.plant().GetPositionLowerLimits(),
                           checker.plant().GetPositionUpperLimits());
  const HPolyhedron domain = std::visit(
      overloaded{[&default_domain](const IrisOptions& iris_options) {
                   return iris_options.bounding_region.value_or(default_domain);
                 },
                 [&default_domain](const IrisNp2Options&) {
                   return default_domain;
                 },
                 [&default_domain](const IrisZoOptions&) {
                   return default_domain;
                 }},
      options.iris_options);

  DRAKE_THROW_UNLESS(domain.ambient_dimension() ==
                     checker.plant().num_positions());
  Eigen::VectorXd last_polytope_sample = domain.UniformSample(generator);

  // Override options which are set too aggressively.
  const int minimum_clique_size = std::max(options.minimum_clique_size,
                                           checker.plant().num_positions() + 1);

  int num_points_per_visibility_round = std::max(
      options.num_points_per_visibility_round, 2 * minimum_clique_size);

  Parallelism max_collision_checker_parallelism{std::min(
      options.parallelism.num_threads(), checker.num_allocated_contexts())};

  log()->debug("Visibility Graph will use {} threads",
               max_collision_checker_parallelism.num_threads());

  int num_iterations = 0;

  std::vector<HPolyhedron> visibility_graph_sets;

  std::unique_ptr<planning::graph_algorithms::MaxCliqueSolverBase>
      default_max_clique_solver;
  // Only construct the default solver if max_clique_solver is null.
  if (max_clique_solver_ptr == nullptr) {
    default_max_clique_solver = MakeDefaultMaxCliqueSolver();
    log()->debug("Using default max clique solver MaxCliqueSolverViaGreedy.");
  }

  const planning::graph_algorithms::MaxCliqueSolverBase* max_clique_solver =
      max_clique_solver_ptr == nullptr ? default_max_clique_solver.get()
                                       : max_clique_solver_ptr;
  auto approximate_coverage = [&]() {
    return ApproximatelyComputeCoverage(
        domain, *sets, checker, options.num_points_per_coverage_check,
        options.point_in_set_tol, options.parallelism, generator,
        &last_polytope_sample);
  };
  while (approximate_coverage() < options.coverage_termination_threshold &&
         num_iterations < options.iteration_limit) {
    log()->info("IrisFromCliqueCover Iteration {}/{}", num_iterations + 1,
                options.iteration_limit);
    Eigen::MatrixXd points(domain.ambient_dimension(),
                           num_points_per_visibility_round);
    for (int i = 0; i < points.cols(); ++i) {
      do {
        last_polytope_sample =
            domain.UniformSample(generator, last_polytope_sample);
      } while (
          // While the last polytope sample is in collision.
          !checker.CheckConfigCollisionFree(last_polytope_sample) ||
          // While the last polytope sample is in any of the sets.
          std::any_of(sets->begin(), sets->end(),
                      [&last_polytope_sample](const HPolyhedron& set) -> bool {
                        return set.PointInSet(last_polytope_sample);
                      }));
      points.col(i) = last_polytope_sample;
    }

    Meshcat* meshcat = GetMeshcatFromOptions(options.iris_options);
    // Show the samples used in build cliques. Debugging visualization.
    if (meshcat != nullptr && domain.ambient_dimension() <= 3) {
      Eigen::Vector3d point_to_draw = Eigen::Vector3d::Zero();
      for (int pt_to_draw = 0; pt_to_draw < points.cols(); ++pt_to_draw) {
        std::string path = fmt::format("iteration{:02}/sample_{:03}",
                                       num_iterations, pt_to_draw);
        meshcat->SetObject(path, Sphere(0.01),
                           geometry::Rgba(1, 0.1, 0.1, 1.0));
        point_to_draw.head(domain.ambient_dimension()) = points.col(pt_to_draw);
        meshcat->SetTransform(path, RigidTransform<double>(point_to_draw));
      }
    }

    Eigen::SparseMatrix<bool> visibility_graph =
        VisibilityGraph(checker, points, max_collision_checker_parallelism);
    // Reserve more space for the newly built sets. Typically, we won't get
    // this worst case number of new cliques, so we only reserve half of the
    // worst case.
    sets->reserve(sets->size() +
                  ComputeMaxNumberOfCliquesInGreedyCliqueCover(
                      visibility_graph.cols(), minimum_clique_size) /
                      2);

    // Now solve the max clique cover and build new sets.
    int num_new_sets{0};
    // The computed cliques from the max clique solver. These will get pulled
    // off the queue by the set builder workers to build the sets.
    AsyncQueue<VectorX<bool>> computed_cliques;

    if (options.parallelism.num_threads() == 1) {
      ComputeGreedyTruncatedCliqueCover(minimum_clique_size, *max_clique_solver,
                                        &visibility_graph, &computed_cliques);
      std::queue<HPolyhedron> new_set_queue =
          IrisWorker(checker, points, 0, options, domain, &computed_cliques,
                     false /* No need to disable meshcat */);
      while (!new_set_queue.empty()) {
        sets->push_back(std::move(new_set_queue.front()));
        new_set_queue.pop();
        ++num_new_sets;
      }
    } else {
      // Compute truncated clique cover.
      std::future<void> clique_future{
          std::async(std::launch::async, ComputeGreedyTruncatedCliqueCover,
                     minimum_clique_size, std::ref(*max_clique_solver),
                     &visibility_graph, &computed_cliques)};

      // We will use one thread to build cliques. If we are building sets using
      // IrisNp2 or IrisZo, we use only one worker thread to produce sets as
      // these methods use parallelism internally in the collision checker. If
      // we use IrisNp to build sets, we use all the remaining threads of
      // parallelism to build sets. If this number is 0, then this function will
      // end up single threaded.
      const int num_builder_threads =
          std::visit(overloaded{[&options](const IrisOptions&) {
                                  return options.parallelism.num_threads() - 1;
                                },
                                [](const IrisNp2Options&) {
                                  return 1;
                                },
                                [](const IrisZoOptions&) {
                                  return 1;
                                }},
                     options.iris_options);
      std::vector<std::future<std::queue<HPolyhedron>>> build_sets_future;
      build_sets_future.reserve(num_builder_threads);
      // Build convex sets.
      for (int i = 0; i < num_builder_threads; ++i) {
        build_sets_future.emplace_back(
            std::async(std::launch::async, IrisWorker, std::ref(checker),
                       points, i, std::ref(options), domain, &computed_cliques,
                       // NOLINTNEXTLINE
                       true /* Disable meshcat since IRIS runs outside the main thread */));
      }

      clique_future.get();
      for (auto& new_set_queue_future : build_sets_future) {
        std::queue<HPolyhedron> new_set_queue{new_set_queue_future.get()};
        while (!new_set_queue.empty()) {
          sets->push_back(std::move(new_set_queue.front()));
          new_set_queue.pop();
          ++num_new_sets;
        }
      }
    }
    log()->debug(
        "{} new sets added in IrisFromCliqueCover at iteration {}. Total sets "
        "= {}",
        num_new_sets, num_iterations, ssize(*sets));

    if (num_new_sets == 0) {
      num_points_per_visibility_round *= 2;
    }
    ++num_iterations;
  }
}
}  // namespace planning
}  // namespace drake
