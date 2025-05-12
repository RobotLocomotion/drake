#include "drake/planning/iris/iris_np2.h"

#include "drake/common/text_logging.h"
#include "drake/geometry/optimization/affine_ball.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/iris_internal.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::Role;
using geometry::SceneGraphInspector;
using geometry::optimization::ConvexSet;
using geometry::optimization::internal::ClosestCollisionProgram;
using geometry::optimization::internal::SamePointConstraint;
using math::RigidTransform;
using multibody::MultibodyPlant;
using systems::Context;

namespace {
// Copied from iris.cc

using geometry::Box;
using geometry::Capsule;
using geometry::Convex;
using geometry::Cylinder;
using geometry::Ellipsoid;
using geometry::HalfSpace;
using geometry::Mesh;
using geometry::Sphere;

using geometry::FrameId;
using geometry::GeometryId;
using geometry::QueryObject;
using geometry::ShapeReifier;

using geometry::optimization::CartesianProduct;
using geometry::optimization::ConvexSet;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;
using geometry::optimization::MinkowskiSum;
using geometry::optimization::VPolytope;

// Constructs a ConvexSet for each supported Shape and adds it to the set.
class IrisConvexSetMaker final : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IrisConvexSetMaker);

  IrisConvexSetMaker(const QueryObject<double>& query,
                     std::optional<FrameId> reference_frame)
      : query_{query}, reference_frame_{reference_frame} {};

  void set_reference_frame(const FrameId& reference_frame) {
    DRAKE_DEMAND(reference_frame.is_valid());
    *reference_frame_ = reference_frame;
  }

  void set_geometry_id(const GeometryId& geom_id) { geom_id_ = geom_id; }

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    // Note: We choose HPolyhedron over VPolytope here, but the IRIS paper
    // discusses a significant performance improvement using a "least-distance
    // programming" instance from CVXGEN that exploited the VPolytope
    // representation.  So we may wish to revisit this.
    set = std::make_unique<HPolyhedron>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Capsule&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<MinkowskiSum>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Cylinder&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set =
        std::make_unique<CartesianProduct>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Ellipsoid&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<Hyperellipsoid>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const HalfSpace&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<HPolyhedron>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Sphere&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<Hyperellipsoid>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Convex&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<VPolytope>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Mesh&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<VPolytope>(query_, geom_id_, reference_frame_);
  }

 private:
  const QueryObject<double>& query_{};
  std::optional<FrameId> reference_frame_{};
  GeometryId geom_id_{};
};

struct GeometryPairWithDistance {
  GeometryId geomA;
  GeometryId geomB;
  double distance;

  GeometryPairWithDistance(GeometryId gA, GeometryId gB, double dist)
      : geomA(gA), geomB(gB), distance(dist) {}

  bool operator<(const GeometryPairWithDistance& other) const {
    return distance < other.distance;
  }
};

int FindCollisionPairIndex(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const std::vector<GeometryPairWithDistance>& sorted_pairs) {
  // Call ComputeSignedDistancePairClosestPoints for each pair of collision
  // geometries until finding a pair that is in collision and returning the
  // corresponding index
  int pair_in_collision = -1;
  int i_pair = 0;
  for (const auto& pair : sorted_pairs) {
    auto query_object = plant.get_geometry_query_input_port()
                            .template Eval<QueryObject<double>>(context);
    const double distance =
        query_object
            .ComputeSignedDistancePairClosestPoints(pair.geomA, pair.geomB)
            .distance;
    if (distance < 0.0) {
      pair_in_collision = i_pair;
      break;
    }
    ++i_pair;
  }

  return pair_in_collision;
}

// Add the tangent to the (scaled) ellipsoid at @p point as a
// constraint.
void AddTangentToPolytope(
    const Hyperellipsoid& E, const Eigen::Ref<const Eigen::VectorXd>& point,
    double configuration_space_margin,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    Eigen::VectorXd* b, int* num_constraints) {
  while (*num_constraints >= A->rows()) {
    // Increase pre-allocated polytope size.
    A->conservativeResize(A->rows() * 2, A->cols());
    b->conservativeResize(b->rows() * 2);
  }

  A->row(*num_constraints) =
      (E.A().transpose() * E.A() * (point - E.center())).normalized();
  (*b)[*num_constraints] =
      A->row(*num_constraints) * point - configuration_space_margin;
  if (A->row(*num_constraints) * E.center() > (*b)[*num_constraints]) {
    throw std::logic_error(
        "The current center of the IRIS region is within "
        "options.sampled_iris_options.configuration_space_margin of being "
        "infeasible.  Check your sample point and/or any additional "
        "constraints you've passed in via the options. The configuration space "
        "surrounding the sample point must have an interior.");
  }
  *num_constraints += 1;
}

}  // namespace
HPolyhedron IrisNp2(const SceneGraphCollisionChecker& checker,
                    const Hyperellipsoid& starting_ellipsoid,
                    const HPolyhedron& domain, const IrisNp2Options& options) {
  auto start = std::chrono::high_resolution_clock::now();

  // Check for features which are currently unsupported.
  if (options.sampled_iris_options.containment_points != std::nullopt) {
    // TODO(cohnt): Support enforcing additional containment points.
    throw std::runtime_error(
        "IrisNp2 does not yet support enforcing additional containment "
        "points.");
  }
  if (options.sampled_iris_options.prog_with_additional_constraints !=
      nullptr) {
    // TODO(cohnt): Support enforcing additional constraints.
    throw std::runtime_error(
        "IrisNp2 does not yet support specifying additional constriants.");
  }

  if (!(options.parameterization.get_parameterization()(
            starting_ellipsoid.center()) == starting_ellipsoid.center())) {
    // TODO(cohnt): Support growing regions along a parameterized subspace.
    throw std::runtime_error(
        "IrisNp2 does not yet support growing regions on a parameterized "
        "subspace.");
  }
  if (!checker.GetAllAddedCollisionShapes().empty()) {
    // TODO(cohnt): Support handling additional collision shapes.
    throw std::runtime_error(
        "IrisNp2 does not yet support collision checkers with added collision "
        "shapes.");
  }
  if (checker.GetPaddingMatrix().cwiseAbs().maxCoeff() != 0.0) {
    // A nonzero value implies nonzero padding. std::nullopt implies varying
    // padding, so at least one such padding is nonzero.
    // TODO(cohnt): Support nonzero padding.
    throw std::runtime_error(
        "IrisNp2 does not yet support collision checkers with nonzero "
        "padding.");
  }

  const auto& plant = checker.plant();
  const auto& context = checker.UpdatePositions(starting_ellipsoid.center());

  // Check the inputs.
  plant.ValidateContext(context);
  const int nq = plant.num_positions();
  const Eigen::VectorXd seed = starting_ellipsoid.center();

  Eigen::VectorXd lower_limits = plant.GetPositionLowerLimits();
  Eigen::VectorXd upper_limits = plant.GetPositionUpperLimits();

  HPolyhedron P(domain);
  DRAKE_THROW_UNLESS(P.IsBounded());

  Hyperellipsoid E = starting_ellipsoid;

  // Make all of the convex sets and supporting quantities.
  auto query_object =
      plant.get_geometry_query_input_port().Eval<QueryObject<double>>(context);
  const SceneGraphInspector<double>& inspector = query_object.inspector();
  IrisConvexSetMaker maker(query_object, inspector.world_frame_id());
  std::unordered_map<GeometryId, copyable_unique_ptr<ConvexSet>> sets{};
  std::unordered_map<GeometryId, const multibody::Frame<double>*> frames{};
  const std::vector<GeometryId> geom_ids =
      inspector.GetAllGeometryIds(Role::kProximity);
  copyable_unique_ptr<ConvexSet> temp_set;
  for (GeometryId geom_id : geom_ids) {
    // Make all sets in the local geometry frame.
    FrameId frame_id = inspector.GetFrameId(geom_id);
    maker.set_reference_frame(frame_id);
    maker.set_geometry_id(geom_id);
    inspector.GetShape(geom_id).Reify(&maker, &temp_set);
    sets.emplace(geom_id, std::move(temp_set));
    frames.emplace(geom_id, &plant.GetBodyFromFrameId(frame_id)->body_frame());
  }

  auto pairs = inspector.GetCollisionCandidates();
  const int n = static_cast<int>(pairs.size());
  auto same_point_constraint =
      std::make_shared<SamePointConstraint>(&plant, context);
  std::map<std::pair<GeometryId, GeometryId>, std::vector<VectorXd>>
      counter_examples;

  // As a surrogate for the true objective, the pairs are sorted by the
  // distance between each collision pair from the seed point configuration.
  // This could improve computation times and produce regions with fewer
  // faces.
  std::vector<GeometryPairWithDistance> sorted_pairs;
  for (const auto& [geomA, geomB] : pairs) {
    const double distance =
        query_object.ComputeSignedDistancePairClosestPoints(geomA, geomB)
            .distance;
    if (distance < 0.0) {
      throw std::runtime_error(fmt::format(
          "The center of starting_ellipsoid is in collision; geometry {} is in "
          "collision with geometry {}",
          inspector.GetName(geomA), inspector.GetName(geomB)));
    }
    sorted_pairs.emplace_back(geomA, geomB, distance);
  }
  std::sort(sorted_pairs.begin(), sorted_pairs.end());

  // On each iteration, we will build the collision-free polytope represented
  // as {x | A * x <= b}.  Here we pre-allocate matrices with a generous
  // maximum size.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      P.A().rows() + 2 * n, nq);
  VectorXd b(P.A().rows() + 2 * n);
  A.topRows(P.A().rows()) = P.A();
  b.head(P.A().rows()) = P.b();
  int num_initial_constraints = P.A().rows();

  DRAKE_THROW_UNLESS(P.PointInSet(seed, 1e-12));

  double best_volume = E.Volume();
  int iteration = 0;
  VectorXd closest(nq);
  RandomGenerator generator(options.sampled_iris_options.random_seed);

  // TODO(cohnt): Allow the user to specify a solver.
  auto solver = solvers::MakeFirstAvailableSolver(
      {solvers::SnoptSolver::id(), solvers::IpoptSolver::id()});

  // For debugging visualization.
  Vector3d point_to_draw = Vector3d::Zero();
  int num_points_drawn = 0;
  bool do_debugging_visualization =
      options.sampled_iris_options.meshcat && nq <= 3;

  const std::string seed_point_msg =
      "IrisNp2: terminating iterations because the seed point is no longer in "
      "the region.";

  // Set up constants for statistical tests.
  double outer_delta_min =
      internal::calc_delta_min(options.sampled_iris_options.delta,
                               options.sampled_iris_options.max_iterations);

  double delta_min = internal::calc_delta_min(
      outer_delta_min,
      options.sampled_iris_options.max_iterations_separating_planes);

  int N_max = internal::unadaptive_test_samples(
      options.sampled_iris_options.epsilon, delta_min,
      options.sampled_iris_options.tau);

  if (options.sampled_iris_options.verbose) {
    log()->info(
        "IrisNp2 finding region that is {} collision free with {} certainty ",
        options.sampled_iris_options.epsilon,
        1 - options.sampled_iris_options.delta);
    log()->info("IrisNp2 worst case test requires {} samples.", N_max);
  }

  // TODO(cohnt): Do an argsort so we don't have to have two separate copies
  std::vector<Eigen::VectorXd> particles;
  std::vector<Eigen::VectorXd> particles_in_collision;
  particles.reserve(N_max);
  particles_in_collision.reserve(N_max);

  while (true) {
    log()->info("IrisNp2 iteration {}", iteration);
    int num_constraints = num_initial_constraints;
    HPolyhedron P_candidate = HPolyhedron(A.topRows(num_initial_constraints),
                                          b.head(num_initial_constraints));
    DRAKE_ASSERT(best_volume > 0);
    // Find separating hyperplanes

    // Separating Planes Step
    int num_iterations_separating_planes = 0;

    double outer_delta = options.sampled_iris_options.delta * 6 /
                         (M_PI * M_PI * (iteration + 1) * (iteration + 1));

    // No need for decaying outer delta if we are guaranteed to terminate after
    // one step. In this case we can be less conservative and set it to our
    // total accepted error probability.
    if (options.sampled_iris_options.max_iterations == 1) {
      outer_delta = options.sampled_iris_options.delta;
    }

    while (num_iterations_separating_planes <
           options.sampled_iris_options.max_iterations_separating_planes) {
      // log()->info("starting inner loop.");
      int k_squared = num_iterations_separating_planes + 1;
      k_squared *= k_squared;
      double delta_k = outer_delta * 6 / (M_PI * M_PI * k_squared);
      int N_k = internal::unadaptive_test_samples(
          options.sampled_iris_options.epsilon, delta_k,
          options.sampled_iris_options.tau);

      particles.resize(N_k);
      particles.at(0) = P_candidate.UniformSample(
          &generator, E.center(), options.sampled_iris_options.mixing_steps);
      // populate particles by uniform sampling
      for (int i = 1; i < N_k; ++i) {
        particles.at(i) = P_candidate.UniformSample(
            &generator, particles.at(i - 1),
            options.sampled_iris_options.mixing_steps);
      }
      // Find all particles in collision
      std::vector<uint8_t> particle_col_free =
          checker.CheckConfigsCollisionFree(
              particles, options.sampled_iris_options.parallelism);
      int number_particles_in_collision = 0;

      particles_in_collision.clear();
      for (size_t i = 0; i < particle_col_free.size(); ++i) {
        if (particle_col_free.at(i) == 0) {
          particles_in_collision.push_back(particles.at(i));
          ++number_particles_in_collision;
        }
      }

      // Sort collision order
      auto my_comparator = [](const VectorXd& t1, const VectorXd& t2,
                              const Hyperellipsoid& E_comparator) {
        return (t1 - E_comparator.center()).squaredNorm() <
               (t2 - E_comparator.center())
                   .squaredNorm();  // or use a custom compare function
      };
      std::sort(
          std::begin(particles_in_collision),
          std::begin(particles_in_collision) + number_particles_in_collision,
          std::bind(my_comparator, std::placeholders::_1, std::placeholders::_2,
                    E));

      if (options.sampled_iris_options.verbose) {
        log()->info("IrisNp2 N_k {}, N_col {}, thresh {}", N_k,
                    number_particles_in_collision,
                    (1 - options.sampled_iris_options.tau) *
                        options.sampled_iris_options.epsilon * N_k);
      }

      // break if threshold is passed
      if (number_particles_in_collision <=
          (1 - options.sampled_iris_options.tau) *
              options.sampled_iris_options.epsilon * N_k) {
        break;
      }
      // warn user if test fails on last iteration
      if (num_iterations_separating_planes ==
          options.sampled_iris_options.max_iterations_separating_planes - 1) {
        log()->warn(
            "IrisNp2 WARNING, separating planes hit max iterations without "
            "passing the bernoulli test, this voids the probabilistic "
            "guarantees!");
      }

      int num_hyperplanes_added = 0;

      for (const auto& particle : particles_in_collision) {
        if (num_hyperplanes_added >
            options.sampled_iris_options.max_separating_planes_per_iteration) {
          break;
        }
        if (!P_candidate.PointInSet(particle)) {
          log()->info("Not in the polytope!");
          continue;
        }

        std::pair<Eigen::VectorXd, int> closest_collision_info;

        closest_collision_info = std::make_pair(
            particle,
            FindCollisionPairIndex(plant, checker.UpdatePositions(particle),
                                   sorted_pairs));

        if (closest_collision_info.second >= 0) {
          // pair is actually in collision
          auto pair_iterator =
              std::next(sorted_pairs.begin(), closest_collision_info.second);
          const auto collision_pair = *pair_iterator;
          std::pair<GeometryId, GeometryId> geom_pair(collision_pair.geomA,
                                                      collision_pair.geomB);

          ClosestCollisionProgram prog(
              same_point_constraint, *frames.at(collision_pair.geomA),
              *frames.at(collision_pair.geomB), *sets.at(collision_pair.geomA),
              *sets.at(collision_pair.geomB), E, A.topRows(num_constraints),
              b.head(num_constraints));

          if (do_debugging_visualization) {
            ++num_points_drawn;
            point_to_draw.head(nq) = particle;
            std::string path = fmt::format("iteration{:02}/{:03}/particle",
                                           iteration, num_points_drawn);
            options.sampled_iris_options.meshcat->SetObject(
                path, Sphere(0.01), geometry::Rgba(0.1, 0.1, 0.1, 1.0));
            options.sampled_iris_options.meshcat->SetTransform(
                path, RigidTransform<double>(point_to_draw));
          }

          // TODO(cohnt): Allow the user to specify the solver options here.
          if (prog.Solve(*solver, particle, {}, &closest)) {
            if (do_debugging_visualization) {
              point_to_draw.head(nq) = closest;
              std::string path = fmt::format("iteration{:02}/{:03}/found",
                                             iteration, num_points_drawn);
              options.sampled_iris_options.meshcat->SetObject(
                  path, Sphere(0.01), geometry::Rgba(0.8, 0.1, 0.8, 1.0));
              options.sampled_iris_options.meshcat->SetTransform(
                  path, RigidTransform<double>(point_to_draw));
            }
            AddTangentToPolytope(
                E, closest,
                options.sampled_iris_options.configuration_space_margin, &A, &b,
                &num_constraints);
            P_candidate = HPolyhedron(A.topRows(num_constraints),
                                      b.head(num_constraints));
            if (options.sampled_iris_options
                    .require_sample_point_is_contained) {
              const bool seed_point_requirement =
                  A.row(num_constraints - 1) * seed <= b(num_constraints - 1);
              if (!seed_point_requirement) {
                log()->info(seed_point_msg);
                return P;
              }
            }
            prog.UpdatePolytope(A.topRows(num_constraints),
                                b.head(num_constraints));
          } else {
            if (do_debugging_visualization) {
              point_to_draw.head(nq) = closest;
              std::string path = fmt::format("iteration{:02}/{:03}/closest",
                                             iteration, num_points_drawn);
              options.sampled_iris_options.meshcat->SetObject(
                  path, Sphere(0.01), geometry::Rgba(0.1, 0.8, 0.8, 1.0));
              options.sampled_iris_options.meshcat->SetTransform(
                  path, RigidTransform<double>(point_to_draw));
            }
          }
        }
      }

      ++num_iterations_separating_planes;
    }

    P = HPolyhedron(A.topRows(num_constraints), b.head(num_constraints));

    iteration++;
    if (iteration >= options.sampled_iris_options.max_iterations) {
      log()->info(
          "IrisNp2: Terminating because the iteration limit "
          "{} has been reached.",
          options.sampled_iris_options.max_iterations);
      break;
    }

    E = P.MaximumVolumeInscribedEllipsoid();
    const double volume = E.Volume();
    const double delta_volume = volume - best_volume;
    if (delta_volume <= options.sampled_iris_options.termination_threshold) {
      log()->info(
          "IrisNp2: Terminating because the hyperellipsoid "
          "volume change {} is below the threshold {}.",
          delta_volume, options.sampled_iris_options.termination_threshold);
      break;
    } else if (delta_volume / best_volume <=
               options.sampled_iris_options.relative_termination_threshold) {
      log()->info(
          "IrisNp2: Terminating because the hyperellipsoid "
          "relative volume change {} is below the threshold {}.",
          delta_volume / best_volume,
          options.sampled_iris_options.relative_termination_threshold);
      break;
    }
    best_volume = volume;
  }
  auto stop = std::chrono::high_resolution_clock::now();
  if (options.sampled_iris_options.verbose) {
    log()->info(
        "IrisNp2 execution time : {} ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start)
            .count());
  }

  return P;
}

}  // namespace planning
}  // namespace drake
