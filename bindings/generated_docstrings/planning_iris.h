#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/planning/iris/iris_common.h"
// #include "drake/planning/iris/iris_from_clique_cover.h"
// #include "drake/planning/iris/iris_np2.h"
// #include "drake/planning/iris/iris_zo.h"

// Symbol: pydrake_doc_planning_iris
constexpr struct /* pydrake_doc_planning_iris */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::planning
    struct /* planning */ {
      // Symbol: drake::planning::CommonSampledIrisOptions
      struct /* CommonSampledIrisOptions */ {
        // Source: drake/planning/iris/iris_common.h
        const char* doc =
R"""(Various options which are common to the sampling-based algorithms
IrisNp2 and IrisZo for generating collision free polytopes in
configuration space.)""";
        // Symbol: drake::planning::CommonSampledIrisOptions::CommonSampledIrisOptions
        struct /* ctor */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::planning::CommonSampledIrisOptions::Serialize
        struct /* Serialize */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background. Note: This only serializes options that
are YAML built-in types.)""";
        } Serialize;
        // Symbol: drake::planning::CommonSampledIrisOptions::configuration_space_margin
        struct /* configuration_space_margin */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(We retreat by this margin from each C-space obstacle in order to avoid
the possibility of requiring an infinite number of faces to
approximate a curved boundary.)""";
        } configuration_space_margin;
        // Symbol: drake::planning::CommonSampledIrisOptions::containment_points
        struct /* containment_points */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Points that are guaranteed to be contained in the final region
provided their convex hull is collision free. Note that if the
containment points are closer than configuration_margin to an obstacle
we will relax the margin in favor of including the containment points.
The matrix ``containment_points`` is expected to be of the shape
dimension times number of points. IrisZo throws if the center of the
starting ellipsoid is not contained in the convex hull of these
containment points.)""";
        } containment_points;
        // Symbol: drake::planning::CommonSampledIrisOptions::delta
        struct /* delta */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Upper bound on the probability the returned region has a
fraction-in-collision greater than ``epsilon``.)""";
        } delta;
        // Symbol: drake::planning::CommonSampledIrisOptions::epsilon
        struct /* epsilon */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Admissible fraction of the region volume allowed to be in collision.)""";
        } epsilon;
        // Symbol: drake::planning::CommonSampledIrisOptions::max_iterations
        struct /* max_iterations */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Maximum number of alternations between the ellipsoid and the
separating planes step (a.k.a. outer iterations).)""";
        } max_iterations;
        // Symbol: drake::planning::CommonSampledIrisOptions::max_iterations_separating_planes
        struct /* max_iterations_separating_planes */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Maximum number of rounds of adding faces to the polytope per outer
iteration.)""";
        } max_iterations_separating_planes;
        // Symbol: drake::planning::CommonSampledIrisOptions::max_separating_planes_per_iteration
        struct /* max_separating_planes_per_iteration */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Maximum number of faces to add per inner iteration. Setting the value
to -1 means there is no limit to the number of faces that can be
added.)""";
        } max_separating_planes_per_iteration;
        // Symbol: drake::planning::CommonSampledIrisOptions::meshcat
        struct /* meshcat */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Passing a meshcat instance may enable debugging visualizations when
the configuration space is <= 3 dimensional.)""";
        } meshcat;
        // Symbol: drake::planning::CommonSampledIrisOptions::mixing_steps
        struct /* mixing_steps */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Number of mixing steps used for hit-and-run sampling.)""";
        } mixing_steps;
        // Symbol: drake::planning::CommonSampledIrisOptions::num_particles
        struct /* num_particles */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Minimum number of particles drawn per inner iteration. Some or all of
these particles, depending on the other algorithm settings, will be
used to find the closest collisions.)""";
        } num_particles;
        // Symbol: drake::planning::CommonSampledIrisOptions::parallelism
        struct /* parallelism */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Number of threads to use when updating the particles. If the user
requests more threads than the CollisionChecker supports, that number
of threads will be used instead. However, see also
``parameterization_is_threadsafe``.)""";
        } parallelism;
        // Symbol: drake::planning::CommonSampledIrisOptions::prog_with_additional_constraints
        struct /* prog_with_additional_constraints */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(By default, IRIS-ZO only considers collision avoidance constraints.
This option can be used to pass additional constraints that should be
satisfied by the output region. We accept these in the form of a
MathematicalProgram:

find q subject to g(q) ≤ 0.

The decision_variables() for the program are taken to define ``q``.
IRIS-ZO will silently ignore any costs in
``prog_with_additional_constraints``. If any constraints are not
threadsafe, then ``parallelism`` will be overridden, and only one
thread will be used.

Note:
    If the user has specified a parameterization, then these
    constraints are imposed on the points in the parameterized space
    Q, not the configuration space C.

Note:
    Internally, these constraints are checked after collisions
    checking is performed.)""";
        } prog_with_additional_constraints;
        // Symbol: drake::planning::CommonSampledIrisOptions::random_seed
        struct /* random_seed */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(This option sets the random seed for random sampling throughout the
algorithm.)""";
        } random_seed;
        // Symbol: drake::planning::CommonSampledIrisOptions::relative_termination_threshold
        struct /* relative_termination_threshold */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(IRIS will terminate if the change in the *volume* of the
hyperellipsoid between iterations is less that this percent of the
previous best volume. This termination condition can be disabled by
setting to a negative value.)""";
        } relative_termination_threshold;
        // Symbol: drake::planning::CommonSampledIrisOptions::relax_margin
        struct /* relax_margin */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Suppose stepping back by configuration_space_margin would cut off the
seed point. If ``relax_margin`` is false, we throw an error, and if
``relax_margin`` is true, we repeatedly divide
configuration_space_margin by two (for that hyperplane only) until the
seed point is not cut off. Ignored if the user has provided
``containment_points``.)""";
        } relax_margin;
        // Symbol: drake::planning::CommonSampledIrisOptions::remove_all_collisions_possible
        struct /* remove_all_collisions_possible */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(A region may satisfy the user-requested fraction in-collision, but
still have some of the samples drawn be in-collision. If this flag is
true, those samples will also be used to produce hyperplanes. This
produces slightly smaller regions with more faces, but the region will
beat the user-requested fraction in-collision by a larger margin.)""";
        } remove_all_collisions_possible;
        // Symbol: drake::planning::CommonSampledIrisOptions::require_sample_point_is_contained
        struct /* require_sample_point_is_contained */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(The initial polytope is guaranteed to contain the point if that point
is collision-free.)""";
        } require_sample_point_is_contained;
        // Symbol: drake::planning::CommonSampledIrisOptions::sample_particles_in_parallel
        struct /* sample_particles_in_parallel */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(If true, the hit-and-run procedure is run in parallel to quickly draw
all the samples necessary. When the statistical test requires many
samples (e.g. due to constructing regions with a very low fraction in
collision with very high probability), the process of drawing the
samples may become a major time cost. Drawing the samples in parallel
can lead to a major speedup, at the cost of a sampling from a
distribution that's slightly further from a uniform distribution (due
to the lower cumulative mixing time).)""";
        } sample_particles_in_parallel;
        // Symbol: drake::planning::CommonSampledIrisOptions::tau
        struct /* tau */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Decision threshold for the unadaptive test. Choosing a small value
increases both the cost and the power of the statistical test.
Increasing the value of ``tau`` makes running an individual test
cheaper but decreases its power to accept a polytope. We find choosing
a value of 0.5 a good trade-off.)""";
        } tau;
        // Symbol: drake::planning::CommonSampledIrisOptions::termination_threshold
        struct /* termination_threshold */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(IRIS will terminate if the change in the *volume* of the
hyperellipsoid between iterations is less that this threshold. This
termination condition can be disabled by setting to a negative value.)""";
        } termination_threshold;
        // Symbol: drake::planning::CommonSampledIrisOptions::verbose
        struct /* verbose */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Enables print statements indicating the progress of IrisZo.)""";
        } verbose;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("configuration_space_margin", configuration_space_margin.doc),
            std::make_pair("containment_points", containment_points.doc),
            std::make_pair("delta", delta.doc),
            std::make_pair("epsilon", epsilon.doc),
            std::make_pair("max_iterations", max_iterations.doc),
            std::make_pair("max_iterations_separating_planes", max_iterations_separating_planes.doc),
            std::make_pair("max_separating_planes_per_iteration", max_separating_planes_per_iteration.doc),
            std::make_pair("meshcat", meshcat.doc),
            std::make_pair("mixing_steps", mixing_steps.doc),
            std::make_pair("num_particles", num_particles.doc),
            std::make_pair("parallelism", parallelism.doc),
            std::make_pair("prog_with_additional_constraints", prog_with_additional_constraints.doc),
            std::make_pair("random_seed", random_seed.doc),
            std::make_pair("relative_termination_threshold", relative_termination_threshold.doc),
            std::make_pair("relax_margin", relax_margin.doc),
            std::make_pair("remove_all_collisions_possible", remove_all_collisions_possible.doc),
            std::make_pair("require_sample_point_is_contained", require_sample_point_is_contained.doc),
            std::make_pair("sample_particles_in_parallel", sample_particles_in_parallel.doc),
            std::make_pair("tau", tau.doc),
            std::make_pair("termination_threshold", termination_threshold.doc),
            std::make_pair("verbose", verbose.doc),
          };
        }
      } CommonSampledIrisOptions;
      // Symbol: drake::planning::IrisFromCliqueCoverOptions
      struct /* IrisFromCliqueCoverOptions */ {
        // Source: drake/planning/iris/iris_from_clique_cover.h
        const char* doc = R"""()""";
        // Symbol: drake::planning::IrisFromCliqueCoverOptions::coverage_termination_threshold
        struct /* coverage_termination_threshold */ {
          // Source: drake/planning/iris/iris_from_clique_cover.h
          const char* doc =
R"""(The fraction of the domain that must be covered before we terminate
the algorithm.)""";
        } coverage_termination_threshold;
        // Symbol: drake::planning::IrisFromCliqueCoverOptions::iris_options
        struct /* iris_options */ {
          // Source: drake/planning/iris/iris_from_clique_cover.h
          const char* doc =
R"""(The options used on internal calls to Iris. The type of this option
determines which variant of Iris is called. Currently, it is
recommended to only run Iris for one iteration when building from a
clique so as to avoid discarding the information gained from the
clique.

Note that ``IrisOptions`` can optionally include a meshcat instance to
provide debugging visualization. If this is provided
``IrisFromCliqueCover`` will provide debug visualization in meshcat
showing where in configuration space it is drawing from. However, if
the parallelism option is set to allow more than 1 thread, then the
debug visualizations of internal Iris calls will be disabled. This is
due to a limitation of drawing to meshcat from outside the main
thread.

Note:
    some of these variants specify a parallelism parameter. In
    IrisInConfigurationSpaceFromCliqueCover, the
    iris_options.parallelism is ignored and the value of parallelism
    specified by ``this.parallelism`` will be used instead.)""";
        } iris_options;
        // Symbol: drake::planning::IrisFromCliqueCoverOptions::iteration_limit
        struct /* iteration_limit */ {
          // Source: drake/planning/iris/iris_from_clique_cover.h
          const char* doc =
R"""(The maximum number of iterations of the algorithm.)""";
        } iteration_limit;
        // Symbol: drake::planning::IrisFromCliqueCoverOptions::minimum_clique_size
        struct /* minimum_clique_size */ {
          // Source: drake/planning/iris/iris_from_clique_cover.h
          const char* doc =
R"""(The minimum size of the cliques used to construct a region. If this is
set lower than the ambient dimension of the space we are trying to
cover, then this option will be overridden to be at least 1 + the
ambient dimension.)""";
        } minimum_clique_size;
        // Symbol: drake::planning::IrisFromCliqueCoverOptions::num_points_per_coverage_check
        struct /* num_points_per_coverage_check */ {
          // Source: drake/planning/iris/iris_from_clique_cover.h
          const char* doc =
R"""(The number of points to sample when testing coverage.)""";
        } num_points_per_coverage_check;
        // Symbol: drake::planning::IrisFromCliqueCoverOptions::num_points_per_visibility_round
        struct /* num_points_per_visibility_round */ {
          // Source: drake/planning/iris/iris_from_clique_cover.h
          const char* doc =
R"""(Number of points to sample when building visibilty cliques. If this
option is less than twice the minimum clique size, it will be
overridden to be at least twice the minimum clique size. If the
algorithm ever fails to find a single clique in a visibility round,
then the number of points in a visibility round will be doubled.)""";
        } num_points_per_visibility_round;
        // Symbol: drake::planning::IrisFromCliqueCoverOptions::parallelism
        struct /* parallelism */ {
          // Source: drake/planning/iris/iris_from_clique_cover.h
          const char* doc =
R"""(The amount of parallelism to use. This algorithm makes heavy use of
parallelism at many points and thus it is highly recommended to set
this to the maximum tolerable parallelism.)""";
        } parallelism;
        // Symbol: drake::planning::IrisFromCliqueCoverOptions::point_in_set_tol
        struct /* point_in_set_tol */ {
          // Source: drake/planning/iris/iris_from_clique_cover.h
          const char* doc =
R"""(The tolerance used for checking whether a point is contained inside an
HPolyhedron. See @ConvexSet∷PointInSet.)""";
        } point_in_set_tol;
        // Symbol: drake::planning::IrisFromCliqueCoverOptions::rank_tol_for_minimum_volume_circumscribed_ellipsoid
        struct /* rank_tol_for_minimum_volume_circumscribed_ellipsoid */ {
          // Source: drake/planning/iris/iris_from_clique_cover.h
          const char* doc =
R"""(The rank tolerance used for computing the
MinimumVolumeCircumscribedEllipsoid of a clique. See
@MinimumVolumeCircumscribedEllipsoid.)""";
        } rank_tol_for_minimum_volume_circumscribed_ellipsoid;
      } IrisFromCliqueCoverOptions;
      // Symbol: drake::planning::IrisInConfigurationSpaceFromCliqueCover
      struct /* IrisInConfigurationSpaceFromCliqueCover */ {
        // Source: drake/planning/iris/iris_from_clique_cover.h
        const char* doc =
R"""(Cover the configuration space in Iris regions using the Visibility
Clique Cover Algorithm as described in

P. Werner, A. Amice, T. Marcucci, D. Rus, R. Tedrake "Approximating
Robot Configuration Spaces with few Convex Sets using Clique Covers of
Visibility Graphs" In 2024 IEEE Internation Conference on Robotics and
Automation. https://arxiv.org/abs/2310.02875

Parameter ``checker``:
    The collision checker containing the plant and its associated
    scene_graph.

Parameter ``generator``:
    There are points in the algorithm requiring randomness. The
    generator controls this source of randomness.

Parameter ``sets``:
    [in/out] initial sets covering the space (potentially empty). The
    cover is written into this vector.

Parameter ``max_clique_solver``:
    The min clique cover problem is approximatley solved by repeatedly
    solving max clique on the uncovered graph and adding this largest
    clique to the cover. The max clique problem is solved by this
    solver. If parallelism is set to allow more than 1 thread, then
    the solver **must** be implemented in C++.

If nullptr is passed as the ``max_clique_solver``, then max clique
will be solved using an instance of MaxCliqueSolverViaGreedy, which is
a fast heuristic. If higher quality cliques are desired, consider
changing the solver to an instance of MaxCliqueSolverViaMip.
Currently, the padding in the collision checker is not forwarded to
the algorithm, and therefore the final regions do not necessarily
respect this padding. Effectively, this means that the regions are
generated as if the padding is set to 0. This behavior may be adjusted
in the future at the resolution of #18830.

Note:
    that MaxCliqueSolverViaMip requires the availability of a
    Mixed-Integer Linear Programming solver (e.g. Gurobi and/or
    Mosek). We recommend enabling those solvers if possible because
    they produce higher quality cliques
    (https://drake.mit.edu/bazel.html#proprietary_solvers). The method
    will throw if ``max_clique_solver`` cannot solve the max clique
    problem.

Note:
    If IrisNp2Options is used, then the collision checker must be a
    SceneGraphCollisionChecker.

Raises:
    RuntimeError Parameterizations are not currently supported for
    ``IrisZo`` and ``IrisNp2`` when running ``IrisFromCliqueCover``.
    This method will throw if options.iris_options is of type
    ``IrisZoOptions`` or ``IrisNp2Options`` and specifies a
    parametrization function. See the documentation of
    ``IrisZoOptions`` and ``IrisNp2Options`` for more information
    about subspace parametrization.

Raises:
    RuntimeError If the
    options.iris_options.prog_with_additional_constraints is not
    nullptr i.e. if a prog with additional constraints is provided.)""";
      } IrisInConfigurationSpaceFromCliqueCover;
      // Symbol: drake::planning::IrisNp2
      struct /* IrisNp2 */ {
        // Source: drake/planning/iris/iris_np2.h
        const char* doc =
R"""(The IRIS-NP2 (Iterative Regional Inflation by Semidefinite and
Nonlinear Programming 2) algorithm, as described in

[Werner et al., 2024] P. Werner, T. Cohn\*, R. H. Jiang\*, T. Seyde,
M. Simchowitz, R. Tedrake, and D. Rus, "Faster Algorithms for Growing
Collision-Free Convex Polytopes in Robot Configuration Space," &nbsp;*
Denotes equal contribution.

https://groups.csail.mit.edu/robotics-center/public_papers/Werner24.pdf

This algorithm constructs probabilistically collision-free polytopes
in robot configuration space using a scene graph collision checker.
The sets are constructed by identifying collisions with sampling and
nonlinear programming. The produced polytope P is probabilistically
collision-free in the sense that one gets to control the probability δ
that the fraction of the volume-in-collision is larger than ε

Pr[λ(P\Cfree)/λ(P) > ε] ≤ δ.

Parameter ``starting_ellipsoid``:
    provides the initial ellipsoid around which to grow the region.
    This is typically a small ball around a collision-free
    configuration (e.g. Hyperellipsoid∷MakeHyperSphere(radius,
    seed_point)). The center of this ellipsoid is required to be
    collision-free.

Parameter ``domain``:
    describes the total region of interest; computed IRIS regions will
    be inside this domain. It must be bounded, and is typically a
    simple bounding box representing joint limits (e.g. from
    HPolyhedron∷MakeBox).

Parameter ``options``:
    contains algorithm parameters such as the desired collision-free
    fraction, confidence level, and various algorithmic settings.

The ``starting_ellipsoid`` and ``domain`` must describe elements in
the same ambient dimension as the configuration space of the robot,
unless a parameterization is specified (in which case, they must match
``options.parameterization_dimension``).

Returns:
    A HPolyhedron representing the computed collision-free region in
    configuration space. $Warning:

This feature is considered to be **experimental** and may change or be
removed at any time, without any deprecation notice ahead of time.

Raises:
    if the center of ``starting_ellipsoid`` is in collision, or
    violates any of the user-specified constraints in
    ``options.prog_with_additional_constraints``.

Note:
    This can be a long running function that needs to solve many QPs.
    If you have a solver which requires a license, consider acquiring
    the license before solving this function. See AcquireLicense for
    more details.

IrisNp2 is still in development, so certain features of
SceneGraphCollisionChecker and parts of [Werner et al., 2024] are not
yet supported.

Raises:
    if any collision pairs in ``checker`` have negative padding.

Raises:
    if any collision geometries have been been added in ``checker``.)""";
      } IrisNp2;
      // Symbol: drake::planning::IrisNp2Options
      struct /* IrisNp2Options */ {
        // Source: drake/planning/iris/iris_np2.h
        const char* doc =
R"""(IrisNp2Options collects all parameters for the IRIS-NP2 algorithm.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

See also:
    IrisNp2 for more details.)""";
        // Symbol: drake::planning::IrisNp2Options::IrisNp2Options
        struct /* ctor */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::planning::IrisNp2Options::Serialize
        struct /* Serialize */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background. Note: This only serializes options that
are YAML built-in types.)""";
        } Serialize;
        // Symbol: drake::planning::IrisNp2Options::add_hyperplane_if_solve_fails
        struct /* add_hyperplane_if_solve_fails */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(Add a hyperplane at a particle in collision if the nonlinear solve
(initialized at that point) fails. Generally leads to regions with
more faces, but helpful for getting the algorithm unstuck if most
nonlinear solves are failing.)""";
        } add_hyperplane_if_solve_fails;
        // Symbol: drake::planning::IrisNp2Options::parameterization
        struct /* parameterization */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(Parameterization of the subspace along which to grow the region.
Default is the identity parameterization, corresponding to growing
regions in the ordinary configuration space.)""";
        } parameterization;
        // Symbol: drake::planning::IrisNp2Options::ray_sampler_options
        struct /* ray_sampler_options */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(Additional options for kRaySampler. Ignored if kGreedySampler is used.)""";
        } ray_sampler_options;
        // Symbol: drake::planning::IrisNp2Options::sampled_iris_options
        struct /* sampled_iris_options */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc = R"""(Options common to IRIS-type algorithms.)""";
        } sampled_iris_options;
        // Symbol: drake::planning::IrisNp2Options::sampling_strategy
        struct /* sampling_strategy */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(Which sampling strategy to use when growing the region. Use "ray" for
kRaySmpler, and "greedy" for kGreedySampler. kRaySampler finds
collisions closer to the ellipsoid center in order to achieve more
efficient hyperplane placement, yielding fewer hyperplanes in the
resulting region, but may take more runtime than kGreedySampler.

Note:
    See §5.3 of [Werner et al., 2024] for further details.)""";
        } sampling_strategy;
        // Symbol: drake::planning::IrisNp2Options::solver
        struct /* solver */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(The user can specify a solver to use for the counterexample search
program. If nullptr (the default value) is given, then
solvers∷MakeFirstAvailableSolver will be used to pick the solver.)""";
        } solver;
        // Symbol: drake::planning::IrisNp2Options::solver_options
        struct /* solver_options */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(Options passed to the counterexample search program solver.)""";
        } solver_options;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("add_hyperplane_if_solve_fails", add_hyperplane_if_solve_fails.doc),
            std::make_pair("parameterization", parameterization.doc),
            std::make_pair("ray_sampler_options", ray_sampler_options.doc),
            std::make_pair("sampled_iris_options", sampled_iris_options.doc),
            std::make_pair("sampling_strategy", sampling_strategy.doc),
            std::make_pair("solver", solver.doc),
            std::make_pair("solver_options", solver_options.doc),
          };
        }
      } IrisNp2Options;
      // Symbol: drake::planning::IrisParameterizationFunction
      struct /* IrisParameterizationFunction */ {
        // Source: drake/planning/iris/iris_common.h
        const char* doc =
R"""(Ordinarily, IRIS algorithms grow collision free regions in the robot's
configuration space C. This allows the user to specify a function
f:Q→C , and grow the region in Q instead. The function should be a map
R^m to R^n, where n is the dimension of the plant configuration space
and m is the input dimension, if specified. If the user provides a
version of the function for Eigen∷VectorX<double>, then the
parameterization can be used with IrisZo. IrisNp2 requires that the
user also provies a version of the function for
Eigen∷VectorX<AutoDiffXd>. If not specified, the input dimension is
assumed to be equal to the output dimension. The user must also
specify whether or not the parameterization function can be called in
parallel.)""";
        // Symbol: drake::planning::IrisParameterizationFunction::IrisParameterizationFunction
        struct /* ctor */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc_3args_parameterization_double_parameterization_is_threadsafe_parameterization_dimension =
R"""(Constructor for when the user only provides a version of the
parameterization function for Eigen∷VectorX<double>.
``parameterization_double`` is the function itself,
``parameterization_is_threadsafe`` specifies whether or not its
threadsafe, and ``parameterization_dimension`` is the input dimension.)""";
          // Source: drake/planning/iris/iris_common.h
          const char* doc_4args_parameterization_double_parameterization_autodiff_parameterization_is_threadsafe_parameterization_dimension =
R"""(Constructor for when the user only provides both versions of the
parameterization function. ``parameterization_double`` is the version
for Eigen∷VectorX<double>, ``parameterization_autodiff_`` is the
version for Eigen∷VectorX<AutoDiffXd>,
``parameterization_is_threadsafe`` specifies whether or not its
threadsafe, and ``parameterization_dimension`` is the input dimension.)""";
          // Source: drake/planning/iris/iris_common.h
          const char* doc_0args =
R"""(Default constructor -- returns the identity mapping, which is
threadsafe and compatible with any dimension configuration space.)""";
          // Source: drake/planning/iris/iris_common.h
          const char* doc_2args_expression_parameterization_variables =
R"""(Alternative constructor that allows the user to define the
parameterization using a ``VectorX<Expression>``. The user must also
provide a vector containing the variables used in
``expression_parameterization``, in the order that they should be
evaluated. Each ``Variable`` in ``variables`` must be used, each
``Variable`` used in ``expression_parameterization`` must appear in
``variables``, and there must be no duplicates in ``variables``.

Note:
    This constructor only populates the VectorX<double>
    parameterization.

Note:
    Expression parameterizations are always threadsafe.

Raises:
    if the number of variables used across
    ``expression_parameterization`` does not match
    ``ssize(variables)``.

Raises:
    if any variables in ``expression_parameterization`` are not listed
    in ``variables``.

Raises:
    if any variables in ``variables`` are not used anywhere in
    ``expression_parameterization``.)""";
          // Source: drake/planning/iris/iris_common.h
          const char* doc_2args_kin_q_star_val =
R"""(Constructs an instance of IrisParameterizationFunction that handles a
rational kinematic parameterization. Regions are grown in the ``s``
variables, so as to minimize collisions in the ``q`` variables. See
RationalForwardKinematics for details.

Note:
    This constructor populates the VectorX<double> and
    VectorX<AutoDiffXd> parameterizations.

Note:
    The user is responsible for ensuring ``kin`` (and the underlying
    MultibodyPlant it is built on) is kept alive. If that object is
    deleted, then the parameterization can no longer be used.)""";
        } ctor;
        // Symbol: drake::planning::IrisParameterizationFunction::ParameterizationFunctionAutodiff
        struct /* ParameterizationFunctionAutodiff */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc = R"""()""";
        } ParameterizationFunctionAutodiff;
        // Symbol: drake::planning::IrisParameterizationFunction::ParameterizationFunctionDouble
        struct /* ParameterizationFunctionDouble */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc = R"""()""";
        } ParameterizationFunctionDouble;
        // Symbol: drake::planning::IrisParameterizationFunction::get_parameterization_autodiff
        struct /* get_parameterization_autodiff */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Get the Eigen∷VectorX<AutoDiffXd> parameterization function.

Note:
    If the user has not specified this with
    ``set_parameterization()``, then the default value of
    ``parameterization_double_`` is the identity function, indicating
    that the regions should be grown in the full configuration space
    (in the standard coordinate system).

Raises:
    If the user has specified the VectorX<double> parameterization but
    not the VectorX<AutoDiffXd> parameterization.)""";
        } get_parameterization_autodiff;
        // Symbol: drake::planning::IrisParameterizationFunction::get_parameterization_dimension
        struct /* get_parameterization_dimension */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Returns what the user has specified as the input dimension for the
parameterization function, or std∷nullopt if it has not been set. A
std∷nullopt value indicates that the chosen IRIS algorithm should use
the ambient configuration space dimension as the input dimension to
the parameterization.)""";
        } get_parameterization_dimension;
        // Symbol: drake::planning::IrisParameterizationFunction::get_parameterization_double
        struct /* get_parameterization_double */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Get the Eigen∷VectorX<double> parameterization function.

Note:
    If the user has not specified this with
    ``set_parameterization()``, then the default value of
    ``parameterization_double_`` is the identity function, indicating
    that the regions should be grown in the full configuration space
    (in the standard coordinate system).)""";
        } get_parameterization_double;
        // Symbol: drake::planning::IrisParameterizationFunction::get_parameterization_is_threadsafe
        struct /* get_parameterization_is_threadsafe */ {
          // Source: drake/planning/iris/iris_common.h
          const char* doc =
R"""(Returns whether or not the user has specified the parameterization to
be threadsafe.

Note:
    The default parameterization is the identity function, which is
    threadsafe.)""";
        } get_parameterization_is_threadsafe;
      } IrisParameterizationFunction;
      // Symbol: drake::planning::IrisZo
      struct /* IrisZo */ {
        // Source: drake/planning/iris/iris_zo.h
        const char* doc =
R"""(The IRIS-ZO (Iterative Regional Inflation by Semidefinite programming
- Zero Order) algorithm, as described in

P. Werner, T. Cohn\*, R. H. Jiang\*, T. Seyde, M. Simchowitz, R.
Tedrake, and D. Rus, "Faster Algorithms for Growing Collision-Free
Convex Polytopes in Robot Configuration Space," &nbsp;* Denotes equal
contribution.

https://groups.csail.mit.edu/robotics-center/public_papers/Werner24.pdf

This algorithm constructs probabilistically collision-free polytopes
in robot configuration space while only relying on a collision
checker. The sets are constructed using a simple parallel zero-order
optimization strategy. The produced polytope P is probabilistically
collision-free in the sense that one gets to control the probability δ
that the fraction of the volume-in-collision is larger than ε

Pr[λ(P\Cfree)/λ(P) > ε] ≤ δ.

Parameter ``starting_ellipsoid``:
    provides the initial ellipsoid around which to grow the region.
    This is typically a small ball around a collision-free
    configuration (e.g. Hyperellipsoid∷MakeHyperSphere(radius,
    seed_point)). The center of this ellipsoid is required to be
    collision-free.

Parameter ``domain``:
    describes the total region of interest; computed IRIS regions will
    be inside this domain. It must be bounded, and is typically a
    simple bounding box representing joint limits (e.g. from
    HPolyhedron∷MakeBox).

Parameter ``options``:
    contains algorithm parameters such as the desired collision-free
    fraction, confidence level, and various algorithmic settings.

The ``starting_ellipsoid`` and ``domain`` must describe elements in
the same ambient dimension as the configuration space of the robot,
unless a parameterization is specified (in which case, they must match
``options.parameterization_dimension``).

Returns:
    A HPolyhedron representing the computed collision-free region in
    configuration space. $Warning:

This feature is considered to be **experimental** and may change or be
removed at any time, without any deprecation notice ahead of time.

Raises:
    if the center of ``starting_ellipsoid`` is in collision, or
    violates any of the user-specified constraints in
    ``options.prog_with_additional_constraints``.

Note:
    This can be a long running function that needs to solve many QPs.
    If you have a solver which requires a license, consider acquiring
    the license before solving this function. See AcquireLicense for
    more details.)""";
      } IrisZo;
      // Symbol: drake::planning::IrisZoOptions
      struct /* IrisZoOptions */ {
        // Source: drake/planning/iris/iris_zo.h
        const char* doc =
R"""(IrisZoOptions collects all parameters for the IRIS-ZO algorithm.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

See also:
    IrisZo for more details.)""";
        // Symbol: drake::planning::IrisZoOptions::IrisZoOptions
        struct /* ctor */ {
          // Source: drake/planning/iris/iris_zo.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::planning::IrisZoOptions::Serialize
        struct /* Serialize */ {
          // Source: drake/planning/iris/iris_zo.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background. Note: This only serializes options that
are YAML built-in types.)""";
        } Serialize;
        // Symbol: drake::planning::IrisZoOptions::bisection_steps
        struct /* bisection_steps */ {
          // Source: drake/planning/iris/iris_zo.h
          const char* doc = R"""(Maximum number of bisection steps.)""";
        } bisection_steps;
        // Symbol: drake::planning::IrisZoOptions::parameterization
        struct /* parameterization */ {
          // Source: drake/planning/iris/iris_zo.h
          const char* doc =
R"""(Parameterization of the subspace along which to grow the region.
Default is the identity parameterization, corresponding to growing
regions in the ordinary configuration space.)""";
        } parameterization;
        // Symbol: drake::planning::IrisZoOptions::sampled_iris_options
        struct /* sampled_iris_options */ {
          // Source: drake/planning/iris/iris_zo.h
          const char* doc =
R"""(Options pertaining to the sampling and termination conditions.)""";
        } sampled_iris_options;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("bisection_steps", bisection_steps.doc),
            std::make_pair("parameterization", parameterization.doc),
            std::make_pair("sampled_iris_options", sampled_iris_options.doc),
          };
        }
      } IrisZoOptions;
      // Symbol: drake::planning::RaySamplerOptions
      struct /* RaySamplerOptions */ {
        // Source: drake/planning/iris/iris_np2.h
        const char* doc =
R"""(RaySamplerOptions contains settings specific to the kRaySampler
strategy for drawing the initial samples.)""";
        // Symbol: drake::planning::RaySamplerOptions::Serialize
        struct /* Serialize */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background. Note: This only serializes options that
are YAML built-in types.)""";
        } Serialize;
        // Symbol: drake::planning::RaySamplerOptions::num_particles_to_walk_towards
        struct /* num_particles_to_walk_towards */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(The number of particles to step towards. Ignored if
only_walk_toward_collisions is true, because we walk toward all
collisions in that case. Must be at least 1.)""";
        } num_particles_to_walk_towards;
        // Symbol: drake::planning::RaySamplerOptions::only_walk_toward_collisions
        struct /* only_walk_toward_collisions */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(If true, the ray stepping strategy is only applied to samples which
are initially in collision. If false, it is applied to all samples.)""";
        } only_walk_toward_collisions;
        // Symbol: drake::planning::RaySamplerOptions::ray_search_num_steps
        struct /* ray_search_num_steps */ {
          // Source: drake/planning/iris/iris_np2.h
          const char* doc =
R"""(The step size for the ray search is defined per-particle, as the
distance between the current ellipsoid center and the particle,
divided by this option. A larger number requires more time for
computing samples, but will lead to the samples in-collision being
closer to the ellipsoid center, and higher quality hyperplanes. We
choose a default value to roughly match the results in [Werner et al.,
2024]. Must be at least 1.)""";
        } ray_search_num_steps;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("num_particles_to_walk_towards", num_particles_to_walk_towards.doc),
            std::make_pair("only_walk_toward_collisions", only_walk_toward_collisions.doc),
            std::make_pair("ray_search_num_steps", ray_search_num_steps.doc),
          };
        }
      } RaySamplerOptions;
    } planning;
  } drake;
} pydrake_doc_planning_iris;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
