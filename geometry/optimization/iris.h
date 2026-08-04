#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/name_value.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace geometry {
namespace optimization {

/** Configuration options for the IRIS algorithm.

@ingroup geometry_optimization
@ingroup planning_iris
*/
struct IrisOptions {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background.
  Note: This only serializes options that are YAML built-in types. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(require_sample_point_is_contained));
    a->Visit(DRAKE_NVP(iteration_limit));
    a->Visit(DRAKE_NVP(termination_threshold));
    a->Visit(DRAKE_NVP(relative_termination_threshold));
    a->Visit(DRAKE_NVP(configuration_space_margin));
    a->Visit(DRAKE_NVP(num_collision_infeasible_samples));
    a->Visit(DRAKE_NVP(num_additional_constraint_infeasible_samples));
    a->Visit(DRAKE_NVP(random_seed));
    a->Visit(DRAKE_NVP(mixing_steps));
    a->Visit(DRAKE_NVP(convexity_radius_stepback));
    a->Visit(DRAKE_NVP(verify_domain_boundedness));
  }

  /** The initial polytope is guaranteed to contain the point if that point is
  collision-free. However, the IRIS alternation objectives do not include (and
  can not easily include) a constraint that the original sample point is
  contained. Therefore, the IRIS paper recommends that if containment is a
  requirement, then the algorithm should simply terminate early if alternations
  would ever cause the set to not contain the point. */
  bool require_sample_point_is_contained{false};

  /** Maximum number of iterations. */
  int iteration_limit{100};

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this threshold. This termination condition can
  be disabled by setting to a negative value. */
  double termination_threshold{2e-2};  // from rdeits/iris-distro.

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this percent of the previous best volume.
  This termination condition can be disabled by setting to a negative value. */
  double relative_termination_threshold{1e-3};  // from rdeits/iris-distro.

  // TODO(russt): Improve the implementation so that we can clearly document the
  // units for this margin.
  /** For IrisNp, we retreat by this margin from each C-space obstacle in order
  to avoid the possibility of requiring an infinite number of faces to
  approximate a curved boundary.
  */
  double configuration_space_margin{1e-2};

  /** For each possible collision, IRIS will search for a counter-example by
  formulating a (likely nonconvex) optimization problem. The initial guess for
  this optimization is taken by sampling uniformly inside the current IRIS
  region. This option controls the termination condition for that
  counter-example search, defining the number of consecutive failures to find a
  counter-example requested before moving on to the next constraint. */
  int num_collision_infeasible_samples{5};

  /** For IrisNp, it can be beneficial to not only specify task-space obstacles
  (passed in through the plant) but also obstacles that are defined by convex
  sets in the configuration space. This option can be used to pass in such
  configuration space obstacles. */
  ConvexSets configuration_obstacles{};

  /** The initial hyperellipsoid that IRIS will use for calculating hyperplanes
  in the first iteration. If no hyperellipsoid is provided, a small hypershpere
  centered at the given sample will be used. */
  std::optional<Hyperellipsoid> starting_ellipse{};

  /** Optionally allows the caller to restrict the space within which IRIS
  regions are allowed to grow. By default, IRIS regions are bounded by the
  `domain` argument in the case of `Iris` or the joint limits of the input
  `plant` in the case of `IrisNp`. If this option is specified, IRIS regions
  will be confined to the intersection between the domain and `bounding_region`.
  */
  std::optional<HPolyhedron> bounding_region{};

  /** If the user knows the intersection of bounding_region and the domain (for
  IRIS) or plant joint limits (for IrisNp) is bounded, setting this flag to
  `false` will skip the boundedness check that IRIS and IrisNp perform (leading
  to a small speedup, as checking boundedness requires solving optimization
  problems). If the intersection turns out to be unbounded, this will lead to
  undefined behavior. */
  bool verify_domain_boundedness{true};

  /** By default, IrisNp certifies regions for collision avoidance constraints
  and joint limits. This option can be used to pass additional constraints that
  should be satisfied by the IRIS region. We accept these in the form of a
  MathematicalProgram:

    find q subject to g(q) ≤ 0.

  The decision_variables() for the program are taken to define `q`. IRIS will
  silently ignore any costs in `prog_with_additional_constraints`, and will
  throw std::runtime_error if it contains any unsupported constraints.

  For example, one could create an InverseKinematics problem with rich
  kinematic constraints, and then pass `InverseKinematics::prog()` into this
  option.
  */
  const solvers::MathematicalProgram* prog_with_additional_constraints{};

  /** For each constraint in `prog_with_additional_constraints`, IRIS will
  search for a counter-example by formulating a (likely nonconvex) optimization
  problem. The initial guess for this optimization is taken by sampling
  uniformly inside the current IRIS region. This option controls the
  termination condition for that counter-example search, defining the number of
  consecutive failures to find a counter-example requested before moving on to
  the next constraint. */
  int num_additional_constraint_infeasible_samples{5};

  /** The only randomization in IRIS is the random sampling done to find
  counter-examples for the additional constraints using in IrisNp. Use this
  option to set the initial seed. */
  int random_seed{1234};

  /** Passing a meshcat instance may enable debugging visualizations; this
  currently only happens in IrisNp and when the configuration space is <= 3
  dimensional.*/
  std::shared_ptr<Meshcat> meshcat{};

  /** A user-defined termination function to
  determine whether the iterations should stop. This function is called after
  computing each hyperplane at every IRIS iteration. If the function returns
  true, then the computations will stop and the last step region will be
  returned. Therefore, it is highly recommended that the termination function
  possesses a monotonic property such that for any two HPolyhedrons A and B such
  that B ⊆ A, we have if termination(A) -> termination(B). For example, a valid
  termination function is to check whether if the region does not contain any of
  a set of desired points.
  ```
  auto termination_func = [](const HPolyhedron& set) {
    for (const VectorXd& point : desired_points) {
      if (!set.PointInSet(point)) {
        return true;
      }
    }
    return false;
  };
  ```
  The algorithm will stop when as soon as the region leaves one
  of the desired points, in a similar way to how @p
  require_sample_point_is_contained is enforced.
  */
  std::function<bool(const HPolyhedron&)> termination_func{};

  /** The `mixing_steps` parameters is passed to HPolyhedron::UniformSample to
  control the total number of hit-and-run steps taken for each new random
  sample. */
  int mixing_steps{10};

  /** The SolverOptions used in the optimization program. */
  std::optional<solvers::SolverOptions> solver_options;

  /** Artificial joint limits are added to continuous revolute joints and planar
  joints with an unbounded revolute degree-of-freedom on a per-region basis. If
  the seed point value for that joint is θ, then the limits are
  θ - π/2 + convexity_radius_stepback and θ + π/2 - convexity_radius_stepback.
  Setting this to a negative number allows growing larger regions, but those
  regions must then be partitioned to be used with GcsTrajectoryOptimization.
  See @ref geometry_optimization_geodesic_convexity for more details.
  IrisNp throws if this value is not smaller than π/2. */
  double convexity_radius_stepback{1e-3};
};

/** The IRIS (Iterative Region Inflation by Semidefinite programming) algorithm,
as described in

R. L. H. Deits and R. Tedrake, “Computing large convex regions of obstacle-free
space through semidefinite programming,” Workshop on the Algorithmic
Fundamentals of Robotics, Istanbul, Aug. 2014.
http://groups.csail.mit.edu/robotics-center/public_papers/Deits14.pdf

This algorithm attempts to locally maximize the volume of a convex polytope
representing obstacle-free space given a sample point and list of convex
obstacles. Rather than compute the volume of the polytope directly, the
algorithm maximizes the volume of an inscribed ellipsoid. It alternates between
finding separating hyperplanes between the ellipsoid and the obstacles and then
finding a new maximum-volume inscribed ellipsoid.

@param obstacles is a vector of convex sets representing the occupied space.
@param sample provides a point in the space; the algorithm is initialized using
a tiny sphere around this point. The algorithm is only guaranteed to succeed if
this sample point is collision free (outside of all obstacles), but in practice
the algorithm can often escape bad initialization (assuming the
require_sample_point_is_contained option is false).
@param domain describes the total region of interest; computed IRIS regions will
be inside this domain.  It must be bounded, and is typically a simple bounding
box (e.g. from HPolyhedron::MakeBox).

The @p obstacles, @p sample, and the @p domain must describe elements in the
same ambient dimension (but that dimension can be any positive integer).

@note Some members of `options` are only applicable to IrisNp. The members
relevant for this function are starting_ellipse, termination_func,
bounding_region, verify_domain_boundedness, require_sample_point_is_contained,
iteration_limit, termination_threshold, relative_termination_threshold.

@ingroup geometry_optimization
@ingroup planning_iris
*/
HPolyhedron Iris(const ConvexSets& obstacles,
                 const Eigen::Ref<const Eigen::VectorXd>& sample,
                 const HPolyhedron& domain,
                 const IrisOptions& options = IrisOptions());

/** Constructs ConvexSet representations of obstacles for IRIS in 3D using the
geometry from a SceneGraph QueryObject. All geometry in the scene with a
proximity role, both anchored and dynamic, are consider to be *fixed*
obstacles frozen in the poses captured in the context used to create the
QueryObject.

When multiple representations are available for a particular geometry (e.g. a
Box can be represented as either an HPolyhedron or a VPolytope), then this
method will prioritize the representation that we expect is most performant
for the current implementation of the IRIS algorithm.

@ingroup geometry_optimization
@ingroup planning_iris
*/
ConvexSets MakeIrisObstacles(
    const QueryObject<double>& query_object,
    std::optional<FrameId> reference_frame = std::nullopt);

/** A variation of the Iris (Iterative Region Inflation by Semidefinite
programming) algorithm which finds collision-free regions in the *configuration
space* of @p plant.  @see Iris for details on the original algorithm. This
variant uses nonlinear optimization (instead of convex optimization) to find
collisions in configuration space; each potential collision is
probabilistically "certified" by restarting the nonlinear optimization from
random initial seeds inside the candidate IRIS region until it fails to find a
collision in `options.num_collision_infeasible_samples` consecutive attempts.

This method constructs a single Iris region in the configuration space of
@p plant. @see planning::IrisInConfigurationSpaceFromCliqueCover for a method to
automatically cover the configuration space with multiple Iris regions.

@param plant describes the kinematics of configuration space.  It must be
connected to a SceneGraph in a systems::Diagram.
@param context is a context of the @p plant. The context must have the
positions of the plant set to the initialIRIS seed configuration.
@param options provides additional configuration options.  In particular,
increasing `options.num_collision_infeasible_samples` increases the chances that
the IRIS regions are collision free but can also significantly increase the
run-time of the algorithm. The same goes for
`options.num_additional_constraints_infeasible_samples`.

@throws std::exception if the sample configuration in @p context is infeasible.
@throws std::exception if termination_func is invalid on the domain. See
IrisOptions.termination_func for more details.

@ingroup geometry_optimization
@ingroup planning_iris
*/
HPolyhedron IrisNp(const multibody::MultibodyPlant<double>& plant,
                   const systems::Context<double>& context,
                   const IrisOptions& options = IrisOptions());

/** Modifies the @p iris_options to facilitate finding a region that contains
the edge between x_1 and x_2. It sets @p iris_options.starting_ellipse to be a
hyperellipsoid that contains the edge, is centered at the midpoint of the
edge and extends in other directions by epsilon. It also sets @p
iris_options.termination_func such that IRIS iterations terminate when the edge
is no longer contained in the IRIS region with tolerance tol.

@throws std::exception if x_1.size() != x_2.size().
@throws std::exception if epsilon <= 0. This is due to the fact that the
hyperellipsoid for @p iris_options.starting_ellipse must have non-zero volume.

@ingroup geometry_optimization
@ingroup planning_iris
*/
void SetEdgeContainmentTerminationCondition(
    IrisOptions* iris_options, const Eigen::Ref<const Eigen::VectorXd>& x_1,
    const Eigen::Ref<const Eigen::VectorXd>& x_2, const double epsilon = 1e-3,
    const double tol = 1e-6);

/** Defines a standardized representation for (named) IrisRegions, which can be
serialized in both C++ and Python.

@ingroup planning_iris */
typedef std::map<std::string, HPolyhedron> IrisRegions;

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
