#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/parallelism.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {

using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;

struct IrisZoOptions {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background.
  Note: This only serializes options that are YAML built-in types. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(num_particles));
    a->Visit(DRAKE_NVP(tau));
    a->Visit(DRAKE_NVP(delta));
    a->Visit(DRAKE_NVP(epsilon));
    a->Visit(DRAKE_NVP(containment_points));
    a->Visit(DRAKE_NVP(force_containment_points));
    a->Visit(DRAKE_NVP(max_iterations));
    a->Visit(DRAKE_NVP(max_iterations_separating_planes));
    a->Visit(DRAKE_NVP(max_separating_planes_per_iteration));
    a->Visit(DRAKE_NVP(bisection_steps));
    a->Visit(DRAKE_NVP(parallelize));
    a->Visit(DRAKE_NVP(verbose));
    a->Visit(DRAKE_NVP(require_sample_point_is_contained));
    a->Visit(DRAKE_NVP(configuration_space_margin));
    a->Visit(DRAKE_NVP(termination_threshold));
    a->Visit(DRAKE_NVP(relative_termination_threshold));
    a->Visit(DRAKE_NVP(random_seed));
    a->Visit(DRAKE_NVP(mixing_steps));
  }

  IrisZoOptions() = default;

  /** Number of particles used to estimate the closest collision*/
  int num_particles = 1e3;

  /** Descision threshold for unadaptive test.*/
  double tau = 0.5;

  /** Upper bound on the error probability that the fraction-in-collision
   * `epsilon` is not met.*/
  double delta = 5e-2;

  /** Admissible fraction of the region volume allowed to be in collision.*/
  double epsilon = 1e-2;

  /** Points that are guaranteed to be contained in the final region
   * provided their convex hull is collision free.*/
  Eigen::MatrixXd containment_points;

  /** If true, sets faces tangent to the sublevelsets of dist(C), where
   * c is the convex hull of the points passed in `containment_points`.
   */
  bool force_containment_points{false};

  /** Number of resampling steps for the gradient updates*/
  // int num_resampling_steps = 1;

  /** Number Iris Iterations*/
  int max_iterations{2};

  /** Maximum number of rounds of adding faces to the polytope*/
  int max_iterations_separating_planes{20};

  /** Maximum number of faces to add per round of samples*/
  int max_separating_planes_per_iteration{-1};

  /** Maximum number of bisection steps per gradient step*/
  int bisection_steps{10};

  /** Parallelize the updates of the particles*/
  bool parallelize{true};

  /* Enables print statements indicating the progress of fast iris**/
  bool verbose{false};

  /** The initial polytope is guaranteed to contain the point if that point is
  collision-free. */
  bool require_sample_point_is_contained{true};

  /** We retreat by this margin from each C-space obstacle in order to avoid the
  possibility of requiring an infinite number of faces to approximate a curved
  boundary.
  */
  double configuration_space_margin{1e-2};

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this threshold. This termination condition can
  be disabled by setting to a negative value. */
  double termination_threshold{2e-2};

  /** IRIS will terminate if the change in the *volume* of the hyperellipsoid
  between iterations is less that this percent of the previous best volume.
  This termination condition can be disabled by setting to a negative value. */
  double relative_termination_threshold{1e-3};  // from rdeits/iris-distro.

  /** For IRIS in configuration space, it can be beneficial to not only specify
  task-space obstacles (passed in through the plant) but also obstacles that are
  defined by convex sets in the configuration space. This option can be used to
  pass in such configuration space obstacles. */
  // ConvexSets configuration_obstacles{};

  /** The only randomization in IRIS is the random sampling done to find
  counter-examples for the additional constraints using in
  IrisInConfigurationSpace. Use this option to set the initial seed. */
  int random_seed{1234};

  // number of mixing steps used for hit and run sampling
  int mixing_steps{50};

  /** Passing a meshcat instance may enable debugging visualizations; this
  currently and when the
  configuration space is <= 3 dimensional.*/
  std::shared_ptr<geometry::Meshcat> meshcat{};
};

/** The IRIS-ZO (Iterative Regional Inflation by Semidefinite programming - Zero
Order) algorithm, as described in

P. Werner, T. Cohn, R. H. Jiang, T. Seyde, M. Simchowitz, R. Tedrake, and D.
Rus, "Faster Algorithms for Growing Collision-Free Convex Polytopes in Robot
Configuration Space,"

https://groups.csail.mit.edu/robotics-center/public_papers/Werner24.pdf

This algorithm constructs probabilistically collision-free polytopes, in robot
configuration space while only relying on a collision checker. The sets are
constructed using a simple parallel zero-order optimization strategy. The
produced polytope P is probabilistically collision-free in the sense that one
gets to control the probability δ that the fraction of the volume-in-collision
is larger than ε

Pr[λ(P\Cfree)/λ(P) > ε] ⋞ δ.

@param starting_ellipsoid provides the initial ellipsoid around which to grow
the region. This is typically a small ball around a collision-free
configuration. The center of this ellipsoid is required to be collision-free.
@param domain describes the total region of interest; computed IRIS regions will
be inside this domain. It must be bounded, and is typically a simple bounding
box representing joint limits (e.g. from HPolyhedron::MakeBox).
@param options contains algorithm parameters such as the desired collision-free
fraction, confidence level, and various algorithmic settings.

The @p starting_ellipsoid and @p domain must describe elements in the same
ambient dimension as the configuration space of the robot.
@return A HPolyhedron representing the computed collision-free region in
configuration space.
@ingroup robot_planning
*/

HPolyhedron IrisZO(const CollisionChecker& checker,
                   const Hyperellipsoid& starting_ellipsoid,
                   const HPolyhedron& domain,
                   const IrisZoOptions& options = IrisZoOptions());
}  // namespace planning
}  // namespace drake
