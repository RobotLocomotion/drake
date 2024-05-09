#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/parallelism.h"
#include "drake/geometry/meshcat.h"
// #include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {

using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;

struct FastIrisOptions {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background.
  Note: This only serializes options that are YAML built-in types. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(num_particles));
    a->Visit(DRAKE_NVP(tau));
    a->Visit(DRAKE_NVP(delta));
    a->Visit(DRAKE_NVP(admissible_proportion_in_collision));
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
  }

  FastIrisOptions() = default;

  /** Number of particles used to estimate the closest collision*/
  int num_particles = 1e3;

  /** Descision threshold for unadaptive test.*/
  double tau = 0.5;

  /** Upper bound on the error probability that the admissible_proportion in
   * collision is not met.*/
  double delta = 5e-2;

  /** Admissible fraction of the region volume allowed to be in collision.*/
  double admissible_proportion_in_collision = 1e-2;

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
  collision-free. However, the IRIS alternation objectives do not include (and
  can not easily include) a constraint that the original sample point is
  contained. Therefore, the IRIS paper recommends that if containment is a
  requirement, then the algorithm should simply terminate early if alternations
  would ever cause the set to not contain the point. */
  bool require_sample_point_is_contained{true};

  /** For IRIS in configuration space, we retreat by this margin from each
  C-space obstacle in order to avoid the possibility of requiring an infinite
  number of faces to approximate a curved boundary.
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

  /** Passing a meshcat instance may enable debugging visualizations; this
  currently and when the
  configuration space is <= 3 dimensional.*/
  std::shared_ptr<geometry::Meshcat> meshcat{};
};

/** Given a seed point and an initial ellipsoidal metric, use sampling based
optimization to find a collision free polytope in cspace.*/

HPolyhedron FastIris(const CollisionChecker& checker,
                     const Hyperellipsoid& starting_ellipsoid,
                     const HPolyhedron& domain,
                     const FastIrisOptions& options = FastIrisOptions());

}  // namespace planning
}  // namespace drake
