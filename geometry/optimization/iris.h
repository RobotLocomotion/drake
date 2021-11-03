#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "drake/common/drake_deprecated.h"
#include "drake/common/symbolic.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace geometry {
namespace optimization {

/** Configuration options for the IRIS algorithm.

@ingroup geometry_optimization
*/
struct IrisOptions {
  IrisOptions() = default;

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
  between iterations is less that this percent of the previouse best volume.
  This termination condition can be disabled by setting to a negative value. */
  double relative_termination_threshold{1e-3};  // from rdeits/iris-distro.

  // TODO(russt): Improve the implementation so that we can clearly document the
  // units for this margin.
  /** For IRIS in configuration space, we retreat by this margin from each
  C-space obstacle in order to avoid the possibility of requiring an infinite
  number of faces to approximate a curved boundary.
  */
  double configuration_space_margin{1e-2};

  /** For IRIS in configuration space, we use IbexSolver to rigorously confirm
  that regions are collision-free. This step may be computationally
  demanding, so we allow it to be disabled for a faster algorithm for obtaining
  regions without the rigorous guarantee. */
  bool enable_ibex = true;
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

@ingroup geometry_optimization
*/
HPolyhedron Iris(const ConvexSets& obstacles,
                 const Eigen::Ref<const Eigen::VectorXd>& sample,
                 const HPolyhedron& domain,
                 const IrisOptions& options = IrisOptions());

/** Constructs ConvexSet representations of obstacles for IRIS in 3D using the
geometry from a SceneGraph QueryObject. All geometry in the scene with a
proximity role, both anchored and dynamic, are consider to be *fixed* obstacles
frozen in the poses captured in the context used to create the QueryObject.

When multiple representations are available for a particular geometry (e.g. a
Box can be represented as either an HPolyhedron or a VPolytope), then this
method will prioritize the representation that we expect is most performant for
the current implementation of the IRIS algorithm.

@ingroup geometry_optimization
*/
ConvexSets MakeIrisObstacles(
    const QueryObject<double>& query_object,
    std::optional<FrameId> reference_frame = std::nullopt);

/** A variation of the Iris (Iterative Region Inflation by Semidefinite
programming) algorithm which finds collision-free regions in the *configuration
space* of @p plant.  @see Iris for details on the original algorithm.
The possibility of this configuration-space variant was suggested in the
original IRIS paper, but substantial new ideas have been employed here to
address the non-convexity of configuration-space obstacles; these will be
documented in a forth-coming publication.

@param plant describes the kinematics of configuration space.  It must be
connected to a SceneGraph in a systems::Diagram.
@param context is a context of the @p plant. The context must have the positions
of the plant set to the initialIRIS seed configuration.
@param options provides additional configuration options.  In particular,
`options.enabled_ibex` may have a significant impact on the runtime of the
algorithm.

@ingroup geometry_optimization
*/
HPolyhedron IrisInConfigurationSpace(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const IrisOptions& options = IrisOptions());

/** A deprecated variation of the IrisInConfigurationSpace method where the
initial Iris seed configuration is provided explicitly instead of via the
context.

@ingroup geometry_optimization
*/
DRAKE_DEPRECATED("2022-03-01",
                 "Use IrisInConfigurationSpace() with sample set in context.")
HPolyhedron IrisInConfigurationSpace(
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& context,
    const Eigen::Ref<const Eigen::VectorXd>& sample,
    const IrisOptions& options = IrisOptions());

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
