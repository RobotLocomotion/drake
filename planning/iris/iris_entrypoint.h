#pragma once

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {

enum class IrisAlgorithm {
  Unset,   ///< Default value -- must be changed by the user.
  Convex,  ///< The original IRIS algorithm for convex obstacles in task space.
  NP,      ///< The IRIS-NP algorithm, which uses nonlinear programming to grow
           ///< regions in configuration space.
  NP2,     ///< The IRIS-NP2 algorithm, an improved version of IRIS-NP.
  ZO,      ///< The IRIS-ZO algorithm, which uses a gradient-free (zero-order)
           ///< optimization to grow regions in configuration space.
  Certified,  ///< The sums-of-squares certified version of the IRIS algorithm,
              ///< for growing regions in the rational parametrization of
              ///< configuration space.
};

enum class IrisRegionSpace {
  Unset,        ///< Default value -- must be changed by the user.
  TaskSpace2d,  ///< Regions are grown in task space along the horizontal (xz)
                ///< plane.
  TaskSpace3d,  ///< Regions are grown in task space.
  TaskSpaceAbstractNd,  ///< Regions are grown in an abstract n-dimensional
                        ///< space. The CollisionChecker is ignored, and only
                        ///< the convex obstacles specified in the options are
                        ///< used. TODO(cohnt): Clean this explanation up.
  ConfigurationSpace,   ///< Regions are grown in configuration space.
  RationalConfigurationSpace,  ///< Regions are grown in the rational
                               ///< parametrization of configuration space.
};

// TODO(cohnt): Annotate each option with which algorithms and spaces they are
// used for.
// TODO(cohnt): Support serialization.
struct IrisOptions {
  /** TODO(cohnt): Document */
  IrisAlgorithm algorithm = IrisAlgorithm::Unset;

  /** TODO(cohnt): Document */
  IrisRegionSpace region_space = IrisRegionSpace::Unset;

  // TODO(cohnt): Add IRIS-NP options.
  // TODO(cohnt): Add (original) IRIS options.
  // TODO(rhjiang): Add IRIS-NP2 options.
  // TODO(wernerpe): Add IRIS-ZO options.
  // TODO(alexandreamice): Add C-IRIS options.
};

/** General entry point for the IRIS algorithm and its variants, used for
generating convex collision-free subsets in task and configuration space.

@pre options.algorithm != IrisAlgorithm::Unset
@pre options.region_space != IrisRegionSpace::Unset
*/
geometry::optimization::HPolyhedron GrowIrisRegion(
    const CollisionChecker& checker, const IrisOptions& options,
    const Eigen::VectorXd seed);

// TODO(cohnt): Add an algorithm/region_space compatibility matrix.
// TODO(cohnt): Add IRIS-NP support.
// TODO(cohnt): Add (original) IRIS support.
// TODO(rhjiang): Add IRIS-NP2 support.
// TODO(wernerpe): Add IRIS-ZO support.
// TODO(alexandreamice): Add C-IRIS support.

}  // namespace planning
}  // namespace drake
