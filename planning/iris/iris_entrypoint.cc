#include "drake/planning/iris/iris_entrypoint.h"

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {

using geometry::optimization::HPolyhedron;

HPolyhedron GrowIrisRegion(const CollisionChecker& checker,
                           const IrisOptions& options,
                           const Eigen::VectorXd seed) {
  DRAKE_DEMAND(options.algorithm != IrisAlgorithm::Unset);
  DRAKE_DEMAND(options.region_space != IrisRegionSpace::Unset);

  if (options.algorithm == IrisAlgorithm::Convex) {
    throw std::runtime_error("IrisAlgorithm::Convex is not supported yet.");
  }
  if (options.algorithm == IrisAlgorithm::NP) {
    throw std::runtime_error("IrisAlgorithm::NP is not supported yet.");
  }
  if (options.algorithm == IrisAlgorithm::NP2) {
    throw std::runtime_error("IrisAlgorithm::NP2 is not supported yet.");
  }
  if (options.algorithm == IrisAlgorithm::ZO) {
    throw std::runtime_error("IrisAlgorithm::ZO is not supported yet.");
  }
  if (options.algorithm == IrisAlgorithm::Certified) {
    throw std::runtime_error("IrisAlgorithm::Certified is not supported yet.");
  }

  unused(checker);

  return HPolyhedron::MakeUnitBox(seed.size());
}

}  // namespace planning
}  // namespace drake
