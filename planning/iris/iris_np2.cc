#include "drake/planning/iris/iris_np2.h"

#include "drake/common/text_logging.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"

namespace drake {
namespace planning {

using Eigen::VectorXd;
using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;

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

  unused(domain);

  auto stop = std::chrono::high_resolution_clock::now();
  if (options.sampled_iris_options.verbose) {
    log()->info(
        "IrisNp2 execution time : {} ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start)
            .count());
  }
  return HPolyhedron::MakeBox(checker.plant().GetPositionLowerLimits(),
                              checker.plant().GetPositionUpperLimits());
}

}  // namespace planning
}  // namespace drake
