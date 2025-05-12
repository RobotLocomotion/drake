#include "drake/planning/iris/iris_np2.h"

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"

namespace drake {
namespace planning {

using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;

HPolyhedron IrisNp2(const SceneGraphCollisionChecker& checker,
                    const Hyperellipsoid& starting_ellipsoid,
                    const HPolyhedron& domain, const IrisNp2Options& options) {
  unused(checker);
  unused(starting_ellipsoid);
  unused(domain);
  unused(options);
  throw std::runtime_error("IrisNp2 is not yet implemented.");
  return HPolyhedron{};
}

}  // namespace planning
}  // namespace drake
