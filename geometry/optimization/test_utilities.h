#pragma once

#include <memory>
#include <tuple>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"

// TODO(jwnimmer-tri) Test utilities belong either in a `test/` subdirectory
// or a `test_utilities/` subdirectory. They should never live in a package-
// level subdirectory alongside Drake library code.

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {

// Constructs a SceneGraph containing only the requested shape as geometry G,
// anchored to the world frame at X_WG. For convience, also returns a default
// context and query object.
std::tuple<std::unique_ptr<SceneGraph<double>>, GeometryId,
           std::unique_ptr<systems::Context<double>>, QueryObject<double>>
MakeSceneGraphWithShape(const Shape& shape, const math::RigidTransformd& X_WG,
                        bool add_proximity_properties = true);

// Returns true iff the convex optimization can make the `point` be in the set.
bool CheckAddPointInSetConstraints(
    const ConvexSet& set, const Eigen::Ref<const Eigen::VectorXd>& point);

}  // namespace internal

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
