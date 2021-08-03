#pragma once

#include <memory>
#include <tuple>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace optimization {

namespace internal {

// Constructs a SceneGraph containing only the requested shape as geometry G,
// anchored to the world frame at X_WG.
std::tuple<std::unique_ptr<SceneGraph<double>>, GeometryId>
MakeSceneGraphWithShape(const Shape& shape, const math::RigidTransformd& X_WG);

// Returns true iff the convex optimization can make the `point` be in the set.
bool CheckAddPointInSetConstraints(
    const ConvexSet& set, const Eigen::Ref<const Eigen::VectorXd>& point);

}  // namespace internal

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
