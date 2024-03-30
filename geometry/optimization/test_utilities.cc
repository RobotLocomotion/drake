#include "drake/geometry/optimization/test_utilities.h"

#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {
using math::RigidTransformd;

std::tuple<std::unique_ptr<SceneGraph<double>>, GeometryId,
           std::unique_ptr<systems::Context<double>>, QueryObject<double>>
MakeSceneGraphWithShape(const Shape& shape, const RigidTransformd& X_WG,
                        bool add_proximity_properties) {
  auto scene_graph = std::make_unique<SceneGraph<double>>();
  SourceId source_id = scene_graph->RegisterSource("test");
  auto instance = std::make_unique<GeometryInstance>(X_WG, shape.Clone(), "G");
  if (add_proximity_properties) {
    instance->set_proximity_properties(ProximityProperties());
  }
  GeometryId geom_id =
      scene_graph->RegisterAnchoredGeometry(source_id, std::move(instance));
  auto context = scene_graph->CreateDefaultContext();
  auto query =
      scene_graph->get_query_output_port().Eval<QueryObject<double>>(*context);
  return std::make_tuple(std::move(scene_graph), geom_id, std::move(context),
                         std::move(query));
}

// Returns true iff the convex optimization can make the `point` be in the set.
bool CheckAddPointInSetConstraints(
    const ConvexSet& set, const Eigen::Ref<const Eigen::VectorXd>& point) {
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(point.size());
  const auto [new_vars, new_constraints] =
      set.AddPointInSetConstraints(&prog, x);
  EXPECT_EQ(prog.num_vars(), x.rows() + new_vars.rows());
  EXPECT_FALSE(new_constraints.empty());
  // x = point.
  prog.AddBoundingBoxConstraint(point, point, x);
  return solvers::Solve(prog).is_success();
}

}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
