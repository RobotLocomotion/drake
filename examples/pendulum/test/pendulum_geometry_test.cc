#include "drake/examples/pendulum/pendulum_geometry.h"

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

using geometry::FramePoseVector;
using math::RotationMatrixd;
using math::RigidTransformd;

math::RigidTransformd ExtractSinglePose(
    const geometry::FramePoseVector<double>& pose_vector) {
  DRAKE_THROW_UNLESS(pose_vector.size() == 1);
  for (const auto& id : pose_vector.frame_ids()) {
    return pose_vector.value(id);
  }
  DRAKE_UNREACHABLE();
}

GTEST_TEST(PendulumGeometryTest, AcceptanceTest) {
  // Specify the diagram.
  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<PendulumPlant>();
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  auto geom = PendulumGeometry::AddToBuilder(
      &builder, plant->get_state_output_port(), scene_graph);
  ASSERT_NE(geom, nullptr);

  // Finish the diagram and create a context.
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context = diagram->GetMutableSubsystemContext(
      *plant, diagram_context.get());
  auto& geom_context = diagram->GetMutableSubsystemContext(
      *geom, diagram_context.get());

  // Zero the pendulum input, but set a non-zero initial state.
  const double theta = 0.2;
  plant->get_input_port().FixValue(&plant_context, PendulumInput<double>{});
  plant->get_mutable_state(&plant_context).set_theta(theta);

  // Check the frame pose.
  const double tolerance = 1E-9;  // Arbitrary.
  const auto& geom_output_value =
      geom->get_output_port(0).Eval<FramePoseVector<double>>(geom_context);
  EXPECT_TRUE(CompareMatrices(
      ExtractSinglePose(geom_output_value).GetAsMatrix4(),
      RigidTransformd(RotationMatrixd::MakeYRotation(theta)).GetAsMatrix4(),
      tolerance));

  // Check other stuff.
  EXPECT_TRUE(geom->HasAnyDirectFeedthrough());
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake
