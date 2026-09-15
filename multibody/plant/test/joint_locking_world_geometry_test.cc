#include <gtest/gtest.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

// Regression test for #24773: locking a joint must not crash when contact
// involves collision geometry attached to the World body (invalid TreeIndex).
GTEST_TEST(JointLockingWorldGeometryTest, WorldContactDoesNotCrash) {
  systems::DiagramBuilder<double> builder;
  auto items = AddMultibodyPlantSceneGraph(&builder, /* time_step = */ 0.01);
  MultibodyPlant<double>& plant = items.plant;

  plant.RegisterCollisionGeometry(
      plant.world_body(), math::RigidTransformd(), geometry::HalfSpace(),
      "ground", geometry::ProximityProperties());
  const RigidBody<double>& box = plant.AddRigidBody(
      "box", SpatialInertia<double>::SolidBoxWithMass(1.0, 0.1, 0.1, 0.1));
  plant.RegisterCollisionGeometry(box, math::RigidTransformd(),
                                  geometry::Box(0.1, 0.1, 0.1), "box",
                                  geometry::ProximityProperties());
  plant.Finalize();

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());

  plant.SetFreeBodyPose(&plant_context, box,
                        math::RigidTransformd(Eigen::Vector3d(0, 0, 0.049)));
  for (JointIndex i : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(i);
    if (joint.num_velocities() == 6) {
      joint.Lock(&plant_context);
    }
  }

  EXPECT_NO_THROW(simulator.AdvanceTo(0.01));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
