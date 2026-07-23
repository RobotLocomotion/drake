/* Minimal reproduction of a crash in the joint-locking contact filter.

When a joint is locked, MultibodyPlant::CalcGeometryContactData() filters out
contacts between bodies whose trees have no unlocked degrees of freedom. For
each contact geometry it does (multibody_plant.cc):

    const BodyIndex body_index = geometry_id_to_body_index.at(geometry_id);
    const TreeIndex tree_index = forest.link_to_tree_index(body_index);
    const SpanningForest::Tree& tree = forest.trees(tree_index);
    return !tree.has_dofs() ||
           per_tree_unlocked_indices[tree_index].size() == 0;

For collision geometry on the World body, link_to_tree_index() returns an
INVALID TreeIndex -- the World body belongs to no tree, which is documented
behavior (spanning_forest.h: "An invalid tree index is returned if the Link's
Mobod is World"). The filter then indexes forest.trees() and
per_tree_unlocked_indices[] with that invalid index. The only guard is a
DRAKE_ASSERT in SpanningForest::trees(), so:

  * assertion-armed builds abort at spanning_forest_inlines.h in trees() with
    "condition 'tree_index.is_valid() && ...' failed", and
  * NDEBUG builds read out of bounds -> SIGSEGV.

The trigger is simply: a locked joint, plus a contact involving World-body
geometry (e.g. a free body resting on a ground half space). */

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

GTEST_TEST(JointLockingWorldGeometrySegfault, Repro) {
  systems::DiagramBuilder<double> builder;
  auto items = AddMultibodyPlantSceneGraph(&builder, /* time_step = */ 0.01);
  MultibodyPlant<double>& plant = items.plant;

  // Collision geometry on the World body (a ground half space).
  plant.RegisterCollisionGeometry(
      plant.world_body(), math::RigidTransformd(), geometry::HalfSpace(),
      "ground", geometry::ProximityProperties());
  // A free body whose collision geometry touches the ground.
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

  // Overlap the box with the ground so a contact exists, then lock the box's
  // (auto-created) floating joint.
  plant.SetFreeBodyPose(&plant_context, box,
                        math::RigidTransformd(Eigen::Vector3d(0, 0, 0.049)));
  for (JointIndex i : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(i);
    if (joint.num_velocities() == 6) joint.Lock(&plant_context);
  }

  // One discrete step evaluates contact and runs the joint-locking filter,
  // which indexes per-tree data with the World body's invalid TreeIndex:
  // DRAKE_ASSERT fires (armed) / SIGSEGV (NDEBUG) here.
  simulator.AdvanceTo(0.01);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
