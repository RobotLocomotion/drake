#include "cylinder_system.h"

namespace drake {

CylinderSystem::CylinderSystem() {
  addRobotFromFile(
      Drake::getDrakePath() + "/examples/BouncingCylinder/cylinder.sdf",
      DrakeJoint::QUATERNION);
  tree_ = getRigidBodyTree().get();

  SetZeroConfiguration();

  // TODO(amcastro-tri): this should not be here but there is no other way
  // to add terrain to the world right now. See #2318.
  SetUpTerrain();
}

const VectorXd& CylinderSystem::get_initial_state() const { return x0_; }

void CylinderSystem::SetZeroConfiguration() {
  x0_ = VectorXd::Zero(getNumStates());
  x0_.head(tree_->number_of_positions()) = tree_->getZeroConfiguration();
}

void CylinderSystem::SetUpTerrain() {
  // TODO(amcastro-tri): move out of here when collision materials kick in.
  penetration_stiffness = 150.0;
  penetration_damping = penetration_stiffness/10.0;

  {  // Adds a flat terrain.
    double box_width = 5;
    double box_depth = 0.5;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    // The top of the box is at z=0.
    T_element_to_link.translation() << 0, 0, -box_depth / 2;
    RigidBody& world = tree_->world();
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;
    world.addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree_->addCollisionElement(
        RigidBody::CollisionElement(geom, T_element_to_link, &world), world,
        "terrain");
    tree_->updateStaticCollisionElements();
  }
}

} // namespace drake

