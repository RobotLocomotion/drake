#include "cylinder_system.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

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
  penetration_stiffness = 5000.0;
  penetration_damping =
      penetration_stiffness / 10.0;
  friction_coefficient = 10.0;  // essentially infinite friction

  {  // Adds a flat terrain.
#if 0
    double box_width = 100;
    double box_depth = 1.0;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    // The top of the box is at z=0.
    T_element_to_link.linear() = Eigen::AngleAxisd(0.2*M_PI/180, Eigen::Vector3d::UnitY()).toRotationMatrix();
    T_element_to_link.translation() << 0, 0, -box_depth / 2;
#endif

    DrakeShapes::Mesh geom("DrivingSurface_simpler_allTris.obj", "/home/amcastro/Documents/NvidiaTerrain/PaloAltoLoop.obj");
    Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    T_element_to_link.linear() = Eigen::AngleAxisd(0.2*M_PI/180, Eigen::Vector3d::UnitY()).toRotationMatrix()*Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    //T_element_to_link.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix()*Eigen::AngleAxisd(-0.2*M_PI/180, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    PRINT_VAR("T_element_to_link.linear():");
    std::cout << T_element_to_link.linear() << std::endl;

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

