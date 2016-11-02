#include "drake/examples/Atlas/atlas_plant.h"

namespace drake {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Vector4d;

AtlasPlant::AtlasPlant() {
  sys_.reset(new drake::RigidBodySystem());
  sys_->AddModelInstanceFromFile(
      GetDrakePath() + "/examples/Atlas/urdf/atlas_convex_hull.urdf",
      systems::plants::joints::kQuaternion);

  x0_ = VectorXd::Zero(sys_->getNumStates());
  SetInitialConfiguration();

  // TODO(amcastro-tri): this should not be here but there is no other way
  // to add terrain to the world right now. See #2318.
  SetUpTerrain();
}

const VectorXd& AtlasPlant::get_initial_state() const { return x0_; }

const std::shared_ptr<RigidBodyTree>& AtlasPlant::get_rigid_body_tree() const {
  return sys_->getRigidBodyTree();
}

bool AtlasPlant::isTimeVarying() const {
  return sys_->isTimeVarying();
}

size_t AtlasPlant::getNumInputs() const {
  return sys_->getNumInputs();
}

AtlasPlant::StateVector<double>
AtlasPlant::output(const double& t,
                   const StateVector<double>& x,
                   const InputVector<double>& u) const {
  return sys_->output(t, x, u);
}

AtlasPlant::StateVector<double>
AtlasPlant::dynamics(const double& t,
                     const StateVector<double>& x,
                     const InputVector<double>& u) const {
  return sys_->dynamics(t, x, u);
}

void AtlasPlant::SetInitialConfiguration() {
  RigidBodyTree* tree = sys_->getRigidBodyTree().get();
  x0_.head(tree->get_num_positions()) = tree->getZeroConfiguration();

  // Magic numbers are initial conditions used in runAtlasWalking.m.
  x0_(2) = 0.844;    // base z
  x0_(10) = 0.27;    // l_arm_shz
  x0_(11) = 0.0;     // l_leg_hpz
  x0_(12) = 0.055;   // l_leg_hpx
  x0_(13) = -0.57;   // l_leg_hpy
  x0_(14) = 1.13;    // l_leg_kny
  x0_(15) = -0.55;   // l_leg_aky
  x0_(16) = -0.055;  // l_leg_akx
  x0_(17) = -1.33;   // l_arm_shx
  x0_(18) = 2.153;   // l_arm_ely
  x0_(19) = 0.5;     // l_arm_elx
  x0_(20) = 0.0985;  // l_arm_uwy
  x0_(21) = 0.0;     // l_arm_mwx
  x0_(22) = 0.0008;  // l_arm_lwy
  x0_(23) = -0.27;   // r_arm_shz
  x0_(24) = 0.0;     // r_leg_hpz
  x0_(25) = -0.055;  // r_leg_hpx
  x0_(26) = -0.57;   // r_leg_hpy
  x0_(27) = 1.13;    // r_leg_kny
  x0_(28) = -0.55;   // r_leg_aky
  x0_(29) = 0.055;   // r_leg_akx
  x0_(30) = 1.33;    // r_arm_shx
  x0_(31) = 2.153;   // r_arm_ely
  x0_(32) = -0.5;    // r_arm_elx
  x0_(33) = 0.0985;  // r_arm_uwy
  x0_(34) = 0.0;     // r_arm_mwx
  x0_(35) = 0.0008;  // r_arm_lwy
  x0_(36) = 0.2564;  // neck_ay
}

void AtlasPlant::SetUpTerrain() {
  // TODO(amcastro-tri): move out of here when collision materials kick in.
  sys_->penetration_stiffness = 1500.0;
  sys_->penetration_damping = 150.0;
  RigidBodyTree* tree = sys_->getRigidBodyTree().get();

  // Adds a flat terrain.
  double box_width = 1000;
  double box_depth = 10;
  DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
  Isometry3d T_element_to_link = Isometry3d::Identity();
  // The top of the box is at z=0.
  T_element_to_link.translation() << 0, 0, -box_depth / 2;
  RigidBody& world = tree->world();
  Vector4d color;
  color << 0.9297, 0.7930, 0.6758, 1;
  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  tree->addCollisionElement(
      DrakeCollision::Element(geom, T_element_to_link, &world), world,
      "terrain");
  tree->updateStaticCollisionElements();
}

}  // namespace drake
