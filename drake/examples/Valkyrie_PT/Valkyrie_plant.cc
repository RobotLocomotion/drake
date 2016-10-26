#include "drake/examples/Valkyrie_PT/Valkyrie_plant.h"

namespace drake {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Vector4d;

ValkyriePlant::ValkyriePlant() {
  sys_.reset(new drake::RigidBodySystem());
  sys_->AddModelInstanceFromFile(
      GetDrakePath() + "/examples/Valkyrie_PT/val_description/urdf/valkyrie_sim_drake.urdf",
      systems::plants::joints::kRollPitchYaw);

  x0_ = VectorXd::Zero(sys_->getNumStates());
  SetInitialConfiguration();

  // TODO(amcastro-tri): this should not be here but there is no other way
  // to add terrain to the world right now. See #2318.
  //SetUpTerrain();
}

const VectorXd& ValkyriePlant::get_initial_state() const { return x0_; }

const std::shared_ptr<RigidBodyTree>& ValkyriePlant::get_rigid_body_tree() const {
  return sys_->getRigidBodyTree();
}

bool ValkyriePlant::isTimeVarying() const {
  return sys_->isTimeVarying();
}

size_t ValkyriePlant::getNumInputs() const {
  return sys_->getNumInputs();
}

ValkyriePlant::StateVector<double>
ValkyriePlant::output(const double& t,
                   const StateVector<double>& x,
                   const InputVector<double>& u) const {
  return sys_->output(t, x, u);
}

ValkyriePlant::StateVector<double>
ValkyriePlant::dynamics(const double& t,
                     const StateVector<double>& x,
                     const InputVector<double>& u) const {
  return sys_->dynamics(t, x, u);
}

void ValkyriePlant::SetInitialConfiguration() {
  RigidBodyTree* tree = sys_->getRigidBodyTree().get();
  x0_.head(tree->get_num_positions()) = tree->getZeroConfiguration();

  // Magic numbers are initial conditions used in reach_start
  /*
  reach_start = [
  0.0; //0
  0.0;
  1.025;
  0.0;
  0.0;
  0.0; //5
  0.0;
  0.0;
  0.0;
  0.0;
  0.0; //10
  0.0;
  0.30019663134302466;
  1.25;
  0.0;
  0.7853981633974483; //15
  1.571;
  0.0;
  0.0;
  0.30019663134302466;
  -1.25; //20
  0.0;
  -0.7853981633974483;
  1.571;
  0.0;
  0.0; //25
  0.0; //26
  0.0;
  -0.49;
  1.205;
  -0.71; //30
  0.0; //31
  0.0;
  0.0;
  -0.49;
  1.205;//35
  -0.71;//36
  0.0];//37
  */
  x0_(2) = 1.025; //base_z

  x0_(12) = 0.30019663134302466; //rightShoulderPitch
  x0_(13) = 1.25; // rightShoulderRoll
  x0_(15) = 0.7853981633974483; // rightElbowPitch
  x0_(16) = 1.571; // rightForearmYaw

  x0_(19) = 0.30019663134302466; // leftShoulderPitch
  x0_(20) = -1.25; // leftShoulderRoll
  x0_(22) = -0.7853981633974483; // leftElbowPitch
  x0_(23) = 1.571; // leftForearmYaw

  x0_(28) = -0.49; // rightHipPitch
  x0_(29) = 1.205; // rightKneePitch
  x0_(30) = -0.71; // rightAnklePitch

  x0_(34) = -0.49; // leftHipPitch
  x0_(35) = 1.025; // leftKneePitch
  x0_(36) = -0.71; // leftAnklePitch

}

void ValkyriePlant::SetUpTerrain() {
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
