
#include <iostream>

#include "drake/Path.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"

using namespace std;
using namespace Drake;
using namespace Eigen;

int main(int argc, char* argv[]) {
  SimulationOptions options;

  // Get the final time of the simulation.
  double final_time =
      argc >= 2 ? atof(argv[1]) : std::numeric_limits<double>::infinity();

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if (!lcm->good()) return 1;

  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(
      getDrakePath() + "/examples/Atlas/urdf/atlas_convex_hull.urdf",
      DrakeJoint::QUATERNION);
  auto const& tree = rigid_body_sys->getRigidBodyTree();

  rigid_body_sys->penetration_stiffness = 1500.0;
  rigid_body_sys->penetration_damping = 150.0;
  options.initial_step_size = 5e-5;
  options.realtime_factor = 0.0;

  {  // add flat terrain
    double box_width = 1000;
    double box_depth = 10;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0,
        -box_depth / 2;  // top of the box is at z=0
    auto& world = tree->bodies[0];
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world->addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        RigidBody::CollisionElement(geom, T_element_to_link, world), *world,
        "terrain");
    tree->updateStaticCollisionElements();
  }

  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
  // magic numbers are initial conditions used in runAtlasWalking.m
  x0(2) = 0.844;    // base z
  x0(10) = 0.27;    // l_arm_shz
  x0(11) = 0.0;     // l_leg_hpz
  x0(12) = 0.055;   // l_leg_hpx
  x0(13) = -0.57;   // l_leg_hpy
  x0(14) = 1.13;    // l_leg_kny
  x0(15) = -0.55;   // l_leg_aky
  x0(16) = -0.055;  // l_leg_akx
  x0(17) = -1.33;   // l_arm_shx
  x0(18) = 2.153;   // l_arm_ely
  x0(19) = 0.5;     // l_arm_elx
  x0(20) = 0.0985;  // l_arm_uwy
  x0(21) = 0.0;     // l_arm_mwx
  x0(22) = 0.0008;  // l_arm_lwy
  x0(23) = -0.27;   // r_arm_shz
  x0(24) = 0.0;     // r_leg_hpz
  x0(25) = -0.055;  // r_leg_hpx
  x0(26) = -0.57;   // r_leg_hpy
  x0(27) = 1.13;    // r_leg_kny
  x0(28) = -0.55;   // r_leg_aky
  x0(29) = 0.055;   // r_leg_akx
  x0(30) = 1.33;    // r_arm_shx
  x0(31) = 2.153;   // r_arm_ely
  x0(32) = -0.5;    // r_arm_elx
  x0(33) = 0.0985;  // r_arm_uwy
  x0(34) = 0.0;     // r_arm_mwx
  x0(35) = 0.0008;  // r_arm_lwy
  x0(36) = 0.2564;  // neck_ay

  auto sys_with_vis = cascade(rigid_body_sys, visualizer);

  runLCM(sys_with_vis, lcm, 0, final_time, x0, options);
}
