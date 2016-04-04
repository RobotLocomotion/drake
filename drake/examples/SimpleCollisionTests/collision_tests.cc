#include "collision_tests.h"

#include "LCMSystem.h"
#include "RigidBodySystem.h"
#include "LinearSystem.h"
#include "BotVisualizer.h"
//#include "drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_control_cmd_t.hpp"
#include "drake/systems/plants/shapes/HeightMapTerrain.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

collisionTests::collisionTests(){
  flat_terrain = true;
  angle = 45.0; //in degrees
  x_pos = 0.0;  //in sphere diamters
  urdf_file = "ball_world.urdf";
  t_final = 0.6;
  with_visualizer = false;
}

collisionTests::collisionTests(const char* c_fname,bool flat_terrain,double angle, double x_pos,double t_final,double delt, bool with_visualizer):
  flat_terrain(flat_terrain),angle(angle),x_pos(x_pos),urdf_file(std::string(c_fname)),t_final(t_final),delt(delt),with_visualizer(with_visualizer)
{}

collisionTests& collisionTests::set_time_step(double dt){
  delt = dt;
  return *this;
}

void collisionTests::run(){
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;

//auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(urdf_file.c_str(), floating_base_type);
  rigid_body_sys->use_multi_contact = false;
  auto const& tree = rigid_body_sys->getRigidBodyTree();

  if (flat_terrain) {  // add flat terrain
    double box_width = 10;
    double box_depth = 1;
    angle = angle * M_PI / 180.0;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    Matrix3d m;
    m = AngleAxisd(angle, Vector3d::UnitY());
    T_element_to_link.linear() = m;
    // This makes the impact to happen when the sphere is at z=-0.5 regardless
    // of the angle. Just useful for debugging.
    T_element_to_link.translation() =
      m * Vector3d(0.0, 0.0, -1.0) + Vector3d(0.0, 0.0, -0.5);
    auto& world = tree->bodies[0];  // world is a body with zero mass and zero
                                    // moment of inertia
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;
    world->addVisualElement(
			    DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
			      RigidBody::CollisionElement(geom, T_element_to_link, world), *world,
			      "terrain");
    tree->updateStaticCollisionElements();
  } else {  // Add Height map
    double terrain_width = 10;
    int terrain_ncells = 16;
    // DrakeShapes::FlatTerrain
    // terrain_geom("terrain",Vector2i(terrain_ncells,terrain_ncells),Vector2d(terrain_width,
    // terrain_width),-M_PI/4.0);
    SinusoidalTerrain terrain_geom("terrain",
                                   Vector2i(terrain_ncells, terrain_ncells),
                                   Vector2d(terrain_width, terrain_width));

    Isometry3d T_element_to_link = Isometry3d::Identity();
    // T_element_to_link.translation() << -terrain_width/2.0,
    // -terrain_width/2.0, 0.0;
    T_element_to_link.translation() << x_pos, 0.0, -2.0;

    auto& world = tree->bodies[0];

    Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;

    world->addVisualElement(
			    DrakeShapes::VisualElement(terrain_geom, T_element_to_link, color));

    tree->addCollisionElement(
			      RigidBody::CollisionElement(terrain_geom, T_element_to_link, world),
			      *world, "terrain");
    tree->updateStaticCollisionElements();
  }

  // tree->drawKinematicTree("graphiviz_test.dot"); //Convert to png image file:
  // dot -Tpng graphiviz_test.dot -o graphiviz_test.png

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  // this replaces the above commented out code with the "auto sys =
  // cascade(vehicle_sys, visualizer);" at the end
  auto visualizer =
    make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);
  auto sys = cascade(rigid_body_sys, visualizer);

  SimulationOptions options = default_simulation_options;
  rigid_body_sys->penetration_stiffness = 50000.0;
  rigid_body_sys->penetration_damping =
    0.0;  // rigid_body_sys->penetration_stiffness / 10.0;
  // rigid_body_sys->friction_coefficient = 10.0;  // essentially infinite
  // friction. Causes ball to rotate
  rigid_body_sys->friction_coefficient = 0.0;
  options.initial_step_size = delt;
  options.timeout_seconds = numeric_limits<double>::infinity();
  options.realtime_factor = 1.0;

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();

  x = x0;

  if(with_visualizer){
    runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), x0, options);
  }else{
    simulate(*rigid_body_sys, 0.0, t_final, x0, options,x);
  }

}

double collisionTests::get_solution(){
  //cout << " x = " << x.transpose() << endl;
  return x(2);
}
