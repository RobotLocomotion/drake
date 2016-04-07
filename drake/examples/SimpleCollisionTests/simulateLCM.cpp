
#include "LCMSystem.h"
#include "RigidBodySystem.h"
#include "LinearSystem.h"
#include "BotVisualizer.h"
#include "drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_control_cmd_t.hpp"
#include "drake/systems/plants/shapes/HeightMapTerrain.h"

using std::sin;
using std::string;
using namespace Eigen;
using namespace Drake;

// Review commments
//1. Fix these namespaces inclusions for varibles in Eigen and son on ...
//2. Add description for HeightMapTerrain constructors explaining why you need a name
//3. Change member names to use the trailing underscore as in Google's style guide
//4. Lets ask Russ why we have two containers with AND without the margins
//5. Use a unique_ptr in m_rawHeightfieldData
//6. Make HeightMapTerrain::clone() = 0? just make sure child MUST implement this method.
//7. Replace virtual's with override's where needed (specially within HeightMap but leave other shapes untouched for now, mayb another Issue?)
//8. Use Google's style guide for types: eg.: use "using std::sin;" at the top of the file or "std::string" elsewhere when needed?
//   See SinusoidalTerrain constructror as an example where a "string&" is passed as an argument
//9. Use Sherm's "non-virtual methods" (is that what he called them?) design to avoid having the user calling "this->computeMinMaxHeights" within
//   SinusoidalTerrain's constructor

class SinusoidalTerrain
    : public DrakeShapes::HeightMapTerrain {
 public:
  SinusoidalTerrain(const string& name, const Eigen::Vector2i& ncells,
                    const Eigen::Vector2d& size, double height = 1.0)
      : HeightMapTerrain(name, ncells, size), m_height(height) {
    for (int i = 0; i < nnodes(0); ++i) {
      double x = i*delta_ell(0);
      for (int j = 0; j < nnodes(1); ++j) {
        double y = j*delta_ell(1);
        double z = m_height*sin(x*3.1416/size(0));
        cellValue(i,j) = z;
      }
    }
    this->computeMinMaxHeights();
  }
  SinusoidalTerrain(const SinusoidalTerrain& other): HeightMapTerrain(other){}
  SinusoidalTerrain *clone() const {
    return new SinusoidalTerrain(*this);
  }
  protected:
    double m_height;
};

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " (world_urdf) -terrain (flat -angle <angle>|height_map -x <x position>) " << std::endl;
    std::cerr << "With: " << std::endl;
    std::cerr << " <angle> in degrees" << std::endl;
    std::cerr << " <x position> in sphere diameters" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Flat terrain example: "  << argv[0] << " ball_world.urdf -terrain flat -angle 45" << std::endl;
    std::cerr << "Heigth map example  : "  << argv[0] << " ball_world.urdf -terrain height_map -x -1.0" << std::endl;
    return 1;
  }

  //Retrieve command line options
  bool flat_terrain = true; //run flat terrain test by default
  double angle = 0.0; //zero angle terrain test is run by default
  double x_pos = 0.0; //flat terrain located at x=0 by default
  {
    char* terrain_type = getCommandLineOption(argv, argv + argc, "-terrain");
    if (terrain_type){
      if(strcmp(terrain_type, "height_map")==0){
        flat_terrain = false;
        char* x_str = getCommandLineOption(argv, argv + argc, "-x");
        if (x_str){
          x_pos = std::stod(std::string(x_str));
        }
      }else if(strcmp(terrain_type, "flat")==0){
        char* angle_str = getCommandLineOption(argv, argv + argc, "-angle");
        if (angle_str){
          angle = std::stod(std::string(angle_str));
        }
      }else{
        throw std::runtime_error("Invalid terrain type option.");
      }
    }

    if(flat_terrain){
      cout << "Running flat terrain test: " << endl;
      cout << "  angle = " << angle << endl;
    }else{
      cout << "Running height map test:" << endl;
      cout << "  x position = " << x_pos << endl;
    }
  }

  // todo: consider moving this logic into the RigidBodySystem class so it can
  // be reused
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;

  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(argv[1], floating_base_type);
  rigid_body_sys->use_multi_contact = false;
  auto const & tree = rigid_body_sys->getRigidBodyTree();

  if (flat_terrain) {  // add flat terrain
    double box_width = 10;
    double box_depth = 1;
    angle = angle*M_PI/180.0;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    Matrix3d m;
    m = AngleAxisd(angle,  Vector3d::UnitY());
    //Vector3d center(0, 0, -1.5);
    T_element_to_link.linear() = m;
    //This makes the impact to happen when the sphere is at z=-0.5 regardless of the angle. Just useful for debugging.
    T_element_to_link.translation() = m*Vector3d(0.0,0.0,-1.0) + Vector3d(0.0,0.0,-0.5);
    auto& world = tree->bodies[0]; //world is a body with zero mass and zero moment of inertia
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;
    world->addVisualElement(DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(RigidBody::CollisionElement(geom, T_element_to_link, world), *world, "terrain");
    tree->updateStaticCollisionElements();
  }else
  {//Add Height map
    double terrain_width = 10;
    int terrain_ncells = 16;
    //DrakeShapes::FlatTerrain terrain_geom("terrain",Vector2i(terrain_ncells,terrain_ncells),Vector2d(terrain_width, terrain_width),-M_PI/4.0);
    SinusoidalTerrain terrain_geom("terrain",Vector2i(terrain_ncells,terrain_ncells),Vector2d(terrain_width, terrain_width));

    Isometry3d T_element_to_link = Isometry3d::Identity();
    //T_element_to_link.translation() << -terrain_width/2.0, -terrain_width/2.0, 0.0;
    T_element_to_link.translation() << x_pos, 0.0, -2.0;

    auto& world = tree->bodies[0];

    Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;

    world->addVisualElement(DrakeShapes::VisualElement(terrain_geom, T_element_to_link, color));

    tree->addCollisionElement(RigidBody::CollisionElement(terrain_geom, T_element_to_link, world), *world, "terrain");
    tree->updateStaticCollisionElements();
  }

  //tree->drawKinematicTree("graphiviz_test.dot"); //Convert to png image file: dot -Tpng graphiviz_test.dot -o graphiviz_test.png

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  //this replaces the above commented out code with the "auto sys = cascade(vehicle_sys, visualizer);" at the end
  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);
  auto sys = cascade(rigid_body_sys, visualizer);

  SimulationOptions options = default_simulation_options;

  //Only the penetration stiffness is non-zero for these tests in order
  //to isolate the numerics for collision only.
  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping = 0.0;
  rigid_body_sys->friction_coefficient = 0.0;
  options.initial_step_size = 1.0e-3;
  options.timeout_seconds = numeric_limits<double>::infinity();

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();

  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), x0, options);

  return 0;
}
