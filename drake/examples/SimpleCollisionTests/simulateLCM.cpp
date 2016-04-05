
#include "collision_tests.h"
#include "drakeAppUtil.h"

#include <iostream>
using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " (world_urdf) -terrain (flat -angle "
                                         "<angle>|height_map -x <x position>) "
              << std::endl;
    std::cerr << "With: " << std::endl;
    std::cerr << " <angle> in degrees" << std::endl;
    std::cerr << " <x position> in sphere diameters" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Flat terrain example: " << argv[0]
              << " ball_world.urdf -terrain flat -angle 45" << std::endl;
    std::cerr << "Heigth map example  : " << argv[0]
              << " ball_world.urdf -terrain height_map -x -1.0" << std::endl;
    return 1;
  }

  // Retrieve command line options
  bool flat_terrain = true;  // run flat terrain test by default
  double angle = 0.0;  // zero angle terrain test is run by default
  double x_pos = 0.0;  // flat terrain located at x=0 by default
  {
    char* terrain_type = getCommandLineOption(argv, argv + argc, "-terrain");
    if (terrain_type) {
      if (strcmp(terrain_type, "height_map") == 0) {
        flat_terrain = false;
        char* x_str = getCommandLineOption(argv, argv + argc, "-x");
        if (x_str) {
          x_pos = std::stod(std::string(x_str));
        }
      } else if (strcmp(terrain_type, "flat") == 0) {
        char* angle_str = getCommandLineOption(argv, argv + argc, "-angle");
        if (angle_str) {
          angle = std::stod(std::string(angle_str));
        }
      } else {
        throw std::runtime_error("Invalid terrain type option.");
      }
    }

    if (flat_terrain) {
      cout << "Running flat terrain test: " << endl;
      cout << "  angle = " << angle << endl;
    } else {
      cout << "Running height map test:" << endl;
      cout << "  x position = " << x_pos << endl;
    }
  }

  collisionTests ball_test(argv[1],flat_terrain,angle,x_pos,std::numeric_limits<double>::infinity(),1.0e-3);
  ball_test.run();

  return 0;
}
