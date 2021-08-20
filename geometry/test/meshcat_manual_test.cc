#include "drake/common/find_resource.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

/**  To test, you must manually run `bazel run //geometry:meshcat_manual_test`,
then follow the instructions on your console. */

namespace drake {
namespace geometry {

using Eigen::Vector3d;
using math::RigidTransformd;

int do_main() {
  Meshcat meshcat;

  // Turn off the background (will appear white).
  meshcat.SetProperty("/Background", "visible", false);

  meshcat.SetObject("sphere", Sphere(.25), Rgba(1.0, 0, 0, 1));
  meshcat.SetTransform("sphere", RigidTransformd(Vector3d{-2.0, 0, 0}));

  meshcat.SetObject("cylinder", Cylinder(.25, .5), Rgba(0.0, 1.0, 0, 1));
  meshcat.SetTransform("cylinder", RigidTransformd(Vector3d{-1.0, 0, 0}));

  meshcat.SetObject("ellipsoid", Ellipsoid(.25, .25, .5), Rgba(1., 0, 1, .5));
  meshcat.SetTransform("ellipsoid", RigidTransformd(Vector3d{0.0, 0, 0}));

  meshcat.SetObject("box", Box(.25, .25, .5), Rgba(0, 0, 1, 1));
  meshcat.SetTransform("box", RigidTransformd(Vector3d{1.0, 0, 0}));

  meshcat.SetObject(
      "obj", Mesh(FindResourceOrThrow(
                      "drake/systems/sensors/test/models/meshes/box.obj"),
                  .25));
  meshcat.SetTransform("obj", RigidTransformd(Vector3d{2.0, 0, 0}));

  std::cout << R"""(
Open up your browser to the URL above.

- The background should be off (white).\n";
- From left to right along the x axis, you should see:
  - a red sphere
  - a green cylinder (with the long axis in z)
  - a pink semi-transparent ellipsoid (long axis in z)
  - a blue box (long axis in z)
  - a gray cube (which will be textured once we add texture support)
)""";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  meshcat.Delete("box");

  std::cout << "- The blue box should have disappeared." << std::endl;
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  meshcat.Delete();
  std::cout << "- Everything else should have disappeared." << std::endl;

  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::cout << "All done." << std::endl;

  return 0;
}

}  // namespace geometry
}  // namespace drake

int main() { return drake::geometry::do_main(); }
