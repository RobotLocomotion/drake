#include "drake/common/find_resource.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

/**  To test, you must manually run `bazel run //geometry:meshcat_manual_test`,
then follow the instructions on your console. */

namespace drake {
namespace geometry {

using Eigen::Vector3d;
using math::RigidTransformd;

int do_main() {
  auto meshcat = std::make_shared<Meshcat>();

  // Turn off the background (it will appear white).
  meshcat->SetProperty("/Background", "visible", false);

  meshcat->SetObject("sphere", Sphere(.25), Rgba(1.0, 0, 0, 1));
  meshcat->SetTransform("sphere", RigidTransformd(Vector3d{-2.0, 0, 0}));

  meshcat->SetObject("cylinder", Cylinder(.25, .5), Rgba(0.0, 1.0, 0, 1));
  meshcat->SetTransform("cylinder", RigidTransformd(Vector3d{-1.0, 0, 0}));

  meshcat->SetObject("ellipsoid", Ellipsoid(.25, .25, .5), Rgba(1., 0, 1, .5));
  meshcat->SetTransform("ellipsoid", RigidTransformd(Vector3d{0.0, 0, 0}));

  meshcat->SetObject("box", Box(.25, .25, .5), Rgba(0, 0, 1, 1));
  meshcat->SetTransform("box", RigidTransformd(Vector3d{1.0, 0, 0}));

  meshcat->SetObject(
      "obj", Mesh(FindResourceOrThrow(
                      "drake/systems/sensors/test/models/meshes/box.obj"),
                  .25));
  meshcat->SetTransform("obj", RigidTransformd(Vector3d{2.0, 0, 0}));

  std::cout << R"""(
Open up your browser to the URL above.

- The background should be off (it will appear white).\n";
- From back to front along the x axis, you should see:
  - a red sphere
  - a green cylinder (with the long axis in z)
  - a pink semi-transparent ellipsoid (long axis in z)
  - a blue box (long axis in z)
  - a gray cube (which will be textured once we add texture support)
)""";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  meshcat->Delete("box");
  meshcat->SetProperty("/Lights/AmbientLight/<object>", "intensity", 0.1);

  std::cout << "- The blue box should have disappeared and the lights should "
               "have dimmed."
            << std::endl;
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  meshcat->Delete();
  std::cout << "- Everything else should have disappeared." << std::endl;

  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  meshcat->SetProperty("/Lights/AmbientLight/<object>", "intensity", 0.6);
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
  multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_no_collision.urdf"));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base").body_frame());
  plant.Finalize();

  builder.ExportInput(plant.get_actuation_input_port(), "actuation_input");
  MeshcatVisualizerParams params;
  params.delete_prefix_on_initialization_event = false;
  MeshcatVisualizerd::AddToBuilder(&builder, scene_graph, meshcat, params);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  diagram->get_input_port().FixValue(context.get(), Eigen::VectorXd::Zero(7));

  diagram->Publish(*context);
  std::cout
      << "- Now you should see a kuka model (from MultibodyPlant/SceneGraph)"
      << std::endl;

  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Finally we'll run the simulation (you should see the robot "
               "fall down) and exit."
            << std::endl;

  systems::Simulator<double> simulator(*diagram, std::move(context));
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(4.0);

  std::cout << "Exiting..." << std::endl;
  return 0;
}

}  // namespace geometry
}  // namespace drake

int main() { return drake::geometry::do_main(); }
