#include "drake/common/find_resource.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

/* To test, you must manually run `bazel run //geometry:meshcat_manual_test`,
then follow the instructions on your console. */

namespace drake {
namespace geometry {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

int do_main() {
  auto meshcat = std::make_shared<Meshcat>();

  meshcat->SetObject("sphere", Sphere(.25), Rgba(1.0, 0, 0, 1));
  meshcat->SetTransform("sphere", RigidTransformd(Vector3d{-3, 0, 0}));

  meshcat->SetObject("cylinder", Cylinder(.25, .5), Rgba(0.0, 1.0, 0, 1));
  meshcat->SetTransform("cylinder", RigidTransformd(Vector3d{-2, 0, 0}));

  meshcat->SetObject("ellipsoid", Ellipsoid(.25, .25, .5), Rgba(1., 0, 1, .5));
  meshcat->SetTransform("ellipsoid", RigidTransformd(Vector3d{-1, 0, 0}));

  meshcat->SetObject("box", Box(.25, .25, .5), Rgba(0, 0, 1, 1));
  meshcat->SetTransform("box", RigidTransformd(Vector3d{0, 0, 0}));

  // The green color of this cube comes from the texture map.
  meshcat->SetObject(
      "obj", Mesh(FindResourceOrThrow(
                      "drake/systems/sensors/test/models/meshes/box.obj"),
                  .25));
  meshcat->SetTransform("obj", RigidTransformd(Vector3d{1, 0, 0}));

  meshcat->SetObject(
      "mustard",
      Mesh(FindResourceOrThrow("drake/manipulation/models/ycb/meshes/"
                               "006_mustard_bottle_textured.obj"), 3.0));
  meshcat->SetTransform("mustard", RigidTransformd(Vector3d{2, 0, 0}));

  const int kPoints = 100000;
  perception::PointCloud cloud(
      kPoints, perception::pc_flags::kXYZs | perception::pc_flags::kRGBs);
  Eigen::Matrix3Xf m = Eigen::Matrix3Xf::Random(3, kPoints);
  cloud.mutable_xyzs() = Eigen::DiagonalMatrix<float, 3>{.25, .25, .5} * m;
  cloud.mutable_rgbs() = (255.0 * (m.array() + 1.0) / 2.0).cast<uint8_t>();
  meshcat->SetObject("point_cloud", cloud, 0.01);
  meshcat->SetTransform("point_cloud", RigidTransformd(Vector3d{3, 0, 0}));
  std::cout << R"""(
Open up your browser to the URL above.

- The background should be grey.
- From left to right along the x axis, you should see:
  - a red sphere
  - a green cylinder (with the long axis in z)
  - a pink semi-transparent ellipsoid (long axis in z)
  - a blue box (long axis in z)
  - a bright green cube (the green comes from a texture map)
  - a yellow mustard bottle w/ label
  - a dense rainbow point cloud in a box (long axis in z)
)""";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Animations:\n";
  MeshcatAnimation animation;
  std::cout << "- the red sphere should move up and down in z.\n";
  animation.SetTransform(0, "sphere", RigidTransformd(Vector3d{-3, 0, 0}));
  animation.SetTransform(20, "sphere", RigidTransformd(Vector3d{-3, 0, 1}));
  animation.SetTransform(40, "sphere", RigidTransformd(Vector3d{-3, 0, 0}));

  std::cout << "- the blue box should spin clockwise about the +z axis.\n";
  animation.SetTransform(0, "box",
                         RigidTransformd(RotationMatrixd::MakeZRotation(0)));
  animation.SetTransform(20, "box",
                         RigidTransformd(RotationMatrixd::MakeZRotation(M_PI)));
  animation.SetTransform(
      40, "box", RigidTransformd(RotationMatrixd::MakeZRotation(2 * M_PI)));
  animation.set_repetitions(4);
  meshcat->SetAnimation(animation);

  // TODO(russt): Do all of these in a single animation pending resolution of
  // https://github.com/rdeits/meshcat/issues/105
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  MeshcatAnimation animation2;
  animation2.SetProperty(0, "cylinder", "visible", true);
  animation2.SetProperty(20, "cylinder", "visible", false);
  animation2.SetProperty(40, "cylinder", "visible", true);
  animation2.set_repetitions(4);
  meshcat->SetAnimation(animation2);

  std::cout << "- the green cylinder should appear and disappear.\n";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  MeshcatAnimation animation3;
  animation3.SetProperty(0, "ellipsoid/<object>", "material.opacity", 1.0);
  animation3.SetProperty(20, "ellipsoid/<object>", "material.opacity", 0.0);
  animation3.SetProperty(40, "ellipsoid/<object>", "material.opacity", 1.0);
  animation3.set_repetitions(4);
  meshcat->SetAnimation(animation3);

  std::cout
      << "- the pink ellipsoid should get less and then more transparent.\n";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


  meshcat->Set2dRenderMode(math::RigidTransform(Eigen::Vector3d{0, -3, 0}), -4,
                           4, -2, 2);

  std::cout << "- The scene should have switched to 2D rendering mode.\n";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  meshcat->Set2dRenderMode(
      math::RigidTransform(math::RotationMatrixd::MakeZRotation(-M_PI / 2.0),
                           Eigen::Vector3d{-3, 0, 0}),
      -2, 2, -2, 2);

  std::cout << "- Now 2D rendering along the +x axis (red sphere in front).\n";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "- The scene should have switched back to 3D.\n";
  meshcat->ResetRenderMode();

  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  // Turn off the background (it will appear white).
  meshcat->SetProperty("/Background", "visible", false);

  meshcat->Delete("box");
  meshcat->SetProperty("/Lights/AmbientLight/<object>", "intensity", 0.1);

  std::cout
      << "- The blue box should have disappeared\n"
      << "- The lights should have dimmed.\n"
      << "- The background should have been disabled (it will appear white)"
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
  params.delete_on_initialization_event = false;
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

  std::cout
      << "Now we'll run the simulation (you should see the robot fall down)."
      << std::endl;

  systems::Simulator<double> simulator(*diagram, std::move(context));
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(4.0);

  meshcat->AddButton("ButtonTest");
  meshcat->AddSlider("SliderTest", 0, 1, 0.01, 0.5);

  std::cout << "I've added a button and a slider to the controls menu.\n";
  std::cout << "- Click the ButtonTest button a few times.\n";
  std::cout << "- Move SliderTest slider.\n";
  std::cout << "- Open a second browser (" << meshcat->web_url()
            << ") and confirm that moving the slider in one updates the slider "
               "in the other.\n";

  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Got " << meshcat->GetButtonClicks("ButtonTest")
            << " clicks on ButtonTest.\n"
            << "Got " << meshcat->GetSliderValue("SliderTest")
            << " value for SliderTest." << std::endl;

  std::cout << "Exiting..." << std::endl;
  return 0;
}

}  // namespace geometry
}  // namespace drake

int main() { return drake::geometry::do_main(); }
