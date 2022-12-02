#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
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

  Vector3d sphere_home{-4, 0, 0};
  meshcat->SetObject("sphere", Sphere(.25), Rgba(1.0, 0, 0, 1));
  meshcat->SetTransform("sphere", RigidTransformd(sphere_home));

  meshcat->SetObject("cylinder", Cylinder(.25, .5), Rgba(0.0, 1.0, 0, 1));
  meshcat->SetTransform("cylinder", RigidTransformd(Vector3d{-3, 0, 0}));

  meshcat->SetObject("ellipsoid", Ellipsoid(.25, .25, .5), Rgba(1., 0, 1, .5));
  meshcat->SetTransform("ellipsoid", RigidTransformd(Vector3d{-2, 0, 0}));

  Vector3d box_home{-1, 0, 0};
  meshcat->SetObject("box", Box(.25, .25, .5), Rgba(0, 0, 1, 1));
  meshcat->SetTransform("box", RigidTransformd(box_home));

  meshcat->SetObject("capsule", Capsule(.25, .5), Rgba(0, 1, 1, 1));
  meshcat->SetTransform("capsule", RigidTransformd(Vector3d{0, 0, 0}));

  // Note that height (in z) is the first argument.
  meshcat->SetObject("cone", MeshcatCone(.5, .25, .5), Rgba(1, 0, 0, 1));
  meshcat->SetTransform("cone", RigidTransformd(Vector3d{1, 0, 0}));

  // The green color of this cube comes from the texture map.
  meshcat->SetObject(
      "obj", Mesh(FindResourceOrThrow(
                      "drake/geometry/render/test/meshes/box.obj"),
                  .25));
  meshcat->SetTransform("obj", RigidTransformd(Vector3d{2, 0, 0}));

  meshcat->SetObject(
      "mustard",
      Mesh(FindResourceOrThrow("drake/manipulation/models/ycb/meshes/"
                               "006_mustard_bottle_textured.obj"), 3.0));
  meshcat->SetTransform("mustard", RigidTransformd(Vector3d{3, 0, 0}));

  {
    const int kPoints = 100000;
    perception::PointCloud cloud(
        kPoints, perception::pc_flags::kXYZs | perception::pc_flags::kRGBs);
    Eigen::Matrix3Xf m = Eigen::Matrix3Xf::Random(3, kPoints);
    cloud.mutable_xyzs() = Eigen::DiagonalMatrix<float, 3>{.25, .25, .5} * m;
    cloud.mutable_rgbs() = (255.0 * (m.array() + 1.0) / 2.0).cast<uint8_t>();
    meshcat->SetObject("point_cloud", cloud, 0.01);
    meshcat->SetTransform("point_cloud", RigidTransformd(Vector3d{4, 0, 0}));
  }

  {
    Eigen::Matrix3Xd vertices(3, 200);
    Eigen::RowVectorXd t = Eigen::RowVectorXd::LinSpaced(200, 0, 10 * M_PI);
    vertices << .25 * t.array().sin(), .25 * t.array().cos(), t / (10 * M_PI);
    meshcat->SetLine("line", vertices, 3.0, Rgba(0, 0, 1, 1));
    meshcat->SetTransform("line", RigidTransformd(Vector3d{5, 0, -.5}));
  }

  {
    Eigen::Matrix3Xd start(3, 4), end(3, 4);
    // clang-format off
    start << -.1, -.1,  .1, .1,
            -.1,  .1, -.1, .1,
            0, 0, 0, 0;
    // clang-format on
    end = start;
    end.row(2) = Eigen::RowVector4d::Ones();
    meshcat->SetLineSegments("line_segments", start, end, 5.0,
                             Rgba(0, 1, 0, 1));
    meshcat->SetTransform("line_segments",
                          RigidTransformd(Vector3d{6, 0, -.5}));
  }

  // The TriangleSurfaceMesh variant of SetObject calls SetTriangleMesh(), so
  // visually inspecting the results of TriangleSurfaceMesh is sufficient here.
  {
    const int face_data[2][3] = {{0, 1, 2}, {2, 3, 0}};
    std::vector<SurfaceTriangle> faces;
    for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
    const Vector3d vertex_data[4] = {
        {0, 0, 0}, {0.5, 0, 0}, {0.5, 0.5, 0}, {0, 0.5, 0.5}};
    std::vector<Vector3d> vertices;
    for (int v = 0; v < 4; ++v) vertices.emplace_back(vertex_data[v]);
    TriangleSurfaceMesh<double> surface_mesh(
        std::move(faces), std::move(vertices));
    meshcat->SetObject("triangle_mesh", surface_mesh, Rgba(.9, 0, .9, 1.0));
    meshcat->SetTransform("triangle_mesh",
                          RigidTransformd(Vector3d{6.75, -.25, 0}));

    meshcat->SetObject("triangle_mesh_wireframe", surface_mesh,
                       Rgba(.9, 0, .9, 1.0), true, 5.0);
    meshcat->SetTransform("triangle_mesh_wireframe",
                          RigidTransformd(Vector3d{7.75, -.25, 0}));
  }

  // SetTriangleColorMesh.
  {
    // clang-format off
    Eigen::Matrix3Xd vertices(3, 4);
    vertices <<
      0, 0.5, 0.5, 0,
      0, 0,   0.5, 0.5,
      0, 0,   0,   0.5;
    Eigen::Matrix3Xi faces(3, 2);
    faces <<
      0, 2,
      1, 3,
      2, 0;
    Eigen::Matrix3Xd colors(3, 4);
    colors <<
      1, 0, 0, 1,
      0, 1, 0, 1,
      0, 0, 1, 0;
    // clang-format on
    meshcat->SetTriangleColorMesh("triangle_color_mesh", vertices, faces,
                                  colors);
    meshcat->SetTransform("triangle_color_mesh",
                          RigidTransformd(Vector3d{8.75, -.25, 0}));
  }

  std::cout << R"""(
Open up your browser to the URL above.

- The background should be grey.
- From left to right along the x axis, you should see:
  - a red sphere
  - a green cylinder (with the long axis in z)
  - a pink semi-transparent ellipsoid (long axis in z)
  - a blue box (long axis in z)
  - a teal capsule (long axis in z)
  - a red cone (expanding in +z, twice as wide in y than in x)
  - a bright green cube (the green comes from a texture map)
  - a yellow mustard bottle w/ label
  - a dense rainbow point cloud in a box (long axis in z)
  - a blue line coiling up (in z).
  - 4 green vertical line segments (in z).
  - a purple triangle mesh with 2 faces.
  - the same purple triangle mesh drawn as a wireframe.
  - the same triangle mesh drawn in multicolor.
)""";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Calling meshcat.Flush(), which will block until all clients "
               "have received all the data)...";
  meshcat->Flush();
  std::cout << "Done." << std::endl;

  std::cout << "Animations:\n";
  MeshcatAnimation animation;
  std::cout << "- the red sphere should move up and down in z.\n";
  animation.SetTransform(0, "sphere", RigidTransformd(sphere_home));
  animation.SetTransform(20, "sphere", RigidTransformd(sphere_home +
                                                       Vector3d::UnitZ()));
  animation.SetTransform(40, "sphere", RigidTransformd(sphere_home));

  std::cout << "- the blue box should spin clockwise about the +z axis.\n";
  animation.SetTransform(0, "box",
                         RigidTransformd(RotationMatrixd::MakeZRotation(0),
                                         box_home));
  animation.SetTransform(20, "box",
                         RigidTransformd(RotationMatrixd::MakeZRotation(M_PI),
                                         box_home));
  animation.SetTransform(
      40, "box", RigidTransformd(RotationMatrixd::MakeZRotation(2 * M_PI),
                                 box_home));
  animation.set_repetitions(4);

  std::cout << "- the green cylinder should appear and disappear.\n";
  animation.SetProperty(0, "cylinder", "visible", true);
  animation.SetProperty(20, "cylinder", "visible", false);
  animation.SetProperty(40, "cylinder", "visible", true);
  animation.set_repetitions(4);

  std::cout
      << "- the pink ellipsoid should get less and then more transparent.\n";
  animation.SetProperty(0, "ellipsoid/<object>", "material.opacity", 1.0);
  animation.SetProperty(20, "ellipsoid/<object>", "material.opacity", 0.0);
  animation.SetProperty(40, "ellipsoid/<object>", "material.opacity", 1.0);
  animation.set_repetitions(4);

  meshcat->SetAnimation(animation);

  std::cout << "You can review/replay the animation from the controls menu.\n";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  meshcat->Set2dRenderMode(math::RigidTransform(Vector3d{0, -3, 0}), -4,
                           4, -2, 2);

  std::cout << "- The scene should have switched to 2D rendering mode.\n";
  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  meshcat->Set2dRenderMode(
      math::RigidTransform(math::RotationMatrixd::MakeZRotation(-M_PI / 2.0),
                           sphere_home),
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

  {
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

    multibody::Parser parser(&plant);

    // Add the hydroelastic spheres and joints between them.
    const std::string hydro_sdf =
        FindResourceOrThrow("drake/multibody/meshcat/test/hydroelastic.sdf");
    parser.AddModels(hydro_sdf);
    const auto& body1 = plant.GetBodyByName("body1");
    plant.AddJoint<multibody::PrismaticJoint>("body1", plant.world_body(),
                                              std::nullopt, body1, std::nullopt,
                                              Vector3d::UnitZ());
    const auto& body2 = plant.GetBodyByName("body2");
    plant.AddJoint<multibody::PrismaticJoint>("body2", plant.world_body(),
                                              std::nullopt, body2, std::nullopt,
                                              Vector3d::UnitX());
    plant.Finalize();

    MeshcatVisualizerParams params;
    params.delete_on_initialization_event = false;
    auto& visualizer = MeshcatVisualizerd::AddToBuilder(
        &builder, scene_graph, meshcat, std::move(params));

    multibody::meshcat::ContactVisualizerParams cparams;
    cparams.newtons_per_meter = 60.0;
    auto& contact = multibody::meshcat::ContactVisualizerd::AddToBuilder(
        &builder, plant, meshcat, std::move(cparams));

    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();

    plant.SetPositions(&plant.GetMyMutableContextFromRoot(context.get()),
                       Eigen::Vector2d{0.1, 0.3});
    diagram->ForcedPublish(*context);
    std::cout << "- Now you should see three colliding hydroelastic spheres."
              << std::endl;
    std::cout << "[Press RETURN to continue]." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    contact.Delete();
    visualizer.Delete();
  }

  {
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
    multibody::Parser parser(&plant);
    parser.AddModels(
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                            "iiwa14_spheres_collision.urdf"));
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
    parser.AddModels(FindResourceOrThrow(
        "drake/examples/kuka_iiwa_arm/models/table/"
        "extra_heavy_duty_table_surface_only_collision.sdf"));
    const double table_height = 0.7645;
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("link"),
                     RigidTransformd(Vector3d{0, 0, -table_height - .01}));
    plant.Finalize();

    builder.ExportInput(plant.get_actuation_input_port(), "actuation_input");
    MeshcatVisualizerParams params;
    params.delete_on_initialization_event = false;
    auto& visualizer = MeshcatVisualizerd::AddToBuilder(
        &builder, scene_graph, meshcat, std::move(params));

    multibody::meshcat::ContactVisualizerParams cparams;
    cparams.newtons_per_meter = 60.0;
    auto& contact = multibody::meshcat::ContactVisualizerd::AddToBuilder(
        &builder, plant, meshcat, std::move(cparams));

    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();
    diagram->get_input_port().FixValue(context.get(), Eigen::VectorXd::Zero(7));

    diagram->ForcedPublish(*context);
    std::cout
        << "- Now you should see a kuka model (from MultibodyPlant/SceneGraph)"
        << std::endl;

    std::cout << "[Press RETURN to continue]." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::cout << "Now we'll run the simulation...\n"
              << "- You should see the robot fall down and hit the table\n"
              << "- You should see the contact force vectors (when it hits)\n"
              << std::endl;

    systems::Simulator<double> simulator(*diagram, std::move(context));
    simulator.set_target_realtime_rate(1.0);
    visualizer.StartRecording();
    simulator.AdvanceTo(4.0);
    visualizer.PublishRecording();
    contact.Delete();

    std::cout
        << "The recorded simulation results should now be available as an "
           "animation.  Use the animation GUI to confirm.  The contact "
           "forces are not recorded (yet)."
        << std::endl;

    std::cout << "[Press RETURN to continue]." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  const std::string html_filename(temp_directory() + "/meshcat_static.html");
  std::ofstream html_file(html_filename);
  html_file << meshcat->StaticHtml();
  html_file.close();

  std::cout << "A standalone HTML file capturing this scene (including the "
               "animation) has been written to file://"
            << html_filename
            << "\nOpen that location in your browser now and confirm that "
               "the iiwa is visible and the animation plays."
            << std::endl;

  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::remove(html_filename.c_str());
  std::cout
      << "Note: I've deleted the temporary HTML file (it's several Mb).\n\n";

  meshcat->AddButton("ButtonTest");
  meshcat->AddButton("Press t Key");
  meshcat->AddButton("Press t Key", "KeyT");  // Now the keycode is assigned.
  meshcat->AddSlider("SliderTest", 0, 1, 0.01, 0.5, "ArrowLeft", "ArrowRight");

  std::cout << "I've added two buttons and a slider to the controls menu.\n";
  std::cout << "- Click the ButtonTest button a few times.\n";
  std::cout << "- Press the 't' key in the meshcat window, which "
               "should be equivalent to pressing the second button.\n";
  std::cout << "The buttons do nothing, but the total number of clicks for "
               "each button will be reported after you press RETURN.\n";
  std::cout << "- Move SliderTest slider.\n";
  std::cout << "- Confirm that the ArrowLeft and ArrowRight keys also move the "
               "slider.\n";
  std::cout << "- Open a second browser (" << meshcat->web_url()
            << ") and confirm that moving the slider in one updates the slider "
               "in the other.\n";

  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  std::cout << "Got " << meshcat->GetButtonClicks("ButtonTest")
            << " clicks on ButtonTest.\n"
            << "Got " << meshcat->GetButtonClicks("Press t Key")
            << " clicks on \"Press t Key\".\n"
            << "Got " << meshcat->GetSliderValue("SliderTest")
            << " value for SliderTest.\n\n" << std::endl;

  std::cout << "Next, we'll test gamepad (i.e., joystick) features.\n\n";
  std::cout
      << "While the Meshcat browser window has focus, click any button on "
      << "your gamepad to activate gamepad support in the browser.\n\n";
  std::cout
      << "Then(after you press RETURN), we'll print the gamepad stats for 5 "
      << "seconds. During that time, move the control sticks and hold some "
      << "buttons and you should see those values reflected in the printouts. "
      << "As long as you see varying values as you move the controls, that's "
      << "sufficient to consider the test passing; the exact values do not "
      << "matter.\n";

  std::cout << "[Press RETURN to continue]." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  Meshcat::Gamepad gamepad = meshcat->GetGamepad();
  if (!gamepad.index) {
    std::cout << "No gamepad activity detected.\n";
  } else {
    for (int i = 0; i < 5; ++i) {
      gamepad = meshcat->GetGamepad();
      std::cout << "Gamepad status:\n";
      std::cout << "  gamepad index: " << *gamepad.index << "\n";
      std::cout << "  buttons: ";
      for (auto const& value : gamepad.button_values) {
        std::cout << value << ", ";
      }
      std::cout << "\n";
      std::cout << "  axes: ";
      for (auto const& value : gamepad.axes) {
        std::cout << value << ", ";
      }
      std::cout << "\n";
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  std::cout << "Exiting..." << std::endl;
  return 0;
}

}  // namespace geometry
}  // namespace drake

int main() { return drake::geometry::do_main(); }
