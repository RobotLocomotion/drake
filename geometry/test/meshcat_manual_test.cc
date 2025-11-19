#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
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
namespace {

using common::MaybePauseForUser;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

namespace fs = std::filesystem;

// Returns an offset pointer inside message that skips over leading newlines.
const char* ltrim(const char* message) {
  while (*message == '\n') {
    ++message;
  }
  return message;
}

// Loads the textured cuboctahedron with hole into an in-memory Mesh.
Mesh GetCuboctahedronInMemory(double scale) {
  const fs::path obj_path = FindResourceOrThrow(
      "drake/examples/scene_graph/cuboctahedron_with_hole.obj");
  const fs::path obj_dir = obj_path.parent_path();
  string_map<FileSource> supporting_files;
  for (const auto* f : {"cuboctahedron_with_hole.mtl", "rainbow_checker.png"}) {
    supporting_files.emplace(f, MemoryFile::Make(std::move(obj_dir / f)));
  }
  return Mesh(
      InMemoryMesh{MemoryFile::Make(obj_path), std::move(supporting_files)},
      scale);
}

// Loads the fully_textured_pyramid.gltf file as an in-memory mesh.
Mesh GetPyramidInMemory(double scale = 1.0) {
  const fs::path gltf_path = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");
  const fs::path gltf_dir = gltf_path.parent_path();
  string_map<FileSource> supporting_files;
  // These are _all_ the files referenced in fully_textured_pyramid.gltf. Only
  // the ktx2 images will render, but the console will complain about not being
  // able to find the .png images if we don't include them as supporting files.
  // We'll add the ktx2 files as path FileSources to test both representations
  // (MemoryFile and path).
  bool have_paths = false;
  for (const auto* f :
       {"fully_textured_pyramid_emissive.png",
        "fully_textured_pyramid_normal.png", "fully_textured_pyramid_omr.png",
        "fully_textured_pyramid_base_color.png",
        "fully_textured_pyramid_emissive.ktx2",
        "fully_textured_pyramid_normal.ktx2", "fully_textured_pyramid_omr.ktx2",
        "fully_textured_pyramid_base_color.ktx2",
        "fully_textured_pyramid.bin"}) {
    const std::string f_name(f);
    if (f_name.ends_with(".ktx2")) {
      supporting_files.emplace(f, gltf_dir / f);
      have_paths = true;
    } else {
      supporting_files.emplace(f, MemoryFile::Make(std::move(gltf_dir / f)));
    }
  }
  // Make sure there's not some erroneous logic in putting the ktx2 files in as
  // path FileSource.
  DRAKE_DEMAND(have_paths);

  return Mesh(
      InMemoryMesh{MemoryFile::Make(gltf_path), std::move(supporting_files)},
      scale);
}

int do_main() {
  auto meshcat = std::make_shared<Meshcat>();

  // For every items we add to the initial array, decrement start_x by one half
  // to keep things centered.
  // Use ++x as the x-position of new items.
  const double start_x = -8.5;
  double x = start_x;

  Vector3d sphere_home{++x, 0, 0};
  // The weird name for the sphere is to confirm that meshcat collapses
  // redundant slashes. We'll subsequently refer to it without the slash to
  // make sure they're equivalent.
  meshcat->SetObject("sphere//scoped_name", Sphere(0.25), Rgba(1.0, 0, 0, 1));
  meshcat->SetTransform("sphere/scoped_name", RigidTransformd(sphere_home));
  // Note: this isn't the preferred means for setting opacity, but it is the
  // simplest way to exercise chained property names.
  meshcat->SetProperty("sphere/scoped_name/<object>", "material.opacity", 0.5);
  meshcat->SetProperty("sphere/scoped_name/<object>", "material.transparent",
                       true);

  // The weird name for the cylinder is to confirm that meshcat elides terminal
  // slashes. We'll subsequently refer to it without the slash to make sure
  // they're equivalent.
  meshcat->SetObject("cylinder/", Cylinder(0.25, 0.5), Rgba(0.0, 1.0, 0, 1));
  meshcat->SetTransform("cylinder", RigidTransformd(Vector3d{++x, 0, 0}));

  // For animation, we'll aim the camera between the cylinder and ellipsoid.
  const Vector3d animation_target{x + 0.5, 0, 0};

  meshcat->SetObject("ellipsoid", Ellipsoid(0.25, 0.25, 0.5),
                     Rgba(1.0, 0, 1, 0.5));
  meshcat->SetTransform("ellipsoid", RigidTransformd(Vector3d{++x, 0, 0}));

  Vector3d box_home{++x, 0, 0};
  meshcat->SetObject("box", Box(0.25, 0.25, 0.5), Rgba(0, 0, 1, 1));
  meshcat->SetTransform("box", RigidTransformd(box_home));

  const std::string polytope_with_hole = FindResourceOrThrow(
      "drake/examples/scene_graph/cuboctahedron_with_hole.obj");
  meshcat->SetObject("obj_as_convex", Convex(polytope_with_hole, 0.25),
                     Rgba(0.8, 0.4, 0.1, 1.0));
  meshcat->SetTransform("obj_as_convex", RigidTransformd(Vector3d{++x, 0, 0}));

  meshcat->SetObject("obj_as_mesh", Mesh(polytope_with_hole, 0.25),
                     Rgba(0.8, 0.4, 0.1, 1.0));
  meshcat->SetTransform("obj_as_mesh", RigidTransformd(Vector3d{x, 1, 0}));

  meshcat->SetObject("capsule", Capsule(0.25, 0.5), Rgba(0, 1, 1, 1));
  meshcat->SetTransform("capsule", RigidTransformd(Vector3d{++x, 0, 0}));

  // Note that height (in z) is the first argument.
  meshcat->SetObject("cone", MeshcatCone(0.5, 0.25, 0.5), Rgba(1, 0, 0, 1));
  meshcat->SetTransform("cone", RigidTransformd(Vector3d{++x, 0, 0}));

  // The color and shininess properties come from PBR materials.
  meshcat->SetObject(
      "gltf",
      Mesh(FindResourceOrThrow(
               "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf"),
           0.5));
  const Vector3d gltf_pose{++x, 0, 0};
  meshcat->SetTransform("gltf", RigidTransformd(gltf_pose));

  meshcat->SetObject("gltf_in_memory", GetPyramidInMemory(0.5));
  meshcat->SetTransform("gltf_in_memory",
                        RigidTransformd(gltf_pose + Vector3d(0, 1.5, 0)));

  auto mustard_obj =
      FindRunfile("drake_models/ycb/meshes/006_mustard_bottle_textured.obj")
          .abspath;
  meshcat->SetObject("mustard", Mesh(mustard_obj, 3.0));
  meshcat->SetTransform("mustard", RigidTransformd(Vector3d{++x, 0, 0}));

  {
    const int kPoints = 100000;
    perception::PointCloud cloud(
        kPoints, perception::pc_flags::kXYZs | perception::pc_flags::kRGBs);
    Eigen::Matrix3Xf m = Eigen::Matrix3Xf::Random(3, kPoints);
    cloud.mutable_xyzs() = Eigen::DiagonalMatrix<float, 3>{0.25, 0.25, 0.5} * m;
    cloud.mutable_rgbs() = (255.0 * (m.array() + 1.0) / 2.0).cast<uint8_t>();
    meshcat->SetObject("point_cloud", cloud, 0.01);
    meshcat->SetTransform("point_cloud", RigidTransformd(Vector3d{++x, 0, 0}));
  }

  {
    Eigen::Matrix3Xd vertices(3, 200);
    Eigen::RowVectorXd t = Eigen::RowVectorXd::LinSpaced(200, 0, 10 * M_PI);
    vertices << 0.25 * t.array().sin(), 0.25 * t.array().cos(), t / (10 * M_PI);
    meshcat->SetLine("line", vertices, 3.0, Rgba(0, 0, 1, 1));
    meshcat->SetTransform("line", RigidTransformd(Vector3d{++x, 0, -0.5}));
  }

  {
    Eigen::Matrix3Xd start(3, 4), end(3, 4);
    // clang-format off
    start << -0.1, -0.1,  0.1,  0.1,
             -0.1,  0.1, -0.1,  0.1,
                0,    0,    0,    0;
    // clang-format on
    end = start;
    end.row(2) = Eigen::RowVector4d::Ones();
    meshcat->SetLineSegments("line_segments", start, end, 5.0,
                             Rgba(0, 1, 0, 1));
    meshcat->SetTransform("line_segments",
                          RigidTransformd(Vector3d{++x, 0, -0.5}));
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
    TriangleSurfaceMesh<double> surface_mesh(std::move(faces),
                                             std::move(vertices));
    meshcat->SetObject("triangle_mesh", surface_mesh, Rgba(0.9, 0, 0.9, 1.0));
    meshcat->SetTransform("triangle_mesh",
                          RigidTransformd(Vector3d{++x, -0.25, 0}));

    meshcat->SetObject("triangle_mesh_wireframe", surface_mesh,
                       Rgba(0.9, 0, 0.9, 1.0), true, 5.0);
    meshcat->SetTransform("triangle_mesh_wireframe",
                          RigidTransformd(Vector3d{++x, -0.25, 0}));
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
                          RigidTransformd(Vector3d{++x, -0.25, 0}));
  }

  // PlotSurface.
  {
    constexpr int nx = 15, ny = 11;
    Eigen::MatrixXd X =
        Eigen::RowVectorXd::LinSpaced(nx, 0, 1).replicate<ny, 1>();
    Eigen::MatrixXd Y = Eigen::VectorXd::LinSpaced(ny, 0, 1).replicate<1, nx>();
    // z = y*sin(5*x)
    Eigen::MatrixXd Z = (Y.array() * (5 * X.array()).sin()).matrix();

    meshcat->PlotSurface("plot_surface", X, Y, Z, Rgba(0, 0, 0.9, 1.0), true);
    meshcat->SetTransform("plot_surface",
                          RigidTransformd(Vector3d{++x, -0.25, 0}));
  }

  std::cout << "\nDo *not* open up your browser to the URL above. Instead use "
            << "the following URL\n\n"
            << meshcat->web_url() << "?tracked_camera=on\n\n";

  MaybePauseForUser();

  // Note: this tests that the parameter on the html page is enough to enable
  // camera tracking. Full camera tracking protocols are tested in
  // meshcat_camera_tracking_test.py.
  if (meshcat->GetTrackedCameraPose() == std::nullopt) {
    std::cout << "Meshcat isn't receiving tracked camera poses from your "
              << "browser. Are you sure your browser is using the full URL?\n"
              << "\n    " << meshcat->web_url() << "?tracked_camera=on\n\n"
              << "If not, use the url with the 'tracked_camera' parameter.\n\n";

    MaybePauseForUser();

    if (meshcat->GetTrackedCameraPose() == std::nullopt) {
      std::cout << "  !!! ERROR !!! It appears that camera tracking isn't "
                << "working!\n";
      return 1;
    } else {
      std::cout << "That did it. Now we can move on.\n";
    }
  }

  std::cout << ltrim(R"""(
Open the developer tools of your browser (F12) and within that panel switch to
the "Console" tab.

Keep that panel visible throughout the entire testing process. Any errors or
warnings displayed in that Console most likely indicate a bug in our code and
should be fixed. (If you are running this test as part of a pull request code
review, be sure to post the message as a Reviewable discussion.)

Less severe Console messages (info, debug, etc.) are not bugs and can be
ignored.

Caveat: At the moment, you might see Console warnings related to deprecations.
Ignore those for now; we'll need to circle back and fix them later.
)""");
  MaybePauseForUser();

  std::cout << ltrim(R"""(
- The background should be grey.
- From left to right along the x axis, you should see:
  - a slightly transparent red sphere
  - a green cylinder (with the long axis in z)
  - a pink semi-transparent ellipsoid (long axis in z)
  - a blue box (long axis in z)
  - an orange polytope (with a similary shaped textured polytope behind it).
    The textured shape has a hole through. The orange polytope is its convex
    hull.
  - a teal capsule (long axis in z)
  - a red cone (expanding in +z, twice as wide in y than in x)
  - two shiny, textured pyramids (created with PBR materials); one read from
    disk, the other loaded from memory.
  - a yellow mustard bottle w/ label
  - a dense rainbow point cloud in a box (long axis in z)
  - a blue line coiling up (in z).
  - 4 green vertical line segments (in z).
  - a purple triangle mesh with 2 faces.
  - the same purple triangle mesh drawn as a wireframe.
  - the same triangle mesh drawn in multicolor.
  - a blue mesh plot of the function z = y*sin(5*x).
)""");
  MaybePauseForUser();

  std::cout << "Calling meshcat.Flush(), which will block until all clients "
               "have received all the data)...";
  meshcat->Flush();
  std::cout << "Done." << std::endl;

  std::cout << "\nAnimations:\n";
  meshcat->SetCameraPose(animation_target + Vector3d{0, -3, 1.5},
                         animation_target);
  std::cout << "The camera has moved to focus on the following animated "
               "geometries:\n";
  MeshcatAnimation animation;
  std::cout << "- the red sphere should move up and down in z.\n";
  animation.SetTransform(0, "sphere/scoped_name", RigidTransformd(sphere_home));
  animation.SetTransform(20, "sphere/scoped_name",
                         RigidTransformd(sphere_home + Vector3d::UnitZ()));
  animation.SetTransform(40, "sphere/scoped_name",
                         RigidTransformd(sphere_home));

  std::cout << "- the blue box should spin clockwise about the +z axis.\n";
  animation.SetTransform(
      0, "box", RigidTransformd(RotationMatrixd::MakeZRotation(0), box_home));
  animation.SetTransform(
      20, "box",
      RigidTransformd(RotationMatrixd::MakeZRotation(M_PI), box_home));
  animation.SetTransform(
      40, "box",
      RigidTransformd(RotationMatrixd::MakeZRotation(2 * M_PI), box_home));
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
  MaybePauseForUser();

  meshcat->Set2dRenderMode(math::RigidTransform(Vector3d{0, -3, 0}), -4, 4, -2,
                           2);

  std::cout << "- The scene should have switched to 2D rendering mode.\n";
  MaybePauseForUser();

  meshcat->Set2dRenderMode(
      math::RigidTransform(math::RotationMatrixd::MakeZRotation(-M_PI / 2.0),
                           sphere_home),
      -2, 2, -2, 2);
  // This call shows that SetCameraTarget() is a no-op for orthographic cameras.
  // The described view should be unaffected by this absurd target point.
  meshcat->SetCameraTarget(Vector3d{0, -100, -50});

  std::cout << "- Now 2D rendering along the +x axis (red sphere in front).\n";
  MaybePauseForUser();

  meshcat->SetCameraPose(Vector3d{2, 2, 2}, Vector3d::Zero());
  std::cout << "- Now we have an isometric 3/4 view.\n";
  MaybePauseForUser();

  std::cout << "- The scene should have switched back to 3D.\n";
  meshcat->ResetRenderMode();

  MaybePauseForUser();

  // Turn off the background (it will appear white).
  meshcat->SetProperty("/Background", "visible", false);

  meshcat->Delete("box");
  meshcat->SetProperty("/Lights/AmbientLight/<object>", "intensity", 0.1);

  std::cout
      << "- The blue box should have disappeared\n"
      << "- The lights should have dimmed.\n"
      << "- The background should have been disabled (it will appear white)"
      << std::endl;
  MaybePauseForUser();

  meshcat->SetCameraTarget(gltf_pose);
  meshcat->SetProperty("/Background", "visible", true);
  meshcat->SetEnvironmentMap(
      FindResourceOrThrow("drake/geometry/test/env_256_cornell_box.png"));

  std::cout << "- An environment map has been loaded from a png -- the Cornell "
            << "box.\n"
            << "  The shiny pyramids should reflect it (the camera has moved "
            << "to focus on the pyramids).\n";
  MaybePauseForUser();

  meshcat->SetEnvironmentMap(
      FindResourceOrThrow("drake/geometry/test/env_256_cornell_box.hdr"));

  std::cout << "- The environment map has been changed for an hdr version of "
            << "the Cornell box. The scene will get brighter as the hdr "
            << "texture contains much more radiant energy\n";
  MaybePauseForUser();

  meshcat->SetProperty("/Render Settings/<object>", "exposure", 0.25);
  std::cout << "- To accommodate the extra radiant energy in the hdr "
            << "environment map, we've changed /Render Settings/<object>'s "
            << "exposure value to 0.25. Open that control, and play with the "
            << "exposure yourself. Lower values will make the scene darker, "
            << "higher values brighter.\n";
  MaybePauseForUser();

  meshcat->SetProperty("/Render Settings/<object>", "exposure", 1.0);
  meshcat->SetEnvironmentMap(
      FindResourceOrThrow("drake/geometry/test/env_256_brick_room.jpg"));

  std::cout << "- The Cornell box has been replaced by a room with brick walls "
            << "loaded from a jpg. The exposure value has been reset to its "
            << "default value of 1.\n";
  MaybePauseForUser();

  std::cout << ltrim(R"""(
- Reloading the page should always succeed. Force a complete reload now using
  Ctrl-Shift-R and confirm you still see same objects, the animation, the brick
  walls, etc.
)""");
  MaybePauseForUser();

  meshcat->SetEnvironmentMap("");
  meshcat->SetCameraTarget(Vector3d::Zero());
  meshcat->Delete();
  std::cout << "- Everything should have disappeared." << std::endl;

  MaybePauseForUser();

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
    meshcat->SetCameraPose(Vector3d{0, -1.5, 1}, Vector3d{0.25, 0, 0});
    std::cout << "- Now you should see three colliding hydroelastic spheres."
              << std::endl;
    MaybePauseForUser();

    contact.Delete();
    visualizer.Delete();
  }

  {
    meshcat->SetCameraPose(Vector3d{-1.0, -1.0, 1.5}, Vector3d{0, 0, 0.5});
    systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
    multibody::Parser parser(&plant);
    parser.AddModelsFromUrl(
        "package://drake_models/iiwa_description/urdf/"
        "iiwa14_spheres_collision.urdf");
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
    parser.AddModels(FindResourceOrThrow(
        "drake/examples/kuka_iiwa_arm/models/table/"
        "extra_heavy_duty_table_surface_only_collision.sdf"));
    const double table_height = 0.7645;
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("link"),
                     RigidTransformd(Vector3d{0, 0, -table_height - 0.01}));
    parser.AddModelsFromUrl(
        "package://drake_models/ycb/006_mustard_bottle.sdf");
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("base_link_mustard"),
                     RigidTransformd(Vector3d{0, -0.3, 0.01}));
    plant.Finalize();

    builder.ExportInput(plant.get_actuation_input_port(), "actuation_input");
    MeshcatVisualizerParams params;
    params.delete_on_initialization_event = false;
    auto& visualizer = MeshcatVisualizerd::AddToBuilder(
        &builder, scene_graph, meshcat, std::move(params));

    multibody::meshcat::ContactVisualizerParams cparams;
    cparams.newtons_per_meter = 60.0;
    multibody::meshcat::ContactVisualizerd::AddToBuilder(
        &builder, plant, meshcat, std::move(cparams));

    auto diagram = builder.Build();
    auto context = diagram->CreateDefaultContext();
    diagram->get_input_port().FixValue(context.get(), Eigen::VectorXd::Zero(7));

    diagram->ForcedPublish(*context);
    std::cout
        << "- Now you should see a kuka model (from MultibodyPlant/SceneGraph) "
           "and a mustard bottle lying on its side, with the label facing up."
        << std::endl;

    MaybePauseForUser();

    std::cout << "Now we'll run the simulation...\n"
              << "- You should see the robot fall down and hit the table\n"
              << "- You should see the contact force vectors (when it hits)\n"
              << std::endl;

    systems::Simulator<double> simulator(*diagram, std::move(context));
    simulator.set_target_realtime_rate(1.0);
    visualizer.StartRecording();
    simulator.AdvanceTo(4.0);
    visualizer.PublishRecording();

    std::cout
        << "The recorded simulation results should now be available as an "
           "animation.  Use the animation GUI to confirm."
        << std::endl;

    MaybePauseForUser();
  }

  std::cout << "Now we'll add back some elements in preparation for testing "
               "the standalone HTML download:\n"
               "  - an environment map\n"
               "  - reposition the camera\n"
               "  - add an in-memory glTF file (textured pyramid)\n"
               "  - add an in-memory obj file (textured cuboctahedron)\n"
               "\n";

  meshcat->SetEnvironmentMap(
      FindResourceOrThrow("drake/geometry/test/env_256_cornell_box.hdr"));
  meshcat->SetProperty("/Render Settings/<object>", "exposure", 0.25);
  meshcat->SetCameraTarget(Vector3d{-0.4, 0, 0});

  meshcat->SetObject("gltf_in_memory", GetPyramidInMemory(0.1));
  meshcat->SetTransform("gltf_in_memory",
                        RigidTransformd(Vector3d(0.25, 0.3, 0.1)));
  meshcat->SetObject("obj_in_memory", GetCuboctahedronInMemory(0.1));
  meshcat->SetTransform("obj_in_memory",
                        RigidTransformd(Vector3d(-0.25, 0.3, 0.1)));

  std::cout
      << "Now we'll check the standalone HTML file capturing this scene.\n"
         "Open this link to download an HTML file:\n\n"
      << "  " << meshcat->web_url() << "/download\n\n"
      << "Open the downloaded file in a new browser tab confirm that:\n"
         "- the camera is focused on the contact point between the robot and "
         "table,\n"
         "- the iiwa is visible,\n"
         "- the mustard bottle visible including its texture (front label),\n"
         "- the animation plays,\n"
         "- the exposure has been set to 0.25,\n"
         "- the environment map is present, and\n"
         "- the browser Console has no warnings nor errors\n"
         "  (use F12 to open the panel with the Console).\n\n";
  std::cout << "When you're done, close the browser tab.\n";

  MaybePauseForUser();

  meshcat->SetEnvironmentMap("");
  meshcat->SetProperty("/Render Settings/<object>", "exposure", 1.0);
  meshcat->SetCameraPose(Vector3d{-1.0, -1.0, 1.5}, Vector3d{0, 0, 0.5});

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

  MaybePauseForUser();

  std::cout << "Got " << meshcat->GetButtonClicks("ButtonTest")
            << " clicks on ButtonTest.\n"
            << "Got " << meshcat->GetButtonClicks("Press t Key")
            << " clicks on \"Press t Key\".\n"
            << "Got " << meshcat->GetSliderValue("SliderTest")
            << " value for SliderTest.\n\n"
            << std::endl;

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

  MaybePauseForUser();

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

  std::cout << "\n";
  std::cout << "Now we'll test the WebXR functionality.\n";
  std::cout << "In a new browser window, open the URL:\n  "
            << meshcat->web_url() << "?webxr=vr&controller=on\n";
  std::cout << "If you don't have VR hardware installed on your machine, "
               "you'll have to install the WebXR API emulator appropriate to "
               "your browser. For Google Chrome "
               "see:\n  https://chrome.google.com/webstore/detail/"
               "webxr-api-emulator/mjddjgeghkdijejnciaefnkjmkafnnje\n"
               "If you are using Firefox see:\n "
               "https://addons.mozilla.org/de/firefox/addon/"
               "webxr-api-emulator/";
  std::cout << "\nIf the emulator is installed properly, you should see a "
               "button at the bottom that says \"Enter VR\".\n";
  std::cout << "Open the developer tools of your browser (F12). At the top "
               "of the developer tools windows click on the double arrows icon "
               "and a new tab should be available saying \" WebXR \". "
               "Make sure to select a device with controllers from the top "
               "drop-down menu (e.g., Oculus Quest). Click the "
               "\"Enter VR\" button. You should see the following:\n"
            << "  - The rendering screen is now split into two images.\n"
            << "  - The meshcat controls are gone (there is a message in the "
               "console informing you of this).\n"
            << "  - You should be able to manipulate the view in the WebXR "
               "emulator to affect what you see."
            << "  - Clicking on the headset/controller "
               "mesh for the first time in "
               "the emulator window will bring up colored arrows which you can "
               "use to move the mesh. Click a second time on the mesh to "
               "switch to rotation mode."
            << "When you're done, close the browser window.\n\n";

  MaybePauseForUser();

  std::cout << "\nNow we'll try it again with *augmented* reality.\n"
            << "In yet another browser window, open:\n"
            << meshcat->web_url() << "?webxr=ar&controller=on\n"
            << "This should be the same as before but with two differences:\n"
            << "  - The button reads \"Enter XR\"\n"
            << "  - When you click the button, the background becomes white. "
               "If you have an actual AR device, you should see the camera's "
               "image as the background.\n"
            << "When you're done, close the browser window.\n\n";

  MaybePauseForUser();

  std::cout << "Exiting..." << std::endl;
  return 0;
}  // NOLINT(readability/fn_size)

}  // namespace
}  // namespace geometry
}  // namespace drake

int main(int argc, char* argv[]) {
  // This enables ":add_text_logging_gflags" to control the spdlog level.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::do_main();
}
