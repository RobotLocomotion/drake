#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace examples {
namespace {

using drake::geometry::Box;
using drake::geometry::CollisionFilterDeclaration;
using drake::geometry::Cylinder;
using drake::geometry::GeometryId;
using drake::geometry::GeometrySet;
using drake::geometry::Meshcat;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraphConfig;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::math::UniformlyRandomRotationMatrix;
using drake::multibody::BodyIndex;
using drake::multibody::CenicIntegrator;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::PackageMap;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::multibody::contact_solvers::icf::IcfSolverParameters;
using drake::multibody::contact_solvers::internal::SapHessianFactorizationType;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::multibody::internal::CompliantContactManager;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::IntegratorBase;
using drake::systems::Simulator;
using drake::systems::SimulatorConfig;
using drake::visualization::ApplyVisualizationConfig;
using drake::visualization::VisualizationConfig;
using drake::yaml::LoadYamlFile;
using drake::yaml::LoadYamlOptions;
using Eigen::Translation3d;
using Eigen::Vector3d;

/* Bespoke settings for this example. */
struct ClutterConfig {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(density));
    a->Visit(DRAKE_NVP(emulate_box_multicontact));
    a->Visit(DRAKE_NVP(num_spheres_per_face));
    a->Visit(DRAKE_NVP(enable_box_box_collision));
    a->Visit(DRAKE_NVP(add_box_corners));
    a->Visit(DRAKE_NVP(num_piles));
    a->Visit(DRAKE_NVP(objects_per_pile));
    a->Visit(DRAKE_NVP(dz));
    a->Visit(DRAKE_NVP(scale_factor));
    a->Visit(DRAKE_NVP(add_sink_walls));
    a->Visit(DRAKE_NVP(enable_boxes));
    a->Visit(DRAKE_NVP(random_offsets));
  }

  double density{};
  bool emulate_box_multicontact{};
  int num_spheres_per_face{};
  bool enable_box_box_collision{};
  bool add_box_corners{};
  int num_piles{};
  int objects_per_pile{};
  double dz{};
  double scale_factor{};
  bool add_sink_walls{};
  bool enable_boxes{};
  bool random_offsets{};
};

/* All configuration settings for this example. */
struct Config {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(simulation_time));
    a->Visit(DRAKE_NVP(simulator_config));
    a->Visit(DRAKE_NVP(plant_config));
    a->Visit(DRAKE_NVP(scene_graph_config));
    a->Visit(DRAKE_NVP(visualization_config));
    a->Visit(DRAKE_NVP(icf_solver_config));
    a->Visit(DRAKE_NVP(clutter_config));
  }

  double simulation_time{};
  SimulatorConfig simulator_config;
  MultibodyPlantConfig plant_config;
  SceneGraphConfig scene_graph_config;
  std::optional<VisualizationConfig> visualization_config;
  IcfSolverParameters icf_solver_config;
  ClutterConfig clutter_config;
};

constexpr double kSinkWidth = 0.8;
constexpr double kSinkLength = 0.8;

// TODO(jwnimmer-tri) Mutable global variables are not allowed by GSG.
std::vector<GeometryId> global_box_geometry_ids;

const RigidBody<double>& AddBox(
    const ClutterConfig& clutter_config, const std::string& name,
    const Vector3<double>& block_dimensions, double mass, bool rigid,
    const Vector4<double>& color, bool emulate_box_multicontact,
    bool add_box_collision, MultibodyPlant<double>* plant) {
  // Ensure the block's dimensions are mass are positive.
  const double LBx = block_dimensions.x();
  const double LBy = block_dimensions.y();
  const double LBz = block_dimensions.z();

  // Describe body B's mass, center of mass, and inertia properties.
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBcm_B =
      UnitInertia<double>::SolidBox(LBx, LBy, LBz);
  const SpatialInertia<double> M_BBcm_B(mass, p_BoBcm_B, G_BBcm_B);

  // Create a rigid body B with the mass properties of a uniform solid block.
  const RigidBody<double>& box = plant->AddRigidBody(name, M_BBcm_B);

  // Box's visual.
  // The pose X_BG of block B's geometry frame G is an identity transform.
  const RigidTransformd X_BG;  // Identity transform.
  plant->RegisterVisualGeometry(box, X_BG, Box(LBx, LBy, LBz), name + "_visual",
                                color);

  // When the TAMSI solver is used, we simply let MultibodyPlant estimate
  // contact parameters based on penetration_allowance and stiction_tolerance.
  ProximityProperties props;
  if (rigid) {
    AddRigidHydroelasticProperties(&props);
  }

  // Box's collision geometry is a solid box.
  if (emulate_box_multicontact) {
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    const Vector4<double> red_50(1.0, 0.0, 0.0, 0.5);
    const int ns = clutter_config.num_spheres_per_face;
    const double radius_x = LBx / ns / 2.0;
    const double radius_y = LBy / ns / 2.0;
    const double radius_z = LBz / ns / 2.0;
    double dx = 2 * radius_x;
    double dy = 2 * radius_y;
    double dz = 2 * radius_z;

    auto add_sphere = [&](const std::string& sphere_name, double x, double y,
                          double z, double radius) {
      const Vector3<double> p_BoSpherei_B(x, y, z);
      const RigidTransformd X_BSpherei(p_BoSpherei_B);
      Sphere shape(radius);
      // Ellipsoid might not be accurate. From console [warning]: "Ellipsoid is
      // primarily for ComputeContactSurfaces in hydroelastic contact model. The
      // accuracy of other collision queries and signed distance queries are not
      // guaranteed."
      plant->RegisterCollisionGeometry(box, X_BSpherei, shape, sphere_name,
                                       props);
    };

    // Add points (zero size spheres) at the corners to avoid spurious
    // interpentrations between boxes and the sink.
    if (clutter_config.add_box_corners) {
      add_sphere("c1", -LBx / 2, -LBy / 2, -LBz / 2, 0);
      add_sphere("c2", +LBx / 2, -LBy / 2, -LBz / 2, 0);
      add_sphere("c3", -LBx / 2, +LBy / 2, -LBz / 2, 0);
      add_sphere("c4", +LBx / 2, +LBy / 2, -LBz / 2, 0);
      add_sphere("c5", -LBx / 2, -LBy / 2, +LBz / 2, 0);
      add_sphere("c6", +LBx / 2, -LBy / 2, +LBz / 2, 0);
      add_sphere("c7", -LBx / 2, +LBy / 2, +LBz / 2, 0);
      add_sphere("c8", +LBx / 2, +LBy / 2, +LBz / 2, 0);
    }

    // Make a "mesh" of non-zero radii spheres.
    for (int i = 0; i < ns; ++i) {
      const double x = -LBx / 2 + radius_x + i * dx;
      for (int j = 0; j < ns; ++j) {
        const double y = -LBy / 2 + radius_y + j * dy;
        for (int k = 0; k < ns; ++k) {
          const double z = -LBz / 2 + radius_z + k * dz;
          if (i == 0 || j == 0 || k == 0 || i == ns - 1 || j == ns - 1 ||
              k == ns - 1) {
            const std::string name_spherei =
                fmt::format("{}_sphere_{}{}{}_collision", name, i, j, k);
            add_sphere(name_spherei, x, y, z, radius_x);
          }
        }  // k
      }  // j
    }  // i
  }

  if (add_box_collision) {
    auto id = plant->RegisterCollisionGeometry(box, X_BG, Box(LBx, LBy, LBz),
                                               name + "_collision", props);
    global_box_geometry_ids.push_back(id);
  }
  return box;
}

void AddSink(const ClutterConfig& clutter_config,
             MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  // Parameters for the sink.
  const double height = 0.8;
  const double wall_thickness = 0.04;
  const double wall_mass = 1.0;
  const Vector4<double> light_blue(0.5, 0.8, 1.0, 0.3);
  const Vector4<double> transparent(0., 0.0, 0.0, 0.0);

  auto add_wall =
      [&](const std::string& name, const Vector3d& dimensions,
          const RigidTransformd& X_WB,
          const Vector4<double>& color) -> const RigidBody<double>& {
    const auto& wall = AddBox(clutter_config, name, dimensions, wall_mass,
                              /* rigid = */ true, color, false, true, plant);
    plant->WeldFrames(plant->world_frame(), wall.body_frame(), X_WB);
    return wall;
  };

  const Vector3d bottom_dimensions(50 * kSinkLength, 50 * kSinkWidth,
                                   wall_thickness);
  const Vector3d side_wall_dimensions(height, kSinkWidth, wall_thickness);
  const Vector3d back_front_wall_dimensions(kSinkLength, wall_thickness,
                                            height);

  add_wall("sink_bottom", bottom_dimensions,
           Translation3d(0, 0, -wall_thickness / 2.0), transparent);

  if (clutter_config.add_sink_walls) {
    add_wall("sink_right", side_wall_dimensions,
             RigidTransformd(RotationMatrixd::MakeYRotation(M_PI_2),
                             Vector3d(kSinkLength / 2.0, 0.0, height / 2.0)),
             light_blue);
    add_wall("sink_left", side_wall_dimensions,
             RigidTransformd(RotationMatrixd::MakeYRotation(M_PI_2),
                             Vector3d(-kSinkLength / 2.0, 0.0, height / 2.0)),
             light_blue);
    add_wall("sink_back", back_front_wall_dimensions,
             Translation3d(0.0, kSinkWidth / 2, height / 2), light_blue);
    add_wall("sink_front", back_front_wall_dimensions,
             Translation3d(0.0, -kSinkWidth / 2, height / 2), light_blue);
  }
}

const RigidBody<double>& AddSphere(const std::string& name, const double radius,
                                   double mass, const Vector4<double>& color,
                                   MultibodyPlant<double>* plant) {
  const UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  const SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody(name, M_Bcm);

  // Add collision geometry.
  const RigidTransformd X_BS = RigidTransformd::Identity();
  const ProximityProperties props;
  plant->RegisterCollisionGeometry(ball, X_BS, Sphere(radius),
                                   name + "_collision", props);

  // Add visual geometry.
  plant->RegisterVisualGeometry(ball, X_BS, Sphere(radius), name + "_visual",
                                color);

  // We add a few spots so that we can appreciate the sphere's
  // rotation, colored on red, green, blue according to the body's axes.
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
  const Vector4<double> green(0.0, 1.0, 0.0, 1.0);
  const Vector4<double> blue(0.0, 0.0, 1.0, 1.0);
  const double visual_radius = 0.2 * radius;
  const Cylinder spot(visual_radius, visual_radius);
  // N.B. We do not place the cylinder's cap exactly on the sphere surface to
  // avoid visualization artifacts when the surfaces are kissing.
  const double radial_offset = radius - 0.45 * visual_radius;
  auto spot_pose = [](const Vector3<double>& position) {
    // The cylinder's z-axis is defined as the normalized vector from the
    // sphere's origin to the cylinder's center `position`.
    const Vector3<double> axis = position.normalized();
    return RigidTransformd(
        Eigen::Quaterniond::FromTwoVectors(Vector3<double>::UnitZ(), axis),
        position);
  };
  plant->RegisterVisualGeometry(ball, spot_pose({radial_offset, 0., 0.}), spot,
                                name + "_x+", red);
  plant->RegisterVisualGeometry(ball, spot_pose({-radial_offset, 0., 0.}), spot,
                                name + "_x-", red);
  plant->RegisterVisualGeometry(ball, spot_pose({0., radial_offset, 0.}), spot,
                                name + "_y+", green);
  plant->RegisterVisualGeometry(ball, spot_pose({0., -radial_offset, 0.}), spot,
                                name + "_y-", green);
  plant->RegisterVisualGeometry(ball, spot_pose({0., 0., radial_offset}), spot,
                                name + "_z+", blue);
  plant->RegisterVisualGeometry(ball, spot_pose({0., 0., -radial_offset}), spot,
                                name + "_z-", blue);
  return ball;
}

std::vector<BodyIndex> AddObjects(const ClutterConfig& clutter_config,
                                  MultibodyPlant<double>* plant) {
  const double radius0 = 0.05;
  const double density = clutter_config.density;  // kg/m^3.

  const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  const Vector4<double> purple(204.0 / 255, 0.0, 204.0 / 255, 1.0);
  const Vector4<double> green(0, 153.0 / 255, 0, 1.0);
  const Vector4<double> cyan(51 / 255, 1.0, 1.0, 1.0);
  const Vector4<double> pink(1.0, 204.0 / 255, 204.0 / 255, 1.0);
  std::vector<Vector4<double>> colors;
  colors.push_back(orange);
  colors.push_back(purple);
  colors.push_back(green);
  colors.push_back(cyan);
  colors.push_back(pink);

  const int seed = 41;
  std::mt19937 generator(seed);
  std::uniform_int_distribution<int> distribution(0, 1);

  auto roll_shape = [&]() {
    if (!clutter_config.enable_boxes) {
      return 0;  // All boxes.
    } else {
      return distribution(generator);
    }
  };

  const int num_objects = clutter_config.objects_per_pile;
  const int num_bodies = plant->num_bodies();

  std::vector<BodyIndex> bodies;
  for (int i = 1; i <= num_objects; ++i) {
    const auto& color = colors[(i - 1) % colors.size()];
    const std::string name = "object" + std::to_string(i + num_bodies);

    double e = clutter_config.scale_factor > 0 ? i - 1 : num_objects - i;
    double scale = std::pow(std::abs(clutter_config.scale_factor), e);

    switch (roll_shape()) {
      case 0: {
        const double radius = radius0 * scale;
        const double volume = 4. / 3. * M_PI * radius * radius * radius;
        const double mass = density * volume;
        bodies.push_back(AddSphere(name, radius, mass, color, plant).index());
        break;
      }
      case 1: {
        const Vector3d box_size = 2 * radius0 * Vector3d::Ones() * scale;
        const double volume = box_size(0) * box_size(1) * box_size(2);
        const double mass = density * volume;
        Vector4<double> color50(color);
        color50.z() = 0.5;
        bodies.push_back(AddBox(clutter_config, name, box_size, mass,
                                /* rigid = */ false, color50,
                                clutter_config.emulate_box_multicontact, true,
                                plant)
                             .index());
        break;
      }
    }
    scale *= clutter_config.scale_factor;
  }

  return bodies;
}

void SetObjectsIntoAPile(const ClutterConfig& clutter_config,
                         const MultibodyPlant<double>& plant,
                         const Vector3d& offset,
                         const std::vector<BodyIndex>& bodies,
                         Context<double>* plant_context) {
  const double delta_z =
      clutter_config.dz;  // Assume objects have a BB of about 10 cm.

  const int seed = 41;
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> distribution(-1e-3, 1e-3);

  int num_objects = clutter_config.objects_per_pile;

  double z = delta_z / 2;
  int i = 1;
  for (auto body_index : bodies) {
    const auto& body = plant.get_body(body_index);
    if (body.is_floating_base_body()) {
      double e = clutter_config.scale_factor > 0 ? i - 1 : num_objects - i;
      double scale = std::pow(std::abs(clutter_config.scale_factor), e);

      if (clutter_config.random_offsets) {
        const RotationMatrixd R_WB =
            UniformlyRandomRotationMatrix<double>(&generator);
        const double x = distribution(generator);
        const double y = distribution(generator);
        const Vector3d p_WB = offset + Vector3d(x, y, z);
        plant.SetFreeBodyPose(plant_context, body, RigidTransformd(R_WB, p_WB));
      } else {
        const Vector3d p_WB = offset + Vector3d(0.0, 0.0, z);
        plant.SetFreeBodyPose(plant_context, body, RigidTransformd(p_WB));
      }

      z += delta_z * scale;
      ++i;
    }
  }
}

int do_main() {
  const Config config = LoadYamlFile<Config>(
      PackageMap().ResolveUrl(
          "package://drake/examples/multibody/clutter/config.yaml"),
      "config", std::nullopt,
      LoadYamlOptions{
          .allow_yaml_with_no_cpp = false,
          .allow_cpp_with_no_yaml = true,
          .retain_map_defaults = true,
      });

  // Build a generic multibody plant.
  DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlant(
      config.plant_config, config.scene_graph_config, &builder);

  AddSink(config.clutter_config, &plant);

  // Add objects to the scene.
  DRAKE_DEMAND(config.clutter_config.num_piles > 0 &&
               config.clutter_config.num_piles <= 4);
  std::vector<std::vector<BodyIndex>> piles;
  for (int i = 0; i < config.clutter_config.num_piles; ++i) {
    piles.push_back(AddObjects(config.clutter_config, &plant));
  }

  // Only box-sphere and sphere-sphere are allowed.
  if (!config.clutter_config.enable_box_box_collision) {
    GeometrySet all_boxes(global_box_geometry_ids);
    scene_graph.collision_filter_manager().Apply(
        CollisionFilterDeclaration().ExcludeWithin(all_boxes));
  }

  plant.Finalize();

  fmt::print("Num positions: {:d}\n", plant.num_positions());
  fmt::print("Num velocities: {:d}\n", plant.num_velocities());

  // Publish contact results for visualization.
  std::shared_ptr<Meshcat> meshcat{nullptr};
  if (config.visualization_config.has_value()) {
    meshcat = std::make_shared<Meshcat>();
    ApplyVisualizationConfig(*config.visualization_config, &builder, nullptr,
                             &plant, &scene_graph, meshcat);
  }
  auto diagram = builder.Build();

  if (config.icf_solver_config.use_dense_algebra && plant.is_discrete()) {
    // Use dense algebra for SAP.
    SapSolverParameters sap_parameters;
    sap_parameters.linear_solver_type = SapHessianFactorizationType::kDense;

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    CompliantContactManager<double>* contact_manager =
        owned_contact_manager.get();
    plant.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    contact_manager->set_sap_solver_parameters(sap_parameters);
  }

  // Set up the simulator with the specified integration scheme.
  Simulator<double> simulator(*diagram);
  ApplySimulatorConfig(config.simulator_config, &simulator);
  IntegratorBase<double>& integrator = simulator.get_mutable_integrator();
  if (config.simulator_config.integration_scheme == "cenic") {
    auto& ci = dynamic_cast<CenicIntegrator<double>&>(integrator);
    ci.SetSolverParameters(config.icf_solver_config);
  }

  // Set the initial position of the objects.
  for (int i = 0; i < config.clutter_config.num_piles; ++i) {
    const double x = (i % 2 == 0) ? -kSinkLength / 4 : kSinkLength / 4;
    const double y = (i / 2 == 0) ? -kSinkWidth / 4 : kSinkWidth / 4;
    SetObjectsIntoAPile(config.clutter_config, plant, Vector3d(x, y, 0.0),
                        piles[i],
                        &diagram->GetMutableSubsystemContext(
                            plant, &simulator.get_mutable_context()));
  }

  simulator.Initialize();
  if (config.visualization_config.has_value()) {
    // Wait for meshcat to load
    fmt::print("Press [ENTER] to continue ...\n");
    getchar();

    const double recording_frames_per_second =
        config.plant_config.time_step == 0
            ? 32.0
            : 1.0 / config.plant_config.time_step;
    meshcat->StartRecording(recording_frames_per_second);
  }

  using clock = std::chrono::steady_clock;
  clock::time_point sim_start_time = clock::now();
  simulator.AdvanceTo(config.simulation_time);
  clock::time_point sim_end_time = clock::now();
  const double sim_time =
      std::chrono::duration<double>(sim_end_time - sim_start_time).count();
  fmt::print("AdvanceTo() time [sec]: {}\n", sim_time);

  if (config.visualization_config.has_value()) {
    meshcat->StopRecording();
    meshcat->PublishRecording();
  }

  PrintSimulatorStatistics(simulator);

  if (config.visualization_config.has_value()) {
    // Wait for meshcat to finish rendering.
    fmt::print("Press [ENTER] to quit ...\n");
    getchar();
  }

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "\nSimulation of a clutter of objects falling into a box container.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
