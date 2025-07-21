#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <utility>

#include <gflags/gflags.h>
#include <valgrind/callgrind.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/temp_directory.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/convex_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace multibody {
namespace examples {
namespace {

constexpr double kHuge = 1.0e40;

// Simulation parameters.
DEFINE_double(simulation_time, 10.0, "Simulation duration in seconds");
DEFINE_double(
    mbp_time_step, 0.0,
    "If mbp_time_step > 0, the fixed-time step period (in seconds) of discrete "
    "updates for the plant (modeled as a discrete system). "
    "If mbp_time_step = 0, the plant is modeled as a continuous system "
    "and no contact forces are displayed.  mbp_time_step must be >= 0.");

// Physical parameters.
DEFINE_double(density, 1000.0, "The density of all objects, in kg/m³.");
DEFINE_double(static_friction, 0.5, "All bodies use this value.");
DEFINE_double(dynamic_friction, 0.5, "All bodies use this value.");
DEFINE_double(box_stiffness, 1.0e5, "Box point contact stiffness in N/m.");
DEFINE_double(sphere_stiffness, 1.0e5,
              "Sphere point contact stiffness in N/m.");
DEFINE_bool(use_hydro, false, "If true, use hydro. Otherwise point contact.");
DEFINE_double(sphere_resolution, 0.02, "Resolution hint for the sphere");
DEFINE_double(dissipation_time_constant, 0.01,
              "Dissipation time constant in seconds.");
DEFINE_double(hc_dissipation, 10.0, "Hunt & Crossley dissipation [s/m].");
DEFINE_double(stiction_tolerance, 1.0e-4, "Stiction tolerance [m/s].");

// Contact geometry parameters.
DEFINE_bool(
    emulate_box_multicontact, true,
    "Emulate multicontact by adding spheres to the faces of box geometries.");
DEFINE_int32(
    num_spheres_per_face, 3,
    "Multi-contact emulation. We place num_sphere x num_spheres_per_face on "
    "each box face, when emulate_box_multicontact = true.");
DEFINE_bool(enable_box_box_collision, false, "Enable box vs. box contact.");
DEFINE_bool(add_box_corners, false,
            "Adds collision points at the corners of each box.");

// Scenario parameters.
DEFINE_int32(num_piles, 4, "Number of piles (1-4).");
DEFINE_int32(objects_per_pile, 5, "Number of objects per pile.");
DEFINE_double(dz, 0.15, "Initial distance between objects in the pile.");
DEFINE_double(scale_factor, 1.0, "Multiplicative factor to generate the pile.");
DEFINE_bool(add_sink_walls, true, "Adds wall of a sink model.");
DEFINE_bool(enable_boxes, false, "Make some of the objects boxes.");
DEFINE_bool(random_offsets, true,
            "Use slight random offsets in initial positions.");

// Visualization.
DEFINE_bool(visualize, true, "Whether to visualize (true) or not (false).");
DEFINE_bool(visualize_forces, false,
            "Whether to visualize forces (true) or not (false).");
DEFINE_double(viz_period, std::numeric_limits<double>::infinity(),
              "Viz period.");

// Logging and verbosity for the convex integrator.
DEFINE_bool(log_solver_stats, false,
            "Whether to log convex integrator statistics to a CSV file.");
DEFINE_bool(print_solver_stats, false,
            "Whether to print convex integrator statistics to the console.");

// Discrete contact solver.
DEFINE_string(discrete_contact_approximation, "lagged",
              "Discrete contact solver. Options are: 'tamsi', 'sap', 'lagged', "
              "'similar'.");
DEFINE_double(near_rigid_threshold, 1.0, "SAP near rigid threshold.");
DEFINE_bool(dense_algebra, false, "Whether to use dense algebra in SAP.");

// Continuous integration parameters
DEFINE_string(
    integrator_jacobian_scheme, "forward",
    "Jacobian computation scheme: 'forward', 'central', 'automatic'.");
DEFINE_bool(full_newton, false, "Update Jacobian every iteration.");
DEFINE_bool(trapezoid, false, "Implicit trapezoid rule for error estimation.");

// Convex integrator parameters.
DEFINE_bool(enable_hessian_reuse, true,
            "Whether to reuse the Hessian factorization between iterations.");
DEFINE_int32(k_max, 10,
             "Maximum number of iterations before re-computing the Hessian.");
DEFINE_double(kappa, 0.05,
              "Scaling factor for the relaxed convergence check (θ method of "
              "Hairer 1996) used to exit early under loose accuracies.");
DEFINE_double(
    alpha_max, 1.0,
    "Maximum line search step size for the convex integrator (α_max).");
DEFINE_double(
    ls_tolerance, 1e-6,
    "Tolerance for the exact line search performed by the convex integrator.");
DEFINE_double(tolerance, 1e-8, "Main solver tolerance");
DEFINE_string(error_estimation_strategy, "half_stepping",
              "Error estimation strategy for the convex integrator. "
              "Options are: 'half_stepping', 'sdirk', 'implicit_trapezoid'.");

using drake::geometry::CollisionFilterDeclaration;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::multibody::ContactResults;
using drake::multibody::MultibodyPlant;
using drake::systems::ImplicitEulerIntegrator;
using drake::systems::IntegratorBase;
using Eigen::Translation3d;
using Eigen::Vector3d;
using clock = std::chrono::steady_clock;
using drake::multibody::contact_solvers::internal::SapHessianFactorizationType;
using drake::multibody::contact_solvers::internal::SapSolverParameters;
using drake::multibody::internal::CompliantContactManager;
using drake::systems::ConvexIntegrator;
using drake::systems::ConvexIntegratorSolverParameters;
using drake::visualization::ApplyVisualizationConfig;
using drake::visualization::VisualizationConfig;

// Parameters
const double width(0.8);
const double length(0.8);

std::vector<geometry::GeometryId> box_geometry_ids;

const RigidBody<double>& AddBox(const std::string& name,
                                const Vector3<double>& block_dimensions,
                                double mass, double stiffness,
                                double static_friction, double dynamic_friction,
                                const Vector4<double>& color,
                                bool emulate_box_multicontact,
                                bool add_box_collision,
                                MultibodyPlant<double>* plant) {
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
  const RigidTransform<double> X_BG;  // Identity transform.
  plant->RegisterVisualGeometry(box, X_BG, geometry::Box(LBx, LBy, LBz),
                                name + "_visual", color);

  // When the TAMSI solver is used, we simply let MultibodyPlant estimate
  // contact parameters based on penetration_allowance and stiction_tolerance.
  geometry::ProximityProperties props;
  props.AddProperty(geometry::internal::kMaterialGroup,
                    geometry::internal::kPointStiffness, stiffness);
  props.AddProperty(geometry::internal::kMaterialGroup, "relaxation_time",
                    FLAGS_dissipation_time_constant);
  props.AddProperty("material", "hunt_crossley_dissipation",
                    FLAGS_hc_dissipation);

  if (FLAGS_use_hydro) {
    if (stiffness == kHuge) {
      AddRigidHydroelasticProperties(1.0 /* Not used for boxes */, &props);
    } else {
      AddCompliantHydroelasticProperties(1.0 /* Not used for boxes */,
                                         stiffness, &props);
    }
  }

  props.AddProperty(geometry::internal::kMaterialGroup,
                    geometry::internal::kFriction,
                    CoulombFriction<double>(static_friction, dynamic_friction));

  // Box's collision geometry is a solid box.
  if (emulate_box_multicontact) {
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    const Vector4<double> red_50(1.0, 0.0, 0.0, 0.5);
    const double radius_x = LBx / FLAGS_num_spheres_per_face / 2.0;
    const double radius_y = LBy / FLAGS_num_spheres_per_face / 2.0;
    const double radius_z = LBz / FLAGS_num_spheres_per_face / 2.0;
    double dx = 2 * radius_x;
    double dy = 2 * radius_y;
    double dz = 2 * radius_z;
    const int ns = FLAGS_num_spheres_per_face;

    auto add_sphere = [&](const std::string& sphere_name, double x, double y,
                          double z, double radius) {
      const Vector3<double> p_BoSpherei_B(x, y, z);
      const RigidTransform<double> X_BSpherei(p_BoSpherei_B);
      geometry::Sphere shape(radius);
      // Ellipsoid might not be accurate. From console [warning]:
      // "Ellipsoid is primarily for ComputeContactSurfaces in
      // hydroelastic contact model. The accuracy of other collision
      // queries and signed distance queries are not guaranteed."
      // geometry::Ellipsoid shape(radius_x, radius_y, radius_z);
      plant->RegisterCollisionGeometry(box, X_BSpherei, shape, sphere_name,
                                       props);
    };

    // Add points (zero size spheres) at the corners to avoid spurious
    // interpentrations between boxes and the sink.
    if (FLAGS_add_box_corners) {
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
    auto id = plant->RegisterCollisionGeometry(
        box, X_BG, geometry::Box(LBx, LBy, LBz), name + "_collision", props);
    box_geometry_ids.push_back(id);
  }
  return box;
}

void AddSink(MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  // Parameters for the sink.
  // const double length = 1.0;
  // const double width = 0.8;
  const double height = 0.8;
  const double wall_thickness = 0.04;
  const double wall_mass = 1.0;
  const double static_friction = FLAGS_static_friction;
  const double dynamic_friction = FLAGS_dynamic_friction;
  const Vector4<double> light_blue(0.5, 0.8, 1.0, 0.3);
  const Vector4<double> transparent(0., 0.0, 0.0, 0.0);

  auto add_wall =
      [&](const std::string& name, const Vector3d& dimensions,
          const RigidTransformd& X_WB,
          const Vector4<double>& color) -> const RigidBody<double>& {
    const double kSinkStiffness = kHuge;
    const auto& wall =
        AddBox(name, dimensions, wall_mass, kSinkStiffness, static_friction,
               dynamic_friction, color, false, true, plant);
    plant->WeldFrames(plant->world_frame(), wall.body_frame(), X_WB);
    return wall;
  };

  const Vector3d bottom_dimensions(50 * length, 50 * width, wall_thickness);
  const Vector3d side_wall_dimensions(height, width, wall_thickness);
  const Vector3d back_front_wall_dimensions(length, wall_thickness, height);

  add_wall("sink_bottom", bottom_dimensions,
           Translation3d(0, 0, -wall_thickness / 2.0), transparent);

  if (FLAGS_add_sink_walls) {
    add_wall("sink_right", side_wall_dimensions,
             RigidTransformd(RotationMatrixd::MakeYRotation(M_PI_2),
                             Vector3d(length / 2.0, 0.0, height / 2.0)),
             light_blue);
    add_wall("sink_left", side_wall_dimensions,
             RigidTransformd(RotationMatrixd::MakeYRotation(M_PI_2),
                             Vector3d(-length / 2.0, 0.0, height / 2.0)),
             light_blue);
    add_wall("sink_back", back_front_wall_dimensions,
             Translation3d(0.0, width / 2, height / 2), light_blue);
    add_wall("sink_front", back_front_wall_dimensions,
             Translation3d(0.0, -width / 2, height / 2), light_blue);
  }
}

const RigidBody<double>& AddSphere(const std::string& name, const double radius,
                                   double mass, double static_friction,
                                   double dynamic_friction,
                                   const Vector4<double>& color,
                                   MultibodyPlant<double>* plant) {
  const UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  const SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody(name, M_Bcm);

  geometry::ProximityProperties props;
  props.AddProperty(geometry::internal::kMaterialGroup,
                    geometry::internal::kPointStiffness,
                    FLAGS_sphere_stiffness);
  props.AddProperty(geometry::internal::kMaterialGroup, "relaxation_time",
                    FLAGS_dissipation_time_constant);
  props.AddProperty(geometry::internal::kMaterialGroup,
                    geometry::internal::kFriction,
                    CoulombFriction<double>(static_friction, dynamic_friction));
  props.AddProperty("material", "hunt_crossley_dissipation",
                    FLAGS_hc_dissipation);

  if (FLAGS_use_hydro) {
    AddCompliantHydroelasticProperties(FLAGS_sphere_resolution,
                                       FLAGS_sphere_stiffness, &props);
  }

  // Add collision geometry.
  const RigidTransformd X_BS = RigidTransformd::Identity();
  plant->RegisterCollisionGeometry(ball, X_BS, geometry::Sphere(radius),
                                   name + "_collision", props);

  // Add visual geometry.
  plant->RegisterVisualGeometry(ball, X_BS, geometry::Sphere(radius),
                                name + "_visual", color);

  // We add a few spots so that we can appreciate the sphere's
  // rotation, colored on red, green, blue according to the body's axes.
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
  const Vector4<double> green(0.0, 1.0, 0.0, 1.0);
  const Vector4<double> blue(0.0, 0.0, 1.0, 1.0);
  const double visual_radius = 0.2 * radius;
  const geometry::Cylinder spot(visual_radius, visual_radius);
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

std::vector<BodyIndex> AddObjects(double scale_factor,
                                  MultibodyPlant<double>* plant) {
  const double radius0 = 0.05;
  const double density = FLAGS_density;  // kg/m^3.
  // const double mass = 0.2;

  const double static_friction = FLAGS_static_friction;
  const double dynamic_friction = FLAGS_dynamic_friction;
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
    if (!FLAGS_enable_boxes) {
      return 0;  // All boxes.
    } else {
      return distribution(generator);
    }
  };

  const int num_objects = FLAGS_objects_per_pile;
  const int num_bodies = plant->num_bodies();

  std::vector<BodyIndex> bodies;
  for (int i = 1; i <= num_objects; ++i) {
    const auto& color = colors[(i - 1) % colors.size()];
    const std::string name = "object" + std::to_string(i + num_bodies);

    double e = FLAGS_scale_factor > 0 ? i - 1 : num_objects - i;
    double scale = std::pow(std::abs(FLAGS_scale_factor), e);

    switch (roll_shape()) {
      case 0: {
        const double radius = radius0 * scale;
        const double volume = 4. / 3. * M_PI * radius * radius * radius;
        const double mass = density * volume;
        bodies.push_back(AddSphere(name, radius, mass, static_friction,
                                   dynamic_friction, color, plant)
                             .index());
        break;
      }
      case 1: {
        const Vector3d box_size = 2 * radius0 * Vector3d::Ones() * scale;
        const double volume = box_size(0) * box_size(1) * box_size(2);
        const double mass = density * volume;
        Vector4<double> color50(color);
        color50.z() = 0.5;
        bodies.push_back(AddBox(name, box_size, mass, FLAGS_box_stiffness,
                                static_friction, dynamic_friction, color50,
                                FLAGS_emulate_box_multicontact, true, plant)
                             .index());
        break;
      }
    }
    scale *= scale_factor;
  }

  return bodies;
}

void SetObjectsIntoAPile(const MultibodyPlant<double>& plant,
                         const Vector3d& offset,
                         const std::vector<BodyIndex>& bodies,
                         systems::Context<double>* plant_context) {
  const double delta_z = FLAGS_dz;  // assume objects have a BB of about 10 cm.

  const int seed = 41;
  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> distribution(-1e-3, 1e-3);

  int num_objects = FLAGS_objects_per_pile;

  double z = delta_z / 2;
  int i = 1;
  for (auto body_index : bodies) {
    const auto& body = plant.get_body(body_index);
    if (body.is_floating()) {
      double e = FLAGS_scale_factor > 0 ? i - 1 : num_objects - i;
      double scale = std::pow(std::abs(FLAGS_scale_factor), e);

      if (FLAGS_random_offsets) {
        const RotationMatrixd R_WB =
            math::UniformlyRandomRotationMatrix<double>(&generator);
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
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_mbp_time_step;
  plant_config.discrete_contact_approximation =
      FLAGS_discrete_contact_approximation;
  plant_config.sap_near_rigid_threshold = FLAGS_near_rigid_threshold;
  plant_config.stiction_tolerance = FLAGS_stiction_tolerance;
  plant_config.contact_model = FLAGS_use_hydro ? "hydroelastic" : "point";
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlant(plant_config, &builder);

  AddSink(&plant);

  // Add objects to the scene.
  DRAKE_DEMAND(FLAGS_num_piles > 0 && FLAGS_num_piles <= 4);
  std::vector<std::vector<BodyIndex>> piles;
  for (int i = 0; i < FLAGS_num_piles; ++i) {
    piles.push_back(AddObjects(FLAGS_scale_factor, &plant));
  }

  // Only box-sphere and sphere-sphere are allowed.
  if (!FLAGS_enable_box_box_collision) {
    geometry::GeometrySet all_boxes(box_geometry_ids);
    scene_graph.collision_filter_manager().Apply(
        CollisionFilterDeclaration().ExcludeWithin(all_boxes));
  }

  plant.Finalize();

  fmt::print("Num positions: {:d}\n", plant.num_positions());
  fmt::print("Num velocities: {:d}\n", plant.num_velocities());

  // Publish contact results for visualization.
  std::shared_ptr<drake::geometry::Meshcat> meshcat{nullptr};
  if (FLAGS_visualize) {
    meshcat = std::make_shared<drake::geometry::Meshcat>();
    VisualizationConfig vis_config;
    vis_config.publish_period = FLAGS_viz_period;
    vis_config.publish_contacts = FLAGS_visualize_forces;
    ApplyVisualizationConfig(vis_config, &builder, nullptr, &plant,
                             &scene_graph, meshcat);
  }
  auto diagram = builder.Build();

  if (FLAGS_dense_algebra && plant.is_discrete()) {
    // Use dense algebra for SAP
    SapSolverParameters sap_parameters;
    sap_parameters.linear_solver_type = SapHessianFactorizationType::kDense;

    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>();
    CompliantContactManager<double>* contact_manager =
        owned_contact_manager.get();
    plant.SetDiscreteUpdateManager(std::move(owned_contact_manager));
    contact_manager->set_sap_solver_parameters(sap_parameters);
  }

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // In the plant's default context, we assume the state of body B in world W is
  // such that X_WB is an identity transform and B's spatial velocity is zero.
  plant.SetDefaultContext(&plant_context);

  // Set the initial position of the objects.
  for (int i = 0; i < FLAGS_num_piles; ++i) {
    const double x = (i % 2 == 0) ? -length / 4 : length / 4;
    const double y = (i / 2 == 0) ? -width / 4 : width / 4;

    SetObjectsIntoAPile(plant, Vector3d(x, y, 0.0), piles[i], &plant_context);
  }

  auto simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  drake::systems::IntegratorBase<double>& integrator =
      simulator->get_mutable_integrator();
  if (FLAGS_simulator_integration_scheme == "convex") {
    auto& ci = dynamic_cast<ConvexIntegrator<double>&>(integrator);
    ci.set_plant(&plant);

    ConvexIntegratorSolverParameters ci_params;
    ci_params.enable_hessian_reuse = FLAGS_enable_hessian_reuse;
    ci_params.max_iterations_for_hessian_reuse = FLAGS_k_max;
    ci_params.kappa = FLAGS_kappa;
    ci_params.alpha_max = FLAGS_alpha_max;
    ci_params.ls_tolerance = FLAGS_ls_tolerance;
    ci_params.tolerance = FLAGS_tolerance;
    ci_params.log_solver_stats = FLAGS_log_solver_stats;
    ci_params.print_solver_stats = FLAGS_print_solver_stats;
    ci_params.use_dense_algebra = FLAGS_dense_algebra;
    ci_params.error_estimation_strategy = FLAGS_error_estimation_strategy;
    ci.set_solver_parameters(ci_params);

  } else if (FLAGS_simulator_integration_scheme == "implicit_euler") {
    auto& ie = dynamic_cast<ImplicitEulerIntegrator<double>&>(integrator);
    using JacobianComputationScheme =
        ImplicitEulerIntegrator<double>::JacobianComputationScheme;
    if (FLAGS_integrator_jacobian_scheme == "forward") {
      ie.set_jacobian_computation_scheme(
          JacobianComputationScheme::kForwardDifference);
    } else if (FLAGS_integrator_jacobian_scheme == "central") {
      ie.set_jacobian_computation_scheme(
          JacobianComputationScheme::kCentralDifference);
    } else if (FLAGS_integrator_jacobian_scheme == "automatic") {
      ie.set_jacobian_computation_scheme(JacobianComputationScheme::kAutomatic);
    } else {
      throw std::logic_error("Invalid jacobian scheme");
    }
    ie.set_use_full_newton(FLAGS_full_newton);
    ie.set_use_implicit_trapezoid_error_estimation(FLAGS_trapezoid);
  }

  simulator->set_publish_every_time_step(true);
  simulator->Initialize();
  if (FLAGS_visualize) {
    // Wait for meshcat to load
    std::cout << "Press [ENTER] to continue ...\n";
    getchar();

    const double recording_frames_per_second =
        FLAGS_mbp_time_step == 0 ? 32 : 1.0 / FLAGS_mbp_time_step;
    meshcat->StartRecording(recording_frames_per_second);
  }

  clock::time_point sim_start_time = clock::now();

  CALLGRIND_START_INSTRUMENTATION;
  // CALLGRIND_TOGGLE_COLLECT;  // Start collection
  simulator->AdvanceTo(FLAGS_simulation_time);
  // CALLGRIND_TOGGLE_COLLECT;  // Stop collection
  CALLGRIND_STOP_INSTRUMENTATION;

  clock::time_point sim_end_time = clock::now();
  const double sim_time =
      std::chrono::duration<double>(sim_end_time - sim_start_time).count();
  std::cout << "AdvanceTo() time [sec]: " << sim_time << std::endl;

  if (FLAGS_visualize) {
    meshcat->StopRecording();
    meshcat->PublishRecording();
  }

  PrintSimulatorStatistics(*simulator);

  if (FLAGS_visualize) {
    // Wait for meshcat to finish rendering.
    std::cout << "Press [ENTER] to quit ...\n";
    getchar();
  }

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "\nSimulation of a clutter of objects falling into a box container.");

  // Set some reasonable defaults for the simulator options (these can be
  // overridden from the command line).
  FLAGS_simulator_integration_scheme = "convex";
  FLAGS_simulator_accuracy = 0.1;
  FLAGS_simulator_max_time_step = 0.01;
  FLAGS_simulator_use_error_control = true;

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::examples::do_main();
}
