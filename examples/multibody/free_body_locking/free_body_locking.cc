#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace examples {
namespace free_body_locking {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time (usually between 0 and 1). "
              "This is documented in Simulator::set_target_realtime_rate().");
DEFINE_double(time_step, 1.0E-3,
              "If time_step > 0, the fixed-time step period (in seconds) of "
              "discrete updates for the plant (modeled as a discrete system). "
              "If time_step = 0, the plant is modeled as a continuous system "
              "and no contact forces are displayed.  time_step must be >= 0.");
DEFINE_double(integration_accuracy, 1.0E-6,
              "When time_step = 0 (plant is modeled as a continuous system), "
              "this is the desired integration accuracy.  This value is not "
              "used if time_step > 0 (fixed-time step).");
DEFINE_double(penetration_allowance, 1.0E-5, "Allowable penetration (meters).");
DEFINE_double(stiction_tolerance, 1.0E-3,
              "Allowable drift speed during stiction (m/s).");
DEFINE_double(inclined_plane_angle_degrees, 15.0,
              "Inclined plane angle (degrees), i.e., angle from Wx to Ax.");
DEFINE_double(inclined_plane_coef_static_friction, 0.5,
              "Inclined plane's coefficient of static friction (no units).");
DEFINE_double(inclined_plane_coef_kinetic_friction, 0.5,
              "Inclined plane's coefficient of kinetic friction (no units).  "
              "When time_step > 0, this value is ignored.  Only the "
              "coefficient of static friction is used in fixed-time step.");
DEFINE_double(bodyB_coef_static_friction, 0.5,
              "Body B's coefficient of static friction (no units).");
DEFINE_double(bodyB_coef_kinetic_friction, 0.5,
              "Body B's coefficient of kinetic friction (no units).  "
              "When time_step > 0, this value is ignored.  Only the "
              "coefficient of static friction is used in fixed-time step.");
DEFINE_bool(is_inclined_plane_half_space, true,
            "Is inclined plane a half-space (true) or box (false).");
DEFINE_int32(num_bodies, 8, "number of free bodies");
DEFINE_double(simulation_interval, 1.0,
              "Interval between actions in the simulation.");

using drake::multibody::MultibodyPlant;

int do_main() {
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));

  const drake::multibody::CoulombFriction<double> coef_friction_bodyB(
      FLAGS_bodyB_coef_static_friction, FLAGS_bodyB_coef_kinetic_friction);
  const drake::multibody::CoulombFriction<double> coef_friction_inclined_plane(
      FLAGS_inclined_plane_coef_static_friction,
      FLAGS_inclined_plane_coef_kinetic_friction);

  const math::RigidTransform<double> X_WC(Vector3<double>::Zero());
  const math::RigidTransform<double> X_WA(
      math::RotationMatrixd::MakeXRotation(-0.2));
  const Vector4<double> white(0.9, 0.9, 0.9, 1.0);
  const Vector4<double> blue(1.0, 0.55, 0.0, 1.0);

  geometry::ProximityProperties props;
  props.AddProperty(geometry::internal::kMaterialGroup,
                    geometry::internal::kFriction, coef_friction_bodyB);
  props.AddProperty(geometry::internal::kHydroGroup,
                    geometry::internal::kElastic, 1e5);
  props.AddProperty(geometry::internal::kHydroGroup,
                    geometry::internal::kRezHint, 0.2);
  props.AddProperty(geometry::internal::kHydroGroup,
                    geometry::internal::kComplianceType,
                    geometry::internal::HydroelasticType::kSoft);

  plant.RegisterVisualGeometry(plant.world_body(), X_WA,
                               geometry::Box(10, 10, 1),
                               "InclinedPlaneVisualGeometry", white);
  plant.RegisterCollisionGeometry(plant.world_body(), X_WA,
                                  geometry::Box(10, 10, 1),
                                  "InclinedPlaneCollisionGeometry", props);

  // Make the cylinder bodies
  SpatialInertia<double> body_inertia(
      0.5, Vector3<double>::Zero(),
      UnitInertia<double>::TriaxiallySymmetric(0.1));
  const auto& body = plant.AddRigidBody("cylinder1", body_inertia);
  plant.RegisterVisualGeometry(body, X_WC, geometry::Cylinder(0.5, 0.2),
                               "cylinder1_viz", blue);
  plant.RegisterCollisionGeometry(body, X_WC, geometry::Cylinder(0.5, 0.2),
                                  "cylinder1_col", props);

  // Make cylinders and joints
  for (int i = 2; i <= FLAGS_num_bodies; ++i) {
    const auto& cylinder =
        plant.AddRigidBody(fmt::format("cylinder{}", i), body_inertia);
    plant.RegisterVisualGeometry(cylinder, X_WC, geometry::Cylinder(0.5, 0.2),
                                 fmt::format("cylinder{}_viz", i), blue);
    plant.RegisterCollisionGeometry(cylinder, X_WC,
                                    geometry::Cylinder(0.5, 0.2),
                                    fmt::format("cylinder{}_col", i), props);

    const auto& cylinder_prev =
        plant.GetBodyByName(fmt::format("cylinder{}", i - 1));

    const auto& joint_const = plant.AddJoint<QuaternionFloatingJoint>(
        fmt::format("{}_{}", i - 1, i), cylinder_prev, {}, cylinder, {}, 0.0,
        0.0);
    auto& joint = plant.GetMutableJointByName<QuaternionFloatingJoint>(
        joint_const.name());
    joint.SetDefaultPose(
        math::RigidTransform<double>(Vector3<double>(0, 0, -0.22)));
  }

  plant.set_contact_model(ContactModel::kPoint);

  plant.Finalize();
  plant.set_penetration_allowance(FLAGS_penetration_allowance);

  // Set the speed tolerance (m/s) for the underlying Stribeck friction model
  // (the allowable drift speed during stiction).  For two points in contact,
  // this is the maximum sliding speed for the points to be regarded as
  // stationary relative to each other (so that static friction is used).
  plant.set_stiction_tolerance(FLAGS_stiction_tolerance);

  // Publish contact results for visualization.
  // TODO(Mitiguy) Ensure contact forces can be displayed when time_step = 0.
  if (FLAGS_time_step > 0)
    ConnectContactResultsToDrakeVisualizer(&builder, plant, scene_graph);

  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // In the plant's default context, we assume the state of body B in world W
  // is such that X_WB is an identity transform and B's spatial velocity is
  // zero.
  plant.SetDefaultContext(&plant_context);

  // Overwrite B's default initial position so it is somewhere above the
  // inclined plane provided `0 < inclined_plane_angle < 40`.
  const auto& bodyB = plant.GetBodyByName("cylinder1");
  const math::RigidTransform<double> X_WB(
      math::RotationMatrix<double>::MakeXRotation(0.5 * M_PI),
      Vector3<double>(0, -2, 1.75));
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, bodyB, X_WB);

  for (int i = 2; i <= FLAGS_num_bodies; ++i) {
    auto& joint = plant.GetJointByName<QuaternionFloatingJoint>(
        fmt::format("{}_{}", i - 1, i));
    joint.Lock(&plant_context);
  }

  bodyB.Lock(&plant_context);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  systems::IntegratorBase<double>& integrator =
      simulator.get_mutable_integrator();

  // Set the integration accuracy when the plant is integrated with a
  // variable- step integrator. This value is not used if time_step > 0
  // (fixed-time step).
  integrator.set_target_accuracy(FLAGS_integration_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_simulation_interval);

  for (int i = 0; i < FLAGS_num_bodies - 1; ++i) {
    auto& joint = plant.GetJointByName<QuaternionFloatingJoint>(
        fmt::format("{}_{}", FLAGS_num_bodies - i - 1, FLAGS_num_bodies - i));
    joint.Unlock(&plant_context);
    simulator.AdvanceTo((i + 2) * FLAGS_simulation_interval);
  }

  bodyB.Unlock(&plant_context);

  simulator.AdvanceTo((FLAGS_num_bodies + 1) * FLAGS_simulation_interval);

  return 0;
}

}  // namespace
}  // namespace free_body_locking
}  // namespace examples
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::examples::free_body_locking::do_main();
}
