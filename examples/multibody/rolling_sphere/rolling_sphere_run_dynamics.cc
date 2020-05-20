#include <chrono>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/random_rotation.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

// Integration parameters.
DEFINE_double(simulation_time, 2.0,
              "Desired duration of the simulation in seconds.");

// Contact model parameters.
DEFINE_string(contact_model, "point",
              "Contact model. Options are: 'point', 'hydroelastic', 'hybrid', "
              "'analytic'.");
DEFINE_double(elastic_modulus, 5.0e4,
              "For hydroelastic (and hybrid) contact, elastic modulus, [Pa].");
DEFINE_double(dissipation, 5.0,
              "For hydroelastic (and hybrid) contact, Hunt & Crossley "
              "dissipation, [s/m].");
DEFINE_double(friction_coefficient, 0.3, "friction coefficient.");
DEFINE_double(regularization_speed, 1.0e-4,
              "Friction regularization speed, [m/s]");
DEFINE_double(resolution_hint, 1.0,
              "Resolution hint for the hydroelastic sphere geometry. Relative "
              "to the radius. Dimensionless.");
DEFINE_bool(rigid_ball, false,
            "If true, the ball is given a rigid hydroelastic representation "
            "(instead of the default soft value). Make sure you have the right "
            "contact model to support this representation.");
DEFINE_bool(soft_ground, false,
            "If true, the ground is given a soft hydroelastic representation "
            "(instead of the default rigid value). Make sure you have the "
            "right contact model to support this representation.");
DEFINE_bool(add_wall, false,
            "If true, adds a wall with soft hydroelastic representation in the "
            "path of the default ball trajectory. This will cause the "
            "simulation to throw when the soft ball hits the wall with the "
            "'hydroelastic' model; use the 'hybrid' or 'point' contact model "
            "to simulate beyond this contact.");

DEFINE_bool(visualize, true,
            "If true, the simulation will publish messages for Drake "
            "visualizer. Useful to turn off during profiling sessions.");

// Sphere's spatial velocity.
DEFINE_double(vx, 1.5,
              "Sphere's initial translational velocity in the x-axis in m/s.");
DEFINE_double(vy, 0.0,
              "Sphere's initial translational velocity in the y-axis in m/s.");
DEFINE_double(vz, 0.0,
              "Sphere's initial translational velocity in the z-axis in m/s.");
DEFINE_double(wx, 0.0,
              "Sphere's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wy, -360.0,
              "Sphere's initial angular velocity in the y-axis in degrees/s.");
DEFINE_double(wz, 0.0,
              "Sphere's initial angular velocity in the y-axis in degrees/s.");

// Sphere's pose.
DEFINE_double(roll, 0.0, "Sphere's initial roll in degrees.");
DEFINE_double(pitch, 0.0, "Sphere's initial pitch in degrees.");
DEFINE_double(yaw, 0.0, "Sphere's initial yaw in degrees.");
DEFINE_double(z0, 0.05, "Sphere's initial position in the z-axis.");

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {
namespace {

using Eigen::Vector3d;
using Eigen::Vector4d;
using drake::geometry::GeometryId;
using drake::geometry::SceneGraph;
using drake::geometry::SourceId;
using drake::lcm::DrakeLcm;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RotationMatrix;
using drake::math::RotationMatrixd;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using drake::multibody::ContactModel;
using drake::multibody::CoulombFriction;
using drake::multibody::ExternallyAppliedSpatialForce;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialForce;
using drake::multibody::SpatialVelocity;
using drake::systems::Context;

// For the case of a soft-sphere in contact with a half-space, the contact force
// in the normal direction can be integrated analytically. Assuming a uniform
// velocity distribution, friction can be approximated using the slip velocity
// at the centroid of the contact patch.
// This system implements this analytical model. It is a stateless system. Its
// inputs are SceneGraph's query port and MultibodyPlant pose and spatial
// velocity ports. Its output consits of the analytical contact force meant to
// be connected into MultibodyPlant's applied spatial force input port.
template <typename T>
class AnalyticHydroelasticModel : public systems::LeafSystem<T> {
 public:
  // The model applies the contact force on `body`, assuming its geometry is a
  // sphere of given `radius`. `elastic_modulus` and `dissipation` define the
  // compliant hydroelastic parameters while `dynamic_friction` and
  // `regularization_velocity` defines the regularized friction model as applied
  // by Drake's hydroelastic model implementation (i.e. friction is regularized
  // with an atan function.)
  AnalyticHydroelasticModel(const Body<T>& body, double radius,
                            double elastic_modulus, double dissipation,
                            double dynamic_friction,
                            double regularization_velocity)
      : body_index_(body.index()),
        radius_(radius),
        elastic_modulus_(elastic_modulus),
        dissipation_(dissipation),
        dynamic_friction_(dynamic_friction),
        regularization_velocity_(regularization_velocity) {
    geometry_query_port_ =
        this->DeclareAbstractInputPort("geometry_query",
                                       Value<geometry::QueryObject<T>>{})
            .get_index();
    body_poses_port_ =
        this->DeclareAbstractInputPort(
                "body_poses", Value<std::vector<math::RigidTransform<T>>>())
            .get_index();
    body_spatial_velocities_port_ =
        this->DeclareAbstractInputPort("body_spatial_velocities",
                                       Value<std::vector<SpatialVelocity<T>>>())
            .get_index();
    spatial_forces_port_ =
        this->DeclareAbstractOutputPort(
                "spatial_forces",
                &AnalyticHydroelasticModel::CalcAnalyticHydroForce)
            .get_index();
  }

  /// Port to perform point contact queries with SceneGraph.
  const systems::InputPort<T>& get_geometry_query_input_port() const {
    return this->get_input_port(geometry_query_port_);
  }

  /// Port to obtain the sphere's pose from MultibodyPlant.
  const systems::InputPort<T>& get_body_poses_input_port() const {
    return this->get_input_port(body_poses_port_);
  }

  /// Port to obtain the sphere's spatial velocity from MultibodyPlant.
  const systems::InputPort<T>& get_body_spatial_velocities_input_port() const {
    return this->get_input_port(body_spatial_velocities_port_);
  }

  /// Output port with the analytical spatial force due to hydroelastic contact.
  /// It is meant to be connected to MultibodyPlant's applied spatial forces
  /// input port.
  const systems::OutputPort<T>& get_spatial_forces_output_port() const {
    return this->get_output_port(spatial_forces_port_);
  }

 private:
  void CalcAnalyticHydroForce(
      const Context<T>& context,
      std::vector<ExternallyAppliedSpatialForce<T>>* output) const {
    const std::vector<geometry::PenetrationAsPointPair<T>> pairs =
        CalcPointPairs(context);
    // Only applicable to the example in this file, for the sphere vs.
    // half-space case.
    DRAKE_DEMAND(pairs.size() <= 1);
    if (pairs.size() == 1) {
      const auto& pair = pairs[0];
      const T depth = pair.depth;
      if (depth < 0) {
        output->clear();
        return;
      }
      // We use the penetration extend field e(r) = 1 - r / R, where `r` is the
      // radial spherical coordinate and `R` is the radius of the sphere. The
      // normal force is the integral of the pressure p(r) = E e(r) over the
      // circular patch. Given the axial symmetry about the center of the patch,
      // we can write this integral as:
      //   P = 2π∫dρ⋅ρ⋅p(r(ρ))
      // with `ρ` the radial (2D) coordinate in the plane of the patch and `r`
      // as before the spherical coordinate.
      const T fn0 = M_PI / 3.0 * elastic_modulus_ * depth * depth *
                    (3 - 2 * depth / radius_);

      // Body kinematics
      const RigidTransform<T>& X_WB = EvalBodyPose(context);
      const RotationMatrix<T>& R_WB = X_WB.rotation();
      const Vector3<T>& p_WB = X_WB.translation();
      const SpatialVelocity<T>& V_WB = EvalBodySpatialVelocity(context);

      // Since the plane is rigid, we know the position of the centroid C is the
      // position of the sphere projected the ground's half space.
      const Vector3<T> p_WC(p_WB(0), p_WB(1), 0.0);
      const Vector3<T> p_BC_W = p_WC - p_WB;
      const Vector3<T> p_BC_B = R_WB.transpose() * p_BC_W;

      // Damping.
      const SpatialVelocity<T> V_WC = V_WB.Shift(p_BC_W);
      const T depth_dot = -V_WC.translational()(2);

      using std::max;
      const T fn = fn0 * max(0.0, 1.0 + depth_dot * dissipation_);

      // In this case we know we have contact between the soft sphere and the
      // plane. Therefore the force vector, at the centroid of the patch C,
      // applied on the sphere is:
      const Vector3<T> fn_Bc_W = fn * Vector3<T>::UnitZ();

      // Friction.
      const Vector3<T>& v_WC = V_WC.translational();
      const Vector3<T> vt_WC(v_WC(0), v_WC(1), 0.0);
      const T sliding_speed = vt_WC.norm();
      const T regularized_friction =
          dynamic_friction_ *
          CalcAtanXOverX(sliding_speed / regularization_velocity_) * 2.0 /
          M_PI / regularization_velocity_ * fn;  // [Ns/m].
      const Vector3<T> ft_Bc_W = -regularized_friction * vt_WC;

      // Torques at C: hydro produces two torques for free:
      // 1. Rolling damping; due to damping in the normal direction.
      // 2. Torsional friction; due to angular velocity of the patch.
      // For this analytical version, we will neglect these effects since we
      // mostly interested on the stiffness introduced by compliance in the
      // normal direction and regularized friction.
      const Vector3<T> t_Bc_W = Vector3<T>::Zero();

      output->resize(1 /* number of forces */);
      (*output)[0].body_index = body_index_;
      (*output)[0].p_BoBq_B = p_BC_B;
      (*output)[0].F_Bq_W = SpatialForce<T>(t_Bc_W, fn_Bc_W + ft_Bc_W);
    }
  }

  const RigidTransform<T>& EvalBodyPose(const Context<T>& context) const {
    const auto& poses_port =
        systems::System<T>::get_input_port(body_poses_port_);
    return poses_port.template Eval<std::vector<math::RigidTransform<T>>>(
        context)[body_index_];
  }

  const SpatialVelocity<T>& EvalBodySpatialVelocity(
      const Context<T>& context) const {
    const auto& velocities_port =
        systems::System<T>::get_input_port(body_spatial_velocities_port_);
    return velocities_port.template Eval<std::vector<SpatialVelocity<T>>>(
        context)[body_index_];
  }

  std::vector<geometry::PenetrationAsPointPair<T>> CalcPointPairs(
      const Context<T>& context) const {
    const auto& query_port =
        systems::System<T>::get_input_port(geometry_query_port_);
    const auto& query_object =
        query_port.template Eval<geometry::QueryObject<T>>(context);
    return query_object.ComputePointPairPenetration();
  }

  // Computes atan(x)/x. The computation is continuously valid even at x = 0
  // (modulo machine epsilon).
  // TODO(amcastro-tri): move this improvement into the actual hydro
  // implementation.
  static T CalcAtanXOverX(const T& x) {
    using std::abs;
    using std::atan;
    if (abs(x) < 1.0e-4) {
      // The taylor expansions for y(x) atan(x)/x and its first derivatives are:
      //   y(x) = 1 - x²/3 +  x⁴/5 + O(x⁶), O(x⁶)  = x⁶/7
      //   y'(x) =   -2x/3 + 4x³/5 + O(x⁵), O'(x⁵) = 6x⁵/7
      // We can compute the relative error in the approxmation as the quotient
      // of the leading terms in the Taylor expansion with the actual value.
      // At x = 1e-4 this amount to the following error estimations:
      // y(x) / O(x⁶)   = 1.43e-25
      // y'(x) / O'(x⁵) = 1.29e-16
      // which is below (double) machine epsilon for both.
      // Therefore CalcAtanXOverX() is continuous "within machine
      // precision".
      const T x2 = x * x;
      return 1.0 - x2 * (1.0 / 3.0 - x2 / 5.0);
    } else {
      return atan(x) / x;
    }
  }

  BodyIndex body_index_;
  double radius_;
  double elastic_modulus_;
  double dissipation_;
  double dynamic_friction_{0.0};
  double regularization_velocity_{1.0e-4};

  systems::InputPortIndex body_poses_port_;
  systems::InputPortIndex body_spatial_velocities_port_;
  systems::InputPortIndex geometry_query_port_;
  systems::OutputPortIndex spatial_forces_port_;
};

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Plant's parameters.
  const double radius = 0.05;   // m
  const double mass = 0.1;      // kg
  const double g = 9.81;        // m/s^2
  const double z0 = FLAGS_z0;        // Initial height.
  const CoulombFriction<double> coulomb_friction(
      FLAGS_friction_coefficient /* static friction */,
      FLAGS_friction_coefficient /* dynamic friction */);

  MultibodyPlant<double>& plant = *builder.AddSystem(MakeBouncingBallPlant(
      radius, mass, FLAGS_elastic_modulus, FLAGS_dissipation, coulomb_friction,
      -g * Vector3d::UnitZ(), FLAGS_rigid_ball, FLAGS_soft_ground,
      FLAGS_resolution_hint,
      &scene_graph));

  if (FLAGS_add_wall) {
    geometry::Box wall{0.2, 4, 0.4};
    const RigidTransformd X_WB(Vector3d{-0.5, 0, 0});
    geometry::ProximityProperties prox_prop;
    geometry::AddContactMaterial(1e8, {}, CoulombFriction<double>(),
                                 &prox_prop);
    geometry::AddSoftHydroelasticProperties(0.1, &prox_prop);
    plant.RegisterCollisionGeometry(plant.world_body(), X_WB, wall,
                                    "wall_collision", std::move(prox_prop));

    geometry::IllustrationProperties illus_prop;
    illus_prop.AddProperty("phong", "diffuse", Vector4d(0.7, 0.5, 0.4, 0.5));
    plant.RegisterVisualGeometry(plant.world_body(), X_WB, wall, "wall_visual",
                                 std::move(illus_prop));
  }

  // Set contact model and parameters.
  if (FLAGS_contact_model == "hydroelastic") {
    plant.set_contact_model(ContactModel::kHydroelasticsOnly);
    plant.Finalize();
  } else if (FLAGS_contact_model == "point") {
    // Plant must be finalized before setting the penetration allowance.
    plant.Finalize();
    // Set how much penetration (in meters) we are willing to accept.
    plant.set_penetration_allowance(0.001);
  } else if (FLAGS_contact_model == "hybrid") {
    plant.set_contact_model(ContactModel::kHydroelasticWithFallback);
    plant.Finalize();
    plant.set_penetration_allowance(0.001);
  } else if (FLAGS_contact_model == "analytic") {
    DRAKE_DEMAND(!plant.is_discrete());  // model must be continuous.
    plant.set_contact_model(ContactModel::kNoContact);
    plant.Finalize();
    const auto& body = plant.GetBodyByName("Ball");
    const auto& analytic_contact =
        *builder.AddSystem<AnalyticHydroelasticModel>(
            body, radius, FLAGS_elastic_modulus, FLAGS_dissipation,
            FLAGS_friction_coefficient, FLAGS_regularization_speed);
    // Analytic hydro forces system input ports.
    builder.Connect(scene_graph.get_query_output_port(),
                    analytic_contact.get_geometry_query_input_port());
    builder.Connect(plant.get_body_poses_output_port(),
                    analytic_contact.get_body_poses_input_port());
    builder.Connect(plant.get_body_spatial_velocities_output_port(),
                    analytic_contact.get_body_spatial_velocities_input_port());
    // Analytic hydro forces system output ports.
    builder.Connect(analytic_contact.get_spatial_forces_output_port(),
                    plant.get_applied_spatial_force_input_port());
  } else {
    throw std::runtime_error("Invalid contact model '" + FLAGS_contact_model +
                             "'.");
  }

  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!plant.get_source_id());

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  if (FLAGS_visualize) {
    geometry::ConnectDrakeVisualizer(&builder, scene_graph);
    ConnectContactResultsToDrakeVisualizer(&builder, plant);
  }
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set the sphere's initial pose.
  math::RotationMatrixd R_WB(math::RollPitchYawd(
      M_PI / 180.0 * Vector3<double>(FLAGS_roll, FLAGS_pitch, FLAGS_yaw)));
  math::RigidTransformd X_WB(R_WB, Vector3d(0.0, 0.0, z0));
  plant.SetFreeBodyPose(
      &plant_context, plant.GetBodyByName("Ball"), X_WB);

  const SpatialVelocity<double> V_WB(Vector3d(FLAGS_wx, FLAGS_wy, FLAGS_wz),
                                     Vector3d(FLAGS_vx, FLAGS_vy, FLAGS_vz));
  plant.SetFreeBodySpatialVelocity(
      &plant_context, plant.GetBodyByName("Ball"), V_WB);

  auto simulator =
      systems::MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  using clock = std::chrono::steady_clock;
  const clock::time_point start = clock::now();
  simulator->AdvanceTo(FLAGS_simulation_time);
  const clock::time_point end = clock::now();
  const double wall_clock_time =
      std::chrono::duration<double>(end - start).count();
  fmt::print("\nSimulator::AdvanceTo() wall clock time: {:.4g} seconds.\n",
             wall_clock_time);

  // We print out the position of the sphere so that we can compare the state of
  // the system when running the same simulation with diffent modeling and
  // simulator parameters.
  const RigidTransformd& X_WB_end =
      plant.EvalBodyPoseInWorld(plant_context, plant.GetBodyByName("Ball"));
  const RotationMatrixd R_WB_end = X_WB_end.rotation();
  const Vector3d p_WB_end = X_WB_end.translation();
  const SpatialVelocity<double>& V_WB_end =
      plant.EvalBodySpatialVelocityInWorld(plant_context,
                                           plant.GetBodyByName("Ball"));
  const Vector3d rpy_end = math::RollPitchYawd(R_WB_end).vector();
  const Vector3d& w_WB_end = V_WB_end.rotational();
  const Vector3d& v_WB_end = V_WB_end.translational();
  fmt::print("\n\nState of the sphere after {:.4g} simulated seconds:\n",
             FLAGS_simulation_time);
  fmt::print("  rpy: {:.4g} {:.4g} {:.4g}\n", rpy_end(0), rpy_end(1),
             rpy_end(2));
  fmt::print("  p_WB: {:.4g} {:.4g} {:.4g}\n", p_WB_end(0), p_WB_end(1),
             p_WB_end(2));
  fmt::print("  w_WB: {:.4g} {:.4g} {:.4g}\n", w_WB_end(0), w_WB_end(1),
             w_WB_end(2));
  fmt::print("  v_WB: {:.4g} {:.4g} {:.4g}\n", v_WB_end(0), v_WB_end(1),
             v_WB_end(2));

  fmt::print("\n\nSimulation statistics:\n");
  systems::PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A rolling sphere demo using Drake's MultibodyPlant, "
      "with SceneGraph visualization. This demo allows to switch between "
      "different contact models and integrators to evaluate performance."
      "Launch drake-visualizer before running this example.");
  // We slow down the default realtime rate to 0.2, so that we can appreciate
  // the motion. Users can still change it on command-line, e.g.
  // --simulator_target_realtime_rate=0.5.
  FLAGS_simulator_target_realtime_rate = 0.2;
  // Simulator default parameters for this demo.
  FLAGS_simulator_accuracy = 1.0e-3;
  FLAGS_simulator_max_time_step = 1.0e-3;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::bouncing_ball::do_main();
}
