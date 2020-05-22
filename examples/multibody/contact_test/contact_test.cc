#include <chrono>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
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

DEFINE_bool(visualize, true,
            "If true, the simulation will publish messages for Drake "
            "visualizer. Useful to turn off during profiling sessions.");

// Contact parameters.
DEFINE_double(dynamic_friction, 1.0,
              "Coefficient of dynamic friction, [-]");
DEFINE_double(static_friction, 1.0,
              "Coefficient of static friction, [-]");              
DEFINE_double(regularization_speed, 1.0e-4,
              "Regularization parameter for the friction model, [m/s]");

namespace drake {
namespace examples {
namespace multibody {
namespace contact_test {
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
using drake::geometry::ProximityProperties;
using drake::geometry::Sphere;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;

const Vector3d green(0, 1, 0);
const Vector3d yellow( 1, 1, 0);

#if 0
template <typename T>
class CompliantContactModel : public systems::LeafSystem<T> {
 public:
  // The model applies the contact force on `body`, assuming its geometry is a
  // sphere of given `radius`. `elastic_modulus` and `dissipation` define the
  // compliant hydroelastic parameters while `dynamic_friction` and
  // `regularization_velocity` defines the regularized friction model as applied
  // by Drake's hydroelastic model implementation (i.e. friction is regularized
  // with an atan function.)
  CompliantContactModel(const Body<T>& body, double radius,
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
                &CompliantContactModel::CalcAnalyticHydroForce)
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
#endif

/// Computes the plane stress effective modulus E* = E/(1-ν²).
double CalcPlaneStrainModulus(double young_modulus, double poisson_ratio) {
  DRAKE_DEMAND(young_modulus >= 0);
  DRAKE_DEMAND(0.0 <= poisson_ratio && poisson_ratio <= 0.5);
  return young_modulus / (1-poisson_ratio * poisson_ratio);
}

ProximityProperties MakeNylonMaterial() { 
  //const double density       = 1100.0;  // kg/m^3
  const double young_modulus = 2.5e9;   // [Pa = N/m²]
  const double poisson_ratio = 0.4;     // [-]
  const double dissipation   = 0.005;   // [s/m]
  const double plane_stress_modulus =
  CalcPlaneStrainModulus(young_modulus, poisson_ratio);
  const CoulombFriction<double> friction(FLAGS_static_friction,
                                         FLAGS_dynamic_friction);
  ProximityProperties properties;
  properties.AddProperty("material", "hertz_modulus", plane_stress_modulus);
  properties.AddProperty("material", "hung_crossley_dissipation", dissipation); 
  properties.AddProperty("material", "coulomb_friction", friction);
  return properties;
}

#if 0
ProximityProperties MakeWallMaterial() {
  ProximityProperties properties;
  properties.AddProperty("material", "coulomb_friction",
                         *friction);
  return properties;
}
#endif

void AddGroundModel(MultibodyPlant<double>* plant) {
  ProximityProperties wall_material = MakeNylonMaterial();

  double box_width = 40.0;
  double half_size = box_width / 2.0;

  // Build geometry such that half-space normlas point into the box.
  const RigidTransformd X_floor;  // Identity.
  const RigidTransformd X_right(RotationMatrixd::MakeXRotation(M_PI_2),
                                Vector3d(0, half_size, 0));
  const RigidTransformd X_left(RotationMatrixd::MakeXRotation(-M_PI_2),
                               Vector3d(0, -half_size, 0));
  const RigidTransformd X_front(RotationMatrixd::MakeYRotation(-M_PI_2),
                                Vector3d(half_size, 0, 0));
  const RigidTransformd X_back(RotationMatrixd::MakeYRotation(M_PI_2),
                               Vector3d(-half_size, 0, 0));

  // Ground contact and visual geometry.
  plant->RegisterCollisionGeometry(plant->world_body(), X_floor,
                                   geometry::HalfSpace{}, "floor_collision",
                                   wall_material);
  plant->RegisterCollisionGeometry(plant->world_body(), X_right,
                                   geometry::HalfSpace{}, "right_collision",
                                   wall_material);
  plant->RegisterCollisionGeometry(plant->world_body(), X_left,
                                   geometry::HalfSpace{}, "left_collision",
                                   wall_material);
  plant->RegisterCollisionGeometry(plant->world_body(), X_front,
                                   geometry::HalfSpace{}, "front_collision",
                                   wall_material);
  plant->RegisterCollisionGeometry(plant->world_body(), X_back,
                                   geometry::HalfSpace{}, "back_collision",
                                   wall_material);

  // Add visual for the ground.
  const Vector4d floor_color = (Vector4d() << green, 1.0).finished();
  const Vector4d walls_color = (Vector4d() << yellow, 0.2).finished();
  const Vector4d front_color = (Vector4d() << yellow, 0.5).finished();
  geometry::Box floor_wall(box_width, box_width, box_width / 200.0);
  geometry::Box right_wall(box_width, box_width / 200.0, 1.5 * box_width);
  plant->RegisterVisualGeometry(plant->world_body(), X_floor, floor_wall,
                                "floor_visual", floor_color);
  plant->RegisterVisualGeometry(plant->world_body(),
                                {RotationMatrixd(), Vector3d(0, half_size, 0)},
                                right_wall, "right_visual", walls_color);
  plant->RegisterVisualGeometry(
      plant->world_body(),
      {RotationMatrixd::MakeZRotation(M_PI), Vector3d(0, -half_size, 0)},
      right_wall, "left_visual", walls_color);
  plant->RegisterVisualGeometry(
      plant->world_body(),
      {RotationMatrixd::MakeZRotation(M_PI_2), Vector3d(-half_size, 0, 0)},
      right_wall, "back_visual", walls_color);
  plant->RegisterVisualGeometry(
      plant->world_body(),
      {RotationMatrixd::MakeZRotation(-M_PI_2), Vector3d(half_size, 0, 0)},
      right_wall, "front_visual", front_color);
}

// Replicates the scenario in one of Simbody's tests:
// Simbody/tests/adhoc/ContactTest.cpp
void BuildSimbodyContactTestModel(MultibodyPlant<double>* plant) {
  AddGroundModel(&*plant);
  
}

int do_main() {
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(0.0));

  BuildSimbodyContactTestModel(&plant);

  if (FLAGS_contact_model == "point") {
    // Plant must be finalized before setting the penetration allowance.
    plant.Finalize();
    // Set how much penetration (in meters) we are willing to accept.
    plant.set_penetration_allowance(0.001);
    plant.set_stiction_tolerance(FLAGS_regularization_speed);
  } else if (FLAGS_contact_model == "compliant") {
    DRAKE_DEMAND(!plant.is_discrete());  // model must be continuous.
    plant.set_contact_model(ContactModel::kNoContact);
    plant.Finalize();
    const auto& body = plant.GetBodyByName("Ball");
#if 0    
    const auto& analytic_contact =
        *builder.AddSystem<CompliantContactModel>(
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
#endif                    
  } else {
    throw std::runtime_error("Invalid contact model '" + FLAGS_contact_model +
                             "'.");
  }

  fmt::print(" nq: {}, nv: {}\n", plant.num_positions(),
             plant.num_velocities());

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

#if 0
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
#endif      

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

  // TODO: loop over bodies and compute "mean/std. dev." of contact penetration
  // towards the end. As a "measure of compliance" to compare simbody's model
  // with out point contact model.

#if 0
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
#endif             

  fmt::print("\n\nSimulation statistics:\n");
  systems::PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace contact_test
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A contact demo using Drake's MultibodyPlant, "
      "with SceneGraph visualization. ");
  // We slow down the default realtime rate to 0.2, so that we can appreciate
  // the motion. Users can still change it on command-line, e.g.
  // --simulator_target_realtime_rate=0.5.
  FLAGS_simulator_target_realtime_rate = 0.2;
  // Simulator default parameters for this demo.
  FLAGS_simulator_accuracy = 1.0e-3;
  FLAGS_simulator_max_time_step = 1.0e-3;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::contact_test::do_main();
}
