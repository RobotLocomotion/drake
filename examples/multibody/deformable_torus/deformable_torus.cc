#include <iostream>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(simulation_time, 12.0, "Desired duration of the simulation [s].");
DEFINE_double(realtime_rate, 1.0, "Desired real time rate.");
DEFINE_double(time_step, 1e-2,
              "Discrete time step for the system [s]. Must be positive.");
DEFINE_double(E, 5e4, "Young's modulus of the deformable body [Pa].");
DEFINE_double(nu, 0.4, "Poisson's ratio of the deformable body, unitless.");
DEFINE_double(density, 1e3, "Mass density of the deformable body [kg/m³].");
DEFINE_double(max_suction_force_density, 2e5,
              "Max suction force density [N/m³].");
DEFINE_double(suction_radius, 0.1, "Max suction force affected distance [m].");
DEFINE_double(beta, 0.01,
              "Stiffness damping coefficient for the deformable body [1/s].");

using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::Mesh;
using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::Body;
using drake::multibody::CoulombFriction;
using drake::multibody::DeformableBodyId;
using drake::multibody::DeformableModel;
using drake::multibody::ExternalForceField;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::fem::DeformableBodyConfig;
using drake::multibody::internal::MultibodyTreeSystem;
using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace {

class PointSourceForceField : public ExternalForceField<double> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointSourceForceField)

  /**
   * @brief Construct a new Point Source Force Field object
   *
   * @param plant The owning MultibodyPlant
   * @param body The body to which the source of the force field is attached to.
   * @param p_BC  The fixed offset from the body origin to the point source of
   * the force field.
   * @param max_value The maximum magnitude of the force density.
   * @param distance  The force density decays linearly. `distance` in meters is
   * the distance from the point source beyond which the force is zero.
   * @param pick_time The time at which the force field is activated.
   * @param drop_time The time at which the force field is deactivated.
   */
  PointSourceForceField(const MultibodyPlant<double>& plant,
                        const Body<double>& body, const Vector3<double>& p_BC,
                        double max_value, double distance, double pick_time,
                        double drop_time)
      : plant_(&plant),
        body_(&body),
        p_BC_(p_BC),
        max_value_(max_value),
        distance_(distance),
        pick_time_(pick_time),
        drop_time_(drop_time) {}

  const systems::InputPort<double>& get_scale_input_port() const {
    return parent_system_or_throw().get_input_port(scale_port_index_);
  }

 private:
  /* Computes the world frame position of the center of the point source force
   field. */
  Vector3<double> CalcPointSourceLocation(
      const systems::Context<double>& context) const {
    return plant_->EvalBodyPoseInWorld(context, *body_) * p_BC_;
  }

  Vector3<double> DoEvaluateAt(const systems::Context<double>& context,
                               const Vector3<double>& p_WQ) const final {
    const Vector3<double>& p_WC = EvalPointSource(context);
    Vector3<double> p_QC_W = p_WC - p_WQ;
    const double dist = p_QC_W.norm();
    if (dist == 0 || dist > distance_) {
      return Vector3<double>::Zero();
    }
    const double magnitude =
        GetScale(context) * (distance_ - dist) * max_value_ / distance_;
    return magnitude * p_QC_W / p_QC_W.norm();
  }

  std::unique_ptr<ExternalForceField<double>> DoClone() const final {
    return std::make_unique<PointSourceForceField>(*this);
  }

  /**
   * We store the center force field as a cache entry so we don't need to
   * repeatedly compute it.
   *
   * @param tree_system The system underlying the plant that owns the cache
   * entry.
   */
  void DoDeclareCacheEntries(MultibodyPlant<double>* tree_system) final {
    point_source_position_cache_index_ =
        this->DeclareCacheEntry(
                tree_system, "point source of the force field",
                systems::ValueProducer(
                    this, &PointSourceForceField::CalcPointSourceLocation),
                {systems::System<double>::xd_ticket()})
            .cache_index();
  }

  void DoDeclareInputPorts(MultibodyPlant<double>* tree_system) final {
    scale_port_index_ =
        this->DeclareVectorInputPort(tree_system, "scaling for suction force",
                                     drake::systems::BasicVector<double>(1))
            .get_index();
  }

  const Vector3<double>& EvalPointSource(
      const systems::Context<double>& context) const {
    return parent_system_or_throw()
        .get_cache_entry(point_source_position_cache_index_)
        .template Eval<Vector3<double>>(context);
  }

  /**
   * Returns the scale of suction force.
   */
  double GetScale(const systems::Context<double>& context) const {
    return get_scale_input_port().Eval<BasicVector<double>>(context)[0];
  }

  const MultibodyPlant<double>* plant_;
  const Body<double>* body_;
  Vector3<double> p_BC_;
  double max_value_{};
  double distance_{};
  double pick_time_{};
  double drop_time_{};
  systems::CacheIndex point_source_position_cache_index_;
  systems::InputPortIndex scale_port_index_;
};

class SuctionCupController : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SuctionCupController);

  /* Constructs a SuctionCupController system with the given parameters.
   @param[in] initial_height Initial height of the gripper.
   @param[in] pick_height    The height at suction turns on.
   @param[in] source_pressure  The pressure of the vacuum source.
   @param[in] max_suction_dist The distance that the suction falls off to zero.
   @param[in] pick_time        The time at which to start suction.
   @param[in] travel_time      The time it takes for the gripper to travel from
                               the initial height to the pick height.
   @param[in] lift_time        The time at which to start raising the gripper.
   @param[in] drop_time        The time at which to turn off the suction. */
  SuctionCupController(double initial_height,
             double pick_height, double source_pressure,
             double max_suction_dist, double pick_time, double travel_time,
             double lift_time, double drop_time)
      : initial_height_(initial_height),
        pick_height_(pick_height),
        source_pressure_(source_pressure),
        max_suction_dist_(max_suction_dist),
        pick_time_(pick_time),
        travel_time_(travel_time),
        lift_time_(lift_time),
        drop_time_(drop_time) {
    desired_state_port_index_ =
        DeclareVectorOutputPort("desired state",
                                drake::systems::BasicVector<double>(2),
                                &SuctionCupController::CalcDesiredState)
            .get_index();
    scale_port_index_ =
        DeclareVectorOutputPort("scaling of suction force",
                                drake::systems::BasicVector<double>(1),
                                &SuctionCupController::CalcScale)
            .get_index();
  }

  const systems::OutputPort<double>& desired_state_output_port() const {
    return get_output_port(desired_state_port_index_);
  }

  const systems::OutputPort<double>& scale_output_port() const {
    return get_output_port(scale_port_index_);
  }

 private:
  void CalcDesiredState(const systems::Context<double>& context,
                        systems::BasicVector<double>* desired_state) const {
    const double t = context.get_time();
    /* Time to start lowering the gripper. */
    const double lower_time = pick_time_ - travel_time_;
    /* Time to start raising the gripper. */
    const double raise_time = lift_time_ - travel_time_;
    if (t < lower_time) {
      desired_state->set_value(Eigen::Vector2d(initial_height_, 0));
    } else if (t < pick_time_) {
      const double v = (pick_height_ - initial_height_) / travel_time_;
      const double dt = t - lower_time;
      desired_state->set_value(Eigen::Vector2d(initial_height_ + dt * v, v));
    } else if (t < raise_time) {
      desired_state->set_value(Eigen::Vector2d(pick_height_, 0));
    } else if (t < lift_time_) {
      const double v = (initial_height_ - pick_height_) / travel_time_;
      const double dt = t - raise_time;
      desired_state->set_value(Eigen::Vector2d(pick_height_ + dt * v, v));
    } else {
      desired_state->set_value(Eigen::Vector2d(initial_height_, 0));
    }
  }

  /* Turn the suction on and off based on time. */
  void CalcScale(const systems::Context<double>& context,
                 systems::BasicVector<double>* scale) const {
    const double time = context.get_time();
    if (time >= pick_time_ && time <= drop_time_) {
      (*scale)[0] = 1.0;
    } else {
      (*scale)[0] = 0.0;
    }
  }

  double initial_height_{};
  double pick_height_{};
  double source_pressure_{};
  double max_suction_dist_{};
  double pick_time_{};
  double travel_time_{};
  double lift_time_{};
  double drop_time_{};

  int desired_state_port_index_{};
  int scale_port_index_{};
};

int do_main() {
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_time_step;
  /* Deformable simulation only works with SAP solver. */
  plant_config.discrete_contact_solver = "sap";

  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  const auto M = SpatialInertia<double>::SolidCubeWithMass(0.1, 0.05);
  ModelInstanceIndex model_instance = plant.AddModelInstance("instance");
  const auto& body = plant.AddRigidBody("body", model_instance, M);
  Capsule box{0.02, 0.1};
  IllustrationProperties cup_illustration_props;
  cup_illustration_props.AddProperty("phong", "diffuse",
                                     Vector4d(0.9, 0.1, 0.1, 0.8));
  plant.RegisterVisualGeometry(body, RigidTransformd::Identity(), box,
                               "cup_visual", cup_illustration_props);
  const auto& prismatic_joint = plant.AddJoint<PrismaticJoint>(
      "translate_z_joint", plant.world_body(), std::nullopt, body, std::nullopt,
      Vector3d::UnitZ());
  auto actuator_index =
      plant.AddJointActuator("prismatic joint actuator", prismatic_joint)
          .index();

  plant.get_mutable_joint_actuator(actuator_index)
      .set_controller_gains({1e6, 1});

  /* Minimum required proximity properties for rigid bodies to interact with
   deformable bodies.
   1. A valid Coulomb friction coefficient, and
   2. A resolution hint. (Rigid bodies need to be tessellated so that
   collision queries can be performed against deformable geometries.) */
  ProximityProperties rigid_proximity_props;
  /* Set the friction coefficient close to that of rubber against rubber. */
  const CoulombFriction<double> surface_friction(0.15, 0.15);
  AddContactMaterial({}, {}, surface_friction, &rigid_proximity_props);
  rigid_proximity_props.AddProperty(geometry::internal::kHydroGroup,
                                    geometry::internal::kRezHint, 0.01);
  /* Set up a ground. */
  Box ground{4, 4, 4};
  const RigidTransformd X_WG(Eigen::Vector3d{0, 0, -2});
  plant.RegisterCollisionGeometry(plant.world_body(), X_WG, ground,
                                  "ground_collision", rigid_proximity_props);
  IllustrationProperties illustration_props;
  illustration_props.AddProperty("phong", "diffuse",
                                 Vector4d(0.7, 0.5, 0.4, 0.8));
  plant.RegisterVisualGeometry(plant.world_body(), X_WG, ground,
                               "ground_visual", std::move(illustration_props));

  plant.RegisterCollisionGeometry(body, RigidTransformd::Identity(), box,
                                  "suction_collision", rigid_proximity_props);

  /* Set up a deformable torus. */
  auto owned_deformable_model =
      std::make_unique<DeformableModel<double>>(&plant);

  DeformableBodyConfig<double> deformable_config;
  deformable_config.set_youngs_modulus(FLAGS_E);
  deformable_config.set_poissons_ratio(FLAGS_nu);
  deformable_config.set_mass_density(FLAGS_density);
  deformable_config.set_stiffness_damping_coefficient(FLAGS_beta);

  const std::string torus_vtk = FindResourceOrThrow(
      "drake/examples/multibody/deformable_torus/torus.vtk");
  /* Load the geometry and scale it down to 65% (to showcase the scaling
   capability and to make the torus suitable for grasping by the gripper). */
  const double scale = 0.65;
  auto torus_mesh = std::make_unique<Mesh>(torus_vtk, scale);
  /* Minor diameter of the torus inferred from the vtk file. */
  const double kL = 0.09 * scale;
  /* Set the initial pose of the torus such that its bottom face is touching
   the ground. */
  const RigidTransformd X_WB(Vector3<double>(0.06, 0.0, kL / 2.0));
  auto torus_instance = std::make_unique<GeometryInstance>(
      X_WB, std::move(torus_mesh), "deformable_torus");

  /* Minimumly required proximity properties for deformable bodies: A valid
   Coulomb friction coefficient. */
  ProximityProperties deformable_proximity_props;
  AddContactMaterial({}, {}, surface_friction, &deformable_proximity_props);
  torus_instance->set_proximity_properties(deformable_proximity_props);

  /* Registration of all deformable geometries ostensibly requires a
   resolution hint parameter that dictates how the shape is tessellated. In
   the case of a `Mesh` shape, the resolution hint is unused because the shape
   is already tessellated. */
  // TODO(xuchenhan-tri): Though unused, we still asserts the resolution hint
  // is positive. Remove the requirement of a resolution hint for meshed
  // shapes.
  const double unused_resolution_hint = 1.0;
  owned_deformable_model->RegisterDeformableBody(
      std::move(torus_instance), deformable_config, unused_resolution_hint);
  const DeformableModel<double>* deformable_model =
      owned_deformable_model.get();

  const double kInitialHeight = 0.5;
  const double kStartSuctionHeight =
      0.15;                        // The height at which to turn on suction.
  const double kWaitTime = 3.0;    // Time to start the action
  const double kTravelTime = 1.0;  // Time to raise the object to top.
  const double kStartSuctionTime = 3.0;  // Time to turn on suction.
  const double kPickUpTime = 6.0;  // Time to finish picking up the object.
  const double kDropTime = 9.0;    // Time to turn off suction.

  auto suction_force = std::make_unique<PointSourceForceField>(
      plant, body, Vector3d(0, 0, -0.05), FLAGS_max_suction_force_density,
      FLAGS_suction_radius, kStartSuctionTime, kDropTime);
  const PointSourceForceField* suction_force_ptr = suction_force.get();
  owned_deformable_model->AddExternalForce(std::move(suction_force));
  plant.AddPhysicalModel(std::move(owned_deformable_model));

  /* All rigid and deformable models have been added. Finalize the plant. */
  plant.Finalize();

  const auto& suction = *builder.AddSystem<SuctionCupController>(
      kInitialHeight, kStartSuctionHeight, FLAGS_max_suction_force_density,
      FLAGS_suction_radius, kWaitTime, kTravelTime, kPickUpTime, kDropTime);
  builder.Connect(suction.desired_state_output_port(),
                  plant.get_desired_state_input_port(model_instance));

  /* It's essential to connect the vertex position port in DeformableModel to
   the source configuration port in SceneGraph when deformable bodies are
   present in the plant. */
  builder.Connect(
      deformable_model->vertex_positions_port(),
      scene_graph.get_source_configuration_port(plant.get_source_id().value()));
  builder.Connect(suction.scale_output_port(),
                  suction_force_ptr->get_scale_input_port());

  /* Add a visualizer that emits LCM messages for visualization. */
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  /* Set initial conditions for the gripper. */
  auto& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  prismatic_joint.set_translation(&plant_context, 0.5);

  /* Build the simulator and run! */
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.Initialize();
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is a demo used to showcase deformable body simulations in Drake. "
      "A simple parallel gripper grasps a deformable torus on the ground, "
      "lifts it up, and then drops it back on the ground. "
      "Launch meldis before running this example. "
      "Refer to README for instructions on meldis as well as optional flags.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
