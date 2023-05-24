#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/multibody/suction_gripper/example_gripper_multibody_model.h"
#include "drake/examples/multibody/suction_gripper/suction_force_model.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake::examples::multibody::suction_gripper {

DEFINE_double(simulation_time, 15.0,
              "Duration of the simulation (in seconds).");

DEFINE_double(initial_angle, M_PI / 4,
              "Initial angle of the package swing (in radian).");

int do_main() {
  const double kMultibodyTimeStep = 0.002;
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<drake::multibody::MultibodyPlant<double>>(
                    kMultibodyTimeStep));

  auto& world_body = plant.world_body();

  // Add an object (cardboard box)
  const double kFrictionCoeff = 1.0;
  drake::multibody::CoulombFriction obj_friction(
      /*static_friction */ kFrictionCoeff,
      /* dynamic_friction*/ kFrictionCoeff);

  const double kPackageMass = 2.73;
  const double kPackageLen = 0.3;
  const double kPackageWidth = 0.15;
  auto& obj_body = plant.AddRigidBody(
      /*name*/ "obj_body",
      /*inertia*/ drake::multibody::SpatialInertia<double>::SolidBoxWithMass(
          kPackageMass, kPackageLen, kPackageWidth, kPackageLen));

  auto obj_body_collision_geom = plant.RegisterCollisionGeometry(
      /*body*/ obj_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/ drake::geometry::Box(kPackageLen, kPackageWidth, kPackageLen),
      /*name*/ "obj_body_collision_geom",
      /*coulomb_friction*/ obj_friction);

  plant.RegisterVisualGeometry(
      /*body*/ obj_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/ drake::geometry::Box(kPackageLen, kPackageWidth, kPackageLen),
      /*name*/ "obj_body_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.6, 0.4, 0.1, 1.0));

  // Add a wrist
  const double kWristMass = 1.0;
  const double kWristInertia = 0.01;
  const double kWristHeight = 1.0;
  auto& wrist_body = plant.AddRigidBody(
      /*name*/ "wrist_body",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          kWristMass, Eigen::Vector3d::Zero(),
          drake::multibody::UnitInertia<double>(kWristInertia, kWristInertia,
                                                kWristInertia)));

  plant.AddJoint<drake::multibody::WeldJoint>(
      "wrist_joint", world_body, drake::math::RigidTransform<double>(),
      wrist_body, drake::math::RigidTransform<double>(),
      drake::math::RigidTransform<double>(
          Eigen::Vector3d(0., 0., kWristHeight)));

  // Add a suction gripper
  ExampleGripperMultibodyModel suction_gripper(&plant, wrist_body);
  auto suction_cup_act_pt_geom_id_vec =
      suction_gripper.get_suction_cup_act_pt_geom_id_vec();
  auto suction_cup_act_pt_geom_id_to_body_idx_map =
      suction_gripper.get_suction_cup_act_pt_geom_id_to_body_idx_map();
  auto suction_cup_edge_pt_geom_id_vec =
      suction_gripper.get_suction_cup_edge_pt_geom_id_vec();

  plant.set_discrete_contact_solver(
      drake::multibody::DiscreteContactSolver::kSap);
  plant.Finalize();

  const double kPumpPressure = -9e4;
  const double kMaxSuctionDist = 0.004;
  const int kNumSuctionCup = 1;
  const double kSuctionModelTimeStep = 0.01;

  auto& suction_pressure_source = *builder.AddSystem<CupPressureSource>(
      kPumpPressure, kMaxSuctionDist, kNumSuctionCup);

  std::unordered_map<drake::geometry::GeometryId, drake::multibody::BodyIndex>
      obj_geom_id_to_body_idx_map = {
          {obj_body_collision_geom, obj_body.index()}};
  auto& cup_obj_interface = *builder.AddSystem<CupObjInterface>(
      kSuctionModelTimeStep, suction_gripper.CalcCupArea(),
      suction_cup_act_pt_geom_id_vec,
      suction_cup_act_pt_geom_id_to_body_idx_map,
      suction_cup_edge_pt_geom_id_vec, obj_geom_id_to_body_idx_map);

  builder.Connect(suction_pressure_source.GetSuctionCupPressureOutputPort(),
                  cup_obj_interface.GetSuctionCupPressureInputPort());
  builder.Connect(cup_obj_interface.GetCupObjDistOutputPort(),
                  suction_pressure_source.GetCupObjDistInputPort());
  builder.Connect(scene_graph.get_query_output_port(),
                  cup_obj_interface.GetGeomQueryInputPort());
  builder.Connect(cup_obj_interface.GetSuctionForceOutputPort(),
                  plant.get_applied_spatial_force_input_port());

  visualization::AddDefaultVisualization(&builder);

  auto diagram_ptr = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram_ptr);
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());

  const double kCupCenterHeightFromGround = 0.623;
  const double kCupEngageOffset = 0.01;
  plant.SetFreeBodyPose(
      &plant_context, obj_body,
      drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw<double>(0., FLAGS_initial_angle, 0.),
          Eigen::Vector3d(-kPackageLen / 2 * sin(FLAGS_initial_angle), 0,
                          kCupCenterHeightFromGround + kCupEngageOffset -
                              kPackageLen / 2 * cos(FLAGS_initial_angle))));

  auto& suction_pressure_source_context =
      suction_pressure_source.GetMyMutableContextFromRoot(
          &simulator.get_mutable_context());
  suction_pressure_source.GetSuctionCmdInputPort().FixValue(
      &suction_pressure_source_context,
      drake::systems::BasicVector<double>({1}));

  simulator.set_target_realtime_rate(1);
  simulator.Initialize();

  simulator.AdvanceTo(FLAGS_simulation_time);

  return EXIT_SUCCESS;
}
}  // namespace drake::examples::multibody::suction_gripper

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A demo showing the package grabbed by a suction cup swinging if "
      "released from a non-zero initial angle.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::suction_gripper::do_main();
}
