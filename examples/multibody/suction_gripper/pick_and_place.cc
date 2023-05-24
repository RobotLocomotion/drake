#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/multibody/suction_gripper/example_gripper_multibody_model.h"
#include "drake/examples/multibody/suction_gripper/suction_force_model.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake::examples::multibody::suction_gripper {

int do_main() {
  // All values are in SI units.
  // --------------------------Multibody model--------------------------
  const double kMultibodyTimeStep = 0.002;
  drake::systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<drake::multibody::MultibodyPlant<double>>(
                    kMultibodyTimeStep));

  auto& world_body = plant.world_body();

  // Add two conveyor belts
  const double kConveyorMass = 100.;
  const double kConveyorLen = 5.;
  const double kConveyorWidth = 0.6;
  const double kConveyorHeight = 0.2;
  const double kFrictionCoeff = 0.6;

  auto& conveyor1_body = plant.AddRigidBody(
      /*name*/ "conveyor1_body",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          /*m*/ kConveyorMass,
          /*p_PScm_E*/ Eigen::Vector3d::Zero(),
          /*G_SP_E*/ drake::multibody::UnitInertia<double>(1., 1., 1.)));

  plant.RegisterVisualGeometry(
      /*body*/ conveyor1_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Box(kConveyorWidth, kConveyorLen, kConveyorHeight),
      /*name*/ "conveyor1_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.2, 0.2, 0.2, 1.));
  plant.RegisterCollisionGeometry(
      /*body*/ conveyor1_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Box(kConveyorWidth, kConveyorLen, kConveyorHeight),
      /*name*/ "conveyor1_collision_goem",
      /*coulomb_friction*/
      drake::multibody::CoulombFriction(
          /*static_friction */ kFrictionCoeff,
          /* dynamic_friction*/ kFrictionCoeff));

  plant.WeldFrames(world_body.body_frame(), conveyor1_body.body_frame(),
                   /*X_AB*/
                   drake::math::RigidTransform<double>(
                       Eigen::Vector3d(0, 0, -kConveyorHeight / 2)));

  auto& conveyor2_body = plant.AddRigidBody(
      /*name*/ "conveyor2_body",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          /*m*/ kConveyorMass,
          /*p_PScm_E*/ Eigen::Vector3d::Zero(),
          /*G_SP_E*/ drake::multibody::UnitInertia<double>(1., 1., 1.)));

  plant.RegisterVisualGeometry(
      /*body*/ conveyor2_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Box(kConveyorWidth, kConveyorLen, kConveyorHeight),
      /*name*/ "conveyor2_visual_geom",
      /*diffuse_color*/ Eigen::Vector4d(0.2, 0.2, 0.2, 1.));
  plant.RegisterCollisionGeometry(
      /*body*/ conveyor2_body,
      /*X_BG*/ drake::math::RigidTransform<double>(),
      /*shape*/
      drake::geometry::Box(kConveyorWidth, kConveyorLen, kConveyorHeight),
      /*name*/ "conveyor2_collision_goem",
      /*coulomb_friction*/
      drake::multibody::CoulombFriction(
          /*static_friction */ kFrictionCoeff,
          /* dynamic_friction*/ kFrictionCoeff));

  const double kConveyorDist = 1.;
  plant.WeldFrames(world_body.body_frame(), conveyor2_body.body_frame(),
                   /*X_AB*/
                   drake::math::RigidTransform<double>(Eigen::Vector3d(
                       kConveyorDist, 0, -kConveyorHeight / 2)));

  // Add an object (cardboard box)
  const double kPackageMass = 2.73;
  const double kPackageLen = 0.3;
  const double kPackageWidth = 0.15;
  drake::multibody::CoulombFriction obj_friction(
      /*static_friction */ kFrictionCoeff,
      /* dynamic_friction*/ kFrictionCoeff);

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

  // Add contact spheres (not needed if using hydroelastic contact model)
  std::vector<Eigen::Vector3d> corner_points = {
      Eigen::Vector3d(-kPackageLen / 2, -kPackageWidth / 2, -kPackageLen / 2),
      Eigen::Vector3d(-kPackageLen / 2, kPackageWidth / 2, -kPackageLen / 2),
      Eigen::Vector3d(kPackageLen / 2, -kPackageWidth / 2, -kPackageLen / 2),
      Eigen::Vector3d(kPackageLen / 2, kPackageWidth / 2, -kPackageLen / 2),
  };
  const double kContactSphereRadius = 1e-3;
  for (size_t corner_idx = 0; corner_idx < corner_points.size(); corner_idx++) {
    plant.RegisterCollisionGeometry(
        obj_body,
        drake::math::RigidTransform<double>(corner_points[corner_idx]),
        drake::geometry::Sphere(kContactSphereRadius),
        "obj_sphere_goem" + std::to_string(corner_idx), obj_friction);
  }

  // Add a wrist
  const double kWristMass = 1.0;
  const double kWristInertia = 0.01;
  auto& wrist_body1 = plant.AddRigidBody(
      /*name*/ "wrist_body1",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          kWristMass, Eigen::Vector3d::Zero(),
          drake::multibody::UnitInertia<double>(kWristInertia, kWristInertia,
                                                kWristInertia)));

  const double kWristJointLowerLimit = 0.;
  const double kWristJointUpperLimit = 1.;
  const double kWristJointDamping = 10.;
  auto& wrist_joint1 = plant.AddJoint<drake::multibody::PrismaticJoint>(
      /*name*/ "wrist_joint1",
      /*parent*/ world_body,
      /*X_PF*/ drake::math::RigidTransform<double>(),
      /*child*/ wrist_body1,
      /*X_BM*/ drake::math::RigidTransform<double>(),
      /*axis*/ Eigen::Vector3d::UnitX(),
      /*pos_lower_limit*/ kWristJointLowerLimit,
      /*pos_upper_limit*/ kWristJointUpperLimit,
      /*damping*/ kWristJointDamping);
  plant.AddJointActuator("wrist_joint1_actuator", wrist_joint1);

  auto& wrist_body2 = plant.AddRigidBody(
      /*name*/ "wrist_body2",
      /*inertia*/ drake::multibody::SpatialInertia<double>(
          kWristMass, Eigen::Vector3d::Zero(),
          drake::multibody::UnitInertia<double>(kWristInertia, kWristInertia,
                                                kWristInertia)));

  auto& wrist_joint2 = plant.AddJoint<drake::multibody::PrismaticJoint>(
      /*name*/ "wrist_joint2",
      /*parent*/ wrist_body1,
      /*X_PF*/ drake::math::RigidTransform<double>(),
      /*child*/ wrist_body2,
      /*X_BM*/ drake::math::RigidTransform<double>(),
      /*axis*/ Eigen::Vector3d::UnitZ(),
      /*pos_lower_limit*/ kWristJointLowerLimit,
      /*pos_upper_limit*/ kWristJointUpperLimit,
      /*damping*/ kWristJointDamping);
  plant.AddJointActuator("wrist_joint2_actuator", wrist_joint2);

  // Add a suction gripper
  ExampleGripperMultibodyModel suction_gripper(&plant, wrist_body2);
  auto suction_cup_act_pt_geom_id_vec =
      suction_gripper.get_suction_cup_act_pt_geom_id_vec();
  auto suction_cup_act_pt_geom_id_to_body_idx_map =
      suction_gripper.get_suction_cup_act_pt_geom_id_to_body_idx_map();
  auto suction_cup_edge_pt_geom_id_vec =
      suction_gripper.get_suction_cup_edge_pt_geom_id_vec();

  plant.set_discrete_contact_solver(
      drake::multibody::DiscreteContactSolver::kSap);
  plant.Finalize();

  // --------------------------Motion model--------------------------
  Eigen::VectorXd time_way_pts(7);
  time_way_pts << 0, 2, 4, 6, 10, 12, 14;

  const double kWristToToolTipOffset = 0.357;
  const double kCupEngageOffset = 0.01;
  const double kPlaceHeight = 0.35;
  Eigen::MatrixXd wrist_xz_traj_way_pts(2, 7);
  wrist_xz_traj_way_pts << kWristJointLowerLimit, kWristJointLowerLimit,
      kWristJointLowerLimit, kWristJointLowerLimit, kWristJointUpperLimit,
      kWristJointUpperLimit, kWristJointUpperLimit, kWristJointUpperLimit,
      kWristJointUpperLimit,
      kPackageLen + kWristToToolTipOffset - kCupEngageOffset,
      kWristJointUpperLimit, kWristJointUpperLimit,
      kPlaceHeight + kWristToToolTipOffset - kCupEngageOffset,
      kWristJointUpperLimit;

  auto wrist_xz_traj =
      drake::trajectories::PiecewisePolynomial<double>::CubicShapePreserving(
          time_way_pts, wrist_xz_traj_way_pts);
  auto& wrist_xz_traj_src =
      *builder.AddSystem<drake::systems::TrajectorySource<double>>(
          wrist_xz_traj, 1);

  auto& pid =
      *builder.AddSystem<drake::systems::controllers::PidController<double>>(
          /*state selection matrix*/ plant.MakeStateSelectorMatrix(
              {wrist_joint1.index(), wrist_joint2.index()}),
          /*Kp*/ Eigen::VectorXd::Constant(2, 1, 1e4),
          /*Ki*/ Eigen::VectorXd::Constant(2, 1, 1e3),
          /*Kd*/ Eigen::VectorXd::Constant(2, 1, 1e2));

  builder.Connect(wrist_xz_traj_src.get_output_port(),
                  pid.get_input_port_desired_state());
  builder.Connect(plant.get_state_output_port(),
                  pid.get_input_port_estimated_state());
  builder.Connect(
      pid.get_output_port_control(),
      plant.get_actuation_input_port(drake::multibody::ModelInstanceIndex(1)));

  // --------------------------Suction model--------------------------
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

  // --------------------------Simulation--------------------------
  drake::systems::Simulator<double> simulator(*diagram_ptr);
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());

  plant.SetFreeBodyPose(&plant_context, obj_body,
                        drake::math::RigidTransform<double>(
                            Eigen::Vector3d(0, 0, kPackageLen / 2)));

  wrist_joint1.set_translation(&plant_context, wrist_xz_traj_way_pts(0, 0));
  wrist_joint2.set_translation(&plant_context, wrist_xz_traj_way_pts(1, 0));

  auto& suction_pressure_source_context =
      suction_pressure_source.GetMyMutableContextFromRoot(
          &simulator.get_mutable_context());
  suction_pressure_source.GetSuctionCmdInputPort().FixValue(
      &suction_pressure_source_context,
      drake::systems::BasicVector<double>({0}));

  simulator.set_target_realtime_rate(1);
  simulator.Initialize();

  simulator.AdvanceTo(time_way_pts(2));
  // Turn on suction to grab the object
  suction_pressure_source.GetSuctionCmdInputPort().FixValue(
      &suction_pressure_source_context,
      drake::systems::BasicVector<double>({1}));

  simulator.AdvanceTo(time_way_pts(5));
  // Turn off suction to release the object
  suction_pressure_source.GetSuctionCmdInputPort().FixValue(
      &suction_pressure_source_context,
      drake::systems::BasicVector<double>({0}));

  simulator.AdvanceTo(time_way_pts(6));
  return EXIT_SUCCESS;
}
}  // namespace drake::examples::multibody::suction_gripper

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A demo showing pick and place with a suction gripper.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::suction_gripper::do_main();
}
