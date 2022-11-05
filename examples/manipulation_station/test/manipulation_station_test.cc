#include "drake/examples/manipulation_station/manipulation_station.h"

#include <map>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

using Eigen::Vector2d;
using Eigen::VectorXd;
using geometry::internal::DummyRenderEngine;
using math::RigidTransform;
using multibody::ExternallyAppliedSpatialForce;
using multibody::RevoluteJoint;
using multibody::SpatialForce;
using systems::BasicVector;
using systems::Context;

// Calculate the spatial inertia of the set S of bodies that make up the gripper
// about Go (the gripper frame's origin), expressed in the gripper frame G.
// The rigid bodies in set S consist of the gripper body G, the left finger, and
// the right finger. For this calculation, the sliding joints associated with
// the fingers are regarded as being in a "zero" configuration.
// @param[in] wsg_sdf_path path to sdf file that when parsed creates the model.
// @param[in] gripper_body_frame_name Name of the frame attached to the
//            gripper's main body.
// @retval M_SGo_G spatial inertia of set S about Go, expressed in frame G.
// @note This function helps unit test the calculation of the gripper's spatial
//   inertia done in CalcGripperSpatialInertia() in manipulation_station.cc.
multibody::SpatialInertia<double> MakeCompositeGripperInertia() {
  // Set timestep to 1.0 since it is arbitrary, to quiet joint limit warnings.
  multibody::MultibodyPlant<double> plant(1.0);
  multibody::Parser parser(&plant);
  const std::string& wsg_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_description/sdf/"
      "schunk_wsg_50_no_tip.sdf");
  parser.AddModels(wsg_sdf_path);
  plant.Finalize();
  const std::string gripper_body_frame_name = "body";
  const auto& frame = plant.GetFrameByName(gripper_body_frame_name);
  const auto& gripper_body = plant.GetRigidBodyByName(frame.body().name());
  const auto& left_finger = plant.GetRigidBodyByName("left_finger");
  const auto& right_finger = plant.GetRigidBodyByName("right_finger");
  const auto& left_slider = plant.GetJointByName("left_finger_sliding_joint");
  const auto& right_slider = plant.GetJointByName("right_finger_sliding_joint");
  const multibody::SpatialInertia<double>& M_GGo_G =
      gripper_body.default_spatial_inertia();
  const multibody::SpatialInertia<double>& M_LLo_L =
      left_finger.default_spatial_inertia();
  const multibody::SpatialInertia<double>& M_RRo_R =
      right_finger.default_spatial_inertia();
  auto calc_finger_pose_in_gripper_frame =
      [](const multibody::Joint<double>& slider) {
    // Pose of the joint's parent frame P (attached on gripper body G) in the
    // frame of the gripper G.
    const RigidTransform<double> X_GP(
        slider.frame_on_parent().GetFixedPoseInBodyFrame());
    // Pose of the joint's child frame C (attached on the slider's finger body)
    // in the frame of the slider's finger F.
    const RigidTransform<double> X_FC(
        slider.frame_on_child().GetFixedPoseInBodyFrame());
    // When the slider's translational dof is zero, then P coincides with C.
    // Therefore:
    const RigidTransform<double> X_GF = X_GP * X_FC.inverse();
    return X_GF;
  };
  // Pose of left finger L in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GL(
      calc_finger_pose_in_gripper_frame(left_slider));
  // Pose of right finger R in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GR(
      calc_finger_pose_in_gripper_frame(right_slider));
  // Helper to compute the spatial inertia of a finger F about the gripper's
  // origin Go, expressed in G.
  auto calc_finger_spatial_inertia_in_gripper_frame =
      [](const multibody::SpatialInertia<double>& M_FFo_F,
         const RigidTransform<double>& X_GF) {
        const auto M_FFo_G = M_FFo_F.ReExpress(X_GF.rotation());
        const auto p_FoGo_G = -X_GF.translation();
        const auto M_FGo_G = M_FFo_G.Shift(p_FoGo_G);
        return M_FGo_G;
      };
  // Shift and re-express in G frame the finger's spatial inertias.
  const auto M_LGo_G =
      calc_finger_spatial_inertia_in_gripper_frame(M_LLo_L, X_GL);
  const auto M_RGo_G =
      calc_finger_spatial_inertia_in_gripper_frame(M_RRo_R, X_GR);
  // With everything about the same point Go and expressed in the same frame G,
  // proceed to compose into composite body C:
  // TODO(amcastro-tri): Implement operator+() in SpatialInertia.
  multibody::SpatialInertia<double> M_CGo_G = M_GGo_G;
  M_CGo_G += M_LGo_G;
  M_CGo_G += M_RGo_G;
  return M_CGo_G;
}

void SetArbitraryState(const ManipulationStation<double>& station,
                       Context<double>* context,
                       const VectorXd& iiwa_position,
                       const VectorXd& iiwa_velocity) {
  station.SetIiwaPosition(context, iiwa_position);
  station.SetIiwaVelocity(context, iiwa_velocity);

  station.GetInputPort("iiwa_position").FixValue(context, iiwa_position);
  station.GetInputPort("iiwa_feedforward_torque").FixValue(context,
                                                           VectorXd::Zero(7));
  double wsg_position = station.GetWsgPosition(*context);
  station.GetInputPort("wsg_position").FixValue(context, wsg_position);
  station.GetInputPort("wsg_force_limit").FixValue(context, 40.);

  // Set desired position to actual position and the desired velocity to the
  // actual velocity.
  const auto& position_to_state = dynamic_cast<
      const systems::StateInterpolatorWithDiscreteDerivative<double>&>(
      station.GetSubsystemByName("desired_state_from_position"));
  auto& position_to_state_context =
      station.GetMutableSubsystemContext(position_to_state, context);
  position_to_state.set_initial_state(&position_to_state_context, iiwa_position,
                                      iiwa_velocity);
  // Ensure that integral terms are zero.
  context->get_mutable_continuous_state_vector().SetZero();
}

VectorXd GetNextIiwaVelocity(const ManipulationStation<double>& station,
                             Context<double>* context,
                             const std::string& iiwa_joint_name) {
  auto next_state = station.AllocateDiscreteVariables();
  station.CalcForcedDiscreteVariableUpdate(*context, next_state.get());

  const auto& plant = station.get_multibody_plant();
  const auto& base_joint =
      plant.GetJointByName<multibody::RevoluteJoint>(iiwa_joint_name);
  const int iiwa_velocity_start =
      plant.num_positions() + base_joint.velocity_start();
  return station.GetSubsystemDiscreteValues(plant, *next_state).value()
      .segment<7>(iiwa_velocity_start);
}

void ApplyExternalForceToManipulator(ManipulationStation<double>* station,
                                     Context<double>* context) {
    DRAKE_DEMAND(station != nullptr);
    DRAKE_DEMAND(context != nullptr);
    auto& plant = station->get_multibody_plant();
    std::vector<ExternallyAppliedSpatialForce<double>> external_forces(1);
    external_forces[0].F_Bq_W = SpatialForce<double>(
        Vector3<double>::Zero(), Vector3<double>::UnitZ() * 10.);
    external_forces[0].p_BoBq_B.setZero();
    external_forces[0].body_index = plant.GetBodyByName("body").index();

    station->GetInputPort("applied_spatial_force").FixValue(
        context, external_forces);
}

GTEST_TEST(ManipulationStationTest, CheckPlantBasics) {
  ManipulationStation<double> station(0.001);
  station.SetupManipulationClassStation();
  multibody::Parser parser(&station.get_mutable_multibody_plant(),
                           &station.get_mutable_scene_graph());
  parser.AddModels(FindResourceOrThrow(
      "drake/examples/manipulation_station/models/061_foam_brick.sdf"));
  station.Finalize();

  auto& plant = station.get_multibody_plant();
  EXPECT_EQ(plant.num_actuated_dofs(), 9);  // 7 iiwa + 2 wsg.

  auto context = station.CreateDefaultContext();
  auto& plant_context = station.GetSubsystemContext(plant, *context);
  VectorXd q = VectorXd::LinSpaced(7, 0.1, 0.7),
           v = VectorXd::LinSpaced(7, 1.1, 1.7),
           q_command = VectorXd::LinSpaced(7, 2.1, 2.7),
           tau_ff = VectorXd::LinSpaced(7, 3.1, 3.7);

  // Set positions and read them back out, multiple ways.
  station.SetIiwaPosition(context.get(), q);
  EXPECT_TRUE(CompareMatrices(q, station.GetIiwaPosition(*context)));
  EXPECT_TRUE(CompareMatrices(q, station.GetOutputPort("iiwa_position_measured")
                                     .Eval<BasicVector<double>>(*context)
                                     .get_value()));
  for (int i = 0; i < 7; i++) {
    EXPECT_EQ(q(i), plant
                        .template GetJointByName<RevoluteJoint>(
                            "iiwa_joint_" + std::to_string(i + 1))
                        .get_angle(plant_context));
  }

  // Set velocities and read them back out, multiple ways.
  station.SetIiwaVelocity(context.get(), v);
  EXPECT_TRUE(CompareMatrices(v, station.GetIiwaVelocity(*context)));
  EXPECT_TRUE(
      CompareMatrices(v, station.GetOutputPort("iiwa_velocity_estimated")
                             .Eval<BasicVector<double>>(*context)
                             .get_value()));
  for (int i = 0; i < 7; i++) {
    EXPECT_EQ(v(i), plant
                        .template GetJointByName<RevoluteJoint>(
                            "iiwa_joint_" + std::to_string(i + 1))
                        .get_angular_rate(plant_context));
  }

  // Check position command pass through.
  station.GetInputPort("iiwa_position").FixValue(context.get(), q_command);
  EXPECT_TRUE(CompareMatrices(q_command,
                              station.GetOutputPort("iiwa_position_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));
  // Check that the additional input port exists and is spelled correctly.
  DRAKE_EXPECT_NO_THROW(station.GetInputPort("applied_spatial_force"));

  // Confirm that iiwa_torque_commanded doesn't depend on applied_spatial_force.
  ApplyExternalForceToManipulator(&station, context.get());

  // Check feedforward_torque command.
  VectorXd tau_with_no_ff = station.GetOutputPort("iiwa_torque_commanded")
                                .Eval<BasicVector<double>>(*context)
                                .get_value();
  // Confirm that default values are zero.
  station.GetInputPort("iiwa_feedforward_torque")
      .FixValue(context.get(), VectorXd::Zero(7));
  EXPECT_TRUE(CompareMatrices(tau_with_no_ff,
                              station.GetOutputPort("iiwa_torque_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));
  station.GetInputPort("iiwa_feedforward_torque")
      .FixValue(context.get(), tau_ff);
  EXPECT_TRUE(CompareMatrices(tau_with_no_ff + tau_ff,
                              station.GetOutputPort("iiwa_torque_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));

  // All ports must be connected if later on we'll ask questions like: "what's
  // the external contact torque?". We therefore fix the gripper related ports.
  double wsg_position = station.GetWsgPosition(*context);
  station.GetInputPort("wsg_position").FixValue(context.get(), wsg_position);
  station.GetInputPort("wsg_force_limit").FixValue(context.get(), 40.);

  // Check iiwa_torque_commanded == iiwa_torque_measured.
  EXPECT_TRUE(CompareMatrices(station.GetOutputPort("iiwa_torque_commanded")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value(),
                              station.GetOutputPort("iiwa_torque_measured")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value()));

  // Check that iiwa_torque_external == 0 (no contact).
  EXPECT_TRUE(station.GetOutputPort("iiwa_torque_external")
                  .Eval<BasicVector<double>>(*context)
                  .get_value()
                  .isZero());

  // Check that the additional output ports exist and are spelled correctly.
  DRAKE_EXPECT_NO_THROW(station.GetOutputPort("contact_results"));
  DRAKE_EXPECT_NO_THROW(station.GetOutputPort("plant_continuous_state"));

  // The station (manipulation station) has both a plant and a controller_plant.
  // The controller plant has a "composite" rigid body whose spatial inertia is
  // associated with the set S of bodies consisting of the gripper body G and
  // the left and right fingers. The spatial inertia of the composite body is
  // equal to the set S's spatial inertia about Go (body G's origin), expressed
  // in G, where the fingers are regarded as being in a "zero" configuration.
  // Verify the spatial inertia stored in the controller_plant matches the
  // calculation returned by MakeCompositeGripperInertia().
  const multibody::MultibodyPlant<double>& controller_plant =
      station.get_controller_plant();
  const multibody::RigidBody<double>& composite_gripper =
      controller_plant.GetRigidBodyByName("wsg_equivalent");
  const multibody::SpatialInertia<double> M_SGo_G_actual =
      composite_gripper.default_spatial_inertia();
  const multibody::SpatialInertia<double> M_SGo_G_expected =
      MakeCompositeGripperInertia();

  const Matrix6<double> M6_actual = M_SGo_G_actual.CopyToFullMatrix6();
  const Matrix6<double> M6_expected = M_SGo_G_expected.CopyToFullMatrix6();
  constexpr double ktol = 32 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(M6_actual, M6_expected, ktol));
}

// Partially check M(q)vdot ≈ Mₑ(q)vdot_desired + τ_feedforward + τ_external
// by setting the right side to zero and confirming that vdot ≈ 0.
GTEST_TEST(ManipulationStationTest, CheckDynamics) {
  const double kTimeStep = 0.002;
  ManipulationStation<double> station(kTimeStep);
  station.SetupManipulationClassStation();
  station.Finalize();

  auto context = station.CreateDefaultContext();
  const VectorXd iiwa_position = VectorXd::LinSpaced(7, 0.735, 0.983);
  const VectorXd iiwa_velocity = VectorXd::LinSpaced(7, -1.23, 0.456);
  SetArbitraryState(station, context.get(), iiwa_position, iiwa_velocity);

  // Expect continuous state from the integral term in the PID from the
  // inverse dynamics controller.
  EXPECT_EQ(context->num_continuous_states(), 7);

  // Check that iiwa_torque_external == 0 (no contact).
  EXPECT_TRUE(station.GetOutputPort("iiwa_torque_external")
                  .Eval<BasicVector<double>>(*context)
                  .get_value()
                  .isZero());

  const VectorXd next_velocity = GetNextIiwaVelocity(station,
                                                     context.get(),
                                                     "iiwa_joint_1");
  // Note: This tolerance could be much smaller if the wsg was not attached.
  const double kTolerance = 1e-4;  // rad/sec.

  // Check that vdot ≈ 0 by checking that next velocity ≈ velocity.
  EXPECT_TRUE(CompareMatrices(iiwa_velocity, next_velocity, kTolerance));
}

// Confirm that `applied_spatial_force` impacts the plant dynamics
// run experiment again to show that τ_external causes vdot ≠ vdot_desired
GTEST_TEST(ManipulationStationTest, CheckDynamicsUnderExternallyAppliedForce) {
  const double kTimeStep = 0.002;
  ManipulationStation<double> station(kTimeStep);
  station.SetupManipulationClassStation();
  station.Finalize();

  auto context = station.CreateDefaultContext();
  const VectorXd iiwa_position = VectorXd::LinSpaced(7, 0.735, 0.983);
  const VectorXd zero_iiwa_velocity = VectorXd::Zero(7);
  SetArbitraryState(station, context.get(), iiwa_position, zero_iiwa_velocity);
  ApplyExternalForceToManipulator(&station, context.get());

  const VectorXd next_velocity = GetNextIiwaVelocity(station,
                                                     context.get(),
                                                     "iiwa_joint_1");

  // Note: This tolerance could be much smaller if the wsg was not attached.
  const double kTolerance = 1e-4;  // rad/sec.

  // The next_velocity is expected to be non-zero due to
  // the applied external forces.
  EXPECT_FALSE(CompareMatrices(zero_iiwa_velocity, next_velocity, kTolerance));
}

GTEST_TEST(ManipulationStationTest, CheckWsg) {
  ManipulationStation<double> station(0.001);
  station.SetupManipulationClassStation();
  station.Finalize();

  auto context = station.CreateDefaultContext();

  const double q = 0.023;
  const double v = 0.12;

  station.SetWsgPosition(context.get(), q);
  EXPECT_EQ(station.GetWsgPosition(*context), q);

  station.SetWsgVelocity(context.get(), v);
  EXPECT_EQ(station.GetWsgVelocity(*context), v);

  EXPECT_TRUE(CompareMatrices(station.GetOutputPort("wsg_state_measured")
                                  .Eval<BasicVector<double>>(*context)
                                  .get_value(),
                              Vector2d(q, v)));

  DRAKE_EXPECT_NO_THROW(station.GetOutputPort("wsg_force_measured"));
}

GTEST_TEST(ManipulationStationTest, CheckRGBDOutputs) {
  ManipulationStation<double> station(0.001);
  station.SetupManipulationClassStation();
  station.Finalize();

  auto context = station.CreateDefaultContext();

  for (const auto& name : station.get_camera_names()) {
    // Make sure the camera outputs can be evaluated, and are non-empty.
    EXPECT_GE(station.GetOutputPort("camera_" + name + "_rgb_image")
                  .Eval<systems::sensors::ImageRgba8U>(*context)
                  .size(),
              0);
    EXPECT_GE(station.GetOutputPort("camera_" + name + "_depth_image")
                  .Eval<systems::sensors::ImageDepth16U>(*context)
                  .size(),
              0);
    EXPECT_GE(station.GetOutputPort("camera_" + name + "_label_image")
                  .Eval<systems::sensors::ImageLabel16I>(*context)
                  .size(),
              0);
  }
}

GTEST_TEST(ManipulationStationTest, CheckCollisionVariants) {
  ManipulationStation<double> station1(0.002);
  station1.SetupManipulationClassStation(IiwaCollisionModel::kNoCollision);

  // In this variant, there are collision geometries from the world and the
  // gripper, but not from the iiwa.
  const int num_collisions =
      station1.get_multibody_plant().num_collision_geometries();

  ManipulationStation<double> station2(0.002);
  station2.SetupManipulationClassStation(IiwaCollisionModel::kBoxCollision);
  // Check for additional collision elements (one for each link, which includes
  // the base).
  EXPECT_EQ(station2.get_multibody_plant().num_collision_geometries(),
            num_collisions + 8);

  // The controlled model does not register with a scene graph, so has zero
  // collisions.
  EXPECT_EQ(station2.get_controller_plant().num_collision_geometries(), 0);
}

GTEST_TEST(ManipulationStationTest, AddManipulandFromFile) {
  ManipulationStation<double> station(0.002);
  const int num_base_instances =
      station.get_multibody_plant().num_model_instances();

  station.AddManipulandFromFile(
      "drake/manipulation/models/ycb/sdf/003_cracker_box.sdf",
      math::RigidTransform<double>::Identity());

  // Check that the cracker box was added.
  EXPECT_EQ(station.get_multibody_plant().num_model_instances(),
            num_base_instances + 1);

  station.AddManipulandFromFile(
      "drake/manipulation/models/ycb/sdf/004_sugar_box.sdf",
      math::RigidTransform<double>::Identity());

  // Check that the sugar box was added.
  EXPECT_EQ(station.get_multibody_plant().num_model_instances(),
            num_base_instances + 2);
}

GTEST_TEST(ManipulationStationTest, SetupClutterClearingStation) {
  ManipulationStation<double> station(0.002);
  station.SetupClutterClearingStation(math::RigidTransform<double>::Identity(),
                                      IiwaCollisionModel::kNoCollision);
  station.Finalize();

  // Make sure we get through the setup and initialization.
  auto context = station.CreateDefaultContext();

  // Check that domain randomization works.
  RandomGenerator generator;
  station.SetRandomContext(context.get(), &generator);
}

GTEST_TEST(ManipulationStationTest, SetupPlanarIiwaStation) {
  ManipulationStation<double> station(0.002);
  station.SetupPlanarIiwaStation();
  station.Finalize();

  // Make sure we get through the setup and initialization.
  auto context = station.CreateDefaultContext();

  // Check that domain randomization works.
  RandomGenerator generator;
  station.SetRandomContext(context.get(), &generator);
}


// Check that making many stations does not exhaust resources.
GTEST_TEST(ManipulationStationTest, MultipleInstanceTest) {
  for (int i = 0; i < 20; ++i) {
    ManipulationStation<double> station;
    station.SetupManipulationClassStation();
    station.Finalize();
  }
}

GTEST_TEST(ManipulationStationTest, RegisterRgbdCameraTest) {
  {
    // Test default setup.
    std::map<std::string, math::RigidTransform<double>> default_poses;

    auto set_default_camera_poses = [&default_poses]() {
      default_poses.emplace(
          "0", math::RigidTransform<double>(
                   math::RollPitchYaw<double>(2.549607, 1.357609, 2.971679),
                   Eigen::Vector3d(-0.228895, -0.452176, 0.486308)));

      default_poses.emplace(
          "1", math::RigidTransform<double>(
                   math::RollPitchYaw<double>(2.617427, -1.336404, -0.170522),
                   Eigen::Vector3d(-0.201813, 0.469259, 0.417045)));

      default_poses.emplace(
          "2", math::RigidTransform<double>(
                   math::RollPitchYaw<double>(-2.608978, 0.022298, 1.538460),
                   Eigen::Vector3d(0.786258, -0.048422, 1.043315)));
    };

    ManipulationStation<double> dut;
    dut.SetupManipulationClassStation();

    std::map<std::string, math::RigidTransform<double>> camera_poses =
        dut.GetStaticCameraPosesInWorld();
    set_default_camera_poses();

    EXPECT_EQ(camera_poses.size(), default_poses.size());

    for (const auto& pair : camera_poses) {
      auto found = default_poses.find(pair.first);
      EXPECT_TRUE(found != default_poses.end());
      EXPECT_TRUE(found->second.IsExactlyEqualTo(pair.second));
    }
  }

  {
    // Test registration to custom frames.
    ManipulationStation<double> dut;
    multibody::MultibodyPlant<double>& plant =
        dut.get_mutable_multibody_plant();

    geometry::render::DepthRenderCamera depth_camera{
        {dut.default_renderer_name(), {640, 480, M_PI_4}, {0.05, 3.0}, {}},
        {0.1, 2.0}};

    const Eigen::Translation3d X_WF0(0, 0, 0.2);
    const Eigen::Translation3d X_F0C0(0.3, 0.2, 0.0);
    const auto& frame0 =
        plant.AddFrame(std::make_unique<multibody::FixedOffsetFrame<double>>(
            "frame0", plant.world_frame(), X_WF0));
    dut.RegisterRgbdSensor("camera0", frame0, X_F0C0, depth_camera);

    const Eigen::Translation3d X_F0F1(0, -0.1, 0.2);
    const Eigen::Translation3d X_F1C1(-0.2, 0.2, 0.33);
    const auto& frame1 =
        plant.AddFrame(std::make_unique<multibody::FixedOffsetFrame<double>>(
            "frame1", frame0, X_F0F1));
    dut.RegisterRgbdSensor("camera1", frame1, X_F1C1, depth_camera);

    std::map<std::string, math::RigidTransform<double>> camera_poses =
        dut.GetStaticCameraPosesInWorld();

    EXPECT_EQ(camera_poses.size(), 2);
    EXPECT_TRUE(camera_poses.at("camera0").IsExactlyEqualTo(X_WF0 * X_F0C0));
    EXPECT_TRUE(
        camera_poses.at("camera1").IsExactlyEqualTo(X_WF0 * X_F0F1 * X_F1C1));
  }
}

// Confirms initialization of renderers. With none specified, the default
// renderer is used. Otherwise, the user-specified renderers are provided.
GTEST_TEST(ManipulationStationTest, ConfigureRenderer) {
  // Case: no user render engines specified; has renderer with default name.
  {
    ManipulationStation<double> dut;
    dut.SetupManipulationClassStation();
    dut.Finalize();
    const auto& scene_graph = dut.get_scene_graph();
    EXPECT_EQ(1, scene_graph.RendererCount());
    EXPECT_TRUE(scene_graph.HasRenderer(dut.default_renderer_name()));
  }

  // Case: multiple user-specified render engines provided.
  {
    ManipulationStation<double> dut;
    dut.SetupManipulationClassStation();
    std::map<std::string, std::unique_ptr<geometry::render::RenderEngine>>
        engines;
    const std::string name1 = "engine1";
    engines[name1] = std::make_unique<DummyRenderEngine>();
    const std::string name2 = "engine2";
    engines[name2] = std::make_unique<DummyRenderEngine>();
    dut.Finalize(std::move(engines));

    const auto& scene_graph = dut.get_scene_graph();
    EXPECT_EQ(2, scene_graph.RendererCount());
    EXPECT_TRUE(scene_graph.HasRenderer(name1));
    EXPECT_TRUE(scene_graph.HasRenderer(name2));
    EXPECT_FALSE(scene_graph.HasRenderer(dut.default_renderer_name()));
  }
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
