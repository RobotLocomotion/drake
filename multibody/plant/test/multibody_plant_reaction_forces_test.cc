#include <memory>
#include <thread>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::Box;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::ConnectContactResultsToDrakeVisualizer;
using drake::multibody::RevoluteJoint;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::Simulator;
using Eigen::Vector3d;

namespace drake {
namespace multibody {

namespace {

// This test simulates a "ladder" leaning against a wall, under the action of
// gravity pullin in the -z axis direction. The bottom of the ladder is pinned
// to the ground by a revolute joint with its axis of revolution aligned with
// the y axis. We apply an external torque to this joint.
// Please run this unit test along with Drake's visualizer to obtain a
// visualization of the setup.
// We run a simulation to reach the steady state in which contact forces balance
// the action of gravity and external actuation. This problem essentially is a
// two dimensional problem in the x-z plane, with the joint axis into this
// plane. We define the length kProblemWidth to be the width into the
// x-z plane along the y axis. However, we make the problem of computing
// reaction forces a bit more interesting by defining a contact geometry that
// is not symmetric along the y-axis leading to an additional reaction torque
// along the z axis. We emulate a single point of contact between the ladder and
// the wall by placing a sphere on the top corner (y positive, z positive in the
// body frame) of the ladder. Therefore, since the point of contact is not at
// y = 0 but offset to the size at y = kProblemWidth / 2, there is an
// additional reaction torque along the z axis.
// In addition, we split the ladder in two and weld them together. This allow us
// to test the computation of reaction forces at a weld joint.
//
// Summarizing, this problem setup computes the (static) reaction force at the
// bottom pin joint holding the ladder to the ground, in the presence of contact
// and actuation.
//
// We perform this test for both continuous and discrete models, and expect
// identical results from a weld joint and a locked revolute joint.
class LadderTest : public ::testing::Test {
 protected:
  void BuildLadderModel(double discrete_update_period, bool locked_joint) {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(
        &builder,
        std::make_unique<MultibodyPlant<double>>(discrete_update_period));

    AddWall();
    AddPinnedLadder(locked_joint);
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity));
    plant_->Finalize();

    // Add visualization for verification of the results when we have the
    // visualizer running.
    geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph_, &lcm_);
    ConnectContactResultsToDrakeVisualizer(
        &builder, *plant_, *scene_graph_, &lcm_);

    diagram_ = builder.Build();
  }

  // Adds the model for a wall anchored to the wall.
  void AddWall() {
    const Vector3d size(kWallWidth, kProblemWidth, kWallHeight);
    const RigidTransformd X_WB = Eigen::Translation3d(
        kDistanceToWall + kWallWidth / 2.0, 0.0, kWallHeight / 2.0);
    const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
    const auto shape = geometry::Box(size(0), size(1), size(2));
    plant_->RegisterVisualGeometry(plant_->world_body(), X_WB, shape,
                                   "wall_visual", green);
    plant_->RegisterCollisionGeometry(
        plant_->world_body(), X_WB, shape, "wall_collision",
        CoulombFriction<double>(kFrictionCoefficient, kFrictionCoefficient));
  }

  // Adds the model for the ladder pinned to the ground at the origin. If @p
  // locked_joint is true, uses a locked revolute joint; otherwise uses a weld
  // joint.
  void AddPinnedLadder(bool locked_joint) {
    // We split the ladder into two halves and join them with a weld joint so
    // that we can evaluate the reaction force right at the middle.
    // We define body frame Bl and Bu for the lower and upper portions of the
    // ladder respectively.
    // Both of these frames's origins are located at the lower end of each half.
    // In particular, the lower frame Bl attaches to the ground with the pin
    // joint.
    const Vector3<double> p_BoBcm_B(0.0, 0.0, kLadderLength / 4.0);
    const UnitInertia<double> G_BBcm =
        UnitInertia<double>::ThinRod(kLadderLength / 2.0, Vector3d::UnitZ());
    const SpatialInertia<double> M_BBo_B =
        SpatialInertia<double>::MakeFromCentralInertia(
            kLadderMass / 2.0, p_BoBcm_B, kLadderMass / 2.0 * G_BBcm);

    // Create a rigid body for the ladder.
    ladder_lower_ = &plant_->AddRigidBody("ladder_lower", M_BBo_B);
    ladder_upper_ = &plant_->AddRigidBody("ladder_upper", M_BBo_B);

    // Both lower and upper sections have a box geometry for visualization.
    auto shape = Box(kLadderWidth, kProblemWidth, kLadderLength / 2.0);
    // We want the side of the box to rest between the pin joint and the contact
    // point at the wall. Therefore we place the geometry frame origin Go with a
    // shift -kLadderWidth / 2.0 in the body frame's x-axis.
    const RigidTransformd X_BV(
        Vector3d(-kLadderWidth / 2.0, 0.0, kLadderLength / 4.0));
    const Vector4<double> light_blue(0.5, 0.8, 1.0, 1.0);
    const Vector4<double> dark_blue(0.0, 0.0, 0.8, 1.0);
    plant_->RegisterVisualGeometry(*ladder_lower_, X_BV, shape,
                                   "LadderLowerVisualGeometry", light_blue);
    plant_->RegisterVisualGeometry(*ladder_upper_, X_BV, shape,
                                   "LadderUpperVisualGeometry", dark_blue);
    // We'll add a sphere to emulate a single point of contact against the wall.
    // We place it at the y+ corner of the ladder geometry so that the contact
    // force causes a non-zero torque at the pin joint for a more interesting
    // case.
    const double point_contact_radius = 5.0e-3;
    const RigidTransformd X_BC(
        Vector3d(0.0, kProblemWidth / 2.0, kLadderLength / 2.0));
    plant_->RegisterCollisionGeometry(
        *ladder_upper_, X_BC, Sphere(point_contact_radius),
        "LadderUpperCollisionGeometry",
        CoulombFriction<double>(kFrictionCoefficient, kFrictionCoefficient));

    // Pin to the floor with a revolute joint.
    pin_ = &plant_->AddJoint<RevoluteJoint>("PinToGround", plant_->world_body(),
                                            {}, *ladder_lower_, {},
                                            Vector3d::UnitY(), kPinDamping);

    // Join the two halves.
    const RigidTransformd X_BlBu(Vector3d(0.0, 0.0, kLadderLength / 2.0));
    if (locked_joint) {
      joint_ = &plant_->AddJoint<RevoluteJoint>("Weld", *ladder_lower_, X_BlBu,
                                               *ladder_upper_, {},
                                               Vector3d::UnitY(), 0.0);
    } else {
      joint_ = &plant_->WeldFrames(ladder_lower_->body_frame(),
                                  ladder_upper_->body_frame(), X_BlBu);
    }

    // Add actuation.
    plant_->AddJointActuator("PinActuator", *pin_);
  }

  // Build and run a simulator, and return it after simulating.
  std::unique_ptr<Simulator<double>> Simulate(
      std::unique_ptr<Context<double>> diagram_context, bool locked_joint) {
    Context<double>* plant_context =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context.get());

    // Set initial condition with the ladder leaning against the wall.
    // We compute the angle in the pin joint for this condition.
    const double theta = std::asin(kDistanceToWall / kLadderLength);
    pin_->set_angle(plant_context, theta);

    // Fix the actuation.
    const Vector1d tau_actuation = kActuationTorque * Vector1d::Ones();
    plant_->get_actuation_input_port().FixValue(plant_context, tau_actuation);

    if (locked_joint) {
        joint_->Lock(plant_context);
    }

    // Sanity check model size.
    auto sanity_check = [this, &locked_joint]() {
      ASSERT_EQ(plant_->num_bodies(), 3);
      ASSERT_EQ(plant_->num_velocities(), locked_joint ? 2 : 1);
      ASSERT_EQ(plant_->num_actuated_dofs(), 1);
    };
    sanity_check();

    // We run a simulation to steady state so that contact forces balance
    // gravity and actuation.
    auto simulator = std::make_unique<Simulator<double>>(
        *diagram_, std::move(diagram_context));
    // The default RK3 integrator requires specifying a very high accuracy to
    // reach steady state within kTolerance and therefore it is very costly.
    // However implicit Euler does a much better job with larger time steps.
    simulator->reset_integrator<systems::ImplicitEulerIntegrator<double>>();
    simulator->get_mutable_integrator().set_maximum_step_size(5e-3);
    simulator->get_mutable_integrator().set_target_accuracy(1e-6);
    simulator->Initialize();
    const double simulation_time = 1.0;  // seconds.
    simulator->AdvanceTo(simulation_time);
    return simulator;
  }

  void VerifyJointReactionForces(
      Context<double>* diagram_context, bool locked_joint) {
    Context<double>* plant_context =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context);
    // Evaluate the reaction forces output port to get the reaction force at the
    // pin joint. Re-express in the world frame W.
    const auto& reaction_forces =
        plant_->get_reaction_forces_output_port()
            .Eval<std::vector<SpatialForce<double>>>(*plant_context);
    ASSERT_EQ(reaction_forces.size(), 2u);
    const SpatialForce<double>& F_Bl_Bl = reaction_forces[pin_->index()];
    const RigidTransformd X_WBl =
        ladder_lower_->EvalPoseInWorld(*plant_context);
    const SpatialForce<double> F_Bl_W = X_WBl.rotation() * F_Bl_Bl;

    // We evaluate the contact forces so that we can perform the balance of
    // forces by hand and compare with the results obtained by evaluating the
    // reaction forces port.
    const ContactResults<double>& contact_results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            *plant_context);
    // There should be a single contact pair.
    ASSERT_EQ(contact_results.num_point_pair_contacts(), 1);
    const PointPairContactInfo<double>& point_pair_contact_info =
        contact_results.point_pair_contact_info(0);

    const double direction =
        point_pair_contact_info.bodyB_index() == ladder_upper_->index() ? 1.0
                                                                        : -1.0;

    // Contact force on the ladder at the contact point, expressed in the world
    // frame.
    const Vector3d f_Bc_W = direction * point_pair_contact_info.contact_force();

    // The contact point.
    const Vector3d& p_WC = point_pair_contact_info.contact_point();

    // Ladder's weight.
    const double weight = kGravity * kLadderMass;

    // Position of the ladder's center of gravity.
    const Vector3d p_WBcm =
        plant_->CalcCenterOfMassPositionInWorld(*plant_context);

    // We validate the numerical results to be within this tolerance value,
    // which is chosen consistently with the time the system is left to reach
    // steady state and the integration accuracy (for the continuous model).
    const double kTolerance = 1.0e-11;

    // The x component of the contact force must counteract the torque due to
    // gravity plus the actuation torque.
    const double tau_g = p_WBcm.x() * weight;  // gravity torque about Bo.
    const double fc_x = (tau_g + kActuationTorque) / p_WC.z();
    const Vector3d f_Bl_W_expected(fc_x, 0.0, weight);
    EXPECT_TRUE(
        CompareMatrices(F_Bl_W.translational(), f_Bl_W_expected, kTolerance));

    // Expected contact force.
    const Vector3d f_C_W_expected(-fc_x, 0.0, 0.0);
    EXPECT_TRUE(CompareMatrices(f_Bc_W, f_C_W_expected, kTolerance));

    // Since the contact point was purposely located at
    // y = kProblemWidth / 2.0, the contact force causes a reaction
    // torque at the pin joint oriented along the z-axis.
    const Vector3d t_Bl_W_expected(0.0, kActuationTorque,
                                   -fc_x * kProblemWidth / 2.0);
    EXPECT_TRUE(
        CompareMatrices(F_Bl_W.rotational(), t_Bl_W_expected, kTolerance));

    // Verify reaction forces at the joint.
    const RigidTransformd X_WBu =
        ladder_upper_->EvalPoseInWorld(*plant_context);
    const SpatialForce<double>& F_Bu_W =
        X_WBu.rotation() * reaction_forces[joint_->index()];
    const Vector3d f_Bu_expected(fc_x, 0.0, weight / 2.0);
    const double t_Bu_y =
        -(p_WBcm.x() / 2.0) * (weight / 2.0) + fc_x * p_WBcm.z();
    const Vector3d t_Bu_expected(0.0, t_Bu_y, -fc_x * kProblemWidth / 2.0);
    EXPECT_TRUE(
        CompareMatrices(F_Bu_W.rotational(), t_Bu_expected, kTolerance));
    EXPECT_TRUE(
        CompareMatrices(F_Bu_W.translational(), f_Bu_expected, kTolerance));
  }

  void TestWithThreads(double time_step, bool locked_joint) {
    SCOPED_TRACE(fmt::format("time_step = []", time_step));
    BuildLadderModel(time_step, locked_joint);
    ASSERT_EQ(plant_->is_discrete(), (time_step != 0.));

    // Create the threads' contexts by cloning a prototype. This will help
    // ensure the context deep copy is properly working.
    auto context_prototype = diagram_->CreateDefaultContext();
    auto simulator_prototype =
        Simulate(std::move(context_prototype), locked_joint);
    VerifyJointReactionForces(
        &simulator_prototype->get_mutable_context(), locked_joint);

    // Running the simulation in multiple threads here gives us a chance to
    // check readiness for context-per-thread usage. Even though all threads
    // are doing the same thing, ThreadSanitizer will be able to detect
    // potential data races.
    static constexpr int kThreads = 2;
    std::vector<std::thread> threads;
    for (int k = 0; k < kThreads; k++) {
      threads.push_back(
          std::thread([this, &simulator_prototype, &locked_joint]() {
              auto context = simulator_prototype->get_context().Clone();
              Simulate(std::move(context), locked_joint);
              // We skip verifying forces here because system evolution
              // invalidates the expected values used above.
            }));
    }
    for (auto& thread : threads) {
      thread.join();
    }
  }

  // This problem essentially is two-dimensional.
  // This is the length in the direction normal to the x-z plane, along the
  // y-axis.
  const double kProblemWidth{1.0};  // [m]

  // Ladder parameters.
  const double kLadderLength{2.0};         // [m]
  const double kLadderMass{7.0};           // [kg]
  const double kLadderWidth{0.15};         // [m]
  const double kFrictionCoefficient{0.0};  // Frictionless contact, [-]

  // Wall parameters.
  const double kWallWidth{0.3};   // [m]
  const double kWallHeight{3.0};  // [m]

  // Pin joint parameters.
  const double kDistanceToWall{1.0};
  const double kPinDamping{0.0};        // [N⋅m⋅s]
  const double kActuationTorque{20.0};  // [N⋅m]

  // We round off gravity for simpler numbers.
  const double kGravity{10.0};  // [m/s²]

  lcm::DrakeLcm lcm_;  // For visualization.
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* ladder_lower_{nullptr};
  const RigidBody<double>* ladder_upper_{nullptr};
  const RevoluteJoint<double>* pin_{nullptr};

  // Either a weld joint, or a locked revolute joint, depending on test
  // configuration.
  const Joint<double>* joint_{nullptr};

  std::unique_ptr<Diagram<double>> diagram_;
};

TEST_F(LadderTest, PinReactionForcesContinuous) {
  static constexpr bool kIsJointLocked = false;
  TestWithThreads(0., kIsJointLocked);
}

TEST_F(LadderTest, PinReactionForcesDiscrete) {
  static constexpr bool kIsJointLocked = false;
  TestWithThreads(1.0e-3, kIsJointLocked);
}

// TODO(joemasterjohn) Expand the continuous locked joint test when continuous
// joint locking is implemented.
TEST_F(LadderTest, PinReactionForcesLockedJointContinuous) {
  static constexpr bool kIsJointLocked = true;
  BuildLadderModel(0, kIsJointLocked);
  auto context = diagram_->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      Simulate(std::move(context), kIsJointLocked), ".*is_state_discrete.*");
}

TEST_F(LadderTest, PinReactionForcesLockedJointDiscrete) {
  static constexpr bool kIsJointLocked = true;
  TestWithThreads(1.0e-3, kIsJointLocked);
}

// This test verifies the computation of joint reaction forces for a case in
// which centrifugal terms cannot be neglected.
// The setup consists of a rod subject to spin about the world's z axis around
// a pin joint. Gravity is aligned with the world's z axis, perpendicular to the
// plane of rotation.
// We thus compute the reaction forces and verify that both centrifugal and
// gravitational terms are correct.
class SpinningRodTest : public ::testing::Test {
 protected:
  void BuildModel(double discrete_update_period) {
    plant_ = std::make_unique<MultibodyPlant<double>>(discrete_update_period);

    // We define the rod's frame origin to be located at the CoM.
    const SpatialInertia<double> M_BBo_B(
        kMass, Vector3d::Zero(),
        UnitInertia<double>::ThinRod(kLength, Vector3d::UnitZ()));
    rod_ = &plant_->AddRigidBody("rod", M_BBo_B);

    // Notice that axis Bz is aligned with the rod. We want to define frame Jb
    // on body B at the attachment point to have its z axis aligned with the
    // world's z axis. Therefore we must rotate it 90 degrees about the B's x
    // axis.
    const RigidTransformd X_BJb(RollPitchYawd(M_PI_2, 0.0, 0.0),
                                Vector3d(0.0, 0.0, -kLength / 2.0));
    pin_ = &plant_->AddJoint<RevoluteJoint>("pin", plant_->world_body(), {},
                                            *rod_, X_BJb, Vector3d::UnitZ());

    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity));

    // Done defining the model.
    plant_->Finalize();

    // Create and set context.
    context_ = plant_->CreateDefaultContext();
    pin_->set_angle(context_.get(), M_PI / 4.0);
    pin_->set_angular_rate(context_.get(), kOmega);
  }

  void VerifyJointReactionForces() {
    const double kTolerance = 20 * std::numeric_limits<double>::epsilon();

    // Evaluate the spatial acceleration of the rod.
    const auto& A_WB_all =
        plant_->get_body_spatial_accelerations_output_port()
            .Eval<std::vector<SpatialAcceleration<double>>>(*context_);
    const SpatialAcceleration<double>& A_WRod = A_WB_all[rod_->index()];

    // Bz is the unit vector along the rod's length.
    const Vector3d Bz_W =
        rod_->EvalPoseInWorld(*context_).rotation().matrix().col(2);

    // There is no damping, therefore we expect the angular acceleration to be
    // zero.
    const Vector3d alpha_WB = Vector3d::Zero();

    // And the translational acceleration to contain the centrifugal
    // acceleration.
    const Vector3d a_WB = -Bz_W * kLength / 2.0 * kOmega * kOmega;

    // Verify the result.
    EXPECT_TRUE(CompareMatrices(A_WRod.rotational(), alpha_WB, kTolerance));
    EXPECT_TRUE(CompareMatrices(A_WRod.translational(), a_WB, kTolerance));

    // Evaluate reaction force at the pin.
    const auto& reaction_forces =
        plant_->get_reaction_forces_output_port()
            .Eval<std::vector<SpatialForce<double>>>(*context_);
    ASSERT_EQ(reaction_forces.size(), 1u);
    const SpatialForce<double>& F_BJb_Jb = reaction_forces[pin_->index()];

    // Verify that the value of the reaction force includes the centripetal
    // component (along Jb's y axis) and the weight component (along Jb's z
    // axis).
    const Vector3d f_BJb_Jb_expected(
        0.0, -kMass * kLength / 2.0 * kOmega * kOmega, kMass * kGravity);
    EXPECT_TRUE(CompareMatrices(F_BJb_Jb.translational(), f_BJb_Jb_expected,
                                kTolerance));

    // The reaction torque must counteract gravity.
    const Vector3d t_BJb_Jb_expected(kMass * kGravity * kLength / 2.0, 0.0,
                                     0.0);
    EXPECT_TRUE(
        CompareMatrices(F_BJb_Jb.rotational(), t_BJb_Jb_expected, kTolerance));
  }

  const double kMass{1.5};      // [kg]
  const double kLength{2.0};    // [m]
  const double kGravity{10.0};  // [m/s²]
  const double kOmega{5.0};     // [rad/s]

  std::unique_ptr<MultibodyPlant<double>> plant_;
  const RigidBody<double>* rod_{nullptr};
  const RevoluteJoint<double>* pin_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

TEST_F(SpinningRodTest, PinReactionForcesContinuous) {
  BuildModel(0);
  ASSERT_FALSE(plant_->is_discrete());
  VerifyJointReactionForces();
}

TEST_F(SpinningRodTest, PinReactionForcesDiscrete) {
  BuildModel(1.0e-3);
  ASSERT_TRUE(plant_->is_discrete());
  VerifyJointReactionForces();
}

// We verify the computation of joint reaction forces in a model where all
// bodies are anchored to the world using weld joints, and therefore the model
// has a zero sized state.
// In this case a body A is welded to the world. A second body B is welded to A
// with a fixed offset along the x axis. Therefore we expect the a non-zero
// moment at the weld joint between the two bodies to counteract the weight of
// body B. The force on A should match the total weight of A plus B.
class WeldedBoxesTest : public ::testing::Test {
 protected:
  void BuildModel(double discrete_update_period) {
    plant_ = std::make_unique<MultibodyPlant<double>>(discrete_update_period);
    AddBoxes();
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity));
    plant_->Finalize();
    plant_context_ = plant_->CreateDefaultContext();
  }

  void AddBoxes() {
    const Vector3d p_BoBcm_B = Vector3d::Zero();
    const UnitInertia<double> G_BBcm =
        UnitInertia<double>::SolidBox(kCubeSize, kCubeSize, kCubeSize);
    const SpatialInertia<double> M_BBo_B =
        SpatialInertia<double>::MakeFromCentralInertia(kBoxMass, p_BoBcm_B,
                                                       G_BBcm);

    // Create two rigid bodies.
    boxA_ = &plant_->AddRigidBody("boxA", M_BBo_B);
    boxB_ = &plant_->AddRigidBody("boxB", M_BBo_B);

    // Desired transformation for the boxes in the world.
    const RigidTransformd X_WA(Vector3d::Zero());
    const RigidTransformd X_WB(Vector3d(kCubeSize, 0, 0));
    const RigidTransformd X_AB = X_WA.inverse() * X_WB;

    // Pin boxA to the world and boxB to boxA with weld joints.
    weld1_ = &plant_->WeldFrames(plant_->world_body().body_frame(),
                                 boxA_->body_frame(), X_WA);
    weld2_ =
        &plant_->WeldFrames(boxA_->body_frame(), boxB_->body_frame(), X_AB);
  }

  void VerifyBodyReactionForces() {
    const auto& reaction_forces =
        plant_->get_reaction_forces_output_port()
            .Eval<std::vector<SpatialForce<double>>>(*plant_context_);

    ASSERT_EQ(reaction_forces.size(), 2u);  // we have two joints.

    // Particulars for this setup:
    //   1. A is weld1's child body and its frame corresponds to the joint's
    //      child frame Jc.
    //   2. Moreover, A is coincident with the world and its origin is located
    //      at A's center of mass Acm.
    // Therefore the reaction at weld1 corresponds to F_Acm_W.
    const SpatialForce<double>& F_Acm_W = reaction_forces[weld1_->index()];

    // Verify the reaction force balances the weight of the two boxes.
    const double box_weight = kBoxMass * kGravity;
    const Vector3d f_Acm_W_expected(0.0, 0.0, 2.0 * box_weight);
    // Box B hangs from box A, and therefore the reaction on weld1 must balance
    // the torque due to gravity on box B applied on box A.
    const Vector3d t_Acm_W_expected =
        -kCubeSize * box_weight * Vector3d::UnitY();
    EXPECT_EQ(F_Acm_W.translational(), f_Acm_W_expected);
    EXPECT_EQ(F_Acm_W.rotational(), t_Acm_W_expected);

    // Particulars for this setup:
    //   1. Body B is weld2's child body and its frame corresponds to the
    //      joint's child frame Jc.
    //   2. There is no rotation between B and the world frame.
    //   3. Frame B's origin is located at B's center of mass Bcm.
    // Therefore the reaction at weld2 corresponds to F_Bcm_W.
    const SpatialForce<double>& F_Bcm_W = reaction_forces[weld2_->index()];
    const Vector3d f_Bcm_W_expected = box_weight * Vector3d::UnitZ();
    const Vector3d t_Bcm_W_expected = Vector3d::Zero();
    EXPECT_EQ(F_Bcm_W.translational(), f_Bcm_W_expected);
    EXPECT_EQ(F_Bcm_W.rotational(), t_Bcm_W_expected);
  }

  const Body<double>* boxA_{nullptr};
  const Body<double>* boxB_{nullptr};
  std::unique_ptr<MultibodyPlant<double>> plant_{nullptr};
  const WeldJoint<double>* weld1_{nullptr};
  const WeldJoint<double>* weld2_{nullptr};
  std::unique_ptr<Context<double>> plant_context_;
  const double kCubeSize{1.5};  // Size of the box, in meters.
  const double kBoxMass{2.0};   // Mass of each box, in Kg.
  // We round off gravity for simpler numbers.
  const double kGravity{10.0};  // [m/s²]
};

TEST_F(WeldedBoxesTest, ReactionForcesDiscrete) {
  BuildModel(1.0e-3);
  ASSERT_TRUE(plant_->is_discrete());
  VerifyBodyReactionForces();
}

TEST_F(WeldedBoxesTest, ReactionForcesContinuous) {
  BuildModel(0.0);
  ASSERT_FALSE(plant_->is_discrete());
  VerifyBodyReactionForces();
}

}  // namespace
}  // namespace multibody
}  // namespace drake
