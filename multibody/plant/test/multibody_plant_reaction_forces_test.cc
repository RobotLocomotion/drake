#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
// plane. We define the length kProblemWidth_ to be the width into the
// x-z plane along the y axis. However, we make the problem of computing
// reaction forces a bit more interesting by defining a contact geometry that
// is not symmetric along the y-axis leading to an additional reaction torque
// along the z axis. We emulate a single point of contact between the ladder and
// the wall by placing a sphere on the top corner (y positive, z positive in the
// body frame) of the ladder. Therefore, since the point of contact is not at
// y = 0 but offset to the size at y = kProblemWidth_ / 2, there is an
// additional reaction torque along the z axis.
// In addition, we split the ladder in two and weld them together. This allow us
// to test the computation of reaction forces at a weld joint.
//
// Summarizing, this problem setup computes the (static) reaction force at the
// bottom pin joint holding the ladder to the ground, in the presence of contact
// and actuation.
//
// We perform this test for both continuous and discrete models.
class LadderTest : public ::testing::Test {
 protected:
  void BuildLadderModel(double discrete_update_period) {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(
        &builder,
        std::make_unique<MultibodyPlant<double>>(discrete_update_period));

    AddWall();
    AddPinnedLadder();
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3d(0.0, 0.0, -kGravity_));
    plant_->Finalize();

    // Add visualization for verification of the results when we have the
    // visualizer running.
    geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph_, &lcm_);
    ConnectContactResultsToDrakeVisualizer(&builder, *plant_, &lcm_);

    diagram_ = builder.Build();

    // Create a context for this system:
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());

    // Set initial condition with the ladder leaning against the wall.
    // We compute the angle in the pin joint for this condition.
    const double theta = std::asin(kDistanceToWall_ / kLadderLength_);
    pin_->set_angle(plant_context_, theta);

    // Fix the actuation.
    const Vector1d tau_actuation = kActuationTorque_ * Vector1d::Ones();
    plant_->get_actuation_input_port().FixValue(plant_context_, tau_actuation);
  }

  // Adds the model for a wall anchored to the wall.
  void AddWall() {
    const Vector3d size(kWallWidth_, kProblemWidth_, kWallHeight_);
    const RigidTransformd X_WB = Eigen::Translation3d(
        kDistanceToWall_ + kWallWidth_ / 2.0, 0.0, kWallHeight_ / 2.0);
    const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
    const auto shape = geometry::Box(size(0), size(1), size(2));
    plant_->RegisterVisualGeometry(plant_->world_body(), X_WB, shape,
                                   "wall_visual", green);
    plant_->RegisterCollisionGeometry(
        plant_->world_body(), X_WB, shape, "wall_collision",
        CoulombFriction<double>(kFrictionCoefficient_, kFrictionCoefficient_));
  }

  // Adds the model for the ladder pinned to the ground at the origin.
  void AddPinnedLadder() {
    // We split the ladder into two halves and join them with a weld joint so
    // that we can evaluate the reaction force right at the middle.
    // We define body frame Bl and Bu for the lower and upper portions of the
    // ladder respectively.
    // Both of these frames's origins are located at the lower end of each half.
    // In particular, the lower frame Bl attaches to the ground with the pin
    // joint.
    const Vector3<double> p_BoBcm_B(0.0, 0.0, kLadderLength_ / 4.0);
    const UnitInertia<double> G_BBcm =
        UnitInertia<double>::ThinRod(kLadderLength_ / 2.0, Vector3d::UnitZ());
    const SpatialInertia<double> M_BBo_B =
        SpatialInertia<double>::MakeFromCentralInertia(
            kLadderMass_ / 2.0, p_BoBcm_B, kLadderMass_ / 2.0 * G_BBcm);

    // Create a rigid body for the ladder.
    ladder_lower_ = &plant_->AddRigidBody("ladder_lower", M_BBo_B);
    ladder_upper_ = &plant_->AddRigidBody("ladder_upper", M_BBo_B);

    // Both lower and upper sections have a box geometry for visualization.
    auto shape = Box(kLadderWidth_, kProblemWidth_, kLadderLength_ / 2.0);
    // We want the side of the box to rest between the pin joint and the contact
    // point at the wall. Therefore we place the geometry frame origin Go with a
    // shift -kLadderWidth_ / 2.0 in the body frame's x-axis.
    const RigidTransformd X_BV(
        Vector3d(-kLadderWidth_ / 2.0, 0.0, kLadderLength_ / 4.0));
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
        Vector3d(0.0, kProblemWidth_ / 2.0, kLadderLength_ / 2.0));
    plant_->RegisterCollisionGeometry(
        *ladder_upper_, X_BC, Sphere(point_contact_radius),
        "LadderUpperCollisionGeometry",
        CoulombFriction<double>(kFrictionCoefficient_, kFrictionCoefficient_));

    // Pin to the floor with a revolute joint.
    pin_ = &plant_->AddJoint<RevoluteJoint>("PinToGround", plant_->world_body(),
                                            {}, *ladder_lower_, {},
                                            Vector3d::UnitY(), kPinDamping_);

    // Weld the two halves.
    const RigidTransformd X_BlBu(Vector3d(0.0, 0.0, kLadderLength_ / 2.0));
    weld_ = &plant_->WeldFrames(ladder_lower_->body_frame(),
                                ladder_upper_->body_frame(), X_BlBu);

    // Add actuation.
    plant_->AddJointActuator("PinActuator", *pin_);
  }

  void VerifyJointReactionForces() {
    // We validate the numerical results to be within this tolerance value,
    // which is chosen consistently with the time the system is left to reach
    // steady state and the integration accuracy (for the continuous model).
    const double kTolerance = 1.0e-11;

    // Sanity check model size.
    ASSERT_EQ(plant_->num_bodies(), 3);
    ASSERT_EQ(plant_->num_velocities(), 1);
    ASSERT_EQ(plant_->num_actuated_dofs(), 1);

    // We run a simulation to steady state so that contact forces balance
    // gravity and actuation.
    Simulator<double> simulator(*diagram_, std::move(diagram_context_));
    // The default RK3 integrator requires specifying a very high accuracy to
    // reach steady state within kTolerance and therefore it is very costly.
    // However implicit Euler does a much better job with larger time steps.
    simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>();
    simulator.get_mutable_integrator().set_maximum_step_size(5e-3);
    simulator.get_mutable_integrator().set_target_accuracy(1e-6);
    simulator.Initialize();
    const double simulation_time = 1.0;  // seconds.
    simulator.AdvanceTo(simulation_time);

    // Evaluate the reaction forces output port to get the reaction force at the
    // pin joint. Re-express in the world frame W.
    const auto& reaction_forces =
        plant_->get_reaction_forces_output_port()
            .Eval<std::vector<SpatialForce<double>>>(*plant_context_);
    ASSERT_EQ(reaction_forces.size(), 2u);
    const SpatialForce<double>& F_Bl_Bl = reaction_forces[pin_->index()];
    const RigidTransformd X_WBl =
        ladder_lower_->EvalPoseInWorld(*plant_context_);
    const SpatialForce<double> F_Bl_W = X_WBl.rotation() * F_Bl_Bl;

    // We evaluate the contact forces so that we can perform the balance of
    // forces by hand and compare with the results obtained by evaluating the
    // reaction forces port.
    const ContactResults<double>& contact_results =
        plant_->get_contact_results_output_port().Eval<ContactResults<double>>(
            *plant_context_);
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
    const double weight = kGravity_ * kLadderMass_;

    // Position of the ladder's center of gravity.
    const Vector3d p_WBcm =
        plant_->CalcCenterOfMassPositionInWorld(*plant_context_);

    // The x component of the contact force must counteract the torque due to
    // gravity plus the actuation torque.
    const double tau_g = p_WBcm.x() * weight;  // gravity torque about Bo.
    const double fc_x = (tau_g + kActuationTorque_) / p_WC.z();
    const Vector3d f_Bl_W_expected(fc_x, 0.0, weight);
    EXPECT_TRUE(
        CompareMatrices(F_Bl_W.translational(), f_Bl_W_expected, kTolerance));

    // Expected contact force.
    const Vector3d f_C_W_expected(-fc_x, 0.0, 0.0);
    EXPECT_TRUE(CompareMatrices(f_Bc_W, f_C_W_expected, kTolerance));

    // Since the contact point was purposely located at
    // y = kProblemWidth_ / 2.0, the contact force causes a reaction
    // torque at the pin joint oriented along the z-axis.
    const Vector3d t_Bl_W_expected(0.0, kActuationTorque_,
                                   -fc_x * kProblemWidth_ / 2.0);
    EXPECT_TRUE(
        CompareMatrices(F_Bl_W.rotational(), t_Bl_W_expected, kTolerance));

    // Verify reaction forces at the weld joint.
    const RigidTransformd X_WBu =
        ladder_upper_->EvalPoseInWorld(*plant_context_);
    const SpatialForce<double>& F_Bu_W =
        X_WBu.rotation() * reaction_forces[weld_->index()];
    const Vector3d f_Bu_expected(fc_x, 0.0, weight / 2.0);
    const double t_Bu_y =
        -(p_WBcm.x() / 2.0) * (weight / 2.0) + fc_x * p_WBcm.z();
    const Vector3d t_Bu_expected(0.0, t_Bu_y, -fc_x * kProblemWidth_ / 2.0);
    EXPECT_TRUE(
        CompareMatrices(F_Bu_W.rotational(), t_Bu_expected, kTolerance));
    EXPECT_TRUE(
        CompareMatrices(F_Bu_W.translational(), f_Bu_expected, kTolerance));
  }

  // This problem essentially is two-dimensional.
  // This is the length in the direction normal to the x-z plane, along the
  // y-axis.
  const double kProblemWidth_{1.0};  // [m]

  // Ladder parameters.
  const double kLadderLength_{2.0};         // [m]
  const double kLadderMass_{7.0};           // [kg]
  const double kLadderWidth_{0.15};         // [m]
  const double kFrictionCoefficient_{0.0};  // Frictionless contact, [-]

  // Wall parameters.
  const double kWallWidth_{0.3};   // [m]
  const double kWallHeight_{3.0};  // [m]

  // Pin joint parameters.
  const double kDistanceToWall_{1.0};
  const double kPinDamping_{0.0};        // [N⋅m⋅s]
  const double kActuationTorque_{20.0};  // [N⋅m]

  // We round off gravity for simpler numbers.
  const double kGravity_{10.0};  // [m/s²]

  lcm::DrakeLcm lcm_;  // For visualization.
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* ladder_lower_{nullptr};
  const RigidBody<double>* ladder_upper_{nullptr};
  const RevoluteJoint<double>* pin_{nullptr};
  const WeldJoint<double>* weld_{nullptr};
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};
};

TEST_F(LadderTest, PinReactionForcesContinuous) {
  BuildLadderModel(0);
  ASSERT_FALSE(plant_->is_discrete());
  VerifyJointReactionForces();
}

TEST_F(LadderTest, PinReactionForcesDiscrete) {
  BuildLadderModel(1.0e-3);
  ASSERT_TRUE(plant_->is_discrete());
  VerifyJointReactionForces();
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

}  // namespace
}  // namespace multibody
}  // namespace drake
