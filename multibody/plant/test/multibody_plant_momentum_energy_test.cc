#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(EmptyMultibodyPlantMomentumTest, CalcSpatialMomentumEmptyPlant) {
  MultibodyPlant<double> plant(0.0);  // Empty plant.
  plant.Finalize();
  std::unique_ptr<systems::Context<double>> context =
      plant.CreateDefaultContext();
  const Vector3<double> p_WoP_W(3, 5, 7);
  const SpatialMomentum<double> spatial_momentum =
      plant.CalcSpatialMomentumInWorldAboutPoint(*context, p_WoP_W);
  EXPECT_EQ(spatial_momentum.get_coeffs(), Vector6<double>::Zero());
}

// Fixture for a two degree-of-freedom pendulum having two links A and B.
// Link A is connected to world (frame W) with a z-axis pin joint (PinJoint1).
// Link B is connected to link A with another z-axis pin joint (PinJoint2).
// Hence links A and B only move in the world's x-y plane (perpendicular to Wz).
// The long axis of link A is parallel to A's unit vector Ax and
// the long axis of link B is parallel to B's unit vector Bx.
// PinJoint1 connects point Wo (world frame W's origin) to link A.
// PinJoint2 connects point Fo (frame F's origin) and Mo (frame M's origin)
// where frame F is fixed/welded to link A and frame M is fixed to link B.
// In the baseline configuration, the origin points Wo Ao Fo Mo Bo are
// sequential along the links (they form a line parallel to Wx = Ax = Bx).
// Note: The applied forces (such as gravity) on this system are irrelevant as
// the plan for this text fixture is limited to testing spatial momentum and
// kinetic energy for an instantaneous given state.
class TwoDofPlanarPendulumTest : public ::testing::Test {
 public:
  // Setup the MBP.
  void SetUp() override {
    // Set a spatial inertia for each link.
    const RotationalInertia<double> I_BBcm(0, Izz_, Izz_);
    const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
    const SpatialInertia<double> M_Bcm =
        SpatialInertia<double>::MakeFromCentralInertia(mass_link_, p_BoBcm_B,
                                                       I_BBcm);

    // Create an empty MultibodyPlant and then add the two links.
    bodyA_model_instance_ = plant_.AddModelInstance("bodyA_model_instance");
    bodyB_model_instance_ = plant_.AddModelInstance("bodyB_model_instance");
    bodyA_ = &plant_.AddRigidBody("BodyA", bodyA_model_instance_, M_Bcm);
    bodyB_ = &plant_.AddRigidBody("BodyB", bodyB_model_instance_, M_Bcm);

    // Create a revolute joint that connects point Wo of the world frame to a
    // unnamed point of link A.  The revolute joint is a distance link_length/2
    // from link A's centroid (point Ao).
    const Vector3<double> p_AoWo_A(-0.5 * link_length_, 0.0, 0.0);
    joint1_ = &plant_.AddJoint<RevoluteJoint>(
        "PinJoint1", plant_.world_body(), std::nullopt, *bodyA_,
        math::RigidTransformd(p_AoWo_A), Vector3<double>::UnitZ());

    // Create a revolute joint that connects point Fo (frame F's origin) to
    // point Mo (frame M's origin), where frame F is fixed/welded to the
    // distal end of link A (Fo is a distance of link_length/2 from Ao) and
    // frame M is fixed/welded to link B.  Mo is a distance of link_length/2
    // from link B's centroid (point Bo).
    const Vector3<double> p_AoFo_A(0.5 * link_length_, 0.0, 0.0);
    const Vector3<double> p_BoMo_B(-0.5 * link_length_, 0.0, 0.0);
    joint2_ = &plant_.AddJoint<RevoluteJoint>(
        "PinJoint2", *bodyA_, math::RigidTransformd(p_AoFo_A), *bodyB_,
        math::RigidTransformd(p_BoMo_B), Vector3<double>::UnitZ());

    // Finalize the plant and create a context to store this plant's state.
    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();

    // Set joints angle and rates to default values.
    const double qA = 0.0, qB = 0;  // Link angles.
    joint1_->set_angle(context_.get(), qA);
    joint2_->set_angle(context_.get(), qB);
    joint1_->set_angular_rate(context_.get(), wAz_);
    joint2_->set_angular_rate(context_.get(), wBz_);

    // Calculate the expected spatial momentum of each body by separately
    // calculating the translational and angular momentum of each body in world.
    // Analytical result for each body's translational momentum in world.
    const double vA = 0.5 * link_length_ * wAz_;
    const double vB = link_length_ * wAz_ + 0.5 * link_length_ * (wAz_ + wBz_);
    const Vector3<double> bodyA_translational_momentum_expected =
        mass_link_ * vA * Vector3<double>::UnitY();
    const Vector3<double> bodyB_translational_momentum_expected =
        mass_link_ * vB * Vector3<double>::UnitY();

    // Analytical result for each body's angular momentum in world about Wo.
    const double mLL = mass_link_ * link_length_ * link_length_;
    const Vector3<double> bodyA_angular_momentum_about_Wo =
        (Izz_ * wAz_ + 0.25 * mLL * wAz_) * Vector3<double>::UnitZ();
    const Vector3<double> bodyB_angular_momentum_about_Wo =
        (Izz_ * (wAz_ + wBz_) + 2.25 * mLL * wAz_ + 0.75 * mLL * wBz_) *
        Vector3<double>::UnitZ();

    // Consolidate translational and angular momentum into spatial momentum.
    L_WAWo_W_expected_ = SpatialMomentum<double>(
        bodyA_angular_momentum_about_Wo, bodyA_translational_momentum_expected);
    L_WBWo_W_expected_ = SpatialMomentum<double>(
        bodyB_angular_momentum_about_Wo, bodyB_translational_momentum_expected);
  }

 protected:
  MultibodyPlant<double> plant_{0.0};
  std::unique_ptr<systems::Context<double>> context_;
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const RevoluteJoint<double>* joint1_{nullptr};
  const RevoluteJoint<double>* joint2_{nullptr};
  ModelInstanceIndex bodyA_model_instance_;
  ModelInstanceIndex bodyB_model_instance_;

  // Expected results to be used in the tests below.
  SpatialMomentum<double> L_WAWo_W_expected_;
  SpatialMomentum<double> L_WBWo_W_expected_;

  // Since the maximum speed in this test is ≈ ω * (2 * link_length) ≈ 24,
  // and mass_link_ = 5, we test that the errors in translational momentum
  // calculations are less than 24 * 5 = 120, ≈ 7 bits (2^7 = 128).
  const double kTolerance_ = 128 * std::numeric_limits<double>::epsilon();

 private:
  const double mass_link_ = 5.0;    // kg
  const double link_length_ = 4.0;  // meters

  // Ixx = 0, Iyy = Izz = m L^2 / 12 are principal moments of inertia for a thin
  // link of mass m and length L about its center of mass.
  const double Izz_ = mass_link_ * link_length_ * link_length_ / 12.0;

  // Default rates of this double pendulum's revolute joints.
  const double wAz_ = 3.0;  // rad/sec
  const double wBz_ = 2.0;  // rad/sec
};

TEST_F(TwoDofPlanarPendulumTest, CalcSpatialMomentumEmptySet) {
  // Form the spatial momentum in the world frame W for an empty list E of
  // ModelInstanceIndex about an arbitrary point P, expressed in W.
  const std::vector<ModelInstanceIndex> model_instances;
  const Vector3<double> p_WoP_W(4, 6, 8);
  const SpatialMomentum<double> L_WEP_W =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                  p_WoP_W);
  EXPECT_EQ(L_WEP_W.get_coeffs(), Vector6<double>::Zero());
}

TEST_F(TwoDofPlanarPendulumTest, CalcSpatialMomentumNonsenseSet) {
  // Ensure a bad error instance in model_instances throws an exception.
  std::vector<ModelInstanceIndex> model_instances;
  ModelInstanceIndex error_index(123);  // 123 is intentionally nonsensical.
  model_instances.push_back(error_index);
  const Vector3<double> p_WoWo_W = Vector3<double>::Zero();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                  p_WoWo_W),
      "CalcSpatialMomentumInWorldAboutPoint\\(\\): This MultibodyPlant method"
      " contains an invalid model_instance.");
}

TEST_F(TwoDofPlanarPendulumTest, CalcBodyASpatialMomentumInWorldAboutPoint) {
  // Calculate body A's spatial momentum in world W, about Wo, expressed in W.
  // Compare to expected results.
  std::vector<ModelInstanceIndex> model_instances;
  model_instances.push_back(bodyA_model_instance_);
  const Vector3<double> p_WoWo_W = Vector3<double>::Zero();
  const SpatialMomentum<double> L_WAWo_W =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                  p_WoWo_W);
  EXPECT_TRUE(CompareMatrices(L_WAWo_W.get_coeffs(),
                              L_WAWo_W_expected_.get_coeffs(), kTolerance_));

  // Calculate body A's spatial momentum in world W, about Acm, expressed in W.
  // Compare to expected results.
  const Vector3<double> p_WoAcm_W =
      plant_.CalcCenterOfMassPositionInWorld(*context_, model_instances);
  const SpatialMomentum<double> L_WAcm_W =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                  p_WoAcm_W);
  const SpatialMomentum<double> L_WAcm_W_expected =
      L_WAWo_W_expected_.Shift(p_WoAcm_W);
  EXPECT_TRUE(CompareMatrices(L_WAcm_W.get_coeffs(),
                              L_WAcm_W_expected.get_coeffs(), kTolerance_));
}

TEST_F(TwoDofPlanarPendulumTest, CalcBodyBSpatialMomentumInWorldAboutPoint) {
  // Calculate body B's spatial momentum in world W, about Bcm, expressed in W.
  // Compare to expected results.
  std::vector<ModelInstanceIndex> model_instances;
  model_instances.push_back(bodyB_model_instance_);
  const Vector3<double> p_WoBcm_W =
      plant_.CalcCenterOfMassPositionInWorld(*context_, model_instances);
  const SpatialMomentum<double> L_WBcm_W =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                  p_WoBcm_W);
  const SpatialMomentum<double> L_WBcm_W_expected =
      L_WBWo_W_expected_.Shift(p_WoBcm_W);
  EXPECT_TRUE(CompareMatrices(L_WBcm_W.get_coeffs(),
                              L_WBcm_W_expected.get_coeffs(), kTolerance_));
}

TEST_F(TwoDofPlanarPendulumTest, CalcSystemSpatialMomentumInWorldAboutWo) {
  // Assemble model instances using methods typically employed by end-users.
  const ModelInstanceIndex bodyA_model_instance =
      plant_.GetModelInstanceByName("bodyA_model_instance");
  const ModelInstanceIndex bodyB_model_instance =
      plant_.GetBodyByName("BodyB").model_instance();
  const std::vector<ModelInstanceIndex> model_instances{bodyA_model_instance,
                                                        bodyB_model_instance};

  // For the system S consisting of bodyA and bodyB, form the system S's
  // spatial momentum in world W about Wo, expressed in W.
  const Vector3<double> p_WoWo_W = Vector3<double>::Zero();
  SpatialMomentum<double> L_WSWo_W =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                  p_WoWo_W);

  // Form S's expected spatial momentum and compare previous answer.
  const SpatialMomentum<double> L_WSWo_W_expected =
      L_WAWo_W_expected_ + L_WBWo_W_expected_;
  EXPECT_TRUE(CompareMatrices(L_WSWo_W.get_coeffs(),
                              L_WSWo_W_expected.get_coeffs(), kTolerance_));

  // Use a different MultibodyPlant method to form the plant system S's spatial
  // momentum in world W about Scm (system's center of mass), expressed in W.
  L_WSWo_W = plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, p_WoWo_W);
  EXPECT_TRUE(CompareMatrices(L_WSWo_W.get_coeffs(),
                              L_WSWo_W_expected.get_coeffs(), kTolerance_));
}

TEST_F(TwoDofPlanarPendulumTest, CalcSystemSpatialMomentumInWorldAboutPoint) {
  // For the system S consisting of bodyA and bodyB, form the system S's spatial
  // momentum in world W about Scm (the system's center of mass) expressed in W.
  // Compare to expected results.
  std::vector<ModelInstanceIndex> model_instances;
  model_instances.push_back(bodyB_model_instance_);
  model_instances.push_back(bodyA_model_instance_);
  const Vector3<double> p_WoScm_W =
      plant_.CalcCenterOfMassPositionInWorld(*context_, model_instances);
  SpatialMomentum<double> L_WScm_W =
      plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                  p_WoScm_W);
  const SpatialMomentum<double> L_WSWo_W_expected =
      L_WAWo_W_expected_ + L_WBWo_W_expected_;
  const SpatialMomentum<double> L_WScm_W_expected =
      L_WSWo_W_expected.Shift(p_WoScm_W);
  EXPECT_TRUE(CompareMatrices(L_WScm_W.get_coeffs(),
                              L_WScm_W_expected.get_coeffs(), kTolerance_));

  // Use a different MultibodyPlant method to form the plant system S's spatial
  // momentum in world W about Scm (system's center of mass), expressed in W.
  L_WScm_W = plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, p_WoScm_W);
  EXPECT_TRUE(CompareMatrices(L_WScm_W.get_coeffs(),
                              L_WScm_W_expected.get_coeffs(), kTolerance_));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
