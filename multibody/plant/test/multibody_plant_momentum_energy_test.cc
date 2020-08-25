#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant.h"
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
  const Vector3<double> p_WoP_W = Vector3<double>::Zero();
  const SpatialMomentum<double> momentum =
      plant.CalcSpatialMomentumInWorldAboutPoint(*context, p_WoP_W);
  EXPECT_EQ(momentum.get_coeffs(), Vector6<double>::Zero());
}

// Fixture for a two degree-of-freedom pendulum having two links A and B.
// Link A is connected to world (frame W) with a z-axis pin joint (PinJoint1).
// Link B is connected to link A with another z-axis pin joint (PinJoint2).
// Hence links A and B only move in the world's x-y plane (perpendicular to Wz).
// The long axis of link A is parallel to A's unit vector Ax and
// the long axis of link B is parallel to B's unit vector Bx.
// PinJoint1 and PinJoint2 are located at distal ends of the links.
// PinJoint1 is collocated with Wo (world frame origin)
// PinJoint2 is collocated with Fo (frame F's origin) and Mo (frame M's origin)
// where frame F is fixed/welded to link A and frame M is fixed to link B.
// In the baseline configuration, the origin points Wo Ao Fo Mo Bo are
// sequential along the links (they form a line parallel to Wx = Ax = Bx).
class TwoDOFPlanarPendulumTest : public ::testing::Test {
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
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    bodyA_ = &plant_->AddRigidBody("BodyA", M_Bcm);
    bodyB_ = &plant_->AddRigidBody("BodyB", M_Bcm);

    // Create a revolute joint that connects point Wo of the world frame to a
    // unnamed point of link A that is a distance of link_length/2 from link A's
    // centroid (point Ao).
    const Vector3<double> p_AoWo_A(-0.5 * link_length_, 0.0, 0.0);
    joint1_ = &plant_->AddJoint<RevoluteJoint>("PinJoint1",
        plant_->world_body(), std::nullopt,
        *bodyA_, math::RigidTransformd(p_AoWo_A), Vector3<double>::UnitZ());

    // Create a revolute joint that connects point Fo (frame F's origin) to
    // point Mo (frame M's origin), where frame F is fixed/welded to the
    // distal end of link A (Fo is a distance of link_length/2 from Ao) and
    // frame M is fixed/welded to link B.  Mo is a distance of link_length/2
    // from link B's centroid (point Bo).
    const Vector3<double> p_AoFo_A(0.5 * link_length_, 0.0, 0.0);
    const Vector3<double> p_BoMo_B(-0.5 * link_length_, 0.0, 0.0);
    joint2_ = &plant_->AddJoint<RevoluteJoint>("PinJoint2",
        *bodyA_, math::RigidTransformd(p_AoFo_A),
        *bodyB_, math::RigidTransformd(p_BoMo_B), Vector3<double>::UnitZ());

    // Finalize the plant and create a context to store this plant's state.
    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();
  }

 protected:
  const double mass_link_ = 5.0;    // kg
  const double link_length_ = 4.0;  // meters
  const double Izz_ = mass_link_ * link_length_ * link_length_ / 12.0;
  const double wAz_ = 3.0;          // rad/sec
  const double wBz_ = 2.0;          // rad/sec

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> context_;
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const RevoluteJoint<double>* joint1_{nullptr};
  const RevoluteJoint<double>* joint2_{nullptr};
};

TEST_F(TwoDOFPlanarPendulumTest, CalcSpatialMomentumInWorldAboutPoint) {
  // Since the maximum speed in this test is ≈ ω * (2 * link_length) ≈ 24,
  // and mass_link_ = 5, we test that the errors in translational momentum
  // calculations are less than 24 * 5 = 120, ≈ 7 bits (2^7 = 128).
  const double kTolerance = 128 * std::numeric_limits<double>::epsilon();

  Eigen::VectorXd state = Eigen::Vector4d(0.0, 0.0, wAz_, wBz_);
  joint1_->set_angle(context_.get(), state[0]);
  joint2_->set_angle(context_.get(), state[1]);
  joint1_->set_angular_rate(context_.get(), state[2]);
  joint2_->set_angular_rate(context_.get(), state[3]);

  // Calculate this plant's spatial momentum in world W about Wo expressed in W.
  const Vector3<double> p_WoWo_W = Vector3<double>::Zero();
  const SpatialMomentum<double> L_WS_W =
      plant_->CalcSpatialMomentumInWorldAboutPoint(*context_, p_WoWo_W);

  // By-hand analysis gives: v_WAcm_W = 0.5 L wAz Wy
  //                  and    v_WBcm_W = (1.0 L wAz + 0.5 L wBz) Wy,
  // so the plant's translational momentum is mass_link_ 2 L wAz Wy.
  const double v1 = 0.5 * link_length_ * wAz_;
  const double v2 = link_length_ * wAz_ + 0.5 * link_length_ *(wAz_ + wBz_);
  const double mv1 = mass_link_ * v1;
  const double mv2 = mass_link_ * v2;
  const Vector3<double> mv_expected = (mv1 + mv2) * Vector3<double>::UnitY();
  EXPECT_TRUE(CompareMatrices(L_WS_W.translational(),
      mv_expected, kTolerance));

  // MotionGenesis result: Izz*wBz + 2*Izz*wAz + 0.75*m*L^2*wBz + 2.5*m*L^2*wAz
  const double mLL = mass_link_ * link_length_ * link_length_;
  const double bodyA_angular_momentum_about_Wo_z =
      (Izz_ * wAz_ + 0.25 * mLL * wAz_);
  const double bodyB_angular_momentum_about_Wo_z =
      (Izz_ * (wAz_ + wBz_) + 2.25 * mLL * wAz_ + 0.75 * mLL * wBz_);
  const Vector3<double> system_angular_momentum_expected =
      (bodyA_angular_momentum_about_Wo_z + bodyB_angular_momentum_about_Wo_z) *
      Vector3<double>::UnitZ();
  EXPECT_TRUE(CompareMatrices(L_WS_W.rotational(),
                              system_angular_momentum_expected, kTolerance));

  // Ensure the spatial momentum for an empty model_instance is zero.
  std::vector<ModelInstanceIndex> model_instances;
  const SpatialMomentum<double> L_WEmpty_W =
      plant_->CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                   p_WoWo_W);
  EXPECT_EQ(L_WEmpty_W.get_coeffs(), Vector6<double>::Zero());

#if 0
  // Calculate body A's spatial momentum in world W, about Wo, expressed in W.
  const ModelInstanceIndex bodyA_model_instance = bodyA_->model_instance();
  model_instances.push_back(bodyA_model_instance);
  const SpatialMomentum<double> L_WA_W =
      plant_->CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                   p_WoWo_W);
  const Vector3<double> mv1_expected = mv1 * Vector3<double>::UnitY();
  EXPECT_TRUE(CompareMatrices(L_WA_W.translational(),
                              mv1_expected, kTolerance));
  const Vector3<double> bodyA_angular_momentum_about_Wo =
    bodyA_angular_momentum_about_Wo_z *  Vector3<double>::UnitZ();
  EXPECT_TRUE(CompareMatrices(L_WA_W.rotational(),
                              bodyA_angular_momentum_about_Wo, kTolerance));

  // Calculate body A's spatial momentum in world W, about Acm, expressed in W.
  const Vector3<double> p_WAcm = plant_->CalcCenterOfMassPosition(*context_,
      model_instances);
  const SpatialMomentum<double> L_WA_W_aboutAcm =
      plant_->CalcSpatialMomentumInWorldAboutPoint(*context_, model_instances,
                                                   p_WAcm);
  EXPECT_TRUE(CompareMatrices(L_WA_W_aboutAcm.translational(),
                              mv1_expected, kTolerance));
  const Vector3<double> bodyA_angular_momentum_about_Acm =
      Izz_ * wAz_ * Vector3<double>::UnitZ();
  EXPECT_TRUE(CompareMatrices(L_WA_W_aboutAcm.rotational(),
                              bodyA_angular_momentum_about_Acm, kTolerance));

  // Try all instances in model_instances.
  model_instances.push_back(sphere_instance_);
  result = ((p_WSo_W + p_SScm_S_) * mass_S_ + (p_WTo_W + p_TTcm_T_) * mass_T_) /
      (mass_S_ + mass_T_);
  p_WCcm = plant_.CalcCenterOfMassPosition(*context_, model_instances);
  EXPECT_TRUE(CompareMatrices(p_WCcm, result, 1e-15));

  // Try error instance in model_instances.
  ModelInstanceIndex error_index(10);
  model_instances.push_back(error_index);
  EXPECT_THROW(plant_.CalcCenterOfMassPosition(*context_, model_instances),
               std::runtime_error);
#endif
}

}  // namespace
}  // namespace multibody
}  // namespace drake
