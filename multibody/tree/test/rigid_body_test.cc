#include "drake/multibody/tree/rigid_body.h"

#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/multibody_tree-inl.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using systems::Context;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

// Test rigid body 2-argument constructor.
GTEST_TEST(RigidBody, RigidBodyConstructor2) {
  // Construct a rigid body with a spatial inertia.
  const double mass = 2;
  const Vector3d p_BoBcm_B(0.4, 0.3, 0.2);
  const UnitInertia<double> G_BBo_B(6, 7, 8);
  const SpatialInertia<double> M_Bo_B(mass, p_BoBcm_B, G_BBo_B);
  const RigidBody<double> B("foo", M_Bo_B);

  // Test that the name made it through.
  EXPECT_EQ(B.name(), "foo");

  // Test that after construction, RigidBody class properly returns its default
  // spatial inertia or parts of it.
  // TODO(amcastro-tri): Replace with EXPECT_EQ once operator==() is overloaded.
  EXPECT_EQ(B.default_spatial_inertia().get_mass(), M_Bo_B.get_mass());
  EXPECT_EQ(B.default_spatial_inertia().get_com(), M_Bo_B.get_com());
  EXPECT_EQ(B.default_spatial_inertia().get_unit_inertia().get_moments(),
            M_Bo_B.get_unit_inertia().get_moments());
  EXPECT_EQ(B.default_spatial_inertia().get_unit_inertia().get_products(),
            M_Bo_B.get_unit_inertia().get_products());
  EXPECT_EQ(B.default_mass(), mass);
  EXPECT_EQ(B.default_com(), p_BoBcm_B);
  const UnitInertia<double>& G_BBo_B_default = B.default_unit_inertia();
  EXPECT_EQ(G_BBo_B_default.get_moments(), G_BBo_B.get_moments());
  EXPECT_EQ(G_BBo_B_default.get_products(), G_BBo_B.get_products());

  // Test that RigidBody class properly calculates rotational inertia.
  const RotationalInertia<double> I_BBo_B_expected = mass * G_BBo_B;
  const RotationalInertia<double> I_BBo_B = B.default_rotational_inertia();
  EXPECT_TRUE(I_BBo_B.IsNearlyEqualTo(I_BBo_B_expected, 4.0 * kEpsilon));
}

// Test rigid body 1-argument constructor.
GTEST_TEST(RigidBody, RigidBodyConstructor1) {
  // Construct a rigid body without specifying any inertia.
  const RigidBody<double> B("bar");
  EXPECT_EQ(B.name(), "bar");

  // Sanity check that the inertia was zero.
  EXPECT_GE(B.default_spatial_inertia().get_mass(), 0.0);
  EXPECT_EQ(B.default_spatial_inertia().get_com(), Vector3d::Zero());
  EXPECT_GE(B.default_mass(), 0.0);
  EXPECT_EQ(B.default_com(), Vector3d::Zero());
}

// Fixture for a MultibodyTree model with a single rigid body and a Context.
class RigidBodyTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Create an empty model and then add a rigid body.
    auto model = std::make_unique<internal::MultibodyTree<double>>();
    const auto M_BBo_B = SpatialInertia<double>::NaN();
    rigid_body_ = &model->AddRigidBody("rigidBody_B", M_BBo_B);

    // Finalize the model and create a default context for this system.
    model->Finalize();
    mbtree_system_ = std::make_unique<internal::MultibodyTreeSystem<double>>(
        std::move(model));
    context_ = mbtree_system_->CreateDefaultContext();
  }

 protected:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> mbtree_system_;
  std::unique_ptr<Context<double>> context_;
  const RigidBody<double>* rigid_body_{nullptr};
};

// Test RigidBody::SetCenterOfMassInBodyFrame(). Reminder: calling this function
// for a rigid body B does not modify I_BBo_B (B's rotational inertia about B's
// body origin Bo) which can cause an invalid I_BBcm_B (B's rotational inertia
// about B's center of mass Bcm).
TEST_F(RigidBodyTest, SetCenterOfMassInBodyFrame) {
  // Construct a multibody plant having a cube B of dimension 2*L with a spatial
  // inertia with all the mass concentrated at Bcm (B's center of mass).
  double mass = 3;     // mass of body B in kilograms.
  const double L = 2;  // x-measure of Bcm's position from Bo in meters.
  Vector3d p_BoBcm_B(L, 0, 0);  // Position from Bo to Bcm, expressed in B.
  const UnitInertia<double> G_BBo_B(0, L * L, L * L);
  SpatialInertia<double> M_BBo_B(mass, p_BoBcm_B, G_BBo_B);
  systems::Context<double>* context_ptr = context_.get();
  rigid_body_->SetSpatialInertiaInBodyFrame(context_ptr, M_BBo_B);

  // Test that the cube has the proper mass, center of mass, inertia properties.
  constexpr double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  const SpatialInertia<double> M_BBo_B_test =
      rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  EXPECT_TRUE(CompareMatrices(M_BBo_B_test.CopyToFullMatrix6(),
                              M_BBo_B.CopyToFullMatrix6(), kTolerance));

  // Verify rotational inertia is of a single particle at B's center of mass.
  RotationalInertia<double> I_BBo_B_expected(mass, p_BoBcm_B);
  RotationalInertia<double> I_BBo_B = M_BBo_B.CalcRotationalInertia();
  EXPECT_TRUE(CompareMatrices(I_BBo_B.CopyToFullMatrix3(),
                              I_BBo_B_expected.CopyToFullMatrix3(),
                              kTolerance));

  // Verify B's unit inertia about Bcm is zero.
  SpatialInertia<double> M_BBcm_B = M_BBo_B.Shift(p_BoBcm_B);
  UnitInertia<double> G_BBcm_B = M_BBcm_B.get_unit_inertia();
  UnitInertia<double> G_BBcm_B_expected(0, 0, 0);
  EXPECT_TRUE(G_BBcm_B.IsNearlyEqualTo(G_BBcm_B_expected, kTolerance));

  // Change the body's mass to 1 and ensure it propagates properly, meaning the
  // mass changes, p_BoBcm_B is unchanged, G_BBo_B and G_BBcm_B are unchanged.
  mass = 1;
  rigid_body_->SetMass(context_ptr, mass);
  EXPECT_EQ(rigid_body_->get_mass(*context_), mass);
  M_BBo_B = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  SpatialInertia<double> M_BBo_B_expected(mass, p_BoBcm_B, G_BBo_B);
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
                              M_BBo_B_expected.CopyToFullMatrix6(),
                              kTolerance));
  M_BBcm_B = M_BBo_B.Shift(p_BoBcm_B);
  G_BBcm_B = M_BBcm_B.get_unit_inertia();
  EXPECT_TRUE(G_BBcm_B.IsNearlyEqualTo(G_BBcm_B_expected, kTolerance));

  // Change p_BoBcm_B and ensure it propagates albeit in a weird way, with
  // unchanged mass, changed p_Bo_Bcm_B, unchanged G_BBo_B, changed G_BBcm_B.
  p_BoBcm_B = Vector3d(L / 2, 0, 0);  // Now p_BoBcm_B = (1, 0, 0).
  rigid_body_->SetCenterOfMassInBodyFrame(context_ptr, p_BoBcm_B);
  const Vector3d p_BoBcm_B_calculated =
      rigid_body_->CalcCenterOfMassInBodyFrame(*context_);
  EXPECT_EQ(p_BoBcm_B_calculated, p_BoBcm_B);  // Position vector changes.
  EXPECT_EQ(rigid_body_->get_mass(*context_), mass);  // Mass is unchanged.
  M_BBo_B = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  const UnitInertia<double> G_BBo_B_test = M_BBo_B.get_unit_inertia();
  EXPECT_TRUE(CompareMatrices(G_BBo_B_test.CopyToFullMatrix3(),
                              G_BBo_B.CopyToFullMatrix3(),
                              kTolerance));  // G_BBo_B is unchanged.

  // Ensure M_BBo_B has proper mass, proper p_BoBcm_B, and unchanged G_BBo_B.
  M_BBo_B_expected = SpatialInertia<double>(mass, p_BoBcm_B, G_BBo_B);
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
                              M_BBo_B_expected.CopyToFullMatrix6(),
                              kTolerance));

  // Verify G_BBcm_B changes in a strange way if p_BBcm_B is changed!
  // G_BBcm_B is _not_ the unit inertia of a particle at B's center of mass.
  G_BBcm_B = G_BBo_B.ShiftToCenterOfMass(p_BoBcm_B);
  G_BBcm_B_expected = UnitInertia<double>(0, 3, 3);  // By-hand calculation.
  EXPECT_TRUE(CompareMatrices(G_BBcm_B.CopyToFullMatrix3(),
                              G_BBcm_B_expected.CopyToFullMatrix3(),
                              kTolerance));

  // Show that calling SetCenterOfMassInBodyFrame() can cause an exception to
  // be thrown due to non-physical mass/inertia properties.
#ifdef DRAKE_ASSERT_IS_ARMED
  p_BoBcm_B = Vector3d(3 * L, 0, 0);  // Now p_BoBcm_B = (6, 0, 0).
  rigid_body_->SetCenterOfMassInBodyFrame(context_ptr, p_BoBcm_B);

  // CalcSpatialInertiaInBodyFrame() does not check whether B's inertia
  // properties make sense. However, the spatial inertia constructor does.
  M_BBo_B = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  const double m = M_BBo_B.get_mass();
  const Vector3d com = M_BBo_B.get_com();
  const UnitInertia<double> G = M_BBo_B.get_unit_inertia();
  // Ensure M_BBcm_B is invalid via the spatial inertia constructor.
  DRAKE_EXPECT_THROWS_MESSAGE(
      SpatialInertia<double>(m, com, G),
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid[^]*");
#endif
}

TEST_F(RigidBodyTest, SetCenterOfMassInBodyFrameAndPreserveCentralInertia) {
  // Create a rigid body B that is a line-segment of length 2*L whose spatial
  // inertia corresponds to all mass concentrated at Bcm (B's center of mass).
  double mass = 3;     // mass of body B in kilograms.
  const double L = 2;  // x-measure of Bcm's position from Bo in meters.
  Vector3d p_BoBcm_B(L, 0, 0);  // Position from Bo to Bcm, expressed in B.
  const UnitInertia<double> G_BBo_B(0, L * L, L * L);  // By-hand calculation.
  SpatialInertia<double> M_BBo_B(mass, p_BoBcm_B, G_BBo_B);
  systems::Context<double>* context_ptr = context_.get();
  rigid_body_->SetSpatialInertiaInBodyFrame(context_ptr, M_BBo_B);

  // Verify body B has the proper mass, center of mass, inertia properties.
  constexpr double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  const SpatialInertia<double> M_BBo_B_test =
      rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  EXPECT_TRUE(CompareMatrices(M_BBo_B_test.CopyToFullMatrix6(),
                              M_BBo_B.CopyToFullMatrix6(), kTolerance));

  // Verify rotational inertia is of a single particle at B's center of mass.
  RotationalInertia<double> I_BBo_B_expected(mass, p_BoBcm_B);
  RotationalInertia<double> I_BBo_B = M_BBo_B.CalcRotationalInertia();
  EXPECT_TRUE(CompareMatrices(I_BBo_B.CopyToFullMatrix3(),
                              I_BBo_B_expected.CopyToFullMatrix3(),
                              kTolerance));

  // Verify B's unit inertia about Bcm is zero.
  SpatialInertia<double> M_BBcm_B = M_BBo_B.Shift(p_BoBcm_B);
  UnitInertia<double> G_BBcm_B = M_BBcm_B.get_unit_inertia();
  UnitInertia<double> G_BBcm_B_expected_zero(0, 0, 0);
  EXPECT_TRUE(G_BBcm_B.IsNearlyEqualTo(G_BBcm_B_expected_zero, kTolerance));

  // Change the body's mass to 1 and ensure it propagates properly, meaning the
  // mass changes, p_BoBcm_B is unchanged, G_BBo_B and G_BBcm_B are unchanged.
  mass = 1;
  rigid_body_->SetMass(context_ptr, mass);
  EXPECT_EQ(rigid_body_->get_mass(*context_), mass);
  M_BBo_B = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  SpatialInertia<double> M_BBo_B_expected(mass, p_BoBcm_B, G_BBo_B);
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
                              M_BBo_B_expected.CopyToFullMatrix6(),
                              kTolerance));
  M_BBcm_B = M_BBo_B.Shift(p_BoBcm_B);
  G_BBcm_B = M_BBcm_B.get_unit_inertia();
  EXPECT_TRUE(G_BBcm_B.IsNearlyEqualTo(G_BBcm_B_expected_zero, kTolerance));

  // Change p_BoBcm_B and ensure it propagates properly.
  // First, ensure p_BoBcm_B properly changes.
  p_BoBcm_B = Vector3d(L / 2, 0, 0);  // Now p_BoBcm_B = (1, 0, 0).
  rigid_body_->SetCenterOfMassInBodyFrameAndPreserveCentralInertia(context_ptr,
                                                                   p_BoBcm_B);
  const Vector3d p_BoBcm_B_calculated =
      rigid_body_->CalcCenterOfMassInBodyFrame(*context_);
  EXPECT_EQ(p_BoBcm_B_calculated, p_BoBcm_B);

  // Ensure mass is unchanged.
  EXPECT_EQ(rigid_body_->get_mass(*context_), mass);

  // Ensure I_BBo_B is properly changed due to new center of mass location.
  M_BBo_B = rigid_body_->CalcSpatialInertiaInBodyFrame(*context_);
  I_BBo_B = M_BBo_B.CalcRotationalInertia();
  I_BBo_B_expected = RotationalInertia<double>(mass, p_BoBcm_B);
  EXPECT_TRUE(CompareMatrices(I_BBo_B.CopyToFullMatrix3(),
                              I_BBo_B_expected.CopyToFullMatrix3(),
                              kTolerance));

  // Verify B's unit inertia about Bcm is still zero.
  M_BBcm_B = M_BBo_B.Shift(p_BoBcm_B);
  G_BBcm_B = M_BBcm_B.get_unit_inertia();
  EXPECT_TRUE(G_BBcm_B.IsNearlyEqualTo(G_BBcm_B_expected_zero, kTolerance));

  // Ensure M_BBo_B has proper mass, proper p_BoBcm_B, and proper G_BBo_B.
  UnitInertia<double> G_BBo_B_expected =
      UnitInertia<double>(0, L * L / 4, L * L / 4);
  M_BBo_B_expected = SpatialInertia<double>(mass, p_BoBcm_B, G_BBo_B_expected);
  EXPECT_TRUE(CompareMatrices(M_BBo_B.CopyToFullMatrix6(),
                              M_BBo_B_expected.CopyToFullMatrix6(),
                              kTolerance));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
