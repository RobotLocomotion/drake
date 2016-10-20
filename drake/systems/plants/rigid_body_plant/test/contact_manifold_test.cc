#include "drake/systems/plants/rigid_body_plant/sampled_contact_manifold.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_plant {
namespace test {
namespace {

using std::unique_ptr;
using std::make_unique;
using std::move;

typedef ContactDetail<double> TestContactDetail;
typedef Vector3<double> TestVector;
typedef WrenchVector<double> TestWrench;

template <typename DerivedA, typename DerivedB>
bool CompareMatrices(const Eigen::MatrixBase<DerivedA>& m1,
                     const Eigen::MatrixBase<DerivedB>& m2) {
  return CompareMatrices(m1, m2, 1e-14 /*threshold*/,
                         MatrixCompareType::absolute);
}

// Tests the sample collision manifold; adding data and performing calculations.
class SampleCollisionManifoldTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Do not change these values; tests rely on them as is.
    p0_ = TestVector::Zero();
    w0_ = TestWrench::Zero();
    p1_ << 0, 1, 0;
    w1_ << 0, 0, 0, 0, 1, 0;
    p2_ << 0, 0, 1;
    w2_ << 0, 0, 0, 0, 0, 2;
    p3_ << 1, 0, 1;
    w3_ << 0, 0, 0, 1, 0, 0;
    origin_ << -10, -10, -10;
  }

 protected:
  TestVector p0_;
  TestWrench w0_;
  TestVector p1_;
  TestWrench w1_;
  TestVector p2_;
  TestWrench w2_;
  TestVector p3_;
  TestWrench w3_;
  // Arbitrary point away from contacts to serve as center of mass.
  TestVector origin_;
};

TestWrench ApplyWrench(const TestVector& center_of_mass,
                       const TestVector& point, const TestWrench& wrench) {
  TestWrench result = TestWrench::Zero();
  result.template tail<3>() += wrench.template tail<3>();
  TestVector torque = (point - center_of_mass).cross(wrench.template tail<3>());
  result.template head<3>() += torque + wrench.template head<3>();
  return result;
}
// Utility function for applying all of the wrenches in a manifold to a
// center of mass.
TestWrench ApplyManifold(const TestVector& center_of_mass,
                         const ContactManifold<double>& manifold) {
  TestWrench result = TestWrench::Zero();
  for (int i = 0; i < manifold.get_num_contacts(); ++i) {
    auto detail = manifold.get_ith_contact(i);
    result += ApplyWrench(center_of_mass, detail->get_application_point(),
                          detail->get_wrench());
  }
  return result;
}

// Performs test on a manifold without appreciable forces.  This could be due
// due to the fact that the manifold is empty, or that the applied forces
// are, in fact, zero.
TEST_F(SampleCollisionManifoldTest, ZeroForceManifold) {
  // Case 1: Empty manifold.
  SampledContactManifold<double> manifold;
  auto net = manifold.ComputeNetResponse();
  EXPECT_TRUE(CompareMatrices(net.get_application_point(), TestVector::Zero()));
  EXPECT_TRUE(CompareMatrices(net.get_wrench(), TestWrench::Zero()));

  // Case 2: Manifold with single zero force.
  auto zero_contact = make_unique<TestContactDetail>(p0_, w0_);
  manifold.AddContactDetail(move(zero_contact));
  net = manifold.ComputeNetResponse();
  EXPECT_TRUE(CompareMatrices(net.get_application_point(), TestVector::Zero()));
  EXPECT_TRUE(CompareMatrices(net.get_wrench(), TestWrench::Zero()));

  // Case 3: Manifold with two zero forces.
  zero_contact = make_unique<TestContactDetail>(p1_, w0_);
  manifold.AddContactDetail(move(zero_contact));
  net = manifold.ComputeNetResponse();
  EXPECT_TRUE(CompareMatrices(net.get_application_point(), TestVector::Zero()));
  EXPECT_TRUE(CompareMatrices(net.get_wrench(), TestWrench::Zero()));
}

// Performs the work of adding details to the manifold, confirming that they
// are properly registered and that the data comes back as provided.
TEST_F(SampleCollisionManifoldTest, AddDetail) {
  // Case 1: Empty manifold
  SampledContactManifold<double> manifold;
  EXPECT_EQ(0, manifold.get_num_contacts());
  EXPECT_THROW(manifold.get_ith_contact(0), std::logic_error);

  // Case 2: Manifold with single zero force.
  auto zero_contact = make_unique<TestContactDetail>(p0_, w0_);
  manifold.AddContactDetail(move(zero_contact));
  EXPECT_EQ(1, manifold.get_num_contacts());
  auto detail = manifold.get_ith_contact(0);
  TestVector p = detail->get_application_point();
  TestWrench w = detail->get_wrench();
  EXPECT_TRUE(CompareMatrices(p, TestVector::Zero()));
  EXPECT_TRUE(CompareMatrices(w, TestWrench::Zero()));

  // Case 3: Manifold with two forces.
  auto unitYForce = make_unique<TestContactDetail>(p1_, w1_);
  manifold.AddContactDetail(move(unitYForce));
  EXPECT_EQ(2, manifold.get_num_contacts());
  detail = manifold.get_ith_contact(1);
  p = detail->get_application_point();
  w = detail->get_wrench();
  EXPECT_TRUE(CompareMatrices(p, p1_));
  EXPECT_TRUE(CompareMatrices(w, w1_));

  // Case 3: Manifold with three forces.
  auto zForce = make_unique<TestContactDetail>(p2_, w2_);
  manifold.AddContactDetail(move(zForce));
  EXPECT_EQ(3, manifold.get_num_contacts());
  detail = manifold.get_ith_contact(2);
  p = detail->get_application_point();
  w = detail->get_wrench();
  EXPECT_TRUE(CompareMatrices(p, p2_));
  EXPECT_TRUE(CompareMatrices(w, w2_));
}

// Tests the net force computation for a manifold with non-zero Forces which
// provide no torque.  The validity of the net wrench is determined by applying
// the net wrench to an arbitrary center of mass (measured and expressed in
// world frame) to the full manifold applied to the same point.  The resultant
// wrench *on that point* should be the same.
TEST_F(SampleCollisionManifoldTest, NetFromTorqueOnly) {
  SampledContactManifold<double> manifold;

  // Case 1: Manifold contains a single force with a unit torque.
  TestWrench w1;
  w1 << 1, 0, 0, 0, 0, 0;
  auto x_torque = make_unique<TestContactDetail>(p0_, w1);
  manifold.AddContactDetail(move(x_torque));
  auto net = manifold.ComputeNetResponse();
  EXPECT_EQ(net.get_application_point(), TestVector::Zero());

  TestWrench expected_wrench = ApplyManifold(origin_, manifold);
  TestWrench net_wrench =
      ApplyWrench(origin_, net.get_application_point(), net.get_wrench());
  EXPECT_TRUE(CompareMatrices(net_wrench, expected_wrench));

  // Case 2: Manifold contains two forces, each with different unit torques.
  TestWrench w2;
  w2 << 0, 1, 0, 0, 0, 0;
  auto y_torque = make_unique<TestContactDetail>(p2_, w2);
  manifold.AddContactDetail(move(y_torque));
  net = manifold.ComputeNetResponse();
  expected_wrench = ApplyManifold(origin_, manifold);
  net_wrench =
      ApplyWrench(origin_, net.get_application_point(), net.get_wrench());
  EXPECT_TRUE(CompareMatrices(net_wrench, expected_wrench));
}

// Tests the net force computation for a manifold with non-zero Forces which
// provide only torque.  The validity of the net wrench is determined by
// applying the net wrench to an arbitrary center of mass (measured and
// expressed in world frame) to the full manifold applied to the same point.
// The resultant wrench *on that point* should be the same.
TEST_F(SampleCollisionManifoldTest, NetFromForceOnly) {
  SampledContactManifold<double> manifold;

  // Case 1: Manifold with single force with no torque.
  auto unit_y_force = make_unique<TestContactDetail>(p1_, w1_);
  manifold.AddContactDetail(move(unit_y_force));
  auto net = manifold.ComputeNetResponse();
  EXPECT_EQ(net.get_application_point(), p1_);
  EXPECT_EQ(net.get_wrench(), w1_);

  // Case 2: Manifold contains two symmetric, planar forces with no torques.
  auto zForce = make_unique<TestContactDetail>(p2_, w2_);
  manifold.AddContactDetail(move(zForce));
  net = manifold.ComputeNetResponse();
  TestVector expected_point;
  expected_point << 0, 1 / 3., 2 / 3.;
  EXPECT_TRUE(CompareMatrices(net.get_application_point(), expected_point));
  TestWrench expected_wrench = ApplyManifold(origin_, manifold);
  TestWrench net_wrench =
      ApplyWrench(origin_, net.get_application_point(), net.get_wrench());
  EXPECT_TRUE(CompareMatrices(net_wrench, expected_wrench));

  // Case 3: Manifold contains two non-planar forces with no torques.
  SampledContactManifold<double> manifold2;
  unit_y_force = make_unique<TestContactDetail>(p1_, w1_);
  manifold2.AddContactDetail(move(unit_y_force));
  auto skewForce = make_unique<TestContactDetail>(p3_, w3_);
  manifold2.AddContactDetail(move(skewForce));
  net = manifold2.ComputeNetResponse();
  expected_point << 0.5, 0.5, 0.5;
  EXPECT_TRUE(CompareMatrices(net.get_application_point(), expected_point));

  expected_wrench = ApplyManifold(origin_, manifold2);
  net_wrench =
      ApplyWrench(origin_, net.get_application_point(), net.get_wrench());
  EXPECT_TRUE(CompareMatrices(net_wrench, expected_wrench));
}
}  // namespace
}  // namespace test
}  // namespace rigid_body_plant
}  // namespace plants
}  // namespace systems
}  // namespace drake
