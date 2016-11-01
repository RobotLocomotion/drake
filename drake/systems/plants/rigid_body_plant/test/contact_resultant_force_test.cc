#include "drake/systems/plants/rigid_body_plant/contact_resultant_force_calculator.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/plants/rigid_body_plant/contact_force.h"

namespace drake {
namespace systems {
namespace {

// Wrapper on CompareMatricies to simplify calls in the tests.
template <typename DerivedA, typename DerivedB>
bool AreEquivalent(const Eigen::MatrixBase<DerivedA>& m1,
                   const Eigen::MatrixBase<DerivedB>& m2) {
  const double kTolerance = 1e-14;
  return CompareMatrices(m1, m2, kTolerance, MatrixCompareType::absolute);
}

// Utility method for asserting the state of the torque component of the wrench.
template <typename DerivedA, typename DerivedB>
bool AssertTorque(const WrenchVector<DerivedA>& wrench,
                  const Eigen::MatrixBase<DerivedB>& torque) {
  Vector3<DerivedA> test_torque = wrench.template head<3>();
  return AreEquivalent(test_torque, torque);
}

// Utility method for asserting the state of the force component of the wrench.
template <typename DerivedA, typename DerivedB>
bool AssertForce(const WrenchVector<DerivedA>& wrench,
                 const Eigen::MatrixBase<DerivedB>& force) {
  Vector3<DerivedA> test_force = wrench.template tail<3>();
  return AreEquivalent(test_force, force);
}

// Tests the ContactForce class.  The class is very simple. It's only
// functionality is:
//    1. Constructor logic,
//    2. Combination of force components into output values.
GTEST_TEST(ContactResultantForceTest, ContactForceTests) {
  // Case 1. Confirm default constructor zeros out the data.
  ContactForce<double> f0;
  ASSERT_EQ(f0.get_normal_force(), Vector3<double>::Zero());
  ASSERT_EQ(f0.get_tangent_force(), Vector3<double>::Zero());
  ASSERT_EQ(f0.get_pure_torque(), Vector3<double>::Zero());
  ASSERT_EQ(f0.get_application_point(), Vector3<double>::Zero());

  Vector3<double> norm, tan, torque, pos;
  norm << 0, 0, 1;
  tan << 0, 1, 0;
  torque << 1, 0, 0;
  pos << 3, 2, 1;

  // Case 2. No pure torque constructor.
  ContactForce<double> f1(pos, norm, tan);
  ASSERT_EQ(f1.get_normal_force(), norm);
  ASSERT_EQ(f1.get_tangent_force(), tan);
  ASSERT_EQ(f1.get_pure_torque(), Vector3<double>::Zero());
  ASSERT_EQ(f1.get_application_point(), pos);

  // Case 3. Fully-specified constructor.
  ContactForce<double> f2(pos, norm, tan, torque);
  ASSERT_EQ(f2.get_normal_force(), norm);
  ASSERT_EQ(f2.get_tangent_force(), tan);
  ASSERT_EQ(f2.get_pure_torque(), torque);
  ASSERT_EQ(f2.get_application_point(), pos);

  // Case 4. Confirm norm and tangent combined in full force
  ASSERT_EQ(f2.get_force(), norm + tan);
  WrenchVector<double> expected_wrench;
  expected_wrench.template head<3>() = torque;
  expected_wrench.template tail<3>() = norm + tan;
  ASSERT_EQ(f2.get_wrench(), expected_wrench);
}

// Tests the various forms for adding forces to the accumulator.  This also
// implicitly the case where the resultant force is computed for a single
// contact force; the contact force *is* the resultant force and its application
// point is the minimum moment point.
GTEST_TEST(ContactResultantForceTest, ForceAccumulationTest) {
  Vector3<double> norm, tan, torque, pos;
  norm << 1, 2, 3;
  tan << 3, 4, 5;
  torque << 6, 7, 8;
  pos << 10, 11, 12;
  WrenchVector<double> full_wrench, torque_free_wrench;
  full_wrench.template head<3>() = torque;
  full_wrench.template tail<3>() = norm + tan;
  torque_free_wrench.template tail<3>() = norm + tan;
  torque_free_wrench.template head<3>() << 0, 0, 0;

  // Case 1: The ContactForce interface -- pass an instance of ContactForce.
  {
    ContactForce<double> force(pos, norm, tan, torque);
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(force);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();
    ASSERT_EQ(min_point, pos);
    ASSERT_EQ(wrench, full_wrench);
  }

  // Case 2: The interface for components without pure torque.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos, norm, tan);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();
    ASSERT_EQ(min_point, pos);
    ASSERT_EQ(wrench, torque_free_wrench);
  }

  // Case 3: The interface for components with all data.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos, norm, tan, torque);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();
    ASSERT_EQ(min_point, pos);
    ASSERT_EQ(wrench, full_wrench);
  }
}

// Tests the case where multiple contacts forces are provided, but the forces:
//    1. Are applied at points which all lie on the same plane.
//    2. Have normal directions perpendicular to the plane.
//    3. Have no tangential forces.
// The results should have the following properties:
//    1. The resultant wrench should have a zero torque component.
//    2. The resultant force should lie 100% in the normal direction.
//    3. The min moment point should lie on the plane with the input points.
GTEST_TEST(ContactResultantForceTest, SimplePlanarContactTest) {
  // Do *not* change these values. The tests below will become invalid.
  Vector3<double> pos1, pos2, pos3;
  Vector3<double> norm, zero;
  pos1 << 1, 0, 0;
  pos2 << 2, 0, 0;
  pos3 << 1, 5, 0;
  norm << 0, 0, 1;
  zero = Vector3<double>::Zero();
  Vector3<double> expected_point;

  // Case 1: Two identical forces -- min. moment point should lie in between
  // them.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, norm, zero, zero);
    calc.AddForce(pos2, norm, zero, zero);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();

    ASSERT_TRUE(AssertTorque(wrench, zero));
    ASSERT_TRUE(AssertForce(wrench, norm * 2));
    expected_point = (pos1 + pos2) * 0.5;
    ASSERT_TRUE(AreEquivalent(min_point, expected_point));
  }

  // Case 2: Two forces of unequal magnitudes.  Min. moment point should lie
  // between the two points based on ratio of force magnitudes.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, norm, zero, zero);
    calc.AddForce(pos2, 2.0 * norm, zero, zero);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();

    ASSERT_TRUE(AssertTorque(wrench, zero));
    ASSERT_TRUE(AssertForce(wrench, norm * 3));
    expected_point = pos1 / 3. + pos2 * 2. / 3.;
    ASSERT_TRUE(AreEquivalent(min_point, expected_point));
  }

  // Case 3: Three forces of unequal magnitude.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, norm, zero, zero);
    calc.AddForce(pos2, 2.0 * norm, zero, zero);
    calc.AddForce(pos3, 3.0 * norm, zero, zero);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();

    ASSERT_TRUE(AssertTorque(wrench, zero));
    ASSERT_TRUE(AssertForce(wrench, norm * 6));
    expected_point << 4.0 / 3, 5.0 / 2, 0;
    ASSERT_TRUE(AreEquivalent(min_point, expected_point));
  }

  // Case 4: Three forces of unequal magnitude with non-zero tangent components.
  // The minimum moment point (w.r.t. normal forces) should be the same as if
  // there were no tangent forces. However, the resultant torque will no longer
  // be zero.
  {
    Vector3<double> tan1, tan2, tan3;
    tan1 << 0.1, 0.1, 0.0;
    tan2 << -0.2, 0.1, 0.0;
    tan3 << 0.1, -0.2, 0.0;
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, norm, tan1, zero);
    calc.AddForce(pos2, 2.0 * norm, tan2, zero);
    calc.AddForce(pos3, 3.0 * norm, tan3, zero);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();

    Vector3<double> expected_torque;
    expected_torque << 0, 0, -0.4;
    ASSERT_TRUE(AssertTorque(wrench, expected_torque));
    ASSERT_TRUE(AssertForce(wrench, norm * 6));
    expected_point << 4.0 / 3, 5.0 / 2, 0;
    ASSERT_TRUE(AreEquivalent(min_point, expected_point));
  }
}

// Tests the case where the contact forces have a zero normal resultant force.
// Confirms that the application point is drawn from the set and any possible
// torque is returned.
GTEST_TEST(ContactResultantForceTest, TangentOnlyPlanarContactTest) {
// Do *not* change these values. The tests below will become invalid.
  Vector3<double> pos1, pos2, expected_torque;
  Vector3<double> norm, tan1, tan2, zero;
  pos1 << 1, 0, 0;
  pos2 << 2, 0, 0;
  norm << 0, 0, 0;
  tan1 << 1, 1, 0;
  tan2 << -1, 2, 0;
  zero = Vector3<double>::Zero();

  // Case 1: Two identical, tangent-only forces . Min. moment point will be
  // the first position (pos1) and, force is <0, 3, 0> and torque is <0, 0, 2>.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, norm, tan1, zero);
    calc.AddForce(pos2, norm, tan2, zero);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();

    expected_torque << 0, 0, 2;
    ASSERT_TRUE(AssertTorque(wrench, expected_torque));
    ASSERT_TRUE(AssertForce(wrench, tan1 + tan2));
    ASSERT_TRUE(AreEquivalent(min_point, pos1));
  }
}

// Tests the case where the application points are arrayed on a plane, but
// the individual contacts have individual normals.
GTEST_TEST(ContactResultantForceTest, SkewNormalPlanarPointTest) {
  Vector3<double> p1, p2, p3, n1, n2, n3, expected_point;
  const Vector3<double> zero = Vector3<double>::Zero();
  p1 << 1, 0, 0;
  p2 << 2, 0, 0;
  p3 << 1, 5, 0;
  n1 << 0, 0, 1;
  n2 << 0, 1, 1;
  n3 << 0.1, -1, 0.5;

  // Case 1: two forces, no tangential forces, but normals lying in different
  // directions.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(p1, n1, zero, zero);
    calc.AddForce(p2, n2, zero, zero);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();

    Vector3<double> expected_torque;
    expected_torque << 0, 0.2, 0.4;
    ASSERT_TRUE(AssertTorque(wrench, expected_torque));
    ASSERT_TRUE(AssertForce(wrench, n1 + n2));
    expected_point << 1.6, 0, 0;
    ASSERT_TRUE(AreEquivalent(min_point, expected_point));
  }

  // Case 2: three forces, no tangential forces, but normals lying in different
  // directions.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(p1, n1, zero, zero);
    calc.AddForce(p2, n2, zero, zero);
    calc.AddForce(p3, n3, zero, zero);
    Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
    WrenchVector<double> wrench = calc.ComputeResultantWrench();

    // these are magic numbers computed outside of this code based on the
    // constant values encoded above.
    Vector3<double> expected_torque;
    expected_torque << 0.023961661341853, -0.000000000000000, 0.599041533546326;
    ASSERT_TRUE(AssertTorque(wrench, expected_torque));
    ASSERT_TRUE(AssertForce(wrench, n1 + n2 + n3));
    expected_point << 1.399361022364217, 0.990415335463259, -0.015974440894569;
    ASSERT_TRUE(AreEquivalent(min_point, expected_point));
  }
}

// This tests a simple case where there are two, skew forces drawn from Paul
//  Mitiguy's book.
GTEST_TEST(ContactResultantForceTest, SkewNormalNonPlanarPointTest) {
  Vector3<double> p1, p2, n1, n2;
  const Vector3<double> zero = Vector3<double>::Zero();
  p1 << -3, 0, 0;
  p2 << 0, 1, -1;
  n1 << 3, 0, 0;
  n2 << 0, 0, 1;

  ContactResultantForceCalculator<double> calc;
  calc.AddForce(p1, n1, zero, zero);
  calc.AddForce(p2, n2, zero, zero);
  Vector3<double> min_point = calc.ComputeMinimumMomentPoint();
  WrenchVector<double> wrench = calc.ComputeResultantWrench();

  Vector3<double> expected_torque;
  expected_torque << 0.9, 0, 0.3;
  ASSERT_TRUE(AssertTorque(wrench, expected_torque));
  ASSERT_TRUE(AssertForce(wrench, n1 + n2));
  Vector3<double> expected_point;
  expected_point << -2.7, 0.1, -0.9;
  ASSERT_TRUE(AreEquivalent(min_point, expected_point));
}
}  // namespace
}  // namespace systems
}  // namespace drake
