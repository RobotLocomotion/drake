/* clang-format off */
#include "drake/multibody/rigid_body_plant/contact_force.h"
#include "drake/multibody/rigid_body_plant/contact_resultant_force_calculator.h"
/* clang-format on */

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/rigid_body_plant/point_contact_detail.h"
#include "drake/util/drakeGeometryUtil.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::unique_ptr;

// Wrapper on CompareMatricies to simplify calls in the tests.
template <typename DerivedA, typename DerivedB>
bool AreEquivalent(const Eigen::MatrixBase<DerivedA>& m1,
                   const Eigen::MatrixBase<DerivedB>& m2) {
  const double kTolerance = Eigen::NumTraits<double>::dummy_precision();
  return CompareMatrices(m1, m2, kTolerance, MatrixCompareType::absolute);
}

// Tests the ContactForce class.  The class is very simple. It's only
// functionality is:
//    1. Constructor logic,
//    2. Combination of force components into output values.
GTEST_TEST(ContactResultantForceTest, ContactForceTests) {
  Vector3<double> normal, force, torque, pos, normal_force, tangent_force;
  normal << 0, 0, 1;
  force << 0, 1, 1;
  normal_force << 0, 0, 1;
  tangent_force << 0, 1, 0;
  torque << 1, 0, 0;
  pos << 3, 2, 1;

  // Case 1. No pure torque constructor.
  ContactForce<double> f1(pos, normal, force);
  EXPECT_EQ(f1.get_normal_force(), normal_force);
  EXPECT_EQ(f1.get_tangent_force(), tangent_force);
  EXPECT_EQ(f1.get_force(), force);
  EXPECT_EQ(f1.get_torque(), Vector3<double>::Zero());
  EXPECT_EQ(f1.get_application_point(), pos);

  // Case 2. Fully-specified constructor.
  ContactForce<double> f2(pos, normal, force, torque);
  EXPECT_EQ(f2.get_normal_force(), normal_force);
  EXPECT_EQ(f2.get_tangent_force(), tangent_force);
  EXPECT_EQ(f1.get_force(), force);
  EXPECT_EQ(f2.get_torque(), torque);
  EXPECT_EQ(f2.get_application_point(), pos);
}

// Tests transforming wrench with respect to a given reference point.
// Should match results from transformSpatialForce.
GTEST_TEST(ContactResultantForceTest,
           ForceAccumulationWithFixedReferencePointTest) {
  Vector3<double> normal, force, torque, pos, ref_point;
  normal << 0, 0, 1;
  force << 1, 0, 1;
  torque << 0, 2, 0;
  pos << 0, 0, 1;
  ref_point << 0, 0, 3;
  ContactForce<double> cforce(pos, normal, force, torque);

  SpatialForce<double> eq_wrench;
  Isometry3<double> transform(Isometry3<double>::Identity());
  transform.translation() = pos - ref_point;
  eq_wrench = transformSpatialForce(transform, cforce.get_spatial_force());

  ContactResultantForceCalculator<double> calc;
  calc.AddForce(cforce);
  ContactForce<double> wrench = calc.ComputeResultant(ref_point);

  EXPECT_EQ(wrench.get_application_point(), ref_point);
  EXPECT_EQ(wrench.get_spatial_force(), eq_wrench);
}

// Tests the various forms for adding forces to the accumulator.  This also
// implicitly the case where the resultant force is computed for a single
// contact force; the contact force *is* the resultant force and its application
// point is the minimum moment point.
GTEST_TEST(ContactResultantForceTest, ForceAccumulationTest) {
  Vector3<double> normal, force, torque, pos, normal_force;
  force << 4, 6, 8;
  normal_force << 1, 2, 3;
  normal = normal_force.normalized();
  torque << 6, 7, 8;
  pos << 10, 11, 12;
  SpatialForce<double> full_wrench, torque_free_wrench;
  full_wrench.template head<3>() = torque;
  full_wrench.template tail<3>() = force;
  torque_free_wrench.template head<3>() << 0, 0, 0;
  torque_free_wrench.template tail<3>() = force;

  // Case 1: The ContactForce interface -- pass an instance of ContactForce.
  {
    ContactForce<double> cforce(pos, normal, force, torque);
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(cforce);
    ContactForce<double> resultant = calc.ComputeResultant();
    EXPECT_EQ(resultant.get_application_point(), pos);
    EXPECT_EQ(resultant.get_spatial_force(), full_wrench);
  }

  // Case 2: The interface for components without pure torque.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos, normal, force);
    ContactForce<double> resultant = calc.ComputeResultant();
    EXPECT_EQ(resultant.get_application_point(), pos);
    EXPECT_EQ(resultant.get_spatial_force(), torque_free_wrench);
  }

  // Case 3: The interface for components with all data.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos, normal, force, torque);
    ContactForce<double> resultant = calc.ComputeResultant();
    EXPECT_EQ(resultant.get_application_point(), pos);
    EXPECT_EQ(resultant.get_spatial_force(), full_wrench);
  }

  // Case 4: The ContactDetail interface.
  {
    ContactForce<double> cforce(pos, normal, force, torque);
    unique_ptr<ContactDetail<double>> detail(
        new PointContactDetail<double>(cforce));
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(move(detail));
    ContactForce<double> resultant = calc.ComputeResultant();
    EXPECT_EQ(resultant.get_application_point(), pos);
    EXPECT_EQ(resultant.get_spatial_force(), full_wrench);
  }
}

// Tests that the accumulation of contact details works as expected.
// Particularly important for the interface where the ContactDetail type is
// implied by the parameters passed.
GTEST_TEST(ContactResultantForceTest, DetailAccumulationTest) {
  Vector3<double> normal, force, torque, pos, normal_force;
  force << 4, 6, 8;
  normal_force << 1, 2, 3;
  normal = normal_force.normalized();
  torque << 6, 7, 8;
  pos << 10, 11, 12;

  // Case 1: The ContactForce interface -- pass an instance of ContactForce.
  {
    std::vector<unique_ptr<ContactDetail<double>>> details;
    ContactResultantForceCalculator<double> calc(&details);
    ContactForce<double> cforce(pos, normal, force, torque);
    calc.AddForce(cforce);
    EXPECT_EQ(details.size(), 1u);
    PointContactDetail<double>* detail =
        dynamic_cast<PointContactDetail<double>*>(details[0].get());
    ASSERT_NE(detail, nullptr);
    auto contact_force = detail->ComputeContactForce();
    EXPECT_EQ(contact_force.get_application_point(), pos);
    EXPECT_EQ(contact_force.get_normal(), normal);
    EXPECT_EQ(contact_force.get_force(), force);
    EXPECT_EQ(contact_force.get_torque(), torque);
  }

  // Case 2: The interface for components without pure torque.
  {
    std::vector<unique_ptr<ContactDetail<double>>> details;
    ContactResultantForceCalculator<double> calc(&details);
    calc.AddForce(pos, normal, force);
    EXPECT_EQ(details.size(), 1u);
    PointContactDetail<double>* detail =
        dynamic_cast<PointContactDetail<double>*>(details[0].get());
    ASSERT_NE(detail, nullptr);
    auto contact_force = detail->ComputeContactForce();
    EXPECT_EQ(contact_force.get_application_point(), pos);
    EXPECT_EQ(contact_force.get_normal(), normal);
    EXPECT_EQ(contact_force.get_force(), force);
    EXPECT_EQ(contact_force.get_torque(), Vector3<double>::Zero());
  }

  // Case 3: The interface for components with all data.
  {
    std::vector<unique_ptr<ContactDetail<double>>> details;
    ContactResultantForceCalculator<double> calc(&details);
    calc.AddForce(pos, normal, force, torque);
    EXPECT_EQ(details.size(), 1u);
    PointContactDetail<double>* detail =
        dynamic_cast<PointContactDetail<double>*>(details[0].get());
    ASSERT_NE(detail, nullptr);
    auto contact_force = detail->ComputeContactForce();
    EXPECT_EQ(contact_force.get_application_point(), pos);
    EXPECT_EQ(contact_force.get_normal(), normal);
    EXPECT_EQ(contact_force.get_force(), force);
    EXPECT_EQ(contact_force.get_torque(), torque);
  }

  // Case 4: The ContactDetail interface.
  {
    std::vector<unique_ptr<ContactDetail<double>>> details;
    ContactResultantForceCalculator<double> calc(&details);
    ContactForce<double> cforce(pos, normal, force, torque);
    unique_ptr<ContactDetail<double>> input_detail(
        new PointContactDetail<double>(cforce));
    calc.AddForce(move(input_detail));
    EXPECT_EQ(details.size(), 1u);
    PointContactDetail<double>* detail =
        dynamic_cast<PointContactDetail<double>*>(details[0].get());
    ASSERT_NE(detail, nullptr);
    auto contact_force = detail->ComputeContactForce();
    EXPECT_EQ(contact_force.get_application_point(), pos);
    EXPECT_EQ(contact_force.get_normal(), normal);
    EXPECT_EQ(contact_force.get_force(), force);
    EXPECT_EQ(contact_force.get_torque(), torque);
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
  Vector3<double> normal, force, zero;
  pos1 << 1, 0, 0;
  pos2 << 2, 0, 0;
  pos3 << 1, 5, 0;
  normal << 0, 0, 1;
  force << 0, 0, 1;
  zero = Vector3<double>::Zero();
  Vector3<double> expected_point;

  // Case 1: Two identical forces -- min. moment point should lie in between
  // them.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, normal, force, zero);
    calc.AddForce(pos2, normal, force, zero);
    ContactForce<double> resultant = calc.ComputeResultant();

    EXPECT_TRUE(AreEquivalent(resultant.get_torque(), zero));
    EXPECT_TRUE(AreEquivalent(resultant.get_force(), force * 2));
    expected_point = (pos1 + pos2) * 0.5;
    EXPECT_TRUE(
        AreEquivalent(resultant.get_application_point(), expected_point));
  }

  // Case 2: Two forces of unequal magnitudes.  Min. moment point should lie
  // between the two points based on ratio of force magnitudes.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, normal, force, zero);
    calc.AddForce(pos2, normal, 2.0 * force, zero);
    ContactForce<double> resultant = calc.ComputeResultant();

    EXPECT_TRUE(AreEquivalent(resultant.get_torque(), zero));
    EXPECT_TRUE(AreEquivalent(resultant.get_force(), force * 3));
    expected_point = pos1 / 3. + pos2 * 2. / 3.;
    EXPECT_TRUE(
        AreEquivalent(resultant.get_application_point(), expected_point));
  }

  // Case 3: Three forces of equal magnitude.  This example came from Paul
  // Mitiguy.
  // Given equal forces applied in parallel directions at three points: O, P, &
  // Q, the center of pressure should be: O + 1/3 * (rP + rQ)
  //  where, rP = P - O, and rQ = Q - O
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, normal, force, zero);
    calc.AddForce(pos2, normal, force, zero);
    calc.AddForce(pos3, normal, force, zero);
    ContactForce<double> resultant = calc.ComputeResultant();

    EXPECT_TRUE(AreEquivalent(resultant.get_torque(), zero));
    EXPECT_TRUE(AreEquivalent(resultant.get_force(), force * 3));
    expected_point = pos1 + ((pos2 - pos1) + (pos3 - pos1)) / 3.0;
    EXPECT_TRUE(
        AreEquivalent(resultant.get_application_point(), expected_point));
  }

  // Case 4: Three forces of unequal magnitude.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, normal, force, zero);
    calc.AddForce(pos2, normal, 2.0 * force, zero);
    calc.AddForce(pos3, normal, 3.0 * force, zero);
    ContactForce<double> resultant = calc.ComputeResultant();

    EXPECT_TRUE(AreEquivalent(resultant.get_torque(), zero));
    EXPECT_TRUE(AreEquivalent(resultant.get_force(), force * 6));
    expected_point << 4.0 / 3, 5.0 / 2, 0;
    EXPECT_TRUE(
        AreEquivalent(resultant.get_application_point(), expected_point));
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
    calc.AddForce(pos1, normal, force + tan1, zero);
    calc.AddForce(pos2, normal, 2.0 * force + tan2, zero);
    calc.AddForce(pos3, normal, 3.0 * force + tan3, zero);
    ContactForce<double> resultant = calc.ComputeResultant();

    Vector3<double> expected_torque;
    expected_torque << 0, 0, -0.4;
    EXPECT_TRUE(AreEquivalent(resultant.get_torque(), expected_torque));
    EXPECT_TRUE(
        AreEquivalent(resultant.get_force(), force * 6 + tan1 + tan2 + tan3));
    expected_point << 4.0 / 3, 5.0 / 2, 0;
    EXPECT_TRUE(
        AreEquivalent(resultant.get_application_point(), expected_point));
  }
}

// Tests the case where the contact forces have a zero normal resultant force.
// Confirms that the application point is drawn from the set and any possible
// torque is returned.
GTEST_TEST(ContactResultantForceTest, TangentOnlyPlanarContactTest) {
  // Do *not* change these values. The tests below will become invalid.
  Vector3<double> pos1, pos2, expected_torque, expected_point;
  Vector3<double> normal, tangent1, tangent2, zero;
  pos1 << 1, 0, 0;
  pos2 << 2, 0, 0;
  normal << 0, 0, 1;
  tangent1 << 1, 1, 0;
  tangent2 << -1, 2, 0;
  zero = Vector3<double>::Zero();
  expected_torque << 0, 0, 0.5;
  expected_point = (pos1 + pos2) * 0.5;

  // Case 1: Two identical, tangent-only forces. Min. moment point will be
  // the centroid and, force is <0, 3, 0> and torque is <0, 0, 0.5>.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos1, normal, tangent1, zero);
    calc.AddForce(pos2, normal, tangent2, zero);
    ContactForce<double> resultant = calc.ComputeResultant();

    EXPECT_TRUE(AreEquivalent(resultant.get_torque(), expected_torque));
    EXPECT_TRUE(AreEquivalent(resultant.get_force(), tangent1 + tangent2));
    EXPECT_TRUE(
        AreEquivalent(resultant.get_application_point(), expected_point));
  }

  // Case 2: Same as case 1, except reverse the order of adding forces to show
  // that it is order invariant.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(pos2, normal, tangent2, zero);
    calc.AddForce(pos1, normal, tangent1, zero);
    ContactForce<double> resultant = calc.ComputeResultant();

    EXPECT_TRUE(AreEquivalent(resultant.get_torque(), expected_torque));
    EXPECT_TRUE(AreEquivalent(resultant.get_force(), tangent1 + tangent2));
    EXPECT_TRUE(
        AreEquivalent(resultant.get_application_point(), expected_point));
  }
}

// Tests the case where the application points are arrayed on a plane, but
// the individual contacts have individual normals.
GTEST_TEST(ContactResultantForceTest, SkewNormalPlanarPointTest) {
  Vector3<double> p1, p2, p3, f1, f2, f3, n1, n2, n3, expected_point;
  const Vector3<double> zero = Vector3<double>::Zero();
  p1 << 1, 0, 0;
  p2 << 2, 0, 0;
  p3 << 1, 5, 0;
  f1 << 0, 0, 1;
  n1 = f1.normalized();
  f2 << 0, 1, 1;
  n2 = f2.normalized();
  f3 << 0.1, -1, 0.5;
  n3 = f3.normalized();

  // Case 1: two forces, no tangential forces, but normals lying in different
  // directions.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(p1, n1, f1, zero);
    calc.AddForce(p2, n2, f2, zero);
    ContactForce<double> resultant = calc.ComputeResultant();

    Vector3<double> expected_torque;
    expected_torque << 0, 0.2, 0.4;
    EXPECT_TRUE(AreEquivalent(resultant.get_torque(), expected_torque));
    EXPECT_TRUE(AreEquivalent(resultant.get_force(), f1 + f2));
    expected_point << 1.6, 0, 0;
    EXPECT_TRUE(
        AreEquivalent(resultant.get_application_point(), expected_point));
  }

  // Case 2: three forces, no tangential forces, but normals lying in different
  // directions.
  {
    ContactResultantForceCalculator<double> calc;
    calc.AddForce(p1, n1, f1, zero);
    calc.AddForce(p2, n2, f2, zero);
    calc.AddForce(p3, n3, f3, zero);
    ContactForce<double> resultant = calc.ComputeResultant();

    // these are magic numbers computed outside of this code based on the
    // constant values encoded above.
    Vector3<double> expected_torque;
    expected_torque << 0.023961661341853, -0.000000000000000, 0.599041533546326;
    EXPECT_TRUE(AreEquivalent(resultant.get_torque(), expected_torque));
    EXPECT_TRUE(AreEquivalent(resultant.get_force(), f1 + f2 + f3));
    expected_point << 1.399361022364217, 0.990415335463259, -0.015974440894569;
    EXPECT_TRUE(
        AreEquivalent(resultant.get_application_point(), expected_point));
  }
}

// This tests a simple case where there are two, skew forces drawn from Paul
//  Mitiguy's book.
GTEST_TEST(ContactResultantForceTest, SkewNormalNonPlanarPointTest) {
  Vector3<double> p1, p2, f1, n1, f2, n2;
  const Vector3<double> zero = Vector3<double>::Zero();
  p1 << -3, 0, 0;
  p2 << 0, 1, -1;
  f1 << 3, 0, 0;
  n1 = f1.normalized();
  f2 << 0, 0, 1;
  n2 = f2.normalized();

  ContactResultantForceCalculator<double> calc;
  calc.AddForce(p1, n1, f1, zero);
  calc.AddForce(p2, n2, f2, zero);
  ContactForce<double> resultant = calc.ComputeResultant();

  Vector3<double> expected_torque;
  expected_torque << 0.9, 0, 0.3;
  EXPECT_TRUE(AreEquivalent(resultant.get_torque(), expected_torque));
  EXPECT_TRUE(AreEquivalent(resultant.get_force(), f1 + f2));
  Vector3<double> expected_point;
  expected_point << -2.7, 0.1, -0.9;
  EXPECT_TRUE(AreEquivalent(resultant.get_application_point(), expected_point));
}
}  // namespace
}  // namespace systems
}  // namespace drake
