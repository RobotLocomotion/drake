#include "drake/multibody/rigid_body_plant/contact_info.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/multibody/rigid_body_plant/contact_force.h"
#include "drake/multibody/rigid_body_plant/point_contact_detail.h"

// Tests the ContactInfo class .
namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::unique_ptr;
using std::move;


// Helper function for generating a contact detail. The x-value of the position
// can be set to help distinguish forces.
unique_ptr<PointContactDetail<double>> MakeDetail(double x = 1.0) {
  Vector3<double> point, normal, force, torque;
  point << x, 2, 3;
  normal << 1, 0, 0;
  force << 2, 0, 1;
  torque << -1, -2, -3;
  ContactForce<double> contact_force(point, normal, force, torque);
  return make_unique<PointContactDetail<double>>(contact_force);
}

// Utility method for confirm that one ContactInfo instance is a copy of
// another.
template <typename T>
void AssertValidCopy(const ContactInfo<T>& test_value,
                     const ContactInfo<T>& expected_value) {
  // They must have the same element ids.
  ASSERT_EQ(test_value.get_element_id_1(), expected_value.get_element_id_1());
  ASSERT_EQ(test_value.get_element_id_2(), expected_value.get_element_id_2());

  // They must have the same *number* of details.
  const auto& test_details = test_value.get_contact_details();
  const auto& expected_details = expected_value.get_contact_details();
  ASSERT_EQ(test_details.size(), expected_details.size());

  // The details must point to different object which have the same value.
  for (size_t i = 0; i < test_details.size(); ++i) {
    const auto& test_detail = test_details[i];
    const auto& expected_detail = expected_details[i];
    ASSERT_NE(test_detail.get(), expected_detail.get());

    auto test_force = test_detail->ComputeContactForce();
    auto expected_force = expected_detail->ComputeContactForce();
    ASSERT_EQ(test_force.get_application_point(),
              expected_force.get_application_point());
    ASSERT_EQ(test_force.get_force(), expected_force.get_force());
    ASSERT_EQ(test_force.get_torque(), expected_force.get_torque());
    ASSERT_EQ(test_force.get_normal(), expected_force.get_normal());
  }
}

// Confirms that the data of a single contact info is appropriately copied both
// through the copy constructor as well as the assignment operator.
GTEST_TEST(ContactInfoTests, CloneDetails) {
  // First, construct a ContactInfo reference.
  drake::multibody::collision::ElementId element_a = 10;
  drake::multibody::collision::ElementId element_b = 20;
  ContactInfo<double> contact_info(element_a, element_b);
  std::vector<unique_ptr<ContactDetail<double>>> details;

  const int kDetailCount = 3;
  for (int i = 0; i < kDetailCount; ++i) {
    details.emplace_back(MakeDetail(i-1));
  }
  contact_info.set_contact_details(move(details));

  // Second, test the copy constructor.
  ContactInfo<double> info_copy(contact_info);
  AssertValidCopy<double>(info_copy, contact_info);

  // Third, test the assignment operator.
  ContactInfo<double> info_assign(element_b, element_a);
  info_assign = contact_info;
  AssertValidCopy<double>(info_assign, contact_info);

  // Fourth, proof-of-life for AutoDiffXd.
  ContactInfo<AutoDiffXd> info_autodiff(element_a, element_b);
  ASSERT_EQ(info_autodiff.get_element_id_1(), element_a);
  ASSERT_EQ(info_autodiff.get_element_id_2(), element_b);
}

template <template <class> class Pointer>
void TestSetDetails() {
  // Set up initial conditions.
  drake::multibody::collision::ElementId element_a = 10;
  drake::multibody::collision::ElementId element_b = 20;
  ContactInfo<double> contact_info(element_a, element_b);

  std::vector<Pointer<ContactDetail<double>>> details;
  const int kDetailCount = 3;
  for (int i = 0; i < kDetailCount; ++i) {
    details.emplace_back(MakeDetail(i-1));
  }
  ASSERT_EQ(contact_info.get_contact_details().size(), 0);
  ASSERT_EQ(details.size(), kDetailCount);

  // Test the move from the details to an *empty* ContactInfo.
  contact_info.set_contact_details(move(details));
  ASSERT_EQ(contact_info.get_contact_details().size(), kDetailCount);
  ASSERT_EQ(details.size(), 0);

  // Used to distinguish contact details by the force application point's
  // x-value.
  const double kOffset = 10.0;
  // Test the move from details to a non-empty ContactInfo.
  for (int i = 0; i < kDetailCount; ++i) {
    details.emplace_back(MakeDetail(i-kOffset));
  }
  ASSERT_EQ(details.size(), kDetailCount);
  ASSERT_EQ(contact_info.get_contact_details().size(), kDetailCount);
  contact_info.set_contact_details(move(details));
  ASSERT_EQ(contact_info.get_contact_details().size(), kDetailCount);
  ASSERT_EQ(details.size(), 0);
  for (int i = 0; i < kDetailCount; ++i) {
    auto force = contact_info.get_contact_details()[i]->ComputeContactForce();
    EXPECT_EQ(force.get_application_point()(0), i-kOffset);
  }
}

// In order to get ADL on TestSetDetails, the unique_ptr<T, Deleter> needs to be
// aliased into my_unique_ptr<T>. As per C++ spec, the templated alias must be
// created at class or namespace scope.
template <class T> using my_unique_ptr = unique_ptr<T, std::default_delete<T>>;

// Tests the functionality where the details are set via a vector of unique_ptr.
GTEST_TEST(ContactInfoTests, SetDetailsUniquePtr) {
  TestSetDetails<my_unique_ptr>();
}

// Tests the functionality where the details are set via a vector of
// copyable_unique_ptr.
GTEST_TEST(ContactInfoTests, SetDetailsCopyableUniquePtr) {
  TestSetDetails<copyable_unique_ptr>();
}

}  // namespace
}  // namespace systems
}  // namespace drake
