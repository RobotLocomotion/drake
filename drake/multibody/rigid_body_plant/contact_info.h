#pragma once

#include <memory>
#include <vector>

#include "drake/multibody/collision/element.h"
#include "drake/multibody/rigid_body_plant/contact_detail.h"
#include "drake/multibody/rigid_body_plant/contact_force.h"

namespace drake {
namespace systems {

/**
 A class containing information regarding contact response between two bodies
 including:
    - The pair of collision elements that are contacting (e1, e2), referenced
        by their unique identifiers.
    - A resultant ContactForce -- a single ContactForce with the equivalent
      effect of applying all individual ContactDetails to element e1.
    - An optional list of ContactDetail instances.

 Some forms of ContactDetail are more expensive than others. However,
 ContactInfo instances will need to be copied. The contact model defines a
 default behavior of whether the ContactDetails are stored in the corresponding
 ContactInfo instance or not. If this happens, the ContactInfo instance will
 contain a valid resultant force, but no contact details.

 Eventually, this beahvior will be subject to user configuration; the
 user will specify whether they want the details to be included in the
 ContactInfo, overriding the contact model's default behavior, and paying the
 corresponding copying cost.

 The resultant force and contact details, if they are included, are all defined
 such that they act on the first element in the pair (e1). Newton's third law
 requires that an equal and opposite force be applied, at exactly the same point
 in space, to e2.

 @tparam T      The scalar type. It must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:
    - double
 */
template <typename T>
class ContactInfo {
 public:
  /**
   Initialize the contact response for two colliding collision elements.

   @param element1      The identifier for the first collision element.
   @param element2      The identifier for the second collision element.
   */
  ContactInfo(DrakeCollision::ElementId element1,
              DrakeCollision::ElementId element2);

  ContactInfo(const ContactInfo<T>& other);
  ContactInfo& operator=(const ContactInfo<T>& other);
  ContactInfo(ContactInfo<T>&& other) = delete;
  ContactInfo& operator=(ContactInfo<T>&& other) = delete;

  DrakeCollision::ElementId get_element_id_1() const { return element1_; }
  DrakeCollision::ElementId get_element_id_2() const { return element2_; }

  void set_resultant_force(const ContactForce<T> force) {
    resultant_force_ = force;
  }

  const ContactForce<T>& get_resultant_force() const {
    return resultant_force_;
  }

  const std::vector<std::unique_ptr<ContactDetail<T>>>& get_contact_details()
      const {
    return contact_details_;
  }

  void set_contact_details(
      std::vector<std::unique_ptr<ContactDetail<T>>>&& details) {
    contact_details_ = std::move(details);
  }

 private:
  DrakeCollision::ElementId element1_{};
  DrakeCollision::ElementId element2_{};
  ContactForce<T> resultant_force_;
  std::vector<std::unique_ptr<ContactDetail<T>>> contact_details_;
};

}  // namespace systems
}  // namespace drake
