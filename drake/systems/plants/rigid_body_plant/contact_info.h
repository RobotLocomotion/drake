#pragma once

#include <memory>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/collision/Element.h"
#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/rigid_body_plant/contact_manifold.h"

namespace drake {
namespace systems {

/**
 A class containing information regarding contact between two bodies including:
    - The pair of collision elements that are contacting (e1, e2) (referenced
        by their unique identifiers.)
    - The collision manifold representing the contact surface and the contact
        forces (@see ContactManifold).

 The forces in the collision manifold are all defined such that they act on
 the first element in the pair (e1).

 @tparam T      The scalar type. It must be a valid Eigen scalar.
 */
template <typename T>
class DRAKE_EXPORT ContactInfo {
 public:
  /**
   Initialize the contact information between two collision elements.

   @param element1      The identifier for the first collision element.
   @param element2      The identifier for the second collision element.
   @param manifold      The manifold defining the contact result.
   */
  ContactInfo(DrakeCollision::ElementId element1,
              DrakeCollision::ElementId element2,
              std::unique_ptr<ContactManifold<T>> manifold);

  ContactInfo(const ContactInfo<T>& other);
  ContactInfo& operator=(const ContactInfo<T>& other);

  ContactInfo* clone() const;

  DrakeCollision::ElementId get_element_id_1() const;
  DrakeCollision::ElementId get_element_id_2() const;
  const ContactManifold<T>& get_contact_manifold() const;

 private:
  DrakeCollision::ElementId element1_;
  DrakeCollision::ElementId element2_;
  std::unique_ptr<ContactManifold<T>> contact_manifold_;
};

extern template class DRAKE_EXPORT ContactInfo<double>;

}  // namespace systems
}  // namespace drake
