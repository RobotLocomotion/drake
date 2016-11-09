#pragma once

#include "drake/multibody/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

/**
 An implementatino of the ContactDetail where the contact details consist of a
 single ContactForce.
 */
template <typename T>
class DRAKE_EXPORT PointContactDetail : public ContactDetail<T> {
 public:
  explicit PointContactDetail(const ContactForce<T>& force);
  std::unique_ptr<ContactDetail<T>> Clone() const override;
  ContactForce<T> ComputeContactForce() const override { return force_; }
 private:
  ContactForce<T> force_;
};

}  // namespace systems
}  // namespace drake
