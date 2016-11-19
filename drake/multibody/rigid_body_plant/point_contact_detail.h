#pragma once

#include "drake/multibody/rigid_body_plant/contact_detail.h"

namespace drake {
namespace systems {

/**
 An implementation of the ContactDetail where the contact details consist of a
 single ContactForce.

 @tparam T      The scalar type. It must be a valid Eigen scalar.

 Instantiated templates for the following ScalarTypes are provided:
   - double
 */
template <typename T>
class PointContactDetail : public ContactDetail<T> {
 public:
  explicit PointContactDetail(const ContactForce<T>& force);
  std::unique_ptr<ContactDetail<T>> Clone() const override;
  ContactForce<T> ComputeContactForce() const override { return force_; }

  // not copyable or movable
  PointContactDetail(const PointContactDetail& other) = delete;
  PointContactDetail& operator=(const PointContactDetail& other) = delete;
  PointContactDetail(PointContactDetail&& other) = delete;
  PointContactDetail& operator=(PointContactDetail&& other) = delete;
 private:
  ContactForce<T> force_;
};

}  // namespace systems
}  // namespace drake
