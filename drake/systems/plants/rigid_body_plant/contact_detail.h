#pragma once

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
  The base class which defines a single point of contact and its corresponding
  Force (ultimately represented as a wrench) - both expressed in the world
  frame.

  The intention is for other contact models to create sub-classes that augment
  this class with additional per-contact data.

  @tparam T      The scalar type. It must be a valid Eigen scalar.
 */
template <typename T>
class DRAKE_EXPORT ContactDetail {
 public:
  /**
   Constructor for a fully specified contact detail: Force and its application
   point -- both expressed in the world frame.

   @param[in] point      The contact point at which the force is applied.
   @param[in] wrench     The contact Force (represented as a wrench).
   */
  ContactDetail(const Vector3<T>& point, const WrenchVector<T>& wrench);

  /** The point the Force is applied, expressed in the world frame */
  const Vector3<T>& get_application_point() const { return application_point_; }

  /** Returns the *spatial* Force (i.e., a wrench) and *not* a linear force. */
  const WrenchVector<T>& get_force() const { return wrench_; }

  virtual ContactDetail* clone() const;

 private:
  /** The point at which the Force is applied - expressed in the world frame. */
  Vector3<T> application_point_{};

  /** The contact Force expressed in the world frame. */
  WrenchVector<T> wrench_{};
};

extern template class DRAKE_EXPORT ContactDetail<double>;

}  // namespace systems
}  // namespace drake
