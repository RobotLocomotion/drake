#pragma once

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
  The base class which defines a single point of contact and its corresponding
  Force (ultimately represented as a wrench).

  The point of contact and Force are expressed in the world frame.
 */
template <typename T>
class DRAKE_EXPORT ContactDetail {
 public:
  /** Default constructor */
  ContactDetail();

  /**
   Constructor for a fully specified contact detail: Force and its application
   point.

   @param[in] point      The contact point at which the force is applied.
   @param[in] wrench     The contact Force (represented as a wrench).
   */
  explicit ContactDetail(const Vector3<T>& point,
                         const WrenchVector<T>& wrench);

  const Vector3<T>& get_application_point() const { return application_point_; }

  const WrenchVector<T>& get_force() const { return wrench_; };

 private:
  /** The point at which the Force is applied -- expressed in the world frame.
   */
  Vector3<T> application_point_{};

  /** The contact Force expressed in the world frame. */
  WrenchVector<T> wrench_{};
};
}
}
