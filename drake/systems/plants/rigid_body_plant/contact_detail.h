#pragma once

#include <memory>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/**
  The base class for defining a contact response.  It consists of a wrench
  and a point.  The point is measured and expressed in the world frame and
  serves as the origin of a world-aligned frame in which the wrench is
  expressed.

  These two pieces of data are considered to be the minimum amount of data,
  regardless of the contact model used. However, the intention is for the
  creation of new sub-classes which augment the set of per-contact point
  data based on other contact models.

  @tparam T      The scalar type. It must be a valid Eigen scalar.
 */
template <typename T>
class DRAKE_EXPORT ContactDetail {
 public:
  /**
   Constructor for a fully specified contact detail: a wrench and its
   application point -- both expressed in the world frame.

   @param[in] point      The contact point at which the force is applied.
   @param[in] wrench     The contact wrench.
   */
  ContactDetail(const Vector3<T>& point, const WrenchVector<T>& wrench);

  /** The point the Force is applied, expressed in the world frame */
  const Vector3<T>& get_application_point() const { return application_point_; }

  /** Returns the *spatial* wrench. */
  const WrenchVector<T>& get_wrench() const { return wrench_; }

  virtual std::unique_ptr<ContactDetail> Clone() const;

 private:
  /** The point at which the wrench is applied, expressed in the world frame. */
  Vector3<T> application_point_{};

  /** The contact wrench expressed in the world frame. */
  WrenchVector<T> wrench_{};
};

}  // namespace systems
}  // namespace drake
