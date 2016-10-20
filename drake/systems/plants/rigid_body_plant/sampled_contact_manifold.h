#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/plants/rigid_body_plant/contact_detail.h"
#include "drake/systems/plants/rigid_body_plant/contact_manifold.h"

namespace drake {
namespace systems {

/**
 This manifold represents a contact surface with a limited number of sampled
 points.  It is, at best, a coarse approximation of the contact surface.  It
 doesn't guarantee that the points form a complete convex hull of the full
 contact area.  It merely provides one or more points which in some meaningful
 sense attempt to sample the contact area.
 */
template <typename T>
class DRAKE_EXPORT SampledContactManifold : public ContactManifold<T> {
 public:
  // This class is not movable
  SampledContactManifold(SampledContactManifold<T>&& other) = delete;
  SampledContactManifold<T>& operator=(SampledContactManifold<T>&& other) =
      delete;

  /** Default constructor */
  SampledContactManifold();

  /**
   Computes a single contact detail -- Force and application point -- which
   is equivalent to applying all individual contact forces individually.

   The "net" contact is defined as follows:
   p = sum_i [p_i * |f_i|] / sum_i |f_i|
   F = sum_i F_i + sum_i <0, 0, 0, (p - p_i) x f_i>

   where p_i is the ith application point.
   F_i is the ith spatial force (aka wrench).
   f_i, |f_i| are the force component (and its magnitude) of the ith spatial
   force, respectively.
   <0, 0, 0, f> is a zero-torque wrench built off the given force.

   The net application point (p) is an approximation of the center of  pressure.

   @returns The single net Force and application point - expressed in the world
            frame.
   */
  ContactDetail<T> ComputeNetResponse() const override;

  /** Reports the number of distinct contact details for this manifold. */
  int get_num_contacts() const override {
    return static_cast<int>(contact_details_.size()); }

  /**
   * Get the ith contact detail in the manifold.
   * @param i       The index of the desired contact detail.
   * @return A pointer to the ith contact detail.
   * @throws std::logic_error if the index is out of range.
   */
  const ContactDetail<T>* get_ith_contact(int i) const override;

  std::unique_ptr<ContactManifold<T>> Clone() const override;

  /**
   Add a new contact detail to the manifold.
   @param[in] detail    The contact detail to add.
   */
  void AddContactDetail(std::unique_ptr<ContactDetail<T>> detail);

 protected:
  SampledContactManifold(const SampledContactManifold& other);

 private:
  std::vector<std::unique_ptr<ContactDetail<T>>> contact_details_;
};

}  // namespace systems
}  // namespace drake
