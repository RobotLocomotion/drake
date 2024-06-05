#pragma once

#include <memory>
#include <vector>

#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/deformable_contact.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"

namespace drake {
namespace geometry {

/** The type used for the SceneGraph contact_summary output port.
 @experimental We ancitipate stabilizing it around 2025-01-01.
 @tparam_default_scalar */
template <typename T>
struct ContactSummary {
  std::shared_ptr<const std::vector<ContactSurface<T>>> surfaces;
  std::shared_ptr<const std::vector<PenetrationAsPointPair<T>>> point_pairs;
  std::shared_ptr<const internal::DeformableContact<T>> deformable_contact;
};

}  // namespace geometry
}  // namespace drake
