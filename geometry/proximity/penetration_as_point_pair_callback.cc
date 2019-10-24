#include "drake/geometry/proximity/penetration_as_point_pair_callback.h"

#include <limits>
#include <utility>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {
namespace penetration_as_point_pair {

using Eigen::Vector3d;
using fcl::CollisionObjectd;
using fcl::CollisionRequestd;
using fcl::CollisionResultd;
using fcl::collide;
using fcl::Contactd;

bool Callback(CollisionObjectd* fcl_object_A_ptr,
              CollisionObjectd* fcl_object_B_ptr,
              void* callback_data) {
  // NOTE: Although this function *takes* non-const pointers to satisfy the
  // fcl api, it should not exploit the non-constness to modify the collision
  // objects. We ensure this by immediately assigning to a const version and
  // not directly using the provided parameters.
  const CollisionObjectd& fcl_object_A = *fcl_object_A_ptr;
  const CollisionObjectd& fcl_object_B = *fcl_object_B_ptr;

  auto& data = *static_cast<CallbackData*>(callback_data);

  // Extract the collision filter keys from the fcl collision objects. These
  // keys will also be used to map the fcl collision object back to the Drake
  // GeometryId for colliding geometries.
  EncodedData encoding_A(fcl_object_A);
  EncodedData encoding_B(fcl_object_B);

  const bool can_collide = data.collision_filter.CanCollideWith(
      encoding_A.encoding(), encoding_B.encoding());

  // NOTE: Here and below, false is returned regardless of whether collision
  // is detected or not because true tells the broadphase manager to terminate.
  // Since we want *all* collisions, we return false.
  if (!can_collide) return false;

  // Unpack the callback data
  const CollisionRequestd& request = data.request;

  // This callback only works for a single contact, this confirms a request
  // hasn't been made for more contacts.
  DRAKE_ASSERT(request.num_max_contacts == 1);
  CollisionResultd result;

  // Perform nearphase collision detection
  collide(&fcl_object_A, &fcl_object_B, request, result);

  if (!result.isCollision()) return false;

  // Process the contact points
  // NOTE: This assumes that the request is configured to use a single
  // contact.
  const Contactd& contact = result.getContact(0);

  // Signed distance is negative when penetration depth is positive.
  const double depth = contact.penetration_depth;

  // TODO(SeanCurtis-TRI): Remove this test when FCL issue 375 is fixed.
  // FCL returns osculation as contact but doesn't guarantee a non-zero
  // normal. Drake isn't really in a position to define that normal from the
  // geometry or contact results so, if the geometry is sufficiently close
  // to osculation, we consider the geometries to be non-penetrating.
  if (depth <= std::numeric_limits<double>::epsilon()) return false;

  // By convention, Drake requires the contact normal to point out of B
  // and into A. FCL uses the opposite convention.
  Vector3d drake_normal = -contact.normal;

  // FCL returns a single contact point centered between the two
  // penetrating surfaces. PenetrationAsPointPair expects
  // two, one on the surface of body A (Ac) and one on the surface of body
  // B (Bc). Choose points along the line defined by the contact point and
  // normal, equidistant to the contact point. Recall that signed_distance
  // is strictly non-positive, so signed_distance * drake_normal points
  // out of A and into B.
  Vector3d p_WAc = contact.pos - 0.5 * depth * drake_normal;
  Vector3d p_WBc = contact.pos + 0.5 * depth * drake_normal;

  PenetrationAsPointPair<double> penetration;
  penetration.depth = depth;
  penetration.id_A = encoding_A.id();
  penetration.id_B = encoding_B.id();
  penetration.p_WCa = p_WAc;
  penetration.p_WCb = p_WBc;
  penetration.nhat_BA_W = drake_normal;
  // Guarantee fixed ordering of pair (A, B). Swap the ids and points on
  // surfaces and then flip the normal.
  if (penetration.id_B < penetration.id_A) {
    std::swap(penetration.id_A, penetration.id_B);
    std::swap(penetration.p_WCa, penetration.p_WCb);
    penetration.nhat_BA_W = -penetration.nhat_BA_W;
  }
  data.point_pairs.push_back(std::move(penetration));

  return false;
}

}  // namespace penetration_as_point_pair
}  // namespace internal
}  // namespace geometry
}  // namespace drake
