#include "drake/geometry/proximity/collisions_exist_callback.h"

#include "drake/geometry/proximity/collision_backend.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace geometry {
namespace internal {
namespace has_collisions {

template <typename Backend>
CallbackData<Backend>::CallbackData(const CollisionFilter* collision_filter_in)
    : collision_filter(*collision_filter_in) {
  DRAKE_DEMAND(collision_filter_in != nullptr);
  Backend::ConfigureHasCollisionRequest(&request);
}

template <typename Backend>
bool Callback(typename Backend::Object* object_A_ptr,
              typename Backend::Object* object_B_ptr, void* callback_data) {
  auto& data = *static_cast<CallbackData<Backend>*>(callback_data);

  const EncodedData encoding_a(*object_A_ptr);
  const EncodedData encoding_b(*object_B_ptr);

  const bool can_collide =
      data.collision_filter.CanCollideWith(encoding_a.id(), encoding_b.id());
  if (!can_collide) return false;

  // Unpack the callback data.
  const typename Backend::CollisionRequest& request = data.request;

  // This callback only works for a single contact, this confirms a request
  // hasn't been made for more contacts.
  DRAKE_ASSERT(request.num_max_contacts == 1);
  typename Backend::CollisionResult result;

  // Perform nearphase collision detection.
  Backend::Collide(object_A_ptr, object_B_ptr, request, &result);

  data.collisions_exist = result.isCollision();
  return data.collisions_exist;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_BACKENDS((&Callback<Backend>));

}  // namespace has_collisions
}  // namespace internal
}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_BACKENDS(
    struct ::drake::geometry::internal::has_collisions::CallbackData);
